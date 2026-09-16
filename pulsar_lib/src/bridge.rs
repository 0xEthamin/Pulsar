//! Bridge from the A2DP sink of the control board to its I2S slot stream.
//!
//! Bluedroid hands the sink 16-bit signed PCM, little endian, one sample per
//! frame on a mono stream and two channels interleaved, left first, on any
//! other. The processing board clocks the I2S link as master, Philips format,
//! two 32-bit slots per frame, and plays the mean of the two slots. Everything
//! between the two that touches a sample lives here: the frame ring that
//! absorbs the drift between the clock of the phone and the clock of the link,
//! the stream gate, the framing of a mono sample into two channels, the
//! widening of each sample into its slot, and the pump that moves one block
//! from the ring to the link through `SlotWriter`.
//!
//! # Frames, never bytes
//!
//! The ring stores whole frames of two channels, so no push, drop or drain can
//! split a frame or swap its two channels. A chunk whose length is not a whole
//! number of frames of its stream, two bytes on a mono stream and four on any
//! other, loses its trailing partial frame, and the next chunk starts on a
//! frame boundary again. Bluedroid decodes whole SBC frames, so a trailing
//! partial frame only comes from a malformed chunk.
//!
//! # Drift
//!
//! The phone clocks the frames it sends on its own crystal and the processing
//! board clocks the link on another, so the fill of the ring walks at the rate
//! of their disagreement: 4.41 frames per second at 100 ppm.
//!
//! Each drain of a started ring samples the fill before it takes a frame. In the
//! pump the drain runs once per block of the link, a steady clock set by the
//! processing board and blind to when the radio delivers. Bluedroid hands the
//! sink one chunk per media packet, at most 15 SBC frames of 128 frames each,
//! so the raw fill carries a sawtooth of up to 1920 frames per packet. Bluedroid
//! also decodes the packets it has queued back to back, so a late packet lands
//! together with the next one and steps the fill by both. Held around 2048
//! frames, the ring rides out a 1024-frame packet late by a whole period, since
//! the trough before it, `2048 - (1024 + BLOCK_FRAMES) / 2 - 1024`, stays over
//! an empty ring. It underruns on a 1920-frame packet late by a whole period,
//! `2048 - (1920 + BLOCK_FRAMES) / 2 - 1920`.
//!
//! The drain averages the samples over windows of `1 << WINDOW_SHIFT` drains,
//! and after each window the estimate moves toward that mean by
//! `TRACK_STEP_FRAMES` at the most. A radio late for a few seconds moves the
//! estimate by a few steps however deep the dip, and a drift of a few frames
//! per second stays well within what the estimate follows.
//!
//! The corrections draw on a signed credit, and a full credit of
//! `PACE_FRAME_DRAINS` pays for one: a removal above, a repeat below. Three
//! terms feed it at every drain. The drift estimate sets the base: the change of
//! the fill estimate plus the corrections applied, averaged through two stages
//! of `1 << DRIFT_SHIFT` drains once the start has settled, so a drift that
//! reaches the fill as 240-frame steps still pays corrections at its own mean
//! rate. The distance of the estimate from the centre adds a slow term inside
//! the band, `1 << INNER_PACE_SHIFT` times weaker, that recentres the fill.
//! Past `DEAD_BAND_FRAMES` the excess adds in full and takes out what a
//! transient left. At most one correction starts every `CORRECTION_SPACING`
//! drains.
//!
//! A correction resamples `CROSSFADE_FRAMES` frames of the ring into one frame
//! fewer or one more, both channels with the same weights, across as many
//! drains as it takes. The resampler reads an 8-tap Kaiser windowed sinc from a
//! table of 257 phases and interpolates between phases. Across a correction its
//! gain stays within `+0.08 / -0.01` dB at 6 kHz and 10 kHz and within
//! `-0.09` dB at 15 kHz, and falls to `-2.4` dB at 18 kHz. Spread over `L`
//! frames, one correction shifts the pitch of a tone at `f` by `f / L` for those
//! `L` frames. A ring of `2 * DEAD_BAND_FRAMES` frames or fewer never corrects.
//!
//! The corrections are the first line. A push into a full started ring drops
//! the incoming frames that do not fit, whole. A drain that runs the ring dry
//! leaves the rest of its block to the pump, which pads it with silence, and
//! the ring then waits for a new start. An underrun therefore costs one gap. A
//! flush, a new stream and an underrun all end with a new start, which rebuilds
//! the estimate, the drift estimate, the credit and the pacing. A suspend
//! flushes, so the stream that resumes learns its drift again.
//!
//! # Start
//!
//! The pre-drain samples of a stream of `C`-frame chunks average
//! `C / 2 - BLOCK_FRAMES / 2` frames under the level that follows a chunk, to
//! within half a block set by where the chunks fall against the drains. A ring
//! that corrects therefore starts at the prefill plus that offset for the
//! largest chunk it has seen, drops the oldest frames above that level, and
//! seeds the estimate at the prefill. It starts on the drain that follows a
//! lone chunk, so the next chunk lands one period later. A backlog that lands in
//! one burst after a stall or a resume makes the ring wait at its level for
//! `START_WAIT_DRAINS` drains, then for any push. Until it starts, a push into
//! a full ring drops the oldest frames rather than the incoming ones. Nothing
//! dropped before a start has played.
//!
//! The start learns the source it plays. The longest run of drains without a
//! push raises the start level to that gap plus a block, at most
//! `START_MARGIN_FRAMES` under the capacity of the ring. The gap loses one
//! drain per `GAP_DECAY_DRAINS` drains of play, 5.6 s. The gap that runs a ring
//! dry within `CLOSE_DRAINS` drains of its start counts. An underrun later than
//! that is a stall, and its gap does not count. A source that resumes short of
//! real time can still run the ring dry once or twice before it catches up,
//! each time as a faded dip.
//!
//! A suspend keeps the largest chunk and the longest gap, since the source and
//! the link that showed them come back with the stream that resumes. A new
//! stream forgets them.
//!
//! # Fades
//!
//! A ring that corrects scales every frame it gives out by a gain in
//! `FADE_FRAMES`ths. The gain rises one step per frame from silence at every
//! start, and follows the frames left in the ring down, so the last frame
//! before the ring runs dry is silent. A push that drops frames from a started
//! ring records where, and the gain falls to silence on the frames either side
//! of the frames lost. A start trims inside the silence before it. A flush of a
//! started ring keeps as many of its oldest frames as the gain has steps and
//! plays them out, fading to silence, before anything else. A gap, a trim, a
//! hard drop and a flush therefore play as a dip rather than a step.
//!
//! # The stream gate
//!
//! The whole chain runs at 44.1 kHz. The gate opens for SBC at 44.1 kHz in
//! every channel mode, mono, dual channel, stereo and joint stereo, which is
//! the set A2DP 1.4.1 table 4.3 makes mandatory for a sink, and every
//! negotiation decides again from the codec it carries. Any other stream leaves
//! the gate closed: every push drops its frames and the pump writes silence.
//!
//! Bluedroid sets the PCM stride of its SBC decoder to the channel count, so a
//! mono stream arrives as one 16-bit sample per frame. A push on a mono stream
//! writes that sample into both channels of the frame the ring stores, so the
//! ring, the drift correction, the fades and the pump run on frames of two
//! channels whatever the stream. That copy is framing, as the widening into
//! slots is: no sample changes value, and the mean the processing board takes
//! of two equal slots is the sample itself.

use crate::constants::SAMPLE_RATE_HZ;

/// Bytes of one decoded PCM frame: two 16-bit samples.
pub const PCM_FRAME_BYTES: usize = 4;

/// Bytes of one I2S frame on the link: two 32-bit slots.
pub const SLOT_FRAME_BYTES: usize = 8;

/// Ratio of slot bytes to PCM bytes.
const WIDENING: usize = 2;

/// Left shift that places a 16-bit sample in the upper half of its slot.
const SLOT_SHIFT: u32 = 16;

/// Frames the ring of the control board holds, 92.9 ms at 44.1 kHz.
pub const RING_FRAMES: usize = 4096;

/// Level the ring refills to before it gives out frames, 46.4 ms at 44.1 kHz.
///
/// Half the ring, so the ring has the same room for a burst from the radio as
/// for a stall of it.
pub const PREFILL_FRAMES: usize = 2048;

/// Frames one pump moves, 5.4 ms at 44.1 kHz.
pub const BLOCK_FRAMES: usize = 240;

/// Log2 of `CROSSFADE_FRAMES`, the resolution of the position a correction
/// resamples at.
const CROSSFADE_SHIFT: u32 = 12;

/// Frames of the ring one correction resamples, 92.9 ms at 44.1 kHz.
///
/// The largest power of two under `CORRECTION_SPACING * BLOCK_FRAMES`, so two
/// corrections never overlap. A correction shifts the pitch of a 6 kHz tone by
/// 1.46 Hz, `6000 / 4096`, for those frames.
const CROSSFADE_FRAMES: usize = 1 << CROSSFADE_SHIFT;

/// Taps of the resampler: frames `i - 3` to `i + 4` around a position `i + mu`.
const SINC_TAPS: usize = 8;

/// Frames the resampler reads past the last frame it took from the ring, and
/// frames it keeps behind it, the newest included.
const SINC_LOOKAHEAD: usize = SINC_TAPS / 2;

/// Log2 of the positions between two table phases.
const SINC_PHASE_SHIFT: u32 = CROSSFADE_SHIFT - 8;

/// Half a step between two table phases, which rounds their interpolation.
const SINC_PHASE_ROUND: i32 = 1 << (SINC_PHASE_SHIFT - 1);

/// Fraction bits of a resampler weight.
const SINC_FRACTION_BITS: u32 = 14;

/// Half a unit of a weighted sum, which rounds a resampled sample to nearest.
const SINC_ROUND: i32 = 1 << (SINC_FRACTION_BITS - 1);

/// Weights of the resampler, one row per phase `k / 256` of a frame, taps
/// `i - 3` to `i + 4`.
///
/// Each row holds `sinc(t - mu) * w(t - mu)` with a Kaiser window of beta 4 over
/// eight frames, scaled so it sums to exactly `1 << SINC_FRACTION_BITS`, the
/// rounding remainder on the largest tap. Row 0 and row 256 pass a frame through
/// untouched. The largest sum of magnitudes, at mid phase, is 1.735, so a
/// weighted sum of full-scale samples stays under `1.735 * 16384 * 32768`,
/// within `i32`.
const SINC_ROWS: [[i16; SINC_TAPS]; 257] =
[
    [0, 0, 0, 16384, 0, 0, 0, 0],
    [-7, 20, -57, 16384, 58, -20, 7, -1],
    [-14, 40, -114, 16386, 116, -41, 14, -3],
    [-21, 60, -170, 16385, 175, -62, 21, -4],
    [-27, 80, -225, 16382, 234, -82, 28, -6],
    [-34, 99, -280, 16379, 294, -103, 36, -7],
    [-41, 119, -334, 16376, 354, -124, 43, -9],
    [-47, 138, -388, 16373, 415, -146, 50, -11],
    [-53, 157, -442, 16366, 477, -167, 58, -12],
    [-60, 176, -494, 16361, 539, -189, 65, -14],
    [-66, 195, -546, 16352, 601, -210, 73, -15],
    [-72, 213, -598, 16346, 664, -232, 80, -17],
    [-79, 232, -649, 16337, 728, -254, 88, -19],
    [-85, 250, -699, 16327, 792, -276, 96, -21],
    [-91, 268, -749, 16316, 856, -298, 104, -22],
    [-97, 286, -799, 16306, 922, -321, 111, -24],
    [-102, 303, -847, 16293, 987, -343, 119, -26],
    [-108, 321, -895, 16280, 1053, -366, 127, -28],
    [-114, 338, -943, 16265, 1120, -388, 135, -29],
    [-120, 355, -990, 16251, 1187, -411, 143, -31],
    [-125, 372, -1036, 16234, 1255, -434, 151, -33],
    [-131, 389, -1082, 16217, 1323, -457, 160, -35],
    [-136, 405, -1127, 16200, 1391, -480, 168, -37],
    [-141, 422, -1172, 16180, 1461, -503, 176, -39],
    [-147, 438, -1216, 16163, 1530, -527, 184, -41],
    [-152, 454, -1260, 16142, 1600, -550, 193, -43],
    [-157, 470, -1303, 16121, 1671, -574, 201, -45],
    [-162, 485, -1345, 16099, 1742, -597, 209, -47],
    [-167, 500, -1387, 16077, 1813, -621, 218, -49],
    [-172, 516, -1428, 16053, 1885, -645, 226, -51],
    [-177, 531, -1468, 16027, 1957, -668, 235, -53],
    [-181, 545, -1508, 16001, 2030, -692, 244, -55],
    [-186, 560, -1548, 15977, 2103, -716, 252, -58],
    [-190, 574, -1586, 15948, 2177, -740, 261, -60],
    [-195, 588, -1624, 15921, 2251, -764, 269, -62],
    [-199, 602, -1662, 15893, 2325, -789, 278, -64],
    [-204, 616, -1699, 15863, 2400, -813, 287, -66],
    [-208, 630, -1735, 15832, 2475, -837, 296, -69],
    [-212, 643, -1771, 15800, 2551, -861, 305, -71],
    [-216, 656, -1806, 15769, 2627, -886, 313, -73],
    [-220, 669, -1841, 15736, 2704, -910, 322, -76],
    [-224, 682, -1875, 15702, 2781, -935, 331, -78],
    [-228, 694, -1908, 15667, 2858, -959, 340, -80],
    [-232, 706, -1941, 15633, 2936, -984, 349, -83],
    [-235, 718, -1973, 15595, 3014, -1008, 358, -85],
    [-239, 730, -2005, 15559, 3092, -1033, 367, -87],
    [-242, 742, -2036, 15520, 3171, -1057, 376, -90],
    [-246, 753, -2066, 15482, 3250, -1082, 385, -92],
    [-249, 764, -2096, 15444, 3329, -1107, 394, -95],
    [-252, 775, -2125, 15402, 3409, -1131, 403, -97],
    [-256, 786, -2154, 15362, 3489, -1156, 412, -99],
    [-259, 797, -2182, 15319, 3570, -1180, 421, -102],
    [-262, 807, -2209, 15277, 3650, -1205, 430, -104],
    [-265, 817, -2236, 15235, 3731, -1230, 439, -107],
    [-268, 827, -2262, 15189, 3813, -1254, 448, -109],
    [-270, 837, -2288, 15144, 3895, -1279, 457, -112],
    [-273, 846, -2313, 15098, 3977, -1303, 466, -114],
    [-276, 855, -2338, 15054, 4059, -1328, 475, -117],
    [-278, 864, -2362, 15006, 4142, -1352, 484, -120],
    [-281, 873, -2385, 14959, 4224, -1377, 493, -122],
    [-283, 882, -2408, 14909, 4308, -1401, 502, -125],
    [-285, 890, -2430, 14860, 4391, -1426, 511, -127],
    [-288, 898, -2451, 14810, 4475, -1450, 520, -130],
    [-290, 906, -2473, 14759, 4559, -1474, 529, -132],
    [-292, 914, -2493, 14707, 4643, -1498, 538, -135],
    [-294, 922, -2513, 14656, 4727, -1523, 547, -138],
    [-296, 929, -2532, 14602, 4812, -1547, 556, -140],
    [-298, 936, -2551, 14549, 4897, -1571, 565, -143],
    [-300, 943, -2569, 14495, 4982, -1595, 574, -146],
    [-301, 950, -2587, 14439, 5067, -1619, 583, -148],
    [-303, 956, -2604, 14383, 5153, -1642, 592, -151],
    [-305, 962, -2620, 14327, 5238, -1666, 601, -153],
    [-306, 969, -2636, 14269, 5324, -1690, 610, -156],
    [-307, 974, -2651, 14212, 5410, -1713, 618, -159],
    [-309, 980, -2666, 14153, 5497, -1737, 627, -161],
    [-310, 985, -2680, 14094, 5583, -1760, 636, -164],
    [-311, 991, -2694, 14033, 5670, -1783, 645, -167],
    [-312, 996, -2707, 13973, 5756, -1806, 653, -169],
    [-313, 1000, -2720, 13913, 5843, -1829, 662, -172],
    [-314, 1005, -2732, 13852, 5930, -1852, 670, -175],
    [-315, 1009, -2743, 13789, 6017, -1875, 679, -177],
    [-316, 1014, -2754, 13725, 6105, -1897, 687, -180],
    [-317, 1018, -2764, 13662, 6192, -1920, 696, -183],
    [-318, 1021, -2774, 13599, 6279, -1942, 704, -185],
    [-318, 1025, -2784, 13533, 6367, -1964, 713, -188],
    [-319, 1028, -2792, 13468, 6455, -1986, 721, -191],
    [-319, 1031, -2801, 13403, 6542, -2008, 729, -193],
    [-320, 1034, -2809, 13338, 6630, -2030, 737, -196],
    [-320, 1037, -2816, 13269, 6718, -2051, 745, -198],
    [-321, 1040, -2823, 13202, 6806, -2073, 754, -201],
    [-321, 1042, -2829, 13134, 6894, -2094, 762, -204],
    [-321, 1044, -2834, 13065, 6982, -2115, 769, -206],
    [-321, 1046, -2840, 12997, 7070, -2136, 777, -209],
    [-321, 1048, -2844, 12924, 7159, -2156, 785, -211],
    [-321, 1050, -2849, 12855, 7247, -2177, 793, -214],
    [-321, 1051, -2852, 12783, 7335, -2197, 801, -216],
    [-321, 1052, -2855, 12713, 7423, -2217, 808, -219],
    [-321, 1053, -2858, 12641, 7511, -2237, 816, -221],
    [-321, 1054, -2860, 12569, 7599, -2256, 823, -224],
    [-320, 1055, -2862, 12494, 7688, -2275, 830, -226],
    [-320, 1055, -2863, 12422, 7776, -2295, 838, -229],
    [-320, 1056, -2864, 12348, 7864, -2314, 845, -231],
    [-319, 1056, -2865, 12274, 7952, -2332, 852, -234],
    [-319, 1056, -2864, 12199, 8040, -2351, 859, -236],
    [-318, 1056, -2864, 12124, 8128, -2369, 866, -239],
    [-317, 1055, -2863, 12048, 8216, -2387, 873, -241],
    [-317, 1055, -2861, 11970, 8304, -2404, 880, -243],
    [-316, 1054, -2859, 11896, 8391, -2422, 886, -246],
    [-315, 1053, -2857, 11818, 8479, -2439, 893, -248],
    [-314, 1052, -2854, 11740, 8567, -2456, 899, -250],
    [-313, 1050, -2851, 11665, 8654, -2473, 905, -253],
    [-312, 1049, -2847, 11585, 8741, -2489, 912, -255],
    [-311, 1047, -2843, 11506, 8829, -2505, 918, -257],
    [-310, 1046, -2838, 11426, 8916, -2521, 924, -259],
    [-309, 1044, -2833, 11347, 9003, -2536, 930, -262],
    [-308, 1042, -2827, 11267, 9089, -2551, 936, -264],
    [-307, 1039, -2822, 11189, 9176, -2566, 941, -266],
    [-306, 1037, -2815, 11108, 9262, -2581, 947, -268],
    [-304, 1034, -2809, 11027, 9349, -2595, 952, -270],
    [-303, 1032, -2801, 10944, 9435, -2609, 958, -272],
    [-302, 1029, -2794, 10864, 9521, -2623, 963, -274],
    [-300, 1026, -2786, 10781, 9607, -2636, 968, -276],
    [-299, 1022, -2778, 10701, 9692, -2649, 973, -278],
    [-297, 1019, -2769, 10618, 9777, -2662, 978, -280],
    [-296, 1016, -2760, 10534, 9863, -2674, 983, -282],
    [-294, 1012, -2750, 10452, 9947, -2686, 987, -284],
    [-292, 1008, -2741, 10368, 10032, -2698, 992, -285],
    [-291, 1004, -2730, 10284, 10117, -2709, 996, -287],
    [-289, 1000, -2720, 10201, 10201, -2720, 1000, -289],
    [-287, 996, -2709, 10117, 10284, -2730, 1004, -291],
    [-285, 992, -2698, 10032, 10368, -2741, 1008, -292],
    [-284, 987, -2686, 9947, 10452, -2750, 1012, -294],
    [-282, 983, -2674, 9863, 10534, -2760, 1016, -296],
    [-280, 978, -2662, 9777, 10618, -2769, 1019, -297],
    [-278, 973, -2649, 9692, 10701, -2778, 1022, -299],
    [-276, 968, -2636, 9607, 10781, -2786, 1026, -300],
    [-274, 963, -2623, 9521, 10864, -2794, 1029, -302],
    [-272, 958, -2609, 9435, 10944, -2801, 1032, -303],
    [-270, 952, -2595, 9349, 11027, -2809, 1034, -304],
    [-268, 947, -2581, 9262, 11108, -2815, 1037, -306],
    [-266, 941, -2566, 9176, 11189, -2822, 1039, -307],
    [-264, 936, -2551, 9089, 11267, -2827, 1042, -308],
    [-262, 930, -2536, 9003, 11347, -2833, 1044, -309],
    [-259, 924, -2521, 8916, 11426, -2838, 1046, -310],
    [-257, 918, -2505, 8829, 11506, -2843, 1047, -311],
    [-255, 912, -2489, 8741, 11585, -2847, 1049, -312],
    [-253, 905, -2473, 8654, 11665, -2851, 1050, -313],
    [-250, 899, -2456, 8567, 11740, -2854, 1052, -314],
    [-248, 893, -2439, 8479, 11818, -2857, 1053, -315],
    [-246, 886, -2422, 8391, 11896, -2859, 1054, -316],
    [-243, 880, -2404, 8304, 11970, -2861, 1055, -317],
    [-241, 873, -2387, 8216, 12048, -2863, 1055, -317],
    [-239, 866, -2369, 8128, 12124, -2864, 1056, -318],
    [-236, 859, -2351, 8040, 12199, -2864, 1056, -319],
    [-234, 852, -2332, 7952, 12274, -2865, 1056, -319],
    [-231, 845, -2314, 7864, 12348, -2864, 1056, -320],
    [-229, 838, -2295, 7776, 12422, -2863, 1055, -320],
    [-226, 830, -2275, 7688, 12494, -2862, 1055, -320],
    [-224, 823, -2256, 7599, 12569, -2860, 1054, -321],
    [-221, 816, -2237, 7511, 12641, -2858, 1053, -321],
    [-219, 808, -2217, 7423, 12713, -2855, 1052, -321],
    [-216, 801, -2197, 7335, 12783, -2852, 1051, -321],
    [-214, 793, -2177, 7247, 12855, -2849, 1050, -321],
    [-211, 785, -2156, 7159, 12924, -2844, 1048, -321],
    [-209, 777, -2136, 7070, 12997, -2840, 1046, -321],
    [-206, 769, -2115, 6982, 13065, -2834, 1044, -321],
    [-204, 762, -2094, 6894, 13134, -2829, 1042, -321],
    [-201, 754, -2073, 6806, 13202, -2823, 1040, -321],
    [-198, 745, -2051, 6718, 13269, -2816, 1037, -320],
    [-196, 737, -2030, 6630, 13338, -2809, 1034, -320],
    [-193, 729, -2008, 6542, 13403, -2801, 1031, -319],
    [-191, 721, -1986, 6455, 13468, -2792, 1028, -319],
    [-188, 713, -1964, 6367, 13533, -2784, 1025, -318],
    [-185, 704, -1942, 6279, 13599, -2774, 1021, -318],
    [-183, 696, -1920, 6192, 13662, -2764, 1018, -317],
    [-180, 687, -1897, 6105, 13725, -2754, 1014, -316],
    [-177, 679, -1875, 6017, 13789, -2743, 1009, -315],
    [-175, 670, -1852, 5930, 13852, -2732, 1005, -314],
    [-172, 662, -1829, 5843, 13913, -2720, 1000, -313],
    [-169, 653, -1806, 5756, 13973, -2707, 996, -312],
    [-167, 645, -1783, 5670, 14033, -2694, 991, -311],
    [-164, 636, -1760, 5583, 14094, -2680, 985, -310],
    [-161, 627, -1737, 5497, 14153, -2666, 980, -309],
    [-159, 618, -1713, 5410, 14212, -2651, 974, -307],
    [-156, 610, -1690, 5324, 14269, -2636, 969, -306],
    [-153, 601, -1666, 5238, 14327, -2620, 962, -305],
    [-151, 592, -1642, 5153, 14383, -2604, 956, -303],
    [-148, 583, -1619, 5067, 14439, -2587, 950, -301],
    [-146, 574, -1595, 4982, 14495, -2569, 943, -300],
    [-143, 565, -1571, 4897, 14549, -2551, 936, -298],
    [-140, 556, -1547, 4812, 14602, -2532, 929, -296],
    [-138, 547, -1523, 4727, 14656, -2513, 922, -294],
    [-135, 538, -1498, 4643, 14707, -2493, 914, -292],
    [-132, 529, -1474, 4559, 14759, -2473, 906, -290],
    [-130, 520, -1450, 4475, 14810, -2451, 898, -288],
    [-127, 511, -1426, 4391, 14860, -2430, 890, -285],
    [-125, 502, -1401, 4308, 14909, -2408, 882, -283],
    [-122, 493, -1377, 4224, 14959, -2385, 873, -281],
    [-120, 484, -1352, 4142, 15006, -2362, 864, -278],
    [-117, 475, -1328, 4059, 15054, -2338, 855, -276],
    [-114, 466, -1303, 3977, 15098, -2313, 846, -273],
    [-112, 457, -1279, 3895, 15144, -2288, 837, -270],
    [-109, 448, -1254, 3813, 15189, -2262, 827, -268],
    [-107, 439, -1230, 3731, 15235, -2236, 817, -265],
    [-104, 430, -1205, 3650, 15277, -2209, 807, -262],
    [-102, 421, -1180, 3570, 15319, -2182, 797, -259],
    [-99, 412, -1156, 3489, 15362, -2154, 786, -256],
    [-97, 403, -1131, 3409, 15402, -2125, 775, -252],
    [-95, 394, -1107, 3329, 15444, -2096, 764, -249],
    [-92, 385, -1082, 3250, 15482, -2066, 753, -246],
    [-90, 376, -1057, 3171, 15520, -2036, 742, -242],
    [-87, 367, -1033, 3092, 15559, -2005, 730, -239],
    [-85, 358, -1008, 3014, 15595, -1973, 718, -235],
    [-83, 349, -984, 2936, 15633, -1941, 706, -232],
    [-80, 340, -959, 2858, 15667, -1908, 694, -228],
    [-78, 331, -935, 2781, 15702, -1875, 682, -224],
    [-76, 322, -910, 2704, 15736, -1841, 669, -220],
    [-73, 313, -886, 2627, 15769, -1806, 656, -216],
    [-71, 305, -861, 2551, 15800, -1771, 643, -212],
    [-69, 296, -837, 2475, 15832, -1735, 630, -208],
    [-66, 287, -813, 2400, 15863, -1699, 616, -204],
    [-64, 278, -789, 2325, 15893, -1662, 602, -199],
    [-62, 269, -764, 2251, 15921, -1624, 588, -195],
    [-60, 261, -740, 2177, 15948, -1586, 574, -190],
    [-58, 252, -716, 2103, 15977, -1548, 560, -186],
    [-55, 244, -692, 2030, 16001, -1508, 545, -181],
    [-53, 235, -668, 1957, 16027, -1468, 531, -177],
    [-51, 226, -645, 1885, 16053, -1428, 516, -172],
    [-49, 218, -621, 1813, 16077, -1387, 500, -167],
    [-47, 209, -597, 1742, 16099, -1345, 485, -162],
    [-45, 201, -574, 1671, 16121, -1303, 470, -157],
    [-43, 193, -550, 1600, 16142, -1260, 454, -152],
    [-41, 184, -527, 1530, 16163, -1216, 438, -147],
    [-39, 176, -503, 1461, 16180, -1172, 422, -141],
    [-37, 168, -480, 1391, 16200, -1127, 405, -136],
    [-35, 160, -457, 1323, 16217, -1082, 389, -131],
    [-33, 151, -434, 1255, 16234, -1036, 372, -125],
    [-31, 143, -411, 1187, 16251, -990, 355, -120],
    [-29, 135, -388, 1120, 16265, -943, 338, -114],
    [-28, 127, -366, 1053, 16280, -895, 321, -108],
    [-26, 119, -343, 987, 16293, -847, 303, -102],
    [-24, 111, -321, 922, 16306, -799, 286, -97],
    [-22, 104, -298, 856, 16316, -749, 268, -91],
    [-21, 96, -276, 792, 16327, -699, 250, -85],
    [-19, 88, -254, 728, 16337, -649, 232, -79],
    [-17, 80, -232, 664, 16346, -598, 213, -72],
    [-15, 73, -210, 601, 16352, -546, 195, -66],
    [-14, 65, -189, 539, 16361, -494, 176, -60],
    [-12, 58, -167, 477, 16366, -442, 157, -53],
    [-11, 50, -146, 415, 16373, -388, 138, -47],
    [-9, 43, -124, 354, 16376, -334, 119, -41],
    [-7, 36, -103, 294, 16379, -280, 99, -34],
    [-6, 28, -82, 234, 16382, -225, 80, -27],
    [-4, 21, -62, 175, 16385, -170, 60, -21],
    [-3, 14, -41, 116, 16386, -114, 40, -14],
    [-1, 7, -20, 58, 16384, -57, 20, -7],
    [0, 0, 0, 0, 16384, 0, 0, 0],
];

/// Fraction bits of the fill estimate.
const ESTIMATE_FRACTION_BITS: u32 = 16;

/// Log2 of the drains one fill window averages.
///
/// 32 drains, 174 ms at 44.1 kHz, four periods of 1920-frame chunks.
const WINDOW_SHIFT: u32 = 5;

/// Frames the estimate moves toward the mean of one window at the most.
///
/// The estimate follows 23 frames per second, `4 * 44100 / (240 * 32)`, 2.5
/// times the cap of the corrections, so it keeps up with the fill while they
/// run. A radio late for 3 s, 18 windows started, moves it by 72 frames at the
/// most.
const TRACK_STEP_FRAMES: i64 = 4;

/// Distance of the estimate from the centre of the ring past which its excess
/// adds to the credit in full, in frames, 2.9 ms at 44.1 kHz.
///
/// A chunk stream locked to the link carries a drift as one 240-frame step of
/// the fill per slip, and the drift estimate spreads the corrections over the
/// steps, so the estimate swings over 240 frames, within `2 * 128`. Twice the
/// band also clears the 72 frames a radio late for 3 s moves the estimate.
const DEAD_BAND_FRAMES: usize = 128;

/// Log2 of how much weaker the term inside the band is than the excess past it.
///
/// At the band edge it adds 0.0039 corrections per drain, `128 / (1024 * 32)`,
/// 0.72 per second, and it recentres the fill with a time constant of 178 s,
/// `1024 * 32` drains, slow against the 55 s between two 240-frame steps of a
/// 100 ppm drift.
const INNER_PACE_SHIFT: u32 = 5;

/// Log2 of the drains each stage of the drift estimate averages over.
///
/// 16384 drains, 89 s at 44.1 kHz. A 240-frame step every 55 s, a 100 ppm drift
/// locked to the link, has its fundamental taken down to 10 percent by each
/// stage, `1 / sqrt(1 + (2 * pi * 89 / 55)^2)`, and to 1 percent by both.
const DRIFT_SHIFT: u32 = 14;

/// Drains after a start before the drift estimate learns, 44.6 s at 44.1 kHz.
///
/// The fill estimate first crosses the start offset: 561 frames at most with
/// 10 ms of jitter, at 23 frames per second, 24.4 s. Movement of the estimate
/// in that time is the start, not a drift.
const DRIFT_WARMUP_DRAINS: u32 = 1 << 13;

/// Largest chunk Bluedroid hands the sink, in frames: 15 SBC frames of 16 blocks
/// and 8 subbands. A larger push counts as this size for the start level.
const LARGEST_CHUNK_FRAMES: usize = 1920;

/// Drains from one correction to the next at the least.
///
/// Twenty drains of `BLOCK_FRAMES` cap the corrections at 9.19 frames per
/// second, 2.08 times the 4.41 frames per second of a 100 ppm drift.
const CORRECTION_SPACING: u32 = 20;

/// Credit that pays for one correction, in frames of excess times drains.
///
/// Past the band an excess of 51 frames, `1024 / 20`, reaches the cap of the
/// corrections, so a start offset or a stall leaves quickly.
const PACE_FRAME_DRAINS: i64 = 1024;

/// Log2 of `FADE_FRAMES`.
const FADE_SHIFT: u32 = 8;

/// Frames a fade takes from full level to silence or back, 5.8 ms at 44.1 kHz.
///
/// A linear ramp over `T` seconds holds the spectrum of a step down by about
/// `1 / (pi * f * T)` against the bare step: 25 dB at 1 kHz and 35 dB at 3 kHz,
/// where the ear is most sensitive. A gap is at least a start level of 46 ms,
/// so the two fades around it take a quarter of its length or less.
const FADE_FRAMES: usize = 1 << FADE_SHIFT;

/// Regions of spliced frames a ring keeps for its fades.
///
/// Two splices closer than `2 * FADE_FRAMES` merge into one region, and a region
/// is forgotten once played. The regions still ahead of the output of a ring
/// of 4096 frames therefore sit more than 512 frames apart and number 8 at the
/// most. A ninth merges into the last.
const SPLICE_SLOTS: usize = 8;

/// Frames a start level stays under the capacity of the ring: two blocks.
const START_MARGIN_FRAMES: usize = 2 * BLOCK_FRAMES;

/// Drains after a start within which the gap that runs the ring dry counts
/// toward the start level, 5.0 s at 44.1 kHz.
///
/// A gap that close to a start is one the start level could have held. A
/// stream that plays for 5 s has shown its steady delivery, and a gap after
/// that is a stall.
const CLOSE_DRAINS: u32 = 919;

/// Drains of play that take one drain off the longest delivery gap measured.
///
/// 1024 drains, 5.6 s, so the longest gap a ring can hold fades out over
/// 95 s of play.
const GAP_DECAY_DRAINS: u32 = 1024;

/// Drains a ring at its start level waits for a lone chunk before it starts
/// anyway.
///
/// 16 drains, 87 ms, two periods of 1920-frame chunks: the chunk after a backlog
/// lands within one period, and a radio late by up to one more period, 43 ms,
/// still starts the ring on a lone chunk.
const START_WAIT_DRAINS: u32 = 16;

/// Failure of a conversion between PCM bytes and slot bytes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BridgeError
{
    /// The PCM bytes do not hold a whole number of frames.
    PartialFrame,
    /// The slot buffer cannot hold the widened frames.
    OutputTooShort,
}

/// Mask of the sampling frequency bits in octet 0 of the SBC codec information
/// element (A2DP specification, SBC codec specific information elements).
const SBC_FREQUENCY_MASK: u8 = 0xF0;

/// Mask of the channel mode bits in octet 0 of the SBC codec information element.
const SBC_CHANNEL_MODE_MASK: u8 = 0x0F;

/// Sampling frequency bit of 16 kHz.
const SBC_16_KHZ: u8 = 0x80;

/// Sampling frequency bit of 32 kHz.
const SBC_32_KHZ: u8 = 0x40;

/// Sampling frequency bit of 44.1 kHz.
const SBC_44_1_KHZ: u8 = 0x20;

/// Sampling frequency bit of 48 kHz.
const SBC_48_KHZ: u8 = 0x10;

/// Channel mode bit of mono.
const SBC_MONO: u8 = 0x08;

/// Bytes of one decoded PCM frame of a mono stream: one 16-bit sample.
const MONO_FRAME_BYTES: usize = 2;

/// Codec of a negotiated A2DP stream.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StreamCodec
{
    /// SBC, carrying octet 0 of its codec information element: the sampling
    /// frequency in the upper four bits, the channel mode in the lower four.
    Sbc(u8),
    /// Any codec other than SBC.
    Other,
}

/// Channels the decoder of a forwarded stream puts in one PCM frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Channels
{
    /// One sample per frame, from SBC in mono.
    Mono,
    /// Two samples per frame, left first, from SBC in any other channel mode.
    Stereo,
}

/// Verdict of the stream gate on a negotiated stream.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StreamVerdict
{
    /// SBC at the rate of the chain. The bridge forwards it, and the channels
    /// name how its decoded frames are laid out.
    Forward(Channels),
    /// A codec other than SBC.
    NotSbc,
    /// SBC at another sampling frequency, in hertz.
    WrongRate(u32),
    /// SBC whose octet 0 names no single sampling frequency or no single
    /// channel mode.
    Malformed,
}

/// Returns the verdict of the stream gate on `codec`.
///
/// Forwards SBC at 44.1 kHz in every channel mode, mono as `Channels::Mono`
/// and the three others as `Channels::Stereo`. A configured stream names
/// exactly one sampling frequency and one channel mode, so an octet with none
/// or several of either is `Malformed`.
#[must_use]
pub const fn stream_verdict(codec: StreamCodec) -> StreamVerdict
{
    let StreamCodec::Sbc(octet) = codec
    else
    {
        return StreamVerdict::NotSbc;
    };

    let frequency = octet & SBC_FREQUENCY_MASK;
    let mode = octet & SBC_CHANNEL_MODE_MASK;

    if mode.count_ones() != 1
    {
        return StreamVerdict::Malformed;
    }

    let rate_hz = match frequency
    {
        SBC_16_KHZ => 16_000,
        SBC_32_KHZ => 32_000,
        SBC_44_1_KHZ => 44_100,
        SBC_48_KHZ => 48_000,
        _ => return StreamVerdict::Malformed,
    };

    if rate_hz != SAMPLE_RATE_HZ
    {
        return StreamVerdict::WrongRate(rate_hz);
    }

    if mode == SBC_MONO
    {
        return StreamVerdict::Forward(Channels::Mono);
    }

    StreamVerdict::Forward(Channels::Stereo)
}

/// What a push did with a chunk.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Pushed
{
    /// Frames the ring took, the leading frames of the chunk.
    pub accepted: usize,
    /// Whole frames dropped, because the ring was full or the gate is closed.
    pub dropped: usize,
    /// Bytes of the trailing partial frame, discarded.
    pub trailing_bytes: usize,
}

/// Widens interleaved 16-bit PCM frames into 32-bit I2S slot frames.
///
/// Writes each sample, in order, into the upper 16 bits of its slot, little
/// endian, so the left sample of a frame lands in slot 0 and the right one in
/// slot 1. The lower 16 bits of every slot are zero.
///
/// Returns the bytes written to `slots`, twice the length of `pcm`.
///
/// # Errors
///
/// `PartialFrame` when `pcm` is not a whole number of frames, `OutputTooShort`
/// when `slots` is shorter than twice `pcm`. Both checks run before the first
/// write.
fn widen_into(pcm: &[u8], slots: &mut [u8]) -> Result<usize, BridgeError>
{
    let (frames, partial) = pcm.as_chunks::<PCM_FRAME_BYTES>();

    if !partial.is_empty()
    {
        return Err(BridgeError::PartialFrame);
    }

    let Some(needed) = pcm.len().checked_mul(WIDENING)
    else
    {
        return Err(BridgeError::OutputTooShort);
    };

    if slots.len() < needed
    {
        return Err(BridgeError::OutputTooShort);
    }

    let (links, _) = slots.as_chunks_mut::<SLOT_FRAME_BYTES>();

    for (&[l0, l1, r0, r1], link) in frames.iter().zip(links.iter_mut())
    {
        let left = widen_sample(l0, l1);
        let right = widen_sample(r0, r1);

        for (out, byte) in link.iter_mut().zip(left.into_iter().chain(right))
        {
            *out = byte;
        }
    }

    Ok(needed)
}

/// Returns the slot bytes of the little endian sample `[low, high]`.
fn widen_sample(low: u8, high: u8) -> [u8; 4]
{
    (i32::from(i16::from_le_bytes([low, high])) << SLOT_SHIFT).to_le_bytes()
}

/// Fixed capacity queue of whole PCM frames.
struct FrameRing<const FRAMES: usize>
{
    frames: [[u8; PCM_FRAME_BYTES]; FRAMES],
    head: usize,
    len: usize,
}

impl<const FRAMES: usize> FrameRing<FRAMES>
{
    /// Builds an empty ring.
    const fn new() -> Self
    {
        Self
        {
            frames: [[0; PCM_FRAME_BYTES]; FRAMES],
            head: 0,
            len: 0,
        }
    }

    /// Keeps the `len` oldest frames at the most.
    fn truncate(&mut self, len: usize)
    {
        self.len = self.len.min(len);
    }

    /// Appends `frame`. Returns false, leaving the ring as it was, when full.
    fn push(&mut self, frame: [u8; PCM_FRAME_BYTES]) -> bool
    {
        if self.len >= FRAMES
        {
            return false;
        }

        let Some(tail) = self
            .head
            .checked_add(self.len)
            .and_then(|end| end.checked_rem(FRAMES))
        else
        {
            return false;
        };

        let Some(slot) = self.frames.get_mut(tail)
        else
        {
            return false;
        };

        *slot = frame;
        self.len = self.len.saturating_add(1);
        true
    }

    /// Returns the frame `offset` places after the oldest, or `None` past the
    /// frames the ring holds.
    fn peek(&self, offset: usize) -> Option<[u8; PCM_FRAME_BYTES]>
    {
        if offset >= self.len
        {
            return None;
        }

        let mut index = self.head.saturating_add(offset);

        if index >= FRAMES
        {
            index = index.saturating_sub(FRAMES);
        }

        self.frames.get(index).copied()
    }

    /// Removes and returns the oldest frame, or `None` when empty.
    fn pop(&mut self) -> Option<[u8; PCM_FRAME_BYTES]>
    {
        if self.len == 0
        {
            return None;
        }

        let frame = *self.frames.get(self.head)?;
        self.head = self
            .head
            .checked_add(1)
            .and_then(|next| next.checked_rem(FRAMES))
            .unwrap_or(0);
        self.len = self.len.saturating_sub(1);
        Some(frame)
    }
}

/// Correction a drain applies to the frames it takes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Correction
{
    /// Gives out one frame more than the drain takes from the ring.
    Insert,
    /// Gives out one frame fewer than the drain takes from the ring.
    Remove,
}

/// Returns `frames` in the fixed point of the fill estimate.
fn fixed_frames(frames: usize) -> i64
{
    i64::from(u32::try_from(frames).unwrap_or(u32::MAX)) << ESTIMATE_FRACTION_BITS
}

/// Fill estimate, drift estimate and pacing of the corrections of a started
/// ring.
struct DriftControl
{
    estimate: i64,
    window_sum: u64,
    window_len: u32,
    previous: i64,
    age: u32,
    drift_fast: i64,
    drift_slow: i64,
    credit: i64,
    cooldown: u32,
}

impl DriftControl
{
    /// Builds a controller whose estimate stands at `mean` frames, with no drift
    /// learnt, free to correct on its next update.
    fn seeded(mean: usize) -> Self
    {
        let estimate = fixed_frames(mean);

        Self
        {
            estimate,
            window_sum: 0,
            window_len: 0,
            previous: estimate,
            age: 0,
            drift_fast: 0,
            drift_slow: 0,
            credit: 0,
            cooldown: 0,
        }
    }

    /// Returns the drift estimate, in fixed point frames per drain.
    const fn drift(&self) -> i64
    {
        self.drift_slow >> DRIFT_SHIFT
    }

    /// Folds the fill sample `fill` into the estimate and returns the
    /// correction for this drain.
    ///
    /// Every `1 << WINDOW_SHIFT` samples the estimate moves toward their mean
    /// by `TRACK_STEP_FRAMES` at the most. Each drain adds to a credit capped at
    /// one correction of either sign: the drift estimate, the distance from
    /// `centre` shifted down by `INNER_PACE_SHIFT`, and the excess past
    /// `DEAD_BAND_FRAMES`. A drain corrects when the credit is full and the last
    /// correction is `CORRECTION_SPACING` drains old or more, and the
    /// correction spends the credit. `idle` false, a correction still running,
    /// defers the next one without spending the spacing.
    fn update(&mut self, fill: usize, centre: usize, idle: bool) -> Option<Correction>
    {
        self.window_sum = self.window_sum.saturating_add(u64::try_from(fill).unwrap_or(u64::MAX));
        self.window_len = self.window_len.saturating_add(1);

        if self.window_len >= 1 << WINDOW_SHIFT
        {
            let mean = i64::try_from(self.window_sum).unwrap_or(i64::MAX) << (ESTIMATE_FRACTION_BITS - WINDOW_SHIFT);
            let step = TRACK_STEP_FRAMES << ESTIMATE_FRACTION_BITS;
            self.estimate = if mean > self.estimate
            {
                self.estimate.saturating_add(step).min(mean)
            }
            else
            {
                self.estimate.saturating_sub(step).max(mean)
            };
            self.window_sum = 0;
            self.window_len = 0;
        }

        let error = self.estimate.saturating_sub(fixed_frames(centre));
        let band = fixed_frames(DEAD_BAND_FRAMES);
        let excess = if error > band
        {
            error.saturating_sub(band)
        }
        else if error < band.saturating_neg()
        {
            error.saturating_add(band)
        }
        else
        {
            0
        };

        let pace = PACE_FRAME_DRAINS << ESTIMATE_FRACTION_BITS;
        let inflow = self
            .drift()
            .saturating_mul(PACE_FRAME_DRAINS)
            .saturating_add(error >> INNER_PACE_SHIFT)
            .saturating_add(excess);
        self.credit = self.credit.saturating_add(inflow).clamp(pace.saturating_neg(), pace);

        let cooling = self.cooldown > 0;
        self.cooldown = self.cooldown.saturating_sub(1);

        let correction = if cooling || !idle
        {
            None
        }
        else if self.credit >= pace
        {
            self.credit = self.credit.saturating_sub(pace);
            Some(Correction::Remove)
        }
        else if self.credit <= pace.saturating_neg()
        {
            self.credit = self.credit.saturating_add(pace);
            Some(Correction::Insert)
        }
        else
        {
            None
        };

        if correction.is_some()
        {
            self.cooldown = CORRECTION_SPACING.saturating_sub(1);
        }

        self.observe(correction);
        correction
    }

    /// Folds the move of the estimate since the last drain and `correction`
    /// into the drift estimate, once `DRIFT_WARMUP_DRAINS` drains have passed.
    ///
    /// A drift moves the fill and the corrections move it back, so the move plus
    /// the frames the corrections take out is the drift whatever paid for them.
    /// An offset the corrections remove moves the estimate by as much as they
    /// take out, and adds nothing.
    fn observe(&mut self, correction: Option<Correction>)
    {
        let moved = self.estimate.saturating_sub(self.previous);
        self.previous = self.estimate;

        if self.age < DRIFT_WARMUP_DRAINS
        {
            self.age = self.age.saturating_add(1);
            return;
        }

        let taken = match correction
        {
            Some(Correction::Remove) => fixed_frames(1),
            Some(Correction::Insert) => fixed_frames(1).saturating_neg(),
            None => 0,
        };
        let sample = moved.saturating_add(taken);
        self.drift_fast = self
            .drift_fast
            .saturating_add(sample)
            .saturating_sub(self.drift_fast >> DRIFT_SHIFT);
        self.drift_slow = self
            .drift_slow
            .saturating_add(self.drift_fast >> DRIFT_SHIFT)
            .saturating_sub(self.drift_slow >> DRIFT_SHIFT);
    }
}

/// Progress of the correction a started ring spreads over its output frames.
#[derive(Debug, Clone, Copy)]
struct Segment
{
    correction: Correction,
    index: usize,
}

/// Returns `frame` scaled by `gain` in `FADE_FRAMES`ths, both channels alike.
///
/// A gain of `FADE_FRAMES` returns the frame bit for bit.
fn faded(frame: [u8; PCM_FRAME_BYTES], gain: u32) -> [u8; PCM_FRAME_BYTES]
{
    if gain >= 1 << FADE_SHIFT
    {
        return frame;
    }

    let gain = i32::try_from(gain).unwrap_or(0);
    let [l0, l1, r0, r1] = frame;
    let [l0, l1] = saturate(i32::from(i16::from_le_bytes([l0, l1])).saturating_mul(gain) >> FADE_SHIFT);
    let [r0, r1] = saturate(i32::from(i16::from_le_bytes([r0, r1])).saturating_mul(gain) >> FADE_SHIFT);
    [l0, l1, r0, r1]
}

/// Returns the little endian 16-bit sample of `value`, saturated at its range.
fn saturate(value: i32) -> [u8; 2]
{
    let clamped = value.clamp(i32::from(i16::MIN), i32::from(i16::MAX));
    i16::try_from(clamped).unwrap_or(0).to_le_bytes()
}

/// Running statistics of a bridge between two snapshots.
#[derive(Debug, Clone, Copy)]
struct StatsAccumulator
{
    samples: u32,
    fill_min: usize,
    fill_max: usize,
    fill_sum: u64,
    inserted: u32,
    removed: u32,
    dropped: u32,
    trimmed: u32,
    underruns: u32,
}

impl StatsAccumulator
{
    /// Builds an accumulator that has seen nothing.
    const fn new() -> Self
    {
        Self
        {
            samples: 0,
            fill_min: usize::MAX,
            fill_max: 0,
            fill_sum: 0,
            inserted: 0,
            removed: 0,
            dropped: 0,
            trimmed: 0,
            underruns: 0,
        }
    }

    /// Folds one fill sample in.
    fn sample(&mut self, fill: usize)
    {
        self.samples = self.samples.saturating_add(1);
        self.fill_min = self.fill_min.min(fill);
        self.fill_max = self.fill_max.max(fill);
        self.fill_sum = self
            .fill_sum
            .saturating_add(u64::try_from(fill).unwrap_or(u64::MAX));
    }

    /// Returns what the accumulator has seen.
    fn snapshot(&self) -> BridgeStats
    {
        let mean = self
            .fill_sum
            .checked_div(u64::from(self.samples))
            .and_then(|mean| usize::try_from(mean).ok())
            .unwrap_or(0);

        BridgeStats
        {
            samples: self.samples,
            fill_min: if self.samples == 0 { 0 } else { self.fill_min },
            fill_max: self.fill_max,
            fill_mean: mean,
            inserted: self.inserted,
            removed: self.removed,
            dropped: self.dropped,
            trimmed: self.trimmed,
            underruns: self.underruns,
        }
    }
}

/// Statistics of a bridge over the period since the previous snapshot.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct BridgeStats
{
    /// Drains of a started ring, each of which sampled the fill.
    pub samples: u32,
    /// Lowest fill sample, in frames, 0 when `samples` is 0.
    pub fill_min: usize,
    /// Highest fill sample, in frames.
    pub fill_max: usize,
    /// Mean of the fill samples, in frames, rounded down.
    pub fill_mean: usize,
    /// Corrections that repeated a frame.
    pub inserted: u32,
    /// Corrections that removed a frame.
    pub removed: u32,
    /// Frames of a forwarded stream dropped because the ring was full.
    pub dropped: u32,
    /// Frames dropped from the ring at a start, before any of them played.
    pub trimmed: u32,
    /// Drains that ran the ring dry before they filled their buffer.
    pub underruns: u32,
}

/// Frame ring, stream gate, prefill and drift correction between the A2DP sink
/// and the I2S pump.
pub struct Bridge<const FRAMES: usize>
{
    ring: FrameRing<FRAMES>,
    prefill: usize,
    forwarding: bool,
    /// The layout of the decoded frames a push takes, from the last `open`.
    channels: Channels,
    largest_chunk: usize,
    burst: usize,
    waited: u32,
    /// The drift controller of a started ring, `None` while the ring waits for
    /// its start.
    drift: Option<DriftControl>,
    segment: Option<Segment>,
    history: [[u8; PCM_FRAME_BYTES]; SINC_LOOKAHEAD],
    /// Frames a flush left to play out before the ring waits for its start.
    tail: usize,
    /// The output gain, and the delivery measure that sets the start.
    fade: Fade,
    delivery: Delivery,
    stats: StatsAccumulator,
}

/// Output gain of a ring that corrects, and the spliced frames it fades around.
#[derive(Debug, Clone, Copy)]
struct Fade
{
    /// Gain in `FADE_FRAMES`ths, moved one step per output frame.
    gain: u32,
    /// Regions of stream positions played silent around a hard drop, oldest
    /// first: the first frame after the drop and the last one merged into it.
    splices: [(u64, u64); SPLICE_SLOTS],
    count: usize,
    /// Frames the ring has taken in and given out, as stream positions.
    accepted: u64,
    taken: u64,
}

impl Fade
{
    /// Builds a fade at silence with no splice.
    const fn new() -> Self
    {
        Self
        {
            gain: 0,
            splices: [(0, 0); SPLICE_SLOTS],
            count: 0,
            accepted: 0,
            taken: 0,
        }
    }

    /// Records a hard drop before stream position `position`.
    fn splice(&mut self, position: u64)
    {
        let merge = (FADE_FRAMES as u64).saturating_mul(2);

        if let Some(last) = self.count.checked_sub(1).and_then(|last| self.splices.get_mut(last))
            && (position <= last.1.saturating_add(merge) || self.count >= SPLICE_SLOTS)
        {
            last.1 = last.1.max(position);
            return;
        }

        if let Some(slot) = self.splices.get_mut(self.count)
        {
            *slot = (position, position);
            self.count = self.count.saturating_add(1);
        }
    }

    /// Forgets the splices from stream position `end` on, and ends a region
    /// that reaches past it on the frame before, so the frame at `end` plays
    /// clear of it.
    fn forget_from(&mut self, end: u64)
    {
        while self.count > 0 && self.splices.get(self.count.saturating_sub(1)).is_some_and(|&(first, _)| first >= end)
        {
            self.count = self.count.saturating_sub(1);
        }

        for region in self.splices.iter_mut().take(self.count)
        {
            region.1 = region.1.min(end.saturating_sub(1));
        }
    }

    /// Returns the gain for the frame at stream position `position`, with
    /// `remaining` frames left in the ring after it.
    ///
    /// The gain rises one step per frame at the most and follows the target
    /// down: the frames left, so the last frame before a dry ring is silent,
    /// and the distance to a splice region, so the frames on either side of it
    /// are. A region is forgotten once played, since the gain after its
    /// silent frames rises no faster than the distance to it.
    fn next_gain(&mut self, position: u64, remaining: usize) -> u32
    {
        // Past every fade, with no splice ahead, the gain stays full.
        if self.count == 0 && remaining >= FADE_FRAMES && self.gain >= 1 << FADE_SHIFT
        {
            return self.gain;
        }

        let full = FADE_FRAMES as u64;
        let mut target = u64::try_from(remaining).unwrap_or(u64::MAX).min(full);

        for &(first, last) in self.splices.iter().take(self.count)
        {
            let distance = if position < first
            {
                first.saturating_sub(1).saturating_sub(position)
            }
            else
            {
                position.saturating_sub(last)
            };
            target = target.min(distance);
        }

        while self.count > 0 && self.splices.first().is_some_and(|&(_, last)| position >= last)
        {
            self.splices.copy_within(1.., 0);
            self.count = self.count.saturating_sub(1);
        }

        let target = u32::try_from(target).unwrap_or(0);
        self.gain = self.gain.saturating_add(1).min(target);
        self.gain
    }
}

/// Measure of how a connected source delivers, which sets its start level.
#[derive(Debug, Clone, Copy)]
struct Delivery
{
    /// Drains since the last push, counted from a start until a flush, or until
    /// the ring runs dry `CLOSE_DRAINS` drains or more after its start. A gap
    /// that far from a start is a stall, which the next start need not answer.
    gap: u32,
    counting: bool,
    /// Longest run of drains without a push, at most the drains a full ring
    /// holds, and drains since it last decayed.
    longest: u32,
    longest_decay: u32,
    /// Drains played since the start.
    played: u32,
}

impl Delivery
{
    /// Builds the measure of a source not yet heard.
    const fn new() -> Self
    {
        Self
        {
            gap: 0,
            counting: false,
            longest: 0,
            longest_decay: 0,
            played: 0,
        }
    }
}

impl<const FRAMES: usize> Bridge<FRAMES>
{
    /// Builds a bridge with its gate closed and its ring empty.
    ///
    /// The ring gives out frames once it holds `prefill_frames` plus the start
    /// offset of its chunks, and the drift correction holds its mean fill
    /// around `prefill_frames`. A prefill above `FRAMES` stands at `FRAMES`.
    #[must_use]
    pub const fn new(prefill_frames: usize) -> Self
    {
        let prefill = if prefill_frames > FRAMES
        {
            FRAMES
        }
        else
        {
            prefill_frames
        };

        Self
        {
            ring: FrameRing::new(),
            prefill,
            forwarding: false,
            channels: Channels::Stereo,
            largest_chunk: 0,
            burst: 0,
            waited: 0,
            drift: None,
            segment: None,
            history: [[0; PCM_FRAME_BYTES]; SINC_LOOKAHEAD],
            tail: 0,
            fade: Fade::new(),
            delivery: Delivery::new(),
            stats: StatsAccumulator::new(),
        }
    }

    /// Starts a stream negotiated on `codec`, empties the ring and forgets how
    /// the previous source delivered.
    ///
    /// Returns the verdict of `stream_verdict` on `codec`. The gate opens on
    /// `Forward` and closes on every other verdict, whatever the previous
    /// stream was, and the pushes that follow take the channels `Forward`
    /// names.
    pub fn open(&mut self, codec: StreamCodec) -> StreamVerdict
    {
        self.flush();
        self.largest_chunk = 0;
        self.delivery = Delivery::new();
        let verdict = stream_verdict(codec);

        if let StreamVerdict::Forward(channels) = verdict
        {
            self.forwarding = true;
            self.channels = channels;
        }
        else
        {
            self.forwarding = false;
        }

        verdict
    }

    /// Closes the gate and empties the ring.
    ///
    /// The next stream opens with `open`, which forgets how this one delivered.
    pub fn close(&mut self)
    {
        self.flush();
        self.forwarding = false;
    }

    /// Empties the ring and waits for a new start.
    ///
    /// A ring that corrects and plays keeps its oldest frames, as many as its
    /// gain has steps, and the drains play them out, fading to silence, before
    /// anything pushed after the flush. The gate, the largest chunk and the
    /// longest delivery gap stay as they are: a suspend changes neither the
    /// source nor the link. The time of the suspend does not count as a
    /// delivery gap.
    pub fn flush(&mut self)
    {
        // The gain is 0 while the ring waits, and within the frames left while
        // it plays or plays out a tail.
        let tail = if Self::corrects()
        {
            self.ring.len.min(usize::try_from(self.fade.gain).unwrap_or(0))
        }
        else
        {
            0
        };

        self.ring.truncate(tail);
        self.tail = tail;
        self.drift = None;
        self.segment = None;
        self.history = [[0; PCM_FRAME_BYTES]; SINC_LOOKAHEAD];
        self.burst = 0;
        self.waited = 0;
        // The next frame pushed follows the tail.
        let end = self.fade.taken.saturating_add(u64::try_from(tail).unwrap_or(u64::MAX));
        self.fade.accepted = end;
        self.fade.forget_from(end);

        if tail == 0
        {
            self.fade.gain = 0;
        }

        self.delivery.gap = 0;
        self.delivery.counting = false;
        self.delivery.played = 0;
    }

    /// Pushes a chunk of decoded PCM.
    ///
    /// A frame of `pcm` is the layout the last `open` named: one sample on a
    /// mono stream, which goes into both channels of the frame the ring
    /// stores, and two on a stereo one. The ring takes the leading whole frames
    /// of `pcm` that fit and drops the rest of the whole frames. A ring that
    /// corrects and has not started makes room instead by dropping its oldest
    /// frames, counted in `BridgeStats::trimmed`. A closed gate drops every
    /// frame. A trailing partial frame is discarded in every case. A push that
    /// drops frames from a started ring that corrects marks the place for the
    /// output to fade around, and every push measures the gap since the one
    /// before it.
    pub fn push(&mut self, pcm: &[u8]) -> Pushed
    {
        match self.channels
        {
            Channels::Mono =>
            {
                let (samples, partial) = pcm.as_chunks::<MONO_FRAME_BYTES>();
                let frames = samples.iter().map(|&[low, high]| [low, high, low, high]);
                self.push_frames(frames, samples.len(), partial.len())
            }
            Channels::Stereo =>
            {
                let (frames, partial) = pcm.as_chunks::<PCM_FRAME_BYTES>();
                self.push_frames(frames.iter().copied(), frames.len(), partial.len())
            }
        }
    }

    /// Pushes `count` stored frames, a chunk trailing `trailing` bytes of a
    /// partial frame. `push` is the whole of its contract.
    fn push_frames<I>(&mut self, frames: I, count: usize, trailing: usize) -> Pushed
    where
        I: Iterator<Item = [u8; PCM_FRAME_BYTES]>,
    {
        let mut pushed = Pushed
        {
            trailing_bytes: trailing,
            ..Pushed::default()
        };
        self.largest_chunk = self.largest_chunk.max(count);
        let evicts = self.drift.is_none() && Self::corrects();

        for frame in frames
        {
            if !self.forwarding
            {
                pushed.dropped = pushed.dropped.saturating_add(1);
            }
            else if self.ring.push(frame)
            {
                pushed.accepted = pushed.accepted.saturating_add(1);
                self.fade.accepted = self.fade.accepted.saturating_add(1);
            }
            else if evicts && self.ring.pop().is_some() && self.ring.push(frame)
            {
                pushed.accepted = pushed.accepted.saturating_add(1);
                self.fade.taken = self.fade.taken.saturating_add(1);
                self.fade.accepted = self.fade.accepted.saturating_add(1);
                self.stats.trimmed = self.stats.trimmed.saturating_add(1);
            }
            else
            {
                if pushed.dropped == 0 && Self::corrects()
                {
                    self.fade.splice(self.fade.accepted);
                }

                pushed.dropped = pushed.dropped.saturating_add(1);
            }
        }

        if self.forwarding && count > 0 && self.delivery.counting
        {
            let delivery = &mut self.delivery;
            let holdable = u32::try_from(FRAMES.checked_div(BLOCK_FRAMES).unwrap_or(0)).unwrap_or(u32::MAX);
            delivery.longest = delivery.longest.max(delivery.gap.min(holdable));
            delivery.gap = 0;
        }

        if self.forwarding
        {
            self.burst = self.burst.saturating_add(count);
            let dropped = u32::try_from(pushed.dropped).unwrap_or(u32::MAX);
            self.stats.dropped = self.stats.dropped.saturating_add(dropped);
        }

        pushed
    }

    /// Returns whether the ring is large enough for the drift correction and
    /// the start rule, more than `2 * DEAD_BAND_FRAMES` frames.
    const fn corrects() -> bool
    {
        FRAMES > DEAD_BAND_FRAMES.saturating_mul(2)
    }

    /// Returns the longest delivery gap measured, in frames.
    ///
    /// The fill falls a block per drain of a gap, so this is also the depth of
    /// the deepest fill dip the source has caused while the ring played.
    fn longest_gap(&self) -> usize
    {
        usize::try_from(self.delivery.longest).unwrap_or(usize::MAX).saturating_mul(BLOCK_FRAMES)
    }

    /// Returns the level a start waits for.
    ///
    /// The base is the prefill plus half of the largest chunk in excess of a
    /// block. A delivery gap measured raises it to the longest one plus a
    /// block, and a level over the base stays `START_MARGIN_FRAMES` under the
    /// capacity of the ring. A ring that does not correct starts at the
    /// prefill.
    ///
    /// A start follows a push, so a gap of `g` drains from that push takes the
    /// fill down by `g` blocks before the next one. The block over it allows a
    /// gap one drain longer. The level stays as low as that because a source
    /// that lands a chunk early after its start adds to it, and a source whose
    /// fill swings by its longest gap plus that lead fits a ring of 4096
    /// frames only when the level holds the gap and no more.
    fn start_level(&self) -> usize
    {
        if !Self::corrects()
        {
            return self.prefill;
        }

        let offset = self
            .largest_chunk
            .min(LARGEST_CHUNK_FRAMES)
            .saturating_sub(BLOCK_FRAMES)
            .checked_div(2)
            .unwrap_or(0);
        let base = self.prefill.saturating_add(offset).min(FRAMES);
        let gap = match self.longest_gap()
        {
            0 => 0,
            frames => frames.saturating_add(BLOCK_FRAMES),
        };
        let highest = FRAMES.saturating_sub(START_MARGIN_FRAMES).max(base);
        base.max(gap).min(highest)
    }

    /// Returns the largest burst, in frames pushed between two drains, a start
    /// takes for a lone chunk: the largest chunk, and two blocks at the least.
    fn lone_burst(&self) -> usize
    {
        self.largest_chunk
            .min(LARGEST_CHUNK_FRAMES)
            .max(BLOCK_FRAMES.saturating_mul(2))
    }

    /// Decides whether the drain that follows a push burst of `burst` frames
    /// starts the ring.
    ///
    /// A ring under its start level waits. A ring that corrects drops its
    /// oldest frames down to the start level, and to one frame at the least,
    /// then starts on the drain after a lone chunk, a burst of at least one
    /// frame and at most `lone_burst`, or on the drain after any push once it
    /// has waited `START_WAIT_DRAINS` drains at its level. The start seeds the
    /// drift estimate at the prefill.
    fn start(&mut self, burst: usize) -> bool
    {
        let level = self.start_level();

        if self.ring.len == 0 || self.ring.len < level
        {
            return false;
        }

        if Self::corrects()
        {
            // A prefill of zero still starts on the frame the ring holds.
            let keep = level.max(1);

            while self.ring.len > keep && self.take_frame().is_some()
            {
                self.stats.trimmed = self.stats.trimmed.saturating_add(1);
            }

            let lone = burst > 0 && burst <= self.lone_burst();

            if !lone && self.waited < START_WAIT_DRAINS
            {
                self.waited = self.waited.saturating_add(1);
                return false;
            }

            if burst == 0
            {
                return false;
            }
        }

        self.waited = 0;
        self.drift = Some(DriftControl::seeded(self.prefill));
        self.delivery.played = 0;
        self.delivery.counting = true;
        self.delivery.gap = 0;
        true
    }

    /// Drains whole frames into `pcm`, oldest first, correcting the drift.
    ///
    /// Returns the bytes written, a whole number of frames, from the start of
    /// `pcm`. Bytes past them are left untouched. Returns 0 while the ring
    /// waits for its start. A drain that empties the ring before it fills
    /// `pcm` makes the ring wait for a new start.
    ///
    /// Every drain of a started ring samples the fill. When the drift estimate
    /// asks for a correction and none is running, the drain starts one that
    /// crossfades `CROSSFADE_FRAMES` frames of the ring into one frame fewer or
    /// one more, across as many drains as it takes. A ring that corrects fades
    /// its output in after a start, out before it runs dry, and around frames a
    /// push dropped. The tail a flush left plays first, and a drain that ends
    /// it returns short.
    pub fn drain_into(&mut self, pcm: &mut [u8]) -> usize
    {
        let burst = core::mem::take(&mut self.burst);

        if self.tail > 0
        {
            return self.drain_tail(pcm);
        }

        if self.delivery.counting
        {
            self.delivery.gap = self.delivery.gap.saturating_add(1);
        }

        if self.drift.is_none() && !self.start(burst)
        {
            return 0;
        }

        let fill = self.ring.len;
        self.stats.sample(fill);

        let centre = FRAMES.checked_div(2).unwrap_or(0);
        let idle = self.segment.is_none();

        if let Some(correction) = self.drift.as_mut().and_then(|drift| drift.update(fill, centre, idle))
        {
            self.segment = Some(Segment
            {
                correction,
                index: 0,
            });
        }

        let (outs, _) = pcm.as_chunks_mut::<PCM_FRAME_BYTES>();
        let mut frames: usize = 0;

        for out in outs
        {
            let Some(frame) = self.next_frame()
            else
            {
                self.drift = None;
                self.segment = None;
                self.stats.underruns = self.stats.underruns.saturating_add(1);

                if self.delivery.played >= CLOSE_DRAINS
                {
                    self.delivery.counting = false;
                }

                break;
            };

            *out = if Self::corrects()
            {
                let position = self.fade.taken.saturating_sub(1);
                faded(frame, self.fade.next_gain(position, self.ring.len))
            }
            else
            {
                frame
            };
            frames = frames.saturating_add(1);
        }

        if self.drift.is_some()
        {
            self.age_delivery();
        }

        frames.saturating_mul(PCM_FRAME_BYTES)
    }

    /// Plays out the tail a flush left into `pcm`, fading it to silence, and
    /// returns the bytes written. The gain follows the frames left in the tail,
    /// so it ends at 0.
    fn drain_tail(&mut self, pcm: &mut [u8]) -> usize
    {
        let (outs, _) = pcm.as_chunks_mut::<PCM_FRAME_BYTES>();
        let mut frames: usize = 0;

        for out in outs
        {
            if self.tail == 0
            {
                break;
            }

            let Some(frame) = self.take_frame()
            else
            {
                self.tail = 0;
                break;
            };

            self.tail = self.tail.saturating_sub(1);
            let position = self.fade.taken.saturating_sub(1);
            *out = faded(frame, self.fade.next_gain(position, self.tail));
            frames = frames.saturating_add(1);
        }

        frames.saturating_mul(PCM_FRAME_BYTES)
    }

    /// Counts one drain of play, and takes a drain off the longest gap once
    /// every `GAP_DECAY_DRAINS` drains.
    fn age_delivery(&mut self)
    {
        let delivery = &mut self.delivery;
        delivery.played = delivery.played.saturating_add(1);
        delivery.longest_decay = delivery.longest_decay.saturating_add(1);

        if delivery.longest_decay >= GAP_DECAY_DRAINS
        {
            delivery.longest_decay = 0;
            delivery.longest = delivery.longest.saturating_sub(1);
        }
    }

    /// Removes the oldest frame from the ring and keeps it as the newest of the
    /// frames the resampler reads behind its position.
    fn take_frame(&mut self) -> Option<[u8; PCM_FRAME_BYTES]>
    {
        let frame = self.ring.pop()?;
        self.fade.taken = self.fade.taken.saturating_add(1);
        let [newest, second, third, _] = self.history;
        self.history = [frame, newest, second, third];
        Some(frame)
    }

    /// Returns the next output frame, or `None` when the ring runs dry.
    ///
    /// Without a correction running it takes the oldest frame. With `L` the
    /// `CROSSFADE_FRAMES` and frames numbered from the first one the correction
    /// takes, `Remove` gives `L - 1` outputs at positions `k + (k + 1) / L`,
    /// then takes frame `L - 1` unplayed and counts, and `Insert` gives `L + 1`
    /// outputs at positions `k - k / L` and counts on its second output. Either
    /// way the frame after the correction follows on without a jump. A ring
    /// holding `SINC_LOOKAHEAD` frames or fewer cannot read ahead, so a running
    /// correction gives out plain frames there until the ring runs dry or refills.
    fn next_frame(&mut self) -> Option<[u8; PCM_FRAME_BYTES]>
    {
        let Some(segment) = self.segment
        else
        {
            return self.take_frame();
        };

        if self.ring.len <= SINC_LOOKAHEAD
        {
            return self.take_frame();
        }

        let index = segment.index;
        let next = Some(Segment
        {
            index: index.saturating_add(1),
            ..segment
        });

        match segment.correction
        {
            Correction::Remove =>
            {
                let _ = self.take_frame();
                let frame = self.resample(index.saturating_add(1));

                if index.saturating_add(2) >= CROSSFADE_FRAMES
                {
                    let _ = self.take_frame();
                    self.stats.removed = self.stats.removed.saturating_add(1);
                    self.segment = None;
                }
                else
                {
                    self.segment = next;
                }

                frame
            }
            Correction::Insert =>
            {
                if index != 1
                {
                    let _ = self.take_frame();
                }

                let position = if index == 0 { 0 } else { CROSSFADE_FRAMES.saturating_sub(index) };
                let frame = self.resample(position);

                if index == 1
                {
                    self.stats.inserted = self.stats.inserted.saturating_add(1);
                }

                self.segment = if index >= CROSSFADE_FRAMES { None } else { next };
                frame
            }
        }
    }

    /// Returns the frame at `position / L` past the newest frame taken, `L` the
    /// `CROSSFADE_FRAMES`, resampled from the three frames before it, itself and
    /// the `SINC_LOOKAHEAD` frames after it, or `None` when the ring holds
    /// fewer than those.
    ///
    /// Position 0 returns the frame itself. The weights come from the two table
    /// rows around the position, interpolated, the same for both channels, and
    /// each sample saturates at the 16-bit range.
    fn resample(&self, position: usize) -> Option<[u8; PCM_FRAME_BYTES]>
    {
        let [current, back1, back2, back3] = self.history;

        if position == 0
        {
            return Some(current);
        }

        let taps =
        [
            back3,
            back2,
            back1,
            current,
            self.ring.peek(0)?,
            self.ring.peek(1)?,
            self.ring.peek(2)?,
            self.ring.peek(3)?,
        ];
        let phase = position >> SINC_PHASE_SHIFT;
        let fraction = i32::try_from(position & ((1 << SINC_PHASE_SHIFT) - 1)).unwrap_or(0);
        let low = SINC_ROWS.get(phase)?;
        let high = SINC_ROWS.get(phase.saturating_add(1))?;

        let mut left = SINC_ROUND;
        let mut right = SINC_ROUND;

        for ((frame, &from), &to) in taps.iter().zip(low).zip(high)
        {
            let from = i32::from(from);
            let spread = i32::from(to).saturating_sub(from).saturating_mul(fraction).saturating_add(SINC_PHASE_ROUND);
            let weight = from.saturating_add(spread >> SINC_PHASE_SHIFT);
            let [l0, l1, r0, r1] = *frame;
            left = left.saturating_add(i32::from(i16::from_le_bytes([l0, l1])).saturating_mul(weight));
            right = right.saturating_add(i32::from(i16::from_le_bytes([r0, r1])).saturating_mul(weight));
        }

        let [l0, l1] = saturate(left >> SINC_FRACTION_BITS);
        let [r0, r1] = saturate(right >> SINC_FRACTION_BITS);
        Some([l0, l1, r0, r1])
    }

    /// Returns the statistics since the previous call and starts a new period.
    pub fn take_stats(&mut self) -> BridgeStats
    {
        let stats = self.stats.snapshot();
        self.stats = StatsAccumulator::new();
        stats
    }
}

/// Source of whole PCM frames for the pump.
pub trait FrameSource
{
    /// Writes whole frames from the start of `pcm` and returns the bytes written.
    fn take_frames(&mut self, pcm: &mut [u8]) -> usize;
}

impl<const FRAMES: usize> FrameSource for Bridge<FRAMES>
{
    fn take_frames(&mut self, pcm: &mut [u8]) -> usize
    {
        self.drain_into(pcm)
    }
}

/// Transmit side of the I2S link.
pub trait SlotWriter
{
    /// Error the channel reports.
    type Error;

    /// Writes a prefix of `bytes` and returns its length.
    ///
    /// Returns 0 when the channel took nothing before its timeout.
    ///
    /// # Errors
    ///
    /// Whatever the channel reports.
    fn write(&mut self, bytes: &[u8]) -> Result<usize, Self::Error>;
}

/// Failure of one pump.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PumpError<E>
{
    /// The scratch buffers cannot carry a block. Nothing was taken.
    Layout(BridgeError),
    /// The channel failed. The block is lost from the byte the channel stopped at.
    Write(E),
}

/// Moves one block from `source` to `writer`.
///
/// Takes as many whole frames as `pcm` holds from `source`, pads the rest of
/// `pcm` with silence, widens the block into `slots` and writes all of it,
/// calling the writer again while it takes nothing. A closed gate, an empty
/// ring or a ring waiting for its prefill therefore writes a block of silence.
///
/// Returns the frames taken from `source`.
///
/// # Errors
///
/// `Layout` when `pcm` is not a whole number of frames or `slots` is shorter
/// than twice `pcm`, before `source` is touched. `Write` when the writer fails.
pub fn pump<S, W>
(
    source: &mut S,
    pcm: &mut [u8],
    slots: &mut [u8],
    writer: &mut W
) -> Result<usize, PumpError<W::Error>>
where
    S: FrameSource,
    W: SlotWriter,
{
    if !pcm.as_chunks::<PCM_FRAME_BYTES>().1.is_empty()
    {
        return Err(PumpError::Layout(BridgeError::PartialFrame));
    }

    if pcm.len().checked_mul(WIDENING).is_none_or(|needed| slots.len() < needed)
    {
        return Err(PumpError::Layout(BridgeError::OutputTooShort));
    }

    let reported = source.take_frames(pcm).min(pcm.len());
    let taken = reported.saturating_sub(reported.checked_rem(PCM_FRAME_BYTES).unwrap_or(0));

    if let Some(rest) = pcm.get_mut(taken..)
    {
        rest.fill(0);
    }

    let written = widen_into(pcm, slots).map_err(PumpError::Layout)?;
    let mut done: usize = 0;

    while let Some(rest) = slots.get(done..written)
    {
        if rest.is_empty()
        {
            break;
        }

        let accepted = writer.write(rest).map_err(PumpError::Write)?;
        done = done.saturating_add(accepted.min(rest.len()));
    }

    Ok(taken.checked_div(PCM_FRAME_BYTES).unwrap_or(0))
}

#[cfg(test)]
mod tests
{
    // A test reports a broken invariant by failing, which is the one place the
    // no-panic rule does not hold.
    #![allow
    (
        clippy::panic,
        clippy::indexing_slicing,
        clippy::arithmetic_side_effects,
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss,
        clippy::cast_possible_wrap,
        clippy::cast_precision_loss,
        clippy::chunks_exact_to_as_chunks
    )]

    use super::*;
    use core::iter::repeat_n;
    use std::vec::Vec;

    /// Ring capacity of the fixture, small so a run crosses every fill level.
    const FIXTURE_FRAMES: usize = 37;

    /// Prefill of the fixture.
    const FIXTURE_PREFILL: usize = 11;

    /// Byte a malformed chunk trails with. A reader that loses the frame
    /// boundary reads it as part of a sample.
    const JUNK: u8 = 0xEE;

    /// Builds frame `index`: the left sample carries the index, the right one
    /// its complement, so no frame is silence and a shifted read shows.
    fn frame(index: u16) -> [u8; PCM_FRAME_BYTES]
    {
        let [l0, l1] = index.to_le_bytes();
        let [r0, r1] = (!index).to_le_bytes();
        [l0, l1, r0, r1]
    }

    /// Builds the bytes of frames `first..first + count`.
    fn frames(first: u16, count: usize) -> Vec<u8>
    {
        (0..count)
            .flat_map(|offset| frame(first.wrapping_add(offset as u16)))
            .collect()
    }

    /// Returns the index a PCM frame carries, failing on a misaligned frame.
    fn index_of(bytes: &[u8]) -> u16
    {
        let left = u16::from_le_bytes([bytes[0], bytes[1]]);
        let right = u16::from_le_bytes([bytes[2], bytes[3]]);
        assert_eq!(right, !left, "frame {bytes:02x?} lost its alignment");
        left
    }

    /// Returns the two slot words of a link frame.
    fn slots_of(bytes: &[u8]) -> (i32, i32)
    {
        (
            i32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]),
            i32::from_le_bytes([bytes[4], bytes[5], bytes[6], bytes[7]]),
        )
    }

    /// Deterministic pseudo random sizes.
    struct Lcg(u64);

    impl Lcg
    {
        /// Returns a value in `0..bound`.
        fn below(&mut self, bound: usize) -> usize
        {
            self.0 = self
                .0
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            ((self.0 >> 33) as usize) % bound
        }
    }

    /// SBC at 44.1 kHz in joint stereo, the stream phones negotiate most.
    const JOINT_STEREO_44_1: StreamCodec = StreamCodec::Sbc(0x21);

    /// SBC at 44.1 kHz in mono.
    const MONO_44_1: StreamCodec = StreamCodec::Sbc(0x28);

    /// The verdict on a stream of two channels the bridge forwards.
    const STEREO: StreamVerdict = StreamVerdict::Forward(Channels::Stereo);

    /// The verdict on a mono stream the bridge forwards.
    const MONO: StreamVerdict = StreamVerdict::Forward(Channels::Mono);

    /// Builds an open fixture bridge with `prefill` frames of prefill.
    fn open_bridge(prefill: usize) -> Bridge<FIXTURE_FRAMES>
    {
        let mut bridge = Bridge::new(prefill);
        assert_eq!(bridge.open(JOINT_STEREO_44_1), STEREO);
        bridge
    }

    /// Asserts that `bridge` drops every pushed frame and pumps silence.
    fn assert_silent(bridge: &mut Bridge<FIXTURE_FRAMES>, context: &str)
    {
        assert_eq!
        (
            bridge.push(&frames(1, 3)),
            Pushed { accepted: 0, dropped: 3, trailing_bytes: 0 },
            "{context}"
        );

        let mut pcm = [0x33; 3 * PCM_FRAME_BYTES];
        let mut slots = [0x5A; 3 * SLOT_FRAME_BYTES];
        let mut link = ChokedLink::new();
        assert_eq!(pump(bridge, &mut pcm, &mut slots, &mut link), Ok(0), "{context}");
        assert_eq!(link.bytes, [0; 3 * SLOT_FRAME_BYTES], "{context}");
    }

    /// Link that takes a varying prefix, and nothing on every third call.
    struct ChokedLink
    {
        bytes: Vec<u8>,
        calls: usize,
        fail_on_call: Option<usize>,
    }

    impl ChokedLink
    {
        fn new() -> Self
        {
            Self
            {
                bytes: Vec::new(),
                calls: 0,
                fail_on_call: None,
            }
        }
    }

    impl SlotWriter for ChokedLink
    {
        type Error = u8;

        fn write(&mut self, bytes: &[u8]) -> Result<usize, u8>
        {
            self.calls += 1;

            if self.fail_on_call == Some(self.calls)
            {
                return Err(7);
            }

            if self.calls.is_multiple_of(3)
            {
                return Ok(0);
            }

            let taken = bytes.len().min(1 + self.calls % 13);
            self.bytes.extend_from_slice(&bytes[..taken]);
            Ok(taken)
        }
    }

    /// Widens one frame.
    fn widen_frame(left: i16, right: i16) -> [u8; SLOT_FRAME_BYTES]
    {
        let [l0, l1] = left.to_le_bytes();
        let [r0, r1] = right.to_le_bytes();
        let mut slots = [0x5A; SLOT_FRAME_BYTES];
        assert_eq!(widen_into(&[l0, l1, r0, r1], &mut slots), Ok(SLOT_FRAME_BYTES));
        slots
    }

    #[test]
    fn every_sample_lands_in_the_upper_half_of_its_slot()
    {
        for sample in i16::MIN..=i16::MAX
        {
            let (left, right) = slots_of(&widen_frame(sample, sample.wrapping_neg()));
            assert_eq!(left, i32::from(sample) * 65_536, "left slot of {sample}");
            assert_eq!(right, i32::from(sample.wrapping_neg()) * 65_536, "right slot of {sample}");
            assert_eq!(left & 0xFFFF, 0);
            assert_eq!(right & 0xFFFF, 0);
        }
    }

    #[test]
    fn full_scale_zero_and_sign_widen_to_their_exact_bytes()
    {
        assert_eq!(widen_frame(i16::MAX, i16::MIN), [0, 0, 0xFF, 0x7F, 0, 0, 0x00, 0x80]);
        assert_eq!(widen_frame(0, -1), [0, 0, 0, 0, 0, 0, 0xFF, 0xFF]);
        assert_eq!(widen_frame(1, -2), [0, 0, 0x01, 0x00, 0, 0, 0xFE, 0xFF]);
    }

    #[test]
    fn the_output_is_little_endian_with_the_left_sample_first()
    {
        let mut slots = [0; 2 * SLOT_FRAME_BYTES];
        let pcm = [0x34, 0x12, 0x78, 0x56, 0xBC, 0x9A, 0xF0, 0xDE];
        assert_eq!(widen_into(&pcm, &mut slots), Ok(16));
        assert_eq!
        (
            slots,
            [
                0, 0, 0x34, 0x12, 0, 0, 0x78, 0x56,
                0, 0, 0xBC, 0x9A, 0, 0, 0xF0, 0xDE,
            ]
        );
    }

    #[test]
    fn widening_refuses_a_partial_frame_or_a_short_output_without_writing()
    {
        for len in [1, 2, 3, 5, 6, 7]
        {
            let mut slots = [0x5A; 16];
            assert_eq!(widen_into(&[1; 7][..len], &mut slots), Err(BridgeError::PartialFrame));
            assert_eq!(slots, [0x5A; 16]);
        }

        let mut short = [0x5A; 15];
        assert_eq!(widen_into(&[1; 8], &mut short), Err(BridgeError::OutputTooShort));
        assert_eq!(short, [0x5A; 15]);

        let mut long = [0x5A; 20];
        assert_eq!(widen_into(&[0; 8], &mut long), Ok(16));
        assert_eq!(long[..16], [0; 16]);
        assert_eq!(long[16..], [0x5A; 4]);

        assert_eq!(widen_into(&[], &mut []), Ok(0));
    }

    #[test]
    fn chained_pushes_and_drains_keep_every_accepted_frame_in_order()
    {
        let mut bridge = open_bridge(FIXTURE_PREFILL);
        let mut rng = Lcg(0x5EED);
        let mut next_index: u16 = 0;
        let mut expected: Vec<u16> = Vec::new();
        let mut output: Vec<u16> = Vec::new();
        let mut level = 0;
        let mut dropped = 0;
        let mut ragged_drains = 0;
        let mut drain = [0; 64 * PCM_FRAME_BYTES + 3];

        for _ in 0..20_000
        {
            let count = rng.below(FIXTURE_FRAMES + 8);
            let trailing = rng.below(PCM_FRAME_BYTES);
            let mut chunk = frames(next_index, count);
            chunk.extend(repeat_n(JUNK, trailing));

            let pushed = bridge.push(&chunk);
            let room = FIXTURE_FRAMES - level;
            assert_eq!
            (
                pushed,
                Pushed
                {
                    accepted: count.min(room),
                    dropped: count.saturating_sub(room),
                    trailing_bytes: trailing,
                }
            );
            expected.extend((0..pushed.accepted).map(|offset| next_index.wrapping_add(offset as u16)));
            next_index = next_index.wrapping_add(count as u16);
            level += pushed.accepted;
            dropped += pushed.dropped;

            let len = rng.below(drain.len() + 1);
            drain.fill(JUNK);
            let written = bridge.drain_into(&mut drain[..len]);
            assert_eq!(written % PCM_FRAME_BYTES, 0);
            assert!(written <= len);
            assert!(drain[written..len].iter().all(|&byte| byte == JUNK));
            ragged_drains += usize::from(!len.is_multiple_of(PCM_FRAME_BYTES) && written > 0);
            output.extend(drain[..written].chunks_exact(PCM_FRAME_BYTES).map(index_of));
            level -= written / PCM_FRAME_BYTES;
        }

        assert_eq!(output[..], expected[..output.len()]);
        assert_eq!(expected.len() - output.len(), level);
        assert!(dropped > 1_000, "the run dropped {dropped} frames");
        assert!(output.len() > 100_000, "the run drained {} frames", output.len());
        assert!(ragged_drains > 1_000, "the run drained {ragged_drains} ragged buffers");
    }

    #[test]
    fn a_full_ring_drops_whole_incoming_frames_at_every_fill_level()
    {
        let mut out = [0; (FIXTURE_FRAMES + 1) * PCM_FRAME_BYTES];

        for level in 0..=FIXTURE_FRAMES
        {
            for incoming in 0..=FIXTURE_FRAMES + 2
            {
                for trailing in 0..PCM_FRAME_BYTES
                {
                    let mut bridge = open_bridge(0);

                    let rotation = (level * 7 + incoming + trailing) % FIXTURE_FRAMES;
                    assert_eq!(bridge.push(&frames(5_000, rotation)).accepted, rotation);
                    assert_eq!(bridge.drain_into(&mut out), rotation * PCM_FRAME_BYTES);

                    assert_eq!(bridge.push(&frames(0, level)).accepted, level);

                    let mut chunk = frames(1_000, incoming);
                    chunk.extend(repeat_n(JUNK, trailing));
                    let room = FIXTURE_FRAMES - level;
                    assert_eq!
                    (
                        bridge.push(&chunk),
                        Pushed
                        {
                            accepted: incoming.min(room),
                            dropped: incoming.saturating_sub(room),
                            trailing_bytes: trailing,
                        }
                    );

                    let written = bridge.drain_into(&mut out);
                    let got: Vec<u16> = out[..written].chunks_exact(PCM_FRAME_BYTES).map(index_of).collect();
                    let want: Vec<u16> = (0..level as u16)
                        .chain((0..incoming.min(room) as u16).map(|offset| 1_000 + offset))
                        .collect();
                    assert_eq!(got, want, "level {level}, incoming {incoming}");

                    assert_eq!(bridge.push(&frame(2_000)).accepted, 1);
                    let written = bridge.drain_into(&mut out);
                    assert_eq!(written, PCM_FRAME_BYTES);
                    assert_eq!(index_of(&out[..PCM_FRAME_BYTES]), 2_000);
                }
            }
        }
    }

    #[test]
    fn the_ring_waits_for_its_prefill_before_and_after_an_underrun()
    {
        let mut bridge = open_bridge(FIXTURE_PREFILL);
        let mut out = [0; 4 * PCM_FRAME_BYTES];

        for index in 0..FIXTURE_PREFILL - 1
        {
            assert_eq!(bridge.push(&frame(index as u16)).accepted, 1);
            assert_eq!(bridge.drain_into(&mut out), 0, "gave out at level {}", index + 1);
        }

        assert_eq!(bridge.push(&frame(10)).accepted, 1);
        assert_eq!(bridge.drain_into(&mut out), 16);
        assert_eq!(bridge.drain_into(&mut out), 16);
        assert_eq!(bridge.drain_into(&mut out), 12);
        assert_eq!(index_of(&out[8..12]), 10);

        assert_eq!(bridge.push(&frames(20, FIXTURE_PREFILL - 1)).accepted, FIXTURE_PREFILL - 1);
        assert_eq!(bridge.drain_into(&mut out), 0);
        assert_eq!(bridge.push(&frame(99)).accepted, 1);
        assert_eq!(bridge.drain_into(&mut out), 16);
        assert_eq!(index_of(&out[..4]), 20);
    }

    #[test]
    fn a_prefill_above_the_capacity_stands_at_the_capacity()
    {
        let mut bridge = Bridge::<4>::new(100);
        assert_eq!(bridge.open(JOINT_STEREO_44_1), STEREO);
        let mut out = [0; 4 * PCM_FRAME_BYTES];
        assert_eq!(bridge.push(&frames(0, 3)).accepted, 3);
        assert_eq!(bridge.drain_into(&mut out), 0);
        assert_eq!(bridge.push(&frames(3, 2)), Pushed { accepted: 1, dropped: 1, trailing_bytes: 0 });
        assert_eq!(bridge.drain_into(&mut out), 16);
    }

    #[test]
    fn a_drain_from_an_empty_ring_writes_nothing()
    {
        let mut bridge = open_bridge(0);
        let mut out = [0xA5; 12];
        assert_eq!(bridge.drain_into(&mut out), 0);
        assert_eq!(out, [0xA5; 12]);

        let mut closed = Bridge::<FIXTURE_FRAMES>::new(0);
        assert_eq!(closed.drain_into(&mut out), 0);
        assert_eq!(out, [0xA5; 12]);
    }

    #[test]
    fn a_ragged_drain_writes_whole_frames_and_leaves_the_tail()
    {
        let mut bridge = open_bridge(0);
        assert_eq!(bridge.push(&frames(0, 5)).accepted, 5);
        let mut out = [JUNK; 7];
        assert_eq!(bridge.drain_into(&mut out), 4);
        assert_eq!(index_of(&out[..4]), 0);
        assert_eq!(out[4..], [JUNK; 3]);
        let mut rest = [0; 16];
        assert_eq!(bridge.drain_into(&mut rest), 16);
        assert_eq!(rest.chunks_exact(4).map(index_of).collect::<Vec<_>>(), [1, 2, 3, 4]);
    }

    #[test]
    fn the_stream_verdict_reads_rate_and_channel_mode()
    {
        let cases =
        [
            (StreamCodec::Sbc(0x21), STEREO),
            (StreamCodec::Sbc(0x22), STEREO),
            (StreamCodec::Sbc(0x24), STEREO),
            (StreamCodec::Sbc(0x28), MONO),
            (StreamCodec::Sbc(0x11), StreamVerdict::WrongRate(48_000)),
            (StreamCodec::Sbc(0x18), StreamVerdict::WrongRate(48_000)),
            (StreamCodec::Sbc(0x42), StreamVerdict::WrongRate(32_000)),
            (StreamCodec::Sbc(0x84), StreamVerdict::WrongRate(16_000)),
            (StreamCodec::Sbc(0x20), StreamVerdict::Malformed),
            (StreamCodec::Sbc(0x01), StreamVerdict::Malformed),
            (StreamCodec::Sbc(0x00), StreamVerdict::Malformed),
            (StreamCodec::Sbc(0x23), StreamVerdict::Malformed),
            (StreamCodec::Sbc(0x29), StreamVerdict::Malformed),
            (StreamCodec::Sbc(0x31), StreamVerdict::Malformed),
            (StreamCodec::Sbc(0xFF), StreamVerdict::Malformed),
            (StreamCodec::Other, StreamVerdict::NotSbc),
        ];

        for (codec, verdict) in cases
        {
            assert_eq!(stream_verdict(codec), verdict, "{codec:?}");
        }

        let forwarded: Vec<u8> = (0..=u8::MAX)
            .filter(|&octet| stream_verdict(StreamCodec::Sbc(octet)) == STEREO)
            .collect();
        assert_eq!(forwarded, [0x21, 0x22, 0x24]);

        let mono: Vec<u8> = (0..=u8::MAX)
            .filter(|&octet| stream_verdict(StreamCodec::Sbc(octet)) == MONO)
            .collect();
        assert_eq!(mono, [0x28]);
    }

    #[test]
    fn only_a_forwarded_stream_opens_the_gate()
    {
        let mut pcm = [0; 3 * PCM_FRAME_BYTES];
        let mut slots = [0x5A; 3 * SLOT_FRAME_BYTES];

        for octet in [0x18, 0x11, 0x42, 0x84, 0x20, 0x23]
        {
            let mut bridge = Bridge::<FIXTURE_FRAMES>::new(0);
            assert!(!matches!(bridge.open(StreamCodec::Sbc(octet)), StreamVerdict::Forward(_)), "octet {octet:#04x}");
            assert_silent(&mut bridge, &std::format!("octet {octet:#04x}"));
        }

        let mut other = Bridge::<FIXTURE_FRAMES>::new(0);
        assert_eq!(other.open(StreamCodec::Other), StreamVerdict::NotSbc);
        assert_silent(&mut other, "codec other than SBC");

        let mut bridge = Bridge::<FIXTURE_FRAMES>::new(0);
        assert_eq!(bridge.open(JOINT_STEREO_44_1), STEREO);
        assert_eq!(bridge.push(&frames(1, 3)).accepted, 3);
        let mut link = ChokedLink::new();
        assert_eq!(pump(&mut bridge, &mut pcm, &mut slots, &mut link), Ok(3));
        let words: Vec<(i32, i32)> = link.bytes.chunks_exact(SLOT_FRAME_BYTES).map(slots_of).collect();
        assert_eq!
        (
            words,
            [
                (0x0001_0000, !0x0001_i32 << 16),
                (0x0002_0000, !0x0002_i32 << 16),
                (0x0003_0000, !0x0003_i32 << 16),
            ]
        );
    }

    #[test]
    fn a_renegotiation_closes_the_gate_and_discards_the_old_stream()
    {
        let mut out = [0; 8 * PCM_FRAME_BYTES];

        for refused in
        [
            StreamCodec::Sbc(0x11),
            StreamCodec::Sbc(0x18),
            StreamCodec::Sbc(0x20),
            StreamCodec::Other,
        ]
        {
            let mut bridge = open_bridge(0);
            assert_eq!(bridge.push(&frames(0, 5)).accepted, 5);
            assert!(!matches!(bridge.open(refused), StreamVerdict::Forward(_)), "{refused:?}");
            assert_eq!(bridge.drain_into(&mut out), 0, "{refused:?}");
            assert_silent(&mut bridge, &std::format!("renegotiated to {refused:?}"));

            assert_eq!(bridge.open(JOINT_STEREO_44_1), STEREO);
            assert_eq!(bridge.drain_into(&mut out), 0);
            assert_eq!(bridge.push(&frame(9)).accepted, 1);
            assert_eq!(bridge.drain_into(&mut out), 4);
            assert_eq!(index_of(&out[..4]), 9);
        }
    }

    #[test]
    fn flush_makes_the_ring_wait_for_its_prefill_again()
    {
        let mut bridge = open_bridge(FIXTURE_PREFILL);
        let mut out = [0; 4 * PCM_FRAME_BYTES];

        assert_eq!(bridge.push(&frames(0, FIXTURE_PREFILL)).accepted, FIXTURE_PREFILL);
        assert_eq!(bridge.drain_into(&mut out), 16);

        bridge.flush();
        assert_eq!(bridge.push(&frames(100, FIXTURE_PREFILL - 1)).accepted, FIXTURE_PREFILL - 1);
        assert_eq!(bridge.drain_into(&mut out), 0);
        assert_eq!(bridge.push(&frame(200)).accepted, 1);
        assert_eq!(bridge.drain_into(&mut out), 16);
        assert_eq!(index_of(&out[..4]), 100);
    }

    #[test]
    fn close_drops_everything_and_flush_keeps_the_gate()
    {
        let mut out = [0; 8 * PCM_FRAME_BYTES];

        let mut bridge = open_bridge(0);
        assert_eq!(bridge.push(&frames(0, 5)).accepted, 5);
        bridge.flush();
        assert_eq!(bridge.drain_into(&mut out), 0);
        assert_eq!(bridge.push(&frame(7)).accepted, 1);
        assert_eq!(bridge.drain_into(&mut out), 4);
        assert_eq!(index_of(&out[..4]), 7);

        assert_eq!(bridge.push(&frames(0, 5)).accepted, 5);
        bridge.close();
        assert_eq!(bridge.drain_into(&mut out), 0);
        assert_eq!(bridge.push(&frames(0, 2)), Pushed { accepted: 0, dropped: 2, trailing_bytes: 0 });
    }

    #[test]
    fn chained_pumps_keep_the_link_frame_aligned_through_partial_writes()
    {
        const BLOCK: usize = 6;
        const PUMPS: usize = 2_000;

        let mut bridge = open_bridge(FIXTURE_PREFILL);
        let mut rng = Lcg(0xB10C);
        let mut link = ChokedLink::new();
        let mut pcm = [0; BLOCK * PCM_FRAME_BYTES];
        let mut slots = [0; BLOCK * SLOT_FRAME_BYTES];
        let mut next_index: u16 = 0;
        let mut expected: Vec<u16> = Vec::new();
        let mut taken = 0;

        for _ in 0..PUMPS
        {
            let count = rng.below(BLOCK + 4);
            let mut chunk = frames(next_index, count);
            chunk.extend(repeat_n(JUNK, rng.below(PCM_FRAME_BYTES)));
            let pushed = bridge.push(&chunk);
            expected.extend((0..pushed.accepted).map(|offset| next_index.wrapping_add(offset as u16)));
            next_index = next_index.wrapping_add(count as u16);

            match pump(&mut bridge, &mut pcm, &mut slots, &mut link)
            {
                Ok(frames) => taken += frames,
                Err(error) => panic!("pump failed: {error:?}"),
            }
        }

        assert_eq!(link.bytes.len(), PUMPS * BLOCK * SLOT_FRAME_BYTES);

        let mut carried: Vec<u16> = Vec::new();
        let mut silent = 0;

        for bytes in link.bytes.chunks_exact(SLOT_FRAME_BYTES)
        {
            let (left, right) = slots_of(bytes);
            assert_eq!(left & 0xFFFF, 0, "link frame {bytes:02x?} lost its alignment");
            assert_eq!(right & 0xFFFF, 0, "link frame {bytes:02x?} lost its alignment");

            if left == 0 && right == 0
            {
                silent += 1;
                continue;
            }

            assert_eq!(right >> 16, !(left >> 16), "link frame {bytes:02x?} lost its alignment");
            carried.push((left >> 16) as u16);
        }

        assert_eq!(carried.len(), taken);
        assert_eq!(carried[..], expected[..carried.len()]);
        assert!(silent > BLOCK, "the run never underran");
        assert!(taken > PUMPS, "the run carried {taken} frames");
    }

    #[test]
    fn a_writer_error_stops_the_pump()
    {
        let mut bridge = open_bridge(0);
        let mut pcm = [0; 2 * PCM_FRAME_BYTES];
        let mut slots = [0; 2 * SLOT_FRAME_BYTES];
        let mut link = ChokedLink::new();
        link.fail_on_call = Some(2);
        assert_eq!(pump(&mut bridge, &mut pcm, &mut slots, &mut link), Err(PumpError::Write(7)));
    }

    #[test]
    fn the_pump_refuses_buffers_that_cannot_carry_a_block_before_taking_frames()
    {
        let mut bridge = open_bridge(0);
        assert_eq!(bridge.push(&frames(0, 2)).accepted, 2);
        let mut link = ChokedLink::new();

        let mut ragged = [0; 5];
        let mut slots = [0; 16];
        assert_eq!
        (
            pump(&mut bridge, &mut ragged, &mut slots, &mut link),
            Err(PumpError::Layout(BridgeError::PartialFrame))
        );

        let mut pcm = [0; 8];
        let mut short = [0; 15];
        assert_eq!
        (
            pump(&mut bridge, &mut pcm, &mut short, &mut link),
            Err(PumpError::Layout(BridgeError::OutputTooShort))
        );

        assert!(link.bytes.is_empty());
        let mut out = [0; 8];
        assert_eq!(bridge.drain_into(&mut out), 8);
    }

    /// Source that fills its whole buffer and reports a byte count of its own.
    struct Misreporting(usize);

    impl FrameSource for Misreporting
    {
        fn take_frames(&mut self, pcm: &mut [u8]) -> usize
        {
            pcm.fill(0x11);
            self.0
        }
    }

    #[test]
    fn the_pump_cuts_an_overstated_take_to_whole_frames_of_its_buffer()
    {
        let mut pcm = [0; 2 * PCM_FRAME_BYTES];
        let mut slots = [0; 2 * SLOT_FRAME_BYTES];
        let mut link = ChokedLink::new();
        assert_eq!(pump(&mut Misreporting(99), &mut pcm, &mut slots, &mut link), Ok(2));
        assert_eq!(link.bytes.len(), 16);
        assert_eq!(slots_of(&link.bytes[..8]), (0x1111_0000, 0x1111_0000));
        assert_eq!(slots_of(&link.bytes[8..]), (0x1111_0000, 0x1111_0000));
    }

    #[test]
    fn the_pump_cuts_a_ragged_take_to_whole_frames_and_pads_the_rest()
    {
        for reported in 5..=7
        {
            let mut pcm = [0; 2 * PCM_FRAME_BYTES];
            let mut slots = [0; 2 * SLOT_FRAME_BYTES];
            let mut link = ChokedLink::new();
            assert_eq!(pump(&mut Misreporting(reported), &mut pcm, &mut slots, &mut link), Ok(1));
            assert_eq!(slots_of(&link.bytes[..8]), (0x1111_0000, 0x1111_0000));
            assert_eq!(link.bytes[8..], [0; 8], "reported {reported}");
        }
    }

    #[test]
    fn the_production_sizes_hold_together()
    {
        const { assert!(PREFILL_FRAMES <= RING_FRAMES) };
        const { assert!(BLOCK_FRAMES <= PREFILL_FRAMES) };
        assert_eq!(PCM_FRAME_BYTES * WIDENING, SLOT_FRAME_BYTES);
    }

    /// Returns the drift estimate of `bridge` in whole frames, -1 before a start.
    fn estimate_of<const FRAMES: usize>(bridge: &Bridge<FRAMES>) -> i64
    {
        bridge.drift.as_ref().map_or(-1, |drift| drift.estimate >> ESTIMATE_FRACTION_BITS)
    }

    /// Pushes `count` frames in chunks of `chunk` frames.
    fn push_chunks(bridge: &mut Bridge<RING_FRAMES>, count: usize, chunk: usize)
    {
        let bytes = frames(0, count);

        for piece in bytes.chunks(chunk * PCM_FRAME_BYTES)
        {
            assert_eq!(bridge.push(piece).accepted * PCM_FRAME_BYTES, piece.len());
        }
    }

    /// Centre of the production ring, where the drift correction holds the
    /// estimate.
    const CENTRE: usize = RING_FRAMES / 2;

    /// Excess over the dead band at which the credit pays for corrections at
    /// their cap, `PACE_FRAME_DRAINS / CORRECTION_SPACING` frames.
    const CAP_EXCESS: usize = PACE_FRAME_DRAINS as usize / CORRECTION_SPACING as usize;

    /// Step of the sampled fill when a chunk stream locked to the link slips by
    /// one drain: the mean of the samples over a chunk moves by one block.
    const SLIP: usize = BLOCK_FRAMES;

    /// Lag of the estimate behind a fill the corrections pull at their cap: one
    /// tracker step and the corrections of one window.
    const TRACK: usize = TRACK_STEP_FRAMES as usize + (1 << WINDOW_SHIFT) / CORRECTION_SPACING as usize;

    /// Farthest a settled estimate strays from the centre: the dead band, the
    /// excess of corrections at their cap, a phase slip and the tracker lag.
    const REACH: usize = DEAD_BAND_FRAMES + CAP_EXCESS + SLIP + TRACK;

    /// Frames per second the corrections move the fill at their cap.
    const CAP_RATE: f64 = 44_100.0 / (BLOCK_FRAMES as f64 * CORRECTION_SPACING as f64);

    /// Frames per second the estimate follows at the most.
    const TRACK_RATE: f64 = TRACK_STEP_FRAMES as f64 * 44_100.0 / (BLOCK_FRAMES as f64 * (1 << WINDOW_SHIFT) as f64);

    /// Link blocks per minute, exact at 44.1 kHz.
    const BLOCKS_PER_MINUTE: u64 = 44_100 * 60 / BLOCK_FRAMES as u64;

    /// Start stretch of every drift simulation, three minutes.
    ///
    /// A start leaves the mean of the samples at most `SLIP / 2 + 441` frames
    /// from the seed with 10 ms of jitter. The estimate reaches it at 23 frames
    /// per second, 25 s, and the corrections take the part past the dead band
    /// out at their cap less a 100 ppm drift, 433 frames at 4.78 frames per
    /// second, 91 s.
    const SETTLE_BLOCKS: u64 = 3 * BLOCKS_PER_MINUTE;

    /// Radio side of a drift simulation.
    #[derive(Debug, Clone, Copy)]
    struct Radio
    {
        /// Rate of the phone against the link, in ppm.
        ppm: i64,
        /// Frames of every chunk.
        chunk: usize,
        /// Offset of the first chunk against the first drain, in link frames.
        phase: u64,
        /// Largest delay of a chunk behind its due time, in link frames.
        jitter: u64,
        /// Seed of the delays.
        seed: u64,
        /// Every chunk whose index is a multiple of this lands with the next
        /// one, two packets decoded back to back. 0 bunches nothing.
        bunch_every: u64,
        /// Link time a radio stall starts and the length of the stall, in
        /// link frames. Every chunk due in the stall lands when it ends.
        stall: (u64, u64),
    }

    impl Radio
    {
        /// Builds a radio with neither bunched packets nor a stall.
        const fn steady(ppm: i64, chunk: usize, phase: u64, jitter: u64, seed: u64) -> Self
        {
            Self { ppm, chunk, phase, jitter, seed, bunch_every: 0, stall: (0, 0) }
        }

        /// Lowest fill after a drain the controller can let through.
        ///
        /// The estimate stays within `REACH` of the centre. The fill sampled
        /// before a drain dips half a chunk under the mean of the samples, a
        /// late chunk takes up to `jitter` frames more, and the drain takes a
        /// block plus the frame of a removal. It falls under zero where the
        /// derivation proves nothing, and there only the simulation holds.
        fn fill_floor(&self) -> i64
        {
            CENTRE as i64 - REACH as i64 - self.jitter as i64 - self.chunk as i64 / 2 - BLOCK_FRAMES as i64 - 1
        }

        /// Highest fill before a drain the controller can let through.
        ///
        /// A late chunk lowers the estimate by up to `jitter` against the fill
        /// on time, and the fill peaks half a chunk over the mean of the
        /// samples.
        fn fill_ceiling(&self) -> i64
        {
            (CENTRE + REACH + self.jitter as usize + self.chunk / 2) as i64
        }

        /// Largest gap between the net corrections and the drift over a run.
        ///
        /// The ring gives out what it took, so the net corrections equal the
        /// frames accepted, less the frames given, less the change of fill. The
        /// frames accepted differ from the drift by one chunk and one delay.
        fn correction_slack(&self) -> u64
        {
            (self.fill_ceiling() - self.fill_floor()) as u64 + self.chunk as u64 + self.jitter
        }

        /// Most corrections a start can spend against the drift.
        ///
        /// The start seeds the estimate at the prefill. The mean of the samples
        /// sits within half a block of it by phase, and chunks late past their
        /// drain lower it by up to `jitter`. The controller takes out the part
        /// past the dead band, and the credit runs on for up to `CAP_EXCESS`.
        fn start_offset_corrections(&self) -> u64
        {
            ((SLIP / 2 + self.jitter as usize).saturating_sub(DEAD_BAND_FRAMES) + CAP_EXCESS) as u64
        }

        /// Most corrections a start can spend over `given` frames: the offset
        /// of `start_offset_corrections` and the drift over the stretch.
        fn start_corrections(&self, given: u64) -> u64
        {
            self.start_offset_corrections() + self.ppm.unsigned_abs() * given / 1_000_000
        }
    }

    /// What a drift simulation saw over one stretch of link time.
    #[derive(Debug, Clone, Copy, Default)]
    struct Stretch
    {
        given: u64,
        fill_min: usize,
        fill_max: usize,
        estimate_min: i64,
        estimate_max: i64,
        inserted: u64,
        removed: u64,
        dropped: u64,
        trimmed: u64,
        underruns: u64,
        short_drains: u64,
    }

    impl Stretch
    {
        /// Builds a stretch that has seen nothing.
        fn new() -> Self
        {
            Self { fill_min: usize::MAX, estimate_min: i64::MAX, estimate_max: i64::MIN, ..Self::default() }
        }

        /// Removals less insertions.
        fn net(&self) -> i64
        {
            self.removed as i64 - self.inserted as i64
        }

        /// Corrections against a drift of `ppm`, and the smaller count of the
        /// two kinds at zero drift.
        fn against(&self, ppm: i64) -> u64
        {
            match ppm.signum()
            {
                1 => self.inserted,
                -1 => self.removed,
                _ => self.inserted.min(self.removed),
            }
        }

        /// Folds the statistics of the bridge in.
        fn close(&mut self, stats: BridgeStats)
        {
            self.inserted = u64::from(stats.inserted);
            self.removed = u64::from(stats.removed);
            self.dropped = u64::from(stats.dropped);
            self.trimmed = u64::from(stats.trimmed);
            self.underruns = u64::from(stats.underruns);
        }
    }

    /// What a drift simulation saw, before and after `SETTLE_BLOCKS`.
    #[derive(Debug, Clone, Copy)]
    struct Outcome
    {
        start: Stretch,
        settled: Stretch,
        started_at: u64,
        conserved: bool,
    }

    /// Runs `radio` into a production bridge, one pump drain per block of the
    /// link, for `blocks` blocks.
    ///
    /// A chunk is due once the phone has clocked its last frame: chunk `k` at
    /// `(k + 1) * chunk * 1e6 / (1e6 + ppm)` link frames past `phase`, late by
    /// a delay drawn in `0..=jitter`, never before the chunk ahead of it, and
    /// moved as `bunch_every` and `stall` say. Every chunk due at a drain lands
    /// before that drain. The fill and estimate figures cover started drains.
    fn simulate(radio: Radio, blocks: u64) -> Outcome
    {
        let mut bridge = Bridge::<RING_FRAMES>::new(PREFILL_FRAMES);
        assert_eq!(bridge.open(JOINT_STEREO_44_1), STEREO);

        let chunk: Vec<u8> = frames(0x1234, radio.chunk);
        let mut pcm = [0; BLOCK_FRAMES * PCM_FRAME_BYTES];
        let mut rng = Lcg(radio.seed);
        let rate = (1_000_000 + radio.ppm) as u128;
        let (stall_at, stall_length) = (u128::from(radio.stall.0) * 1_000, u128::from(radio.stall.1) * 1_000);

        // Due time of chunk `index`, in thousandths of a link frame.
        let mut due = |index: u64, previous: u128| -> u128
        {
            let bunched = u64::from(radio.bunch_every > 0 && index.is_multiple_of(radio.bunch_every.max(1)));
            let clocked = (u128::from(index + bunched) + 1) * radio.chunk as u128 * 1_000_000_000 / rate;
            let delay = rng.below(radio.jitter as usize + 1) as u128 * 1_000;
            let mut at = (clocked + delay + u128::from(radio.phase) * 1_000).max(previous);

            if (stall_at..stall_at + stall_length).contains(&at)
            {
                at = stall_at + stall_length;
            }

            at
        };

        let mut outcome = Outcome
        {
            start: Stretch::new(),
            settled: Stretch::new(),
            started_at: u64::MAX,
            conserved: false,
        };
        let mut index: u64 = 0;
        let mut next_due = due(0, 0);
        let mut accepted: u64 = 0;
        let mut level_at_settle: u64 = 0;

        for block in 0..blocks
        {
            if block == SETTLE_BLOCKS
            {
                outcome.start.close(bridge.take_stats());
                accepted = 0;
                level_at_settle = bridge.ring.len as u64;
            }

            let now = u128::from(block) * BLOCK_FRAMES as u128 * 1_000;

            while next_due <= now
            {
                accepted += bridge.push(&chunk).accepted as u64;
                index += 1;
                next_due = due(index, next_due);
            }

            let was_started = bridge.drift.is_some();
            let written = (bridge.drain_into(&mut pcm) / PCM_FRAME_BYTES) as u64;

            if written == 0 && outcome.started_at == u64::MAX
            {
                continue;
            }

            outcome.started_at = outcome.started_at.min(block);
            let stretch = if block < SETTLE_BLOCKS { &mut outcome.start } else { &mut outcome.settled };
            stretch.given += written;
            stretch.short_drains += u64::from(written < BLOCK_FRAMES as u64);

            if bridge.drift.is_some()
            {
                let estimate = estimate_of(&bridge);
                stretch.estimate_min = stretch.estimate_min.min(estimate);
                stretch.estimate_max = stretch.estimate_max.max(estimate);
                stretch.fill_min = stretch.fill_min.min(bridge.ring.len);

                if was_started
                {
                    stretch.fill_max = stretch.fill_max.max(bridge.ring.len + written as usize);
                }
            }
        }

        outcome.settled.close(bridge.take_stats());
        let settled = &outcome.settled;
        outcome.conserved = level_at_settle + accepted + settled.inserted
            == bridge.ring.len as u64 + settled.given + settled.removed + settled.trimmed;
        outcome
    }

    /// Asserts every property a steady drift simulation of `radio` must hold.
    fn assert_holds(radio: Radio, outcome: &Outcome)
    {
        let context = std::format!("{radio:?} {outcome:?}");
        let (start, settled) = (&outcome.start, &outcome.settled);
        assert!(outcome.conserved, "counters do not add up: {context}");

        for stretch in [start, settled]
        {
            assert_eq!(stretch.dropped, 0, "hard drops: {context}");
            assert_eq!(stretch.underruns, 0, "underruns: {context}");
            assert_eq!(stretch.short_drains, 0, "short drains: {context}");
        }

        let wait = START_WAIT_DRAINS as usize * BLOCK_FRAMES;
        assert!
        (
            outcome.started_at * BLOCK_FRAMES as u64 <= (PREFILL_FRAMES + LARGEST_CHUNK_FRAMES + radio.chunk + wait) as u64 + radio.jitter,
            "late start: {context}"
        );
        assert!
        (
            start.inserted + start.removed <= radio.start_corrections(start.given),
            "start spent {} corrections over {}: {context}",
            start.inserted + start.removed,
            radio.start_corrections(start.given)
        );
        assert!
        (
            start.against(radio.ppm) <= radio.start_offset_corrections(),
            "start spent {} corrections against the drift: {context}",
            start.against(radio.ppm)
        );
        assert!
        (
            settled.estimate_min >= (CENTRE - REACH) as i64 && settled.estimate_max <= (CENTRE + REACH) as i64,
            "estimate out of its band: {context}"
        );
        assert!
        (
            settled.fill_min as i64 >= radio.fill_floor() && settled.fill_max as i64 <= radio.fill_ceiling(),
            "fill out of [{}, {}]: {context}",
            radio.fill_floor(),
            radio.fill_ceiling()
        );

        let drift = radio.ppm * settled.given as i64 / 1_000_000;
        assert!
        (
            settled.net().abs_diff(drift) <= radio.correction_slack(),
            "net corrections {} against a drift of {drift} frames: {context}",
            settled.net()
        );
        if radio.ppm == 0
        {
            // At zero drift the drift estimate and the term inside the band
            // wander around zero and pay a trickle of corrections of both signs.
            // Measured over 27 streams of 60 minutes, three chunk sizes, three
            // jitters and three phases: 5.6 a minute at the most, 1.2 on
            // average. The bound allows 8 a minute.
            let minutes = settled.given / (44_100 * 60);
            assert!(settled.inserted + settled.removed <= 8 * minutes + 8, "corrections at zero drift: {context}");
        }
        else
        {
            assert_eq!(settled.against(radio.ppm), 0, "settled corrections against the drift: {context}");
        }
    }

    /// Drifts of the sweeps, in ppm. The derivations of the bounds cover 100 ppm
    /// and less. 200 ppm rides on the cap and holds in simulation.
    const DRIFTS: [i64; 9] = [-200, -100, -40, -10, 0, 10, 40, 100, 200];

    #[test]
    #[cfg_attr(coverage, ignore = "the unit tests of the controller, the start and the crossfade reach the same lines in a fraction of the time")]
    fn the_drift_correction_holds_the_ring_over_drift_chunk_size_phase_and_jitter()
    {
        // 128 frames is one SBC frame of 16 blocks and 8 subbands, 240 one
        // block of the link, 1024 the 8 frames of a 1008-byte media packet at
        // bitpool 53, 1920 the 15 frames a media packet carries at most. 441
        // frames of jitter is 10 ms.
        std::thread::scope(|scope|
        {
            for ppm in DRIFTS
            {
                scope.spawn(move ||
                {
                    for chunk in [128, 240, 1024, 1920]
                    {
                        for jitter in [0, 441]
                        {
                            for phase in [0, 131]
                            {
                                let radio = Radio::steady(ppm, chunk, phase, jitter, 0xD1F7 ^ phase);
                                let outcome = simulate(radio, SETTLE_BLOCKS + 4 * BLOCKS_PER_MINUTE);
                                std::println!("{radio:?} {outcome:?}");
                                assert_holds(radio, &outcome);

                                // With neither drift nor jitter the only offset is
                                // the phase of the start, within half a block of
                                // the seed, and the term inside the band takes it
                                // to the centre one frame per correction.
                                if jitter == 0 && ppm == 0
                                {
                                    let spent = outcome.start.inserted + outcome.start.removed
                                        + outcome.settled.inserted + outcome.settled.removed;
                                    assert!(spent <= (SLIP / 2 + TRACK) as u64, "{radio:?} {outcome:?}");
                                }
                            }
                        }
                    }
                });
            }
        });
    }

    #[test]
    #[cfg_attr(coverage, ignore = "the unit tests of the controller, the start and the crossfade reach the same lines in a fraction of the time")]
    fn the_drift_correction_holds_the_ring_for_three_hours()
    {
        std::thread::scope(|scope|
        {
            for (rank, ppm) in (0_u64..).zip(DRIFTS)
            {
                scope.spawn(move ||
                {
                    let radio = Radio::steady(ppm, 1920, rank * 29, 220, 0x3_0000 + rank);
                    let outcome = simulate(radio, SETTLE_BLOCKS + 180 * BLOCKS_PER_MINUTE);
                    std::println!("{radio:?} {outcome:?}");
                    assert_holds(radio, &outcome);
                });
            }
        });
    }

    #[test]
    #[cfg_attr(coverage, ignore = "the unit tests of the controller, the start and the crossfade reach the same lines in a fraction of the time")]
    fn a_packet_late_by_a_period_of_1024_frames_lands_with_the_next_without_a_gap()
    {
        // Every seventh packet waits for the next one and Bluedroid decodes
        // the two back to back, a 2048-frame step. The trough before it,
        // 2048 - (1024 + 240) / 2 - 1024 frames on a ring held at 2048, stays
        // over an empty ring. The derived floor of `Radio::fill_floor` with the
        // late period counted as jitter does not, so this rests on the
        // simulation.
        std::thread::scope(|scope|
        {
            for ppm in DRIFTS
            {
                scope.spawn(move ||
                {
                    let radio = Radio { bunch_every: 7, ..Radio::steady(ppm, 1_024, 17, 220, 0xB0 ^ ppm.unsigned_abs()) };
                    let outcome = simulate(radio, SETTLE_BLOCKS + 4 * BLOCKS_PER_MINUTE);
                    std::println!("{radio:?} {outcome:?}");
                    let context = std::format!("{radio:?} {outcome:?}");

                    for stretch in [&outcome.start, &outcome.settled]
                    {
                        assert_eq!((stretch.dropped, stretch.underruns, stretch.short_drains), (0, 0, 0), "{context}");
                    }

                    assert!(outcome.conserved, "{context}");
                    assert!(outcome.settled.fill_max <= RING_FRAMES - BLOCK_FRAMES, "{context}");
                    assert_eq!(outcome.settled.against(ppm), 0, "{context}");
                });
            }
        });
    }

    #[test]
    #[cfg_attr(coverage, ignore = "the unit tests of the controller, the start and the crossfade reach the same lines in a fraction of the time")]
    fn a_radio_stall_longer_than_the_fill_costs_one_gap_and_no_frame_of_the_backlog()
    {
        // A stall of 68 ms or 100 ms outlasts the fill, so the ring runs dry
        // once. The chunks due in the stall land together when it ends, the
        // ring that has not started keeps the newest of them, and it starts on
        // the chunk after the backlog, one period before the next. The mean
        // then sits within half a block and the jitter of the seed, as at any
        // start.
        std::thread::scope(|scope|
        {
            for ppm in DRIFTS
            {
                scope.spawn(move ||
                {
                    for chunk in [1_024, 1_920]
                    {
                        for stall in [3_000, 4_410]
                        {
                            let at = SETTLE_BLOCKS * BLOCK_FRAMES as u64 + 60 * 44_100;
                            let radio = Radio { stall: (at, stall), ..Radio::steady(ppm, chunk, 17, 220, 0x57 ^ stall) };
                            let outcome = simulate(radio, SETTLE_BLOCKS + 4 * BLOCKS_PER_MINUTE);
                            std::println!("{radio:?} {outcome:?}");
                            let context = std::format!("{radio:?} {outcome:?}");
                            let settled = &outcome.settled;

                            assert_eq!((outcome.start.dropped, outcome.start.underruns), (0, 0), "{context}");
                            assert_eq!((settled.underruns, settled.dropped), (1, 0), "{context}");
                            assert!(outcome.conserved, "{context}");
                            assert!(settled.against(ppm) <= radio.start_offset_corrections(), "{context}");
                        }
                    }
                });
            }
        });
    }

    /// Event on the phone side of a radio episode, times in seconds.
    #[derive(Debug, Clone, Copy)]
    enum Episode
    {
        /// Every chunk due in `length` seconds from `at` lands `delay` frames late.
        Late { at: f64, delay: f64, length: f64 },
        /// Every chunk due in `length` seconds from `at` lands when the stall ends.
        Stall { at: f64, length: f64 },
        /// The phone suspends at `at` for `length` seconds, then resumes with
        /// `backlog` chunks landing at once and `small` chunks of `small_frames`
        /// frames first.
        Pause { at: f64, length: f64, backlog: usize, small: usize, small_frames: usize },
    }

    /// Frames that land at a link time in frames, or the flush of a suspend.
    #[derive(Debug, Clone, Copy)]
    struct Landing
    {
        at: f64,
        frames: usize,
        flush: bool,
    }

    /// Lays out what a phone at `ppm` delivers over `seconds`: chunks of `chunk`
    /// frames late by a draw in `0..=jitter` frames, never before the chunk
    /// ahead of them, and moved by `episodes`.
    fn landings(ppm: i64, seconds: f64, chunk: usize, jitter: usize, episodes: &[Episode]) -> Vec<Landing>
    {
        let mut rng = Lcg(0xE915 ^ ppm.unsigned_abs());
        let mut spans: Vec<(f64, f64, usize, usize, usize)> = Vec::new();
        let mut from = 0.0;
        let mut resume = (0, 0, 0);

        for &episode in episodes
        {
            if let Episode::Pause { at, length, backlog, small, small_frames } = episode
            {
                spans.push((from, at, resume.0, resume.1, resume.2));
                from = at + length;
                resume = (backlog, small, small_frames);
            }
        }

        spans.push((from, seconds, resume.0, resume.1, resume.2));
        let mut out: Vec<Landing> = Vec::new();
        let mut previous = 0.0_f64;

        for (index, &(begin, end, backlog, small, small_frames)) in spans.iter().enumerate()
        {
            if index > 0
            {
                out.push(Landing { at: spans[index - 1].1 * 44_100.0, frames: 0, flush: true });
            }

            let size = |k: usize| if k < small { small_frames } else { chunk };
            let held: usize = (0..backlog).map(size).sum();
            let mut produced = 0;

            for k in 0..
            {
                produced += size(k);
                let clocked = (produced as f64 - held as f64).max(0.0) * 1e6 / (1e6 + ppm as f64);
                let mut at = begin * 44_100.0 + clocked;

                if at >= end * 44_100.0
                {
                    break;
                }

                at += rng.below(jitter + 1) as f64;

                for &episode in episodes
                {
                    match episode
                    {
                        Episode::Late { at: late, delay, length } if at >= late * 44_100.0 && at < (late + length) * 44_100.0 =>
                        {
                            at += delay;
                        }
                        Episode::Stall { at: stall, length } if at >= stall * 44_100.0 && at < (stall + length) * 44_100.0 =>
                        {
                            at = (stall + length) * 44_100.0;
                        }
                        _ => {}
                    }
                }

                at = at.max(previous);
                previous = at;
                out.push(Landing { at, frames: size(k), flush: false });
            }
        }

        out.sort_by(|a, b| a.at.total_cmp(&b.at));
        out
    }

    /// Corrections, underruns and hard drops of every drain of a radio episode.
    struct Trace
    {
        inserted: Vec<u32>,
        removed: Vec<u32>,
        underruns: Vec<u32>,
        dropped: Vec<u32>,
    }

    /// Plays `landings` into a production bridge for `seconds`.
    fn play(landings: &[Landing], seconds: f64) -> Trace
    {
        let mut bridge = Bridge::<RING_FRAMES>::new(PREFILL_FRAMES);
        assert_eq!(bridge.open(JOINT_STEREO_44_1), STEREO);
        let bytes = frames(0, LARGEST_CHUNK_FRAMES);
        let mut pcm = [0; BLOCK_FRAMES * PCM_FRAME_BYTES];
        let blocks = (seconds * 44_100.0 / BLOCK_FRAMES as f64) as usize;
        let mut trace = Trace { inserted: Vec::new(), removed: Vec::new(), underruns: Vec::new(), dropped: Vec::new() };
        let mut next = 0;

        for block in 0..blocks
        {
            let now = (block * BLOCK_FRAMES) as f64;

            while next < landings.len() && landings[next].at <= now
            {
                let landing = landings[next];

                if landing.flush
                {
                    bridge.flush();
                }
                else
                {
                    let _ = bridge.push(&bytes[..landing.frames * PCM_FRAME_BYTES]);
                }

                next += 1;
            }

            let _ = bridge.drain_into(&mut pcm);
            let stats = bridge.take_stats();
            trace.inserted.push(stats.inserted);
            trace.removed.push(stats.removed);
            trace.underruns.push(stats.underruns);
            trace.dropped.push(stats.dropped);
        }

        trace
    }

    /// What a radio episode spent over one window of link time.
    #[derive(Debug)]
    struct Window
    {
        inserted: u64,
        removed: u64,
        underruns: u64,
        dropped: u64,
        drift: f64,
        cap_run: f64,
    }

    impl Window
    {
        /// Corrections against a drift of `ppm`, and the smaller count of the
        /// two kinds at zero drift.
        fn against(&self, ppm: i64) -> u64
        {
            match ppm.signum()
            {
                1 => self.inserted,
                -1 => self.removed,
                _ => self.inserted.min(self.removed),
            }
        }

        /// Distance of the net corrections from the drift, in frames.
        fn excess(&self) -> f64
        {
            (self.removed as f64 - self.inserted as f64 - self.drift).abs()
        }
    }

    /// Returns what `trace` spent from `from` to `to` seconds, with the longest
    /// run of corrections at their spacing, in seconds.
    fn window(trace: &Trace, from: f64, to: f64, ppm: i64) -> Window
    {
        let first = (from * 44_100.0 / BLOCK_FRAMES as f64) as usize;
        let last = ((to * 44_100.0 / BLOCK_FRAMES as f64) as usize).min(trace.inserted.len());
        let sum = |values: &[u32]| values[first..last].iter().map(|&value| u64::from(value)).sum::<u64>();
        let marks: Vec<usize> = (first..last).filter(|&drain| trace.inserted[drain] + trace.removed[drain] > 0).collect();
        let mut longest = 0;
        let mut run_start = 0;

        for (rank, &mark) in marks.iter().enumerate()
        {
            if rank == 0 || mark - marks[rank - 1] > CORRECTION_SPACING as usize + 1
            {
                run_start = mark;
            }

            longest = longest.max(mark - run_start);
        }

        Window
        {
            inserted: sum(&trace.inserted),
            removed: sum(&trace.removed),
            underruns: sum(&trace.underruns),
            dropped: sum(&trace.dropped),
            drift: ppm as f64 * 1e-6 * ((last - first) * BLOCK_FRAMES) as f64,
            cap_run: (longest * BLOCK_FRAMES) as f64 / 44_100.0,
        }
    }

    #[test]
    #[cfg_attr(coverage, ignore = "the unit tests of the controller, the start and the crossfade reach the same lines in a fraction of the time")]
    fn radio_episodes_spend_corrections_within_their_derived_bounds()
    {
        // Chunks of 1920 frames with up to 220 frames of jitter. The radio
        // runs 620 frames late for 3 s or for one packet once a minute,
        // stalls for 100 ms and 68 ms, and resumes after a suspend on a
        // backlog, on smaller chunks, or on both.
        const CHUNK: usize = 1_920;
        const JITTER: usize = 220;

        std::thread::scope(|scope|
        {
            for ppm in [-100_i64, -40, 0, 40, 100]
            {
                scope.spawn(move ||
                {
                    let drift_rate = ppm.unsigned_abs() as f64 * 44_100.0 * 1e-6;
                    let net_cap = CAP_RATE - drift_rate;
                    // A radio late for 3 s moves the estimate one step per
                    // window started, and the net corrections of a window then
                    // differ from the drift by the swing of the held estimate.
                    let windows = (3.0 * 44_100.0 / (BLOCK_FRAMES << WINDOW_SHIFT) as f64).ceil() as usize;
                    let dip = TRACK_STEP_FRAMES as usize * windows;
                    let late_excess = (2 * (DEAD_BAND_FRAMES + CAP_EXCESS) + dip) as f64;
                    let late_run = (dip + TRACK + SLIP / 2) as f64 / net_cap;
                    // A restart lands the mean within half a block and the
                    // jitter of the seed.
                    let offset = SLIP / 2 + JITTER;
                    let restart_against = ((offset - DEAD_BAND_FRAMES) + CAP_EXCESS) as u64;
                    let restart_excess = late_excess + offset as f64;
                    let restart_run = (offset - DEAD_BAND_FRAMES) as f64 / net_cap + offset as f64 / TRACK_RATE;

                    let late_every = |length: f64| -> Vec<Episode>
                    {
                        (0..8).map(|i| Episode::Late { at: 90.0 + 60.0 * f64::from(i), delay: 620.0, length }).collect()
                    };

                    for (name, episodes) in [("late 3 s", late_every(3.0)), ("late packet", late_every(CHUNK as f64 / 44_100.0))]
                    {
                        let trace = play(&landings(ppm, 600.0, CHUNK, JITTER, &episodes), 600.0);

                        for i in 0..8
                        {
                            let at = 90.0 + 60.0 * f64::from(i);
                            let w = window(&trace, at, at + 60.0, ppm);
                            let context = std::format!("{ppm} ppm {name} at {at} s: {w:?}");
                            std::println!("EPISODE {context}");
                            assert_eq!((w.underruns, w.dropped), (0, 0), "{context}");
                            assert_eq!(w.against(ppm), 0, "{context}");
                            assert!(w.excess() <= late_excess, "{context} over {late_excess}");
                            assert!(w.cap_run <= late_run, "{context} over {late_run} s");
                        }
                    }

                    let stalls = [Episode::Stall { at: 90.0, length: 0.1 }, Episode::Stall { at: 210.0, length: 0.068 }];
                    let pauses =
                    [
                        Episode::Pause { at: 90.0, length: 3.0, backlog: 3, small: 0, small_frames: 0 },
                        Episode::Pause { at: 180.0, length: 3.0, backlog: 0, small: 5, small_frames: 1_280 },
                        Episode::Pause { at: 270.0, length: 3.0, backlog: 4, small: 3, small_frames: 1_024 },
                        Episode::Pause { at: 360.0, length: 3.0, backlog: 0, small: 0, small_frames: 0 },
                    ];
                    let stall_trace = play(&landings(ppm, 330.0, CHUNK, JITTER, &stalls), 330.0);
                    let pause_trace = play(&landings(ppm, 450.0, CHUNK, JITTER, &pauses), 450.0);
                    let restarts =
                    [
                        ("stall 100 ms", &stall_trace, 90.0, 1),
                        ("stall 68 ms", &stall_trace, 210.0, 1),
                        ("resume on a backlog", &pause_trace, 93.0, 0),
                        ("resume on smaller chunks", &pause_trace, 183.0, 0),
                        ("resume on both", &pause_trace, 273.0, 0),
                        ("resume", &pause_trace, 363.0, 0),
                    ];

                    for (name, trace, at, underruns) in restarts
                    {
                        let w = window(trace, at, at + 60.0, ppm);
                        let context = std::format!("{ppm} ppm {name} at {at} s: {w:?}");
                        std::println!("EPISODE {context}");
                        assert_eq!((w.underruns, w.dropped), (underruns, 0), "{context}");
                        assert!(w.against(ppm) <= restart_against, "{context} over {restart_against}");
                        assert!(w.excess() <= restart_excess, "{context} over {restart_excess}");
                        assert!(w.cap_run <= restart_run, "{context} over {restart_run} s");
                    }
                });
            }
        });
    }

    /// Builds `count` frames of `left` and `right`, as bytes.
    fn signal(count: usize, left: impl Fn(usize) -> i16, right: impl Fn(usize) -> i16) -> Vec<u8>
    {
        (0..count)
            .flat_map(|n|
            {
                let [l0, l1] = left(n).to_le_bytes();
                let [r0, r1] = right(n).to_le_bytes();
                [l0, l1, r0, r1]
            })
            .collect()
    }

    /// Returns the two samples of every frame of `bytes`.
    fn samples_of(bytes: &[u8]) -> Vec<[i16; 2]>
    {
        bytes
            .chunks_exact(PCM_FRAME_BYTES)
            .map(|frame| [i16::from_le_bytes([frame[0], frame[1]]), i16::from_le_bytes([frame[2], frame[3]])])
            .collect()
    }

    /// Pushes `count` frames of `bytes`, from frame `from`, in chunks of `chunk`
    /// frames.
    fn push_span(bridge: &mut Bridge<RING_FRAMES>, bytes: &[u8], from: usize, count: usize, chunk: usize)
    {
        let span = &bytes[from * PCM_FRAME_BYTES..(from + count) * PCM_FRAME_BYTES];

        for piece in span.chunks(chunk * PCM_FRAME_BYTES)
        {
            assert_eq!(bridge.push(piece).accepted * PCM_FRAME_BYTES, piece.len());
        }
    }

    /// Builds `count` mono frames of `sample`, one 16-bit sample each, as bytes.
    fn mono_signal(count: usize, sample: impl Fn(usize) -> i16) -> Vec<u8>
    {
        (0..count).flat_map(|n| sample(n).to_le_bytes()).collect()
    }

    /// Runs `bridge` through a start, `drains` drains fed a block each with a
    /// push past the room of the ring at drain 40, and returns the bytes the
    /// pumps put on the link and the frames the pushes dropped.
    ///
    /// `bytes` holds frames of `frame_bytes` each, which is how the chunks are
    /// cut, so a mono and a stereo bridge fed the same frames run the same
    /// schedule.
    fn run_link(bridge: &mut Bridge<RING_FRAMES>, bytes: &[u8], frame_bytes: usize, drains: usize) -> (Vec<u8>, usize)
    {
        let mut pcm = [0; BLOCK_FRAMES * PCM_FRAME_BYTES];
        let mut slots = [0; BLOCK_FRAMES * SLOT_FRAME_BYTES];
        let mut link = ChokedLink::new();
        let mut fed = 0;
        let mut dropped = 0;
        let mut push = |bridge: &mut Bridge<RING_FRAMES>, frames: usize, fed: &mut usize|
        {
            for piece in bytes[*fed * frame_bytes..(*fed + frames) * frame_bytes].chunks(BLOCK_FRAMES * frame_bytes)
            {
                dropped += bridge.push(piece).dropped;
            }

            *fed += frames;
        };

        push(bridge, 1_148, &mut fed);

        for drain in 0..drains
        {
            if drain == 40
            {
                let room = RING_FRAMES - bridge.ring.len;
                push(bridge, room + 300, &mut fed);
            }
            else if drain > 0
            {
                push(bridge, BLOCK_FRAMES, &mut fed);
            }

            assert!(pump(bridge, &mut pcm, &mut slots, &mut link).is_ok());
        }

        (link.bytes, dropped)
    }

    #[test]
    fn a_mono_stream_puts_its_sample_on_both_slots_through_corrections_and_fades()
    {
        // A mono bridge and a stereo bridge fed the same samples, the stereo
        // one on both channels of every frame, run one schedule: a start that
        // fades in, a prefill 900 frames under the centre that sets insertions
        // running, and a push past the room of the ring that fades around the
        // frames it drops. The link of the mono bridge is the link of the
        // stereo one byte for byte, and every frame of it carries one word on
        // both slots. A mono push that read two samples a frame, or wrote the
        // sample into one channel only, parts on the first frame it plays.
        const DRAINS: usize = 70;
        const LENGTH: usize = 1_148 + DRAINS * BLOCK_FRAMES + RING_FRAMES + 300;

        let tone = |n: usize| (30_000.0 * (2.0 * core::f64::consts::PI * 997.0 * n as f64 / 44_100.0).sin()).round() as i16;
        let mono = mono_signal(LENGTH, tone);
        let stereo = signal(LENGTH, tone, tone);

        let mut one = Bridge::<RING_FRAMES>::new(1_148);
        assert_eq!(one.open(MONO_44_1), MONO);
        let (mono_link, mono_dropped) = run_link(&mut one, &mono, MONO_FRAME_BYTES, DRAINS);

        let mut two = Bridge::<RING_FRAMES>::new(1_148);
        assert_eq!(two.open(JOINT_STEREO_44_1), STEREO);
        let (stereo_link, stereo_dropped) = run_link(&mut two, &stereo, PCM_FRAME_BYTES, DRAINS);

        let stats = one.take_stats();
        assert!(stats.inserted >= 2, "the run corrected {stats:?}");
        assert!(mono_dropped > 0, "the run dropped nothing, so no fade played around a drop");
        assert_eq!(mono_dropped, stereo_dropped);
        assert_eq!(mono_link.len(), DRAINS * BLOCK_FRAMES * SLOT_FRAME_BYTES);
        assert!(mono_link == stereo_link, "the mono link parted from the stereo link of the same samples");

        let words: Vec<(i32, i32)> = mono_link.chunks_exact(SLOT_FRAME_BYTES).map(slots_of).collect();
        let loud = words.iter().filter(|(left, _)| left.unsigned_abs() > 1 << 29).count();

        for (n, (left, right)) in words.iter().enumerate()
        {
            assert_eq!(left, right, "frame {n} of the mono link");
        }

        assert!(loud > words.len() / 4, "the mono link carried {loud} loud frames of {}", words.len());
    }

    #[test]
    fn a_renegotiation_between_mono_and_stereo_keeps_every_frame_aligned()
    {
        // A mono chunk trailing one byte, then stereo, then mono again, each
        // stream opened on the one before without a close. The trailing byte is
        // discarded as a stereo trailing partial frame is, and every frame
        // after a renegotiation is read at the stride of the stream it belongs
        // to: an open that kept the stride of the previous stream shifts every
        // frame after it.
        let mut out = [0; 8 * PCM_FRAME_BYTES];
        let mut bridge = Bridge::<FIXTURE_FRAMES>::new(0);

        assert_eq!(bridge.open(JOINT_STEREO_44_1), STEREO);
        assert_eq!(bridge.push(&frames(0, 2)).accepted, 2);

        assert_eq!(bridge.open(MONO_44_1), MONO);
        assert_eq!(bridge.drain_into(&mut out), 0, "the stereo frames outlived the mono open");

        let mut chunk = mono_signal(3, |n| [0x1234, -0x0765, i16::MIN][n]);
        chunk.push(JUNK);
        assert_eq!(bridge.push(&chunk), Pushed { accepted: 3, dropped: 0, trailing_bytes: 1 });
        assert_eq!(bridge.push(&mono_signal(1, |_| i16::MAX)).accepted, 1);
        assert_eq!(bridge.drain_into(&mut out), 4 * PCM_FRAME_BYTES);
        assert_eq!
        (
            samples_of(&out[..4 * PCM_FRAME_BYTES]),
            [[0x1234, 0x1234], [-0x0765, -0x0765], [i16::MIN, i16::MIN], [i16::MAX, i16::MAX]]
        );

        assert_eq!(bridge.push(&mono_signal(2, |_| 0x0101)).accepted, 2);
        assert_eq!(bridge.open(JOINT_STEREO_44_1), STEREO);
        assert_eq!(bridge.drain_into(&mut out), 0, "the mono frames outlived the stereo open");
        assert_eq!(bridge.push(&frames(9, 2)).accepted, 2);
        assert_eq!(bridge.drain_into(&mut out), 2 * PCM_FRAME_BYTES);
        assert_eq!(index_of(&out[..4]), 9);
        assert_eq!(index_of(&out[4..8]), 10);

        assert_eq!(bridge.open(MONO_44_1), MONO);
        assert_eq!(bridge.push(&mono_signal(2, |n| [7, -7][n])).accepted, 2);
        assert_eq!(bridge.drain_into(&mut out), 2 * PCM_FRAME_BYTES);
        assert_eq!(samples_of(&out[..2 * PCM_FRAME_BYTES]), [[7, 7], [-7, -7]]);
    }

    /// Fills a fresh bridge to `level` frames of `bytes` and starts it: all but
    /// the last block before a drain that stays under the level, then the last
    /// block alone, then the drain that starts the ring. Returns the output of
    /// that first drain.
    fn start_on(bridge: &mut Bridge<RING_FRAMES>, bytes: &[u8], level: usize) -> Vec<u8>
    {
        let mut pcm = [0; BLOCK_FRAMES * PCM_FRAME_BYTES];
        push_span(bridge, bytes, 0, level - BLOCK_FRAMES, BLOCK_FRAMES);
        assert_eq!(bridge.drain_into(&mut pcm), 0, "started under its level");
        push_span(bridge, bytes, level - BLOCK_FRAMES, BLOCK_FRAMES, BLOCK_FRAMES);
        assert_eq!(bridge.drain_into(&mut pcm), pcm.len(), "did not start at its level");
        pcm.to_vec()
    }

    #[test]
    fn the_estimate_moves_toward_each_window_mean_by_a_bounded_step()
    {
        let window = 1_usize << WINDOW_SHIFT;

        for (seed, fill) in [(2_048, 2_400), (2_048, 1_700), (2_000, 2_003), (2_000, 2_000)]
        {
            let mut control = DriftControl::seeded(seed);

            for step in 1..=100_i64
            {
                for _ in 0..window - 1
                {
                    let _ = control.update(fill, 2_048, false);
                    assert_eq!(control.estimate >> ESTIMATE_FRACTION_BITS, (seed as i64 + (step - 1) * TRACK_STEP_FRAMES * (fill as i64 - seed as i64).signum()).clamp(seed.min(fill) as i64, seed.max(fill) as i64));
                }

                let _ = control.update(fill, 2_048, false);
                let moved = (seed as i64 + step * TRACK_STEP_FRAMES * (fill as i64 - seed as i64).signum()).clamp(seed.min(fill) as i64, seed.max(fill) as i64);
                assert_eq!(control.estimate, moved << ESTIMATE_FRACTION_BITS, "seed {seed} fill {fill} window {step}");
            }
        }

        // A window mean counts every sample: 31 samples at 2048 and one at
        // 3072 average 2080, which the estimate reaches in eight windows.
        let mut control = DriftControl::seeded(2_048);

        for _ in 0..8
        {
            for _ in 0..window - 1
            {
                let _ = control.update(2_048, 2_048, false);
            }

            let _ = control.update(3_072, 2_048, false);
        }

        assert_eq!(control.estimate, 2_080 << ESTIMATE_FRACTION_BITS);
    }

    #[test]
    fn a_start_trims_to_half_a_chunk_over_the_prefill_and_seeds_the_estimate_there()
    {
        let mut pcm = [0; BLOCK_FRAMES * PCM_FRAME_BYTES];

        for (chunk, start) in
        [
            (1, PREFILL_FRAMES),
            (128, PREFILL_FRAMES),
            (240, PREFILL_FRAMES),
            (241, PREFILL_FRAMES),
            (242, PREFILL_FRAMES + 1),
            (1_024, PREFILL_FRAMES + 392),
            (1_920, PREFILL_FRAMES + 840),
        ]
        {
            let mut bridge = Bridge::<RING_FRAMES>::new(PREFILL_FRAMES);
            assert_eq!(bridge.open(JOINT_STEREO_44_1), STEREO);
            assert_eq!(bridge.push(&frames(0, chunk)).accepted, chunk);
            assert_eq!(bridge.start_level(), start, "chunk {chunk}");

            push_chunks(&mut bridge, start - chunk.min(start) - 1, BLOCK_FRAMES);
            assert_eq!(bridge.drain_into(&mut pcm), 0, "chunk {chunk} started under its level");

            // One lone push of 151 frames reaches the level and overshoots it.
            push_chunks(&mut bridge, 151, BLOCK_FRAMES);
            let overshoot = bridge.ring.len - start;
            assert_eq!(bridge.drain_into(&mut pcm), pcm.len(), "chunk {chunk}");

            let stats = bridge.take_stats();
            assert_eq!((stats.trimmed as usize, stats.fill_max, stats.samples), (overshoot, start, 1), "chunk {chunk}");
            assert_eq!(estimate_of(&bridge), PREFILL_FRAMES as i64);
        }

        // Chunks under two blocks land two to a drain, which still makes a
        // lone burst.
        let mut small_chunks = Bridge::<RING_FRAMES>::new(PREFILL_FRAMES);
        assert_eq!(small_chunks.open(JOINT_STEREO_44_1), STEREO);
        push_chunks(&mut small_chunks, 1_920, 128);
        assert_eq!(small_chunks.drain_into(&mut pcm), 0);
        push_chunks(&mut small_chunks, 256, 128);
        assert_eq!(small_chunks.drain_into(&mut pcm), pcm.len(), "two chunks of 128 frames waited");

        // A flush forgets the burst of the stream it ends.
        let mut flushed = Bridge::<RING_FRAMES>::new(300);
        assert_eq!(flushed.open(JOINT_STEREO_44_1), STEREO);
        assert_eq!(flushed.push(&frames(0, 500)).accepted, 500);
        flushed.flush();
        assert_eq!(flushed.push(&frames(0, 500)).accepted, 500);
        assert_eq!(flushed.drain_into(&mut pcm), pcm.len(), "a flushed burst made the start wait");

        // A push larger than any Bluedroid chunk counts as the largest one.
        let mut large = Bridge::<RING_FRAMES>::new(PREFILL_FRAMES);
        assert_eq!(large.open(JOINT_STEREO_44_1), STEREO);
        assert_eq!(large.push(&frames(0, 4_000)).accepted, 4_000);
        assert_eq!(large.start_level(), PREFILL_FRAMES + 840);
        assert_eq!(large.lone_burst(), LARGEST_CHUNK_FRAMES);

        // A ring too small to correct starts at its prefill, trims nothing and
        // needs no lone chunk.
        let mut small = Bridge::<FIXTURE_FRAMES>::new(FIXTURE_PREFILL);
        assert_eq!(small.open(JOINT_STEREO_44_1), STEREO);
        assert_eq!(small.push(&frames(0, 30)).accepted, 30);
        let mut out = [0; 30 * PCM_FRAME_BYTES];
        assert_eq!(small.drain_into(&mut out), out.len());
        assert_eq!(small.take_stats().trimmed, 0);
    }

    #[test]
    fn a_start_waits_for_a_lone_chunk_after_a_backlog_and_no_longer_than_its_deadline()
    {
        let mut pcm = [0; BLOCK_FRAMES * PCM_FRAME_BYTES];
        let start = PREFILL_FRAMES + 840;

        // A backlog of three chunks lands between two drains: the ring trims
        // to its level and waits, a drain with no push waits too, and the
        // drain after one lone chunk starts it.
        let mut bridge = Bridge::<RING_FRAMES>::new(PREFILL_FRAMES);
        assert_eq!(bridge.open(JOINT_STEREO_44_1), STEREO);
        assert_eq!(bridge.push(&frames(0, 1_920)).accepted, 1_920);
        assert_eq!(bridge.push(&frames(1_920, 1_920)).accepted, 1_920);
        assert_eq!(bridge.push(&frames(3_840, 1_920)).accepted, 1_920);
        assert_eq!(bridge.drain_into(&mut pcm), 0);
        assert_eq!(bridge.ring.len, start);
        assert_eq!(bridge.drain_into(&mut pcm), 0);
        assert_eq!(bridge.push(&frames(5_760, 1_920)).accepted, 1_920);
        assert_eq!(bridge.drain_into(&mut pcm), pcm.len());
        let first = 5_760 + 1_920 - start;
        assert_eq!(index_of(&bridge.history[0]), (first + BLOCK_FRAMES - 1) as u16);
        assert_eq!(bridge.take_stats().trimmed as usize, 7_680 - 4_096 + 4_096 - start);

        // Two chunks between every pair of drains never make a lone chunk, so
        // the ring starts after START_WAIT_DRAINS drains of waiting.
        let mut pairs = Bridge::<RING_FRAMES>::new(PREFILL_FRAMES);
        assert_eq!(pairs.open(JOINT_STEREO_44_1), STEREO);
        assert_eq!(waiting_drains(&mut pairs), START_WAIT_DRAINS);
    }

    #[test]
    fn a_ring_that_has_not_started_drops_its_oldest_frames_when_full()
    {
        let mut bridge = Bridge::<RING_FRAMES>::new(PREFILL_FRAMES);
        assert_eq!(bridge.open(JOINT_STEREO_44_1), STEREO);
        assert_eq!(bridge.push(&frames(0, RING_FRAMES)).accepted, RING_FRAMES);
        assert_eq!(bridge.push(&frames(10_000, 100)), Pushed { accepted: 100, dropped: 0, trailing_bytes: 0 });
        let stats = bridge.take_stats();
        assert_eq!((stats.trimmed, stats.dropped), (100, 0));
        assert_eq!(index_of(&frame(100)), 100);
        assert_eq!(bridge.ring.pop().map(|oldest| index_of(&oldest)), Some(100));

        // Once started, a full ring drops the incoming frames and counts them.
        let mut started = Bridge::<RING_FRAMES>::new(PREFILL_FRAMES);
        assert_eq!(started.open(JOINT_STEREO_44_1), STEREO);
        let bytes = frames(0, RING_FRAMES);
        let _ = start_on(&mut started, &bytes, PREFILL_FRAMES);
        let room = RING_FRAMES - started.ring.len;
        assert_eq!(started.push(&frames(0, room + 7)), Pushed { accepted: room, dropped: 7, trailing_bytes: 0 });
        assert_eq!(started.take_stats().dropped, 7);
    }

    #[test]
    fn a_zero_prefill_starts_on_the_one_frame_the_ring_holds()
    {
        let mut pcm = [0; BLOCK_FRAMES * PCM_FRAME_BYTES];
        let mut bridge = Bridge::<RING_FRAMES>::new(0);
        assert_eq!(bridge.open(JOINT_STEREO_44_1), STEREO);

        // Past the wait for a lone chunk too, an empty ring never starts.
        for _ in 0..2 * START_WAIT_DRAINS
        {
            assert_eq!(bridge.drain_into(&mut pcm), 0, "an empty ring started");
        }

        assert_eq!(bridge.take_stats().underruns, 0);

        assert_eq!(bridge.push(&frames(0, 3)).accepted, 3);
        assert_eq!(bridge.drain_into(&mut pcm), PCM_FRAME_BYTES);
        assert_eq!(index_of(&bridge.history[0]), 2);
        assert_eq!(pcm[..PCM_FRAME_BYTES], [0; PCM_FRAME_BYTES], "the last frame before a dry ring sounded");
        let stats = bridge.take_stats();
        assert_eq!((stats.trimmed, stats.underruns), (2, 1));
    }

    #[test]
    fn flush_and_an_underrun_keep_what_the_source_showed_and_open_and_close_forget_it()
    {
        let mut pcm = [0; BLOCK_FRAMES * PCM_FRAME_BYTES];
        let gap_level = 13 * BLOCK_FRAMES + BLOCK_FRAMES;

        for restart in ["flush", "underrun", "open", "close"]
        {
            let mut bridge = Bridge::<RING_FRAMES>::new(PREFILL_FRAMES);
            assert_eq!(bridge.open(JOINT_STEREO_44_1), STEREO);
            assert_eq!(bridge.push(&frames(0, 1_920)).accepted, 1_920);
            assert_eq!(bridge.push(&frames(0, 1_920)).accepted, 1_920);
            assert_eq!(bridge.drain_into(&mut pcm), 0);
            assert_eq!(bridge.push(&frames(0, 1_920)).accepted, 1_920);
            assert_eq!(bridge.drain_into(&mut pcm), pcm.len());

            // Topped up, the ring rides through 13 drains without a push, and
            // the push after them measures the gap.
            assert_eq!(bridge.push(&frames(0, 1_200)).accepted, 1_200);

            for _ in 0..13
            {
                assert_eq!(bridge.drain_into(&mut pcm), pcm.len(), "{restart}");
            }

            assert_eq!(bridge.push(&frames(0, 1_920)).accepted, 1_920);
            assert_eq!((bridge.delivery.longest, bridge.start_level()), (13, gap_level), "{restart}");

            let level = match restart
            {
                "flush" =>
                {
                    bridge.flush();
                    gap_level
                }
                "underrun" =>
                {
                    run_dry(&mut bridge);
                    assert_eq!(bridge.take_stats().underruns, 1);
                    gap_level
                }
                "open" =>
                {
                    assert_eq!(bridge.open(JOINT_STEREO_44_1), STEREO);
                    PREFILL_FRAMES
                }
                _ =>
                {
                    bridge.close();
                    assert_eq!(bridge.open(JOINT_STEREO_44_1), STEREO);
                    PREFILL_FRAMES
                }
            };

            assert!(bridge.drift.is_none() && bridge.segment.is_none(), "{restart}");
            assert_eq!(bridge.start_level(), level, "{restart}");
        }
    }

    #[test]
    fn a_flush_a_new_stream_and_an_underrun_restart_the_estimate()
    {
        let mut pcm = [0; BLOCK_FRAMES * PCM_FRAME_BYTES];
        let bytes = frames(0, 4 * RING_FRAMES);

        for restart in ["flush", "open", "close", "underrun"]
        {
            let mut bridge = Bridge::<RING_FRAMES>::new(PREFILL_FRAMES);
            assert_eq!(bridge.open(JOINT_STEREO_44_1), STEREO);
            let _ = start_on(&mut bridge, &bytes, PREFILL_FRAMES);

            // Blocks the ring sits far over the dead band drive the estimate
            // up and start a correction.
            push_chunks(&mut bridge, 1_900, BLOCK_FRAMES);

            for _ in 0..40 * (1 << WINDOW_SHIFT)
            {
                assert_eq!(bridge.drain_into(&mut pcm), pcm.len());
                push_chunks(&mut bridge, BLOCK_FRAMES, BLOCK_FRAMES);
            }

            assert!(bridge.take_stats().removed > 0, "{restart}");
            assert!(estimate_of(&bridge) > (PREFILL_FRAMES + DEAD_BAND_FRAMES) as i64);

            match restart
            {
                "flush" => bridge.flush(),
                "open" => assert_eq!(bridge.open(JOINT_STEREO_44_1), STEREO),
                "close" =>
                {
                    bridge.close();
                    assert_eq!(bridge.open(JOINT_STEREO_44_1), STEREO);
                }
                _ =>
                {
                    run_dry(&mut bridge);
                    assert_eq!(bridge.take_stats().underruns, 1);
                }
            }

            assert!(bridge.segment.is_none() && bridge.drift.is_none(), "{restart}");
            let _ = play_out_tail(&mut bridge);

            // Restarted at its centre and fed one block per drain, the ring
            // sits in the dead band, so a restarted estimate never corrects.
            let level = bridge.start_level();
            let _ = start_on(&mut bridge, &bytes, level);

            for _ in 0..8 * CORRECTION_SPACING
            {
                push_chunks(&mut bridge, BLOCK_FRAMES, BLOCK_FRAMES);
                assert_eq!(bridge.drain_into(&mut pcm), pcm.len(), "{restart}");
            }

            let stats = bridge.take_stats();
            assert_eq!((stats.inserted, stats.removed), (0, 0), "{restart}");
        }
    }


    /// Drains the tail a flush left, pushing nothing, and returns its output.
    /// Fails when the tail takes more than two drains or does not end silent.
    fn play_out_tail(bridge: &mut Bridge<RING_FRAMES>) -> Vec<u8>
    {
        let mut pcm = [0; BLOCK_FRAMES * PCM_FRAME_BYTES];
        let mut output = Vec::new();
        let mut drains = 0;

        while bridge.tail > 0
        {
            let written = bridge.drain_into(&mut pcm);
            output.extend_from_slice(&pcm[..written]);
            drains += 1;
            assert!(drains <= 2, "the tail took more than two drains");
        }

        assert!(output.len() < 2 * pcm.len(), "the tail filled two drains");
        assert!(output.rchunks(PCM_FRAME_BYTES).next().is_none_or(|last| last == [0; PCM_FRAME_BYTES]), "the tail did not end silent");
        assert_eq!(bridge.fade.gain, 0);
        output
    }

    /// Drains `bridge`, pushing nothing, until a drain comes back short, and
    /// fails when the ring does not run dry within the drains its capacity
    /// allows.
    fn run_dry(bridge: &mut Bridge<RING_FRAMES>)
    {
        let mut pcm = [0; BLOCK_FRAMES * PCM_FRAME_BYTES];

        for _ in 0..=RING_FRAMES / BLOCK_FRAMES + 1
        {
            if bridge.drain_into(&mut pcm) < pcm.len()
            {
                return;
            }
        }

        panic!("the ring did not run dry");
    }

    /// Pushes two 480-frame chunks between every pair of drains, which never
    /// makes a lone chunk, and returns the drains the ring waited at its level
    /// before it started. Fails when it has not started within 256 drains.
    fn waiting_drains(bridge: &mut Bridge<RING_FRAMES>) -> u32
    {
        let mut pcm = [0; BLOCK_FRAMES * PCM_FRAME_BYTES];
        let mut waited = 0;

        for _ in 0..256
        {
            push_chunks(bridge, 960, 480);

            if bridge.drain_into(&mut pcm) > 0
            {
                return waited;
            }

            if bridge.ring.len >= bridge.start_level()
            {
                waited += 1;
            }
        }

        panic!("the start never came");
    }

    #[test]
    fn a_start_that_ran_out_its_wait_and_a_flush_mid_wait_leave_the_next_wait_whole()
    {
        let mut pcm = [0; BLOCK_FRAMES * PCM_FRAME_BYTES];

        // A start that waited its whole wait, then a gap, then the same
        // backlog: the second start waits the whole wait again.
        let mut bridge = Bridge::<RING_FRAMES>::new(PREFILL_FRAMES);
        assert_eq!(bridge.open(JOINT_STEREO_44_1), STEREO);
        assert_eq!(waiting_drains(&mut bridge), START_WAIT_DRAINS);
        run_dry(&mut bridge);
        assert_eq!(bridge.take_stats().underruns, 1);
        assert_eq!(waiting_drains(&mut bridge), START_WAIT_DRAINS);

        // A suspend in the middle of a wait: the stream that resumes waits
        // the whole wait.
        let mut flushed = Bridge::<RING_FRAMES>::new(PREFILL_FRAMES);
        assert_eq!(flushed.open(JOINT_STEREO_44_1), STEREO);
        let mut waited = 0;

        for _ in 0..256
        {
            push_chunks(&mut flushed, 960, 480);
            assert_eq!(flushed.drain_into(&mut pcm), 0, "started before its wait ran out");

            if flushed.ring.len >= flushed.start_level()
            {
                waited += 1;
            }

            if waited == START_WAIT_DRAINS / 2
            {
                break;
            }
        }

        assert_eq!(waited, START_WAIT_DRAINS / 2);
        flushed.flush();
        assert_eq!(waiting_drains(&mut flushed), START_WAIT_DRAINS);
    }

    /// Returns the modified Bessel function of order zero, from its series.
    fn bessel_i0(x: f64) -> f64
    {
        let (mut sum, mut term, mut k) = (1.0, 1.0, 1.0);

        while term > 1e-20 * sum
        {
            term *= (x / (2.0 * k)).powi(2);
            sum += term;
            k += 1.0;
        }

        sum
    }

    /// Returns the weights of the resampler at `position / CROSSFADE_FRAMES`,
    /// computed as `resample` does, as whole weight units.
    fn weights_at(position: usize) -> [i32; SINC_TAPS]
    {
        let phase = position >> SINC_PHASE_SHIFT;
        let fraction = (position & ((1 << SINC_PHASE_SHIFT) - 1)) as i32;
        let mut weights = [0; SINC_TAPS];

        for (t, weight) in weights.iter_mut().enumerate()
        {
            let from = i32::from(SINC_ROWS[phase][t]);
            let to = i32::from(SINC_ROWS[(phase + 1).min(256)][t]);
            *weight = from + (((to - from) * fraction + SINC_PHASE_ROUND) >> SINC_PHASE_SHIFT);
        }

        weights
    }

    #[test]
    fn the_resampler_table_is_the_kaiser_windowed_sinc_it_claims_and_keeps_the_treble_level()
    {
        const BETA: f64 = 4.0;

        for (phase, row) in SINC_ROWS.iter().enumerate()
        {
            let mu = phase as f64 / 256.0;
            let ideal: Vec<f64> = (-3..=4)
                .map(|t|
                {
                    let d = f64::from(t) - mu;
                    let sinc = if d.abs() < 1e-15 { 1.0 } else { (core::f64::consts::PI * d).sin() / (core::f64::consts::PI * d) };
                    let r = d / 4.0;
                    let window = if r.abs() <= 1.0 { bessel_i0(BETA * (1.0 - r * r).max(0.0).sqrt()) / bessel_i0(BETA) } else { 0.0 };
                    sinc * window
                })
                .collect();
            let total: f64 = ideal.iter().sum();
            let mut expected: Vec<i32> = ideal.iter().map(|v| (v / total * 16_384.0).round_ties_even() as i32).collect();
            let mut largest = 0;

            for t in 1..SINC_TAPS
            {
                if ideal[t].abs() > ideal[largest].abs()
                {
                    largest = t;
                }
            }

            expected[largest] += 16_384 - expected.iter().sum::<i32>();
            assert_eq!(row.map(i32::from).to_vec(), expected, "phase {phase}");
        }

        // Gain of the resampler at every position a correction reads, and the
        // sum of magnitudes that bounds a weighted sum of full-scale samples.
        let mut largest_magnitude = 0;

        for (hz, low, high) in [(6_000.0, -0.01, 0.08), (10_000.0, -0.01, 0.08), (15_000.0, -0.09, 0.02)]
        {
            let w = 2.0 * core::f64::consts::PI * hz / 44_100.0;

            for position in 1..CROSSFADE_FRAMES
            {
                let weights = weights_at(position);
                largest_magnitude = largest_magnitude.max(weights.iter().map(|w| w.abs()).sum::<i32>());
                let mu = position as f64 / CROSSFADE_FRAMES as f64;
                let (mut re, mut im) = (0.0, 0.0);

                for (t, &weight) in weights.iter().enumerate()
                {
                    let delay = t as f64 - 3.0 - mu;
                    re += f64::from(weight) * (w * delay).cos();
                    im += f64::from(weight) * (w * delay).sin();
                }

                let gain = 20.0 * (re.hypot(im) / 16_384.0).log10();
                assert!((low..=high).contains(&gain), "{hz} Hz at position {position}: {gain:+.3} dB");
            }
        }

        assert!(i64::from(largest_magnitude) * 32_768 + i64::from(SINC_ROUND) < i64::from(i32::MAX));
    }

    /// Returns the frames the drains give out for `input`, from the definition
    /// of the resampler in floating point over the weights `resample` reads,
    /// with a correction of kind `correction` starting at each output frame
    /// listed in `openings`, and frames before the first one silent. Each frame
    /// carries the slack its fixed point may stray by: half a sample of
    /// rounding.
    fn reference_stream
    (
        input: &[[i16; 2]],
        correction: Correction,
        openings: &[usize],
        total: usize
    ) -> Vec<([f64; 2], f64)>
    {
        let length = CROSSFADE_FRAMES;
        let sample = |n: usize, channel: usize| f64::from(input[n][channel]);
        let unit = f64::from(1_u32 << SINC_FRACTION_BITS);
        let resampled = |i: usize, position: usize| -> ([f64; 2], f64)
        {
            let weights = weights_at(position);
            let mut values = [0.0; 2];

            for (channel, value) in values.iter_mut().enumerate()
            {
                let sum: f64 = weights
                    .iter()
                    .enumerate()
                    .map(|(t, &weight)| if i + t >= 3 { f64::from(weight) * sample(i + t - 3, channel) } else { 0.0 })
                    .sum();
                *value = (sum / unit).clamp(f64::from(i16::MIN), f64::from(i16::MAX));
            }

            (values, 0.5)
        };

        let mut out: Vec<([f64; 2], f64)> = Vec::new();
        let mut at = 0;

        while out.len() < total
        {
            if !openings.contains(&out.len())
            {
                out.push(([sample(at, 0), sample(at, 1)], 0.0));
                at += 1;
                continue;
            }

            match correction
            {
                Correction::Remove =>
                {
                    for k in 0..length - 1
                    {
                        out.push(resampled(at + k, k + 1));
                    }
                }
                Correction::Insert =>
                {
                    out.push(([sample(at, 0), sample(at, 1)], 0.0));

                    for k in 1..length
                    {
                        out.push(resampled(at + k - 1, length - k));
                    }

                    out.push(([sample(at + length - 1, 0), sample(at + length - 1, 1)], 0.0));
                }
            }

            at += length;
        }

        out.truncate(total);
        out
    }

    /// Returns `reference` as it plays from a start: frame `n` at a gain of
    /// `n + 1` in `FADE_FRAMES`ths until full, rounded down.
    fn faded_in(reference: Vec<([f64; 2], f64)>) -> Vec<([f64; 2], f64)>
    {
        reference
            .into_iter()
            .enumerate()
            .map(|(n, (want, slack))|
            {
                if n + 1 >= FADE_FRAMES
                {
                    return (want, slack);
                }

                let gain = (n + 1) as f64 / FADE_FRAMES as f64;
                ([want[0] * gain - 0.5, want[1] * gain - 0.5], slack * gain + 0.5)
            })
            .collect()
    }

    #[test]
    fn a_correction_resamples_one_frame_out_or_in_across_drains_and_keeps_the_frames_aligned()
    {
        const BLOCKS: usize = 70;
        const LENGTH: usize = 22_000;

        let tone = |n: usize, hz: f64, phase: f64| -> i16
        {
            (30_000.0 * (2.0 * core::f64::consts::PI * hz * n as f64 / 44_100.0 + phase).sin()).round() as i16
        };
        let square = |n: usize, period: usize| -> i16
        {
            if n % period < period / 2 { i16::MAX } else { i16::MIN }
        };

        let cases: [(&str, Vec<u8>); 3] =
        [
            ("sine 1 kHz", signal(LENGTH, |n| tone(n, 1_000.0, 0.0), |n| tone(n, 1_000.0, 0.0).saturating_neg())),
            ("sine 15 kHz", signal(LENGTH, |n| tone(n, 15_000.0, 0.3), |n| tone(n, 15_000.0, 0.3).saturating_neg())),
            ("full scale square", signal(LENGTH, |n| square(n, 50), |n| square(n, 34))),
        ];

        // A prefill 900 frames off the centre puts the estimate past the band,
        // and a push of one block per drain keeps it there.
        for (prefill, correction) in [(2_948, Correction::Remove), (1_148, Correction::Insert)]
        {
            for (name, input) in &cases
            {
                let context = std::format!("{name}, {correction:?}");
                let mut bridge = Bridge::<RING_FRAMES>::new(prefill);
                assert_eq!(bridge.open(JOINT_STEREO_44_1), STEREO);

                let mut output = start_on(&mut bridge, input, prefill);
                let mut fed = prefill;
                let mut openings: Vec<usize> = Vec::new();
                let mut pcm = [0; BLOCK_FRAMES * PCM_FRAME_BYTES];

                if bridge.segment.is_some()
                {
                    openings.push(0);
                }

                for _ in 1..BLOCKS
                {
                    push_span(&mut bridge, input, fed, BLOCK_FRAMES, BLOCK_FRAMES);
                    fed += BLOCK_FRAMES;
                    let running = bridge.segment.is_some();
                    let at = output.len() / PCM_FRAME_BYTES;
                    assert_eq!(bridge.drain_into(&mut pcm), pcm.len(), "{context}");
                    output.extend_from_slice(&pcm);

                    if !running && bridge.segment.is_some()
                    {
                        openings.push(at);
                    }
                }

                let samples_in = samples_of(input);
                let samples_out = samples_of(&output);
                assert!(openings.len() >= 3, "{context}: corrections started at {openings:?}");
                let reference = faded_in(reference_stream(&samples_in, correction, &openings, samples_out.len()));

                for (n, (got, (want, slack))) in samples_out.iter().zip(&reference).enumerate()
                {
                    for channel in 0..2
                    {
                        assert!
                        (
                            (f64::from(got[channel]) - want[channel]).abs() <= slack + 1e-9,
                            "{context}: frame {n} channel {channel} gives {} for {} within {slack}",
                            got[channel],
                            want[channel]
                        );
                    }

                    if name.starts_with("sine")
                    {
                        assert!((i32::from(got[0]) + i32::from(got[1])).abs() <= 1, "{context}: frame {n} lost its pairing");
                    }
                }

                let stats = bridge.take_stats();
                let spacing = openings.windows(2).map(|pair| pair[1] - pair[0]).min().unwrap_or(0);
                assert!(spacing >= CROSSFADE_FRAMES, "{context}: {openings:?}");
                let done = match correction
                {
                    Correction::Remove => stats.removed,
                    Correction::Insert => stats.inserted,
                };
                assert!(done as usize + 1 >= openings.len() && done as usize <= openings.len(), "{context}: {stats:?}");
            }
        }
    }

    #[test]
    fn a_flush_forgets_the_running_correction_and_the_frames_behind_it()
    {
        // A full-scale stream starts with a correction on its first drain and
        // is flushed while the correction runs. The quiet stream after it
        // starts its own correction on its first drain, reading silence behind
        // its first frame and nothing of the stream before.
        const PREFILL: usize = 3_600;
        let mut bridge = Bridge::<RING_FRAMES>::new(PREFILL);
        assert_eq!(bridge.open(JOINT_STEREO_44_1), STEREO);
        let square = |n: usize| if n % 6 < 3 { i16::MAX } else { i16::MIN };
        let loud = signal(PREFILL + 4 * BLOCK_FRAMES, square, square);
        let _ = start_on(&mut bridge, &loud, PREFILL);
        let mut pcm = [0; BLOCK_FRAMES * PCM_FRAME_BYTES];
        push_span(&mut bridge, &loud, PREFILL, BLOCK_FRAMES, BLOCK_FRAMES);
        assert_eq!(bridge.drain_into(&mut pcm), pcm.len());
        assert!(bridge.segment.is_some_and(|segment| segment.index > 0), "no correction ran at the flush");
        bridge.flush();
        assert_eq!(play_out_tail(&mut bridge).len(), FADE_FRAMES * PCM_FRAME_BYTES);

        let tone = |n: usize| (8_000.0 * (2.0 * core::f64::consts::PI * 3_000.0 * n as f64 / 44_100.0).sin()).round() as i16;
        let quiet = signal(PREFILL + 4 * BLOCK_FRAMES, tone, |n| tone(n).saturating_neg());
        let mut output = start_on(&mut bridge, &quiet, PREFILL);
        assert!(bridge.segment.is_some_and(|segment| segment.index == BLOCK_FRAMES), "the new stream did not open its own correction");
        push_span(&mut bridge, &quiet, PREFILL, BLOCK_FRAMES, BLOCK_FRAMES);
        assert_eq!(bridge.drain_into(&mut pcm), pcm.len());
        output.extend_from_slice(&pcm);

        let got = samples_of(&output);
        let reference = faded_in(reference_stream(&samples_of(&quiet), Correction::Remove, &[0], got.len()));

        for (n, (frame, (want, slack))) in got.iter().zip(&reference).enumerate()
        {
            for channel in 0..2
            {
                assert!
                (
                    (f64::from(frame[channel]) - want[channel]).abs() <= slack + 1e-9,
                    "frame {n} channel {channel} gives {} for {}",
                    frame[channel],
                    want[channel]
                );
            }
        }
    }

    /// Returns the lowest and highest 10 ms level of `samples`, in dB against a
    /// sine of `peak`.
    fn level_range(samples: &[i16], peak: f64) -> (f64, f64)
    {
        let reference = peak / 2.0_f64.sqrt();
        samples
            .chunks_exact(441)
            .map(|window|
            {
                let power = window.iter().map(|&s| f64::from(s) * f64::from(s)).sum::<f64>() / 441.0;
                20.0 * (power.sqrt() / reference).log10()
            })
            .fold((f64::MAX, f64::MIN), |(low, high), level| (low.min(level), high.max(level)))
    }

    #[test]
    #[cfg_attr(coverage, ignore = "the table test reaches the same lines in a fraction of the time")]
    fn corrections_at_their_cap_keep_the_level_of_a_treble_tone()
    {
        // A prefill 1552 frames over the centre runs the corrections at their
        // cap for the whole capture, their resampling tiling 85 percent of the
        // time. The table test holds the gain of every position within
        // +0.08 and -0.01 dB at 6 and 10 kHz and within +0.02 and -0.09 dB at
        // 15 kHz. These bounds add 0.05 dB for the rounding of the samples and
        // the pitch shift inside a 10 ms window. A two-tap linear mix dips
        // 0.82, 2.4 and 6.3 dB there, and a four-point cubic 0.10, 0.74 and
        // 3.5 dB.
        std::thread::scope(|scope|
        {
            for (hz, low, high) in [(6_000.0, -0.06, 0.13), (10_000.0, -0.06, 0.13), (15_000.0, -0.14, 0.07)]
            {
                scope.spawn(move ||
                {
                    let mut bridge = Bridge::<RING_FRAMES>::new(3_600);
                    assert_eq!(bridge.open(JOINT_STEREO_44_1), STEREO);
                    let sine = |n: usize| (30_000.0 * (2.0 * core::f64::consts::PI * hz * n as f64 / 44_100.0).sin()).round() as i16;
                    let mut pcm = [0; BLOCK_FRAMES * PCM_FRAME_BYTES];
                    let mut left = Vec::new();
                    let mut spent = 0;
                    let drains = 15 * 44_100 / BLOCK_FRAMES;
                    let skip = 3 * 44_100 / BLOCK_FRAMES;

                    for drain in 0..drains
                    {
                        let from = drain * BLOCK_FRAMES;
                        let _ = bridge.push(&signal(BLOCK_FRAMES, |n| sine(from + n), |n| sine(from + n).saturating_neg()));
                        let written = bridge.drain_into(&mut pcm) / PCM_FRAME_BYTES;
                        let stats = bridge.take_stats();

                        if drain >= skip
                        {
                            spent += stats.inserted + stats.removed;
                            left.extend(samples_of(&pcm[..written * PCM_FRAME_BYTES]).iter().map(|frame| frame[0]));
                        }
                    }

                    let (lowest, highest) = level_range(&left, 30_000.0);
                    std::println!("{hz} Hz: {spent} corrections, 10 ms level {lowest:+.3}..{highest:+.3} dB");
                    assert!(spent >= 100, "{hz} Hz: the corrections did not run at their cap: {spent}");
                    assert!(lowest >= low && highest <= high, "{hz} Hz: 10 ms level {lowest:+.3}..{highest:+.3} dB out of {low}..{high}");
                });
            }
        });
    }

    /// Feeds `drains` drains of `fill` to `control` and returns the drains that
    /// corrected, with their kind.
    fn corrections_of(control: &mut DriftControl, fill: usize, drains: usize) -> Vec<(usize, Correction)>
    {
        (0..drains)
            .filter_map(|drain| control.update(fill, 2_048, true).map(|kind| (drain, kind)))
            .collect()
    }

    /// Frames of credit one drain adds for an estimate `offset` frames from the
    /// centre, with no drift learnt: a thirty-second of the offset, and the
    /// excess past the band in full.
    fn inflow(offset: i64) -> f64
    {
        let band = DEAD_BAND_FRAMES as i64;
        let excess = if offset > band { offset - band } else if offset < -band { offset + band } else { 0 };
        offset as f64 / f64::from(1_u32 << INNER_PACE_SHIFT) + excess as f64
    }

    #[test]
    fn the_credit_pays_for_the_offset_inside_the_band_slowly_and_past_it_in_full()
    {
        const DRAINS: usize = 4_096;

        for offset in [0_i64, 1, 32, 64, 128, 129, 200, 320, 1_000, -1, -64, -128, -129, -320, -1_000]
        {
            let fill = (2_048 + offset) as usize;
            let mut control = DriftControl::seeded(fill);
            let fired = corrections_of(&mut control, fill, DRAINS);
            let kind = if offset > 0 { Correction::Remove } else { Correction::Insert };
            assert!(fired.iter().all(|&(_, k)| k == kind), "offset {offset}");

            let per_drain = inflow(offset).abs();
            let first = if per_drain > 0.0 { (PACE_FRAME_DRAINS as f64 / per_drain).ceil() as usize - 1 } else { usize::MAX };

            if first >= DRAINS
            {
                assert!(fired.is_empty(), "offset {offset}: {fired:?}");
                continue;
            }

            assert_eq!(fired.first().map(|&(drain, _)| drain), Some(first), "offset {offset}");
            assert!(fired.windows(2).all(|pair| pair[1].0 - pair[0].0 >= CORRECTION_SPACING as usize), "offset {offset}");
            let interval = (PACE_FRAME_DRAINS as f64 / per_drain).max(f64::from(CORRECTION_SPACING));
            let expected = ((DRAINS - first) as f64 / interval).ceil();
            assert!((fired.len() as f64 - expected).abs() <= 1.0, "offset {offset}: {} corrections, expected {expected}", fired.len());
        }

        // The credit stops at one correction, so a long stretch far past the
        // band pays for nothing once the estimate comes back near the edge.
        let far = 2_048 + DEAD_BAND_FRAMES + 1_000;
        let mut control = DriftControl::seeded(far);
        assert_eq!(corrections_of(&mut control, far, 400).len(), 20);
        let edge = 2_048 + DEAD_BAND_FRAMES + 1;
        control.estimate = fixed_frames(edge);
        assert!(corrections_of(&mut control, edge, 100).len() <= 1, "an uncapped credit kept paying");

        // The same on the side of the insertions.
        let deep = 2_048 - DEAD_BAND_FRAMES - 1_000;
        let mut control = DriftControl::seeded(deep);
        assert_eq!(corrections_of(&mut control, deep, 400).len(), 20);
        let edge = 2_048 - DEAD_BAND_FRAMES - 1;
        control.estimate = fixed_frames(edge);
        assert!(corrections_of(&mut control, edge, 100).len() <= 1, "an uncapped credit kept paying for insertions");
    }

    #[test]
    fn a_correction_still_running_defers_the_next_without_spending_the_spacing()
    {
        let fill = 2_048 + DEAD_BAND_FRAMES + 500;
        let mut control = DriftControl::seeded(fill);
        let mut fired = Vec::new();

        for drain in 0..80
        {
            // Drains 20 to 29 carry a correction still running.
            if control.update(fill, 2_048, !(20..30).contains(&drain)).is_some()
            {
                fired.push(drain);
            }
        }

        let spacing = CORRECTION_SPACING as usize;
        let first = (PACE_FRAME_DRAINS as f64 / inflow(fill as i64 - 2_048)).ceil() as usize - 1;
        let mut expected = std::vec![first];

        while let Some(&last) = expected.last()
        {
            let next = if (20..30).contains(&(last + spacing)) { 30 } else { last + spacing };

            if next >= 80
            {
                break;
            }

            expected.push(next);
        }

        assert_eq!(fired, expected);
        assert!(expected.contains(&30), "the run never deferred a correction");
    }

    #[test]
    fn the_drift_estimate_learns_the_walk_of_the_fill_once_the_start_has_settled()
    {
        // A fill that walks one frame every `period` drains. Without
        // corrections the estimate walks with it. With them the corrections take
        // the walk out and the fill holds still. Both are the same drift.
        for (period, sign, corrected) in [(40_i64, 1_i64, false), (40, -1, false), (400, 1, false), (40, 1, true), (40, -1, true)]
        {
            let context = std::format!("period {period} sign {sign} corrected {corrected}");
            let mut control = DriftControl::seeded(10_000);
            let mut fill = 10_000_i64;
            let drains = i64::from(DRIFT_WARMUP_DRAINS) + 12 * (1 << DRIFT_SHIFT);

            for drain in 0..drains
            {
                if drain % period == 0
                {
                    fill += sign;
                }

                match control.update(fill as usize, 10_000, corrected)
                {
                    Some(Correction::Remove) => fill -= 1,
                    Some(Correction::Insert) => fill += 1,
                    None => {}
                }

                if drain < i64::from(DRIFT_WARMUP_DRAINS)
                {
                    assert_eq!(control.drift(), 0, "{context}: learnt during the start");
                }
            }

            let expected = (fixed_frames(1) / period * sign) as f64;
            let learnt = control.drift() as f64;
            assert!((learnt - expected).abs() <= expected.abs() * 0.03 + 2.0, "{context}: learnt {learnt}, walk {expected}");

            if corrected
            {
                assert!((fill - 10_000).abs() <= DEAD_BAND_FRAMES as i64, "{context}: the corrections let the fill walk to {fill}");
            }
        }
    }

    #[test]
    fn a_late_radio_that_catches_up_leaves_the_drift_estimate_where_it_was()
    {
        // Every minute the fill sinks 620 frames for 3 s, then comes back. The
        // estimate dips and returns, so the drift estimate swings and settles
        // back at zero.
        let mut control = DriftControl::seeded(2_048);
        let minute = 44_100 * 60 / BLOCK_FRAMES;
        let late = 3 * 44_100 / BLOCK_FRAMES;
        let mut swing = 0_i64;

        for drain in 0..10 * minute
        {
            let fill = if drain > minute && drain % minute < late { 2_048 - 620 } else { 2_048 };
            let _ = control.update(fill, 2_048, true);
            swing = swing.max(control.drift().abs());
        }

        // A dip of 72 frames of the estimate, in and out within 6 s, moves the
        // first stage by at most 72 frames over 16384 drains and the second by
        // a sixth of that, well under a hundredth of a frame per drain.
        assert!(swing <= fixed_frames(1) / 100, "drift estimate swung to {swing}");
        assert!(control.drift().abs() <= fixed_frames(1) / 1_000, "drift estimate left at {}", control.drift());
    }

    #[test]
    fn short_drains_let_a_correction_run_to_its_end_before_the_next()
    {
        // Drains of 32 frames stretch a correction over 128 drains, longer than
        // the spacing, and the next one waits for it to end.
        let bytes = frames(0, 4 * RING_FRAMES);
        let mut bridge = Bridge::<RING_FRAMES>::new(3_000);
        assert_eq!(bridge.open(JOINT_STEREO_44_1), STEREO);
        let _ = start_on(&mut bridge, &bytes, 3_000);
        let mut pcm = [0; 32 * PCM_FRAME_BYTES];
        let mut indices = Vec::new();
        let mut fed = 3_000;

        for _ in 0..200
        {
            push_span(&mut bridge, &bytes, fed, 32, 32);
            fed += 32;
            assert_eq!(bridge.drain_into(&mut pcm), pcm.len());

            match bridge.segment
            {
                Some(segment) => indices.push(segment.index),
                None if !indices.is_empty() => break,
                None => {}
            }
        }

        assert!(indices.len() >= 120, "{indices:?}");
        assert!(indices.windows(2).all(|pair| pair[1] == pair[0] + 32), "a correction restarted: {indices:?}");
        assert_eq!(bridge.take_stats().removed, 1);
    }

    #[test]
    fn an_underrun_in_the_middle_of_a_correction_ends_it_and_keeps_the_counts_true()
    {
        let mut pcm = [0; BLOCK_FRAMES * PCM_FRAME_BYTES];

        for (prefill, correction) in [(3_000, Correction::Remove), (1_100, Correction::Insert)]
        {
            let bytes = frames(0, RING_FRAMES);
            let mut bridge = Bridge::<RING_FRAMES>::new(prefill);
            assert_eq!(bridge.open(JOINT_STEREO_44_1), STEREO);
            let mut given = start_on(&mut bridge, &bytes, prefill).len() / PCM_FRAME_BYTES;

            for _ in 0..64
            {
                if bridge.segment.is_some()
                {
                    break;
                }

                given += bridge.drain_into(&mut pcm) / PCM_FRAME_BYTES;
            }

            assert!(bridge.segment.is_some(), "{correction:?}: no correction started");

            // Nothing more lands, so the ring runs dry inside the correction.
            for _ in 0..=RING_FRAMES / BLOCK_FRAMES + 1
            {
                let written = bridge.drain_into(&mut pcm) / PCM_FRAME_BYTES;
                given += written;

                if written < BLOCK_FRAMES
                {
                    break;
                }
            }

            let stats = bridge.take_stats();
            assert!(bridge.segment.is_none() && bridge.drift.is_none(), "{correction:?}");
            assert_eq!(stats.underruns, 1, "{correction:?}");
            // The ring gave out what it took, one frame fewer per counted
            // removal and one more per counted insertion, and a correction that
            // waits for its read ahead leaves those frames in the ring.
            assert_eq!
            (
                prefill + stats.inserted as usize,
                given + stats.removed as usize + bridge.ring.len,
                "{correction:?} {stats:?}"
            );
            assert!(bridge.ring.len <= SINC_LOOKAHEAD, "{correction:?}");
        }
    }

    #[test]
    fn the_statistics_cover_the_period_since_the_last_snapshot()
    {
        let mut pcm = [0; BLOCK_FRAMES * PCM_FRAME_BYTES];
        let mut bridge = Bridge::<RING_FRAMES>::new(3_000);
        assert_eq!(bridge.take_stats(), BridgeStats::default());
        assert_eq!(bridge.push(&frames(0, 10)).dropped, 10);
        assert_eq!(bridge.take_stats().dropped, 0, "a closed gate drops without a hard drop");

        // The ring starts at 3000 frames, past the band edge, so the credit pays
        // a removal on drain `first` and the spacing lets the next one start
        // CORRECTION_SPACING drains later. A removal takes its frame out when
        // its last output leaves, on the drain `(CROSSFADE_FRAMES - 2) /
        // BLOCK_FRAMES` after it started.
        let first = (PACE_FRAME_DRAINS as f64 / inflow(3_000 - 2_048)).ceil() as usize - 1;
        let second = first + CORRECTION_SPACING as usize;
        let lands = (CROSSFADE_FRAMES - 2) / BLOCK_FRAMES;
        let drains = second + lands + 2;
        assert_eq!(bridge.open(JOINT_STEREO_44_1), STEREO);
        let bytes = frames(0, 3 * RING_FRAMES);
        push_span(&mut bridge, &bytes, 0, 3_000 - BLOCK_FRAMES, BLOCK_FRAMES);
        assert_eq!(bridge.drain_into(&mut pcm), 0);
        push_chunks(&mut bridge, 600, 200);
        assert_eq!(bridge.drain_into(&mut pcm), 0, "a burst of 600 frames made a lone chunk");
        assert_eq!(bridge.push(&frames(0, 200)).accepted, 200);
        let mut fills = Vec::new();

        for drain in 0..drains
        {
            if drain > 0
            {
                assert_eq!(bridge.push(&frames(0, BLOCK_FRAMES)).accepted, BLOCK_FRAMES);
            }

            fills.push(bridge.ring.len);
            assert_eq!(bridge.drain_into(&mut pcm), pcm.len());
        }

        // The start drain holds the lone chunk over the level and trims it
        // before it samples.
        let expected: Vec<usize> = (0..drains)
            .map(|drain| 3_000 - usize::from(drain > first + lands) - usize::from(drain > second + lands))
            .collect();
        assert_eq!(fills[0], 3_200);
        assert_eq!(fills[1..], expected[1..]);
        assert_eq!
        (
            bridge.take_stats(),
            BridgeStats
            {
                samples: drains as u32,
                fill_min: expected[drains - 1],
                fill_max: 3_000,
                fill_mean: expected.iter().sum::<usize>() / drains,
                inserted: 0,
                removed: 2,
                dropped: 0,
                trimmed: (3_000 - BLOCK_FRAMES + 600 + 200 - 3_000) as u32,
                underruns: 0,
            }
        );
        assert_eq!(bridge.take_stats(), BridgeStats::default());
    }

    #[test]
    #[cfg_attr(coverage, ignore = "the unit tests of the controller reach the same lines in a fraction of the time")]
    fn a_drift_that_reaches_the_fill_in_block_steps_is_corrected_at_its_own_rate()
    {
        // Chunks of 1920, 960 and 240 frames, whole blocks, and no jitter: the
        // arrivals keep one phase against the drains, so the drift reaches the
        // sampled fill as one 240-frame step per slip, 55 s apart at 99 ppm.
        // From 300 s, once the drift estimate has settled, no 10 s window may
        // reach 90 percent of the cap, which the step cycling filled to the cap
        // and emptied to 5 or fewer, and every window stays within half and
        // twice the drift. Measured: 36 to 73 corrections a window for 43.7 of
        // drift at 99 ppm, 11 to 34 for 18.5 at 42 ppm.
        const SECONDS: f64 = 900.0;
        const FROM: f64 = 300.0;

        std::thread::scope(|scope|
        {
            for ppm in [-99_i64, -42, 42, 99]
            {
                scope.spawn(move ||
                {
                    for chunk in [1_920, 960, 240]
                    {
                        let trace = play(&landings(ppm, SECONDS, chunk, 0, &[]), SECONDS);
                        let per_window = 10 * 44_100 / BLOCK_FRAMES;
                        let cap = per_window as f64 / f64::from(CORRECTION_SPACING);
                        let drift = ppm.unsigned_abs() as f64 * 1e-6 * (per_window * BLOCK_FRAMES) as f64;
                        let first = (FROM * 44_100.0) as usize / BLOCK_FRAMES;
                        let counts: Vec<u32> = (first..trace.inserted.len())
                            .collect::<Vec<_>>()
                            .chunks_exact(per_window)
                            .map(|drains| drains.iter().map(|&d| trace.inserted[d] + trace.removed[d]).sum())
                            .collect();
                        let context = std::format!("{ppm} ppm, chunks of {chunk}: {counts:?}");
                        std::println!("{context}");
                        assert!(counts.len() >= 50, "{context}");
                        assert_eq!(trace.underruns.iter().sum::<u32>() + trace.dropped.iter().sum::<u32>(), 0, "{context}");
                        assert!(counts.iter().all(|&count| f64::from(count) < 0.9 * cap), "{context}: a window at the cap");
                        assert!
                        (
                            counts.iter().all(|&count| f64::from(count) >= drift / 2.0 && f64::from(count) <= 2.0 * drift),
                            "{context}: a window off the drift of {drift}"
                        );
                    }
                });
            }
        });
    }

    #[test]
    fn the_correction_constants_hold_together()
    {
        const { assert!(CROSSFADE_FRAMES == 1 << CROSSFADE_SHIFT) };
        const { assert!(CROSSFADE_FRAMES < CORRECTION_SPACING as usize * BLOCK_FRAMES) };
        const { assert!(SINC_ROWS.len() == (1 << (CROSSFADE_SHIFT - SINC_PHASE_SHIFT)) + 1) };
        const { assert!(SINC_LOOKAHEAD * 2 == SINC_TAPS) };
        const { assert!(2 * DEAD_BAND_FRAMES < RING_FRAMES) };
        const { assert!(PREFILL_FRAMES + (LARGEST_CHUNK_FRAMES - BLOCK_FRAMES) / 2 < RING_FRAMES) };
        // The cap of the corrections, in frames per second, clears a 100 ppm
        // drift twice over.
        const { assert!(44_100 * 1_000_000 / (BLOCK_FRAMES as u64 * CORRECTION_SPACING as u64) > 2 * 44_100 * 100) };
        // The estimate moves faster than the corrections at their cap move
        // the fill.
        const { assert!(TRACK_STEP_FRAMES as u64 * CORRECTION_SPACING as u64 > 1 << WINDOW_SHIFT) };
    }

    #[test]
    fn every_gap_trim_and_drop_steps_no_further_than_the_fade_allows_on_a_full_scale_sine_and_square()
    {
        // The gain moves one step in FADE_FRAMES a frame. Between two output
        // frames of a signal whose own step reaches `D`, the output therefore
        // steps by `D + 32_768 / FADE_FRAMES + 1` at the most: the move of the
        // signal, one gain step on a full-scale frame, and the rounding down.
        // A square of constant magnitude keeps `D` out of its magnitude.
        const LENGTH: usize = 140_000;
        let sine = |n: usize| (f64::from(i16::MAX) * (2.0 * core::f64::consts::PI * 441.0 * n as f64 / 44_100.0).sin()).round() as i16;
        let square = |n: usize| if n % 100 < 50 { i16::MAX } else { -i16::MAX };
        let gain_step = 32_768 / FADE_FRAMES as i32 + 1;
        let sine_step = (f64::from(i16::MAX) * 2.0 * core::f64::consts::PI * 441.0 / 44_100.0).ceil() as i32 + 1;

        for (name, input) in [("sine", signal(LENGTH, sine, |n| sine(n).saturating_neg())), ("square", signal(LENGTH, square, square))]
        {
            let mut bridge = Bridge::<RING_FRAMES>::new(PREFILL_FRAMES);
            assert_eq!(bridge.open(JOINT_STEREO_44_1), STEREO);
            let mut output: Vec<u8> = Vec::new();
            let mut fed = 0;
            let mut pcm = [0; BLOCK_FRAMES * PCM_FRAME_BYTES];
            let mut openings = 0;

            // Chunks of 1920 frames, one every 8 drains: a backlog of three
            // trims at the start, a stall of 20 drains runs the ring dry, the
            // backlog after it trims again, and a surplus of two chunks on the
            // started ring drops frames.
            for drain in 0..500_usize
            {
                let chunks = match drain
                {
                    0 | 120 | 400 => 3,
                    100..=119 => 0,
                    _ => usize::from(drain % 8 == 0),
                };

                for _ in 0..chunks
                {
                    let _ = bridge.push(&input[fed * PCM_FRAME_BYTES..(fed + 1_920) * PCM_FRAME_BYTES]);
                    fed += 1_920;
                }

                let waiting = bridge.drift.is_none();
                let written = bridge.drain_into(&mut pcm);
                openings += usize::from(waiting && written > 0);
                pcm[written..].fill(0);
                output.extend_from_slice(&pcm);
            }

            let stats = bridge.take_stats();
            assert!(stats.trimmed > 0 && stats.dropped > 0, "{name}: {stats:?}");
            assert_eq!((stats.underruns, openings, stats.inserted + stats.removed), (1, 2, 0), "{name}: {stats:?}");
            let got = samples_of(&output);

            for (n, pair) in got.windows(2).enumerate()
            {
                for (channel, (&before, &after)) in pair[0].iter().zip(&pair[1]).enumerate()
                {
                    let (before, after) = (i32::from(before), i32::from(after));
                    let step = if name == "sine" { (after - before).abs() } else { (after.abs() - before.abs()).abs() };
                    let bound = if name == "sine" { sine_step + gain_step } else { gain_step };
                    assert!(step <= bound, "{name}: frame {n} channel {channel} steps {before} to {after}, over {bound}");
                }
            }
        }
    }

    /// What a source does just before a drain of a delivery profile.
    #[derive(Debug, Clone, Copy)]
    enum Arrival
    {
        Push(usize),
        Flush,
    }

    /// Returns the drain at `seconds` of link time.
    fn drain_at(seconds: f64) -> u32
    {
        (seconds * 44_100.0 / BLOCK_FRAMES as f64) as u32
    }

    /// Adds chunks of `chunk` frames from drain `from` to drain `to`, at
    /// `percent` of real time, `group` at a time, one group ahead of real
    /// time. Group `k` lands `late[k % late.len()]` drains after it is due.
    fn paced(arrivals: &mut Vec<(u32, Arrival)>, from: u32, to: u32, chunk: usize, percent: usize, group: usize, late: &[u32])
    {
        let mut sent = 0;
        let mut k = 0;

        for drain in from..to
        {
            let due = (drain - from) as usize * BLOCK_FRAMES * percent / 100 + chunk * group;

            while sent + chunk * group <= due
            {
                for _ in 0..group
                {
                    arrivals.push((drain + late[k % late.len()], Arrival::Push(chunk)));
                }

                sent += chunk * group;
                k += 1;
            }
        }
    }

    /// Holds every push due from drain `from` to drain `to` until drain `to`.
    fn stall(arrivals: &mut [(u32, Arrival)], from: u32, to: u32)
    {
        for (drain, arrival) in arrivals.iter_mut()
        {
            if matches!(arrival, Arrival::Push(_)) && (from..to).contains(drain)
            {
                *drain = to;
            }
        }
    }

    /// Underruns, hard drops and play of every drain of a delivery profile.
    struct Replay
    {
        underruns: Vec<u32>,
        dropped: Vec<u32>,
        playing: Vec<bool>,
    }

    /// Plays `arrivals` into a production bridge for `drains` drains. Every push
    /// to a started ring drops exactly what overflows the room left, and a push
    /// to a ring that has not started drops nothing.
    fn replay(mut arrivals: Vec<(u32, Arrival)>, drains: u32) -> Replay
    {
        arrivals.sort_by_key(|&(drain, _)| drain);
        let mut bridge = Bridge::<RING_FRAMES>::new(PREFILL_FRAMES);
        assert_eq!(bridge.open(JOINT_STEREO_44_1), STEREO);
        let bytes = frames(0, LARGEST_CHUNK_FRAMES);
        let mut pcm = [0; BLOCK_FRAMES * PCM_FRAME_BYTES];
        let mut out = Replay { underruns: Vec::new(), dropped: Vec::new(), playing: Vec::new() };
        let mut next = 0;

        for drain in 0..drains
        {
            while let Some(&(_, arrival)) = arrivals.get(next).filter(|&&(at, _)| at <= drain)
            {
                match arrival
                {
                    Arrival::Flush => bridge.flush(),
                    Arrival::Push(count) =>
                    {
                        let overflow = if bridge.drift.is_some() { count.saturating_sub(RING_FRAMES - bridge.ring.len) } else { 0 };
                        assert_eq!(bridge.push(&bytes[..count * PCM_FRAME_BYTES]).dropped, overflow, "drain {drain}");
                    }
                }

                next += 1;
            }

            let _ = bridge.drain_into(&mut pcm);
            let stats = bridge.take_stats();
            out.underruns.push(stats.underruns);
            out.dropped.push(stats.dropped);
            out.playing.push(bridge.drift.is_some());
        }

        out
    }

    /// Checks that the ring ran dry `underruns` times at the most from drain
    /// `from` to drain `to` and dropped nothing there, and that it plays at `to`
    /// unless the episode ends in silence.
    fn assert_episode(replay: &Replay, name: &str, (from, to): (u32, u32), underruns: u32, silent: bool)
    {
        let range = from as usize..to as usize;
        let ran_dry: u32 = replay.underruns[range.clone()].iter().sum();
        let dropped: u32 = replay.dropped[range].iter().sum();
        std::println!("PROFILE {name} drains {from}..{to}: underruns {ran_dry}, dropped {dropped}");
        assert!(ran_dry <= underruns, "{name} from drain {from}: {ran_dry} underruns");
        assert_eq!(dropped, 0, "{name} from drain {from}");
        assert!(silent || replay.playing[to as usize - 1], "{name} from drain {from}: not playing at its end");
    }

    #[test]
    fn delivery_profiles_run_dry_and_drop_only_within_their_derived_bounds()
    {
        let end = drain_at(60.0);

        // A resume short of real time: chunks of 1920 frames, a suspend with a
        // flush 0.6 s after the last chunk, 3 s of pause, 3 s of 512-frame
        // chunks at 70 % of real time, the backlog at twice real time until
        // the source is back on time, then chunks landing 12 and 4 drains
        // apart.
        let mut resumed = Vec::new();
        let resume = drain_at(23.6);
        let owed = drain_at(3.0) as usize * BLOCK_FRAMES * 30 / 100;
        let repaid = resume + drain_at(3.0) + owed.div_ceil(BLOCK_FRAMES) as u32;
        paced(&mut resumed, 0, drain_at(20.0), 1_920, 100, 1, &[0]);
        resumed.push((drain_at(20.6), Arrival::Flush));
        paced(&mut resumed, resume, resume + drain_at(3.0), 512, 70, 1, &[0]);
        paced(&mut resumed, resume + drain_at(3.0), repaid, 1_920, 200, 1, &[0]);
        paced(&mut resumed, repaid, end, 1_920, 100, 1, &[0, 4]);
        let run = replay(resumed, end);
        assert_episode(&run, "short resume, before the suspend", (0, drain_at(20.6)), 1, true);
        // Short of real time, the ring runs dry once per cycle of refill to
        // its level at 70 % and play at a 30 % loss, 2888 / 168 + 2888 / 72 or
        // 57 drains, so 10 times in the 551 drains of the trickle, each a faded
        // dip. Repaying the backlog, the source runs ahead of real time by up
        // to the frames it owes, and the ring drops what overflows it, as the
        // replay checks on every push. Five seconds after the source is back
        // on time, the ring holds it.
        let phase = |from: u32, to: u32| -> (u32, u32)
        {
            let range = from as usize..to as usize;
            (run.underruns[range.clone()].iter().sum(), run.dropped[range].iter().sum())
        };
        let cycle = PREFILL_FRAMES as f64 + 840.0;
        let cycles = (f64::from(drain_at(3.0)) / (cycle / 168.0 + cycle / 72.0)).ceil() as u32;
        let (trickle, _) = phase(resume, resume + drain_at(3.0));
        let (late, _) = phase(resume + drain_at(3.0), end);
        std::println!("PROFILE short resume: {trickle} underruns in the trickle, bound {cycles}, then {late}, dropped {:?}", phase(resume, end).1);
        assert!(trickle <= cycles && late == 0, "{trickle} and {late} underruns");
        assert_episode(&run, "short resume, on time", (repaid + drain_at(5.0), end), 0, false);

        // A steady sender of 1024-frame chunks pauses with a flush and resumes
        // on time: no underrun, and play within 16 drains of the resume.
        let mut steady = Vec::new();
        paced(&mut steady, 0, drain_at(20.0), 1_024, 100, 1, &[0]);
        steady.push((drain_at(20.0), Arrival::Flush));
        paced(&mut steady, drain_at(22.0), drain_at(40.0), 1_024, 100, 1, &[0]);
        let run = replay(steady, drain_at(40.0));
        assert_episode(&run, "steady", (0, drain_at(20.0)), 0, true);
        assert_episode(&run, "steady resume", (drain_at(22.0), drain_at(40.0)), 0, false);
        assert!(run.playing[drain_at(22.0) as usize + 16], "the steady resume waited");

        // A very bursty sender lands three 1024-frame chunks at once, 12.8
        // drains apart. After a pause the ring keeps the gap it measured.
        let mut bursty = Vec::new();
        paced(&mut bursty, 0, drain_at(30.0), 1_024, 100, 3, &[0]);
        bursty.push((drain_at(30.0), Arrival::Flush));
        paced(&mut bursty, drain_at(31.0), end, 1_024, 100, 3, &[0]);
        let run = replay(bursty, end);
        assert_episode(&run, "bursty", (0, drain_at(30.0)), 1, true);
        assert_episode(&run, "bursty resume", (drain_at(31.0), end), 0, false);

        // Back-to-back backlogs: stalls of 200 ms 2 s apart, then two stalls
        // of 30 drains, the second from the drain the first backlog lands.
        let mut backlogs = Vec::new();
        let twin = drain_at(20.0);
        paced(&mut backlogs, 0, drain_at(40.0), 1_920, 100, 1, &[0]);
        stall(&mut backlogs, drain_at(10.0), drain_at(10.2));
        stall(&mut backlogs, drain_at(12.0), drain_at(12.2));
        stall(&mut backlogs, twin, twin + 30);
        stall(&mut backlogs, twin + 31, twin + 61);
        let run = replay(backlogs, drain_at(40.0));
        assert_episode(&run, "backlogs", (drain_at(10.0), drain_at(12.0)), 1, false);
        assert_episode(&run, "backlogs", (drain_at(12.0), twin), 1, false);
        assert_episode(&run, "backlogs", (twin, twin + 31), 1, true);
        assert_episode(&run, "backlogs", (twin + 31, drain_at(40.0)), 1, false);

        // Stalls of 12, 18 and 55 drains far from a start, and one of 18 drains
        // soon after the restart that follows the longest.
        let mut stalls = Vec::new();
        let late = drain_at(40.0) + 100;
        paced(&mut stalls, 0, end, 1_920, 100, 1, &[0]);
        stall(&mut stalls, drain_at(10.0), drain_at(10.0) + 12);
        stall(&mut stalls, drain_at(25.0), drain_at(25.0) + 18);
        stall(&mut stalls, drain_at(40.0), drain_at(40.0) + 55);
        stall(&mut stalls, late, late + 18);
        let run = replay(stalls, end);
        let starts = [drain_at(10.0), drain_at(25.0), drain_at(40.0), late, end];

        for pair in starts.windows(2)
        {
            assert_episode(&run, "stalls", (pair[0], pair[1]), 1, false);
        }

        // A surplus of three chunks on a started ring overflows it, and the
        // replay checks that it drops the overflow and nothing more.
        let mut surplus = Vec::new();
        paced(&mut surplus, 0, drain_at(20.0), 1_920, 100, 1, &[0]);
        surplus.extend([(drain_at(10.0), Arrival::Push(1_920)); 3]);
        let run = replay(surplus, drain_at(20.0));
        assert!(run.dropped.iter().sum::<u32>() > 0, "the surplus dropped nothing");
        assert_eq!(run.underruns.iter().sum::<u32>(), 0);
    }

    #[test]
    fn a_gap_that_runs_the_ring_dry_close_to_its_start_raises_the_level_until_play_wears_it_down()
    {
        let mut pcm = [0; BLOCK_FRAMES * PCM_FRAME_BYTES];
        let block = frames(0, BLOCK_FRAMES);
        let bytes = frames(0, RING_FRAMES);
        let mut bridge = Bridge::<RING_FRAMES>::new(PREFILL_FRAMES);
        assert_eq!(bridge.open(JOINT_STEREO_44_1), STEREO);
        let _ = start_on(&mut bridge, &bytes, PREFILL_FRAMES);

        // The gap that runs the ring dry right after its start counts, up to
        // the drains a full ring holds, and the start level rises to its cap.
        run_dry(&mut bridge);

        for _ in 0..30
        {
            assert_eq!(bridge.drain_into(&mut pcm), 0);
        }

        assert_eq!(bridge.push(&block).accepted, BLOCK_FRAMES);
        assert_eq!(bridge.delivery.longest as usize, RING_FRAMES / BLOCK_FRAMES);
        assert_eq!(bridge.start_level(), RING_FRAMES - START_MARGIN_FRAMES);

        // A source on time restarts the ring once it reaches that level,
        // without any further wait.
        let mut silent = 0;

        while bridge.drain_into(&mut pcm) == 0
        {
            assert_eq!(bridge.push(&block).accepted, BLOCK_FRAMES);
            silent += 1;
        }

        assert_eq!(silent, (RING_FRAMES - START_MARGIN_FRAMES).div_ceil(BLOCK_FRAMES) - 1);

        // Play takes one drain off the gap per GAP_DECAY_DRAINS drains.
        let measured = bridge.delivery.longest;

        for _ in 0..2 * GAP_DECAY_DRAINS
        {
            assert_eq!(bridge.push(&block).accepted, BLOCK_FRAMES);
            assert_eq!(bridge.drain_into(&mut pcm), pcm.len());
        }

        assert_eq!(bridge.delivery.longest, measured - 2);

        // The gap that runs the ring dry that far from its start is a stall
        // the next start does not answer.
        let longest = bridge.delivery.longest;
        run_dry(&mut bridge);

        for _ in 0..10
        {
            assert_eq!(bridge.drain_into(&mut pcm), 0);
        }

        assert_eq!(bridge.push(&block).accepted, BLOCK_FRAMES);
        assert_eq!(bridge.delivery.longest, longest);
    }

    #[test]
    fn the_fade_and_restart_constants_hold_together()
    {
        const { assert!(FADE_FRAMES == 1 << FADE_SHIFT) };
        // Splice regions more than two fades apart fill the slots before they
        // span a full ring and the frame after it.
        const { assert!(SPLICE_SLOTS * (2 * FADE_FRAMES + 1) > RING_FRAMES + 1) };
        // The highest start level is over the base start level of the largest
        // chunk.
        const { assert!(PREFILL_FRAMES + 840 < RING_FRAMES - START_MARGIN_FRAMES) };
        // A gap close to a start comes within 5 s of it.
        const { assert!(CLOSE_DRAINS as usize * BLOCK_FRAMES >= 5 * 44_100) };
    }

    #[test]
    fn a_start_fades_in_over_its_length_and_the_last_frames_before_a_dry_ring_fade_out()
    {
        // A constant signal shows the gain of every frame as it plays.
        const LEVEL: i16 = 16_384;
        let mut pcm = [0; BLOCK_FRAMES * PCM_FRAME_BYTES];
        let mut bridge = Bridge::<RING_FRAMES>::new(PREFILL_FRAMES);
        assert_eq!(bridge.open(JOINT_STEREO_44_1), STEREO);
        let steady = signal(PREFILL_FRAMES + 2 * BLOCK_FRAMES, |_| LEVEL, |_| -LEVEL);
        let mut output = start_on(&mut bridge, &steady, PREFILL_FRAMES);
        push_span(&mut bridge, &steady, PREFILL_FRAMES, BLOCK_FRAMES, BLOCK_FRAMES);

        loop
        {
            let written = bridge.drain_into(&mut pcm);
            output.extend_from_slice(&pcm[..written]);

            if written < pcm.len()
            {
                break;
            }
        }

        let got = samples_of(&output);
        let played = got.len();
        assert_eq!(played, PREFILL_FRAMES + BLOCK_FRAMES);

        for (n, frame) in got.iter().enumerate()
        {
            let gain = (n + 1).min(played - 1 - n).min(FADE_FRAMES) as i32;
            let want = [(i32::from(LEVEL) * gain) >> FADE_SHIFT, (-i32::from(LEVEL) * gain) >> FADE_SHIFT];
            assert_eq!([i32::from(frame[0]), i32::from(frame[1])], want, "frame {n} of {played}");
        }

        // A frame at full gain passes bit for bit, one at half gain halves.
        assert_eq!(faded(frame(0x8001), 256), frame(0x8001));
        assert_eq!(faded([0x00, 0x80, 0xFF, 0x7F], 128), [0x00, 0xC0, 0xFF, 0x3F]);
    }

    #[test]
    fn a_hard_drop_fades_to_silence_on_both_sides_of_the_frames_it_lost()
    {
        // The same drop on a fresh bridge and on one that flushed a playing
        // ring first, with frames of the flushed stream left behind: the fade
        // lands on the frames of the new stream either way.
        const LEVEL: i16 = 16_384;
        let steady = signal(3 * RING_FRAMES, |_| LEVEL, |_| LEVEL);

        for flushed in [false, true]
        {
            let mut pcm = [0; BLOCK_FRAMES * PCM_FRAME_BYTES];
            let mut bridge = Bridge::<RING_FRAMES>::new(PREFILL_FRAMES);
            assert_eq!(bridge.open(JOINT_STEREO_44_1), STEREO);

            if flushed
            {
                let _ = start_on(&mut bridge, &steady, PREFILL_FRAMES);
                push_span(&mut bridge, &steady, 0, BLOCK_FRAMES, BLOCK_FRAMES);
                assert_eq!(bridge.drain_into(&mut pcm), pcm.len());
                assert!(bridge.ring.len > 1_000);
                bridge.flush();
                let _ = play_out_tail(&mut bridge);
            }

            let mut output = start_on(&mut bridge, &steady, PREFILL_FRAMES);
            let room = RING_FRAMES - bridge.ring.len;

            // A push of 300 frames more than the room drops them, and the
            // frames of the next push land after the frames lost. The start
            // trimmed nothing, so output frame `n` is frame `n` of the stream,
            // and the first frame after the drop is frame `PREFILL + room`.
            let span = &steady[..(room + 300) * PCM_FRAME_BYTES];
            assert_eq!(bridge.push(span), Pushed { accepted: room, dropped: 300, trailing_bytes: 0 });
            let at = PREFILL_FRAMES + room;

            for _ in 0..RING_FRAMES / BLOCK_FRAMES + 4
            {
                assert_eq!(bridge.drain_into(&mut pcm), pcm.len());
                output.extend_from_slice(&pcm);
                assert_eq!(bridge.push(&steady[..BLOCK_FRAMES * PCM_FRAME_BYTES]).dropped, 0);
            }

            let got = samples_of(&output);
            let stats = bridge.take_stats();
            assert_eq!((stats.trimmed, stats.inserted, stats.removed), (0, 0, 0));
            assert!(got.len() > at + FADE_FRAMES);

            for (n, frame) in got.iter().enumerate().skip(FADE_FRAMES)
            {
                let distance = if n < at { at - 1 - n } else { n - at };
                let gain = distance.min(FADE_FRAMES) as i32;
                assert_eq!(i32::from(frame[0]), (i32::from(LEVEL) * gain) >> FADE_SHIFT, "flushed {flushed}, frame {n}, splice at {at}");
            }

            assert_eq!(bridge.fade.count, 0, "a played splice stayed");
        }
    }

    #[test]
    fn drops_close_together_fade_as_one_and_a_flush_forgets_the_drops_past_its_tail()
    {
        const LEVEL: i16 = 16_384;
        let steady = signal(3 * RING_FRAMES, |_| LEVEL, |_| LEVEL);
        let gain_of = |frame: &[i16; 2]| i32::from(frame[0]);
        let at_gain = |gain: usize| (i32::from(LEVEL) * gain.min(FADE_FRAMES) as i32) >> FADE_SHIFT;
        let mut pcm = [0; BLOCK_FRAMES * PCM_FRAME_BYTES];

        // Plays `drains` drains fed a block each, appending the output.
        let play = |bridge: &mut Bridge<RING_FRAMES>, output: &mut Vec<u8>, drains: usize|
        {
            let mut pcm = [0; BLOCK_FRAMES * PCM_FRAME_BYTES];

            for _ in 0..drains
            {
                assert_eq!(bridge.drain_into(&mut pcm), pcm.len());
                output.extend_from_slice(&pcm);
                assert_eq!(bridge.push(&steady[..BLOCK_FRAMES * PCM_FRAME_BYTES]).dropped, 0);
            }
        };

        // Overfills a started ring, then drops again 480 frames later. Returns
        // the output so far and the stream frame after each drop.
        let two_drops = |bridge: &mut Bridge<RING_FRAMES>| -> (Vec<u8>, usize, usize)
        {
            let mut pcm = [0; BLOCK_FRAMES * PCM_FRAME_BYTES];
            let mut output = start_on(bridge, &steady, PREFILL_FRAMES);
            let room = RING_FRAMES - bridge.ring.len;
            assert_eq!(bridge.push(&steady[..(room + 300) * PCM_FRAME_BYTES]).dropped, 300);

            for _ in 0..2
            {
                assert_eq!(bridge.drain_into(&mut pcm), pcm.len());
                output.extend_from_slice(&pcm);
            }

            assert_eq!(bridge.push(&steady[..530 * PCM_FRAME_BYTES]).dropped, 50);
            (output, PREFILL_FRAMES + room, PREFILL_FRAMES + room + 480)
        };

        // Two drops 480 frames apart play as one silence from the frame before
        // the first to the frame after the second.
        let mut bridge = Bridge::<RING_FRAMES>::new(PREFILL_FRAMES);
        assert_eq!(bridge.open(JOINT_STEREO_44_1), STEREO);
        let (mut output, first, last) = two_drops(&mut bridge);
        play(&mut bridge, &mut output, RING_FRAMES / BLOCK_FRAMES + 6);
        let got = samples_of(&output);
        assert!(got.len() > last + FADE_FRAMES);

        for (n, frame) in got.iter().enumerate().skip(FADE_FRAMES)
        {
            let distance = if n < first { first - 1 - n } else { n.saturating_sub(last) };
            assert_eq!(gain_of(frame), at_gain(distance), "frame {n}, drops at {first} and {last}");
        }

        // A flush forgets the drops past its tail, whether it comes right after
        // the drop, inside the silence of the two drops, or on the frame before
        // a drop. The next stream fades in and plays at full gain throughout.
        for (case, drains, tail) in [("after the drop", 0, BLOCK_FRAMES), ("inside the silence", 16, 0), ("before a drop", 14, FADE_FRAMES)]
        {
            let mut bridge = Bridge::<RING_FRAMES>::new(PREFILL_FRAMES);
            assert_eq!(bridge.open(JOINT_STEREO_44_1), STEREO);

            if case == "before a drop"
            {
                let _ = start_on(&mut bridge, &steady, PREFILL_FRAMES);
                let room = RING_FRAMES - bridge.ring.len;
                assert_eq!(bridge.push(&steady[..(room + 300) * PCM_FRAME_BYTES]).dropped, 300);

                for _ in 0..drains + 2
                {
                    assert_eq!(bridge.drain_into(&mut pcm), pcm.len());
                }
            }
            else if case == "after the drop"
            {
                let _ = start_on(&mut bridge, &steady, PREFILL_FRAMES);
                let room = RING_FRAMES - bridge.ring.len;
                assert_eq!(bridge.push(&steady[..(room + 300) * PCM_FRAME_BYTES]).dropped, 300);
            }
            else
            {
                let _ = two_drops(&mut bridge);

                for _ in 0..drains
                {
                    assert_eq!(bridge.drain_into(&mut pcm), pcm.len());
                }
            }

            bridge.flush();
            assert_eq!(bridge.tail, tail, "{case}");
            let _ = play_out_tail(&mut bridge);
            let level = bridge.start_level();
            let mut output = start_on(&mut bridge, &steady, level);
            play(&mut bridge, &mut output, RING_FRAMES / BLOCK_FRAMES + 4);

            for (n, frame) in samples_of(&output).iter().enumerate()
            {
                assert_eq!(gain_of(frame), at_gain(n + 1), "{case}: frame {n}");
            }
        }
    }

    #[test]
    fn a_flush_close_or_new_stream_plays_the_oldest_frames_out_to_silence()
    {
        // A full-scale 441 Hz sine steps by 2060 at the most. A cut at its
        // peak would step by 32767. The tail steps by the sine and one gain
        // step, `2060 + 129`, and the frames past the tail never play.
        let sine = |n: usize| (f64::from(i16::MAX) * (2.0 * core::f64::consts::PI * 441.0 * n as f64 / 44_100.0).sin()).round() as i16;
        let input = signal(40_000, sine, |n| sine(n).saturating_neg());
        let bound = (f64::from(i16::MAX) * 2.0 * core::f64::consts::PI * 441.0 / 44_100.0).ceil() as i32 + 1 + 32_768 / FADE_FRAMES as i32 + 1;

        for (index, restart) in ["flush", "close", "open"].into_iter().enumerate()
        {
            let mut bridge = Bridge::<RING_FRAMES>::new(PREFILL_FRAMES);
            assert_eq!(bridge.open(JOINT_STEREO_44_1), STEREO);
            let mut output: Vec<u8> = Vec::new();
            let mut pcm = [0; BLOCK_FRAMES * PCM_FRAME_BYTES];
            let mut fed = 0;
            let cut = 40 + index;

            for drain in 0..60_usize
            {
                if drain == cut
                {
                    assert!(bridge.ring.len > FADE_FRAMES && bridge.fade.gain == 256, "{restart}");
                    match restart
                    {
                        "flush" => bridge.flush(),
                        "close" => bridge.close(),
                        _ => assert_eq!(bridge.open(JOINT_STEREO_44_1), STEREO),
                    }
                    assert_eq!(bridge.tail, FADE_FRAMES, "{restart}");
                }

                // A push after a flush waits behind the tail.
                if drain % 8 == 0
                {
                    let _ = bridge.push(&input[fed * PCM_FRAME_BYTES..(fed + 1_920) * PCM_FRAME_BYTES]);
                    fed += 1_920;
                }

                let written = bridge.drain_into(&mut pcm);
                pcm[written..].fill(0);
                output.extend_from_slice(&pcm);

                if drain == cut
                {
                    assert_eq!(written, pcm.len(), "{restart}");
                }
                else if drain == cut + 1
                {
                    assert_eq!((written, bridge.tail), ((FADE_FRAMES - BLOCK_FRAMES) * PCM_FRAME_BYTES, 0), "{restart}");
                }
            }

            let got = samples_of(&output);

            for (n, pair) in got.windows(2).enumerate()
            {
                for (channel, (&before, &after)) in pair[0].iter().zip(&pair[1]).enumerate()
                {
                    let step = (i32::from(after) - i32::from(before)).abs();
                    assert!(step <= bound, "{restart}: frame {n} channel {channel} steps {before} to {after}, over {bound}");
                }
            }

            // Past the tail, only a new start plays again.
            let after_tail = (cut + 2) * BLOCK_FRAMES;
            let quiet_to = if restart == "close" { got.len() } else { after_tail + 4 * BLOCK_FRAMES };
            assert!(got[after_tail..quiet_to.min(got.len())].iter().all(|frame| *frame == [0, 0]), "{restart}");
        }
    }

}
