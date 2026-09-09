#!/usr/bin/env bash
# Reads the mute path back out of the linked processing board image.
#
# The loudspeaker carries no analog filter and no analog mute, so the store that
# drives XSMT low is the only thing between a fault and the drivers. The source
# of pulsar_dsp claims six properties which neither a test nor the compiler
# checks:
#
#   1. every fault and interrupt vector points at a handler that mutes,
#   2. the mute store is the first instruction of the routine that carries it
#      that is not a frame push, a register move or an immediate constant,
#   3. the interrupt mask and its barrier follow that store immediately,
#   4. the post-mortem record is 36 bytes, lies inside .uninit, and falls
#      outside the range the startup zero fill walks,
#   5. once the routine that carries the mute has built the record address in a
#      register, it makes nine word stores through that register, at the nine
#      record offsets, in rising offset order,
#   6. each of the three start-up guards writes its own cause into the refusal
#      word, and parks on the instruction after that write.
#
# Each claim is checked here against the disassembly of the image that ships.
#
# Claim 1 turns every new handler into a decision. A vector that stops pointing
# at the hard fault handler or at the default handler trips this gate, and the
# way through it is to say what that handler does about the drivers, not to add
# its address to the allowed set.
#
# Claim 2 reads every instruction between the vector and the mute store. A
# handler that only forwards to the shared mute holds no store of its own, so
# the entry frame is read and the call it makes is followed one level down. One
# level is the whole descent: both vectors of this image reach the mute in one
# call, and a path that takes more frames is a path this gate has not read.
#
# What a frame may run ahead of the mute is a whitelist, BARE_INSTRUCTION, and
# it applies to the entry frame as much as to the routine under it. A blacklist
# of memory access mnemonics cannot be finished: strb, strd, stm, stmdb, ldmia,
# ldrex, strex, ldrsb, ldrsh, vldr, vstr, push, pop, tbb and tbh all reach
# memory, and a handler that stores to a peripheral before it forwards can
# raise XSMT as easily as lower it. One memory access is exempt, and it is
# named rather than waived: the ICSR read of the trampoline cortex-m-rt puts in
# front of the default handler, matched as the three instruction sequence it
# is, at that one address. The frame around it faces the whitelist like any
# other.
#
# Claims 4 and 5 exist because the record is written and never read, so nothing
# else notices when it stops being written or starts being erased.
#
# Claim 4 resolves the two literals the startup zero fill loop actually loads,
# rather than looking for a value that could be any of several symbols. It fails
# on a memory.x that drags __ebss past the record, which the SECTIONS comment of
# that file describes, on a placement that leaves .uninit, and on the
# zero-init-ram feature of cortex-m-rt, which swaps the bounds of that loop for
# _ram_start and _ram_end and so walks over the record.
#
# Which loop that is has to be read, not assumed, and two different readings
# separate it from the two other loops cortex-m-rt can emit ahead of main.
#
# The SHAPE separates it from the .data copy. Both walk a pointer up to a limit
# and store through a post-incrementing stm, but the copy loads each word first,
# so its body is ldm then stm and the loop runs five instructions where the zero
# fill runs four. The copy is refused on that count before any value is read,
# and taking the first loop of the shape, which is what position does, is what
# reads the copy as the zero fill the day the two are emitted the other way
# round.
#
# The VALUE separates it from paint-stack, which is the only other four
# instruction loop of that shape and the only place the shape alone decides
# nothing. It stores 0xcccccccc where the zero fill stores 0, so the loop is
# picked by the value its store carries.
#
# Reading that value has three outcomes and not two. A value read as something
# other than zero paints memory rather than clearing it, and putting that loop
# aside is a reading. A value the walk cannot read is not a reading, and putting
# that loop aside would rule out a loop whose bounds this gate has never seen,
# so it turns the claim red. Two loops carrying zero, or none, turn it red the
# same way. Nothing here falls back on position.
#
# Claim 5 fails when the nine stores stop being nine separate word stores made
# in rising offset order. Dropping the volatile qualifier does exactly that: the
# compiler then merges neighbouring words into strd pairs and reorders them,
# which costs the property the order carries, that the magic lands first and the
# checksum last so an interrupted write fails validation rather than reading as
# a record.
#
# Rising line numbers are the order the compiler emitted those stores in, not
# the order they run, and the two agree only on a straight line. So the run from
# the completed record address to the last store is required to be one: every
# instruction in it hands control to the next, and none of them is entered from
# elsewhere. Without that, a store the routine reaches by a branch, or skips by
# one, reads here as a store it always makes.
#
# Claim 5 reads a shape, and three things it does not read are worth naming. It
# cannot see a volatile qualifier, so it catches the code shape a lost one
# produces rather than the loss itself. It matches the nine offsets and the
# order, never the value stored, so which status register reaches which record
# word is checked by nothing here and by no host test. And it counts str and
# str.w through one base register, so a strd, strb, stm, register offset or
# pre-indexed store, or any store reaching the record through another register,
# is invisible to it.
#
# Claim 6 covers the one thing claim 5 names as unread, the value stored, on the
# one seam where nothing else reads it. The three start-up guards carry one
# domain and hand the same type to the same tagging function, so which guard
# writes which cause is checked by no host test either: pulsar_dsp is a [[bin]],
# and no host crosses that seam. Swapping two causes between two guards leaves
# the compiler, clippy, every host test and claims 1 to 5 green, and sends a
# person reading the record at a probe to the wrong guard. So this claim reads
# the value: it resolves the immediate each guard builds ahead of its store into
# the refusal word and requires the three, in the order the guards run, to be
# the three start-up codes.
#
# This gate has only ever been EXTENDED. Claim 6 stands over the five above,
# none of which is weakened to admit it, and no claim here has been loosened to
# let a change through. When a gate has to be widened before a change fits, the
# change is what needs reading, not the gate.
#
# Claim 6 finds the arms by the call, not by the value: every call the entry
# function makes to the routine that carries the mute has to be a word store
# away from it, and that store is the arm writing its refusal word. It then
# reads the store backwards over the instructions that build the stored
# register. A movw, a mov or a movs ends that walk with a value, a movt or a two
# operand add carries it on, and anything else naming the register ends the walk
# with nothing.
#
# That walk stops at every control flow boundary rather than reading through
# one, and the two boundaries stop it for two different reasons. A call DESTROYS
# the value: AAPCS leaves r0 to r3, ip and lr unpreserved across one, so after a
# bl the register holds what the callee returned and not what a move ahead of it
# put there, and the bl names none of the six. A branch makes the path
# UNCERTAIN: what stands textually ahead of one is not what ran ahead of it.
#
# So what the walk may cross is a whitelist, WALKABLE_INSTRUCTION, of forms that
# fall through to the next instruction and write no general register they do not
# name. Several of them write the condition flags without naming them, which is
# outside what any walk here tracks: a walk follows one general register from a
# store back to the instruction that built it, and no claim reads a flag.
# A list of call and branch mnemonics to refuse cannot be finished, and this one
# would have to hold bl, blx, b, bx, cbz, cbnz, tbb, tbh and every conditional
# branch of the fifteen condition codes at two widths. A mnemonic missing from a
# whitelist ends a walk with nothing and turns this claim red, and a mnemonic
# missing from a blacklist is crossed in silence.
#
# A block boundary is neither an instruction nor a mnemonic, and llvm-objdump
# marks none in the stream, so the walk reads the boundaries off the operands:
# every address the function names is taken as a place control can arrive at,
# and the walk refuses to move past one. That names more addresses than the
# branches alone do, which ends a walk early rather than letting one through.
#
# Where a walk may BEGIN is a separate question from what it may cross, and each
# caller answers it. Refusing to move past a boundary says nothing about the
# instruction the walk starts on, so a start that is itself entered from
# elsewhere is reached on a path the walk never read, and the value it resolves
# belongs to whichever predecessor happens to sit above. Claim 6 starts on the
# store of an arm and requires that nothing names it. Claim 4 starts on the
# compare of a loop, which its own back branch always names, so it counts the
# namers and requires exactly one. Neither is a stronger reading than the other,
# they are the same reading of two different shapes.
#
# Claims 4 and 5 read the same way, over their own function: one walk finds the
# loop bounds of the startup zero fill and the zero it stores, the other
# requires the record stores to stand on one line. Three claims of this gate
# therefore rest on one reading of what a line of disassembly lets a reader
# assume about the line above it, and it is written once.
#
# So a word this claim reports was built by the instructions between it and its
# store, on the one path that reaches that store, and an arm whose word it
# cannot read is counted as unread rather than guessed at.
#
# Where that store lands is read apart from the walk. The base register has to
# be one the entry function loads with a single address, and that address plus
# the offset of the store has to be the refusal word. A base the function loads
# two ways is refused rather than resolved, and neither the address nor the
# offset is taken on trust from the arm.
#
# Three things claim 6 does not read. It matches the order the arms are emitted
# in, never which guard branches to which block, so a compiler that reorders the
# cold blocks turns it red and the path gets read. Nothing here proves the base
# register still holds that address where the store runs, only that the function
# builds no other address in it. And an arm whose word is not built from
# immediates is unread: the clock arm, the transport arm and the release arm
# compute theirs from the fault they carry, so no claim here says what value any
# of the three writes. Where each of them writes it is read on the terms above,
# the same ones a guard arm faces.
#
# Both counts are read, which is what makes an arm added to main a decision
# rather than a silence. A guard whose word is immediate joins STARTUP_REFUSALS
# or the read count is wrong, and a stage that computes its word joins
# COMPUTED_REFUSALS or the unread count is. Reading only the first count leaves
# a stage free to park with an unread word and be named nowhere.
#
# What no claim here covers is the stack. This gate reads instructions and says
# nothing about the stack pointer they run on. A handler entered on a corrupt
# one faults on its own frame push, ahead of every instruction inspected below,
# and a green gate says nothing about that case.
#
# Usage: scripts/mute-gate.sh

set -euo pipefail
cd "$(dirname "$0")/.."

CRATE=pulsar_dsp
TARGET=thumbv7em-none-eabihf
BIN="$CRATE/target/$TARGET/release/$CRATE"

# RM0433 Rev 7 section 12.4. GPIOE sits at 0x5802_1000, BSRR at offset 0x18,
# and the reset half of BSRR starts at bit 16, so clearing PE7 is 0x80_0000.
GPIOE_BASE_LOW='#0x1000'
GPIOE_BASE_HIGH='#0x5802'
BSRR_OFFSET='#0x18'
BR7_MASK='#0x800000'

# Words 0 and 1 of the vector table are the initial stack pointer and the reset
# vector. Everything past them is a fault or an interrupt.
FIRST_HANDLER_WORD=2

# Cortex-M7 exceptions plus the H743 interrupt lines. A parse that returns far
# fewer words than this has read the wrong section.
MIN_HANDLER_WORDS=100

# Shape of the post-mortem record, owned by pulsar_lib::postmortem. Nine words,
# magic first, checksum last.
RECORD_SYMBOL=FAULT_RECORD
RECORD_SECTION=.uninit
RECORD_WORDS=9
RECORD_BYTES=$((RECORD_WORDS * 4))

# Word the start-up guards of main write their refusal into, owned by
# pulsar_dsp. It is a private static, so the name a build keeps is the demangled
# one and not the mangled one, whose hash moves with the crate metadata.
REFUSAL_SYMBOL='pulsar_dsp::REFUSAL'

# Refusal word each start-up guard writes, in the order the guards run, and the
# guard each belongs to. The high half is the domain
# pulsar_lib::postmortem::RefusalDomain gives the start-up, and the low half the
# StartupFault discriminant of that guard.
STARTUP_REFUSALS=(
    "00010001 the core peripheral handle guard"
    "00010003 the BusFault arming read-back"
    "00010002 the device peripheral handle guard"
)

# Arms of main whose refusal word this gate does not read, in the order they are
# emitted. Each builds its word out of the fault it carries, so the backward walk
# ends with nothing rather than a value. Where the store lands is read all the
# same: an unread arm faces the base register and offset check a read one faces,
# so a computed word still has to reach the refusal word and nowhere else. What
# is checked on top of that is that there are this many of them: a stage that
# parks with a computed word and is named on no line below turns claim 6 red, so
# a new one is read by a person before it ships.
COMPUTED_REFUSALS=(
    "the audio clock bring-up"
    "the output transport bring-up"
    "the converter mute release gate"
)

# Instructions a frame may run ahead of the mute store, or ahead of the call
# that reaches it: the frame push, a register move, an immediate constant, a
# call. Anything else fails the gate, whatever it does, which is what makes the
# REFUSAL complete where a list of memory access mnemonics to refuse never is.
# What the list admits is not read that far. A call is on it, and only the call
# an entry frame makes is followed, so a bl standing in the routine that carries
# the mute, ahead of its store, passes here with its callee unread.
BARE_INSTRUCTION='^(push \{[^}]*\}|movs?(w|t|\.w)? [^,]+, [^,]+|bl 0x[0-9a-f]+( <[^>]*>)?|nop)$'

# Instructions the backward walk of park_arms may cross. Every form here writes
# only the registers it names and falls through to the one after it, so crossing
# one leaves the register a store reads untouched and the path straight. A
# whitelist for the same reason BARE_INSTRUCTION is one: a mnemonic missing here
# ends a walk with nothing and turns claim 6 red, where a mnemonic missing from
# a list of calls and branches to refuse is crossed in silence.
WALK_MNEMONIC='movw|movt|mov|mvn|addw|add|adc|subw|sub|sbc|rsb'
WALK_MNEMONIC="$WALK_MNEMONIC|and|orn|orr|eor|bic|lsl|lsr|asr|ror"
WALK_MNEMONIC="$WALK_MNEMONIC|cmp|cmn|tst|teq|clz|rev16|revsh|rev"
WALK_MNEMONIC="$WALK_MNEMONIC|uxtb|uxth|sxtb|sxth|ubfx|sbfx|bfi|bfc"
WALK_MNEMONIC="$WALK_MNEMONIC|mul|mla|mls|umull|smull|udiv|sdiv"
WALK_MNEMONIC="$WALK_MNEMONIC|ldrsb|ldrsh|ldrb|ldrh|ldrd|ldr"
WALK_MNEMONIC="$WALK_MNEMONIC|strb|strh|strd|str|mrs|msr"

# Condition codes the assembler writes between a mnemonic and its width suffix.
# A conditional data processing form still writes only what it names.
WALK_CONDITION='(eq|ne|cs|hs|cc|lo|mi|pl|vs|vc|hi|ls|ge|lt|gt|le)?'

WALKABLE_INSTRUCTION="^(($WALK_MNEMONIC)s?$WALK_CONDITION(\\.w)? |nop\$)"

# The one memory access exempt from that list. PM0253 section 4.3.3 puts ICSR
# at 0xE000ED04, and the trampoline cortex-m-rt places in front of the default
# handler reads it to recover the interrupt number it hands on.
ICSR_LOW='#0xed04'
ICSR_HIGH='#0xe000'

echo "==== building $CRATE (release) ===="
(cd "$CRATE" && cargo build --release --locked)

llvm_bin="$(cd "$CRATE" && rustc --print target-libdir)/../bin"
objdump="$llvm_bin/llvm-objdump"
nm="$llvm_bin/llvm-nm"

if [ ! -x "$objdump" ] || [ ! -x "$nm" ]
then
    echo "ERROR: the pinned toolchain has no llvm-tools" >&2
    echo "Install it with: rustup component add llvm-tools" >&2
    exit 1
fi

if [ ! -f "$BIN" ]
then
    echo "ERROR: $BIN was not produced" >&2
    exit 1
fi

syms="$("$nm" --demangle "$BIN")"
sized_syms="$("$nm" --demangle --print-size "$BIN")"
sections="$("$objdump" -h "$BIN")"
disasm="$("$objdump" -d --no-show-raw-insn "$BIN")"

# Prints the 8 digit load address of one symbol, or nothing when it is absent.
sym_addr()
{
    awk -v want="$1" '$3 == want { print $1; exit }' <<< "$syms"
}

# Prints the hexadecimal size of one symbol, or nothing when it carries none.
# The awk here stays POSIX: the runner does not always provide gawk, and
# strtonum is a gawk extension. Every hexadecimal value is converted in bash.
sym_size()
{
    awk -v want="$1" '$4 == want { print $2; exit }' <<< "$sized_syms"
}

# Prints the hexadecimal load address and size of one output section.
section_bounds()
{
    awk -v want="$1" '$2 == want { print $4, $3; exit }' <<< "$sections"
}

# Prints the instructions of the function at one address, one per line, with the
# address column, the tabs and the objdump comments stripped.
body()
{
    awk -v want="$1" '
        /^[0-9a-f]+ <.*>:$/ { inside = ($1 == want); next }
        !inside { next }
        /^[[:space:]]*[0-9a-f]+:/ {
            sub(/^[^\t]*\t/, "")
            sub(/[[:space:]]*@ .*$/, "")
            gsub(/[[:space:]]+/, " ")
            sub(/^ /, "")
            sub(/ $/, "")
            print
            next
        }
        { inside = 0 }
    ' <<< "$disasm"
}

# Prints the instructions of the function at one address, each behind its own
# load address, with the objdump annotation kept. A pc relative load therefore
# still names the literal it reads, and a literal pool entry still shows its
# value, which is what resolving the startup zero fill needs.
annotated_body()
{
    awk -v want="$1" '
        /^[0-9a-f]+ <.*>:$/ { inside = ($1 == want); next }
        !inside { next }
        /^[[:space:]]*[0-9a-f]+:/ {
            at = $1
            sub(/^[^\t]*\t/, "")
            gsub(/[[:space:]]+/, " ")
            sub(/^ /, "")
            sub(/ $/, "")
            print at " " $0
            next
        }
        { inside = 0 }
    ' <<< "$disasm"
}

# Reads one annotated function body into the four forms the backward walk needs.
# WALK_INSN holds the instruction of each line, WALK_NOTE the objdump annotation
# behind it, WALK_SPOT its load address, and WALK_ENTERED how many times the
# function names each address in an operand.
#
# A block boundary is neither an instruction nor a mnemonic, and llvm-objdump
# marks none in the stream, so WALK_ENTERED is what stands in for one: every
# address the function names is taken as a place control can arrive at. That
# names more addresses than the branches alone do, which ends a walk early
# rather than letting one through.
#
# It is a count and not a set because one namer and two are different answers
# where the walk starts on a branch target on purpose. A loop header is named by
# its own back branch, so asking whether it is named at all says nothing there,
# and asking how many name it says whether anything else reaches it.
read_walk()
{
    local -a raw
    local index line key

    unset WALK_INSN WALK_NOTE WALK_SPOT WALK_ENTERED
    declare -ga WALK_INSN WALK_NOTE WALK_SPOT
    declare -gA WALK_ENTERED

    mapfile -t raw <<< "$1"

    for index in "${!raw[@]}"
    do
        line="${raw[index]}"
        printf -v key '%x' "$((16#${line%%:*}))"
        WALK_SPOT[index]="$key"
        line="${line#* }"
        WALK_INSN[index]="${line%% @ *}"
        WALK_NOTE[index]=""
        if [[ "$line" == *" @ "* ]]
        then
            WALK_NOTE[index]="${line#* @ }"
        fi
    done

    for index in "${!WALK_INSN[@]}"
    do
        line="${WALK_INSN[index]}"
        while [[ "$line" =~ 0x[0-9a-f]+ ]]
        do
            printf -v key '%x' "$((BASH_REMATCH[0]))"
            WALK_ENTERED["$key"]=$((${WALK_ENTERED["$key"]-0} + 1))
            line="${line#*"${BASH_REMATCH[0]}"}"
        done
    done
}

# Succeeds when the instruction at one index hands control to the one after it
# and writes no general register it does not name. That is the whole of what a
# reader of this disassembly may assume about a line it did not stop on, and the
# two halves of it fail for two different reasons. The condition flags are
# outside it: several whitelisted forms write them without naming them, and no
# walk here follows anything but one general register.
#
# A call DESTROYS a value: AAPCS leaves r0 to r3, ip and lr unpreserved across
# one, so after a bl a register holds what the callee returned and not what a
# move ahead of it put there, and the bl names none of the six. A branch makes
# the path UNCERTAIN: what stands textually ahead of one is not what ran ahead
# of it.
#
# So the form has to be on the WALKABLE_INSTRUCTION whitelist. One thing that
# list alone lets through is a write to pc, which is a branch wearing an
# ordinary mnemonic, so a destination of pc is refused on its own.
falls_through()
{
    [[ "${WALK_INSN[$1]-}" =~ $WALKABLE_INSTRUCTION ]] \
        && [[ ! "${WALK_INSN[$1]-}" =~ ^[a-z0-9.]+\ pc(,|$) ]]
}

# Succeeds when the function names the address of the instruction at one index,
# which is where control can arrive from somewhere other than the line above it.
enters_here()
{
    [ -n "${WALK_ENTERED[${WALK_SPOT[$1]-}]-}" ]
}

# Prints the index of the last instruction ahead of one index that names one
# register, or nothing when a control flow boundary stands in the way first.
# read_walk fills the arrays it reads.
walk_back_to()
{
    local src=$1
    local at=$(($2 - 1))
    local padded

    while [ "$at" -ge 0 ]
    do
        padded=" ${WALK_INSN[at]//[,\[\]\{\}]/ } "

        if [[ "$padded" == *" $src "* ]]
        then
            printf '%s' "$at"
            return 0
        fi

        if ! falls_through "$at" || enters_here "$at"
        then
            return 1
        fi

        at=$((at - 1))
    done

    return 1
}

# Succeeds when control reaches the instruction at the second index from the one
# at the first without leaving the straight line: everything between them hands
# control on, and nothing past the first is entered from elsewhere.
straight_line()
{
    local at

    for ((at = $1; at < $2; at++))
    do
        if ! falls_through "$at" || enters_here "$((at + 1))"
        then
            return 1
        fi
    done
}

# Prints the immediate one register holds where the instruction at one index
# runs, or nothing when the walk cannot read it.
#
# A movw, a mov or a movs ends the walk with a value, a movt or a two operand
# add carries it on, and anything else naming the register ends it with nothing.
# So a value printed here was built by the instructions between it and that
# index, on the one path that reaches it, and by nothing else.
walk_immediate()
{
    local src=$1
    local at=$2
    local -a chain
    local line value step part

    chain=()

    while at="$(walk_back_to "$src" "$at")"
    do
        line="${WALK_INSN[at]}"

        if [[ "$line" =~ ^(movw|movs|mov|mov\.w)\ $src,\ \#(0x[0-9a-f]+)$ ]]
        then
            value=$((${BASH_REMATCH[2]}))

            for step in ${chain[@]+"${chain[@]}"}
            do
                if [[ "$step" =~ ^movt\ $src,\ \#(0x[0-9a-f]+)$ ]]
                then
                    part=$((${BASH_REMATCH[1]}))
                    value=$(((value & 0xFFFF) | (part << 16)))
                elif [[ "$step" =~ ^adds?\ $src,\ \#(0x[0-9a-f]+)$ ]]
                then
                    part=$((${BASH_REMATCH[1]}))
                    value=$((value + part))
                fi
            done

            printf '%08x' "$value"
            return 0
        fi

        if [[ ! "$line" =~ ^(movt|adds?)\ $src,\ \#(0x[0-9a-f]+)$ ]] \
            || [ -n "${WALK_ENTERED[${WALK_SPOT[at]}]-}" ]
        then
            return 1
        fi

        chain=("$line" ${chain[@]+"${chain[@]}"})
    done

    return 1
}

# Prints the pointer, the limit and the stored register of the cortex-m-rt
# startup loop whose compare stands at one index, or nothing when no loop stands
# there.
#
# The four instructions are required in order and both branch targets have to
# close the loop, so the whole body is read rather than sampled: the compare,
# the exit branch over the store, the store, and the branch back to the compare.
# That pins the three registers, and it proves the body writes neither the limit
# nor the stored register.
#
# The shape is all this reads. Whether the compare is reached from anywhere but
# the line above it is the caller's question, because the two answers to it are
# not the two answers to this one: a shape that does not match means no loop
# stands here, and a loop whose header something else names is a loop that
# stands here and cannot be read. Answering both with the same silence is what
# lets one of them disappear.
startup_loop()
{
    local at=$1
    local pointer limit stored exit_at back_at

    if [[ ! "${WALK_INSN[at + 2]-}" =~ ^stm\ (r[0-9]+)!,\ \{(r[0-9]+)\}$ ]] \
        || [ -z "${WALK_SPOT[at + 4]-}" ]
    then
        return 1
    fi
    pointer="${BASH_REMATCH[1]}"
    stored="${BASH_REMATCH[2]}"

    # The compare is symmetric, so either operand may be the pointer.
    if [[ "${WALK_INSN[at]-}" =~ ^cmp\ (r[0-9]+),\ ${pointer}$ ]] \
        || [[ "${WALK_INSN[at]-}" =~ ^cmp\ ${pointer},\ (r[0-9]+)$ ]]
    then
        limit="${BASH_REMATCH[1]}"
    else
        return 1
    fi

    if [[ ! "${WALK_INSN[at + 1]-}" =~ ^beq(\.w)?\ 0x([0-9a-f]+)( |$) ]]
    then
        return 1
    fi
    printf -v exit_at '%x' "$((16#${BASH_REMATCH[2]}))"

    if [[ ! "${WALK_INSN[at + 3]-}" =~ ^b(\.w)?\ 0x([0-9a-f]+)( |$) ]]
    then
        return 1
    fi
    printf -v back_at '%x' "$((16#${BASH_REMATCH[2]}))"

    if [ "$exit_at" != "${WALK_SPOT[at + 4]}" ] \
        || [ "$back_at" != "${WALK_SPOT[at]}" ]
    then
        return 1
    fi

    printf '%s %s %s' "$pointer" "$limit" "$stored"
}

# Prints the low and the high bound of the startup zero fill, read out of the
# loop that performs it.
#
# The zero fill is not simply the first loop a search finds, and two readings
# separate it from the other loops cortex-m-rt emits ahead of main. startup_loop
# takes the shape: the .data copy loads each word before storing it, so it runs
# five instructions where this loop runs four, and it is refused on that count.
# The value the store carries takes the rest: paint-stack wears the same four
# instruction shape and stores 0xcccccccc, where the zero fill stores 0.
#
# The classification fails closed on every uncertainty. No loop carrying zero,
# two carrying it, or one of the shape whose stored value the walk cannot read
# at all, and this comes back with nothing so claim 4 turns red and a person
# reads it. It never falls back on position. The unreadable case is the one that
# costs a build: paint-stack materialises its value with an ldr of a literal,
# which the assembler renders as a move only while 0xcccccccc stays an encodable
# immediate, so turning that feature on can turn this claim red rather than let
# it pick between two loops it has not both read.
#
# Which symbols the two literals came from does not matter here, and must not:
# the bounds move with the build, and a value compared by name matches whatever
# else happens to share it.
zero_fill_bounds()
{
    local reset lines index registers found
    local at pointer limit stored painted lo_at hi_at lo hi
    # objdump names the literal a pc relative load reads in its annotation.
    local pool='\[pc,\ \#0x[0-9a-f]+\]$'

    reset="$(sym_addr Reset)"
    if [ -z "$reset" ]
    then
        return 1
    fi
    lines="$(annotated_body "$reset")"
    if [ -z "$lines" ]
    then
        return 1
    fi
    read_walk "$lines"

    found=""
    for index in "${!WALK_INSN[@]}"
    do
        if ! registers="$(startup_loop "$index")"
        then
            continue
        fi
        read -r pointer limit stored <<< "$registers"

        # The walk below starts on this compare, which its own back branch
        # names, so one namer is what says no other block reaches it and the
        # setup on the line above is what every pass sees. A second namer is a
        # loop of the shape this walk cannot read, not a loop of another kind,
        # so it ends the classification here rather than dropping out of the
        # count the way a painting loop does.
        if [ "${WALK_ENTERED[${WALK_SPOT[index]}]-0}" -ne 1 ]
        then
            return 1
        fi

        # The setup reaches the compare in a straight line, so what it leaves in
        # the stored register is what the first pass of the loop stores, and the
        # loop body writes that register on no pass.
        #
        # Three outcomes, not two. A loop that stores a value this walk reads as
        # something other than zero paints memory rather than clearing it, and
        # dropping it is a reading. A loop whose value the walk cannot read at
        # all is not a reading, and dropping it would rule out a loop this gate
        # has never seen the bounds of, so it ends the classification instead.
        if ! painted="$(walk_immediate "$stored" "$index")"
        then
            return 1
        fi

        if [ "$painted" != "00000000" ]
        then
            continue
        fi

        if [ -n "$found" ]
        then
            return 1
        fi
        found="$index $pointer $limit"
    done

    if [ -z "$found" ]
    then
        return 1
    fi
    read -r index pointer limit <<< "$found"

    # Both bounds come from the literal pool, and objdump names the entry each
    # load reads. The loads are found by the same walk, so a bound built on a
    # path this gate has not read is refused rather than resolved.
    if ! at="$(walk_back_to "$pointer" "$index")" \
        || [[ ! "${WALK_INSN[at]}" =~ ^ldr\ ${pointer},\ $pool ]] \
        || [[ ! "${WALK_NOTE[at]}" =~ ^0x([0-9a-f]+) ]]
    then
        return 1
    fi
    lo_at="${BASH_REMATCH[1]}"

    if ! at="$(walk_back_to "$limit" "$index")" \
        || [[ ! "${WALK_INSN[at]}" =~ ^ldr\ ${limit},\ $pool ]] \
        || [[ ! "${WALK_NOTE[at]}" =~ ^0x([0-9a-f]+) ]]
    then
        return 1
    fi
    hi_at="${BASH_REMATCH[1]}"

    lo="$(sed -nE "s/^${lo_at}: \.word 0x([0-9a-f]+)\$/\1/p" <<< "$lines")"
    hi="$(sed -nE "s/^${hi_at}: \.word 0x([0-9a-f]+)\$/\1/p" <<< "$lines")"
    if [ -z "$lo" ] || [ -z "$hi" ]
    then
        return 1
    fi

    printf '%s %s\n' "$lo" "$hi"
}

# Drops the three instruction ICSR read of the default handler trampoline, and
# nothing else. The three have to stand together, in this order, through one
# register, at the ICSR address. A read that differs anywhere stays in the
# stream and faces BARE_INSTRUCTION.
without_icsr_read()
{
    awk -v low="$ICSR_LOW" -v high="$ICSR_HIGH" '
        { line[NR] = $0 }
        END {
            for (i = 1; i <= NR; i++)
            {
                reg = ""
                if (line[i] ~ ("^movw r[0-9]+, " low "$"))
                {
                    split(line[i], field, " ")
                    reg = substr(field[2], 1, length(field[2]) - 1)
                }
                if (reg != "" && i + 2 <= NR \
                    && line[i + 1] == "movt " reg ", " high \
                    && line[i + 2] == "ldr " reg ", [" reg "]")
                {
                    i += 2
                    continue
                }
                print line[i]
            }
        }
    ' <<< "$1"
}

# Prints the line number of the first instruction of a frame that is not a bare
# one, or nothing when the frame runs only bare instructions.
first_foreign()
{
    grep -nvE "$BARE_INSTRUCTION" <<< "$1" | head -1 | cut -d: -f1
}

# Prints the instructions that carry the mute for the handler at one address,
# through the body printer named as the first argument. Which frame carries the
# mute is decided on the plain body either way, so the two printers always land
# on the same frame.
#
# The entry frame is read first, and it is read, not skipped: everything it runs
# up to its call has to be bare, so a handler that reaches memory on its way
# down fails here rather than passing unseen. A frame that is bare all the way
# to its call holds nothing to read, so the routine it calls is the one that
# carries the mute, and the descent stops there. Anything else is handed back as
# the mute routine, where the first instruction that is not bare has to be the
# mute store.
#
# Only a bl is followed. A frame that reaches the shared routine by branching
# rather than calling is handed back as the mute routine and fails on its own
# branch, which is the safe way round: the gate goes red and the path gets read.
mute_body()
{
    local printer=$1 addr=$2
    local lines call_at callee carrier

    lines="$(body "$addr")"
    if [ -z "$lines" ]
    then
        echo "FAIL: the frame at 0x$addr has no body in the disassembly" >&2
        return 1
    fi

    carrier="$addr"
    call_at="$(grep -nE '^bl 0x[0-9a-f]+' <<< "$lines" | head -1 | cut -d: -f1)"

    if [ -n "$call_at" ] \
        && [ -z "$(first_foreign \
            "$(without_icsr_read "$(sed -n "1,${call_at}p" <<< "$lines")")")" ]
    then
        callee="$(sed -n "${call_at}p" <<< "$lines" | cut -d' ' -f2)"
        carrier="$(printf '%08x' "$callee")"
        if [ -z "$(body "$carrier")" ]
        then
            echo "FAIL: the frame at 0x$addr forwards to $callee, which has no" \
                "body in the disassembly" >&2
            return 1
        fi
    fi

    "$printer" "$carrier"
}

fail()
{
    echo "FAIL: $*" >&2
    return 1
}

# Prints the address the frame at one address calls, or nothing when that frame
# runs anything but bare instructions ahead of its call.
forwarded_to()
{
    local lines call_at

    lines="$(body "$1")"
    call_at="$(grep -nE '^bl 0x[0-9a-f]+' <<< "$lines" | head -1 | cut -d: -f1)"
    if [ -z "$call_at" ] \
        || [ -n "$(first_foreign "$(sed -n "1,${call_at}p" <<< "$lines")")" ]
    then
        return 1
    fi

    printf '%08x' "$(sed -n "${call_at}p" <<< "$lines" | cut -d' ' -f2)"
}

# Prints the instructions of the function the entry attribute wraps, through the
# body printer named as the argument.
#
# cortex-m-rt emits main as a frame that forwards to it, the same shape a
# forwarding fault handler wears, so the descent is the same one mute_body makes
# and stops at the same depth.
entry_body()
{
    local printer=$1 addr callee

    addr="$(sym_addr main)"
    if [ -z "$addr" ]
    then
        echo "FAIL: the image has no main symbol" >&2
        return 1
    fi

    if ! callee="$(forwarded_to "$addr")"
    then
        echo "FAIL: main at 0x$addr does not forward to the function it wraps" >&2
        return 1
    fi

    "$printer" "$callee"
}

# Prints the address of the routine that carries the mute, read out of the hard
# fault handler that forwards to it.
mute_routine()
{
    local addr callee

    addr="$(sym_addr HardFault)"
    if [ -z "$addr" ] || ! callee="$(forwarded_to "$addr")"
    then
        echo "FAIL: the hard fault handler forwards to no routine" >&2
        return 1
    fi

    printf '%s' "$callee"
}

# Prints one line per call the entry function makes to the routine that carries
# the mute: the refusal word the store ahead of that call carries, then the base
# register and the offset that store reaches memory through. A word the walk
# cannot read comes back as a dash. It reads the annotated body, because the
# addresses are what the block boundaries are read off.
#
# The word is read backwards from the store, over the instructions that build
# the stored register. A movw, a mov or a movs ends the walk with a value, a
# movt or a two operand add carries it on, and anything else naming that
# register ends the walk with a dash.
#
# The walk stops at every control flow boundary rather than reading through it.
# It crosses only WALKABLE_INSTRUCTION forms, so a call ends it rather than
# being stepped over as an instruction that names no register, and so does a
# branch of any mnemonic. It refuses to move past an address the function names
# in an operand, so a block entered from elsewhere ends it too. So a word
# printed here was built by the instructions between it and its store, on the
# one path that reaches that store, and by nothing else.
park_arms()
{
    local mute=$1
    local index store src base off value

    read_walk "$2"

    for index in "${!WALK_INSN[@]}"
    do
        if [ "${WALK_INSN[index]%% *}" != "bl" ] \
            || [ "$(cut -d' ' -f2 <<< "${WALK_INSN[index]}")" != "$mute" ]
        then
            continue
        fi

        store="${WALK_INSN[index - 1]-}"
        if [[ ! "$store" =~ ^str(\.w)?\ (r[0-9]+|lr|ip),\ \[(r[0-9]+)(,\ \#(0x[0-9a-f]+))?\]$ ]]
        then
            echo "FAIL: the arm calling the mute routine at line $((index + 1))" \
                "of the entry function runs \"$store\" ahead of the call, not a" \
                "store" >&2
            return 1
        fi
        src="${BASH_REMATCH[2]}"
        base="${BASH_REMATCH[3]}"
        off=$((${BASH_REMATCH[5]:-0}))

        # A store the function enters from elsewhere is reached on a path this
        # walk has not read, whatever stands textually ahead of it.
        if [ -n "${WALK_ENTERED[${WALK_SPOT[index - 1]}]-}" ] \
            || ! value="$(walk_immediate "$src" "$((index - 1))")"
        then
            printf -- '- %s %s\n' "$base" "$off"
            continue
        fi

        printf '%s %s %s\n' "$value" "$base" "$off"
    done
}

# Prints the address the entry function loads one register with, or nothing when
# it loads it with no address or with more than one.
#
# The halves are collected over the whole function rather than paired where they
# stand, so a register the function loads with two different addresses comes
# back empty and the arm reaching memory through it is refused.
loaded_address()
{
    local reg=$1 lines=$2 lo hi

    lo="$(sed -nE "s/^movw ${reg}, #(0x[0-9a-f]+)\$/\1/p" <<< "$lines" | sort -u)"
    hi="$(sed -nE "s/^movt ${reg}, #(0x[0-9a-f]+)\$/\1/p" <<< "$lines" | sort -u)"

    if [ -z "$lo" ] || [ -z "$hi" ] \
        || [ "$(wc -l <<< "$lo")" -ne 1 ] || [ "$(wc -l <<< "$hi")" -ne 1 ]
    then
        return 1
    fi

    printf '%08x' "$(((hi << 16) | lo))"
}

# Checks claim 6 against the entry function.
check_startup_refusals()
{
    local lines walk mute arms slot index expected label word base off carried
    local -a resolved unread

    if ! lines="$(entry_body body)" || [ -z "$lines" ]
    then
        return 1
    fi

    if ! walk="$(entry_body annotated_body)" || [ -z "$walk" ]
    then
        return 1
    fi

    slot="$(sym_addr "$REFUSAL_SYMBOL")"
    if [ -z "$slot" ]
    then
        fail "the image has no $REFUSAL_SYMBOL symbol"
        return 1
    fi

    if ! mute="$(mute_routine)"
    then
        return 1
    fi

    if ! arms="$(park_arms "0x$(printf '%x' "$((16#$mute))")" "$walk")" \
        || [ -z "$arms" ]
    then
        fail "no arm of the entry function calls the routine that carries the" \
            "mute"
        return 1
    fi

    resolved=()
    unread=()

    while read -r word base off
    do
        if ! carried="$(loaded_address "$base" "$lines")"
        then
            fail "an arm of the entry function stores its refusal word through" \
                "$base, which the function loads with no single address"
            return 1
        fi

        if [ "$((16#$carried + off))" -ne "$((16#$slot))" ]
        then
            fail "an arm of the entry function stores its refusal word at" \
                "0x$carried plus $off, and $REFUSAL_SYMBOL is at 0x$slot"
            return 1
        fi

        if [ "$word" != "-" ]
        then
            resolved+=("$word")
        else
            unread+=("$base")
        fi
    done <<< "$arms"

    if [ "${#resolved[@]}" -ne "${#STARTUP_REFUSALS[@]}" ]
    then
        fail "${#resolved[@]} arms of the entry function build their refusal" \
            "word from immediates, the start-up has ${#STARTUP_REFUSALS[@]}" \
            "guards"
        return 1
    fi

    if [ "${#unread[@]}" -ne "${#COMPUTED_REFUSALS[@]}" ]
    then
        fail "${#unread[@]} arms of the entry function compute their refusal" \
            "word, this gate names ${#COMPUTED_REFUSALS[@]} stages that do"
        return 1
    fi

    for index in "${!STARTUP_REFUSALS[@]}"
    do
        expected="${STARTUP_REFUSALS[index]%% *}"
        label="${STARTUP_REFUSALS[index]#* }"

        if [ "${resolved[index]}" != "$expected" ]
        then
            fail "guard $((index + 1)) writes 0x${resolved[index]} to the" \
                "refusal word, 0x$expected is the code of $label"
            return 1
        fi
    done

    echo "PASS: the ${#resolved[@]} start-up guards write their own cause into" \
        "0x$slot, in the order they run, and park on the next instruction," \
        "beside the ${#unread[@]} stages that compute their word"
}

# Checks claims 2 and 3 on one handler.
check_mute_path()
{
    local label=$1 addr=$2
    local lines line_no store prologue src base off after mask

    if ! lines="$(mute_body body "$addr")" || [ -z "$lines" ]
    then
        fail "$label at 0x$addr has no reachable body"
        return 1
    fi

    # The mute store is the first instruction of the routine that is not a bare
    # one. Reading it that way is what makes the check complete: whatever stands
    # ahead of it, of whatever mnemonic, is the failure.
    line_no="$(first_foreign "$lines")"
    if [ -z "$line_no" ]
    then
        fail "$label never stores to a peripheral"
        return 1
    fi
    store="$(sed -n "${line_no}p" <<< "$lines")"

    if [[ "$store" =~ ^str\ (r[0-9]+),\ \[(r[0-9]+),\ (#0x[0-9a-f]+)\]$ ]]
    then
        src="${BASH_REMATCH[1]}"
        base="${BASH_REMATCH[2]}"
        off="${BASH_REMATCH[3]}"
    else
        fail "$label runs \"$store\" ahead of the mute store"
        return 1
    fi

    if [ "$off" != "$BSRR_OFFSET" ]
    then
        fail "$label stores at offset $off, BSRR is at $BSRR_OFFSET"
        return 1
    fi

    prologue="$(sed -n "1,$((line_no - 1))p" <<< "$lines")"

    if ! grep -qxF "movw $base, $GPIOE_BASE_LOW" <<< "$prologue" \
        || ! grep -qxF "movt $base, $GPIOE_BASE_HIGH" <<< "$prologue"
    then
        fail "$label does not build the GPIOE address in $base"
        return 1
    fi

    mask=0
    grep -qxF "mov $src, $BR7_MASK" <<< "$prologue" && mask=1
    grep -qxF "mov.w $src, $BR7_MASK" <<< "$prologue" && mask=1
    if [ "$mask" -ne 1 ]
    then
        fail "$label does not build the PE7 reset mask in $src"
        return 1
    fi

    after="$(sed -n "$((line_no + 1))p" <<< "$lines")"
    if [ "$after" != "cpsid i" ]
    then
        fail "$label does not mask interrupts right after the mute store, found: $after"
        return 1
    fi

    after="$(sed -n "$((line_no + 2))p" <<< "$lines")"
    case "$after" in
        isb*) ;;
        *)
            fail "$label does not synchronise after the mask, found: $after"
            return 1
            ;;
    esac

    echo "PASS: nothing but a frame push, a register move, an immediate" \
        "constant or a call stands ahead of the mute store of $label, and the" \
        "mask and the barrier follow it"
}

# Checks claim 4 against the symbol table, the section headers and the reset
# handler.
check_record_slot()
{
    local addr size fill fill_lo fill_hi bounds base length

    addr="$(sym_addr "$RECORD_SYMBOL")"
    if [ -z "$addr" ]
    then
        fail "the image has no $RECORD_SYMBOL symbol"
        return 1
    fi

    size="$(sym_size "$RECORD_SYMBOL")"
    if [ -z "$size" ] || [ "$((16#$size))" -ne "$RECORD_BYTES" ]
    then
        fail "$RECORD_SYMBOL spans ${size:-no} bytes, the record is" \
            "$RECORD_BYTES"
        return 1
    fi

    fill="$(zero_fill_bounds)"
    if [ -z "$fill" ]
    then
        fail "the startup zero fill loop could not be read out of the" \
            "reset handler"
        return 1
    fi
    read -r fill_lo fill_hi <<< "$fill"

    if [ "$((16#$addr))" -lt "$((16#$fill_hi))" ] \
        && [ "$((16#$addr + RECORD_BYTES))" -gt "$((16#$fill_lo))" ]
    then
        fail "$RECORD_SYMBOL at 0x$addr lies in the startup zero fill," \
            "which walks 0x$fill_lo to 0x$fill_hi"
        return 1
    fi

    bounds="$(section_bounds "$RECORD_SECTION")"
    if [ -z "$bounds" ]
    then
        fail "the image has no $RECORD_SECTION section"
        return 1
    fi
    read -r base length <<< "$bounds"

    if [ "$((16#$addr))" -lt "$((16#$base))" ] \
        || [ "$((16#$addr + RECORD_BYTES))" -gt "$((16#$base + 16#$length))" ]
    then
        fail "$RECORD_SYMBOL at 0x$addr falls outside $RECORD_SECTION," \
            "which runs 0x$length bytes from 0x$base"
        return 1
    fi

    echo "PASS: $RECORD_SYMBOL is $RECORD_BYTES bytes at 0x$addr, inside" \
        "$RECORD_SECTION and clear of the startup zero fill, which walks" \
        "0x$fill_lo to 0x$fill_hi"
}

# Checks claim 5 on the routine that carries the mute for one handler.
check_record_write()
{
    local label=$1 addr=$2
    local lines walk record low high base index offset target stores
    local line_no previous low_at high_at
    # The compiler sources a record word from any allocatable register, and it
    # reaches for lr and ip once the low ones are taken.
    local source='(r[0-9]+|lr|ip)'

    if ! lines="$(mute_body body "$addr")" || [ -z "$lines" ] \
        || ! walk="$(mute_body annotated_body "$addr")" || [ -z "$walk" ]
    then
        fail "$label at 0x$addr has no reachable body"
        return 1
    fi
    read_walk "$walk"

    record="$(sym_addr "$RECORD_SYMBOL")"
    if [ -z "$record" ]
    then
        fail "the image has no $RECORD_SYMBOL symbol"
        return 1
    fi

    low="$(printf '#0x%x' "$((16#$record & 0xFFFF))")"
    high="$(printf '#0x%x' "$((16#$record >> 16))")"

    # Every RAM address in this image shares the high half, so the low half is
    # what picks the register out. The two halves are then required in order,
    # and the nine stores are required after them.
    base="$(sed -nE "s/^movw (r[0-9]+), ${low}\$/\1/p" <<< "$lines" | head -1)"
    if [ -z "$base" ]
    then
        fail "$label loads the low half of the record address 0x$record" \
            "into no register"
        return 1
    fi

    low_at="$(grep -nxF "movw $base, $low" <<< "$lines" \
        | head -1 | cut -d: -f1)"
    high_at="$(grep -nxF "movt $base, $high" <<< "$lines" \
        | head -1 | cut -d: -f1)"
    if [ -z "$high_at" ] || [ "$high_at" -le "$low_at" ]
    then
        fail "$label does not complete the record address 0x$record in" \
            "$base after loading its low half"
        return 1
    fi

    # Only what follows the completed address can be a record store. The mute
    # sequence ahead of it stores through registers this one is free to reuse,
    # and an unoffset store there wears the same shape as record word 0.
    lines="$(sed -n "$((high_at + 1)),\$p" <<< "$lines")"

    previous=0
    for ((index = 0; index < RECORD_WORDS; index++))
    do
        offset=$((index * 4))
        if [ "$offset" -eq 0 ]
        then
            target="\\[$base\\]"
        else
            target="$(printf '\\[%s, #0x%x\\]' "$base" "$offset")"
        fi

        line_no="$(grep -nE "^str(\.w)? ${source}, ${target}\$" <<< "$lines" \
            | head -1 | cut -d: -f1)"
        if [ -z "$line_no" ]
        then
            fail "$label stores no record word at offset $offset through $base"
            return 1
        fi

        if [ "$line_no" -le "$previous" ]
        then
            fail "$label stores the record word at offset $offset ahead of" \
                "the word before it"
            return 1
        fi
        previous=$line_no
    done

    stores="$(grep -cE "^str(\.w)? ${source}, \\[${base}(, #0x[0-9a-f]+)?\\]\$" \
        <<< "$lines" || true)"
    if [ "$stores" -ne "$RECORD_WORDS" ]
    then
        fail "$label makes $stores stores through $base, the record is" \
            "$RECORD_WORDS words"
        return 1
    fi

    # Rising line numbers are the order the compiler emitted the stores in, not
    # the order they run. The two agree only on a straight line, so the run from
    # the completed address to the last store is required to be one: a store the
    # routine reaches by a branch, or skips by one, counts here as a store the
    # routine always makes.
    if ! straight_line "$((high_at - 1))" "$((high_at + previous - 1))"
    then
        fail "$label does not reach its $RECORD_WORDS record stores from the" \
            "address it builds in $base on one straight line"
        return 1
    fi

    echo "PASS: the mute routine of $label builds the record address in" \
        "$base, then makes $RECORD_WORDS word stores through it at the record" \
        "offsets, in rising order, on one straight line"
}

status=0

hard_fault="$(sym_addr HardFault)"
default_handler="$(sym_addr DefaultHandler)"

if [ -z "$hard_fault" ] || [ -z "$default_handler" ]
then
    echo "FAIL: the image has no HardFault or no DefaultHandler symbol" >&2
    exit 1
fi

# Claim 1. Every vector past the reset entry either points at the hard fault
# handler or at the default handler, or is a reserved zero. Thumb entries carry
# the low bit set.
allowed_a="$(printf '%08x' "$((0x$hard_fault + 1))")"
allowed_b="$(printf '%08x' "$((0x$default_handler + 1))")"

words="$("$objdump" -s -j .vector_table "$BIN" | awk '
    BEGIN { n = 0 }
    /^[[:space:]]*[0-9a-f]+ [0-9a-f]{8}/ {
        for (i = 2; i <= 5 && i <= NF; i++)
        {
            if ($i !~ /^[0-9a-f]{8}$/) continue
            print n, substr($i, 7, 2) substr($i, 5, 2) substr($i, 3, 2) substr($i, 1, 2)
            n++
        }
    }')"

handlers=0
while read -r index word
do
    [ "$index" -lt "$FIRST_HANDLER_WORD" ] && continue
    [ "$word" = "00000000" ] && continue
    handlers=$((handlers + 1))
    if [ "$word" != "$allowed_a" ] && [ "$word" != "$allowed_b" ]
    then
        echo "FAIL: vector word $index points at 0x$word, which is neither handler" >&2
        status=1
    fi
done <<< "$words"

if [ "$handlers" -lt "$MIN_HANDLER_WORDS" ]
then
    echo "FAIL: only $handlers handler vectors read, expected at least $MIN_HANDLER_WORDS" >&2
    status=1
elif [ "$status" -eq 0 ]
then
    echo "PASS: all $handlers fault and interrupt vectors lead to a muting handler"
fi

check_record_slot || status=1
check_startup_refusals || status=1

check_mute_path "the hard fault handler" "$hard_fault" || status=1
check_record_write "the hard fault handler" "$hard_fault" || status=1

if [ "$default_handler" != "$hard_fault" ]
then
    check_mute_path "the default handler" "$default_handler" || status=1
    check_record_write "the default handler" "$default_handler" || status=1
fi

# The panic handler is emitted only once something in the image can panic. While
# nothing can, there is no code to read and no claim to check. The compiler
# places it in a namespace of its own, so the lookup matches the last component.
panic_handler="$(awk '
    $3 == "rust_begin_unwind" || $3 ~ /::rust_begin_unwind$/ { print $1; exit }
    ' <<< "$syms")"
if [ -n "$panic_handler" ]
then
    check_mute_path "the panic handler" "$panic_handler" || status=1
    check_record_write "the panic handler" "$panic_handler" || status=1
else
    echo "NOTE: no panic handler in the image, nothing in it can panic"
fi

if [ "$status" -ne 0 ]
then
    echo
    echo "the mute path of $CRATE no longer matches what its source claims" >&2
    exit 1
fi

echo
echo "mute gate green"
