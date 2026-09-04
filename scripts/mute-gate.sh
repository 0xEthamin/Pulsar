#!/usr/bin/env bash
# Reads the mute path back out of the linked processing board image.
#
# The loudspeaker carries no analog filter and no analog mute, so the store that
# drives XSMT low is the only thing between a fault and the drivers. The source
# of pulsar_dsp claims five properties which neither a test nor the compiler
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
#      record offsets, in rising offset order.
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
# Claim 5 fails when the nine stores stop being nine separate word stores made
# in rising offset order. Dropping the volatile qualifier does exactly that: the
# compiler then merges neighbouring words into strd pairs and reorders them,
# which costs the property the order carries, that the magic lands first and the
# checksum last so an interrupted write fails validation rather than reading as
# a record.
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

# Instructions a frame may run ahead of the mute store, or ahead of the call
# that reaches it: the frame push, a register move, an immediate constant, a
# call. Anything else fails the gate, whatever it does, which is what makes the
# check complete where a list of memory access mnemonics to refuse never is.
BARE_INSTRUCTION='^(push \{[^}]*\}|movs?(w|t|\.w)? [^,]+, [^,]+|bl 0x[0-9a-f]+( <[^>]*>)?|nop)$'

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

# Prints the low and the high bound of the startup zero fill, read out of the
# loop that performs it. cortex-m-rt writes that loop as a pointer register
# walking up to a limit register, both loaded from the literal pool, and the
# store is the only post-incrementing stm in the reset handler. Which symbols
# the two literals came from does not matter here, and must not: the bounds move
# with the build, and a value compared by name matches whatever else happens to
# share it.
zero_fill_bounds()
{
    local reset lines found at pointer limit lo_at hi_at lo hi

    reset="$(sym_addr Reset)"
    if [ -z "$reset" ]
    then
        return 1
    fi
    lines="$(annotated_body "$reset")"

    found="$(grep -nE ' stm r[0-9]+!, \{r[0-9]+\}$' <<< "$lines" | head -1)"
    if [ -z "$found" ]
    then
        return 1
    fi
    at="${found%%:*}"
    pointer="$(sed -nE 's/^.* stm (r[0-9]+)!, \{r[0-9]+\}$/\1/p' \
        <<< "${found#*:}")"

    lines="$(sed -n "1,${at}p" <<< "$lines")"

    # The loop exits when the limit register meets the pointer register.
    limit="$(sed -nE "s/^.* cmp (r[0-9]+), ${pointer}\$/\1/p" <<< "$lines" \
        | tail -1)"
    if [ -z "$limit" ]
    then
        return 1
    fi

    lo_at="$(sed -nE "s/^.* ldr ${pointer}, \[pc, #0x[0-9a-f]+\] @ 0x([0-9a-f]+) .*\$/\1/p" \
        <<< "$lines" | tail -1)"
    hi_at="$(sed -nE "s/^.* ldr ${limit}, \[pc, #0x[0-9a-f]+\] @ 0x([0-9a-f]+) .*\$/\1/p" \
        <<< "$lines" | tail -1)"
    if [ -z "$lo_at" ] || [ -z "$hi_at" ]
    then
        return 1
    fi

    lines="$(annotated_body "$reset")"
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

# Prints the instructions that carry the mute for the handler at one address.
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
    local addr=$1
    local lines call_at callee

    lines="$(body "$addr")"
    if [ -z "$lines" ]
    then
        echo "FAIL: the frame at 0x$addr has no body in the disassembly" >&2
        return 1
    fi

    call_at="$(grep -nE '^bl 0x[0-9a-f]+' <<< "$lines" | head -1 | cut -d: -f1)"

    if [ -n "$call_at" ] \
        && [ -z "$(first_foreign \
            "$(without_icsr_read "$(sed -n "1,${call_at}p" <<< "$lines")")")" ]
    then
        callee="$(sed -n "${call_at}p" <<< "$lines" | cut -d' ' -f2)"
        lines="$(body "$(printf '%08x' "$callee")")"
        if [ -z "$lines" ]
        then
            echo "FAIL: the frame at 0x$addr forwards to $callee, which has no" \
                "body in the disassembly" >&2
            return 1
        fi
    fi

    printf '%s\n' "$lines"
}

fail()
{
    echo "FAIL: $*" >&2
    return 1
}

# Checks claims 2 and 3 on one handler.
check_mute_path()
{
    local label=$1 addr=$2
    local lines line_no store prologue src base off after mask

    if ! lines="$(mute_body "$addr")" || [ -z "$lines" ]
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

    echo "PASS: only a frame push and register moves stand ahead of the mute" \
        "store of $label, and the mask and the barrier follow it"
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
    local lines record low high base index offset target stores
    local line_no previous low_at high_at
    # The compiler sources a record word from any allocatable register, and it
    # reaches for lr and ip once the low ones are taken.
    local source='(r[0-9]+|lr|ip)'

    if ! lines="$(mute_body "$addr")" || [ -z "$lines" ]
    then
        fail "$label at 0x$addr has no reachable body"
        return 1
    fi

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

    echo "PASS: the mute routine of $label builds the record address in" \
        "$base, then makes $RECORD_WORDS word stores through it at the record" \
        "offsets, in rising order"
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
