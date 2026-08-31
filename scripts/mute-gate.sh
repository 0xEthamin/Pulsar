#!/usr/bin/env bash
# Reads the mute path back out of the linked processing board image.
#
# The loudspeaker carries no analog filter and no analog mute, so the store that
# drives XSMT low is the only thing between a fault and the drivers. The source
# of pulsar_dsp claims three properties about that store which neither a test
# nor the compiler checks:
#
#   1. every fault and interrupt vector points at a handler that mutes,
#   2. the mute store is the first thing a handler does to a peripheral,
#   3. the interrupt mask and its barrier follow that store immediately.
#
# Each claim is checked here against the disassembly of the image that ships.
#
# Claim 1 turns every new handler into a decision. A vector that stops pointing
# at the hard fault handler or at the default handler trips this gate, and the
# way through it is to say what that handler does about the drivers, not to add
# its address to the allowed set.
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
disasm="$("$objdump" -d --no-show-raw-insn "$BIN")"

# Prints the 8 digit load address of one symbol, or nothing when it is absent.
sym_addr()
{
    awk -v want="$1" '$3 == want { print $1; exit }' <<< "$syms"
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

# Prints the instructions that carry the mute for the handler at one address. A
# handler that only tail calls the shared routine holds no store of its own, so
# the first call is followed one level down.
mute_body()
{
    local lines callee
    lines="$(body "$1")"
    if [ -z "$lines" ]
    then
        return 1
    fi
    if ! grep -qE '^str ' <<< "$lines"
    then
        callee="$(grep -m1 -oE '^bl 0x[0-9a-f]+' <<< "$lines" | cut -d' ' -f2)"
        if [ -z "$callee" ]
        then
            return 1
        fi
        lines="$(body "$(printf '%08x' "$callee")")"
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

    line_no="$(grep -nE '^str ' <<< "$lines" | head -1 | cut -d: -f1)"
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
        fail "$label opens with a store this gate cannot read: $store"
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

    if grep -qE '^(ldr|ldrb|ldrh|ldrd|ldm|str|strb|strh|strd|stm)[ .]' <<< "$prologue"
    then
        fail "$label touches memory before the mute store"
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

    echo "PASS: $label mutes first, then masks, then synchronises"
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

check_mute_path "the hard fault handler" "$hard_fault" || status=1

if [ "$default_handler" != "$hard_fault" ]
then
    check_mute_path "the default handler" "$default_handler" || status=1
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
