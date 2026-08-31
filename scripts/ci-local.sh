#!/usr/bin/env bash
# Local mirror of the CI pipeline (.github/workflows/ci.yml).
#
# Runs the same gates without GitHub. Reports land at the repository root under
# the names sonar-project.properties expects: clippy-report.json, lcov.info and
# cargo-deny-<crate>.sarif.
#
# Usage: scripts/ci-local.sh [--quick] [--strict]
#   --quick    skip the control board, whose ESP-IDF build is the long one
#   --strict   a missing optional tool fails the run instead of skipping it
#
# Optional SonarQube upload: export SONAR_HOST_URL and SONAR_TOKEN, have
# sonar-scanner on PATH, and the final stage pushes the analysis.

set -euo pipefail
cd "$(dirname "$0")/.."

QUICK=0
STRICT=0
while [ $# -gt 0 ]
do
    case "$1" in
        --quick) QUICK=1 ;;
        --strict) STRICT=1 ;;
        *) echo "unknown option: $1" >&2; exit 2 ;;
    esac
    shift
done

CRATES="pulsar_lib pulsar_dsp pulsar_ctrl"
COVERAGE_FLOOR=95

passed=()
failed=()
skipped=()

run()
{
    local name=$1
    shift
    echo
    echo "==== ${name} ===="
    if "$@"
    then
        passed+=("$name")
    else
        failed+=("$name")
    fi
}

skip()
{
    local name=$1 how=$2
    echo
    echo "==== ${name} ==== SKIPPED (install with: ${how})"
    if [ "$STRICT" = 1 ]
    then
        failed+=("$name (missing tool)")
    else
        skipped+=("$name")
    fi
}

have()
{
    command -v "$1" >/dev/null 2>&1
}

# The three crates are three separate cargo projects. Cargo picks up
# .cargo/config.toml and rust-toolchain.toml from the current directory, not
# from the manifest, so every stage enters the crate rather than pointing at it.
lib_stage()
{
    (
        cd pulsar_lib
        export RUSTFLAGS="-D warnings"
        cargo check --locked --all-targets || exit 1
        cargo test --locked || exit 1
        cargo clippy --locked --all-targets --message-format=json \
            > ../clippy-report-lib.json || true
        cargo clippy --locked --all-targets -- -D warnings
    )
}

# RUSTFLAGS stays unset from here on: setting it replaces the rustflags the
# target configs carry (-Tlink.x, --cfg espidf_time64), and the link would break
# on the first one. The warning gate rides on clippy instead.
dsp_stage()
{
    (
        cd pulsar_dsp
        cargo build --locked --release || exit 1
        cargo clippy --locked --all-targets --message-format=json \
            > ../clippy-report-dsp.json || true
        cargo clippy --locked --all-targets -- -D warnings
    ) || return 1
    scripts/mute-gate.sh
}

ctrl_stage()
{
    (
        cd pulsar_ctrl
        cargo build --locked --release || exit 1
        cargo clippy --locked --release --message-format=json \
            > ../clippy-report-ctrl.json || true
        cargo clippy --locked --release -- -D warnings
    )
}

coverage_stage()
{
    (
        cd pulsar_lib
        cargo llvm-cov --locked --lcov --output-path ../lcov.info || exit 1
        cargo llvm-cov report --fail-under-lines "$COVERAGE_FLOOR"
    )
}

# Advisories (RustSec), licenses, banned and yanked crates, untrusted sources,
# duplicate versions. deny.toml sits at the repository root and covers both
# targets, so the three crates answer to the one policy. The SARIF export runs
# first and never blocks, so the readable gate below is what fails the stage.
deny_stage()
{
    local c status=0
    for c in $CRATES
    do
        cargo deny --manifest-path "$c/Cargo.toml" --config deny.toml \
            --format sarif check > "cargo-deny-$c.sarif" || true
    done
    for c in $CRATES
    do
        echo ">> deny $c"
        cargo deny --manifest-path "$c/Cargo.toml" --config deny.toml check \
            -A advisory-not-detected || status=1
    done
    return "$status"
}

# The control board is left out: udeps needs nightly and nightly has no Xtensa
# backend. The processing board needs its target on nightly too, since udeps
# resolves the dependency graph for the target it is asked about.
udeps_stage()
{
    local c status=0
    for c in pulsar_lib pulsar_dsp
    do
        if [ "$c" = pulsar_dsp ] \
            && ! rustup target list --installed --toolchain nightly \
                | grep -qx thumbv7em-none-eabihf
        then
            echo ">> udeps pulsar_dsp SKIPPED (rustup target add --toolchain nightly thumbv7em-none-eabihf)"
            if [ "$STRICT" = 1 ]
            then
                status=1
            fi
            continue
        fi
        echo ">> udeps $c"
        (cd "$c" && cargo +nightly udeps --all-targets --locked) || status=1
    done
    return "$status"
}

# pipeline stages, same order as the workflow

run "core (host check, test, clippy)" lib_stage

run "processing board (thumbv7em, mute gate)" dsp_stage

if [ "$QUICK" = 0 ]
then
    run "control board (xtensa, ESP-IDF)" ctrl_stage
else
    skip "control board" "drop --quick"
fi

if have cargo-llvm-cov
then
    run "coverage (llvm-cov, floor ${COVERAGE_FLOOR})" coverage_stage
else
    skip "coverage" "cargo install cargo-llvm-cov"
fi

if have cargo-deny
then
    run "deny (advisories, licenses, sources, yanked; sarif)" deny_stage
else
    skip "deny" "cargo install cargo-deny"
fi

if have cargo-udeps && rustup toolchain list | grep -q nightly
then
    run "udeps (nightly)" udeps_stage
else
    skip "udeps" "cargo install cargo-udeps (and a nightly toolchain)"
fi

if have cargo-outdated
then
    echo
    echo "==== outdated (informational, never blocking) ===="
    for c in $CRATES
    do
        echo ">> outdated $c"
        (cd "$c" && cargo outdated --root-deps-only) || true
    done
else
    skip "outdated" "cargo install cargo-outdated"
fi

cat clippy-report-*.json > clippy-report.json 2>/dev/null || true

if [ -n "${SONAR_HOST_URL:-}" ] && [ -n "${SONAR_TOKEN:-}" ] && have sonar-scanner
then
    run "sonar-scanner" sonar-scanner \
        -Dsonar.host.url="$SONAR_HOST_URL" \
        -Dsonar.token="${SONAR_TOKEN}"
else
    skip "sonar" "export SONAR_HOST_URL and SONAR_TOKEN, install sonar-scanner"
fi

# summary

echo
echo "===== summary ====="
for s in "${passed[@]:-}";  do [ -n "$s" ] && echo "PASS    $s"; done
for s in "${skipped[@]:-}"; do [ -n "$s" ] && echo "SKIP    $s"; done
for s in "${failed[@]:-}";  do [ -n "$s" ] && echo "FAIL    $s"; done

if [ "${#failed[@]}" -gt 0 ]
then
    exit 1
fi
echo "all executed stages green"
