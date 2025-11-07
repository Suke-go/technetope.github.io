#!/usr/bin/env bash
#
# Helper to send the MOTHER Earth (Missing Fundamental) timeline via the
# scheduler CLI. Override defaults with environment variables:
#   HOST, PORT, LEAD, SPACING, DRY_RUN (0 to disable),
#   SCHED_BIN, TIMELINE, TARGET_MAP, OSC_CONFIG.

#DRY_RUN=0 HOST=172.20.10.15 \
#  SCHED_BIN=$PWD/build/acoustics/scheduler/agent_a_scheduler \
#  acoustics/archive/mother_mf/tools/run_mother_mf.sh

set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

detect_repo_root() {
  if command -v git >/dev/null 2>&1; then
    if git_root="$(git -C "${script_dir}" rev-parse --show-toplevel 2>/dev/null)"; then
      printf '%s\n' "${git_root}"
      return
    fi
  fi

  candidate="${script_dir}"
  while [ "${candidate}" != "/" ]; do
    if [ -d "${candidate}/.git" ] || [ -f "${candidate}/.git" ]; then
      printf '%s\n' "${candidate}"
      return
    fi
    candidate="$(dirname "${candidate}")"
  done

  # Fallback: assume archive lives under acoustics/ and repo root one level up.
  printf '%s\n' "$(cd "${script_dir}/../../../.." && pwd)"
}

repo_root="$(detect_repo_root)"

SCHED_BIN="${SCHED_BIN:-${repo_root}/build/acoustics/scheduler/agent_a_scheduler}"
# SCHED_BIN="${SCHED_BIN:-${repo_root}/build/scheduler/agent_a_scheduler}"
TIMELINE="${TIMELINE:-${repo_root}/acoustics/pc_tools/scheduler/examples/mother_mf_a_section.json}"
TARGET_MAP="${TARGET_MAP:-${repo_root}/acoustics/tests/test01/targets_mother.json}"
HOST="${HOST:-255.255.255.255}"
PORT="${PORT:-9000}"
LEAD="${LEAD:-8.0}"
SPACING="${SPACING:-0.05}"
DRY_RUN="${DRY_RUN:-1}"
OSC_CONFIG="${OSC_CONFIG:-${repo_root}/acoustics/secrets/osc_config.json}"

if [[ ! -x "${SCHED_BIN}" ]]; then
  echo "Scheduler binary not found: ${SCHED_BIN}" >&2
  echo "Build it first (e.g. cmake --build build/scheduler)" >&2
  exit 1
fi

if [[ ! -f "${TIMELINE}" ]]; then
  echo "Timeline JSON not found: ${TIMELINE}" >&2
  exit 1
fi

if [[ ! -f "${TARGET_MAP}" ]]; then
  echo "Target map not found: ${TARGET_MAP}" >&2
  exit 1
fi

if [[ ! -f "${OSC_CONFIG}" ]]; then
  echo "OSC config not found: ${OSC_CONFIG}" >&2
  echo "Create it from acoustics/secrets/osc_config.example.json and keep it local." >&2
  exit 1
fi

cmd=(
  "${SCHED_BIN}"
  "--target-map" "${TARGET_MAP}"
  "--host" "${HOST}"
  "--port" "${PORT}"
  "--lead-time" "${LEAD}"
  "--bundle-spacing" "${SPACING}"
  "--osc-config" "${OSC_CONFIG}"
)

if [[ "${DRY_RUN}" != "0" ]]; then
  cmd+=(--dry-run)
fi

if [[ $# -gt 0 ]]; then
  cmd+=("$@")
fi

cmd+=("${TIMELINE}")

echo "Executing: ${cmd[*]}"
exec "${cmd[@]}"
