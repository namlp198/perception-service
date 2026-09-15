#!/usr/bin/env bash
set -euo pipefail

preset="${1:-debug}"
ctest --preset "${preset}"
