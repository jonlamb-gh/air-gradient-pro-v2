#!/usr/bin/env bash

set -euo pipefail

cargo flash --release --chip STM32F407ZGTx --reset-halt --restore-unwritten --connect-under-reset --features="debug"

exit 0
