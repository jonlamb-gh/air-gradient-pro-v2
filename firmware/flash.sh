#!/usr/bin/env bash

set -euo pipefail

cargo flash --release --chip STM32F407ZGTx --restore-unwritten

exit 0
