#!/bin/sh
make clean
make GCC_PATH=/home/user/tools/gcc-riscv-none-embed-8.2-x86_64/bin DEBUG_BUILD=1
../../OpenOCD/bin/openocd -f "../../OpenOCD/bin/wch-riscv.cfg" -c init -c "adapter speed 4000" -c "program build/Template.elf" -c "reset run" -c "exit"
