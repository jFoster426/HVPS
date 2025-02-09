cls
@echo ---- Beginning Build ----
@echo off

xc8-cc -O0 -mcpu=pic18f56q71 main.c -o FW_PIC.hex

del *.cmf *.d *.elf *.hxl *.p1 *.sdb *.sym *.lst *.o *.s *.rlf