TARGET=flash
SYS_SOURCES=$(wildcard ./../../sys/*.c)
INTERNAL_SOURCES=./../../sys/boot.S $(SYS_SOURCES) $(SOURCES)
CC=riscv64-unknown-elf-gcc
OBJCOPY=riscv64-unknown-elf-objcopy
OBJDUMP=riscv64-unknown-elf-objdump
SIZE=riscv64-unknown-elf-size
NM=riscv64-unknown-elf-nm
READELF=riscv64-unknown-elf-readelf
OPT?=NONE
ABI?=rv32i  # Or rv32e for embedded version with reduced register count

DEFINES =

ifeq ($(OPT),SIZE)
    OPT_FLAGS=-Os
else
    OPT_FLAGS=-O0
endif

ifeq ($(ABI),rv32e)
    ARCH_FLAGS = -march=rv32e_zicsr
    ABI_FLAGS = -mabi=ilp32e
    DEFINES += -DRV32E
else
    ARCH_FLAGS = -march=rv32i_zicsr
    ABI_FLAGS = -mabi=ilp32
endif


CFLAGS=-nostdlib -Wl,--build-id=none -Wl,--gc-sections $(OPT_FLAGS) -fdata-sections -ffunction-sections -nostartfiles $(ARCH_FLAGS) $(ABI_FLAGS) $(DEFINES) -I./../../sys -I./include -T./../../sys/link.ld -g
LDFLAGS=

all: elf flat fpga copy mem_usage

elf: $(TARGET).elf

flat: $(TARGET).bin

fpga: $(TARGET).txt

$(TARGET).elf: $(INTERNAL_SOURCES)
	@$(CC) $(CFLAGS) -o $(TARGET).elf $(INTERNAL_SOURCES)
	@printf "%-8s %s\n" "CC" "$(INTERNAL_SOURCES)"
	
$(TARGET).bin: $(TARGET).elf
	@$(OBJCOPY) -O binary -j ".reset" -j ".isr_common" -j ".text" -j ".rodata*" -j ".srodata*" -j ".data*" -j ".sdata.*" $(TARGET).elf $(TARGET).bin
	@printf "%-8s %s\n" "OBJCOPY" "$< -> $@"
	
$(TARGET).txt: $(TARGET).bin
	@xxd -g 4 -u -ps -c 4 $(TARGET).bin > $(TARGET).txt
	@printf "%-8s %s\n" "XXD" "$< -> $@"
	
mem_usage: $(TARGET).elf
	@$(SIZE) $(TARGET).elf --format=sysv | ./../../scripts/stats.py --mem-usage --flash-size=12288 --sram-size=4096 --newline
	
copy: $(TARGET).txt
	@cp $(TARGET).txt ./../../../../rtl/memory/
	@printf "%-8s %s\n" "CP" "$< -> /memory"
	
disasm: $(TARGET).elf
	@$(OBJDUMP) -dS $(TARGET).elf -j ".text"

disasm_all: $(TARGET).elf
	@$(OBJDUMP) -dS $(TARGET).elf -j ".reset" -j ".isr_common" -j ".text"
	
section_contents: $(TARGET).elf
	@$(OBJDUMP) -s $(TARGET).elf
	
sections: $(TARGET).elf
	@$(READELF) --sections $(TARGET).elf

symbols: $(TARGET).elf
	@$(NM) --print-size --size-sort --reverse-sort -td flash.elf | ./../../scripts/stats.py --newline --sym-size
	
clean:
	rm -rf *.o
	rm -rf *.elf
	rm -rf *.bin
	rm -rf *.txt


