# Makefile — RISC-V bare-metal build
CC      = riscv-none-elf-gcc
OBJCOPY = riscv-none-elf-objcopy
OBJDUMP ?= $(shell command -v riscv-none-elf-objdump 2>/dev/null || command -v riscv64-unknown-elf-objdump 2>/dev/null || echo objdump)

# -msmall-data-limit=0            : small-data(.sdata/.sbss)/gp 상대주소 비활성 (gp 미설정 대응)
# -fno-jump-tables: switch 점프테이블(.rodata) 금지 (하버드: IMEM은 데이터로 못 읽음)
# -ffreestanding : 베어메탈
CFLAGS = -march=rv32i -mabi=ilp32 -nostdlib -nostartfiles -O1 \
         -msmall-data-limit=0 -fno-jump-tables -ffreestanding

# Build options:
#   make             -> MMIO LED/HEX blink diagnostic
#   make TEST=sensor -> sensor MMIO polling test
TEST ?= blink
MAIN = main_mmio_$(TEST)_test.c

SRCS   = startup.s $(MAIN)
TARGET = program

all: $(TARGET).hex

$(TARGET).elf: $(SRCS)
	$(CC) $(CFLAGS) -T link.ld -o $@ $^

# IMEM 초기화용 hex (워드폭 4로 고정 → $readmemh 워드 정렬 보장)
$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -O verilog --verilog-data-width=4 $< $@

dump: $(TARGET).elf
	$(OBJDUMP) -d $< > $(TARGET).dump

clean:
	rm -f $(TARGET).elf $(TARGET).hex $(TARGET).dump

.PHONY: all dump clean
