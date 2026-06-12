# make_hex.py
# ELF 바이너리를 $readmemh용 hex 파일로 변환
# 사용법: python make_hex.py

import struct
import subprocess
import os

# 1. ELF → 바이너리 변환
print("Converting ELF to binary...")
subprocess.run([
    "riscv-none-elf-objcopy",
    "-O", "binary",
    "program.elf",
    "program.bin"
], check=True)

# 2. 바이너리 → 32비트 워드 hex 변환
print("Converting binary to hex...")
with open("program.bin", "rb") as f:
    data = f.read()

with open("program.hex", "w") as f:
    f.write("@00000000\n")
    for i in range(0, len(data), 4):
        word = data[i:i+4]
        # 4바이트 미만이면 0으로 패딩
        if len(word) < 4:
            word = word + b'\x00' * (4 - len(word))
        # 리틀엔디안 32비트 정수로 변환
        val = struct.unpack('<I', word)[0]
        f.write(f'{val:08X}\n')

word_count = (len(data) + 3) // 4
print(f"Done! {word_count} words written to program.hex")
print(f"Copy program.hex to your Quartus project root folder")

# 3. 정리
os.remove("program.bin")
