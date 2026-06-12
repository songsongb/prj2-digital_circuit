# startup.s  (베어메탈 시작 코드 - 보강판)
#  - 스택 포인터를 DMEM 최상단(0x400)으로 → 저주소 전역변수와 충돌 방지
#  - .bss 영역 0 초기화 (하드웨어엔 데이터 로더가 없으므로 직접 클리어)
#  - 사용하는 명령(addi/bge/sw/jal/jalr)은 전부 코어가 지원함을 검증함
.section .text
.global _start

_start:
    la   sp, __stack_top        # 스택: DMEM 맨 위 (0x400)

    # ── .bss 0으로 클리어 ──
    la   t0, __bss_start
    la   t1, __bss_end
1:
    bge  t0, t1, 2f
    sw   zero, 0(t0)
    addi t0, t0, 4
    j    1b
2:
    call main                   # 링커 relax 시 jal, 아니면 auipc+jalr (둘 다 지원)

_halt:
    j _halt
