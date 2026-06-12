/* link.ld  (보강판) */
/* Harvard 구조: IMEM(명령), DMEM(데이터) 분리, 각각 0x0 부터 1KB */
/* ★ 주의: 이 코어는 IMEM 을 데이터로 못 읽습니다.                */
/*    → .rodata(const/문자열/점프테이블)는 lw 로 접근 불가.        */
/*      C 코드에서 const 배열/문자열을 두지 말고,                  */
/*      컴파일 시 -fno-jump-tables 로 switch 점프테이블도 끄세요.  */

MEMORY {
    IMEM (rx)  : ORIGIN = 0x00000000, LENGTH = 1K
    DMEM (rwx) : ORIGIN = 0x00000000, LENGTH = 1K
}

SECTIONS {
    .text : {
        *(.text) *(.text.*)
    } > IMEM

    .rodata : {                 /* 가급적 비워둘 것 (위 주의 참고) */
        *(.rodata) *(.rodata.*)
    } > IMEM

    .data : {
        __data_start = .;
        *(.data) *(.data.*)
        *(.sdata) *(.sdata.*)
        __data_end = .;
    } > DMEM

    .bss (NOLOAD) : {
        __bss_start = .;
        *(.bss) *(.bss.*)
        *(.sbss) *(.sbss.*)
        *(COMMON)
        __bss_end = .;
    } > DMEM

    /* 스택 최상단 = DMEM 끝 (0x400). 첫 push 는 0x3FC 에 기록됨 */
    __stack_top = ORIGIN(DMEM) + LENGTH(DMEM);
}
