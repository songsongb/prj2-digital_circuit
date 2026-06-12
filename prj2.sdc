# prj2.sdc
# DE2 보드 타이밍 제약 파일 (TOP = TOP_SOC_MMIO_TEST)

# 50MHz 입력 클럭 (주기 20ns)
create_clock -name CLOCK_50 -period 20.000 [get_ports CLOCK_50]

# LCM_PLL(altpll) 파생 클럭 자동 유도 + 불확실성
derive_pll_clocks
derive_clock_uncertainty

# 비동기 입력은 타이밍 분석 제외
set_false_path -from [get_ports {KEY[0]}]
set_false_path -from [get_ports {KEY[1]}]
set_false_path -from [get_ports UART_RXD]
set_false_path -from [get_ports {GPIO_1[0]}]
# ※ OV7670 PCLK(카메라 입력 클럭)은 별도 create_clock이 이상적이나,
#   꽂힌 GPIO 핀을 확인한 뒤 추가하세요. 우선 derive_pll_clocks로 LCM 도메인은 커버됨.
