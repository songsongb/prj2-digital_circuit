// =============================================================
// HL_PARSER.v   HuskyLens UART 프레임 파서  (공식 프로토콜 v0.5.1)
//
// 실제 프레임 형식 (★ 기존 코드가 CMD/LEN 순서를 뒤바꿔 파싱하던 버그 수정):
//   [0x55][0xAA][0x11][LEN(1바이트)][CMD][DATA x LEN][CHKSUM]
//   CHKSUM = (0x55+0xAA+0x11+LEN+CMD + 모든 DATA) 의 하위 8비트
//
// 처리하는 반환 명령:
//   0x29 INFO  : DATA[0]=객체수 low, [1]=high  → OBJ_COUNT (없으면 NO_RESULT)
//   0x2A BLOCK : 일반 알고리즘(Tag/Object/Face/Color) 결과
//                DATA: xC(0:1) yC(2:3) W(4:5) H(6:7) ID(8:9)  (LE)
//   0x2B ARROW : Line tracking 결과
//                DATA: xOrigin(0:1) yOrigin(2:3) xTarget(4:5) yTarget(6:7) ID(8:9)
//
// 결과 매핑:
//   ALGO_ID  = 반환 CMD(0x2A/0x2B)  → SW 가 BLOCK/ARROW 구분 가능
//   OBJ_X/Y  = BLOCK: 중심좌표  / ARROW: origin 좌표
//   OBJ_W/H  = BLOCK: 폭/높이   / ARROW: target 좌표
//   OBJ_ID   = 학습 ID
// =============================================================
module HL_PARSER (
    input        CLK,
    input        RESET,

    input  [7:0] RX_DATA,
    input        RX_VALID,

    output reg [7:0]  ALGO_ID,
    output reg [7:0]  OBJ_ID,
    output reg [15:0] OBJ_X,
    output reg [15:0] OBJ_Y,
    output reg [15:0] OBJ_W,
    output reg [15:0] OBJ_H,
    output reg [7:0]  OBJ_COUNT,
    output reg        DATA_VALID,   // 1클럭 펄스: BLOCK/ARROW 유효 수신
    output reg        NO_RESULT     // 1클럭 펄스: 객체 0개(INFO count=0)
);
    localparam HEADER1 = 8'h55, HEADER2 = 8'hAA, HEADER3 = 8'h11;

    localparam S_H1=4'd0, S_H2=4'd1, S_H3=4'd2,
               S_LEN=4'd3, S_CMD=4'd4, S_DATA=4'd5, S_CHKSUM=4'd6;

    reg [3:0]  state;
    reg [7:0]  cmd_reg;
    reg [7:0]  len_reg;          // 길이는 1바이트
    reg [7:0]  data_buf [0:31];
    reg [4:0]  data_idx;
    reg [7:0]  chksum_calc;

    always @(posedge CLK) begin
        if (RESET) begin
            state <= S_H1; DATA_VALID <= 0; NO_RESULT <= 0;
            OBJ_COUNT <= 0; data_idx <= 0; chksum_calc <= 0;
        end else begin
            DATA_VALID <= 0;
            NO_RESULT  <= 0;

            if (RX_VALID) begin
                case (state)
                    S_H1: if (RX_DATA == HEADER1) begin
                              state <= S_H2; chksum_calc <= HEADER1;
                          end

                    S_H2: if (RX_DATA == HEADER2) begin
                              state <= S_H3; chksum_calc <= chksum_calc + HEADER2;
                          end else state <= S_H1;

                    S_H3: if (RX_DATA == HEADER3) begin
                              state <= S_LEN; chksum_calc <= chksum_calc + HEADER3;
                          end else state <= S_H1;

                    // ★ LEN 이 CMD 보다 먼저 온다
                    S_LEN: begin
                              len_reg     <= RX_DATA;
                              chksum_calc <= chksum_calc + RX_DATA;
                              state       <= S_CMD;
                          end

                    S_CMD: begin
                              cmd_reg     <= RX_DATA;
                              chksum_calc <= chksum_calc + RX_DATA;
                              data_idx    <= 0;
                              if (len_reg == 0) state <= S_CHKSUM; // 데이터 없음(예: OK)
                              else              state <= S_DATA;
                          end

                    S_DATA: begin
                              if (data_idx < 32) data_buf[data_idx] <= RX_DATA;
                              chksum_calc <= chksum_calc + RX_DATA;
                              if (data_idx == len_reg - 1) state <= S_CHKSUM;
                              data_idx <= data_idx + 1'b1;
                          end

                    S_CHKSUM: begin
                              state <= S_H1;
                              if (RX_DATA == chksum_calc) begin
                                  case (cmd_reg)
                                      8'h29: begin   // INFO: 객체 수
                                          OBJ_COUNT <= data_buf[0];
                                          if (data_buf[0] == 8'd0 && data_buf[1] == 8'd0)
                                              NO_RESULT <= 1;
                                      end
                                      8'h2A: begin   // BLOCK
                                          ALGO_ID <= 8'h2A;
                                          OBJ_X   <= {data_buf[1], data_buf[0]};
                                          OBJ_Y   <= {data_buf[3], data_buf[2]};
                                          OBJ_W   <= {data_buf[5], data_buf[4]};
                                          OBJ_H   <= {data_buf[7], data_buf[6]};
                                          OBJ_ID  <= data_buf[8];
                                          DATA_VALID <= 1;
                                      end
                                      8'h2B: begin   // ARROW (Line)
                                          ALGO_ID <= 8'h2B;
                                          OBJ_X   <= {data_buf[1], data_buf[0]}; // xOrigin
                                          OBJ_Y   <= {data_buf[3], data_buf[2]}; // yOrigin
                                          OBJ_W   <= {data_buf[5], data_buf[4]}; // xTarget
                                          OBJ_H   <= {data_buf[7], data_buf[6]}; // yTarget
                                          OBJ_ID  <= data_buf[8];
                                          DATA_VALID <= 1;
                                      end
                                      default: ; // 0x2E OK 등은 무시
                                  endcase
                              end
                              chksum_calc <= 0;
                          end

                    default: state <= S_H1;
                endcase
            end
        end
    end
endmodule
