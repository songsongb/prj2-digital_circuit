// =============================================================
// huskylens_sensor_core.v
//
// Pure RTL HuskyLens sensor core.
// - No RV32I, IMEM, DMEM_MMIO, or firmware instantiation.
// - Reuses the existing working UART_RX, UART_TX, HL_PARSER, TIMER.
// - Sends the same command packets used by sw/main.c.
// =============================================================
module huskylens_sensor_core #(
    parameter DEBUG_SLOW = 0,
    parameter SLOW_TICK_MAX = 50000000
) (
    input        clk,
    input        rstn,

    // HuskyLens UART physical pins
    input        hl_uart_rx,
    output       hl_uart_tx,

    // Optional debug PC UART output. PC RX is reserved for later use.
    input        pc_uart_rx,
    output       pc_uart_tx,

    // Controls
    input        enable,
    input        manual_mode,
    input        start_scan,
    input        debug_slow_mode,

    // Current polling mode
    output reg [1:0] current_mode,
    output           mode_tick,

    // Parser/result status. data_valid/no_result are 1-clock pulses.
    output reg       data_valid,
    output reg       no_result,
    output reg [7:0] algo,
    output reg [7:0] id,
    output reg [7:0] x,
    output reg [7:0] y,
    output reg [15:0] w,
    output reg [15:0] h,
    output reg [7:0] obj_count,

    // Line tracking result
    output reg [2:0] line_cmd,

    // Debug
    output           uart_rx_activity,
    output           frame_seen,
    output           header_seen,
    output [7:0]     rx_count,
    output [7:0]     debug_state
);
    localparam MODE_LINE   = 2'd0;
    localparam MODE_TAG    = 2'd1;
    localparam MODE_OBJECT = 2'd2;

    localparam CMD_STOP  = 3'd0;
    localparam CMD_GO    = 3'd1;
    localparam CMD_LEFT  = 3'd2;
    localparam CMD_RIGHT = 3'd3;
    localparam CMD_BACK  = 3'd4;

    localparam ALGO_OBJECT = 8'd2;
    localparam ALGO_LINE   = 8'd3;
    localparam ALGO_TAG    = 8'd5;

    localparam HL_BAUD_DIV = 5208;
    localparam PC_BAUD_DIV = 5208;

    localparam TX_SET_ALGO = 1'b0;
    localparam TX_REQUEST  = 1'b1;

    localparam S_IDLE      = 4'd0;
    localparam S_LOAD_SET  = 4'd1;
    localparam S_LOAD_REQ  = 4'd2;
    localparam S_SEND      = 4'd3;
    localparam S_WAIT_BUSY = 4'd4;
    localparam S_WAIT_DONE = 4'd5;
    localparam S_NEXT      = 4'd6;

    wire reset_h = ~rstn;

    wire timer_flag;
    reg  timer_clear;
    reg  timer_flag_d;
    wire normal_mode_tick;
    reg  slow_mode_tick;
    reg [31:0] slow_tick_count;
    wire use_slow_tick;
    assign use_slow_tick = debug_slow_mode | (DEBUG_SLOW != 0);
    assign normal_mode_tick = timer_flag & ~timer_flag_d;
    assign mode_tick = use_slow_tick ? slow_mode_tick : normal_mode_tick;

    wire [7:0] hl_rx_data;
    wire       hl_rx_valid;
    reg  [7:0] hl_tx_data;
    reg        hl_tx_send;
    wire       hl_tx_busy;

    wire [7:0]  parser_algo;
    wire [7:0]  parser_id;
    wire [15:0] parser_x;
    wire [15:0] parser_y;
    wire [15:0] parser_w;
    wire [15:0] parser_h;
    wire [7:0]  parser_count;
    wire        parser_data_valid;
    wire        parser_no_result;

    reg [2:0] cycle_slot;
    reg [3:0] tx_state;
    reg       packet_kind;
    reg [1:0] packet_mode;
    reg [3:0] packet_len;
    reg [3:0] byte_index;
    reg       request_after_set;
    reg       algo_configured;
    reg       start_scan_d;

    reg        byte_seen_reg;
    reg        frame_seen_reg;
    reg        header_seen_reg;
    reg [7:0]  rx_count_reg;
    reg [1:0]  hdr_state;
    wire       unused_pc_rx;

    wire scan_start_pulse = start_scan & ~start_scan_d;
    wire run_enable = enable & ((manual_mode & start_scan) |
                                (~manual_mode & (start_scan | algo_configured)));

    assign unused_pc_rx = pc_uart_rx;
    assign uart_rx_activity = byte_seen_reg;
    assign frame_seen = frame_seen_reg;
    assign header_seen = header_seen_reg;
    assign rx_count = rx_count_reg;
    assign debug_state = {tx_state[3:0], 1'b0, cycle_slot[2:0]};

    // PC debug keeps the existing working behavior: echo HuskyLens RX bytes.
    UART_TX #(.BAUD_DIV(PC_BAUD_DIV)) uart_pc_tx (
        .CLK(clk), .RESET(reset_h), .DATA(hl_rx_data), .SEND(hl_rx_valid),
        .TX(pc_uart_tx), .BUSY()
    );

    UART_RX #(.BAUD_DIV(HL_BAUD_DIV)) uart_hl_rx (
        .CLK(clk), .RESET(reset_h), .RX(hl_uart_rx),
        .DATA(hl_rx_data), .VALID(hl_rx_valid)
    );

    UART_TX #(.BAUD_DIV(HL_BAUD_DIV)) uart_hl_tx (
        .CLK(clk), .RESET(reset_h), .DATA(hl_tx_data), .SEND(hl_tx_send),
        .TX(hl_uart_tx), .BUSY(hl_tx_busy)
    );

    HL_PARSER hl_parser (
        .CLK(clk), .RESET(reset_h),
        .RX_DATA(hl_rx_data), .RX_VALID(hl_rx_valid),
        .ALGO_ID(parser_algo), .OBJ_ID(parser_id),
        .OBJ_X(parser_x), .OBJ_Y(parser_y),
        .OBJ_W(parser_w), .OBJ_H(parser_h),
        .OBJ_COUNT(parser_count),
        .DATA_VALID(parser_data_valid), .NO_RESULT(parser_no_result)
    );

    TIMER timer_inst (
        .CLK(clk), .RESET(reset_h), .CLEAR(timer_clear), .FLAG(timer_flag)
    );

    function [1:0] schedule_mode;
        input [2:0] slot;
        begin
            case (slot)
                3'd0: schedule_mode = MODE_LINE;
                3'd1: schedule_mode = MODE_LINE;
                3'd2: schedule_mode = MODE_TAG;
                3'd3: schedule_mode = MODE_LINE;
                3'd4: schedule_mode = MODE_LINE;
                3'd5: schedule_mode = MODE_OBJECT;
                default: schedule_mode = MODE_LINE;
            endcase
        end
    endfunction

    function [7:0] mode_algo;
        input [1:0] mode;
        begin
            case (mode)
                MODE_LINE:   mode_algo = ALGO_LINE;
                MODE_TAG:    mode_algo = ALGO_TAG;
                MODE_OBJECT: mode_algo = ALGO_OBJECT;
                default:     mode_algo = ALGO_LINE;
            endcase
        end
    endfunction

    function [7:0] set_algo_checksum;
        input [1:0] mode;
        begin
            case (mode)
                MODE_LINE:   set_algo_checksum = 8'h42;
                MODE_TAG:    set_algo_checksum = 8'h44;
                MODE_OBJECT: set_algo_checksum = 8'h41;
                default:     set_algo_checksum = 8'h42;
            endcase
        end
    endfunction

    function [7:0] packet_byte;
        input       kind;
        input [1:0] mode;
        input [3:0] index;
        begin
            if (kind == TX_REQUEST) begin
                case (index)
                    4'd0: packet_byte = 8'h55;
                    4'd1: packet_byte = 8'hAA;
                    4'd2: packet_byte = 8'h11;
                    4'd3: packet_byte = 8'h00;
                    4'd4: packet_byte = 8'h20;
                    4'd5: packet_byte = 8'h30;
                    default: packet_byte = 8'h00;
                endcase
            end else begin
                case (index)
                    4'd0: packet_byte = 8'h55;
                    4'd1: packet_byte = 8'hAA;
                    4'd2: packet_byte = 8'h11;
                    4'd3: packet_byte = 8'h02;
                    4'd4: packet_byte = 8'h2D;
                    4'd5: packet_byte = mode_algo(mode);
                    4'd6: packet_byte = 8'h00;
                    4'd7: packet_byte = set_algo_checksum(mode);
                    default: packet_byte = 8'h00;
                endcase
            end
        end
    endfunction

    always @(posedge clk) begin
        if (reset_h) begin
            current_mode      <= MODE_LINE;
            cycle_slot        <= 3'd0;
            tx_state          <= S_IDLE;
            packet_kind       <= TX_SET_ALGO;
            packet_mode       <= MODE_LINE;
            packet_len        <= 4'd0;
            byte_index        <= 4'd0;
            request_after_set <= 1'b0;
            algo_configured   <= 1'b0;
            hl_tx_data        <= 8'd0;
            hl_tx_send        <= 1'b0;
            timer_clear       <= 1'b0;
            timer_flag_d      <= 1'b0;
            slow_mode_tick    <= 1'b0;
            slow_tick_count   <= 32'd0;
            start_scan_d      <= 1'b0;
        end else begin
            hl_tx_send   <= 1'b0;
            timer_clear  <= timer_flag;
            timer_flag_d <= timer_flag;
            slow_mode_tick <= 1'b0;
            start_scan_d <= start_scan;

            if (!enable) begin
                slow_tick_count <= 32'd0;
            end else if (use_slow_tick) begin
                if (slow_tick_count == SLOW_TICK_MAX - 1) begin
                    slow_tick_count <= 32'd0;
                    slow_mode_tick  <= 1'b1;
                end else begin
                    slow_tick_count <= slow_tick_count + 1'b1;
                end
            end

            if (!enable) begin
                tx_state        <= S_IDLE;
                algo_configured <= 1'b0;
            end else if (scan_start_pulse && tx_state == S_IDLE) begin
                packet_mode       <= current_mode;
                packet_kind       <= TX_SET_ALGO;
                packet_len        <= 4'd8;
                byte_index        <= 4'd0;
                request_after_set <= 1'b1;
                algo_configured   <= 1'b1;
                tx_state          <= S_SEND;
            end else begin
                case (tx_state)
                    S_IDLE: begin
                        if (mode_tick && run_enable) begin
                            packet_mode <= schedule_mode(cycle_slot);
                            current_mode <= schedule_mode(cycle_slot);
                            if (cycle_slot == 3'd5) cycle_slot <= 3'd0;
                            else                    cycle_slot <= cycle_slot + 1'b1;

                            if (!algo_configured ||
                                (schedule_mode(cycle_slot) != current_mode)) begin
                                tx_state          <= S_LOAD_SET;
                                request_after_set <= 1'b1;
                                algo_configured   <= 1'b1;
                            end else begin
                                tx_state          <= S_LOAD_REQ;
                                request_after_set <= 1'b0;
                            end
                        end
                    end

                    S_LOAD_SET: begin
                        packet_kind <= TX_SET_ALGO;
                        packet_len  <= 4'd8;
                        byte_index  <= 4'd0;
                        tx_state    <= S_SEND;
                    end

                    S_LOAD_REQ: begin
                        packet_kind <= TX_REQUEST;
                        packet_len  <= 4'd6;
                        byte_index  <= 4'd0;
                        tx_state    <= S_SEND;
                    end

                    S_SEND: begin
                        if (!hl_tx_busy) begin
                            hl_tx_data <= packet_byte(packet_kind, packet_mode, byte_index);
                            hl_tx_send <= 1'b1;
                            tx_state   <= S_WAIT_BUSY;
                        end
                    end

                    S_WAIT_BUSY: begin
                        tx_state <= S_WAIT_DONE;
                    end

                    S_WAIT_DONE: begin
                        if (!hl_tx_busy) begin
                            tx_state <= S_NEXT;
                        end
                    end

                    S_NEXT: begin
                        if (byte_index == packet_len - 1'b1) begin
                            if (request_after_set && packet_kind == TX_SET_ALGO) begin
                                request_after_set <= 1'b0;
                                tx_state <= S_LOAD_REQ;
                            end else begin
                                tx_state <= S_IDLE;
                            end
                        end else begin
                            byte_index <= byte_index + 1'b1;
                            tx_state <= S_SEND;
                        end
                    end

                    default: tx_state <= S_IDLE;
                endcase
            end
        end
    end

    always @(posedge clk) begin
        if (reset_h) begin
            data_valid <= 1'b0;
            no_result  <= 1'b0;
            algo       <= 8'd0;
            id         <= 8'd0;
            x          <= 8'd0;
            y          <= 8'd0;
            w          <= 16'd0;
            h          <= 16'd0;
            obj_count  <= 8'd0;
            line_cmd   <= CMD_STOP;
        end else begin
            data_valid <= 1'b0;
            no_result  <= 1'b0;

            if (parser_data_valid) begin
                data_valid <= 1'b1;
                algo       <= parser_algo;
                id         <= parser_id;
                x          <= parser_x[7:0];
                y          <= parser_y[7:0];
                w          <= parser_w;
                h          <= parser_h;
                obj_count  <= parser_count;

                if (current_mode == MODE_LINE && parser_algo == 8'h2B) begin
                    if (parser_w >= 16'd130 && parser_w <= 16'd190)
                        line_cmd <= CMD_GO;
                    else if (parser_w > 16'd190)
                        line_cmd <= CMD_LEFT;
                    else
                        line_cmd <= CMD_RIGHT;
                end
            end

            if (parser_no_result) begin
                no_result <= 1'b1;
                obj_count <= parser_count;
                if (current_mode == MODE_LINE)
                    line_cmd <= CMD_STOP;
            end
        end
    end

    always @(posedge clk) begin
        if (reset_h) begin
            byte_seen_reg   <= 1'b0;
            frame_seen_reg  <= 1'b0;
            header_seen_reg <= 1'b0;
            rx_count_reg    <= 8'd0;
            hdr_state       <= 2'd0;
        end else begin
            if (hl_rx_valid) begin
                byte_seen_reg <= 1'b1;
                rx_count_reg  <= rx_count_reg + 1'b1;
                case (hdr_state)
                    2'd0: hdr_state <= (hl_rx_data == 8'h55) ? 2'd1 : 2'd0;
                    2'd1: hdr_state <= (hl_rx_data == 8'hAA) ? 2'd2 :
                                       (hl_rx_data == 8'h55) ? 2'd1 : 2'd0;
                    2'd2: begin
                        if (hl_rx_data == 8'h11) header_seen_reg <= 1'b1;
                        hdr_state <= (hl_rx_data == 8'h55) ? 2'd1 : 2'd0;
                    end
                    default: hdr_state <= 2'd0;
                endcase
            end

            if (parser_data_valid | parser_no_result)
                frame_seen_reg <= 1'b1;
        end
    end

endmodule
