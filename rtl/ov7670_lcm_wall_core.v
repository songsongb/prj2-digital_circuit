// --------------------------------------------------------------------
// OV7670 camera + TRDB_LCM display + wall detection core
//
// GPIO_0 / JP1 : TRDB_LCM
// GPIO_1 / JP2 : OV7670
//
// GPIO_1[4]    = OV7670_XCLK output
// GPIO_1[5]    = OV7670_PCLK input
// GPIO_1[6]    = OV7670_HREF input
// GPIO_1[7]    = OV7670_VSYNC input
// GPIO_1[15:8] = OV7670_D[7:0] input
// --------------------------------------------------------------------

module ov7670_lcm_wall_core (
    input        CLOCK_50,
    input        rstn,

    inout [35:0] GPIO_0,
    inout [35:0] GPIO_1,

    input [1:0]  display_mode,
    input        byte_phase_sel,

    output       ov_pclk_alive,
    output       ov_vsync,
    output       ov_href,
    output       wall_detect,
    output [7:0] front_avg_dbg,
    output [7:0] floor_avg_dbg,
    output       wall_update_toggle,
    output [7:0] camera_pixel_dbg
);

// ============================================================
// TRDB_LCM signal declaration and GPIO_0 / JP1 mapping
// ============================================================

wire    [7:0]   LCM_DATA;
wire            LCM_GRST;
wire            LCM_SHDB;
wire            LCM_DCLK;
wire            LCM_HSYNC;
wire            LCM_VSYNC;
wire            LCM_SCLK;
wire            LCM_SDAT;
wire            LCM_SCEN;

assign GPIO_0 = 36'hzzzzzzzzz;
assign GPIO_1 = 36'hzzzzzzzzz;

assign GPIO_0[18] = LCM_DATA[6];
assign GPIO_0[19] = LCM_DATA[7];
assign GPIO_0[20] = LCM_DATA[4];
assign GPIO_0[21] = LCM_DATA[5];
assign GPIO_0[22] = LCM_DATA[2];
assign GPIO_0[23] = LCM_DATA[3];
assign GPIO_0[24] = LCM_DATA[0];
assign GPIO_0[25] = LCM_DATA[1];

assign GPIO_0[26] = LCM_VSYNC;
assign GPIO_0[28] = LCM_SCLK;
assign GPIO_0[29] = LCM_DCLK;
assign GPIO_0[30] = LCM_GRST;
assign GPIO_0[31] = LCM_SHDB;
assign GPIO_0[33] = LCM_SCEN;
assign GPIO_0[34] = LCM_SDAT;
assign GPIO_0[35] = LCM_HSYNC;

assign LCM_GRST = rstn;
assign LCM_SHDB = 1'b1;

// ============================================================
// OV7670 GPIO_1 / JP2 mapping
// ============================================================

wire        OV7670_PCLK;
wire        OV7670_HREF;
wire        OV7670_VSYNC;
wire [7:0]  OV7670_D;
reg         OV7670_XCLK;

assign GPIO_1[4] = OV7670_XCLK;

assign OV7670_PCLK  = GPIO_1[5];
assign OV7670_HREF  = GPIO_1[6];
assign OV7670_VSYNC = GPIO_1[7];
assign OV7670_D     = GPIO_1[15:8];

assign ov_vsync         = OV7670_VSYNC;
assign ov_href          = OV7670_HREF;

// ============================================================
// OV7670 XCLK generation
// CLOCK_50 / 2 = 25 MHz
// ============================================================

always @(posedge CLOCK_50 or negedge rstn) begin
    if (!rstn)
        OV7670_XCLK <= 1'b0;
    else
        OV7670_XCLK <= ~OV7670_XCLK;
end

// ============================================================
// OV7670 low-resolution grayscale frame buffer
// ============================================================

parameter LOW_W        = 160;
parameter LOW_H        = 120;
parameter LOW_PIXELS   = 19200;
parameter CAM_X_SAMPLE = 2;
parameter CAM_Y_SAMPLE = 2;

reg [7:0] frame_buf [0:LOW_PIXELS-1];

reg [7:0]  camera_pixel;
reg [23:0] pclk_counter;
reg        pclk_alive;
reg [10:0] cam_x_count;
reg [10:0] cam_y_count;
reg [2:0]  cam_x_sample_count;
reg [2:0]  cam_y_sample_count;
reg [7:0]  cam_low_x;
reg [6:0]  cam_low_y;
reg        cam_href_d;
reg        byte_phase;

assign ov_pclk_alive    = pclk_alive;
assign camera_pixel_dbg = camera_pixel;

// ============================================================
// OV7670 wall detection
// ============================================================

parameter FRONT_X_MIN = 8'd40;
parameter FRONT_X_MAX = 8'd119;
parameter FRONT_Y_MIN = 7'd20;
parameter FRONT_Y_MAX = 7'd70;

parameter FLOOR_X_MIN = 8'd40;
parameter FLOOR_X_MAX = 8'd119;
parameter FLOOR_Y_MIN = 7'd85;
parameter FLOOR_Y_MAX = 7'd115;

parameter FRONT_SHIFT = 4'd12;
parameter FLOOR_SHIFT = 4'd11;
parameter WALL_ON_ABS_TH   = 8'd110;
parameter WALL_ON_DIFF_TH  = 8'd50;
parameter WALL_OFF_ABS_TH  = 8'd90;
parameter WALL_OFF_DIFF_TH = 8'd25;
parameter WALL_DEBOUNCE_FRAMES = 2'd3;

reg [23:0] front_sum;
reg [23:0] floor_sum;
reg [7:0]  front_avg_reg;
reg [7:0]  floor_avg_reg;
reg        wall_detect_reg;
reg        wall_update_toggle_reg;
reg        cam_vsync_d;
reg [1:0]  wall_on_cnt;
reg [1:0]  wall_off_cnt;

wire sampled_in_front_roi;
wire sampled_in_floor_roi;
wire [11:0] front_avg_shifted;
wire [12:0] floor_avg_shifted;
wire [7:0]  front_avg_calc;
wire [7:0]  floor_avg_calc;
wire [8:0]  front_avg_calc_ext;
wire [8:0]  floor_avg_calc_ext;
wire        wall_on_raw;
wire        wall_off_raw;

assign wall_detect        = wall_detect_reg;
assign front_avg_dbg      = front_avg_reg;
assign floor_avg_dbg      = floor_avg_reg;
assign wall_update_toggle = wall_update_toggle_reg;

assign sampled_in_front_roi =
        (cam_low_x >= FRONT_X_MIN) && (cam_low_x <= FRONT_X_MAX) &&
        (cam_low_y >= FRONT_Y_MIN) && (cam_low_y <= FRONT_Y_MAX);

assign sampled_in_floor_roi =
        (cam_low_x >= FLOOR_X_MIN) && (cam_low_x <= FLOOR_X_MAX) &&
        (cam_low_y >= FLOOR_Y_MIN) && (cam_low_y <= FLOOR_Y_MAX);

assign front_avg_shifted = front_sum >> FRONT_SHIFT;
assign floor_avg_shifted = floor_sum >> FLOOR_SHIFT;

assign front_avg_calc = (|front_avg_shifted[11:8]) ? 8'hFF :
                                                  front_avg_shifted[7:0];
assign floor_avg_calc = (|floor_avg_shifted[12:8]) ? 8'hFF :
                                                  floor_avg_shifted[7:0];

assign front_avg_calc_ext = {1'b0, front_avg_calc};
assign floor_avg_calc_ext = {1'b0, floor_avg_calc};

assign wall_on_raw =
        (front_avg_calc_ext > {1'b0, WALL_ON_ABS_TH}) &&
        (front_avg_calc_ext > (floor_avg_calc_ext + {1'b0, WALL_ON_DIFF_TH}));

assign wall_off_raw =
        (front_avg_calc_ext < {1'b0, WALL_OFF_ABS_TH}) ||
        (front_avg_calc_ext < (floor_avg_calc_ext + {1'b0, WALL_OFF_DIFF_TH}));

function [14:0] low_buf_addr;
    input [6:0] y;
    input [7:0] x;
    begin
        low_buf_addr = ({8'd0, y} << 7) + ({8'd0, y} << 5) + {7'd0, x};
    end
endfunction

always @(posedge OV7670_PCLK or negedge rstn) begin
    if (!rstn) begin
        camera_pixel           <= 8'd0;
        pclk_counter           <= 24'd0;
        pclk_alive             <= 1'b0;
        cam_x_count            <= 11'd0;
        cam_y_count            <= 11'd0;
        cam_x_sample_count     <= 3'd0;
        cam_y_sample_count     <= 3'd0;
        cam_low_x              <= 8'd0;
        cam_low_y              <= 7'd0;
        cam_href_d             <= 1'b0;
        byte_phase             <= 1'b0;
        front_sum              <= 24'd0;
        floor_sum              <= 24'd0;
        front_avg_reg          <= 8'd0;
        floor_avg_reg          <= 8'd0;
        wall_detect_reg        <= 1'b0;
        wall_update_toggle_reg <= 1'b0;
        cam_vsync_d            <= 1'b0;
        wall_on_cnt            <= 2'd0;
        wall_off_cnt           <= 2'd0;
    end else begin
        cam_href_d  <= OV7670_HREF;
        cam_vsync_d <= OV7670_VSYNC;

        if (pclk_counter == 24'd5_000_000) begin
            pclk_counter <= 24'd0;
            pclk_alive   <= ~pclk_alive;
        end else begin
            pclk_counter <= pclk_counter + 24'd1;
        end

        if (OV7670_VSYNC) begin
            if (!cam_vsync_d) begin
                front_avg_reg          <= front_avg_calc;
                floor_avg_reg          <= floor_avg_calc;

                if (wall_on_raw) begin
                    if (wall_on_cnt < WALL_DEBOUNCE_FRAMES)
                        wall_on_cnt <= wall_on_cnt + 1'b1;
                end else begin
                    wall_on_cnt <= 2'd0;
                end

                if (wall_off_raw) begin
                    if (wall_off_cnt < WALL_DEBOUNCE_FRAMES)
                        wall_off_cnt <= wall_off_cnt + 1'b1;
                end else begin
                    wall_off_cnt <= 2'd0;
                end

                if (!wall_detect_reg) begin
                    if (wall_on_raw && (wall_on_cnt >= (WALL_DEBOUNCE_FRAMES - 1'b1)))
                        wall_detect_reg <= 1'b1;
                end else begin
                    if (wall_off_raw && (wall_off_cnt >= (WALL_DEBOUNCE_FRAMES - 1'b1)))
                        wall_detect_reg <= 1'b0;
                end

                wall_update_toggle_reg <= ~wall_update_toggle_reg;
            end

            cam_x_count        <= 11'd0;
            cam_y_count        <= 11'd0;
            cam_x_sample_count <= 3'd0;
            cam_y_sample_count <= 3'd0;
            cam_low_x          <= 8'd0;
            cam_low_y          <= 7'd0;
            byte_phase         <= 1'b0;
            front_sum          <= 24'd0;
            floor_sum          <= 24'd0;
        end else begin
            if (!cam_href_d && OV7670_HREF) begin
                byte_phase         <= 1'b0;
                cam_x_sample_count <= 3'd0;
                cam_low_x          <= 8'd0;
            end

            if (cam_href_d && !OV7670_HREF) begin
                cam_x_count        <= 11'd0;
                cam_x_sample_count <= 3'd0;
                cam_low_x          <= 8'd0;
                byte_phase         <= 1'b0;
                cam_y_count        <= cam_y_count + 11'd1;

                if (cam_y_sample_count == (CAM_Y_SAMPLE - 1)) begin
                    cam_y_sample_count <= 3'd0;
                    if (cam_low_y < (LOW_H - 1))
                        cam_low_y <= cam_low_y + 7'd1;
                end else begin
                    cam_y_sample_count <= cam_y_sample_count + 3'd1;
                end
            end
        end

        if (OV7670_HREF) begin
            camera_pixel <= OV7670_D;

            if (!OV7670_VSYNC) begin
                if (byte_phase == byte_phase_sel) begin
                    if (cam_y_sample_count == 3'd0 &&
                        cam_x_sample_count == 3'd0 &&
                        cam_low_x < LOW_W &&
                        cam_low_y < LOW_H) begin
                        frame_buf[low_buf_addr(cam_low_y, cam_low_x)] <= OV7670_D;

                        if (sampled_in_front_roi)
                            front_sum <= front_sum + {16'd0, OV7670_D};

                        if (sampled_in_floor_roi)
                            floor_sum <= floor_sum + {16'd0, OV7670_D};
                    end

                    cam_x_count <= cam_x_count + 11'd1;

                    if (cam_x_sample_count == (CAM_X_SAMPLE - 1)) begin
                        cam_x_sample_count <= 3'd0;
                        if (cam_low_x < (LOW_W - 1))
                            cam_low_x <= cam_low_x + 8'd1;
                    end else begin
                        cam_x_sample_count <= cam_x_sample_count + 3'd1;
                    end
                end

                byte_phase <= ~byte_phase;
            end
        end
    end
end

// ============================================================
// LCD timing / pattern generator signals
// ============================================================

wire            iCLK;
wire            iRST_N;
reg     [10:0]  H_Cont;
reg     [10:0]  V_Cont;
reg     [7:0]   Tmp_DATA;
reg             oVGA_H_SYNC;
reg             oVGA_V_SYNC;
wire            CLK_25;

assign iCLK       = CLK_25;
assign iRST_N     = rstn;
assign LCM_VSYNC  = oVGA_V_SYNC;
assign LCM_HSYNC  = oVGA_H_SYNC;
assign LCM_DCLK   = ~CLK_25;

parameter H_SYNC_CYC   = 1;
parameter H_SYNC_BACK  = 151;
parameter H_SYNC_ACT   = 960;
parameter H_SYNC_FRONT = 59;
parameter H_SYNC_TOTAL = 1171;

parameter V_SYNC_CYC   = 1;
parameter V_SYNC_BACK  = 13;
parameter V_SYNC_ACT   = 240;
parameter V_SYNC_FRONT = 8;
parameter V_SYNC_TOTAL = 262;

// ============================================================
// TRDB_LCM low-resolution buffer read and spatial scaling
// ============================================================

parameter LCD_SCALE_X = 6;
parameter LCD_SCALE_Y = 2;

wire lcd_active;
wire [10:0] lcd_x;
wire [10:0] lcd_y;
wire [7:0]  low_x_div;
wire [6:0]  low_y_div;
wire [7:0]  low_x_limited;
wire [6:0]  low_y_limited;

reg [14:0]  frame_rd_addr;
reg [7:0]   frame_rd_data;
reg [7:0]   ov7670_spatial_pixel;

assign lcd_active = (H_Cont >= H_SYNC_BACK) &&
                    (H_Cont < (H_SYNC_TOTAL - H_SYNC_FRONT)) &&
                    (V_Cont >= V_SYNC_BACK) &&
                    (V_Cont < (V_SYNC_TOTAL - V_SYNC_FRONT));

assign lcd_x = H_Cont - H_SYNC_BACK;
assign lcd_y = V_Cont - V_SYNC_BACK;

assign low_x_div = lcd_x / LCD_SCALE_X;
assign low_y_div = lcd_y / LCD_SCALE_Y;

assign low_x_limited = (low_x_div >= LOW_W) ? (LOW_W - 1) : low_x_div;
assign low_y_limited = (low_y_div >= LOW_H) ? (LOW_H - 1) : low_y_div;

always @(posedge iCLK or negedge iRST_N) begin
    if (!iRST_N) begin
        frame_rd_addr        <= 15'd0;
        frame_rd_data        <= 8'd0;
        ov7670_spatial_pixel <= 8'd0;
    end else begin
        frame_rd_data <= frame_buf[frame_rd_addr];

        if (lcd_active) begin
            frame_rd_addr        <= low_buf_addr(low_y_limited, low_x_limited);
            ov7670_spatial_pixel <= frame_rd_data;
        end else begin
            frame_rd_addr        <= 15'd0;
            ov7670_spatial_pixel <= 8'h00;
        end
    end
end

assign LCM_DATA = (display_mode == 2'b00) ? Tmp_DATA :
                  (display_mode == 2'b01) ? ov7670_spatial_pixel :
                  (display_mode == 2'b10) ? 8'h7F :
                                            8'hFF;

// ============================================================
// PLL for TRDB_LCM
// ============================================================

LCM_PLL u0 (
    .inclk0 (CLOCK_50),
    .c0     (CLK_25)
);

// ============================================================
// Original pattern generator
// ============================================================

reg [1:0] MOD_3;

always @(posedge iCLK or negedge iRST_N) begin
    if (!iRST_N) begin
        Tmp_DATA <= 8'h00;
        MOD_3   <= 2'b00;
    end else begin
        if (H_Cont > H_SYNC_BACK &&
            H_Cont < (H_SYNC_TOTAL - H_SYNC_FRONT)) begin

            if (MOD_3 < 2'b10)
                MOD_3 <= MOD_3 + 1'b1;
            else
                MOD_3 <= 2'b00;

            Tmp_DATA <= Tmp_DATA + 1'b1;
        end else begin
            MOD_3   <= 2'b00;
            Tmp_DATA <= 8'h00;
        end
    end
end

// ============================================================
// H_Sync Generator
// ============================================================

always @(posedge iCLK or negedge iRST_N) begin
    if (!iRST_N) begin
        H_Cont      <= 11'd0;
        oVGA_H_SYNC <= 1'b0;
    end else begin
        if (H_Cont < H_SYNC_TOTAL)
            H_Cont <= H_Cont + 1'b1;
        else
            H_Cont <= 11'd0;

        if (H_Cont < H_SYNC_CYC)
            oVGA_H_SYNC <= 1'b0;
        else
            oVGA_H_SYNC <= 1'b1;
    end
end

// ============================================================
// V_Sync Generator
// ============================================================

always @(posedge iCLK or negedge iRST_N) begin
    if (!iRST_N) begin
        V_Cont      <= 11'd0;
        oVGA_V_SYNC <= 1'b0;
    end else begin
        if (H_Cont == 11'd0) begin
            if (V_Cont < V_SYNC_TOTAL)
                V_Cont <= V_Cont + 1'b1;
            else
                V_Cont <= 11'd0;

            if (V_Cont < V_SYNC_CYC)
                oVGA_V_SYNC <= 1'b0;
            else
                oVGA_V_SYNC <= 1'b1;
        end
    end
end

// ============================================================
// TRDB_LCM 3-wire configuration
// ============================================================

I2S_LCM_Config u4 (
    .iCLK      (CLOCK_50),
    .iRST_N    (rstn),
    .I2S_SCLK  (LCM_SCLK),
    .I2S_SDAT  (LCM_SDAT),
    .I2S_SCEN  (LCM_SCEN)
);

endmodule
