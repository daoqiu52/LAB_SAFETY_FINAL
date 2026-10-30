module seg_scan(
	input           clk,
	input           rst_n,
	output reg[5:0] seg_sel,
	output reg[7:0] seg_data,
	input[7:0]      seg_data_0,
	input[7:0]      seg_data_1,
	input[7:0]      seg_data_2,
	input[7:0]      seg_data_3,
	input[7:0]      seg_data_4,
	input[7:0]      seg_data_5
);
parameter SCAN_FREQ = 200;
parameter CLK_FREQ = 50000000;

parameter SCAN_COUNT = CLK_FREQ /(SCAN_FREQ * 6) - 1;

reg[31:0] scan_timer;
reg[3:0] scan_sel;
always@(posedge clk or negedge rst_n)
begin
	if(rst_n == 1'b0)
	begin
		scan_timer <= 32'd0;
		scan_sel <= 4'd0;
	end
	else if(scan_timer >= SCAN_COUNT)
	begin
		scan_timer <= 32'd0;
		if(scan_sel == 4'd5)
			scan_sel <= 4'd0;
		else
			scan_sel <= scan_sel + 4'd1;
	end
	else
		begin
			scan_timer <= scan_timer + 32'd1;
		end
end
always@(posedge clk or negedge rst_n)
begin
	if(rst_n == 1'b0)
	begin
		seg_sel <= 6'b111111;
		seg_data <= 8'hff;
	end
	else
	begin
		case(scan_sel)
			4'd0:
			begin
				seg_sel <= 6'b11_1110;
				seg_data <= seg_data_0;
			end
			4'd1:
			begin
				seg_sel <= 6'b11_1101;
				seg_data <= seg_data_1;
			end
			4'd2:
			begin
				seg_sel <= 6'b11_1011;
				seg_data <= seg_data_2;
			end
			4'd3:
			begin
				seg_sel <= 6'b11_0111;
				seg_data <= seg_data_3;
			end
			4'd4:
			begin
				seg_sel <= 6'b10_1111;
				seg_data <= seg_data_4;
			end
			4'd5:
			begin
				seg_sel <= 6'b01_1111;
				seg_data <= seg_data_5;
			end
			default:
			begin
				seg_sel <= 6'b11_1111;
				seg_data <= 8'hff;
			end
		endcase
	end
end

endmodule

// Included in seg_scan.v so existing TD source lists need no new RTL entry.
// Inputs and event pulses must be synchronous to clk. dig_n[0] is leftmost.
module seg7_panel #(
    parameter integer DIGITS = 8, // supported: 6 or 8
    parameter integer STARTUP_DIAGNOSTICS = 0,
    parameter integer ERROR_DISPLAY = 0,
    parameter integer ERROR_DETAIL = 0,
    parameter integer WRITE_DIAGNOSTICS = 0,
    parameter integer FAT_DIAGNOSTICS = 0,
    parameter integer SCAN_STEP_CYCLES = 8192,
    parameter integer PAGE_PERIOD_CYCLES = 50000000,
    parameter integer HOLD_CYCLES = 100000000
)(
    input wire clk,
    input wire rst,
    input wire picture_valid,
    input wire [3:0] picture_number,
    input wire [7:0] volume_level,
    input wire [7:0] brightness_percent, // 0..100 percent
    input wire picture_event,
    input wire volume_event,
    output reg [DIGITS-1:0] dig_n,
    output reg [7:0] seg_n,
    input wire [2:0] error_code,
    input wire [2:0] error_stage,
    input wire [3:0] error_detail,
    input wire [7:0] fs_error
);
localparam [1:0] PIC = 0, VOL = 1, BRI = 2;
reg [1:0] page;
reg [31:0] page_count, hold_count, scan_count;
reg [2:0] scan_index;
reg [7:0] selected_segment;
wire [11:0] volume_bcd = to_bcd(volume_level);
wire [11:0] brightness_bcd = to_bcd(brightness_percent);

// Eight shift/add-3 steps; no divider is inferred.
function [11:0] to_bcd;
    input [7:0] value;
    reg [19:0] shift;
    integer i;
    begin
        shift = {12'd0, value};
        for (i = 0; i < 8; i = i + 1) begin
            if (shift[11:8] >= 5) shift[11:8] = shift[11:8] + 4'd3;
            if (shift[15:12] >= 5) shift[15:12] = shift[15:12] + 4'd3;
            if (shift[19:16] >= 5) shift[19:16] = shift[19:16] + 4'd3;
            shift = shift << 1;
        end
        to_bcd = shift[19:8];
    end
endfunction

function [7:0] decimal_segment;
    input [3:0] value;
    begin
        case (value)
            0: decimal_segment = 8'hc0;
            1: decimal_segment = 8'hf9;
            2: decimal_segment = 8'ha4;
            3: decimal_segment = 8'hb0;
            4: decimal_segment = 8'h99;
            5: decimal_segment = 8'h92;
            6: decimal_segment = 8'h82;
            7: decimal_segment = 8'hf8;
            8: decimal_segment = 8'h80;
            9: decimal_segment = 8'h90;
            default: decimal_segment = 8'hff;
        endcase
    end
endfunction

always @(posedge clk or posedge rst) begin
    if (rst) begin
        page <= PIC;
        page_count <= 0;
        hold_count <= 0;
    end else if (volume_event || picture_event) begin
        // Volume wins simultaneous events; every event restarts the hold.
        page <= volume_event ? VOL : PIC;
        hold_count <= HOLD_CYCLES;
        page_count <= 0;
    end else if (hold_count != 0) begin
        hold_count <= hold_count - 1'b1;
        page_count <= 0;
        if (hold_count == 1)
            page <= (page == BRI) ? PIC : page + 1'b1;
    end else if (page_count >= PAGE_PERIOD_CYCLES - 1) begin
        page_count <= 0;
        page <= (page == BRI) ? PIC : page + 1'b1;
    end else begin
        page_count <= page_count + 1'b1;
    end
end

// When enabled, hold the startup page until a real image is committed.
// picture_number carries the startup stage while picture_valid is low.

always @(posedge clk or posedge rst) begin
    if (rst) begin
        scan_count <= 0;
        scan_index <= 0;
        dig_n <= {DIGITS{1'b1}};
        seg_n <= 8'hff;
    end else begin
        if (scan_count >= SCAN_STEP_CYCLES - 1) begin
            scan_count <= 0;
            scan_index <= (scan_index == DIGITS - 1) ? 3'd0 : scan_index + 1'b1;
        end else begin
            scan_count <= scan_count + 1'b1;
        end
        // Register the digit and its segments together.
        dig_n <= ~({{(DIGITS-1){1'b0}}, 1'b1} << scan_index);
        seg_n <= selected_segment;
    end
end

// 8 digits: "    PICn", "VOL  ddd", "brI  ddd".
// 6 digits: "  PICn",   "VOLddd",   "brIddd" (brightness percent).
// Seven-segment approximations: V uses U, B uses b, R uses r, I uses 1.
always @* begin
    selected_segment = 8'hff;
    case (page)
        PIC: begin
            if (scan_index == DIGITS-4) selected_segment = 8'h8c; // P
            if (scan_index == DIGITS-3) selected_segment = 8'hf9; // I
            if (scan_index == DIGITS-2) selected_segment = 8'hc6; // C
            if (scan_index == DIGITS-1)
                selected_segment = picture_valid ? decimal_segment(picture_number) : 8'hbf;
        end
        VOL: begin
            if (scan_index == 0) selected_segment = 8'hc1; // V/U
            if (scan_index == 1) selected_segment = 8'hc0; // O
            if (scan_index == 2) selected_segment = 8'hc7; // L
            if (scan_index == DIGITS-3) selected_segment = decimal_segment(volume_bcd[11:8]);
            if (scan_index == DIGITS-2) selected_segment = decimal_segment(volume_bcd[7:4]);
            if (scan_index == DIGITS-1) selected_segment = decimal_segment(volume_bcd[3:0]);
        end
        BRI: begin
            if (scan_index == 0) selected_segment = 8'h83; // b
            if (scan_index == 1) selected_segment = 8'haf; // r
            if (scan_index == 2) selected_segment = 8'hf9; // I
            if (scan_index == DIGITS-3) selected_segment = decimal_segment(brightness_bcd[11:8]);
            if (scan_index == DIGITS-2) selected_segment = decimal_segment(brightness_bcd[7:4]);
            if (scan_index == DIGITS-1) selected_segment = decimal_segment(brightness_bcd[3:0]);
        end
        default: selected_segment = 8'hff;
    endcase
    if (STARTUP_DIAGNOSTICS && !picture_valid) begin
        selected_segment = 8'hff;
        if (scan_index == DIGITS-3) selected_segment = 8'h92; // S
        if (scan_index == DIGITS-2) selected_segment = 8'ha1; // d
        if (scan_index == DIGITS-1) selected_segment = decimal_segment(picture_number);
    end
    if(ERROR_DISPLAY && error_code!=0) begin
        selected_segment=8'hff;
        if(scan_index==DIGITS-3) selected_segment=8'h86; // E
        if(scan_index==DIGITS-2) selected_segment=8'haf; // r
        if(scan_index==DIGITS-1) selected_segment=decimal_segment({1'b0,error_code});
        if(ERROR_DETAIL) begin
            // Six digits: "Er2-3 "; retain the error and show its latched stage.
            selected_segment=8'hff;
            if(scan_index==DIGITS-6) selected_segment=8'h86;
            if(scan_index==DIGITS-5) selected_segment=8'haf;
            if(scan_index==DIGITS-4) selected_segment=decimal_segment({1'b0,error_code});
            if(scan_index==DIGITS-3) selected_segment=8'hbf;
            if(scan_index==DIGITS-2) selected_segment=decimal_segment({1'b0,error_stage});
            if(WRITE_DIAGNOSTICS && error_stage==4 && error_detail!=0 && scan_index==DIGITS-1)
                selected_segment=decimal_segment(error_detail);
        end
    end
    if(FAT_DIAGNOSTICS && fs_error!=0) begin
        selected_segment=8'hff;
        if(scan_index==DIGITS-4) selected_segment=8'h8e; // F
        if(scan_index==DIGITS-3) selected_segment=8'h92; // S
        if(scan_index==DIGITS-2) selected_segment=decimal_segment(fs_error[7:4]);
        if(scan_index==DIGITS-1) selected_segment=decimal_segment(fs_error[3:0]);
    end
end
endmodule

// Temporary board diagnostics. Inputs are sticky activity bits or stable status.
module black_screen_diag #(
    parameter integer PAGE_CYCLES = 50000000,
    parameter integer SCAN_CYCLES = 8192
)(
    input wire clk, rst,
    input wire [5:0] status_async,
    input wire de,
    input wire [23:0] picture_rgb,
    output wire [23:0] rgb,
    output reg [5:0] dig_n,
    output reg [7:0] seg_n
);
reg [5:0] status_meta, status_sync;
reg [31:0] page_count, scan_count;
reg test_page;
reg [2:0] digit_index;
reg [9:0] x;
// Red left half, green right half. Use the existing active-video timing.
assign rgb = test_page ? (x < 320 ? 24'hff0000 : 24'h00ff00) : picture_rgb;
always @(posedge clk or posedge rst) begin
    if(rst) begin
        status_meta <= 0; status_sync <= 0;
        page_count <= 0; scan_count <= 0;
        test_page <= 0; digit_index <= 0; x <= 0;
        dig_n <= 6'h3f; seg_n <= 8'hff;
    end else begin
        status_meta <= status_async;
        status_sync <= status_meta;
        if(!de) x <= 0;
        else if(x < 639) x <= x + 1'b1;
        if(page_count == PAGE_CYCLES-1) begin
            page_count <= 0; test_page <= ~test_page;
        end else page_count <= page_count + 1'b1;
        if(scan_count == SCAN_CYCLES-1) begin
            scan_count <= 0;
            digit_index <= digit_index == 5 ? 3'd0 : digit_index + 1'b1;
        end else scan_count <= scan_count + 1'b1;
        dig_n <= ~(6'b1 << digit_index);
        seg_n <= status_sync[5-digit_index] ? 8'hf9 : 8'hc0;
    end
end
endmodule
