module video_delay
#(
	parameter DATA_WIDTH = 24,                      // Video data one clock data width
    parameter ENABLE_BRIGHTNESS = 0
)
(
	input                       video_clk,          // Video pixel clock
	input                       rst,
	output                      read_en,            // Read data enable
	input[DATA_WIDTH - 1:0]     read_data,          // Read data
	input                      hs,                 // horizontal synchronization
	input                      vs,                 // vertical synchronization
	input                      de,                 // video valid

	output                      hs_r,                 // horizontal synchronization
	output                      vs_r,                 // vertical synchronization
	output                      de_r,                 // video valid
	output[DATA_WIDTH - 1:0]    vout_data,          // video data
    input [1:0]                brightness_gray

);
reg [20:0] hs_d;
reg [20:0] vs_d;
reg [20:0] de_d;
reg[DATA_WIDTH - 1:0]  vout_data_r;
wire [DATA_WIDTH-1:0] adjusted_read_data;
generate if (ENABLE_BRIGHTNESS && DATA_WIDTH == 24) begin : g_brightness
    reg [1:0] gray_meta, gray_sync, frame_gray;
    reg vs_previous;
    // Registered Gray code crosses the domain; latch only in vertical blanking.
    always @(posedge video_clk or posedge rst) begin
        if(rst) begin
            gray_meta <= 0; gray_sync <= 0; frame_gray <= 0;
            vs_previous <= 0;
        end else begin
            gray_meta <= brightness_gray;
            gray_sync <= gray_meta;
            vs_previous <= vs;
            if(vs && !vs_previous) frame_gray <= gray_sync;
        end
    end
    function [7:0] scale_channel;
        input [7:0] value;
        input [1:0] gray;
        reg [9:0] triple;
        begin
            triple = {2'b00,value} + {1'b0,value,1'b0};
            case(gray)
                2'b00: scale_channel = value;
                2'b01: scale_channel = triple[9:2];
                2'b11: scale_channel = {1'b0,value[7:1]};
                2'b10: scale_channel = {2'b0,value[7:2]};
            endcase
        end
    endfunction
    assign adjusted_read_data = {scale_channel(read_data[23:16],frame_gray),
                                 scale_channel(read_data[15:8],frame_gray),
                                 scale_channel(read_data[7:0],frame_gray)};
end else begin : g_unscaled
    assign adjusted_read_data = read_data;
end endgenerate

assign read_en = de_d[18];
assign hs_r = hs_d[20];
assign vs_r = vs_d[20];
assign de_r = de_d[20];
assign vout_data = vout_data_r;
always@(posedge video_clk or posedge rst)
begin
	if(rst == 1'b1)
		vout_data_r <= {DATA_WIDTH{1'b0}};
	else if(de_d[19])
		vout_data_r <= adjusted_read_data;
	else
		vout_data_r <= {DATA_WIDTH{1'b0}};
end
always @(posedge video_clk or posedge rst)begin
	if(rst)begin
    	hs_d <= 20'b0;
        vs_d <= 20'b0;
        de_d <= 20'b0;
    end
	else begin
    	
    	hs_d <= {hs_d[19:0],hs};
        vs_d <= {vs_d[19:0],vs};
        de_d <= {de_d[19:0],de};
    end
end
endmodule
