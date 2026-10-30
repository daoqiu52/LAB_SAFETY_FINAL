module bmp_read #(
    parameter integer CLK_FREQ_HZ = 100000000,
    parameter integer SD_READ_TIMEOUT_MS = 500
)(
    input                       clk,
    input                       rst,
    output                      ready,

    // 上电扫描：从 scan_start_sector 开始，顺序寻找前 scan_target_count 张 BMP
    input                       scan_start,
    input  [31:0]               scan_start_sector,
    input  [31:0]               scan_max_sector,
    input  [2:0]                scan_target_count,
    output reg                  scan_done,
    output reg                  scan_found_valid,
    output reg [31:0]           scan_found_sector,
    output reg [2:0]            scan_found_total,

    // 按指定扇区加载一张图到 SDRAM
    input                       load_start,
    input                       load_abort,
    input  [31:0]               load_sector,

    input                       sd_init_done,
    output reg [3:0]            state_code,
    input  [15:0]               bmp_width,
    input  [15:0]               bmp_height,

    output reg                  write_req,
    input                       write_req_ack,

    output reg                  sd_sec_read,
    output reg [31:0]           sd_sec_read_addr,
    input  [7:0]                sd_sec_read_data,
    input                       sd_sec_read_data_valid,
    input                       sd_sec_read_end,

    output reg                  bmp_data_wr_en,
    output reg [23:0]           bmp_data,
    output reg [2:0]            bmp_error,
    output reg                  load_error,
    output reg                  load_done
);

localparam ST_IDLE      = 3'd0;
localparam ST_SCAN      = 3'd1;
localparam ST_LOAD_HDR  = 3'd2;
localparam ST_LOAD_WAIT = 3'd3;
localparam ST_LOAD_DATA = 3'd4;

reg [2:0]  state;
reg [9:0]  rd_cnt;

reg [7:0]  header_0;
reg [7:0]  header_1;
reg [31:0] file_len;
reg [31:0] pixel_offset;
reg [31:0] width;
reg [31:0] height;
reg [15:0] bit_count;
reg [31:0] compression;
reg [31:0] dib_size;
reg [15:0] planes;
reg [31:0] pixel_count, row_byte, sd_timeout_cnt;
reg [9:0] sector_bytes;
localparam [2:0] ERR_NONE=0, ERR_HEADER=1, ERR_TIMEOUT=2, ERR_TRUNCATED=3;
localparam [63:0] TIMEOUT_RAW = (64'd1 * CLK_FREQ_HZ * SD_READ_TIMEOUT_MS) / 1000;
localparam [31:0] TIMEOUT_CYCLES = TIMEOUT_RAW < 1 ? 1 : TIMEOUT_RAW;
wire [31:0] row_data_bytes = {16'd0,bmp_width} * 32'd3;
wire [31:0] row_stride = (row_data_bytes + 3) & 32'hfffffffc;
wire [63:0] expected_pixels = {48'd0,bmp_width} * {48'd0,bmp_height};
wire [63:0] pixel_end = {32'd0,pixel_offset} + ({32'd0,row_stride} * {48'd0,bmp_height});
wire signature_ok = header_0=="B" && header_1=="M";
wire layout_ok = dib_size>=40 && planes==1 && pixel_offset>=54 &&
                 {32'd0,pixel_offset}>=(64'd14+dib_size) && pixel_offset<file_len;
wire short_file = signature_ok && layout_ok && pixel_end<=64'hffffffff && {32'd0,file_len}<pixel_end;
wire sector_complete = ({1'b0,sector_bytes} + (sd_sec_read_data_valid ? 11'd1 : 11'd0))==512;

reg [31:0] scan_sector;
reg [31:0] load_sector_latched;
reg [31:0] bmp_len_cnt;
reg [1:0]  bmp_byte_idx;

wire header_match;
wire bmp_data_valid;
wire [31:0] file_sector_count;
wire [31:0] next_scan_sector_if_match;
wire [31:0] next_scan_sector_if_miss;
wire [32:0] scan_next_wide = {1'b0,scan_sector} + {1'b0,file_sector_count};
wire safe_skip = signature_ok && layout_ok && file_len>=54 &&
                 file_sector_count!=0 && !scan_next_wide[32] &&
                 scan_next_wide<=({1'b0,scan_max_sector}+33'd1);

assign ready = (state == ST_IDLE);
assign header_match = signature_ok && layout_ok &&
                      (width == {16'd0,bmp_width}) &&
                      (height == {16'd0,bmp_height}) && bmp_width!=0 && bmp_height!=0 &&
                      (bit_count    == 16'd24) &&
                      (compression  == 32'd0) && pixel_end<=64'hffffffff &&
                      ({32'd0,file_len}>=pixel_end);
assign bmp_data_valid = (sd_sec_read_data_valid == 1'b1) &&
                        (bmp_len_cnt >= pixel_offset) &&
                        ({32'd0,bmp_len_cnt} < pixel_end) && row_byte<row_data_bytes;
assign file_sector_count = {9'd0,file_len[31:9]} + (|file_len[8:0]);
assign next_scan_sector_if_match = scan_sector + file_sector_count;
assign next_scan_sector_if_miss  = safe_skip ? scan_next_wide[31:0] : scan_sector + 32'd1;

always @(posedge clk or posedge rst) begin
    if(rst) begin sector_bytes<=0; sd_timeout_cnt<=0; end
    else if(load_abort || !sd_init_done || state==ST_IDLE) begin
        sector_bytes<=0; sd_timeout_cnt<=0;
    end else begin
        if(sd_sec_read_end || state==ST_LOAD_WAIT) sector_bytes<=0;
        else if(sd_sec_read_data_valid && sector_bytes<1023) sector_bytes<=sector_bytes+1'b1;
        if(sd_sec_read_end || (state==ST_LOAD_WAIT && write_req_ack)) sd_timeout_cnt<=0;
        else if(sd_timeout_cnt<TIMEOUT_CYCLES) sd_timeout_cnt<=sd_timeout_cnt+1'b1;
    end
end
always @(posedge clk or posedge rst) begin
    if(rst) begin pixel_count<=0; row_byte<=0; end
    else if(state!=ST_LOAD_DATA) begin pixel_count<=0; row_byte<=0; end
    else begin
        if(bmp_data_valid && bmp_byte_idx==2) pixel_count<=pixel_count+1'b1;
        if(sd_sec_read_data_valid && bmp_len_cnt>=pixel_offset && {32'd0,bmp_len_cnt}<pixel_end)
            row_byte <= row_byte==row_stride-1 ? 0 : row_byte+1'b1;
    end
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        rd_cnt <= 10'd0;
    end else if ((state == ST_SCAN) || (state == ST_LOAD_HDR)) begin
        if (sd_sec_read_end)
            rd_cnt <= 10'd0;
        else if (sd_sec_read_data_valid)
            rd_cnt <= rd_cnt + 10'd1;
    end else begin
        rd_cnt <= 10'd0;
    end
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        header_0     <= 8'd0;
        header_1     <= 8'd0;
        file_len     <= 32'd0;
        pixel_offset <= 32'd54;
        width        <= 32'd0;
        height       <= 32'd0;
        bit_count    <= 16'd0;
        compression  <= 32'd0;
        dib_size <= 0;
        planes <= 0;
    end else if (((state == ST_SCAN) || (state == ST_LOAD_HDR)) && sd_sec_read_data_valid) begin
        case (rd_cnt)
            10'd0 : header_0 <= sd_sec_read_data;
            10'd1 : header_1 <= sd_sec_read_data;

            10'd2 : file_len[7:0] <= sd_sec_read_data;
            10'd3 : file_len[15:8] <= sd_sec_read_data;
            10'd4 : file_len[23:16] <= sd_sec_read_data;
            10'd5 : file_len[31:24] <= sd_sec_read_data;

            10'd10: pixel_offset[7:0] <= sd_sec_read_data;
            10'd11: pixel_offset[15:8] <= sd_sec_read_data;
            10'd12: pixel_offset[23:16] <= sd_sec_read_data;
            10'd13: pixel_offset[31:24] <= sd_sec_read_data;
            10'd14: dib_size[7:0] <= sd_sec_read_data;
            10'd15: dib_size[15:8] <= sd_sec_read_data;
            10'd16: dib_size[23:16] <= sd_sec_read_data;
            10'd17: dib_size[31:24] <= sd_sec_read_data;
            10'd26: planes[7:0] <= sd_sec_read_data;
            10'd27: planes[15:8] <= sd_sec_read_data;

            10'd18: width[7:0] <= sd_sec_read_data;
            10'd19: width[15:8] <= sd_sec_read_data;
            10'd20: width[23:16] <= sd_sec_read_data;
            10'd21: width[31:24] <= sd_sec_read_data;

            10'd22: height[7:0] <= sd_sec_read_data;
            10'd23: height[15:8] <= sd_sec_read_data;
            10'd24: height[23:16] <= sd_sec_read_data;
            10'd25: height[31:24] <= sd_sec_read_data;

            10'd28: bit_count[7:0] <= sd_sec_read_data;
            10'd29: bit_count[15:8] <= sd_sec_read_data;

            10'd30: compression[7:0] <= sd_sec_read_data;
            10'd31: compression[15:8] <= sd_sec_read_data;
            10'd32: compression[23:16] <= sd_sec_read_data;
            10'd33: compression[31:24] <= sd_sec_read_data;
            default: ;
        endcase
    end
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        bmp_len_cnt <= 32'd0;
    end else if (state == ST_LOAD_DATA) begin
        if (sd_sec_read_data_valid)
            bmp_len_cnt <= bmp_len_cnt + 32'd1;
    end else begin
        bmp_len_cnt <= 32'd0;
    end
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        bmp_byte_idx <= 2'd0;
    end else if (state == ST_LOAD_DATA) begin
        if (bmp_data_valid)
            bmp_byte_idx <= (bmp_byte_idx == 2'd2) ? 2'd0 : (bmp_byte_idx + 2'd1);
    end else begin
        bmp_byte_idx <= 2'd0;
    end
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        bmp_data_wr_en <= 1'b0;
        bmp_data       <= 24'd0;
    end else if (state == ST_LOAD_DATA && !load_abort && sd_init_done) begin
        if (bmp_data_valid) begin
            case (bmp_byte_idx)
                2'd0: begin
                    bmp_data_wr_en <= 1'b0;
                    bmp_data[7:0]  <= sd_sec_read_data;
                end
                2'd1: begin
                    bmp_data_wr_en <= 1'b0;
                    bmp_data[15:8] <= sd_sec_read_data;
                end
                2'd2: begin
                    bmp_data_wr_en  <= 1'b1;
                    bmp_data[23:16] <= sd_sec_read_data;
                end
                default: begin
                    bmp_data_wr_en <= 1'b0;
                end
            endcase
        end else begin
            bmp_data_wr_en <= 1'b0;
        end
    end else begin
        bmp_data_wr_en <= 1'b0;
    end
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        state             <= ST_IDLE;
        bmp_error<=ERR_NONE; load_error<=0; load_done<=0;
        state_code        <= 4'd0;
        sd_sec_read       <= 1'b0;
        sd_sec_read_addr  <= 32'd0;
        write_req         <= 1'b0;
        scan_done         <= 1'b0;
        scan_found_valid  <= 1'b0;
        scan_found_sector <= 32'd0;
        scan_found_total  <= 3'd0;
        scan_sector       <= 32'd0;
        load_sector_latched <= 32'd0;
    end else if (!sd_init_done) begin
        load_error<=0; load_done<=0;
        state             <= ST_IDLE;
        state_code        <= 4'd0;
        sd_sec_read       <= 1'b0;
        sd_sec_read_addr  <= 32'd0;
        write_req         <= 1'b0;
        scan_done         <= 1'b0;
        scan_found_valid  <= 1'b0;
        scan_found_sector <= 32'd0;
        scan_found_total  <= 3'd0;
        scan_sector       <= 32'd0;
        load_sector_latched <= 32'd0;
    end else if (load_abort) begin
        load_error<=0; load_done<=0;
        // Abort only the current load. Keep the completed scan/catalog so
        // the controller can retry; clearing scan_done here deadlocks it.
        state               <= ST_IDLE;
        state_code          <= 4'd0;
        sd_sec_read         <= 1'b0;
        write_req           <= 1'b0;
        scan_found_valid    <= 1'b0;
        load_sector_latched <= 32'd0;
    end else if(state!=ST_IDLE && sd_timeout_cnt>=TIMEOUT_CYCLES-1 && !sd_sec_read_end &&
                !(state==ST_LOAD_WAIT && write_req_ack)) begin
        bmp_error<=ERR_TIMEOUT; load_error<=1; load_done<=0;
        sd_sec_read<=0; write_req<=0; state<=ST_IDLE; scan_found_valid<=0;
    end else begin
        scan_found_valid <= 1'b0;
        load_error<=0; load_done<=0;

        case (state)
            ST_IDLE: begin
                state_code  <= 4'd1;
                sd_sec_read <= 1'b0;
                write_req   <= 1'b0;

                if (scan_start) begin
                    bmp_error<=ERR_NONE;
                    scan_done        <= 1'b0;
                    scan_found_total <= 3'd0;
                    scan_sector      <= scan_start_sector;
                    sd_sec_read_addr <= scan_start_sector;
                    state            <= ST_SCAN;
                end else if (load_start) begin
                    bmp_error<=ERR_NONE;
                    load_sector_latched <= load_sector;
                    sd_sec_read_addr    <= load_sector;
                    state               <= ST_LOAD_HDR;
                end
            end

            ST_SCAN: begin
                state_code  <= 4'd2;
                sd_sec_read <= 1'b1;

                if (sd_sec_read_end) begin
                    sd_sec_read <= 1'b0;

                    if(!sector_complete) begin
                        bmp_error<=ERR_TRUNCATED; load_error<=1; state<=ST_IDLE;
                    end else if (header_match) begin
                        scan_found_valid  <= 1'b1;
                        scan_found_sector <= scan_sector;
                        scan_found_total  <= scan_found_total + 3'd1;

                        if ((scan_found_total + 3'd1 >= scan_target_count) || scan_next_wide[32] || (scan_next_wide > {1'b0,scan_max_sector})) begin
                            scan_done        <= 1'b1;
                            state            <= ST_IDLE;
                            sd_sec_read_addr <= next_scan_sector_if_match;
                            scan_sector      <= next_scan_sector_if_match;
                        end else begin
                            sd_sec_read_addr <= next_scan_sector_if_match;
                            scan_sector      <= next_scan_sector_if_match;
                        end
                    end else begin
                        if (scan_sector >= scan_max_sector || next_scan_sector_if_miss>scan_max_sector) begin
                            scan_done <= 1'b1;
                            state     <= ST_IDLE;
                        end else begin
                            sd_sec_read_addr <= next_scan_sector_if_miss;
                            scan_sector      <= next_scan_sector_if_miss;
                        end
                    end
                end
            end

            ST_LOAD_HDR: begin
                state_code  <= 4'd2;
                sd_sec_read <= 1'b1;

                if (sd_sec_read_end) begin
                    sd_sec_read <= 1'b0;
                    if (header_match && sector_complete) begin
                        write_req        <= 1'b1;
                        sd_sec_read_addr <= load_sector_latched;
                        state            <= ST_LOAD_WAIT;
                    end else begin
                        bmp_error<= !sector_complete || short_file ? ERR_TRUNCATED : ERR_HEADER;
                        load_error<=1;
                        state <= ST_IDLE;
                    end
                end
            end

            ST_LOAD_WAIT: begin
                state_code <= 4'd3;
                if (write_req_ack) begin
                    write_req <= 1'b0;
                    state     <= ST_LOAD_DATA;
                end
            end

            ST_LOAD_DATA: begin
                state_code  <= 4'd4;
                sd_sec_read <= 1'b1;

                if (sd_sec_read_end) begin
                    sd_sec_read <= 1'b0;
                    if(!sector_complete) begin
                        bmp_error<=ERR_TRUNCATED; load_error<=1; state<=ST_IDLE;
                    end else if ({32'd0,bmp_len_cnt} + (sd_sec_read_data_valid ? 64'd1 : 64'd0) >= pixel_end) begin
                        if({32'd0,pixel_count} + ((bmp_data_valid && bmp_byte_idx==2) ? 64'd1 : 64'd0) != expected_pixels) begin
                            bmp_error<=ERR_TRUNCATED; load_error<=1;
                        end else begin load_done<=1; bmp_error<=ERR_NONE; end
                        state <= ST_IDLE;
                    end else begin
                        sd_sec_read_addr <= sd_sec_read_addr + 32'd1;
                    end
                end
            end

            default: begin
                state       <= ST_IDLE;
                state_code  <= 4'd1;
                sd_sec_read <= 1'b0;
                write_req   <= 1'b0;
            end
        endcase
    end
end

endmodule

// FAT32 application module: sd_sector_adapter (kept here for TD GUI source-list compatibility).
`timescale 1ns/1ps
// One owner at a time. Completion is delayed one clock after the last byte,
// so buffered consumers see all 512 bytes before parsing. Errors are bounded.
module sd_sector_adapter #(
 parameter integer CLK_FREQ_HZ=100000000, TIMEOUT_MS=500
)(input clk,input rst_n,input sector_req,input [31:0] sector_lba,
 output reg sector_busy,output reg sector_valid,output reg [7:0] sector_byte,
 output reg [8:0] sector_byte_index,output reg sector_done,output reg [7:0] sector_error,
 output reg sd_sec_read,output reg [31:0] sd_sec_read_addr,
 input sd_sec_read_data_valid,input [7:0] sd_sec_read_data,input sd_sec_read_end);
 localparam [63:0] RAW_CYCLES=(64'd1*CLK_FREQ_HZ*TIMEOUT_MS)/1000;
 localparam [31:0] LIMIT=RAW_CYCLES<1 ? 1 : RAW_CYCLES;
 reg [31:0] watchdog;
 reg [9:0] count;
 // Register the complete byte tuple before FAT directory/cluster decoding.
 // The board-failing build violated the directory cluster write-enable path.
 reg done_pending;
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   sector_busy<=0;sector_done<=0;sector_error<=0;sd_sec_read<=0;
   sd_sec_read_addr<=0;watchdog<=0;count<=0;
   sector_valid<=0;sector_byte<=0;sector_byte_index<=0;done_pending<=0;
  end else begin
   sector_done<=done_pending;done_pending<=0;
   sector_valid<=sector_busy && sd_sec_read_data_valid && count<512;
   sector_byte<=sd_sec_read_data;sector_byte_index<=count[8:0];
   if(!sector_busy) begin
    if(sector_req && !sector_done && !done_pending) begin
     sector_busy<=1;sector_error<=0;sd_sec_read<=1;
     sd_sec_read_addr<=sector_lba;watchdog<=0;count<=0;
    end
   end else begin
    if(sd_sec_read_data_valid && count<1023) count<=count+1'b1;
    if(sd_sec_read_end) begin
     sector_busy<=0;done_pending<=1;sd_sec_read<=0;
     if(({1'b0,count}+(sd_sec_read_data_valid?11'd1:11'd0))!=512) sector_error<=8'h12;
    end else if(watchdog>=LIMIT-1) begin
     sector_busy<=0;done_pending<=1;sd_sec_read<=0;sector_error<=8'h11;
    end else watchdog<=watchdog+1'b1;
   end
  end
 end
endmodule

// FAT32 application module: fat32_volume_scanner (kept here for TD GUI source-list compatibility).
`timescale 1ns/1ps

module fat32_volume_scanner #(parameter AUDIO_ENABLE=0)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        scan_start,
    output wire        scan_busy,
    output reg         scan_done,
    output reg  [7:0]  fs_error,
    output reg  [31:0] partition_lba,
    output reg  [7:0]  sectors_per_cluster,
    output reg  [31:0] fat_start_lba,
    output reg  [31:0] data_start_lba,
    output reg  [31:0] max_cluster,
    output reg  [3:0]  resource_valid,
    output reg  [31:0] resource0_cluster,
    output reg  [31:0] resource1_cluster,
    output reg  [31:0] resource2_cluster,
    output reg  [31:0] resource3_cluster,
    output reg  [31:0] resource0_size,
    output reg  [31:0] resource1_size,
    output reg  [31:0] resource2_size,
    output reg  [31:0] resource3_size,
    output reg         audio_found,
    output reg  [31:0] audio_cluster,audio_size,
    output reg         sector_req,
    output reg  [31:0] sector_lba,
    input  wire        sector_busy,
    input  wire        sector_valid,
    input  wire [7:0]  sector_byte,
    input  wire [8:0]  sector_byte_index,
    input  wire        sector_done,
    input  wire [7:0]  sector_error
);

    localparam ERR_NONE            = 8'h00;
    localparam ERR_SECTOR          = 8'h10;
    localparam ERR_BOOT_SIGNATURE  = 8'h20;
    localparam ERR_UNSUPPORTED_BPB = 8'h21;
    localparam ERR_RESOURCE_MISSING= 8'h22;

    localparam ST_IDLE       = 4'd0;
    localparam ST_READ0_REQ  = 4'd1;
    localparam ST_READ0_WAIT = 4'd2;
    localparam ST_BOOT_PARSE = 4'd3;
    localparam ST_BOOT_REQ   = 4'd4;
    localparam ST_BOOT_WAIT  = 4'd5;
    localparam ST_ROOT_REQ   = 4'd6;
    localparam ST_ROOT_WAIT  = 4'd7;
    localparam ST_DIR_SCAN   = 4'd8;
    localparam ST_FINISH     = 4'd9;
    localparam ST_BOOT_LAYOUT= 4'd10;
    localparam ST_BOOT_ROOT  = 4'd11;
    localparam ST_ROOT_FAT_REQ=4'd12, ST_ROOT_FAT_WAIT=4'd13, ST_ROOT_NEXT=4'd14;
    localparam ST_READ0_CHECK=5'd15, ST_BOOT_CHECK=5'd16;
    localparam ST_BUFFER_READ=5'd17, ST_BUFFER_USE=5'd18;
    localparam ST_BOOT_LIMITS=5'd19, ST_BOOT_VALIDATE=5'd20, ST_BOOT_ADDRESS=5'd21;
    localparam ST_ROOT_SHIFT=5'd22, ST_ROOT_ADDRESS=5'd23;
    localparam BUFFER_READ0=2'd0, BUFFER_BOOT=2'd1, BUFFER_MBR=2'd2;
    reg [7:0] root_sector_index;
    reg directory_end;
    reg [31:0] root_current, root_anchor, root_power, root_distance;
    reg [13:0] root_hops;
    reg [31:0] root_next;
    reg [31:0] partition_size;
    reg partitioned;
    reg [1:0] partition_slot;
    reg [31:0] cluster_count, root_offset;
    reg [32:0] required_fat_entries;
    wire [32:0] volume_end={1'b0,partition_lba}+{1'b0,total_sectors};
    wire [32:0] overhead={17'd0,reserved_sectors}+{1'b0,fat_span};

    reg [4:0] state;
    // One write port and one synchronous read port; no reset on the RAM.
    reg [7:0] sector_buffer [0:511]; /* fehdl force_ram=1, ram_style="bram" */
    reg [8:0] buffer_addr;
    reg [7:0] buffer_data;
    reg [1:0] buffer_kind;
    reg [7:0] boot_jump;
    reg [15:0] boot_signature;
    reg [15:0] bpb_bytes_per_sector, bpb_root_entries;
    reg [15:0] bpb_total16, bpb_fat16, bpb_version;
    reg [7:0] bpb_spc, bpb_flags;
    reg [7:0] mbr_type;
    reg [31:0] mbr_start, mbr_size;
    reg [4:0] directory_entry_index;
    reg [31:0] root_cluster;
    reg [31:0] total_sectors;
    reg [31:0] fat_sectors;
    reg [7:0] fat_count;
    reg [15:0] reserved_sectors;
    reg [31:0] root_lba;
    reg [31:0] data_sector_count;
    reg [31:0] fat_span;
    reg [2:0] sectors_per_cluster_shift;
    reg [7:0] dir_name0;
    reg [7:0] dir_name1;
    reg [7:0] dir_name2;
    reg [7:0] dir_name3;
    reg [7:0] dir_name4;
    reg [7:0] dir_name5;
    reg [7:0] dir_name6;
    reg [7:0] dir_name7;
    reg [7:0] dir_name8;
    reg [7:0] dir_name9;
    reg [7:0] dir_name10;
    reg       dir_entry_match,dir_audio_match;
    // Registered per-file commits isolate wide metadata registers from parser decode.
    reg [4:0] dir_commit;
    reg [1:0] dir_resource_index;
    reg [15:0] dir_cluster_high;
    reg [15:0] dir_cluster_low;
    reg [31:0] dir_file_size;

    assign scan_busy = (state != ST_IDLE);


    function valid_spc;
        input [7:0] value;
        begin
            valid_spc = value != 0 && ((value & (value - 1'b1)) == 0);
        end
    endfunction

    function [2:0] spc_shift;
        input [7:0] value;
        begin
            case (value)
                8'd2:   spc_shift = 3'd1;
                8'd4:   spc_shift = 3'd2;
                8'd8:   spc_shift = 3'd3;
                8'd16:  spc_shift = 3'd4;
                8'd32:  spc_shift = 3'd5;
                8'd64:  spc_shift = 3'd6;
                8'd128: spc_shift = 3'd7;
                default: spc_shift = 3'd0;
            endcase
        end
    endfunction

    task finish_with_error;
        input [7:0] code;
        begin
            fs_error <= code;
            state <= ST_FINISH;
        end
    endtask

    // Buffer only boot/MBR sectors. Directory and FAT entries retain their
    // existing byte-stream parsers and do not use this memory.
    always @(posedge clk) begin
        if (rst_n && sector_valid &&
            (state == ST_READ0_WAIT || state == ST_BOOT_WAIT))
            sector_buffer[sector_byte_index] <= sector_byte;
        if (state == ST_BUFFER_READ)
            buffer_data <= sector_buffer[buffer_addr];
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_IDLE;
            buffer_addr<=0;buffer_kind<=BUFFER_READ0;
            boot_jump<=0;boot_signature<=0;
            bpb_bytes_per_sector<=0;bpb_root_entries<=0;
            bpb_total16<=0;bpb_fat16<=0;bpb_version<=0;
            bpb_spc<=0;bpb_flags<=0;
            mbr_type<=0;mbr_start<=0;mbr_size<=0;
            root_sector_index<=0;directory_end<=0;root_current<=0;
            root_anchor<=0;root_power<=1;root_distance<=0;root_hops<=0;root_next<=0;
            partitioned<=0;partition_size<=0;partition_slot<=0;
            scan_done <= 1'b0;
            fs_error <= ERR_NONE;
            partition_lba <= 32'd0;
            sectors_per_cluster <= 8'd0;
            fat_start_lba <= 32'd0;
            data_start_lba <= 32'd0;
            max_cluster <= 32'd0;
            resource_valid <= 4'd0;
            audio_found<=0;audio_cluster<=0;audio_size<=0;dir_audio_match<=0;
            resource0_cluster <= 32'd0;
            resource1_cluster <= 32'd0;
            resource2_cluster <= 32'd0;
            resource3_cluster <= 32'd0;
            resource0_size <= 32'd0;
            resource1_size <= 32'd0;
            resource2_size <= 32'd0;
            resource3_size <= 32'd0;
            sector_req <= 1'b0;
            sector_lba <= 32'd0;
            directory_entry_index <= 5'd0;
            root_cluster <= 32'd0;
            total_sectors <= 32'd0;
            fat_sectors <= 32'd0;
            fat_count <= 8'd0;
            reserved_sectors <= 16'd0;
            root_lba <= 32'd0;
            data_sector_count <= 32'd0;
            cluster_count<=0;root_offset<=0;required_fat_entries<=0;
            fat_span <= 32'd0;
            sectors_per_cluster_shift <= 3'd0;
            dir_name0 <= 8'd0;
            dir_name1 <= 8'd0;
            dir_name2 <= 8'd0;
            dir_name3 <= 8'd0;
            dir_name4 <= 8'd0;
            dir_name5 <= 8'd0;
            dir_name6 <= 8'd0;
            dir_name7 <= 8'd0;
            dir_name8 <= 8'd0;
            dir_name9 <= 8'd0;
            dir_name10 <= 8'd0;
            dir_entry_match <= 1'b0;
            dir_resource_index <= 2'd3;
            dir_commit <= 5'd0;
            dir_cluster_high <= 16'd0;
            dir_cluster_low <= 16'd0;
            dir_file_size <= 32'd0;
        end else begin
            scan_done <= 1'b0;
            sector_req <= 1'b0;

            // Size MSB was captured on byte 31. Commit on the next clock;
            // even with consecutive bytes, the next entry cannot change metadata yet.
            dir_commit <= 5'd0;
            if(dir_commit[4]) begin
                audio_found<=1;audio_cluster<={dir_cluster_high,dir_cluster_low};
                audio_size<=dir_file_size;
            end
            if(dir_commit[0]) begin
                resource0_cluster<={dir_cluster_high,dir_cluster_low};
                resource0_size<=dir_file_size;resource_valid[0]<=1'b1;
            end
            if(dir_commit[1]) begin
                resource1_cluster<={dir_cluster_high,dir_cluster_low};
                resource1_size<=dir_file_size;resource_valid[1]<=1'b1;
            end
            if(dir_commit[2]) begin
                resource2_cluster<={dir_cluster_high,dir_cluster_low};
                resource2_size<=dir_file_size;resource_valid[2]<=1'b1;
            end
            if(dir_commit[3]) begin
                resource3_cluster<={dir_cluster_high,dir_cluster_low};
                resource3_size<=dir_file_size;resource_valid[3]<=1'b1;
            end

            if (sector_valid && state == ST_ROOT_WAIT) begin
                case (sector_byte_index[4:0])
                    5'd0: begin
                        dir_name0 <= sector_byte;
                        dir_entry_match <= 1'b0;
                        dir_audio_match <= 1'b0;
                        if(sector_byte==0) directory_end<=1;
                    end
                    5'd1:  dir_name1 <= sector_byte;
                    5'd2:  dir_name2 <= sector_byte;
                    5'd3:  dir_name3 <= sector_byte;
                    5'd4:  dir_name4 <= sector_byte;
                    5'd5:  dir_name5 <= sector_byte;
                    5'd6:  dir_name6 <= sector_byte;
                    5'd7:  dir_name7 <= sector_byte;
                    5'd8:  dir_name8 <= sector_byte;
                    5'd9:  dir_name9 <= sector_byte;
                    5'd10: dir_name10 <= sector_byte;
                    5'd11: begin
                        if(AUDIO_ENABLE && !directory_end && (sector_byte & 8'hd8)==0 &&
                           {dir_name0,dir_name1,dir_name2,dir_name3,dir_name4,dir_name5,dir_name6,dir_name7,dir_name8,dir_name9,dir_name10}=="AUDIO   PCM")
                            dir_audio_match<=1;
                        if (!directory_end && (sector_byte & 8'hd8)==0 &&
                            dir_name0 == "P" && dir_name1 == "I" &&
                            dir_name2 == "C" && dir_name3 >= "1" &&
                            dir_name3 <= "4" && dir_name4 == " " &&
                            dir_name5 == " " && dir_name6 == " " &&
                            dir_name7 == " " && dir_name8 == "B" &&
                            dir_name9 == "M" && dir_name10 == "P") begin
                            dir_entry_match <= 1'b1;
                            dir_resource_index <= dir_name3 - "1";
                        end
                    end
                    5'd20: dir_cluster_high[7:0] <= sector_byte;
                    5'd21: dir_cluster_high[15:8] <= sector_byte;
                    5'd26: dir_cluster_low[7:0] <= sector_byte;
                    5'd27: dir_cluster_low[15:8] <= sector_byte;
                    5'd28: dir_file_size[7:0] <= sector_byte;
                    5'd29: dir_file_size[15:8] <= sector_byte;
                    5'd30: dir_file_size[23:16] <= sector_byte;
                    5'd31: begin
                        dir_file_size[31:24] <= sector_byte;
                        dir_commit[4] <= dir_audio_match && !audio_found;
                        dir_commit[0] <= dir_entry_match && dir_resource_index==0 && !resource_valid[0];
                        dir_commit[1] <= dir_entry_match && dir_resource_index==1 && !resource_valid[1];
                        dir_commit[2] <= dir_entry_match && dir_resource_index==2 && !resource_valid[2];
                        dir_commit[3] <= dir_entry_match && dir_resource_index==3 && !resource_valid[3];
                    end
                    default: begin end
                endcase
            end

            if(sector_valid && state==ST_ROOT_FAT_WAIT) begin
                if(sector_byte_index=={root_current[6:0],2'b00}) root_next[7:0]<=sector_byte;
                if(sector_byte_index=={root_current[6:0],2'b00}+9'd1) root_next[15:8]<=sector_byte;
                if(sector_byte_index=={root_current[6:0],2'b00}+9'd2) root_next[23:16]<=sector_byte;
                if(sector_byte_index=={root_current[6:0],2'b00}+9'd3) root_next[31:24]<={4'd0,sector_byte[3:0]};
            end

            case (state)
                ST_IDLE: if (scan_start) begin
                    audio_found<=0;audio_cluster<=0;audio_size<=0;dir_audio_match<=0;
                    fs_error <= ERR_NONE;
                    partition_lba <= 32'd0;
                    resource_valid <= 4'd0;
                    partitioned<=0;partition_size<=0;partition_slot<=0;directory_end<=0;
                    root_sector_index<=0;root_hops<=0;root_power<=1;root_distance<=0;
                    state <= ST_READ0_REQ;
                end
                ST_READ0_REQ: if (!sector_busy) begin
                    sector_lba <= 32'd0;
                    sector_req <= 1'b1;
                    state <= ST_READ0_WAIT;
                end
                ST_READ0_WAIT: if (sector_done) begin
                    if (sector_error != 0)
                        finish_with_error(sector_error);
                    else begin
                        buffer_addr<=0;buffer_kind<=BUFFER_READ0;
                        state<=ST_BUFFER_READ;
                    end
                end
                // The RAM output is registered in READ; consume it in USE.
                // Read BPB bytes 0..47 and the two signature bytes, or one
                // 16-byte MBR entry. Every relevant field is overwritten.
                ST_BUFFER_READ: state<=ST_BUFFER_USE;
                ST_BUFFER_USE: begin
                    if (buffer_kind == BUFFER_MBR) begin
                        // Entry base is 446 + slot*16 (low nibble E).
                        case (buffer_addr[3:0])
                            4'h2: mbr_type<=buffer_data;
                            4'h6: mbr_start[7:0]<=buffer_data;
                            4'h7: mbr_start[15:8]<=buffer_data;
                            4'h8: mbr_start[23:16]<=buffer_data;
                            4'h9: mbr_start[31:24]<=buffer_data;
                            4'ha: mbr_size[7:0]<=buffer_data;
                            4'hb: mbr_size[15:8]<=buffer_data;
                            4'hc: mbr_size[23:16]<=buffer_data;
                            4'hd: mbr_size[31:24]<=buffer_data;
                            default: begin end
                        endcase
                    end else begin
                        case (buffer_addr)
                            9'd0: boot_jump<=buffer_data;
                            9'd11: bpb_bytes_per_sector[7:0]<=buffer_data;
                            9'd12: bpb_bytes_per_sector[15:8]<=buffer_data;
                            9'd13: bpb_spc<=buffer_data;
                            9'd14: reserved_sectors[7:0]<=buffer_data;
                            9'd15: reserved_sectors[15:8]<=buffer_data;
                            9'd16: fat_count<=buffer_data;
                            9'd17: bpb_root_entries[7:0]<=buffer_data;
                            9'd18: bpb_root_entries[15:8]<=buffer_data;
                            9'd19: bpb_total16[7:0]<=buffer_data;
                            9'd20: bpb_total16[15:8]<=buffer_data;
                            9'd22: bpb_fat16[7:0]<=buffer_data;
                            9'd23: bpb_fat16[15:8]<=buffer_data;
                            9'd32: total_sectors[7:0]<=buffer_data;
                            9'd33: total_sectors[15:8]<=buffer_data;
                            9'd34: total_sectors[23:16]<=buffer_data;
                            9'd35: total_sectors[31:24]<=buffer_data;
                            9'd36: fat_sectors[7:0]<=buffer_data;
                            9'd37: fat_sectors[15:8]<=buffer_data;
                            9'd38: fat_sectors[23:16]<=buffer_data;
                            9'd39: fat_sectors[31:24]<=buffer_data;
                            9'd40: bpb_flags<=buffer_data;
                            9'd42: bpb_version[7:0]<=buffer_data;
                            9'd43: bpb_version[15:8]<=buffer_data;
                            9'd44: root_cluster[7:0]<=buffer_data;
                            9'd45: root_cluster[15:8]<=buffer_data;
                            9'd46: root_cluster[23:16]<=buffer_data;
                            9'd47: root_cluster[31:24]<=buffer_data;
                            9'd510: boot_signature[7:0]<=buffer_data;
                            9'd511: boot_signature[15:8]<=buffer_data;
                            default: begin end
                        endcase
                    end
                    if (buffer_kind == BUFFER_MBR && buffer_addr[3:0] == 4'hd)
                        state<=ST_DIR_SCAN;
                    else if (buffer_kind != BUFFER_MBR && buffer_addr == 9'd511)
                        state<=buffer_kind==BUFFER_READ0 ? ST_READ0_CHECK : ST_BOOT_CHECK;
                    else begin
                        buffer_addr<=buffer_kind!=BUFFER_MBR && buffer_addr==9'd47 ?
                                     9'd510 : buffer_addr+1'b1;
                        state<=ST_BUFFER_READ;
                    end
                end
                ST_READ0_CHECK: begin
                    if (boot_signature != 16'haa55)
                        finish_with_error(ERR_BOOT_SIGNATURE);
                    else if ((boot_jump == 8'heb || boot_jump == 8'he9) &&
                             bpb_bytes_per_sector==512 && bpb_root_entries==0 && bpb_fat16==0) begin
                        partition_lba<=0;state<=ST_BOOT_PARSE;
                    end else begin
                        // Check all four primary entries in their original order.
                        partition_slot<=0;buffer_addr<=9'd446;buffer_kind<=BUFFER_MBR;
                        state<=ST_BUFFER_READ;
                    end
                end
                ST_DIR_SCAN: begin
                    if(mbr_type==8'h0b || mbr_type==8'h0c) begin
                        if(mbr_start==0 || mbr_size==0 ||
                           {1'b0,mbr_start}+{1'b0,mbr_size}>33'h100000000)
                            finish_with_error(ERR_UNSUPPORTED_BPB);
                        else begin
                            partitioned<=1;partition_size<=mbr_size;
                            partition_lba<=mbr_start;
                            sector_lba<=mbr_start;state<=ST_BOOT_REQ;
                        end
                    end else if(partition_slot==3) finish_with_error(ERR_UNSUPPORTED_BPB);
                    else begin
                        partition_slot<=partition_slot+1'b1;
                        buffer_addr<=9'd462+{3'd0,partition_slot,4'd0};
                        state<=ST_BUFFER_READ;
                    end
                end
                ST_BOOT_REQ: if (!sector_busy) begin
                    sector_req <= 1'b1;
                    state <= ST_BOOT_WAIT;
                end
                ST_BOOT_WAIT: if (sector_done) begin
                    if (sector_error != 0)
                        finish_with_error(sector_error);
                    else begin
                        buffer_addr<=0;buffer_kind<=BUFFER_BOOT;
                        state<=ST_BUFFER_READ;
                    end
                end
                ST_BOOT_CHECK: begin
                    if (boot_signature != 16'haa55)
                        finish_with_error(ERR_BOOT_SIGNATURE);
                    else state<=ST_BOOT_PARSE;
                end
                ST_BOOT_PARSE: begin
                    if (bpb_bytes_per_sector != 16'd512 || !valid_spc(bpb_spc) ||
                        reserved_sectors == 0 || (fat_count != 1 && fat_count != 2) ||
                        bpb_root_entries != 0 || bpb_fat16 != 0 || total_sectors == 0 ||
                        fat_sectors == 0 || fat_sectors>32'h7fffffff || root_cluster < 2 ||
                        bpb_total16!=0 || bpb_version!=0 ||
                        (bpb_flags[7] && bpb_flags[3:0]>=fat_count)) begin
                        finish_with_error(ERR_UNSUPPORTED_BPB);
                    end else begin
                        sectors_per_cluster <= bpb_spc;
                        sectors_per_cluster_shift <= spc_shift(bpb_spc);
                        fat_span <= fat_count == 2 ? (fat_sectors << 1) : fat_sectors;
                        fat_start_lba <= partition_lba + reserved_sectors +
                          ((bpb_flags[7] && bpb_flags[3:0]==1) ? fat_sectors:0);
                        state <= ST_BOOT_LAYOUT;
                    end
                end
                ST_BOOT_LAYOUT: begin
                    data_start_lba <= partition_lba + reserved_sectors + fat_span;
                    data_sector_count <= total_sectors - reserved_sectors - fat_span;
                    if(overhead>={1'b0,total_sectors} || volume_end>33'h100000000 ||
                       (partitioned && total_sectors>partition_size)) finish_with_error(ERR_UNSUPPORTED_BPB);
                    else state <= ST_BOOT_ROOT;
                end
                // Separate barrel shifts, additions and validation at 100 MHz.
                // No directory request is issued until all checks pass.
                ST_BOOT_ROOT: begin
                    cluster_count <= data_sector_count >> sectors_per_cluster_shift;
                    root_offset <= root_cluster - 32'd2;
                    state <= ST_BOOT_LIMITS;
                end
                ST_BOOT_LIMITS: begin
                    max_cluster <= cluster_count + 1'b1;
                    required_fat_entries <= {1'b0,cluster_count} + 33'd2;
                    root_offset <= root_offset << sectors_per_cluster_shift;
                    state <= ST_BOOT_VALIDATE;
                end
                ST_BOOT_VALIDATE: begin
                    // Keep the carry: count+2 must not wrap for malformed BPBs.
                    // FAT sectors*128 fits exactly in 39 bits (formerly 64).
                    if(cluster_count<65525 || cluster_count>32'h0fffffee ||
                       root_cluster>max_cluster || {fat_sectors,7'd0}<{6'd0,required_fat_entries})
                        finish_with_error(ERR_UNSUPPORTED_BPB);
                    else state <= ST_BOOT_ADDRESS;
                end
                ST_BOOT_ADDRESS: begin
                    root_lba <= data_start_lba + root_offset;
                    sector_lba <= data_start_lba + root_offset;
                    root_current<=root_cluster;root_anchor<=root_cluster;
                    state <= ST_ROOT_REQ;
                end
                ST_ROOT_REQ: if (!sector_busy) begin
                    dir_entry_match <= 1'b0;
                    sector_req <= 1'b1;
                    state <= ST_ROOT_WAIT;
                end
                ST_ROOT_WAIT: if (sector_done) begin
                    if (sector_error != 0)
                        finish_with_error(sector_error);
                    else begin
                        if(resource_valid==4'hf && (!AUDIO_ENABLE || audio_found)) state<=ST_FINISH;
                        else if(directory_end) begin if(resource_valid!=0) state<=ST_FINISH; else finish_with_error(ERR_RESOURCE_MISSING); end
                        else if(root_sector_index+1<sectors_per_cluster) begin
                            root_sector_index<=root_sector_index+1'b1;sector_lba<=sector_lba+1'b1;state<=ST_ROOT_REQ;
                        end else begin
                            sector_lba<=fat_start_lba+(root_current>>7);state<=ST_ROOT_FAT_REQ;
                        end
                    end
                end
                ST_ROOT_FAT_REQ: if(!sector_busy) begin sector_req<=1;state<=ST_ROOT_FAT_WAIT;end
                ST_ROOT_FAT_WAIT: if(sector_done) begin
                    if(sector_error!=0) finish_with_error(sector_error);
                    else state<=ST_ROOT_NEXT;
                end
                ST_ROOT_NEXT: begin
                    if(root_next>=32'h0ffffff8) begin if(resource_valid!=0) state<=ST_FINISH; else finish_with_error(ERR_RESOURCE_MISSING); end
                    else if(root_next<2 || root_next>max_cluster || root_next>=32'h0ffffff0) finish_with_error(8'h24);
                    else if(root_next==root_current || root_next==root_anchor) finish_with_error(8'h23);
                    else if(root_hops==8191) finish_with_error(8'h26);
                    else begin
                        root_hops<=root_hops+1'b1;root_distance<=root_distance+1'b1;
                        if(root_distance+1>=root_power) begin root_anchor<=root_next;root_power<=root_power<<1;root_distance<=0;end
                        root_current<=root_next;root_sector_index<=0;
                        // Reuse boot address staging: subtract, shift, add in separate cycles.
                        root_offset<=root_next-32'd2;state<=ST_ROOT_SHIFT;
                    end
                end
                ST_ROOT_SHIFT: begin
                    root_offset<=root_offset<<sectors_per_cluster_shift;
                    state<=ST_ROOT_ADDRESS;
                end
                ST_ROOT_ADDRESS: begin
                    sector_lba<=data_start_lba+root_offset;
                    state<=ST_ROOT_REQ;
                end
                ST_FINISH: begin
                    scan_done <= 1'b1;
                    state <= ST_IDLE;
                end
                default: state <= ST_IDLE;
            endcase
        end
    end
endmodule


// FAT32 application module: fat32_file_streamer (kept here for TD GUI source-list compatibility).
`timescale 1ns/1ps

module fat32_file_streamer #(
    parameter integer MAX_VISITED_CLUSTERS = 8192,
    parameter RANGE_ENABLE = 0
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        file_start,
    input  wire [31:0] range_start,range_end,
    input  wire [31:0] first_cluster,
    input  wire [31:0] file_size,
    input  wire [7:0]  sectors_per_cluster,
    input  wire [31:0] fat_start_lba,
    input  wire [31:0] data_start_lba,
    input  wire [31:0] max_cluster,
    output wire        file_busy,
    output reg         file_done,
    output reg         file_valid,
    output reg  [7:0]  file_byte,
    output reg  [31:0] file_byte_offset,
    output reg  [7:0]  fs_error,
    output reg         sector_req,
    output reg  [31:0] sector_lba,
    input  wire        sector_busy,
    input  wire        sector_valid,
    input  wire [7:0]  sector_byte,
    input  wire [8:0]  sector_byte_index,
    input  wire        sector_done,
    input  wire [7:0]  sector_error
);
    localparam ERR_NONE              = 8'h00;
    localparam ERR_SECTOR            = 8'h10;
    localparam ERR_FAT_LOOP          = 8'h23;
    localparam ERR_CLUSTER_RANGE     = 8'h24;
    localparam ERR_EARLY_EOF         = 8'h25;
    localparam ERR_VISITED_EXHAUSTED = 8'h26;

    localparam ST_IDLE       = 4'd0;
    localparam ST_DATA_REQ   = 4'd1;
    localparam ST_DATA_WAIT  = 4'd2;
    localparam ST_FAT_REQ    = 4'd3;
    localparam ST_FAT_WAIT   = 4'd4;
    localparam ST_FAT_PARSE  = 4'd5;
    localparam ST_FINISH     = 4'd7;
    localparam ST_ADDR_SUB=4'd8, ST_ADDR_SHIFT=4'd9, ST_ADDR_ADD=4'd10;
    reg [31:0] cluster_offset;
    reg [2:0] cluster_shift;
    function [2:0] shift_for_spc;
      input [7:0] value;
      begin case(value) 2:shift_for_spc=1;4:shift_for_spc=2;8:shift_for_spc=3;16:shift_for_spc=4;32:shift_for_spc=5;64:shift_for_spc=6;128:shift_for_spc=7;default:shift_for_spc=0;endcase end
    endfunction

    localparam integer CHAIN_COUNT_WIDTH = $clog2(MAX_VISITED_CLUSTERS + 1);

    reg [3:0] state;
    reg [31:0] current_cluster;
    reg [31:0] next_cluster;
    reg [31:0] remaining_bytes;
    reg [31:0] skip_remaining;
    localparam ST_SKIP_ADVANCE=4'd11;
    reg [7:0] sector_in_cluster;
    reg [8:0] fat_entry_offset;
    reg [7:0] fat_byte0, fat_byte1, fat_byte2, fat_byte3;
    reg [CHAIN_COUNT_WIDTH-1:0] chain_count;
    reg final_chain_check;
    reg [31:0] chain_anchor, chain_power, chain_distance;

    assign file_busy = (state != ST_IDLE);

    task finish_with_error;
        input [7:0] code;
        begin
            fs_error <= code;
            state <= ST_FINISH;
        end
    endtask

    task advance_to_next_cluster;
        input [31:0] cluster_value;
        begin
            chain_count <= chain_count + 1'b1;
            current_cluster <= cluster_value;
            state <= ST_ADDR_SUB;
        end
    endtask

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_IDLE;
            file_done <= 1'b0;
            file_valid <= 1'b0;
            file_byte <= 8'd0;
            file_byte_offset <= 32'd0;
            fs_error <= ERR_NONE;
            sector_req <= 1'b0;
            sector_lba <= 32'd0;
            current_cluster <= 32'd0;
            next_cluster <= 32'd0;
            remaining_bytes <= 32'd0;cluster_offset<=0;cluster_shift<=0;skip_remaining<=0;
            sector_in_cluster <= 8'd0;
            fat_entry_offset <= 9'd0;
            fat_byte0 <= 8'd0;
            fat_byte1 <= 8'd0;
            fat_byte2 <= 8'd0;
            fat_byte3 <= 8'd0;
            chain_count <= 0;
            final_chain_check <= 1'b0;
            chain_anchor<=0;chain_power<=1;chain_distance<=0;
        end else begin
            file_done <= 1'b0;
            file_valid <= 1'b0;
            sector_req <= 1'b0;

            if (state == ST_DATA_WAIT && sector_valid && remaining_bytes != 0) begin
                file_valid <= 1'b1;
                file_byte <= sector_byte;
                file_byte_offset <= file_size - remaining_bytes;
                remaining_bytes <= remaining_bytes - 1'b1;
            end

            if (state == ST_FAT_WAIT && sector_valid) begin
                if (sector_byte_index == fat_entry_offset)
                    fat_byte0 <= sector_byte;
                else if (sector_byte_index == fat_entry_offset + 1'b1)
                    fat_byte1 <= sector_byte;
                else if (sector_byte_index == fat_entry_offset + 2'd2)
                    fat_byte2 <= sector_byte;
                else if (sector_byte_index == fat_entry_offset + 2'd3)
                    fat_byte3 <= sector_byte;
            end

            case (state)
                ST_IDLE: if (file_start) begin
                    fs_error <= ERR_NONE;
                    file_byte_offset <= 32'd0;
                    remaining_bytes <= file_size;
                    skip_remaining <= RANGE_ENABLE ? range_start : 0;
                    cluster_shift <= shift_for_spc(sectors_per_cluster);
                    sector_in_cluster <= 8'd0;
                    chain_count <= 1;
                    final_chain_check <= 1'b0;
                    chain_anchor<=first_cluster;chain_power<=1;chain_distance<=0;
                    if (file_size == 0) begin
                        state <= ST_FINISH;
                    end else if (first_cluster < 2 || first_cluster > max_cluster ||
                                 sectors_per_cluster == 0) begin
                        finish_with_error(ERR_CLUSTER_RANGE);
                    end else begin
                        current_cluster <= first_cluster;
                        state <= ST_ADDR_SUB;
                    end
                end
                ST_ADDR_SUB: begin cluster_offset<=current_cluster-2;state<=ST_ADDR_SHIFT;end
                ST_ADDR_SHIFT: begin cluster_offset<=cluster_offset<<cluster_shift;state<=ST_ADDR_ADD;end
                ST_ADDR_ADD: begin sector_lba<=data_start_lba+cluster_offset;state<=ST_DATA_REQ;end
                ST_DATA_REQ: if (!sector_busy) begin
                    // Sector-aligned PCM seek: walk FAT links, skip payload IO.
                    if(RANGE_ENABLE && skip_remaining>=512 && remaining_bytes>=512) begin
                        skip_remaining<=skip_remaining-512;
                        remaining_bytes<=remaining_bytes-512;
                        state<=ST_SKIP_ADVANCE;
                    end else begin sector_req<=1'b1;state<=ST_DATA_WAIT;end
                end
                ST_DATA_WAIT,ST_SKIP_ADVANCE: if (sector_done || state==ST_SKIP_ADVANCE) begin
                    if (state==ST_DATA_WAIT && sector_error != 0) begin
                        finish_with_error(sector_error);
                    end else if(RANGE_ENABLE && range_end<file_size &&
                                remaining_bytes<=file_size-range_end) begin
                        // A segment boundary is not the FAT chain's true EOF.
                        state<=ST_FINISH;
                    end else if (remaining_bytes == 0 ||
                                 (remaining_bytes == 1 && sector_valid)) begin
                        final_chain_check <= 1'b1;
                        sector_in_cluster <= 8'd0;
                        fat_entry_offset <= {current_cluster[6:0], 2'b00};
                        sector_lba <= fat_start_lba + (current_cluster >> 7);
                        state <= ST_FAT_REQ;
                    end else if (sector_in_cluster + 1'b1 < sectors_per_cluster) begin
                        sector_in_cluster <= sector_in_cluster + 1'b1;
                        sector_lba <= sector_lba + 1'b1;
                        state <= ST_DATA_REQ;
                    end else begin
                        sector_in_cluster <= 8'd0;
                        fat_entry_offset <= {current_cluster[6:0], 2'b00};
                        sector_lba <= fat_start_lba + (current_cluster >> 7);
                        state <= ST_FAT_REQ;
                    end
                end
                ST_FAT_REQ: if (!sector_busy) begin
                    sector_req <= 1'b1;
                    state <= ST_FAT_WAIT;
                end
                ST_FAT_WAIT: if (sector_done) begin
                    if (sector_error != 0)
                        finish_with_error(sector_error);
                    else
                        state <= ST_FAT_PARSE;
                end
                ST_FAT_PARSE: begin
                    next_cluster <= {4'd0, fat_byte3[3:0], fat_byte2, fat_byte1, fat_byte0};
                    if (final_chain_check) begin
                        final_chain_check <= 1'b0;
                        if ({fat_byte3[3:0], fat_byte2, fat_byte1, fat_byte0} >= 28'hffffff8)
                            state <= ST_FINISH;
                        else if ({4'd0, fat_byte3[3:0], fat_byte2, fat_byte1, fat_byte0} < 2 ||
                                 {4'd0, fat_byte3[3:0], fat_byte2, fat_byte1, fat_byte0} > max_cluster)
                            finish_with_error(ERR_CLUSTER_RANGE);
                        else
                            finish_with_error(ERR_FAT_LOOP);
                    end else if ({fat_byte3[3:0], fat_byte2, fat_byte1, fat_byte0} >= 28'hffffff8) begin
                        finish_with_error(ERR_EARLY_EOF);
                    end else if ({4'd0, fat_byte3[3:0], fat_byte2, fat_byte1, fat_byte0} < 2 ||
                                 {4'd0, fat_byte3[3:0], fat_byte2, fat_byte1, fat_byte0} > max_cluster) begin
                        finish_with_error(ERR_CLUSTER_RANGE);
                    end else if ({4'd0,fat_byte3[3:0],fat_byte2,fat_byte1,fat_byte0}>=32'h0ffffff0) begin
                        finish_with_error(ERR_CLUSTER_RANGE);
                    end else if (chain_count >= MAX_VISITED_CLUSTERS) begin
                        finish_with_error(ERR_VISITED_EXHAUSTED);
                    end else if ({4'd0, fat_byte3[3:0], fat_byte2, fat_byte1, fat_byte0} == current_cluster ||
                                 {4'd0, fat_byte3[3:0], fat_byte2, fat_byte1, fat_byte0} == chain_anchor) begin
                        finish_with_error(ERR_FAT_LOOP);
                    end else begin
                        chain_distance<=chain_distance+1'b1;
                        if(chain_distance+1>=chain_power) begin
                            chain_anchor<={4'd0,fat_byte3[3:0],fat_byte2,fat_byte1,fat_byte0};
                            chain_power<=chain_power<<1;chain_distance<=0;
                        end
                        advance_to_next_cluster({4'd0, fat_byte3[3:0], fat_byte2, fat_byte1, fat_byte0});
                    end
                end
                ST_FINISH: begin
                    file_done <= 1'b1;
                    state <= ST_IDLE;
                end
                default: state <= ST_IDLE;
            endcase
        end
    end
endmodule

// FAT32 application module: bmp_file_decoder (kept here for TD GUI source-list compatibility).
`timescale 1ns/1ps
// Consumes a bounded FAT file stream; no assumptions about physical adjacency.
module bmp_file_decoder #(parameter VARIABLE_SIZE=0,MAX_DIMENSION=8192)(input clk,input rst,input start,input [31:0] directory_size,
 input [15:0] expected_width,input [15:0] expected_height,
 input byte_valid,input [7:0] byte_data,input [31:0] byte_offset,
 input file_done,input [7:0] file_error,
 output reg pixel_valid,output reg [23:0] pixel,
 output reg done,output reg [2:0] error,
 output reg header_valid,output [15:0] source_width,source_height);
 reg [7:0] header[0:53];
 reg checked;
 reg [31:0] offset,end_offset,pixels;
 // A 16-bit source width needs at most 196608 padded bytes per row.
 reg [17:0] row_pos;
 reg [1:0] channel;
 assign source_width=VARIABLE_SIZE ? {header[19],header[18]} : expected_width;
 assign source_height=VARIABLE_SIZE ? {header[23],header[22]} : expected_height;
 reg [17:0] row_bytes,stride,stride_last;
 reg [31:0] want_pixels;
 reg [47:0] body_bytes;
 reg [63:0] last_byte;
 reg [32:0] min_offset;
 reg layout_q,size_bad;
 function [31:0] h32;
 input integer i;
 begin h32={header[i+3],header[i+2],header[i+1],header[i]};end
 endfunction

 wire layout_ok=header[0]=="B" && header[1]=="M" && {header[17],header[16],header[15],header[14]}>=40 &&
  {header[21],header[20],header[19],header[18]}=={16'd0,source_width} && {header[25],header[24],header[23],header[22]}=={16'd0,source_height} &&
  source_width!=0 && source_height!=0 && (!VARIABLE_SIZE || (source_width<=MAX_DIMENSION && source_height<=MAX_DIMENSION)) && {header[27],header[26]}==1 &&
  {header[29],header[28]}==24 && {header[33],header[32],header[31],header[30]}==0 && {header[13],header[12],header[11],header[10]}>=54 &&
  {1'b0,header[13],header[12],header[11],header[10]}>=min_offset && {header[13],header[12],header[11],header[10]}<{header[5],header[4],header[3],header[2]} && last_byte<=64'hffffffff;
 // Header fields settle by byte 33; validation occurs at byte 53, leaving
 // twenty byte intervals for this arithmetic pipeline (even at one byte/clock).
 always @(posedge clk) begin
  row_bytes<={16'd0,source_width}*3;
  stride<=(row_bytes+18'd3)&18'h3fffc;
  // Precompute before header validation; remove subtractor from byte path.
  stride_last<=stride-18'd1;
  want_pixels<=source_width*source_height;
  body_bytes<=stride*source_height;
  last_byte<={32'd0,header[13],header[12],header[11],header[10]}+body_bytes;
  min_offset<=33'd14+{header[17],header[16],header[15],header[14]};
  layout_q<=layout_ok;
  size_bad<=({header[5],header[4],header[3],header[2]}>directory_size ||
             last_byte>{32'd0,directory_size} ||
             last_byte>{32'd0,header[5],header[4],header[3],header[2]});
 end
 integer k;
 always @(posedge clk or posedge rst) begin
  if(rst) begin
   checked<=0;offset<=0;end_offset<=0;row_pos<=0;pixels<=0;channel<=0;
   pixel_valid<=0;pixel<=0;done<=0;error<=0;header_valid<=0;
   for(k=0;k<54;k=k+1) header[k]<=0;
  end else begin
   pixel_valid<=0;done<=0;header_valid<=0;
   if(start) begin
    checked<=0;row_pos<=0;pixels<=0;channel<=0;error<=0;
   end else begin
    if(byte_valid && byte_offset<54) header[byte_offset[5:0]]<=byte_data;
    // All fields used below end at byte 33; latch validation on byte 53.
    // The earliest pixel (54) then uses only registered layout information.
    if(byte_valid && byte_offset==53 && !checked) begin
     checked<=1;offset<={header[13],header[12],header[11],header[10]};end_offset<=last_byte[31:0];
     if(!layout_q) error<=1;
     else if(size_bad) error<=3;
     else header_valid<=1;
    end
    if(byte_valid && checked && error==0 && byte_offset>=offset && byte_offset<end_offset) begin
     row_pos<=row_pos==stride_last ? 18'd0:row_pos+18'd1;
     if(row_pos<row_bytes) begin
      case(channel)
       0: begin pixel[7:0]<=byte_data;channel<=1;end
       1: begin pixel[15:8]<=byte_data;channel<=2;end
       2: begin pixel[23:16]<=byte_data;channel<=0;pixel_valid<=1;pixels<=pixels+1'b1;end
      endcase
     end
    end
    if(file_done) begin
     done<=1;
     if(file_error!=0) error<=file_error==8'h11 ? 2:3;
     else if(error==0 && (!checked || pixels!=want_pixels || channel!=0)) error<=3;
    end
   end
  end
 end
endmodule


// FAT32 application module: fat32_bmp_reader (kept here for TD GUI source-list compatibility).
`timescale 1ns/1ps
// Application-compatible catalog/load interface. Catalog tokens 0..3 are file
// identities, not LBAs. Slots name PIC1.BMP .. PIC4.BMP; missing slots report FS22 on load.
module fat32_bmp_reader #(
 parameter integer CLK_FREQ_HZ=100000000, SD_READ_TIMEOUT_MS=500,
 parameter AUDIO_ENABLE=0, SCALING_ENABLE=0, PRESERVE_ASPECT=1,
 parameter AUDIO_SEGMENTS=0, AUDIO_SEGMENT_BYTES=3456000
)(input clk,input rst,output ready,input scan_start,
 input [31:0] scan_start_sector,scan_max_sector,input [2:0] scan_target_count,
 output reg scan_done,output reg scan_found_valid,output reg [31:0] scan_found_sector,
 output reg [2:0] scan_found_total,
 input load_start,load_abort,input [31:0] load_sector,input sd_init_done,
 output reg [3:0] state_code,input [15:0] bmp_width,bmp_height,
 output reg write_req,input write_req_ack,
 output sd_sec_read,output [31:0] sd_sec_read_addr,input [7:0] sd_sec_read_data,
 input sd_sec_read_data_valid,sd_sec_read_end,
 output bmp_data_wr_en,output [23:0] bmp_data,
 output reg [2:0] bmp_error,output reg load_error,output reg load_done,
 output reg [7:0] fs_error,
 output reg [31:0] file_width,file_height,
 input audio_enable,audio_sector_ready,input [1:0] audio_index,
 output audio_byte_valid,output [7:0] audio_byte,
 output reg audio_finished,output reg [7:0] audio_error);
 // Metadata comes from the file stream, not configured HDMI dimensions.
 always @(posedge clk or posedge rst) begin
  if(rst) begin file_width<=0;file_height<=0;end
  else if(file_go && state==LOAD) begin file_width<=0;file_height<=0;end
  else if(file_valid && state==LOAD) begin
   case(file_offset)
    18:file_width[7:0]<=file_byte;19:file_width[15:8]<=file_byte;
    20:file_width[23:16]<=file_byte;21:file_width[31:24]<=file_byte;
    22:file_height[7:0]<=file_byte;23:file_height[15:8]<=file_byte;
    24:file_height[23:16]<=file_byte;25:file_height[31:24]<=file_byte;
   endcase
  end
 end
 localparam IDLE=0,SCAN=1,REPORT=2,ACK=3,LOAD=4,AUDIO=5;
 reg audio_attempted,audio_cancel;
 reg [31:0] audio_start_offset,audio_end_offset;
 function [31:0] segment_offset;
 input [1:0] index;
 begin case(index)
  0:segment_offset=0;1:segment_offset=AUDIO_SEGMENT_BYTES;
  2:segment_offset=2*AUDIO_SEGMENT_BYTES;default:segment_offset=3*AUDIO_SEGMENT_BYTES;
 endcase end endfunction
 wire audio_found;
 wire [31:0] audio_cluster,audio_size;
 assign audio_byte_valid=AUDIO_ENABLE && state==AUDIO && file_valid &&
                         (!AUDIO_SEGMENTS || (audio_enable && file_offset>=audio_start_offset && file_offset<audio_end_offset));
 assign audio_byte=file_byte;
 reg [2:0] state;
 reg [1:0] report_index;
 reg scan_go,file_go;
 reg [31:0] selected_cluster,selected_size,ack_wait;
 localparam [63:0] ACK_LIMIT=(64'd1*CLK_FREQ_HZ*SD_READ_TIMEOUT_MS)/1000;
 wire scanner_done,scanner_busy,scan_req,file_req;
 wire [31:0] scan_lba,file_lba,partition_lba,fat_lba,data_lba,max_cluster;
 wire [7:0] spc,scan_error,file_error;
 wire [3:0] resources;
 wire [31:0] c0,c1,c2,c3,s0,s1,s2,s3;
 wire sector_busy,sector_valid,sector_done;
 wire [7:0] sector_byte,sector_error;
 wire [8:0] sector_index;
 wire file_busy,file_done,file_valid;
 wire [7:0] file_byte;
 wire [31:0] file_offset;
 wire decoder_done;
 wire [2:0] decoder_error;
 wire raw_done,raw_pixel_valid,header_valid,scale_sector_ready;
 wire [2:0] raw_error;
 wire [23:0] raw_pixel;
 wire [15:0] source_width,source_height;
 wire cancel=rst || load_abort || !sd_init_done;
 assign ready=state==IDLE;
 fat32_volume_scanner #(.AUDIO_ENABLE(AUDIO_ENABLE)) scanner(
  .audio_found(audio_found),.audio_cluster(audio_cluster),.audio_size(audio_size),
  .clk(clk),.rst_n(!(rst || (state==SCAN && cancel))),.scan_start(scan_go),.scan_busy(scanner_busy),.scan_done(scanner_done),.fs_error(scan_error),
  .partition_lba(partition_lba),.sectors_per_cluster(spc),.fat_start_lba(fat_lba),.data_start_lba(data_lba),.max_cluster(max_cluster),
  .resource_valid(resources),.resource0_cluster(c0),.resource1_cluster(c1),.resource2_cluster(c2),.resource3_cluster(c3),
  .resource0_size(s0),.resource1_size(s1),.resource2_size(s2),.resource3_size(s3),
  .sector_req(scan_req),.sector_lba(scan_lba),.sector_busy(sector_busy),
  .sector_valid(sector_valid && state==SCAN),.sector_byte(sector_byte),.sector_byte_index(sector_index),
  .sector_done(sector_done && state==SCAN),.sector_error(sector_error));
 fat32_file_streamer #(.MAX_VISITED_CLUSTERS(AUDIO_ENABLE ? 1048576 : 8192),.RANGE_ENABLE(AUDIO_SEGMENTS)) streamer(
  .clk(clk),.rst_n(!(cancel || audio_cancel)),.file_start(file_go),.first_cluster(selected_cluster),.file_size(selected_size),
  .range_start(state==AUDIO ? audio_start_offset : 32'd0),.range_end(state==AUDIO ? audio_end_offset : selected_size),
  .sectors_per_cluster(spc),.fat_start_lba(fat_lba),.data_start_lba(data_lba),.max_cluster(max_cluster),
  .file_busy(file_busy),.file_done(file_done),.file_valid(file_valid),.file_byte(file_byte),.file_byte_offset(file_offset),.fs_error(file_error),
  .sector_req(file_req),.sector_lba(file_lba),.sector_busy(sector_busy || (state==AUDIO && (!audio_sector_ready || (AUDIO_SEGMENTS && !audio_enable))) || (state==LOAD && !scale_sector_ready)),
  .sector_valid(sector_valid && (state==LOAD || state==AUDIO)),.sector_byte(sector_byte),.sector_byte_index(sector_index),
  .sector_done(sector_done && (state==LOAD || state==AUDIO)),.sector_error(sector_error));
 sd_sector_adapter #(.CLK_FREQ_HZ(CLK_FREQ_HZ),.TIMEOUT_MS(SD_READ_TIMEOUT_MS)) adapter(
  .clk(clk),.rst_n(!cancel),.sector_req(state==SCAN?scan_req:(state==LOAD || (state==AUDIO && (!AUDIO_SEGMENTS || audio_enable)))?file_req:1'b0),
  .sector_lba(state==SCAN?scan_lba:file_lba),.sector_busy(sector_busy),.sector_valid(sector_valid),
  .sector_byte(sector_byte),.sector_byte_index(sector_index),.sector_done(sector_done),.sector_error(sector_error),
  .sd_sec_read(sd_sec_read),.sd_sec_read_addr(sd_sec_read_addr),.sd_sec_read_data_valid(sd_sec_read_data_valid),
  .sd_sec_read_data(sd_sec_read_data),.sd_sec_read_end(sd_sec_read_end));
 bmp_file_decoder #(.VARIABLE_SIZE(SCALING_ENABLE)) decoder(.clk(clk),.rst(cancel),.start(file_go && state==LOAD),.directory_size(selected_size),
  .expected_width(bmp_width),.expected_height(bmp_height),.byte_valid(file_valid && state==LOAD),.byte_data(file_byte),.byte_offset(file_offset),
  .file_done(file_done && state==LOAD),.file_error(file_error),.pixel_valid(raw_pixel_valid),.pixel(raw_pixel),.done(raw_done),.error(raw_error),
  .header_valid(header_valid),.source_width(source_width),.source_height(source_height));
 generate if(SCALING_ENABLE) begin : g_scale
  bmp_bilinear_fit #(.PRESERVE_ASPECT(PRESERVE_ASPECT)) scaler(.clk(clk),.rst(cancel),.start(file_go && state==LOAD),
   .header_valid(header_valid),.source_width(source_width),.source_height(source_height),
   .in_valid(raw_pixel_valid),.in_pixel(raw_pixel),.source_done(raw_done),.source_error(raw_error),
   .sector_ready(scale_sector_ready),.out_valid(bmp_data_wr_en),.out_pixel(bmp_data),
   .done(decoder_done),.error(decoder_error));
 end else begin : g_no_scale
  assign scale_sector_ready=1'b1;
  assign bmp_data_wr_en=raw_pixel_valid;assign bmp_data=raw_pixel;
  assign decoder_done=raw_done;assign decoder_error=raw_error;
 end endgenerate
 always @(posedge clk or posedge rst) begin
  if(rst) begin
   state<=IDLE;scan_go<=0;file_go<=0;scan_done<=0;scan_found_valid<=0;scan_found_sector<=0;scan_found_total<=0;
   write_req<=0;load_done<=0;load_error<=0;bmp_error<=0;fs_error<=0;state_code<=0;report_index<=0;
   selected_cluster<=0;selected_size<=0;ack_wait<=0;
   audio_attempted<=0;audio_finished<=0;audio_error<=0;audio_cancel<=0;audio_start_offset<=0;audio_end_offset<=0;
  end else begin
   scan_go<=0;file_go<=0;load_done<=0;load_error<=0;scan_found_valid<=0;audio_cancel<=0;
   if(AUDIO_SEGMENTS && !audio_enable && state==IDLE) begin audio_attempted<=0;audio_finished<=0;audio_error<=0;end
   if(load_abort || !sd_init_done) begin
    state<=IDLE;write_req<=0;
    if(state==AUDIO) begin audio_finished<=1;audio_error<=8'h41;end
   end
   else case(state)
    IDLE: begin
     state_code<=1;
     if(scan_start) begin
      scan_done<=0;scan_found_total<=0;scan_go<=1;state<=SCAN;fs_error<=0;bmp_error<=0;
     end else if(load_start) begin
      fs_error<=0;bmp_error<=0;ack_wait<=0;
      case(load_sector)
       0:begin selected_cluster<=c0;selected_size<=s0;end
       1:begin selected_cluster<=c1;selected_size<=s1;end
       2:begin selected_cluster<=c2;selected_size<=s2;end
       3:begin selected_cluster<=c3;selected_size<=s3;end
      endcase
      if(load_sector>3 || !resources[load_sector[1:0]]) begin fs_error<=8'h22;bmp_error<=1;load_error<=1;end
      else begin write_req<=1;state<=ACK;end
     end else if(AUDIO_ENABLE && audio_enable && !audio_attempted) begin
      audio_attempted<=1;
      if(!audio_found) begin audio_finished<=1;audio_error<=AUDIO_SEGMENTS ? 8'h42 : 0;end
      else if(audio_size==0 || audio_size[0] || (AUDIO_SEGMENTS && audio_size!=4*AUDIO_SEGMENT_BYTES)) begin audio_finished<=1;audio_error<=8'h40;end
      else begin
       audio_start_offset<=AUDIO_SEGMENTS ? segment_offset(audio_index) : 0;
       audio_end_offset<=AUDIO_SEGMENTS ? segment_offset(audio_index)+AUDIO_SEGMENT_BYTES : audio_size;
       selected_cluster<=audio_cluster;selected_size<=audio_size;file_go<=1;state<=AUDIO;
      end
     end
    end
    SCAN: begin
     state_code<=2;
     if(scanner_done) begin
      if(scan_error!=0) begin fs_error<=scan_error;bmp_error<=scan_error == 8'h11 ? 3'd2 : 3'd1;load_error<=1;state<=IDLE;end
      else begin report_index<=0;state<=REPORT;end
     end
    end
    REPORT: begin
     scan_found_valid<=1;scan_found_sector<={30'd0,report_index};scan_found_total<={1'b0,report_index}+1'b1;
     if(report_index==3) begin scan_done<=1;state<=IDLE;end
     else report_index<=report_index+1'b1;
    end
    ACK: begin
     state_code<=3;
     if(write_req_ack) begin write_req<=0;file_go<=1;state<=LOAD;end
     else if(ack_wait>=ACK_LIMIT-1) begin write_req<=0;bmp_error<=2;load_error<=1;state<=IDLE;end
     else ack_wait<=ack_wait+1'b1;
    end
    LOAD: begin
     state_code<=4;
     if(decoder_done) begin
      state<=IDLE;fs_error<=file_error;bmp_error<=decoder_error;
      if(decoder_error!=0) load_error<=1;else load_done<=1;
     end
    end
    AUDIO: if(AUDIO_SEGMENTS && !audio_enable && !sector_busy) begin
     // Never abort an SD transaction halfway through its data block.
     audio_cancel<=1;state<=IDLE;audio_attempted<=0;audio_finished<=0;audio_error<=0;
    end else if(file_done) begin
     audio_finished<=1;audio_error<=file_error;state<=IDLE;
    end
    default:state<=IDLE;
   endcase
  end
 end
endmodule



// Loading-time RGB bilinear scaler. Only two horizontally resampled scan lines
// are retained; SDRAM always receives the existing 640x480 bottom-up stream.
// Sector admission reserves space for an ENTIRE unthrottled 512-byte sector.
module bmp_bilinear_fit #(parameter OUTPUT_GAP=31,PRESERVE_ASPECT=1)(
 input clk,rst,start,header_valid,input [15:0] source_width,source_height,
 input in_valid,input [23:0] in_pixel,input source_done,input [2:0] source_error,
 output sector_ready,output out_valid,output reg [23:0] out_pixel,
 output reg done,output reg [2:0] error);
 localparam IDLE=0,FIT_WAIT=1,SET_X=2,X_WAIT=3,SET_Y=4,Y_WAIT=5,
            BEGIN_ROW=6,NEED_ROW=7,H_FETCH=8,H_GOT=9,H_MIX=10,H_STORE=11,
            SKIP_FETCH=12,SKIP_GOT=13,O_FETCH=14,O_WAIT=15,O_MIX=16,
            O_EMIT=17,O_GAP=18,WAIT_SOURCE=19,H_WAIT1=20,H_WAIT2=21,
            O_WAIT1=22,O_WAIT2=23,O_CAPTURE=24,H_TARGET=25,H_ROUTE=26,ASPECT=27,H_SELECT=28;
 reg [4:0] state;
 reg [15:0] sw,sh,sx,sy,sw_last,sh_last,h_right;
 reg [9:0] fit_w_last,fit_w_penultimate;
 reg [8:0] image_penultimate_y;
 reg [9:0] fit_w,pad_x,dx,ox;
 reg [8:0] fit_h,pad_y,oy;
 reg wide_image,black_row;
 reg [25:0] aspect_w,aspect_h;
 reg [31:0] step_x,step_y,hpos,vpos;
 reg [23:0] previous,current,mixed;
 reg same_x;
 reg [7:0] gap;
 reg source_ended;
 reg [23:0] fifo[0:511];
 reg [8:0] wp,rp;
 reg [9:0] count;
 reg [23:0] fifo_q;
 wire pop=(state==H_FETCH || state==SKIP_FETCH || state==WAIT_SOURCE) && count!=0;
 assign sector_ready=count<128;
 assign out_valid=state==O_EMIT && error==0;
 always @(posedge clk) begin
  if(in_valid && count<512) fifo[wp]<=in_pixel;
  if(pop) fifo_q<=fifo[rp];
 end
 always @(posedge clk or posedge rst) begin
  if(rst) begin wp<=0;rp<=0;count<=0;source_ended<=0;end
  else if(start) begin wp<=0;rp<=0;count<=0;source_ended<=0;end
  else begin
   if(source_done) source_ended<=1;
   if(in_valid && count<512) wp<=wp+1'b1;
   if(pop) rp<=rp+1'b1;
   case({in_valid && count<512,pop})
    2'b10:count<=count+1'b1;
    2'b01:count<=count-1'b1;
    default:count<=count;
   endcase
  end
 end
 wire [15:0] ty0=vpos[31:16];
 // vpos settles before BEGIN_ROW and stays fixed while source rows are loaded.
 // Register the clamped next-row coordinate before NEED_ROW comparisons.
 reg [15:0] ty1;
 always @(posedge clk or posedge rst)
  if(rst) ty1<=0;
  else if(state==BEGIN_ROW) ty1<=(ty0>=sh_last) ? ty0 : ty0+1'b1;
 wire [15:0] hx0=hpos[31:16];
 wire [9:0] inner_x=ox-pad_x;
 wire inside_x=ox>=pad_x && ox<pad_x+fit_w;
 wire [23:0] interpolated;
 reg [23:0] line_even[0:1023],line_odd[0:1023];
 reg [23:0] even_q,odd_q;
 always @(posedge clk) begin
  if(state==H_STORE && !sy[0]) line_even[dx]<=interpolated;
  if(state==O_FETCH) even_q<=line_even[inner_x];
 end
 always @(posedge clk) begin
  if(state==H_STORE && sy[0]) line_odd[dx]<=interpolated;
  if(state==O_FETCH) odd_q<=line_odd[inner_x];
 end
 wire [23:0] mix_a=(state==H_MIX) ? (same_x ? current : previous) : (ty0[0] ? odd_q : even_q);
 wire [23:0] mix_b=(state==H_MIX) ? current : (ty1[0] ? odd_q : even_q);
 wire [7:0] weight=(state==H_MIX) ? hpos[15:8] : vpos[15:8];
 bmp_rgb_lerp_pipeline interpolate(.clk(clk),.a(mix_a),.b(mix_b),.w(weight),.rgb(interpolated));
 reg divide_start;
 reg [31:0] numerator,denominator;
 wire divide_done;
 wire [31:0] quotient;
 bmp_scale_divider divider(.clk(clk),.rst(rst || start),.start(divide_start),
   .numerator(numerator),.denominator(denominator[15:0]),.done(divide_done),.quotient(quotient));
 // Coordinates are initialized while idle, before header/configuration.
 // An abort invalidates output through state/error; it need not gate every
 // coordinate register enable through the source-reader control network.
 always @(posedge clk or posedge rst) begin
  if(rst) begin hpos<=0;vpos<=0;end
  else begin
   if(state==IDLE || state==NEED_ROW) hpos<=0;
   else if(state==H_STORE && dx!=fit_w_last)
    hpos<=dx==fit_w_penultimate ? {sw_last,16'd0} : hpos+step_x;
   if(state==IDLE) vpos<=0;
   else if(state==O_GAP && gap==0 && ox==639 && oy!=479 && !black_row)
    vpos<=(oy==image_penultimate_y) ? {sh_last,16'd0} : vpos+step_y;
  end
 end
 // Configuration values are consumed only after their own divider completion.
 // Keep their enables independent of the streaming error/state priority mux.
 always @(posedge clk or posedge rst) begin
  if(rst) begin step_x<=0;step_y<=0;end
  else begin
   if(state==X_WAIT && divide_done) step_x<=quotient;
   if(state==Y_WAIT && divide_done) step_y<=quotient;
  end
 end
 always @(posedge clk or posedge rst) begin
  if(rst) begin
   state<=IDLE;sw<=0;sh<=0;sx<=0;sy<=0;fit_w<=0;fit_h<=0;pad_x<=0;pad_y<=0;
   dx<=0;ox<=0;oy<=0;wide_image<=0;black_row<=0;aspect_w<=0;aspect_h<=0;same_x<=0;
   sw_last<=0;sh_last<=0;h_right<=0;fit_w_last<=0;fit_w_penultimate<=0;image_penultimate_y<=0;
   previous<=0;current<=0;mixed<=0;gap<=0;out_pixel<=0;done<=0;error<=0;
   divide_start<=0;numerator<=0;denominator<=1;
  end else begin
   done<=0;divide_start<=0;
   if(start) begin
    state<=IDLE;error<=0;sx<=0;sy<=0;ox<=0;oy<=0;
   end else if((in_valid && count==512) || (source_done && source_error!=0)) begin
    error<=(source_error!=0) ? source_error : 3'd3;done<=1;state<=IDLE;
   end else case(state)
    IDLE: if(header_valid) begin
     sw<=source_width;sh<=source_height;sw_last<=source_width-1'b1;sh_last<=source_height-1'b1;
     aspect_w<=source_width*26'd480;aspect_h<=source_height*26'd640;
     if(PRESERVE_ASPECT) state<=ASPECT;
     else begin
      // Stretch the complete source independently in X/Y; never add black bars.
      fit_w<=640;fit_h<=480;pad_x<=0;pad_y<=0;state<=SET_X;
     end
    end
    ASPECT: begin
     wide_image<=aspect_w>=aspect_h;
     if(aspect_w>=aspect_h) begin
      fit_w<=640;numerator<={6'd0,aspect_h};denominator<={16'd0,sw};
     end else begin
      fit_h<=480;numerator<={6'd0,aspect_w};denominator<={16'd0,sh};
     end
     divide_start<=1;state<=FIT_WAIT;
    end
    FIT_WAIT: if(divide_done) begin
     if(wide_image) begin fit_h<=quotient==0 ? 9'd1 : quotient[8:0];pad_x<=0;end
     else begin fit_w<=quotient==0 ? 10'd1 : quotient[9:0];pad_y<=0;end
     state<=SET_X;
    end
    SET_X: begin
     pad_x<=(640-fit_w)>>1;pad_y<=(480-fit_h)>>1;
     fit_w_last<=fit_w-1'b1;fit_w_penultimate<=fit_w-2;
     image_penultimate_y<=((480-fit_h)>>1)+fit_h-2;
     numerator<={sw_last,16'd0};denominator<=fit_w>1 ? fit_w-1'b1 : 32'd1;
     divide_start<=1;state<=X_WAIT;
    end
    X_WAIT: if(divide_done) state<=SET_Y;
    SET_Y: begin
     numerator<={sh_last,16'd0};denominator<=fit_h>1 ? fit_h-1'b1 : 32'd1;
     divide_start<=1;state<=Y_WAIT;
    end
    Y_WAIT: if(divide_done) state<=BEGIN_ROW;
    BEGIN_ROW: begin
     ox<=0;
     black_row<=oy<pad_y || oy>=pad_y+fit_h;
     if(oy<pad_y || oy>=pad_y+fit_h) state<=O_FETCH;
     else state<=NEED_ROW;
    end
    NEED_ROW: begin
     sx<=0;dx<=0;h_right<=sw_last==0 ? 16'd0 : 16'd1;
     if(sy>ty1) state<=O_FETCH;
     else if(sy<ty0) state<=SKIP_FETCH;
     else state<=H_FETCH;
    end
    H_FETCH: if(count!=0) state<=H_GOT;
    H_GOT: begin
     previous<=sx==0 ? fifo_q : current;current<=fifo_q;
     if(sx>=h_right) state<=H_SELECT;
     else begin sx<=sx+1'b1;state<=H_FETCH;end
    end
    H_SELECT: begin same_x<=hx0==sx;state<=H_MIX;end
    H_MIX: state<=H_WAIT1;
    H_WAIT1: state<=H_WAIT2;
    H_WAIT2: state<=H_STORE;
    H_STORE: begin
     if(dx==fit_w_last) begin
      if(sx==sw_last) begin sy<=sy+1'b1;state<=NEED_ROW;end
      else begin sx<=sx+1'b1;state<=SKIP_FETCH;end
     end
     else begin
      dx<=dx+1'b1;
      state<=H_TARGET;
     end
    end
    H_TARGET: begin h_right<=hx0>=sw_last ? hx0 : hx0+1'b1;state<=H_ROUTE;end
    H_ROUTE: if(h_right<=sx) state<=H_SELECT;
             else begin sx<=sx+1'b1;state<=H_FETCH;end
    SKIP_FETCH: if(count!=0) state<=SKIP_GOT;
    SKIP_GOT: if(sx==sw_last) begin sy<=sy+1'b1;state<=NEED_ROW;end
              else begin sx<=sx+1'b1;state<=SKIP_FETCH;end
    O_FETCH: state<=O_WAIT;
    O_WAIT: state<=O_MIX;
    O_MIX: state<=O_WAIT1;
    O_WAIT1: state<=O_WAIT2;
    O_WAIT2: state<=O_CAPTURE;
    O_CAPTURE: begin out_pixel<=(black_row || !inside_x) ? 24'd0 : interpolated;state<=O_EMIT;end
    O_EMIT: begin gap<=OUTPUT_GAP;state<=O_GAP;end
    O_GAP: if(gap!=0) gap<=gap-1'b1;
           else if(ox==639) begin
            if(oy==479) state<=WAIT_SOURCE;
            else begin
             oy<=oy+1'b1;state<=BEGIN_ROW;

            end
           end else begin ox<=ox+1'b1;state<=O_FETCH;end
    WAIT_SOURCE: if(source_ended && count==0) begin done<=1;state<=IDLE;end
    default: state<=IDLE;
   endcase
  end
 end
endmodule

// Restoring unsigned division: bounded 32 cycles, no combinational divider.
module bmp_scale_divider(input clk,rst,start,input [31:0] numerator,input [15:0] denominator,
 output reg done,output reg [31:0] quotient);
 reg busy;
 reg [5:0] count;
 reg [31:0] dividend;
 reg [15:0] divisor;
 reg [16:0] remainder;
 wire [16:0] shifted={remainder[15:0],dividend[31]};
 wire subtract=shifted>={1'b0,divisor};
 always @(posedge clk or posedge rst) begin
  if(rst) begin busy<=0;count<=0;dividend<=0;divisor<=1;remainder<=0;quotient<=0;done<=0;end
  else begin
   done<=0;
   if(start && !busy) begin busy<=1;count<=0;dividend<=numerator;divisor<=denominator;remainder<=0;end
   else if(busy) begin
    remainder<=subtract ? shifted-{1'b0,divisor} : shifted;
    dividend<={dividend[30:0],subtract};
    if(count==31) begin quotient<={dividend[30:0],subtract};done<=1;busy<=0;end
    else count<=count+1'b1;
   end
  end
 end
endmodule

// Three registered stages: operand selection, products, then rounded sum.
// The weights total exactly 256 and every RGB channel is preserved.
module bmp_rgb_lerp_pipeline(input clk,input [23:0] a,b,input [7:0] w,output [23:0] rgb);
 reg [23:0] aq,bq;
 reg [8:0] wa,wb;
 reg [15:0] ar,ag,ab,br,bg,bb;
 reg [16:0] sr,sg,sb;
 always @(posedge clk) begin
  aq<=a;bq<=b;wa<=9'd256-{1'b0,w};wb<={1'b0,w};
  ar<=aq[23:16]*wa;ag<=aq[15:8]*wa;ab<=aq[7:0]*wa;
  br<=bq[23:16]*wb;bg<=bq[15:8]*wb;bb<=bq[7:0]*wb;
  sr<={1'b0,ar}+{1'b0,br}+17'd128;
  sg<={1'b0,ag}+{1'b0,bg}+17'd128;
  sb<={1'b0,ab}+{1'b0,bb}+17'd128;
 end
 assign rgb={sr[15:8],sg[15:8],sb[15:8]};
endmodule
