`timescale 1ns/1ps
module frame_read_write
#
(
	parameter MEM_DATA_BITS          = 32,
	parameter READ_DATA_BITS         = 32,
	parameter WRITE_DATA_BITS        = 32,
	parameter ADDR_BITS              = 21,
	parameter BURST_BITS             = 9,//
	parameter BURST_SIZE             = 256,
	parameter WRITE_V_FLIP           = 1,      // 1: vertical flip when writing TF/BMP frame into SDRAM
	parameter FRAME_WIDTH            = 640,
	parameter FRAME_HEIGHT           = 480,
    parameter TRANSITIONS            = 0,
    parameter THUMBNAILS             = 0
) 
(
	input                            rst,                  
	input                            mem_clk,                    // external memory controller user interface clock
	input							 Sdr_init_done,
	input							 Sdr_init_ref_vld,
    input							 Sdr_busy,
	
    /*
    output                           rd_burst_req,               // to external memory controller,send out a burst read request
	output[BURST_BITS - 1:0]         rd_burst_len,               // to external memory controller,data length of the burst read request, not bytes
	output[ADDR_BITS - 1:0]          rd_burst_addr,              // to external memory controller,base address of the burst read request 
	input                            rd_burst_data_valid,        // from external memory controller,read data valid 
	input[MEM_DATA_BITS - 1:0]       rd_burst_data,              // from external memory controller,read request data
	input                            rd_burst_finish,            // from external memory controller,burst read finish
	*/
	output							 App_rd_en,
	output[ADDR_BITS - 1:0]			 App_rd_addr,
	input							 Sdr_rd_en,					 //read_data_valid
	input[MEM_DATA_BITS - 1:0]		 Sdr_rd_dout,
	
	input                            read_clk,                   // data read module clock
	input                            read_req,                   // data read module read request,keep '1' until read_req_ack = '1'
	output                           read_req_ack,               // data read module read request response
	output                           read_finish,                // data read module read request finish
	input[ADDR_BITS - 1:0]           read_addr_0,                // data read module read request base address 0, used when read_addr_index = 0
	input[ADDR_BITS - 1:0]           read_addr_1,                // data read module read request base address 1, used when read_addr_index = 1
	input[ADDR_BITS - 1:0]           read_addr_2,                // data read module read request base address 1, used when read_addr_index = 2
	input[ADDR_BITS - 1:0]           read_addr_3,                // data read module read request base address 1, used when read_addr_index = 3
	input[1:0]                       read_addr_index,            // select valid base address from read_addr_0 read_addr_1 read_addr_2 read_addr_3
	input[ADDR_BITS - 1:0]           read_len,                   // data read module read request data length
	input                            read_en,                    // data read module read request for one data, read_data valid next clock
	output[READ_DATA_BITS  - 1:0]    read_data,                  // read data
	/*
	output                           wr_burst_req,               // to external memory controller,send out a burst write request
	output[BURST_BITS - 1:0]         wr_burst_len,               // to external memory controller,data length of the burst write request, not bytes
	output[ADDR_BITS - 1:0]          wr_burst_addr,              // to external memory controller,base address of the burst write request 
	input                            wr_burst_data_req,          // from external memory controller,write data request ,before data 1 clock
	output[MEM_DATA_BITS - 1:0]      wr_burst_data,              // to external memory controller,write data
	input                            wr_burst_finish,            // from external memory controller,burst write finish
	*/
	output							 App_wr_en,
	output[ADDR_BITS - 1:0]			 App_wr_addr,
	output[MEM_DATA_BITS - 1:0]	 App_wr_din,
	output[3:0]						 App_wr_dm,
	
	//
	input                            write_clk,                  // data write module clock
	input                            write_req,                  // data write module write request,keep '1' until read_req_ack = '1'
	output                           write_req_ack,              // data write module write request response
	output                           write_finish,               // data write module write request finish
	input[ADDR_BITS - 1:0]           write_addr_0,               // data write module write request base address 0, used when write_addr_index = 0
	input[ADDR_BITS - 1:0]           write_addr_1,               // data write module write request base address 1, used when write_addr_index = 1
	input[ADDR_BITS - 1:0]           write_addr_2,               // data write module write request base address 1, used when write_addr_index = 2
	input[ADDR_BITS - 1:0]           write_addr_3,               // data write module write request base address 1, used when write_addr_index = 3
	input[1:0]                       write_addr_index,           // select valid base address from write_addr_0 write_addr_1 write_addr_2 write_addr_3
	input[ADDR_BITS - 1:0]           write_len,                  // data write module write request data length
	input                            write_en,                   // data write module write request for one data
	input[WRITE_DATA_BITS - 1:0]     write_data,                // write data
    input                          write_recovery_reset,
    output [5:0]                   write_diagnostics,
    output reg                     read_underflow,
    input                          wipe_active,
    input [1:0]                    wipe_old_bank,
    input [9:0]                    wipe_boundary,
    input [1:0]                    wipe_kind,
    input                          thumbnail_write,
    input                          thumbnail_read

);
wire[BURST_BITS - 1:0]                           wrusedw;                    // write used words
wire[BURST_BITS - 1:0]                           rdusedw;                    // read used words

wire 								 App_rd_busy;
wire								 App_wr_busy;

wire read_fifo_empty;
wire [ADDR_BITS-1:0] linear_read_addr;
wire read_fifo_aclr, write_fifo_aclr;
wire [ADDR_BITS-1:0] effect_read_addr, normal_write_addr;
generate if(THUMBNAILS) begin : g_thumbnails
    thumbnail_memory_address #(.ADDR_BITS(ADDR_BITS)) addresses(
      .clk(mem_clk),.rst(rst),.read_clear(read_fifo_aclr),
      .write_clear(write_fifo_aclr),.read_enable(App_rd_en),.write_enable(App_wr_en),
      .preview(thumbnail_read),.thumbnail_write(thumbnail_write),.write_bank(write_addr_index),
      .normal_read(effect_read_addr),.normal_write(normal_write_addr),
      .read_address(App_rd_addr),.write_address(App_wr_addr));
end else begin : g_no_thumbnails
    assign App_rd_addr=effect_read_addr;
    assign App_wr_addr=normal_write_addr;
end endgenerate

reg read_request_d;
always @(posedge read_clk or posedge rst) begin
    if(rst) begin read_underflow<=0;read_request_d<=0;end
    else begin
        read_request_d<=read_req;
        if(read_req && !read_request_d) read_underflow<=0;
        else if(read_en && (read_fifo_empty || read_fifo_aclr)) read_underflow<=1;
    end
end

wire O_wr_busy;
wire O_rd_busy;
// Global reset enters this multi-clock integration block asynchronously.
// Existing FIFO IPs synchronize release internally in both FIFO clock domains.
reg [2:0] global_mem_release, global_write_release;
always @(posedge mem_clk or posedge rst)
    if(rst) global_mem_release<=3'b111;
    else global_mem_release<={global_mem_release[1:0],1'b0};
always @(posedge write_clk or posedge rst)
    if(rst) global_write_release<=3'b111;
    else global_write_release<={global_write_release[1:0],1'b0};
wire reader_reset = rst || global_mem_release[2];
generate if(TRANSITIONS) begin : g_transition_address
    transition_read_address #(.ADDR_BITS(ADDR_BITS),.FRAME_WIDTH(FRAME_WIDTH)) mapper(
      .clk(mem_clk),.rst(reader_reset),.frame_clear(read_fifo_aclr),
      .read_enable(App_rd_en),.linear_addr(linear_read_addr),
      .new_bank(read_addr_index),.old_bank(wipe_old_bank),
      .wipe_active(wipe_active),.kind(wipe_kind),.boundary(wipe_boundary),
      .base0(read_addr_0),.base1(read_addr_1),.base2(read_addr_2),.base3(read_addr_3),
      .mapped_addr(effect_read_addr));
end else begin : g_linear_address
    assign effect_read_addr=linear_read_addr;
end endgenerate
wire source_reset = rst || global_write_release[2];
// Abort only the writer/FIFO; the display reader and SDRAM keep running.
// Asynchronous assertion, synchronous release in each participating domain.
reg [2:0] recovery_mem, recovery_write;
always @(posedge mem_clk or posedge rst or posedge write_recovery_reset)
    if(rst || write_recovery_reset) recovery_mem<=3'b111;
    else recovery_mem<={recovery_mem[1:0],1'b0};
always @(posedge write_clk or posedge rst or posedge write_recovery_reset)
    if(rst || write_recovery_reset) recovery_write<=3'b111;
    else recovery_write<={recovery_write[1:0],1'b0};
wire writer_reset = rst || recovery_mem[2];
wire write_fifo_empty, write_fifo_full;
wire write_fifo_reset = write_fifo_aclr || writer_reset || recovery_write[2];
// Register the available-word estimate before burst arithmetic/arbitration.
// Subtract this edge's dequeue so the delayed estimate never includes a word
// already consumed. Newly synchronized writes become available one cycle later.
reg [9:0] writer_available;
always @(posedge mem_clk or posedge write_fifo_reset) begin
    if(write_fifo_reset) writer_available<=0;
    else if(write_fifo_empty) writer_available<=0;
    else writer_available<=rdusedw - (App_wr_en ? 10'd1 : 10'd0);
end
// The protected writer acknowledges while its FIFO clear is STILL asserted.
// Complete that internal request first; acknowledge the application only after
// clear has released in write_clk (including the FIFO's two release cycles).
localparam WR_IDLE=2'd0, WR_REQUEST=2'd1, WR_DRAIN=2'd2, WR_READY=2'd3;
reg [1:0] write_handshake;
wire write_req_ack_mem;
reg [1:0] write_ack_sync;
reg [3:0] fifo_release_guard;
wire write_req_mem = write_handshake==WR_REQUEST;
assign write_req_ack = write_handshake==WR_READY;
always @(posedge write_clk or posedge write_fifo_reset) begin
    if(write_fifo_reset) fifo_release_guard<=4'b1111;
    else fifo_release_guard<={fifo_release_guard[2:0],1'b0};
end
wire handshake_reset = source_reset || recovery_write[2];
always @(posedge write_clk or posedge handshake_reset) begin
    if(handshake_reset) begin
        write_handshake<=WR_IDLE; write_ack_sync<=0;
    end else begin
        write_ack_sync<={write_ack_sync[0],write_req_ack_mem};
        case(write_handshake)
            WR_IDLE: if(write_req && !recovery_write[2] && !write_ack_sync[1])
                         write_handshake<=WR_REQUEST;
            WR_REQUEST: if(write_ack_sync[1]) write_handshake<=WR_DRAIN;
            WR_DRAIN: if(!write_ack_sync[1] && fifo_release_guard==0)
                          write_handshake<=WR_READY;
            WR_READY: if(!write_req) write_handshake<=WR_IDLE;
            default: write_handshake<=WR_IDLE;
        endcase
    end
end
// Passive observers only. No diagnostic signal drives the data/control path.
reg diag_req_d, diag_overflow, diag_reset_drop;
reg diag_underflow, diag_finished, diag_all_consumed;
reg [ADDR_BITS-1:0] diag_consumed;
reg [3:0] diag_meta, diag_sync;
always @(posedge write_clk or posedge source_reset) begin
    if(source_reset) begin
        diag_req_d<=0; diag_overflow<=0; diag_reset_drop<=0;
        diag_meta<=0; diag_sync<=0;
    end else begin
        diag_req_d<=write_req;
        diag_meta<={diag_finished,diag_all_consumed,!write_fifo_empty,diag_underflow};
        diag_sync<=diag_meta;
        if(write_req && !diag_req_d) begin
            diag_overflow<=0; diag_reset_drop<=0;
        end else if(write_en) begin
            if(write_fifo_full) diag_overflow<=1;
            if(write_fifo_reset) diag_reset_drop<=1;
        end
    end
end
always @(posedge mem_clk or posedge writer_reset) begin
    if(writer_reset) begin
        diag_consumed<=0; diag_underflow<=0; diag_finished<=0; diag_all_consumed<=0;
    end else if(write_fifo_aclr) begin
        diag_consumed<=0; diag_underflow<=0; diag_finished<=0; diag_all_consumed<=0;
    end else begin
        if(write_finish) diag_finished<=1;
        if(App_wr_en) begin
            if(write_fifo_empty) diag_underflow<=1;
            else begin
                if(!(&diag_consumed)) diag_consumed<=diag_consumed+1'b1;
                if(diag_consumed==write_len-1'b1) diag_all_consumed<=1;
            end
        end
    end
end
assign write_diagnostics={diag_sync,diag_reset_drop,diag_overflow};

assign App_wr_busy = O_wr_busy;//下个时钟写指令拉高
assign App_rd_busy = O_rd_busy;//读指令还没有过去十个时钟;因为写指令优先，不需要在意下个时钟是否是读指令
assign App_wr_dm = 4'b0000;

//instantiate an asynchronous FIFO 
wfifo_32_32_512 write_buf
	(
	.clkr                      	(mem_clk                  ),          // Read side clock
	.clkw                      	(write_clk                ),          // Write side clock
	.rst                       	(write_fifo_reset),
	.we                      	(write_en                 ),          // Write Request
	.re                      	(App_wr_en        		  ),          // Read Request
	.di                       	(write_data               ),          // Input Data
	.empty_flag                 (write_fifo_empty         ),          // Read side Empty flag
	.full_flag                  (write_fifo_full          ),          // Write side Full flag
	.wrusedw                	(              	  		  ),          // Read Used Words
	.rdusedw                	(rdusedw                  ),          // Write Used Words
	.dout                       (App_wr_din		          )
);

frame_fifo_write
#
(
	.MEM_DATA_BITS              (MEM_DATA_BITS            ),
	.ADDR_BITS                  (ADDR_BITS                ),
	.BURST_BITS                 (BURST_BITS               ),
	.BURST_SIZE                 (BURST_SIZE               ),
	.WRITE_V_FLIP               (WRITE_V_FLIP             ),
	.FRAME_WIDTH                (FRAME_WIDTH              ),
	.FRAME_HEIGHT               (FRAME_HEIGHT             )
) 
frame_fifo_write_m0              
(  
	.rst                        (writer_reset             ),
	.mem_clk                    (mem_clk                  ),
    .Sdr_init_done				(Sdr_init_done),
	.Sdr_init_ref_vld			(Sdr_init_ref_vld),
    .Sdr_busy					(Sdr_busy),
	.App_rd_busy				(App_rd_busy),
    .O_wr_busy					(O_wr_busy),
	
	.App_wr_en					(App_wr_en),
    .App_wr_addr				(normal_write_addr),
	.write_req                  (write_req_mem            ),
	.write_req_ack              (write_req_ack_mem        ),
	.write_finish               (write_finish             ),
	.write_addr_0               (write_addr_0             ),
	.write_addr_1               (write_addr_1             ),
	.write_addr_2               (write_addr_2             ),
	.write_addr_3               (write_addr_3             ),
	.write_addr_index           (write_addr_index         ),    
	.write_len                  (write_len                ),
	.fifo_aclr                  (write_fifo_aclr          ),
	.rdusedw                 	(writer_available         )
);

//instantiate an asynchronous FIFO 
rfifo_32_32_512 read_buf
	(
	.clkr                     	(read_clk                   ),          // Read side clock
	.clkw                     	(mem_clk                    ),          // Write side clock
	.rst                      	(read_fifo_aclr || reader_reset),          // IP synchronizes release
	.we                     	(Sdr_rd_en       			),          // Write Request
	.re                     	(read_en                    ),          // Read Request
	.di                      	(Sdr_rd_dout                ),          // Input Data
	.empty_flag					(read_fifo_empty            ),          // Read side Empty flag
	.full_flag					(                           ),          // Write side Full flag
	.wrusedw                	(wrusedw         	  	  		 ),          // Read Used Words
	.rdusedw                	(                  			 ),          // Write Used Words
	.dout						(read_data                  )
);

frame_fifo_read
#
(
	.MEM_DATA_BITS              (MEM_DATA_BITS            ),
	.ADDR_BITS                  (ADDR_BITS                ),
	.BURST_BITS                 (BURST_BITS               ),
	.BURST_SIZE                 (TRANSITIONS ? ((FRAME_WIDTH % 40 == 0) ? 40 : 8) : BURST_SIZE)
)
frame_fifo_read_m0
(
	.rst                        (reader_reset             ),
	.mem_clk                    (mem_clk                  ),
    .Sdr_init_done				(Sdr_init_done),
	.Sdr_init_ref_vld			(Sdr_init_ref_vld),
    .Sdr_busy					(Sdr_busy),
    .Sdr_rd_en					(Sdr_rd_en),
	.App_wr_busy				(App_wr_busy),
    .O_rd_busy					(O_rd_busy),
	
    .App_rd_en					(App_rd_en),
    .App_rd_addr				(linear_read_addr),
	.read_req                   (read_req                 ),
	.read_req_ack               (read_req_ack             ),
	.read_finish                (read_finish              ),
	.read_addr_0                (read_addr_0              ),
	.read_addr_1                (read_addr_1              ),
	.read_addr_2                (read_addr_2              ),
	.read_addr_3                (read_addr_3              ),
	.read_addr_index            (read_addr_index          ),    
	.read_len                   (read_len                 ),
	.fifo_aclr                  (read_fifo_aclr           ),
	.wrusedw                	(wrusedw                  )
);

endmodule

// One read per output pixel. Forty-word read bursts keep all 40-pixel wipe
// boundaries and 640-pixel line boundaries BETWEEN bursts, never within one.
// Parameters are held in pixel_clk for the entire frame, before read_req;
// the protected reader's synchronized request and FIFO clear provide settling.
// Application address translation only; the original FIFO controllers are unchanged.
// Four packed 160x120 banks follow the four 640x480 original banks.
// Read bursts remain 40 words, so tile and scan-line changes occur between bursts.
module thumbnail_memory_address #(parameter ADDR_BITS=21)(
 input clk,rst,read_clear,write_clear,read_enable,write_enable,
 input preview,thumbnail_write,input [1:0] write_bank,
 input [ADDR_BITS-1:0] normal_read,normal_write,
 output [ADDR_BITS-1:0] read_address,write_address);
 localparam [ADDR_BITS-1:0] BASE=1228800;
 reg preview_q,write_q;
 reg [9:0] x;
 reg [8:0] y;
 reg [14:0] row_offset;
 reg [ADDR_BITS-1:0] wp;
 // Image edges align to the protected 40-word read burst.
 wire right_tile=x>=360;
 wire [9:0] local_x=right_tile ? x-10'd360 : x-10'd120;
 wire [ADDR_BITS-1:0] tile_base=BASE+(y>=280 ? 21'd38400 : 21'd0)+
                                            (right_tile ? 21'd19200 : 21'd0);
 wire image_window=((x>=120 && x<280) || (x>=360 && x<520)) &&
                   ((y>=104 && y<224) || (y>=280 && y<400));
 assign read_address=preview_q ? (image_window ?
                          tile_base+row_offset+local_x : BASE) : normal_read;
 assign write_address=write_q ? wp : normal_write;
 always @(posedge clk or posedge rst) begin
  if(rst) begin preview_q<=0;x<=0;y<=0;row_offset<=19040;end
  else if(read_clear) begin preview_q<=preview;x<=0;y<=0;row_offset<=19040;end
  else if(read_enable) begin
   if(x==639) begin
    x<=0;y<=y+1'b1;
    if(y==103 || y==279) row_offset<=19040;
    else if((y>=104 && y<223) || (y>=280 && y<399)) row_offset<=row_offset-15'd160;
   end else x<=x+1'b1;
  end
 end
 always @(posedge clk or posedge rst) begin
  if(rst) begin write_q<=0;wp<=BASE;end
  else if(write_clear) begin
   write_q<=thumbnail_write;
   case(write_bank)
    0:wp<=BASE;
    1:wp<=BASE+21'd19200;
    2:wp<=BASE+21'd38400;
    3:wp<=BASE+21'd57600;
   endcase
  // Store bottom-up, contiguously: never jump addresses inside a write burst.
  // Reading reverses rows, with every jump aligned to a 40-word burst boundary.
  end else if(write_enable) wp<=wp+1'b1;
 end
endmodule

// Transition geometry shared by the memory read mapper and the pixel-domain
// light. Kind 0 opens from the centre toward the top/bottom; kind 1 opens
// toward the left/right. The horizontal opening moves in 40-pixel increments
// so picture-bank changes remain between the protected reader's 40-word bursts.
// q is the eased progress, 0..1023. The pixel light is darken(c,dk)+glow.
module transition_geometry(
 input [1:0] kind,input [9:0] q,x,input [8:0] y,
 output reg use_new,output reg [8:0] src_row,
 output reg [1:0] dk,output reg [7:0] glow);
 function [7:0] seam;
  input [10:0] d;
  case(d) 0:seam=255;1:seam=190;2:seam=110;3:seam=60;4:seam=30;5:seam=14;6:seam=6; default:seam=0; endcase
 endfunction
 function [1:0] shadow;
  input [9:0] d;
  shadow=d==0 ? 2'd0 : d<=2 ? 2'd3 : d<=5 ? 2'd2 : d<=10 ? 2'd1 : 2'd0;
 endfunction
 function [10:0] absd;
  input signed [11:0] a;
  absd=a[11] ? -a : a;
 endfunction
 // Vertical half-height is 0..240; horizontal half-width is 0..320.
 // Horizontal edges align with the SDRAM reader's 40-word bursts.
 wire [13:0] q15={q,4'b0}-{4'b0,q};
 wire [8:0] v_half=({1'b0,q15[13:5]}+10'd1)>>1;
 wire [10:0] q_round={1'b0,q}+11'd64;
 wire [3:0] h_steps=q_round[10:7];
 wire [8:0] h_half={h_steps,5'b0}+{2'b0,h_steps,3'b0};
 wire [8:0] v_top=9'd240-v_half, v_bottom=9'd240+v_half;
 wire [9:0] h_left=10'd320-{1'b0,h_half}, h_right=10'd320+{1'b0,h_half};
 wire [8:0] v_dl=y-v_top, v_dr=v_bottom-9'd1-y;
 wire [9:0] h_dl=x-h_left, h_dr=h_right-10'd1-x;
 wire [7:0] v_seam_top=seam(absd($signed({3'b0,y})-$signed({3'b0,v_top})));
 wire [7:0] v_seam_bottom=seam(absd($signed({3'b0,y})-$signed({3'b0,v_bottom})));
 wire [7:0] h_seam_left=seam(absd($signed({2'b0,x})-$signed({2'b0,h_left})));
 wire [7:0] h_seam_right=seam(absd($signed({2'b0,x})-$signed({2'b0,h_right})));
 wire [10:0] push_round={1'b0,q}+11'd32;
 wire [4:0] push_steps=push_round[10:6];
 wire [9:0] push_amount={push_steps,5'b0}+{2'b0,push_steps,3'b0};
 wire [9:0] push_edge=kind[0] ? push_amount:10'd640-push_amount;
 always @* begin
  use_new=0;src_row=y;dk=0;glow=0;
  if(kind==0) begin
   if(y<v_top) src_row=y+v_half;
   else if(y>=v_bottom) src_row=y-v_half;
   else use_new=1;
   if(v_half!=0 && v_half!=9'd240) begin
    glow=v_seam_top>v_seam_bottom ? v_seam_top : v_seam_bottom;
    if(use_new) dk=shadow({1'b0,(v_dl<v_dr ? v_dl : v_dr)}+10'd1);
   end
  end else if(kind==1) begin
   use_new=(x>=h_left && x<h_right);
   if(h_half!=0 && h_half!=9'd320) begin
    glow=h_seam_left>h_seam_right ? h_seam_left : h_seam_right;
    if(use_new) dk=shadow((h_dl<h_dr ? h_dl : h_dr)+10'd1);
   end
  end else begin
   use_new=kind[0] ? x<push_edge:x>=push_edge;
   if(push_amount!=0 && push_amount!=640) begin
    glow=seam(absd($signed({2'b0,x})-$signed({2'b0,push_edge})));
    dk=shadow(absd($signed({2'b0,x})-$signed({2'b0,push_edge})));
   end
  end
 end
endmodule

// Maps the reader's linear frame addresses onto the old/new pictures during a
// transition. Parameters are held in pixel_clk for the whole frame before
// read_req and sampled during the FIFO clear. Vertical offsets are prepared
// a line ahead; horizontal changes use 40-pixel-aligned boundaries.
// No SDRAM burst crosses a picture-bank or row-address discontinuity.
module transition_read_address #(parameter ADDR_BITS=21,FRAME_WIDTH=640)(
 input clk,rst,frame_clear,read_enable,input [ADDR_BITS-1:0] linear_addr,
 input [1:0] new_bank,old_bank,input wipe_active,input [1:0] kind,input [9:0] boundary,
 input [ADDR_BITS-1:0] base0,base1,base2,base3,
 output [ADDR_BITS-1:0] mapped_addr);
 function [ADDR_BITS-1:0] base;
 input [1:0] bank;
 begin case(bank) 0:base=base0;1:base=base1;2:base=base2;3:base=base3;endcase end
 endfunction
 reg active_q;reg [1:0] kind_q;
 reg [1:0] new_q,old_q;
 reg [9:0] q_q;
 reg [9:0] xi;
 reg [8:0] la_y,s1_y;
 reg priming;
 // Prepare vertical rows two cycles ahead. The horizontal mode keeps both
 // banks' line bases, then changes banks only between 40-word bursts.
 wire v_use;wire [8:0] v_row;
 transition_geometry geometry(.kind(2'd0),.q(q_q),.x(10'd0),.y(la_y),
  .use_new(v_use),.src_row(v_row),.dk(),.glow());
 reg s1_use;reg [8:0] s1_row;
 reg [ADDR_BITS-1:0] la_base,cur_base,la_old_base,la_new_base,cur_old_base,cur_new_base;
 wire [ADDR_BITS-1:0] v_offset={s1_row,9'b0}+{2'b0,s1_row,7'b0};
 wire [ADDR_BITS-1:0] h_offset={s1_y,9'b0}+{2'b0,s1_y,7'b0};
 wire [10:0] q_round={1'b0,boundary}+11'd64;
 wire [3:0] h_steps=q_round[10:7];
 wire [8:0] h_half={h_steps,5'b0}+{2'b0,h_steps,3'b0};
 wire [10:0] push_round={1'b0,boundary}+11'd32;
 wire [4:0] push_steps=push_round[10:6];
 wire [9:0] push_amount={push_steps,5'b0}+{2'b0,push_steps,3'b0};
 reg [9:0] push_q,push_edge_q;
 reg [8:0] h_q;
 reg [9:0] left_q,right_q;
 wire line_end=read_enable && xi==FRAME_WIDTH-1;
 always @(posedge clk or posedge rst) begin
  if(rst) begin
   push_q<=0;push_edge_q<=640;active_q<=0;kind_q<=0;new_q<=0;old_q<=0;q_q<=0;xi<=0;priming<=0;la_y<=0;
   s1_use<=0;s1_row<=0;s1_y<=0;la_base<=0;cur_base<=0;
   la_old_base<=0;la_new_base<=0;cur_old_base<=0;cur_new_base<=0;
   h_q<=0;left_q<=10'd320;right_q<=10'd320;
  end else begin
   s1_use<=v_use;s1_row<=v_row;s1_y<=la_y;
   la_base<=base(s1_use ? new_q : old_q)+v_offset;
   la_old_base<=base(old_q)+h_offset;
   la_new_base<=base(new_q)+h_offset;
   if(frame_clear) begin
    active_q<=wipe_active;kind_q<=kind;new_q<=new_bank;old_q<=old_bank;q_q<=boundary;
    push_q<=push_amount;push_edge_q<=kind[0] ? push_amount:10'd640-push_amount;
    h_q<=h_half;left_q<=10'd320-{1'b0,h_half};right_q<=10'd320+{1'b0,h_half};
    xi<=0;la_y<=0;priming<=1;
   end else begin
    if(priming) begin
     cur_base<=la_base;cur_old_base<=la_old_base;cur_new_base<=la_new_base;
     priming<=0;la_y<=1;
    end else if(line_end) begin
     cur_base<=la_base;cur_old_base<=la_old_base;cur_new_base<=la_new_base;
     la_y<=la_y+1'b1;
    end
    if(read_enable) xi<=xi==FRAME_WIDTH-1 ? 10'd0 : xi+1'b1;
   end
  end
 end
 wire h_new=xi>=left_q && xi<right_q;
 wire [9:0] h_col=h_new ? xi : (xi<left_q ? xi+{1'b0,h_q} : xi-{1'b0,h_q});
 wire push_new=kind_q[0] ? xi<push_edge_q:xi>=push_edge_q;
 wire [9:0] push_col=kind_q[0] ? (push_new ? xi+10'd640-push_q:xi-push_q):
                                             (push_new ? xi-push_edge_q:xi+push_q);
 wire use_horizontal=kind_q!=0;
 wire select_new=kind_q[1] ? push_new:h_new;
 wire [ADDR_BITS-1:0] line_base=use_horizontal ? (select_new ? cur_new_base:cur_old_base):cur_base;
 wire [9:0] source_col=kind_q[1] ? push_col:kind_q[0] ? h_col:xi;
 assign mapped_addr=active_q ? line_base+{{(ADDR_BITS-10){1'b0}},source_col} : linear_addr;
endmodule
