// PCM streams are signed 16-bit stereo; all ports use the pixel clock domain.
// Lesson 12 v3 (2026-10-06): ambient audio visual on the 640x480 frame.
//  A. Three interweaving glowing sine curves (pink, cyan, violet) centred on
//     y=YC, drawn as crisp antialiased lines: positions in 1/4 row, each
//     column joined to the previous one, a white-hot core and a short halo. Amplitude follows the smoothed level (instant attack, ~130 ms
//     release, log scale); the curves pinch to the centre line at the screen
//     edges and drift sideways, faster when the music is louder.
//  B. The lower part of the frame dims gently into a dark stage (full 4x4
//     ordered dither, 0 % at y=SHADE_Y rising to 69 % near the bottom) so the
//     light reads even over white pictures.
//  C. Screen edges glow with the music (squared level, slower release).
//  Everything fades in/out over 15 frames; when faded out rgb_out equals
//  rgb_in exactly. Picture and sync pass through a plain two-clock delay
//  (also in reset): rgb_in's long upstream overlay chain is not lengthened,
//  and the light is computed in two register stages (P1 intensities, P2 colour).
module audio_waveform_video #(parameter integer YC=426)(
 input push,push_right,input [9:0] push_shift,
 input clk,rst,vsync,de,enable,pcm_valid,input signed [15:0] pcm_left,
 input [23:0] rgb_in,output reg [23:0] rgb_out,output reg vsync_out,de_out);
 localparam [9:0] CX=384, SHADE_Y=480, EDGE=32;
 localparam [23:0] C_PINK=24'h864f1a, C_CYAN=24'hb7402e, C_VIOLET=24'h565432;

 // ---------------- raster position of the current input pixel ----------------
 reg [9:0] scan_x,y;
 wire [9:0] x;wire push_new;
 page_push_coordinate u_push(.active(push),.right(push_right),.amount(push_shift),.scan_x(scan_x),.source_x(x),.new_page(push_new));
 reg vs_q,de_q;
 wire frame_start=vs_q && !vsync;
 always @(posedge clk or posedge rst) begin
  if(rst) begin scan_x<=0;y<=0;vs_q<=1;de_q<=0;end
  else begin
   vs_q<=vsync;de_q<=de;
   if(!vsync) begin scan_x<=0;y<=0;end
   else if(de) scan_x<=scan_x+1'b1;
   else begin scan_x<=0;if(de_q) y<=y+1'b1;end
  end
 end

 // ---------------- per-frame envelopes and phases ----------------
 function [6:0] log_level;  // 8*msb + next three bits: 6 dB per 8 steps
  input [15:0] m;
  integer i;
  begin
   log_level=0;
   for(i=3;i<16;i=i+1) if(m[i]) log_level={i[3:0],m[i-1-:3]};
  end
 endfunction
 wire [15:0] magnitude=pcm_left[15] ? (~pcm_left+16'd1) : pcm_left;
 reg [15:0] frame_peak,level,glow_env;
 reg [5:0] amp;              // 0..48 rows
 reg [7:0] glow_gain;
 reg [9:0] ph1,ph2,ph3;      // 1/1024 cycle
 reg [3:0] fade;
 // >= : a steady tone (peak equal to the envelope) must hold, not decay.
 wire [15:0] level_next=frame_peak>=level ? frame_peak : level-(level>>3);
 wire [15:0] glow_next=frame_peak>=glow_env ? frame_peak : glow_env-(glow_env>>4);
 wire [6:0] l_amp=log_level(level_next), l_glow=log_level(glow_next);
 wire [9:0] a_scaled={3'd0,l_amp-7'd56}*10'd11;
 wire [5:0] amp_next=l_amp<=7'd56 ? 6'd0 : a_scaled[9:4]>6'd48 ? 6'd48 : a_scaled[9:4];
 wire [9:0] g_scaled={3'd0,l_glow-7'd56}*10'd7;
 wire [7:0] g_lin=l_glow<=7'd56 ? 8'd0 : g_scaled[9] ? 8'hff : g_scaled[8:1];
 wire [15:0] g_sq=g_lin*g_lin;
 wire [3:0] speed=amp[5:2];  // louder -> faster drift
 always @(posedge clk or posedge rst) begin
  if(rst) begin
   frame_peak<=0;level<=0;glow_env<=0;amp<=0;glow_gain<=0;fade<=0;
   ph1<=0;ph2<=10'd341;ph3<=10'd683;
  end else if(frame_start) begin
   frame_peak<=0;
   level<=level_next;glow_env<=glow_next;
   // Limit envelope movement to one row/frame to avoid visible pumping.
   if(amp<amp_next) amp<=amp+1'b1;
   else if(amp>amp_next) amp<=amp-1'b1;
   glow_gain<={1'b0,g_sq[15:9]}+{3'd0,g_sq[15:11]};  // 5/8 of square
   ph1<=ph1+10'd4+speed;
   ph2<=ph2-10'd3-{1'b0,speed[3:1]};
   ph3<=ph3+10'd7+{speed,1'b0};
   if(enable) begin if(fade!=4'd15) fade<=fade+1'b1; end
   else if(fade!=0) fade<=fade-1'b1;
  end else if(enable && pcm_valid && magnitude>frame_peak) frame_peak<=magnitude;
 end

 // ---------------- tables ----------------
 function [7:0] taper_rom;  // cos^1.6 over 0..320 px from the centre, 8 px bins
  input [5:0] i;
  case(i)
   0:taper_rom=255; 1:taper_rom=255; 2:taper_rom=254; 3:taper_rom=252; 4:taper_rom=250; 5:taper_rom=247; 6:taper_rom=244; 7:taper_rom=240; 8:taper_rom=235; 9:taper_rom=230; 10:taper_rom=225; 11:taper_rom=219; 12:taper_rom=212; 13:taper_rom=205; 14:taper_rom=198; 15:taper_rom=190; 16:taper_rom=182; 17:taper_rom=173; 18:taper_rom=165; 19:taper_rom=156; 20:taper_rom=146; 21:taper_rom=137; 22:taper_rom=128; 23:taper_rom=118; 24:taper_rom=109; 25:taper_rom=100; 26:taper_rom=90; 27:taper_rom=81; 28:taper_rom=72; 29:taper_rom=63; 30:taper_rom=55; 31:taper_rom=47; 32:taper_rom=39; 33:taper_rom=32; 34:taper_rom=25; 35:taper_rom=19; 36:taper_rom=13; 37:taper_rom=8; 38:taper_rom=4; 39:taper_rom=1;
   default:taper_rom=0; endcase
 endfunction
 function [7:0] scale8;  // c*w/256
  input [7:0] c,w;
  reg [15:0] p;
  begin p=c*w;scale8=p[15:8]; end
 endfunction
 function [23:0] scale24;
  input [23:0] c;input [7:0] w;
  scale24={scale8(c[23:16],w),scale8(c[15:8],w),scale8(c[7:0],w)};
 endfunction
 function [7:0] sat_add;
  input [7:0] a,b;
  reg [8:0] s;
  begin s=a+b;sat_add=s[8] ? 8'hff : s[7:0]; end
 endfunction
 function [23:0] sat_add24;
  input [23:0] a,b;
  sat_add24={sat_add(a[23:16],b[23:16]),sat_add(a[15:8],b[15:8]),sat_add(a[7:0],b[7:0])};
 endfunction

 // ---------------- per-column pipeline, four pixels ahead ----------------
 // Curves span 1.5, 2.25 and 3.25 cycles across the 640 px screen.
 wire [9:0] xf=x+10'd4;
 wire [9:0] df=xf>=CX ? xf-CX : CX-xf;
 wire [13:0] xa1=xf*4'd10, xa2=xf*4'd15, xa3={xf,4'b0}+{xf,2'b0}+xf;      // x*10, x*15, x*21
 reg [9:0] an1,an2,an3;reg [5:0] bin0;reg [2:0] frac0;
 wire signed [8:0] s1,s2,s3;
 reg [7:0] wa1,wb1;reg [2:0] frac1;
 waveform_sine_bram u_sine1(clk,an1,s1);
 waveform_sine_bram u_sine2(clk,an2,s2);
 waveform_sine_bram u_sine3(clk,an3,s3);
 reg signed [8:0] s1b,s2b,s3b;reg [7:0] w2;
 // Curve rows in 1/4 row units (+-188, positive is up): current and previous column.
 reg signed [8:0] r1,r2,r3,p1,p2,p3;reg [7:0] w3;
 wire signed [9:0] tdiff=$signed({2'b0,wb1})-$signed({2'b0,wa1});
 wire signed [12:0] tstep=tdiff*$signed({1'b0,frac1});
 wire [7:0] w_lerp=wa1+tstep[10:3];
 wire [13:0] h_mult=amp*w2;          // 0..48*255
 // Keep all taper fractional bits until the final quarter-row rounding.
 wire signed [24:0] o1=s1b*$signed({1'b0,h_mult}), o2=s2b*$signed({1'b0,h_mult}), o3=s3b*$signed({1'b0,h_mult});
 wire signed [24:0] o2_scaled=o2-(o2>>>2), o3_scaled=o3>>>1;
 always @(posedge clk) begin
  // S0: angles and taper bin
  an1<=xa1[11:2]+ph1;an2<=xa2[11:2]+ph2;an3<=xa3[11:2]+ph3;
  bin0<=df[9:3]>7'd39 ? 6'd39 : df[8:3];frac0<=df[9:3]>7'd39 ? 3'd7 : df[2:0];
  // S1: sine and taper lookups
  // Sine BRAMs perform the same one-clock S1 lookup in parallel.
  wa1<=taper_rom(bin0);wb1<=taper_rom(bin0+1'b1);frac1<=frac0;
  // S2: interpolated taper
  s1b<=s1;s2b<=s2;s3b<=s3;w2<=w_lerp;
  // S3: row offsets (curve 2 scaled 3/4, curve 3 scaled 1/2 of curve 1) and taper
  r1<=(o1+25'sd16384)>>>15;
  r2<=(o2_scaled+25'sd16384)>>>15;r3<=(o3_scaled+25'sd16384)>>>15;
  p1<=r1;p2<=r2;p3<=r3;
  w3<=w2;
 end

 // ---------------- per-pixel light for the current input pixel ----------------
 wire signed [10:0] yrel=$signed({1'b0,YC[9:0]})-$signed({1'b0,y});   // up positive
 // Line profile in 1/4 rows: approximately 4-pixel width at half intensity
 // with an antialiased edge, then a short halo.
 function [7:0] profile;
  input [12:0] q;
  case(q) 0,1,2,3,4,5,6:profile=255;7:profile=224;8:profile=160;9:profile=80;10:profile=24;default:profile=0;endcase
 endfunction
 function [7:0] hot;      // white-hot centre of the line
  input [12:0] q;
  case(q) 0:hot=255;1:hot=170;2:hot=85; default:hot=0; endcase
 endfunction
 // Distance (1/4 rows) from this pixel's centre to the span joining the
 // previous column's position to this column's: steep slopes stay connected.
 function [12:0] span_gap;
  input signed [10:0] row;            // pixel row, up positive, in whole rows
  input signed [8:0] a,b;             // span ends in 1/4 rows
  reg signed [12:0] yq,lo,hi;
  begin
   yq={row,2'b00};
   lo=a<b ? a : b; hi=a<b ? b : a;
   span_gap=yq<lo ? lo-yq : yq>hi ? yq-hi : 13'd0;
  end
 endfunction
 wire [12:0] q1=span_gap(yrel,r1,x==0 ? r1 : p1);
 wire [12:0] q2=span_gap(yrel,r2,x==0 ? r2 : p2);
 wire [12:0] q3=span_gap(yrel,r3,x==0 ? r3 : p3);
 wire [12:0] qmin=q1<q2 ? (q1<q3 ? q1 : q3) : (q2<q3 ? q2 : q3);
 wire [7:0] i1=scale8(profile(q1),w3);
 wire [7:0] i2=scale8(profile(q2),w3);
 wire [7:0] i3=scale8(profile(q3),w3);
 wire [7:0] i0=scale8(hot(qmin),w3)>>2;            // white core, also at crossings
 wire [9:0] ex=x<10'd320 ? x : 10'd639-x, ey=y<10'd240 ? y : 10'd479-y;
 wire [9:0] e=ex<ey ? ex : ey;
 wire [5:0] er=EDGE-e[5:0];
 wire [11:0] edge_sq=er*er;                           // (32-e)^2
 wire [7:0] edge_lin=e>=EDGE ? 8'd0 : edge_sq[11:10]!=0 ? 8'hff : edge_sq[9:2];
 wire [15:0] glow_t=edge_lin*glow_gain;
 // Edge colour runs violet (left) -> cyan (right) in 64 steps.
 wire [5:0] t=x[9:4];                                // 0..39
 function [7:0] mix;
  input [7:0] a,b;input [5:0] f;
  reg signed [9:0] d;reg signed [15:0] p;
  begin d=$signed({2'b0,b})-$signed({2'b0,a});p=d*$signed({1'b0,f});mix=a+p[13:6]; end
 endfunction
 wire [5:0] tw=t+(t>>1)+(t>>3);                     // ~1.6*t, 0..62
 wire [23:0] edge_col={mix(C_VIOLET[23:16],C_CYAN[23:16],tw),
                       mix(C_VIOLET[15:8],C_CYAN[15:8],tw),
                       mix(C_VIOLET[7:0],C_CYAN[7:0],tw)};
 function [7:0] fade8;
  input [7:0] a;input [3:0] f;
  reg [12:0] p;
  begin p=a*({1'b0,f}+5'd1);fade8=p[11:4]; end
 endfunction
 // Bottom shade: k/16 darkening, ordered 4x4 dither.
 function [3:0] bayer;
  input [1:0] a,b;
  case({b,a})
   0:bayer=0;1:bayer=8;2:bayer=2;3:bayer=10;4:bayer=12;5:bayer=4;6:bayer=14;7:bayer=6;
   8:bayer=3;9:bayer=11;10:bayer=1;11:bayer=9;12:bayer=15;13:bayer=7;14:bayer=13;15:bayer=5;
  endcase
 endfunction
 wire [9:0] sy=y-SHADE_Y+{6'd0,bayer(x[1:0],y[1:0])};
 wire [3:0] k=y<SHADE_Y ? 4'd0 : sy[9:4]>6'd11 ? 4'd11 : sy[7:4];
 wire [7:0] kf=k*({1'b0,fade}+5'd1);
 wire touched=fade!=0 && de && (x>=140 && x<620 && y>=400 && y<450);

 // ---------------- P1: per-pixel intensities ----------------
 reg [7:0] i1_p,i2_p,i3_p,i0_p,gt_p;
 reg [23:0] edge_col_p,rgb_p;
 reg [3:0] k_p;
 reg vs_p,de_p,touched_p;
 always @(posedge clk) begin
  i1_p<=i1;i2_p<=i2;i3_p<=i3;i0_p<=i0;gt_p<=glow_t[15:8];edge_col_p<=edge_col;
  k_p<=kf[7:4];rgb_p<=rgb_in;vs_p<=vsync;de_p<=de;
 end
 // ---------------- P2: colour, fade, output registers ----------------
 wire [23:0] curves=sat_add24(sat_add24(scale24(C_PINK,i1_p),scale24(C_CYAN,i2_p)),
                              sat_add24(scale24(C_VIOLET,i3_p),{3{i0_p}}));
 wire [23:0] light=sat_add24(sat_add24(scale24(C_PINK,i1_p),scale24(C_CYAN,i2_p)),scale24(C_VIOLET,i3_p));
 reg [23:0] rgb_q,light_q;
 reg [3:0] k_q;
 reg touched_q;
 always @(posedge clk) begin
  rgb_q<=rgb_p;vsync_out<=vs_p;de_out<=de_p;k_q<=k_p;
  light_q<={fade8(light[23:16],fade),fade8(light[15:8],fade),fade8(light[7:0],fade)};
 end
 always @(posedge clk or posedge rst)
  if(rst) begin touched_p<=0;touched_q<=0; end
  else begin touched_p<=touched;touched_q<=touched_p; end
 function [7:0] shade;
  input [7:0] c;input [3:0] kk;
  reg [11:0] p;
  begin p=c*kk;shade=c-p[11:4]; end
 endfunction
 function [7:0] ink;input [7:0] c,a;begin ink=c>a ? c-a:0;end endfunction
 always @* rgb_out=touched_q ?
  {ink(rgb_q[23:16],light_q[23:16]),ink(rgb_q[15:8],light_q[15:8]),ink(rgb_q[7:0],light_q[7:0])}:rgb_q;
endmodule

// The caller changes mux selection only after a pending sample is accepted.
module audio_source_mux (
    input wire wav_valid, output wire wav_ready,
    input wire signed [15:0] wav_left, wav_right,
    input wire wav_active,
    input wire arp_valid, output wire arp_ready,
    input wire signed [15:0] arp_left, arp_right,
    input wire force_arpeggio,
    output wire out_valid, input wire out_ready,
    output wire signed [15:0] out_left, out_right
);
wire use_wav = wav_active && !force_arpeggio;
assign out_valid = use_wav ? wav_valid : arp_valid;
assign out_left = use_wav ? wav_left : arp_left;
assign out_right = use_wav ? wav_right : arp_right;
assign wav_ready = use_wav && out_ready;
assign arp_ready = !use_wav && out_ready;
endmodule

// 8 KiB asynchronous PCM queue. Only one playback per reset; no unilateral
// pointer reset on EOF/error. Tail samples drain before returning to the tone.
// Reserve a WHOLE sector before allowing the FAT streamer to issue a request.
module wav_pcm_buffer #(parameter ADDR_WIDTH=12, PREFILL=256)(
 input wr_clk,wr_rst,byte_valid,input [7:0] byte_data,input file_finished,
 output sector_ready,output reg overflow,
 input rd_clk,rd_rst,input sample_ready,
 output signed [15:0] sample_data,output sample_valid,
 output reg stream_active,output reg stream_end,output reg underrun,
 output reg [31:0] sample_counter);
 localparam DEPTH=1<<ADDR_WIDTH;
 reg [15:0] memory[0:DEPTH-1]; /* fehdl force_ram=1, ram_style="bram" */
 reg [ADDR_WIDTH:0] wr_bin,wr_gray,rd_bin,rd_gray;
 (* async_reg="true" *) reg [ADDR_WIDTH:0] rd_sync1,rd_sync2,wr_sync1,wr_sync2;
 (* async_reg="true" *) reg [2:0] done_sync;
 reg low_pending;reg [7:0] low_byte;
 reg holding;reg signed [15:0] read_sample;
 function [ADDR_WIDTH:0] binary;
 input [ADDR_WIDTH:0] gray;integer i;
 begin binary[ADDR_WIDTH]=gray[ADDR_WIDTH];
  for(i=ADDR_WIDTH-1;i>=0;i=i-1) binary[i]=binary[i+1]^gray[i];
 end endfunction
 // Register Gray decoding separately from occupancy subtraction. The extra
 // write-domain cycle only delays recognition of newly freed space.
 reg [ADDR_WIDTH:0] rd_binary_wr;
 always @(posedge wr_clk or posedge wr_rst)
  if(wr_rst) rd_binary_wr<=0;
  else rd_binary_wr<=binary(rd_sync2);
 wire [ADDR_WIDTH:0] wr_level=wr_bin-rd_binary_wr;
 wire [ADDR_WIDTH:0] rd_level=binary(wr_sync2)-rd_bin;
 wire [ADDR_WIDTH:0] wr_next=wr_bin+1'b1,rd_next=rd_bin+1'b1;
 wire store_word=byte_valid && low_pending && wr_level<DEPTH && !file_finished;
 wire fetch=stream_active && (!holding || sample_ready) && rd_level!=0;
 assign sector_ready=wr_level<=DEPTH-258 && !overflow;
 assign sample_data=read_sample;
 assign sample_valid=stream_active && holding;
 always @(posedge wr_clk) if(store_word) memory[wr_bin[ADDR_WIDTH-1:0]]<={byte_data,low_byte};
 always @(posedge rd_clk) if(fetch) read_sample<=memory[rd_bin[ADDR_WIDTH-1:0]];
 always @(posedge wr_clk or posedge wr_rst) begin
  if(wr_rst) begin wr_bin<=0;wr_gray<=0;rd_sync1<=0;rd_sync2<=0;low_pending<=0;low_byte<=0;overflow<=0;end
  else begin
   rd_sync1<=rd_gray;rd_sync2<=rd_sync1;
   if(byte_valid && !file_finished) begin
    if(!low_pending) begin low_byte<=byte_data;low_pending<=1;end
    else begin
     low_pending<=0;
     if(wr_level>=DEPTH) overflow<=1;
     else begin wr_bin<=wr_next;wr_gray<=wr_next^(wr_next>>1);end
    end
   end
  end
 end
 always @(posedge rd_clk or posedge rd_rst) begin
  if(rd_rst) begin
   rd_bin<=0;rd_gray<=0;wr_sync1<=0;wr_sync2<=0;done_sync<=0;holding<=0;
   stream_active<=0;stream_end<=0;underrun<=0;sample_counter<=0;
  end else begin
   wr_sync1<=wr_gray;wr_sync2<=wr_sync1;done_sync<={done_sync[1:0],file_finished};
   if(!stream_active && !stream_end && (rd_level>=PREFILL || (done_sync[2] && rd_level!=0))) stream_active<=1;
   if(done_sync[2] && rd_level==0 && (!holding || sample_ready)) begin stream_active<=0;stream_end<=1;end
   if(fetch) begin rd_bin<=rd_next;rd_gray<=rd_next^(rd_next>>1);holding<=1;end
   else if(sample_ready) holding<=0;
   if(sample_ready && sample_valid) sample_counter<=sample_counter+1'b1;
   if(sample_ready && stream_active && !holding && !done_sync[2]) underrun<=1;
  end
 end
endmodule

// Coherent eight-bit settings mailbox; payload stays fixed until acknowledged.
module audio_volume_cdc #(parameter [7:0] DEFAULT_VOLUME=128)(
 input source_clk,source_rst,input [7:0] source_volume,
 input pixel_clk,pixel_rst,output reg [7:0] pixel_volume);
 reg [7:0] payload;reg req,ack;
 (* async_reg="true" *) reg [1:0] ack_sync;
 (* async_reg="true" *) reg [2:0] req_sync;
 always @(posedge source_clk or posedge source_rst) begin
  if(source_rst) begin payload<=DEFAULT_VOLUME;req<=0;ack_sync<=0;end
  else begin
   ack_sync<={ack_sync[0],ack};
   if(req==ack_sync[1] && payload!=source_volume) begin payload<=source_volume;req<=~req;end
  end
 end
 always @(posedge pixel_clk or posedge pixel_rst) begin
  if(pixel_rst) begin pixel_volume<=DEFAULT_VOLUME;ack<=0;req_sync<=0;end
  else begin
   req_sync<={req_sync[1:0],req};
   if(req_sync[2]!=ack) begin pixel_volume<=payload;ack<=req_sync[2];end
  end
 end
endmodule

// Demand-driven DDS: phase/note duration advance only on accepted samples.
// mode=0: fixed 1 kHz test; mode=1: six-note sequence for media_id.
module pcm_tone_source #(
    parameter integer MEDIA_MODE = 0,
    parameter integer NOTE_SAMPLES = 12000,
    parameter integer BLIP_SAMPLES = 4800
)(
    input wire clk, rst, restart,
    input wire [1:0] media_id,
    output wire sample_valid, input wire sample_ready,
    output wire signed [15:0] sample_left, sample_right
);
reg [19:0] phase;
reg [31:0] note_count, blip_count;
reg [2:0] note_index;
function [19:0] note_inc;
    input [1:0] id;
    input [2:0] n;
    reg [15:0] hz;
    begin
        case(id)
            0:case(n) 0,5:hz=262;1,3:hz=330;2:hz=392;default:hz=294;endcase
            1:case(n) 0,5:hz=220;1,4:hz=262;2:hz=330;default:hz=294;endcase
            2:case(n) 0,5:hz=175;1,3:hz=220;2:hz=262;default:hz=196;endcase
            3:case(n) 0,5:hz=196;1,3:hz=247;2:hz=294;default:hz=220;endcase
        endcase
        case(hz)
            175:note_inc=3823;196:note_inc=4282;220:note_inc=4806;
            247:note_inc=5396;262:note_inc=5723;294:note_inc=6423;
            330:note_inc=7209;default:note_inc=8563;
        endcase
    end
endfunction
function signed [15:0] sine;
 input [5:0] i;
 begin case(i)
  0:sine=0;
  1:sine=686;
  2:sine=1366;
  3:sine=2032;
  4:sine=2679;
  5:sine=3300;
  6:sine=3889;
  7:sine=4441;
  8:sine=4950;
  9:sine=5411;
  10:sine=5820;
  11:sine=6173;
  12:sine=6467;
  13:sine=6699;
  14:sine=6865;
  15:sine=6966;
  16:sine=7000;
  17:sine=6966;
  18:sine=6865;
  19:sine=6699;
  20:sine=6467;
  21:sine=6173;
  22:sine=5820;
  23:sine=5411;
  24:sine=4950;
  25:sine=4441;
  26:sine=3889;
  27:sine=3300;
  28:sine=2679;
  29:sine=2032;
  30:sine=1366;
  31:sine=686;
  32:sine=0;
  33:sine=-686;
  34:sine=-1366;
  35:sine=-2032;
  36:sine=-2679;
  37:sine=-3300;
  38:sine=-3889;
  39:sine=-4441;
  40:sine=-4950;
  41:sine=-5411;
  42:sine=-5820;
  43:sine=-6173;
  44:sine=-6467;
  45:sine=-6699;
  46:sine=-6865;
  47:sine=-6966;
  48:sine=-7000;
  49:sine=-6966;
  50:sine=-6865;
  51:sine=-6699;
  52:sine=-6467;
  53:sine=-6173;
  54:sine=-5820;
  55:sine=-5411;
  56:sine=-4950;
  57:sine=-4441;
  58:sine=-3889;
  59:sine=-3300;
  60:sine=-2679;
  61:sine=-2032;
  62:sine=-1366;
  63:sine=-686;
 endcase end
endfunction
assign sample_valid = !rst && !restart;
// Equal nominal peak for all four teaching pages: 3500 (half the old 7000).
// 2048-sample attack/release (~43 ms), followed by a 1/8 one-pole low-pass.
// Shift/add envelope scaling avoids consuming another DSP multiplier.
wire signed [15:0] raw_tone=sine(phase[19:14]);
reg [6:0] note_gain;
reg [4:0] envelope_div;
reg signed [23:0] soft_state;
function signed [22:0] envelope_scale;
 input signed [15:0] value;
 input [6:0] weight;
 reg signed [22:0] v,total;
 integer k;
 begin
  v={{7{value[15]}},value};total=0;
  for(k=0;k<7;k=k+1) if(weight[k]) total=total+(v<<<k);
  envelope_scale=total;
 end
endfunction
wire signed [22:0] weighted_tone=envelope_scale(raw_tone,note_gain);
wire signed [15:0] soft_target=weighted_tone>>>7;
wire signed [24:0] soft_delta=$signed({soft_target[15],soft_target,8'b0})-
                                   $signed({soft_state[23],soft_state});
assign sample_left = MEDIA_MODE ? soft_state[23:8] : raw_tone;
always @(posedge clk or posedge rst) begin
 if(rst) begin note_gain<=0;envelope_div<=0;soft_state<=0;end
 else if(restart) begin note_gain<=0;envelope_div<=0;soft_state<=0;end
 else if(MEDIA_MODE && sample_valid && sample_ready) begin
  soft_state<=soft_state+(soft_delta>>>3);
  envelope_div<=envelope_div+1'b1;
  if(note_count==NOTE_SAMPLES-1) begin note_gain<=0;envelope_div<=0;end
  else if(envelope_div==31) begin
   if(note_count>=NOTE_SAMPLES-2048) begin
    if(note_gain!=0) note_gain<=note_gain-1'b1;
   end else if(note_gain<64) note_gain<=note_gain+1'b1;
  end
 end
end
assign sample_right = sample_left;
always @(posedge clk or posedge rst) begin
    if(rst) begin
        phase<=0; note_count<=0; note_index<=0; blip_count<=0;
    end else if(restart) begin
        phase<=0; note_count<=0; note_index<=0;
        blip_count<=MEDIA_MODE ? BLIP_SAMPLES : 0;
    end else if(sample_valid && sample_ready) begin
        phase <= phase + (!MEDIA_MODE ? 20'd21845 :
                          blip_count!=0 ? 20'd32768 : note_inc(media_id,note_index));
        if(blip_count!=0) blip_count<=blip_count-1'b1;
        else if(MEDIA_MODE) begin
            if(note_count==NOTE_SAMPLES-1) begin
                note_count<=0; note_index<=note_index==5 ? 3'd0 : note_index+1'b1;
            end else note_count<=note_count+1'b1;
        end
    end
end
endmodule

module media_audio_output #(
    parameter integer PIXEL_HZ = 25000000,
    parameter integer MODE_SELECT = 0,
    parameter integer VIDEO_FADE = 0
)(
    input wire clk, rst,
    // These controls must be synchronous to clk. Top provides coherent CDC.
    input wire enable, force_test, force_arpeggio,
    input wire [1:0] media_id,
    input wire [7:0] volume,
    input wire [7:0] video_alpha,
    input wire narration_mode,
    input wire wav_active, wav_valid,
    input wire signed [15:0] wav_left, wav_right,
    output wire wav_ready,
    output reg audio_valid,
    output reg [23:0] audio_left, audio_right,
    output reg acr_valid,
    output wire [19:0] acr_n, acr_cts,
    output reg [1:0] selected_source,
    output reg underrun
);
localparam [1:0] MEDIA=0, WAV=1, TEST=2, SILENCE=3;
wire [1:0] requested = !enable ? SILENCE : force_test ? TEST :
                       (MODE_SELECT && narration_mode) ? WAV :
                       (wav_active && !force_arpeggio) ? WAV : MEDIA;
reg [1:0] active_media;
reg [31:0] sample_acc;
wire sample_tick = sample_acc >= PIXEL_HZ-48000;
reg [8:0] gain; // 0..256; approximately 5.3 ms fade each way
reg restart;
reg [5:0] acr_count;
wire transition = requested!=selected_source ||
                  (selected_source==MEDIA && media_id!=active_media);
wire test_valid, arp_valid, test_ready, arp_ready;
wire signed [15:0] test_l,test_r,arp_l,arp_r,mux_l,mux_r;
wire mux_valid,mux_ready;
wire ready = sample_tick && selected_source!=SILENCE && !restart &&
             (selected_source!=WAV || wav_active);
pcm_tone_source u_test(clk,rst,restart,active_media,test_valid,test_ready,test_l,test_r);
pcm_tone_source #(.MEDIA_MODE(1),.NOTE_SAMPLES(36000),.BLIP_SAMPLES(0)) u_media(clk,rst,restart,active_media,arp_valid,arp_ready,arp_l,arp_r);
audio_source_mux u_mux (
    .wav_valid(wav_valid),.wav_ready(wav_ready),.wav_left(wav_left),.wav_right(wav_right),
    .wav_active(selected_source==WAV),.arp_valid(arp_valid),.arp_ready(arp_ready),
    .arp_left(arp_l),.arp_right(arp_r),.force_arpeggio(1'b0),
    .out_valid(mux_valid),.out_ready(mux_ready),.out_left(mux_l),.out_right(mux_r)
);
assign mux_ready = ready && selected_source!=TEST;
assign test_ready = ready && selected_source==TEST;
wire chosen_valid = selected_source==TEST ? test_valid : mux_valid;
wire signed [15:0] chosen_l = selected_source==TEST ? test_l : mux_l;
wire signed [15:0] chosen_r = selected_source==TEST ? test_r : mux_r;
// Full volume 255 uses unity gain; other levels are volume/256.
wire signed [9:0] vol_gain = volume==255 ? 10'sd256 : $signed({2'b00,volume});
wire signed [25:0] vl = chosen_l * vol_gain;
wire signed [25:0] vr = chosen_r * vol_gain;
wire signed [35:0] gl = vl * $signed({1'b0,gain});
wire signed [35:0] gr = vr * $signed({1'b0,gain});
wire signed [9:0] picture_gain = !VIDEO_FADE || video_alpha==255 ? 10'sd256 : $signed({2'b00,video_alpha});
wire signed [45:0] final_l=gl*picture_gain,final_r=gr*picture_gain;
// Signed16 -> signed24 (left aligned), incorporating /256 volume and /256 fade.
assign acr_n = 20'd6144;
assign acr_cts = PIXEL_HZ / 1000;
always @(posedge clk or posedge rst) begin
    if(rst) begin
        sample_acc<=0; audio_valid<=0; audio_left<=0; audio_right<=0;
        acr_valid<=0; acr_count<=0; selected_source<=SILENCE;
        active_media<=0; gain<=0; restart<=0; underrun<=0;
    end else begin
        audio_valid<=0; acr_valid<=0; restart<=0;
        if(sample_tick) begin
            sample_acc<=sample_acc+48000-PIXEL_HZ;
            audio_valid<=1;
            audio_left<=0; audio_right<=0;
            if(ready && chosen_valid) begin
                audio_left<=final_l >>> 16; audio_right<=final_r >>> 16;
            end
            if(ready && !chosen_valid) underrun<=1;
            if(acr_count==47) begin acr_count<=0; acr_valid<=1; end
            else acr_count<=acr_count+1'b1;
            if(transition) begin
                if(gain!=0) gain<=gain-1'b1;
                else begin
                    selected_source<=requested;
                    active_media<=media_id;
                    restart<=1;
                end
            end else if(selected_source!=SILENCE && gain<256) gain<=gain+1'b1;
        end else sample_acc<=sample_acc+48000;
    end
end
endmodule


// Application Flash settings modules (kept in this registered source file).
// U33 application Flash ONLY. Never connect this block to U39 configuration SPI.
// Reserve the final two 4-KiB sectors of the 8-MiB part. Alternating records
// preserve the previous value through interrupted erase/program operations.
module flash_volume_store #(
 parameter integer START_CYCLES=1000000, SETTLE_CYCLES=100000000,
 parameter integer TIMEOUT_CYCLES=200000000, SPI_DIV=20,
 parameter [7:0] DEFAULT_VOLUME=128
)(input clk,rst,input [7:0] volume,
 output reg restore_valid,output reg [7:0] restore_volume,
 output reg ready,output reg [7:0] error,
 output cs_n,sclk,mosi,input miso);
 reg start; reg [4:0] count; reg [159:0] tx;
 wire done; wire [159:0] rx;
 flash_settings_spi #(.DIV(SPI_DIV)) spi(clk,rst,start,count,tx,miso,cs_n,sclk,mosi,done,rx);
 reg [5:0] state,ret;
 reg [31:0] timer,deadline,sequence_number;
 reg active_sector,target_sector,have_record;
 reg [127:0] record0,record1,expected;
 reg [7:0] saved_volume,pending_volume,observed_volume;
 reg [1:0] operation;
 wire [23:0] address=target_sector ? 24'h7ff000:24'h7fe000;
 wire [63:0] payload={24'h564f4c,sequence_number+32'd1,pending_volume};
 function valid_record;
 input [127:0] r;
 begin valid_record=(r[127:104]==24'h564f4c) && (r[63:0]==~r[127:64]) &&
 (r[71:64]==0 || r[71:64]==64 || r[71:64]==128 || r[71:64]==192 || r[71:64]==255); end
 endfunction
 task command;
 input [159:0] data; input [4:0] bytes; input [5:0] next;
 begin tx<=data;count<=bytes;start<=1;ret<=next;state<=63;end
 endtask
 task fail;
 input [7:0] code;
 begin error<=code;ready<=1;state<=62;end
 endtask
 always @(posedge clk or posedge rst) begin
 if(rst) begin
  start<=0;count<=0;tx<=0;state<=0;ret<=0;timer<=0;deadline<=0;
  sequence_number<=0;active_sector<=0;target_sector<=0;have_record<=0;
  record0<=0;record1<=0;expected<=0;operation<=0;
  saved_volume<=DEFAULT_VOLUME;pending_volume<=DEFAULT_VOLUME;observed_volume<=DEFAULT_VOLUME;
  ready<=0;error<=0;restore_valid<=0;restore_volume<=DEFAULT_VOLUME;
 end else begin
  start<=0;restore_valid<=0;
  if(state!=62 && deadline<TIMEOUT_CYCLES) deadline<=deadline+1;
  case(state)
   0: if(timer==START_CYCLES-1) begin timer<=0; command({8'h05,152'd0},2,7);end else timer<=timer+1;
   7: if(deadline>=TIMEOUT_CYCLES) fail(8'h52);
      else if(rx[0]) command({8'h05,152'd0},2,7);
      else command({8'h9f,152'd0},4,1);
   1: if(rx[23:0]==24'hef4017 || rx[23:0]==24'hef3017)
       command({8'h03,24'h7fe000,128'd0},20,2);
      else fail(8'h50);
   2: begin record0<=rx[127:0];command({8'h03,24'h7ff000,128'd0},20,3);end
   3: begin record1<=rx[127:0];state<=4;end
   4: begin
     ready<=1;restore_valid<=1;timer<=0;state<=5;
     if(valid_record(record1) && (!valid_record(record0) ||
         ($signed(record1[103:72]-record0[103:72])>0))) begin
       active_sector<=1;have_record<=1;sequence_number<=record1[103:72];
       saved_volume<=record1[71:64];restore_volume<=record1[71:64];
     end else if(valid_record(record0)) begin
       active_sector<=0;have_record<=1;sequence_number<=record0[103:72];
       saved_volume<=record0[71:64];restore_volume<=record0[71:64];
     end
   end
   5: begin // give application time to apply restore; no boot-time write
     observed_volume<=volume;timer<=0;state<=6;
   end
   6: begin
     deadline<=0;
     if(volume!=observed_volume) begin observed_volume<=volume;timer<=0;end
     else if(volume==saved_volume) timer<=0;
     else if(timer==SETTLE_CYCLES-1) begin
       pending_volume<=volume;target_sector<=have_record ? ~active_sector:0;
       operation<=0;timer<=0;state<=10;
     end else timer<=timer+1;
   end
   10: command({8'h06,152'd0},1,11); // write enable
   11: command({8'h05,152'd0},2,12);
   12: if(!rx[1] || rx[0]) fail(8'h51);
       else if(operation==0) command({8'h20,address,128'd0},4,13);
       else begin expected<={payload,~payload};command({8'h02,address,payload,~payload},20,13);end
   13: command({8'h05,152'd0},2,14);
   14: if(deadline>=TIMEOUT_CYCLES) fail(8'h52);
       else if(rx[0]) state<=13;
       else if(operation==0) begin operation<=1;state<=10;end
       else command({8'h03,address,128'd0},20,15);
   15: if(rx[127:0]!=expected) fail(8'h53);
       else begin
         saved_volume<=pending_volume;active_sector<=target_sector;have_record<=1;
         sequence_number<=sequence_number+1;timer<=0;state<=6;
       end
   63: if(done) state<=ret;
   62: ; // fail closed until reset; audio/buttons remain operational
   default: state<=0;
  endcase
 end
 end
endmodule

// Mode 0, MSB first; CS is high for several system clocks between commands.
module flash_settings_spi #(parameter integer DIV=20)(
 input clk,rst,start,input [4:0] bytes,input [159:0] tx,input miso,
 output reg cs_n,sclk,output mosi,output reg done,output reg [159:0] rx);
 reg [159:0] shift; reg [7:0] bits; reg [15:0] divider;
 assign mosi=shift[159];
 always @(posedge clk or posedge rst) begin
  if(rst) begin cs_n<=1;sclk<=0;done<=0;rx<=0;shift<=0;bits<=0;divider<=0;end
  else begin
   done<=0;
   if(cs_n) begin
    sclk<=0;divider<=0;
    if(start) begin cs_n<=0;shift<=tx;bits<={bytes,3'b000};rx<=0;end
   end else if(divider==DIV-1) begin
    divider<=0;sclk<=~sclk;
    if(!sclk) rx<={rx[158:0],miso};
    else if(bits==1) begin cs_n<=1;sclk<=0;done<=1;end
    else begin bits<=bits-1;shift<={shift[158:0],1'b0};end
   end else divider<=divider+1;
  end
 end
endmodule

// 1024-phase quarter-wave lookup; three independent read ports use three BRAMs.
// Quarter magnitudes use low eight data bits (parity unused). One-clock latency.
module waveform_sine_bram(input clk,input [9:0] angle,output signed [8:0] value);
 wire [7:0] addr=angle[8] ? ~angle[7:0] : angle[7:0];
 reg negative;
 always @(posedge clk) negative<=angle[9];
 wire [8:0] q;
 assign value=negative ? -$signed({1'b0,q[7:0]}) : $signed({1'b0,q[7:0]});
`ifndef __ICARUS__
 EG_PHY_BRAM #(
  .MODE("SP8K"),.DATA_WIDTH_A("9"),.DATA_WIDTH_B("9"),
  .REGMODE_A("NOREG"),.REGMODE_B("NOREG"),.GSR("DISABLE"),
  .CEAMUX("1"),.CEBMUX("0"),.OCEAMUX("1"),.OCEBMUX("0"),
  .WEAMUX("0"),.WEBMUX("0"),.RSTAMUX("0"),.RSTBMUX("0"),
  .CSA0("1"),.CSA1("1"),.CSA2("1"),
  .INIT_00(256'h312f2e2c2b292826252322201e1d1b1a1817151412100f0d0c0a090705040201),
  .INIT_01(256'h615f5e5d5b5a5857555452514f4e4c4b494846454342403f3d3c3a3937363433),
  .INIT_02(256'h8d8c8a89888685848281807e7d7c7a797876757372716f6e6c6b696867656462),
  .INIT_03(256'hb4b3b2b0afaeadacabaaa8a7a6a5a4a2a1a09f9e9c9b9a99979695939291908e),
  .INIT_04(256'hd4d3d2d1d0cfcecdcccbcacac9c8c7c6c5c4c3c2c1c0bfbdbcbbbab9b8b7b6b5),
  .INIT_05(256'hebebeae9e9e8e8e7e6e6e5e4e3e3e2e1e1e0dfdedddddcdbdadad9d8d7d6d5d4),
  .INIT_06(256'hfafaf9f9f9f8f8f8f7f7f6f6f6f5f5f4f4f3f3f2f2f1f1f0f0efefeeeeedecec),
  .INIT_07(256'hfffffffffffffffffffffefefefefefefefefdfdfdfdfdfcfcfcfcfbfbfbfbfa),
  .INIT_08(256'h0000000000000000000000000000000000000000000000000000000000000000),
  .INIT_09(256'h0000000000000000000000000000000000000000000000000000000000000000),
  .INIT_0A(256'h0000000000000000000000000000000000000000000000000000000000000000),
  .INIT_0B(256'h0000000000000000000000000000000000000000000000000000000000000000),
  .INIT_0C(256'h0000000000000000000000000000000000000000000000000000000000000000),
  .INIT_0D(256'h0000000000000000000000000000000000000000000000000000000000000000),
  .INIT_0E(256'h0000000000000000000000000000000000000000000000000000000000000000),
  .INIT_0F(256'h0000000000000000000000000000000000000000000000000000000000000000),
  .INIT_10(256'h0000000000000000000000000000000000000000000000000000000000000000),
  .INIT_11(256'h0000000000000000000000000000000000000000000000000000000000000000),
  .INIT_12(256'h0000000000000000000000000000000000000000000000000000000000000000),
  .INIT_13(256'h0000000000000000000000000000000000000000000000000000000000000000),
  .INIT_14(256'h0000000000000000000000000000000000000000000000000000000000000000),
  .INIT_15(256'h0000000000000000000000000000000000000000000000000000000000000000),
  .INIT_16(256'h0000000000000000000000000000000000000000000000000000000000000000),
  .INIT_17(256'h0000000000000000000000000000000000000000000000000000000000000000),
  .INIT_18(256'h0000000000000000000000000000000000000000000000000000000000000000),
  .INIT_19(256'h0000000000000000000000000000000000000000000000000000000000000000),
  .INIT_1A(256'h0000000000000000000000000000000000000000000000000000000000000000),
  .INIT_1B(256'h0000000000000000000000000000000000000000000000000000000000000000),
  .INIT_1C(256'h0000000000000000000000000000000000000000000000000000000000000000),
  .INIT_1D(256'h0000000000000000000000000000000000000000000000000000000000000000),
  .INIT_1E(256'h0000000000000000000000000000000000000000000000000000000000000000),
  .INIT_1F(256'h0000000000000000000000000000000000000000000000000000000000000000)
 ) sine_bram(
  .clka(clk),.addra({2'b00,addr,3'b000}),.doa(q),
  .dia(9'd0),.csa(3'b111),.cea(1'b1),.ocea(1'b1),.wea(1'b0),.rsta(1'b0),
  .clkb(1'b0),.addrb(13'd0),.dob(),.dib(9'd0),.csb(3'b000),
  .ceb(1'b0),.oceb(1'b0),.web(1'b0),.rstb(1'b0));
`else
 // Syntax/portable model shares the exact table bytes used in hardware.
 reg [7:0] table_data[0:255];
 reg [7:0] q_data;
 assign q={1'b0,q_data};
 initial begin
  table_data[0]=8'd1;
  table_data[1]=8'd2;
  table_data[2]=8'd4;
  table_data[3]=8'd5;
  table_data[4]=8'd7;
  table_data[5]=8'd9;
  table_data[6]=8'd10;
  table_data[7]=8'd12;
  table_data[8]=8'd13;
  table_data[9]=8'd15;
  table_data[10]=8'd16;
  table_data[11]=8'd18;
  table_data[12]=8'd20;
  table_data[13]=8'd21;
  table_data[14]=8'd23;
  table_data[15]=8'd24;
  table_data[16]=8'd26;
  table_data[17]=8'd27;
  table_data[18]=8'd29;
  table_data[19]=8'd30;
  table_data[20]=8'd32;
  table_data[21]=8'd34;
  table_data[22]=8'd35;
  table_data[23]=8'd37;
  table_data[24]=8'd38;
  table_data[25]=8'd40;
  table_data[26]=8'd41;
  table_data[27]=8'd43;
  table_data[28]=8'd44;
  table_data[29]=8'd46;
  table_data[30]=8'd47;
  table_data[31]=8'd49;
  table_data[32]=8'd51;
  table_data[33]=8'd52;
  table_data[34]=8'd54;
  table_data[35]=8'd55;
  table_data[36]=8'd57;
  table_data[37]=8'd58;
  table_data[38]=8'd60;
  table_data[39]=8'd61;
  table_data[40]=8'd63;
  table_data[41]=8'd64;
  table_data[42]=8'd66;
  table_data[43]=8'd67;
  table_data[44]=8'd69;
  table_data[45]=8'd70;
  table_data[46]=8'd72;
  table_data[47]=8'd73;
  table_data[48]=8'd75;
  table_data[49]=8'd76;
  table_data[50]=8'd78;
  table_data[51]=8'd79;
  table_data[52]=8'd81;
  table_data[53]=8'd82;
  table_data[54]=8'd84;
  table_data[55]=8'd85;
  table_data[56]=8'd87;
  table_data[57]=8'd88;
  table_data[58]=8'd90;
  table_data[59]=8'd91;
  table_data[60]=8'd93;
  table_data[61]=8'd94;
  table_data[62]=8'd95;
  table_data[63]=8'd97;
  table_data[64]=8'd98;
  table_data[65]=8'd100;
  table_data[66]=8'd101;
  table_data[67]=8'd103;
  table_data[68]=8'd104;
  table_data[69]=8'd105;
  table_data[70]=8'd107;
  table_data[71]=8'd108;
  table_data[72]=8'd110;
  table_data[73]=8'd111;
  table_data[74]=8'd113;
  table_data[75]=8'd114;
  table_data[76]=8'd115;
  table_data[77]=8'd117;
  table_data[78]=8'd118;
  table_data[79]=8'd120;
  table_data[80]=8'd121;
  table_data[81]=8'd122;
  table_data[82]=8'd124;
  table_data[83]=8'd125;
  table_data[84]=8'd126;
  table_data[85]=8'd128;
  table_data[86]=8'd129;
  table_data[87]=8'd130;
  table_data[88]=8'd132;
  table_data[89]=8'd133;
  table_data[90]=8'd134;
  table_data[91]=8'd136;
  table_data[92]=8'd137;
  table_data[93]=8'd138;
  table_data[94]=8'd140;
  table_data[95]=8'd141;
  table_data[96]=8'd142;
  table_data[97]=8'd144;
  table_data[98]=8'd145;
  table_data[99]=8'd146;
  table_data[100]=8'd147;
  table_data[101]=8'd149;
  table_data[102]=8'd150;
  table_data[103]=8'd151;
  table_data[104]=8'd153;
  table_data[105]=8'd154;
  table_data[106]=8'd155;
  table_data[107]=8'd156;
  table_data[108]=8'd158;
  table_data[109]=8'd159;
  table_data[110]=8'd160;
  table_data[111]=8'd161;
  table_data[112]=8'd162;
  table_data[113]=8'd164;
  table_data[114]=8'd165;
  table_data[115]=8'd166;
  table_data[116]=8'd167;
  table_data[117]=8'd168;
  table_data[118]=8'd170;
  table_data[119]=8'd171;
  table_data[120]=8'd172;
  table_data[121]=8'd173;
  table_data[122]=8'd174;
  table_data[123]=8'd175;
  table_data[124]=8'd176;
  table_data[125]=8'd178;
  table_data[126]=8'd179;
  table_data[127]=8'd180;
  table_data[128]=8'd181;
  table_data[129]=8'd182;
  table_data[130]=8'd183;
  table_data[131]=8'd184;
  table_data[132]=8'd185;
  table_data[133]=8'd186;
  table_data[134]=8'd187;
  table_data[135]=8'd188;
  table_data[136]=8'd189;
  table_data[137]=8'd191;
  table_data[138]=8'd192;
  table_data[139]=8'd193;
  table_data[140]=8'd194;
  table_data[141]=8'd195;
  table_data[142]=8'd196;
  table_data[143]=8'd197;
  table_data[144]=8'd198;
  table_data[145]=8'd199;
  table_data[146]=8'd200;
  table_data[147]=8'd201;
  table_data[148]=8'd202;
  table_data[149]=8'd202;
  table_data[150]=8'd203;
  table_data[151]=8'd204;
  table_data[152]=8'd205;
  table_data[153]=8'd206;
  table_data[154]=8'd207;
  table_data[155]=8'd208;
  table_data[156]=8'd209;
  table_data[157]=8'd210;
  table_data[158]=8'd211;
  table_data[159]=8'd212;
  table_data[160]=8'd212;
  table_data[161]=8'd213;
  table_data[162]=8'd214;
  table_data[163]=8'd215;
  table_data[164]=8'd216;
  table_data[165]=8'd217;
  table_data[166]=8'd218;
  table_data[167]=8'd218;
  table_data[168]=8'd219;
  table_data[169]=8'd220;
  table_data[170]=8'd221;
  table_data[171]=8'd221;
  table_data[172]=8'd222;
  table_data[173]=8'd223;
  table_data[174]=8'd224;
  table_data[175]=8'd225;
  table_data[176]=8'd225;
  table_data[177]=8'd226;
  table_data[178]=8'd227;
  table_data[179]=8'd227;
  table_data[180]=8'd228;
  table_data[181]=8'd229;
  table_data[182]=8'd230;
  table_data[183]=8'd230;
  table_data[184]=8'd231;
  table_data[185]=8'd232;
  table_data[186]=8'd232;
  table_data[187]=8'd233;
  table_data[188]=8'd233;
  table_data[189]=8'd234;
  table_data[190]=8'd235;
  table_data[191]=8'd235;
  table_data[192]=8'd236;
  table_data[193]=8'd236;
  table_data[194]=8'd237;
  table_data[195]=8'd238;
  table_data[196]=8'd238;
  table_data[197]=8'd239;
  table_data[198]=8'd239;
  table_data[199]=8'd240;
  table_data[200]=8'd240;
  table_data[201]=8'd241;
  table_data[202]=8'd241;
  table_data[203]=8'd242;
  table_data[204]=8'd242;
  table_data[205]=8'd243;
  table_data[206]=8'd243;
  table_data[207]=8'd244;
  table_data[208]=8'd244;
  table_data[209]=8'd245;
  table_data[210]=8'd245;
  table_data[211]=8'd246;
  table_data[212]=8'd246;
  table_data[213]=8'd246;
  table_data[214]=8'd247;
  table_data[215]=8'd247;
  table_data[216]=8'd248;
  table_data[217]=8'd248;
  table_data[218]=8'd248;
  table_data[219]=8'd249;
  table_data[220]=8'd249;
  table_data[221]=8'd249;
  table_data[222]=8'd250;
  table_data[223]=8'd250;
  table_data[224]=8'd250;
  table_data[225]=8'd251;
  table_data[226]=8'd251;
  table_data[227]=8'd251;
  table_data[228]=8'd251;
  table_data[229]=8'd252;
  table_data[230]=8'd252;
  table_data[231]=8'd252;
  table_data[232]=8'd252;
  table_data[233]=8'd253;
  table_data[234]=8'd253;
  table_data[235]=8'd253;
  table_data[236]=8'd253;
  table_data[237]=8'd253;
  table_data[238]=8'd254;
  table_data[239]=8'd254;
  table_data[240]=8'd254;
  table_data[241]=8'd254;
  table_data[242]=8'd254;
  table_data[243]=8'd254;
  table_data[244]=8'd254;
  table_data[245]=8'd254;
  table_data[246]=8'd255;
  table_data[247]=8'd255;
  table_data[248]=8'd255;
  table_data[249]=8'd255;
  table_data[250]=8'd255;
  table_data[251]=8'd255;
  table_data[252]=8'd255;
  table_data[253]=8'd255;
  table_data[254]=8'd255;
  table_data[255]=8'd255;
 end
 always @(posedge clk) q_data<=table_data[addr];
`endif
endmodule

