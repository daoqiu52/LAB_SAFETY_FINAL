
module sd_card_bmp #(
    parameter integer CLK_FREQ_HZ       = 100_000_000,
    parameter [31:0]  SCAN_START_SECTOR = 32'd0,
    parameter [31:0]  SCAN_MAX_SECTOR   = 32'd131071,
    parameter [2:0]   SCAN_TARGET_COUNT = 3'd4,
    parameter integer LONG_PRESS_MS      = 1000,   // logical KEY3 brightness hold
    parameter integer AUTO_PLAY_MS       = 3000,   // 自动轮播间隔，默认3秒
    parameter integer PANEL_DIGITS       = 6,
    parameter integer LOAD_TIMEOUT_MS    = 5000,
    parameter integer CHORD_HOLD_MS      = 3000,
    parameter integer KEY_DEBOUNCE_MS    = 20,
    parameter integer KEY3_CLICK_TOGGLES_AUTO = 0, // 0: next; 1: toggle autoplay
    parameter integer SD_READ_TIMEOUT_MS = 500,
    parameter integer REQUIRE_MEMORY_READY = 0,
    parameter integer FOUR_BANK_MODE = 0,
    parameter integer THUMBNAILS = 0,
    parameter integer SCALING_ENABLE = 0,
    parameter integer PRESERVE_ASPECT = 1,
    parameter integer TRANSITIONS = 0,
    parameter integer MODE_SELECT = 0,
    parameter integer MENU_ENABLE = 0,
    parameter integer AUDIO_SEGMENT_SECONDS = 36,
    parameter integer WAV_ENABLE = 0,
    parameter [7:0] DEFAULT_VOLUME = 128
)(
    input                       clk,
    input                       rst,
    input                       key_next,
    input                       key_auto,
    input                       menu_mode,
    output [5:0]                menu_settings,
    output [2:0]                menu_selection,
    output                     menu_confirm,
    output                     key_beep_n,
    input                      rtc_page,
    output                     rtc_launch,rtc_prev,rtc_next,rtc_enter,
    input  [15:0]               bmp_width,
    input  [15:0]               bmp_height,
    output reg                  display_valid,
    output reg [3:0]            state_code,         // 数码管显示（复用端口，内容改为图片编号）

    input                       write_finish_toggle,
    output reg [1:0]            write_buf_idx,
    output reg [1:0]            disp_buf_idx,

    output                      write_req,
    input                       write_req_ack,
    output                      write_en,
    output [31:0]               write_data,
    output                      SD_nCS,
    output                      SD_DCLK,
    output                      SD_MOSI,
    input                       SD_MISO,
    // Panel inputs use clk; connect to the real volume controller when present.
    input [7:0]                 volume_level,
    input                       volume_event,
    output [PANEL_DIGITS-1:0]    panel_dig_n,
    output [7:0]                panel_seg_n,
    output reg [1:0]            brightness_gray,
    output reg                  save_event,
    output reg [31:0]           save_config_data,
    input                       key_play_n, // physical K3/B1, active low
    output wire                 write_recovery_reset,
    output reg [2:0]             last_bmp_error,
    input [5:0]                 write_diagnostics,
    input                       memory_ready,
    input                       frame_bank_toggle,
    input [1:0]                 frame_bank_index,
    input                       frame_bank_valid,
    input                       display_read_fault,
    output wire [127:0]         osd_sizes,
    output wire                 osd_auto_enabled,
    output wire [7:0]           osd_error_code,
    output reg [3:0]            image_complete,
    input                       transition_busy,
    output wire                 transition_manual,
    input                       publicity_mode,audio_stream_end,
    output reg                  audio_clear,
    input                       audio_sector_ready,
    output                      audio_byte_valid,
    output [7:0]                audio_byte,
    output                      audio_finished,
    output [7:0]                audio_error,
    output reg [7:0]            user_volume,
    output reg                  audio_muted,
    input                       volume_restore_valid,
    input [7:0]                 volume_restore,
    output reg                  thumbnail_write,
    output reg                  thumbnail_mode
);

wire raw_prev,raw_next,raw_bright,raw_chord,raw_play,raw_volume;
wire prev_released,next_released,play_released;
reg mute_guard,mute_fired;
localparam integer MUTE_CYCLES=CLK_FREQ_HZ;
reg [$clog2(MUTE_CYCLES+1)-1:0] mute_count;
wire mute_combo=!prev_released && !play_released && next_released && !menu_open;
wire mute_cancel=mute_guard || mute_combo;
always @(posedge clk or posedge rst) begin
 if(rst) begin audio_muted<=0;mute_guard<=0;mute_fired<=0;mute_count<=0;end
 else begin
  if(menu_defaults) audio_muted<=0;
  if(mute_combo) mute_guard<=1;
  if(prev_released && next_released && play_released) begin
   mute_guard<=0;mute_fired<=0;mute_count<=0;
  end else if(mute_combo && !mute_fired) begin
   if(mute_count==MUTE_CYCLES-1) begin audio_muted<=~audio_muted;mute_fired<=1;end
   else mute_count<=mute_count+1'b1;
  end else mute_count<=0;
 end
end
wire menu_adjust,menu_defaults,menu_guard,menu_guard_raw;
wire menu_open = MENU_ENABLE && menu_mode;
assign rtc_launch=menu_adjust && menu_selection==7 && !rtc_page;
assign rtc_prev=menu_open && !menu_guard && raw_prev;
assign rtc_next=menu_open && !menu_guard && raw_next;
assign rtc_enter=menu_open && !menu_guard && raw_play && !raw_prev && !raw_next;
assign menu_guard = MENU_ENABLE && menu_guard_raw;
wire key2_event = raw_prev && !menu_open && !menu_guard;
wire key3_event = raw_next && !menu_open && !menu_guard;
wire key3_long_event = raw_bright && !menu_open && !menu_guard;
wire router_save_event = raw_chord && !menu_open && !menu_guard;
settings_menu_control #(.CLK_FREQ_HZ(CLK_FREQ_HZ)) u_menu_control(
 .clk(clk),.rst(rst),.open(menu_open),.rtc_page(rtc_page),
 .keys_released(key_next && key_auto && key_play_n),
 .prev_event(raw_prev),.next_event(raw_next),.adjust_event(raw_play),
 .other_event(raw_volume || raw_bright || raw_chord),
 .settings(menu_settings),.selection(menu_selection),.confirm(menu_confirm),
 .adjust(menu_adjust),.defaults(menu_defaults),.guard(menu_guard_raw),.beep_n(key_beep_n));
wire publicity = MODE_SELECT && publicity_mode;
reg publicity_q,publicity_start;
reg [10:0] audio_reset_count;
reg [1:0] audio_picture;
wire audio_session_ready=scan_done && first_image_committed && !load_busy &&
                        image_complete==4'hf && !thumbnail_mode && !publicity_start &&
                        !switch_wait && !switching_effect;
// Both FIFO domains stay reset while seeking a new page. Re-arm only after
// the SD reader has drained its current sector and the displayed bank settled.
always @(posedge clk or posedge rst) begin
 if(rst) begin publicity_q<=0;audio_clear<=1;audio_reset_count<=0;audio_picture<=0;end
 else begin
  publicity_q<=publicity;
  if(!MODE_SELECT) begin audio_clear<=0;audio_reset_count<=0;end
  else if(!publicity || !audio_session_ready || img_idx!=audio_picture) begin
   audio_clear<=1;audio_reset_count<=0;audio_picture<=img_idx;
  end else if(audio_clear) begin
   if(audio_reset_count!=1023) audio_reset_count<=audio_reset_count+1'b1;
   else if(bmp_ready) audio_clear<=0;
  end
 end
end
always @(posedge clk or posedge rst)
    if(rst) thumbnail_mode<=0;
    else if(MODE_SELECT && publicity && !publicity_q) thumbnail_mode<=0;
    else if(THUMBNAILS && thumbnail_mode && play_click) thumbnail_mode<=0;
    else if(THUMBNAILS && router_save_event) thumbnail_mode<=~thumbnail_mode;
assign transition_manual = bank_manual;
wire switching_effect = TRANSITIONS && transition_busy;
wire key_next_press = key3_event && !KEY3_CLICK_TOGGLES_AUTO;
wire play_click = raw_play && !menu_open && !menu_guard;
wire volume_click = (raw_volume && !menu_open && !menu_guard) ||
                    (menu_adjust && menu_selection==1);
reg volume_changed;
always @(posedge clk or posedge rst) begin
 if(rst) begin user_volume<=DEFAULT_VOLUME;volume_changed<=0;end
 else if(menu_defaults) begin user_volume<=DEFAULT_VOLUME;volume_changed<=1;end
 else if(WAV_ENABLE && volume_click) begin
 volume_changed<=1;
 case(user_volume)
  0:user_volume<=64;64:user_volume<=128;128:user_volume<=192;
  192:user_volume<=255;default:user_volume<=0;
 endcase
 end else if(volume_restore_valid && !volume_changed) user_volume<=volume_restore;
end
wire key_auto_press = !thumbnail_mode && (play_click || (key3_event && KEY3_CLICK_TOGGLES_AUTO));
reg [1:0] brightness_level;

wire             sd_sec_read;
wire [31:0]      sd_sec_read_addr;
wire [7:0]       sd_sec_read_data;
wire             sd_sec_read_data_valid;
wire             sd_sec_read_end;
wire [3:0]       state_code_i;
wire             bmp_data_wr_en;
wire [23:0]      bmp_data;
wire             sd_init_done;
wire             bmp_ready;
wire             reader_scan_done;
reg              catalog_ready;
wire             scan_done = catalog_ready;
wire [2:0]       bmp_error;
wire             load_error, reader_load_done;
wire [7:0] reader_fs_error;
reg [7:0] last_fs_error;
wire [31:0] file_width,file_height;
reg [31:0] cached_sizes[0:3];
integer size_i;
assign osd_sizes={cached_sizes[3],cached_sizes[2],cached_sizes[1],cached_sizes[0]};
always @(posedge clk or posedge rst) begin
 if(rst) begin for(size_i=0;size_i<4;size_i=size_i+1) cached_sizes[size_i]<=0;end
 else if(load_busy && source_done_seen && write_done_seen && !load_failed)
  cached_sizes[load_idx]<={file_width[15:0],file_height[15:0]};
end
assign osd_auto_enabled=auto_play_en;
assign osd_error_code=last_fs_error!=0 ? last_fs_error : last_bmp_error!=0 ? {5'd0,last_bmp_error} :
                      (WAV_ENABLE ? audio_error : 8'd0);
reg [2:0] bank_ack_sync, read_fault_sync;
reg selection_armed, bank_manual, bank_picture_event;
wire bank_ack_event = bank_ack_sync[2] ^ bank_ack_sync[1];
wire [3:0] preload_missing = ~(image_complete | bad_images);
wire [1:0] preload_index = available_index(2'd3,img_found_count,~preload_missing,1'b0);
wire [1:0] cached_next = available_index(img_idx,img_found_count,~image_complete,1'b0);
wire [1:0] cached_prev = available_index(img_idx,img_found_count,~image_complete,1'b1);
wire switch_wait = selection_armed && (!display_valid || disp_buf_idx!=img_idx);

reg              retry_count, retry_pending, scan_retry_count, scan_failed;
reg [3:0]        bad_images;
reg [5:0]        recovery_count;
reg [31:0]       recovery_wait;
wire recovery_active = recovery_count!=0;
assign write_recovery_reset = recovery_active;
// Watch lack of progress, not the total duration of a healthy multi-sector load.
wire load_progress = (!source_done_seen && sd_sec_read_end) ||
                     reader_load_done || write_finish_pulse;
wire load_complete = source_done_seen && write_done_seen;
wire load_failed = load_busy && (load_error ||
                  (!load_progress && !load_complete && load_timeout_cnt>=LOAD_TIMEOUT_CYCLES-1));
reg [2:0] last_error_stage;
reg [3:0] last_error_detail;
wire [1:0] first_available = available_index(2'd3, img_found_count, bad_images, 1'b0);
wire [1:0] next_available = available_index(img_idx,img_found_count,bad_images,1'b0);
wire [1:0] prev_available = available_index(img_idx,img_found_count,bad_images,1'b1);
wire any_available = img_found_count!=0 && !bad_images[first_available];
wire             scan_found_valid;
wire [31:0]      scan_found_sector;
wire [2:0]       scan_found_total;

reg              scan_start_pulse;
reg              load_start_pulse;
reg [31:0]       load_sector;
reg              scan_kicked;
reg              first_image_committed;
reg              auto_play_en;
reg [31:0]       auto_cnt;
reg [2:0]        img_found_count;
reg [1:0]        img_idx;              // 当前真正显示中的图片编号
reg [1:0]        load_idx;             // 当前正在写入的图片编号
reg [1:0]        pending_buf_idx;      // 当前正在写入的目标缓冲区
reg [31:0]       img_sector0;
reg [31:0]       img_sector1;
reg [31:0]       img_sector2;
reg [31:0]       img_sector3;
reg              next_req_pending;
reg              load_busy;
reg              manual_load;
reg              load_timeout_seen;
reg [31:0]       load_timeout_cnt;

reg              load_abort;

// bmp_ready 先表示“源文件读取/送 FIFO 完成”；真正切显示要等 write_finish_toggle 同步后
reg              source_done_seen;
reg              write_done_seen;

// 同步 mem_clk 域的 write_finish_toggle
reg [2:0]        wrfin_tgl_sync;
reg         prev_req_pending;      // 上一张请求挂起标志

wire             write_finish_pulse;

wire auto_tick;
wire [1:0] next_from_current;

wire thumbnail_sample;
thumbnail_sample_4x u_thumbnail_sample(
 .clk(clk),.rst(rst || load_abort),.start(load_start_pulse),
 .pixel_valid(bmp_data_wr_en),.sample_valid(thumbnail_sample));
assign write_en = (THUMBNAILS && thumbnail_write) ? thumbnail_sample : bmp_data_wr_en;
assign write_data = {bmp_data[23:16], bmp_data[15:8], bmp_data[7:0], 8'b0};
    
wire [31:0] auto_play_cycles = publicity ? (64'd1*CLK_FREQ_HZ*AUDIO_SEGMENT_SECONDS) : (64'd1*CLK_FREQ_HZ*AUTO_PLAY_MS/1000);
assign auto_tick = publicity && audio_error==0 ? (!audio_clear && audio_stream_end) : (auto_cnt == (auto_play_cycles - 1));

assign next_from_current = next_available;
assign write_finish_pulse = wrfin_tgl_sync[2] ^ wrfin_tgl_sync[1];
localparam integer PANEL_SCAN_CYCLES = (64'd8192 * CLK_FREQ_HZ) / 50000000;
localparam integer LOAD_TIMEOUT_CYCLES = (CLK_FREQ_HZ / 1000) * LOAD_TIMEOUT_MS;
// Before the first commit: Sd1 init, Sd2 scan, Sd3 no BMP,
// Sd4 write handshake/header, Sd5 source read, Sd6 SDRAM finish, Sd7 timeout.
wire [3:0] startup_stage = !sd_init_done ? 4'd1 :
                          load_timeout_seen ? 4'd7 :
                          !scan_done ? 4'd2 :
                          (img_found_count == 0) ? 4'd3 :
                          !load_busy ? 4'd4 :
                          source_done_seen ? 4'd6 :
                          (state_code_i == 4'd4) ? 4'd5 : 4'd4;
// Same percentages as the HDMI BRI overlay, in the controller clock domain.
wire [7:0] panel_brightness_percent = brightness_level==0 ? 8'd100 :
                                      brightness_level==1 ? 8'd75 :
                                      brightness_level==2 ? 8'd50 : 8'd25;
wire panel_picture_event = FOUR_BANK_MODE ? bank_picture_event : sd_init_done && !load_failed && load_busy && source_done_seen && write_done_seen && manual_load;

// Keep panel data and events in the SD controller clock domain (100 MHz).
seg7_panel #(
    .DIGITS(PANEL_DIGITS),
    .STARTUP_DIAGNOSTICS(1),
    .ERROR_DISPLAY(1),
    .ERROR_DETAIL(1),
    .WRITE_DIAGNOSTICS(1),
    .FAT_DIAGNOSTICS(1),
    .SCAN_STEP_CYCLES(PANEL_SCAN_CYCLES < 1 ? 1 : PANEL_SCAN_CYCLES),
    .PAGE_PERIOD_CYCLES(CLK_FREQ_HZ),
    .HOLD_CYCLES(CLK_FREQ_HZ * 2)
) u_seg7_panel (
    .clk(clk), .rst(rst),
    .picture_valid(display_valid),
    .picture_number(display_valid ? ({2'b00, img_idx} + 4'd1) : startup_stage),
    .volume_level(WAV_ENABLE ? user_volume : volume_level), .brightness_percent(panel_brightness_percent),
    .picture_event(panel_picture_event), .volume_event(volume_event || (WAV_ENABLE && volume_click)),
    .error_code(last_bmp_error),
    .error_stage(last_error_stage),
    .error_detail(last_error_detail),
    .fs_error(last_fs_error),
    .dig_n(panel_dig_n), .seg_n(panel_seg_n)
);
// state_code 由下方 always 块驱动（显示当前图片编号），不能再 assign 到 state_code_i，否则双重驱动

// HX4S20C mapping: top key1/B2 is K2 (previous); key2/C1 is K4 (next).
// Reuse release-click/long-hold suppression with the other input released.
dual_key_chord_router #(
    .CLK_FREQ_HZ(CLK_FREQ_HZ), .DEBOUNCE_MS(KEY_DEBOUNCE_MS),
    .LONG_PRESS_MS(LONG_PRESS_MS), .CHORD_HOLD_MS(CHORD_HOLD_MS)
) u_play_key (
    .clk(clk), .rst(rst), .cancel(mute_cancel), .key2_released(), .key3_released(play_released),
    .key2_n(1'b1), .key3_n(menu_guard ? 1'b1:key_play_n),
    .key2_event(), .key3_event(raw_play), .key3_long_event(raw_volume), .save_event()
);
dual_key_chord_router #(
    .CLK_FREQ_HZ(CLK_FREQ_HZ), .DEBOUNCE_MS(KEY_DEBOUNCE_MS),
    .LONG_PRESS_MS(LONG_PRESS_MS), .CHORD_HOLD_MS(CHORD_HOLD_MS)
) u_key_router (
    .clk(clk), .rst(rst), .cancel(mute_cancel), .key2_released(prev_released), .key3_released(next_released),
    .key2_n(menu_guard ? 1'b1:key_next), .key3_n(menu_guard ? 1'b1:key_auto),
    .key2_event(raw_prev), .key3_event(raw_next),
    .key3_long_event(raw_bright), .save_event(raw_chord)
);

// Normal steps follow Gray order. Menu reset may jump; the top-level menu
// mailbox coherently transfers brightness before the protected video stage.
// Configuration snapshot: [31:16] period_ms, [15:3] reserved,
// [2] autoplay enabled, [1:0] brightness level (0=100%,1=75%,2=50%,3=25%).
localparam [15:0] SAVED_PERIOD_MS = AUTO_PLAY_MS;
always @(posedge clk or posedge rst) begin
    if(rst) begin
        brightness_level <= 0;
        brightness_gray <= 0;
        save_event <= 0;
        save_config_data <= 0;
    end else begin
        save_event <= router_save_event;
        if(menu_defaults) begin
            brightness_level <= 0;
            brightness_gray <= 0;
        end else if(key3_long_event || (menu_adjust && menu_selection==0)) begin
            brightness_level <= brightness_level + 2'd1;
            case(brightness_level)
                0: brightness_gray <= 2'b01;
                1: brightness_gray <= 2'b11;
                2: brightness_gray <= 2'b10;
                3: brightness_gray <= 2'b00;
            endcase
        end
        if(router_save_event)
            save_config_data <= {SAVED_PERIOD_MS, 13'd0, auto_play_en, brightness_level};
    end
end

function [1:0] available_index;
    input [1:0] cur;
    input [2:0] count;
    input [3:0] bad;
    input reverse;
    reg [1:0] candidate;
    reg found;
    integer n;
    begin
        candidate=cur; found=0; available_index=0;
        for(n=0;n<4;n=n+1) begin
            candidate = reverse ? prev_index_limited(candidate,count) : next_index_limited(candidate,count);
            if(!found && {1'b0,candidate}<count && !bad[candidate]) begin
                available_index=candidate; found=1;
            end
        end
    end
endfunction

function [1:0] next_index_limited;//计算下一张图片编号
    input [1:0] cur;
    input [2:0] count;
    begin
        case (count)
            3'd0: next_index_limited = 2'd0;
            3'd1: next_index_limited = 2'd0;
            3'd2: next_index_limited = (cur == 2'd1) ? 2'd0 : (cur + 2'd1);
            3'd3: next_index_limited = (cur == 2'd2) ? 2'd0 : (cur + 2'd1);
            default: next_index_limited = (cur == 2'd3) ? 2'd0 : (cur + 2'd1);
        endcase
    end
endfunction

function [31:0] sector_lut;
    input [1:0] idx;
    begin
        case (idx)
            2'd0: sector_lut = img_sector0;
            2'd1: sector_lut = img_sector1;
            2'd2: sector_lut = img_sector2;
            2'd3: sector_lut = img_sector3;
            default: sector_lut = img_sector0;
        endcase
    end
endfunction

function [1:0] next_buf_lut;
    input [1:0] cur_disp_buf;
    input       valid_now;
    begin
        if (!valid_now)
            next_buf_lut = 2'd0;                 // 首图固定写 buffer0
        else if (cur_disp_buf == 2'd0)
            next_buf_lut = 2'd1;
        else
            next_buf_lut = 2'd0;
    end
endfunction

// === 新增：计算上一张图片编号（和next_index_limited对称，减1而不是加1）===
function [1:0] prev_index_limited;
        input [1:0] cur;      // 当前图片编号
        input [2:0] count;    // 已找到的图片总数
        begin
            case(count)
                3'd0: prev_index_limited = 2'd0;
                3'd1: prev_index_limited = 2'd0;
                3'd2: prev_index_limited = (cur == 2'd0) ? 2'd1 : (cur - 2'd1);
                3'd3: prev_index_limited = (cur == 2'd0) ? 2'd2 : (cur - 2'd1);
                default: prev_index_limited = (cur == 2'd0) ? 2'd3 : (cur - 2'd1);
            endcase
        end
 endfunction

always @(posedge clk or posedge rst) begin
    if (rst) begin
        wrfin_tgl_sync        <= 3'b000;
        thumbnail_write      <= 0;
        image_complete<=0;bank_ack_sync<=0;read_fault_sync<=0;selection_armed<=0;bank_manual<=0;bank_picture_event<=0;
        scan_start_pulse      <= 1'b0;
        load_start_pulse      <= 1'b0;
        load_sector           <= 32'd0;
        scan_kicked           <= 1'b0;
        first_image_committed <= 1'b0;
        auto_play_en          <= 1'b0;
        publicity_start<=0;
        auto_cnt              <= 32'd0;
        img_found_count       <= 3'd0;
        img_idx               <= 2'd0;
        load_idx              <= 2'd0;
        pending_buf_idx       <= 2'd0;
        write_buf_idx         <= 2'd0;
        disp_buf_idx          <= 2'd0;
        img_sector0           <= 32'd0;
        img_sector1           <= 32'd0;
        img_sector2           <= 32'd0;
        img_sector3           <= 32'd0;
        next_req_pending      <= 1'b0;
        load_busy             <= 1'b0;
        manual_load           <= 1'b0;
        load_timeout_seen     <= 1'b0;
        state_code            <= 4'd0;
        load_timeout_cnt      <= 32'd0;
        load_abort            <= 1'b0;
        source_done_seen      <= 1'b0;
        write_done_seen       <= 1'b0;
        display_valid         <= 1'b0;

        prev_req_pending      <= 1'b0;
        catalog_ready<=0; retry_count<=0; retry_pending<=0;
        scan_retry_count<=0; scan_failed<=0; bad_images<=0;
        recovery_count<=0; recovery_wait<=0; last_bmp_error<=0; last_error_stage<=0; last_error_detail<=0;last_fs_error<=0;
    end else begin
        wrfin_tgl_sync   <= {wrfin_tgl_sync[1:0], write_finish_toggle};
        bank_ack_sync<={bank_ack_sync[1:0],frame_bank_toggle};
        read_fault_sync<={read_fault_sync[1:0],display_read_fault};
        bank_picture_event<=0;
        // The pixel-domain reader reports the bank actually latched for this frame.
        if(FOUR_BANK_MODE && bank_ack_event && frame_bank_valid &&
           image_complete[frame_bank_index] && frame_bank_index==disp_buf_idx) begin
            img_idx<=frame_bank_index;display_valid<=1;first_image_committed<=1;
            bank_picture_event<=bank_manual && (!display_valid || img_idx!=frame_bank_index);
            bank_manual<=0;
        end
        scan_start_pulse <= 1'b0;
        load_start_pulse <= 1'b0;
        load_abort       <= 1'b0;

        if (recovery_count!=0) recovery_count<=recovery_count-1'b1;
        if (reader_scan_done) catalog_ready<=1;

        // Highest priority: errors can never coincide with a frame commit.
        if (load_failed || (load_error && !load_busy)) begin
            last_bmp_error <= load_error ? bmp_error : 3'd2;
            last_fs_error <= load_error ? reader_fs_error : 8'd0;
            // Snapshot before recovery clears write-side state. Explicit cases
            // also leave omitted diagnostic inputs in old testbenches at zero.
            last_error_detail<=0;
            if(source_done_seen) begin
                case(1'b1)
                    write_diagnostics[0]: last_error_detail<=1; // write while full
                    write_diagnostics[1]: last_error_detail<=2; // write during FIFO reset
                    write_diagnostics[2]: last_error_detail<=3; // read while empty
                    write_diagnostics[5]: last_error_detail<=4; // finish existed in memory domain
                    write_diagnostics[4]: last_error_detail<=5; // all words consumed, no finish
                    write_diagnostics[3]: last_error_detail<=6; // words remain queued
                    !write_diagnostics[3]: last_error_detail<=7; // empty before full frame consumed
                    default: last_error_detail<=0;
                endcase
            end
            last_error_stage <= source_done_seen ? 3'd4 :
                                state_code_i==4'd4 ? 3'd3 :
                                state_code_i==4'd3 ? 3'd2 : 3'd1;
            load_timeout_seen<=!load_error || bmp_error==2;
            load_busy<=0; load_abort<=1; load_start_pulse<=0;
            source_done_seen<=0; write_done_seen<=0; load_timeout_cnt<=0;
            recovery_count<=32; recovery_wait<=0; auto_cnt<=0;
            if(load_busy) begin
                if(!retry_count) begin retry_count<=1; retry_pending<=1; end
                else begin
                    retry_pending<=0; retry_count<=0; bad_images[load_idx]<=1;
                end
            end else if(!catalog_ready) begin
                img_found_count<=0; scan_kicked<=0;
                if(!scan_retry_count) scan_retry_count<=1;
                else scan_failed<=1;
            end
        end else if(recovery_active) begin
            source_done_seen<=0; write_done_seen<=0;
        end else if(retry_pending) begin
            // Reinitialize the SD protocol and write-side FIFO before retrying.
            if(sd_init_done && bmp_ready) begin
                retry_pending<=0; load_start_pulse<=1; load_busy<=1;
                source_done_seen<=0; write_done_seen<=0; load_timeout_cnt<=0;
                recovery_wait<=0;
            end else if(recovery_wait>=LOAD_TIMEOUT_CYCLES-1) begin
                retry_pending<=0; retry_count<=0; bad_images[load_idx]<=1;
                last_bmp_error<=2;
                last_error_stage<=5;
                last_error_detail<=0;
                last_fs_error<=0;
            end else recovery_wait<=recovery_wait+1'b1;
        end else if (!sd_init_done) begin
            // Preserve the front buffer/catalog while the card reinitializes.
            // In-flight failures above have already cancelled the back buffer.
            auto_cnt<=0;
        end else begin
            if (scan_found_valid) begin
                case (img_found_count)
                    0: img_sector0<=scan_found_sector;
                    1: img_sector1<=scan_found_sector;
                    2: img_sector2<=scan_found_sector;
                    3: img_sector3<=scan_found_sector;
                endcase
                if(img_found_count<4) img_found_count<=img_found_count+1'b1;
            end
            if(load_busy && reader_load_done) source_done_seen<=1;
            if(load_busy && write_finish_pulse) write_done_seen<=1;
            if(load_busy && source_done_seen && write_done_seen) begin
                load_busy<=0; load_timeout_cnt<=0;
                source_done_seen<=0; write_done_seen<=0;
                if(THUMBNAILS && FOUR_BANK_MODE && !thumbnail_write) begin
                    // Re-read the same file, sampling only the thumbnail pixels.
                    thumbnail_write<=1; load_start_pulse<=1; load_busy<=1;
                end else if(FOUR_BANK_MODE) begin
                    image_complete[load_idx]<=1;
                    if(!selection_armed) begin disp_buf_idx<=load_idx;selection_armed<=1;end
                end else begin
                    disp_buf_idx<=pending_buf_idx; img_idx<=load_idx;
                    display_valid<=1; first_image_committed<=1;
                end
                load_timeout_seen<=0; last_bmp_error<=0; retry_count<=0; last_error_stage<=0; last_error_detail<=0;last_fs_error<=0;
            end else if(load_busy && load_progress) load_timeout_cnt<=0;
            else if(load_busy) load_timeout_cnt<=load_timeout_cnt+1'b1;
            else load_timeout_cnt<=0;

            state_code <= display_valid ? ({2'b0,img_idx}+4'd1) : 4'd0;

            if (!scan_kicked && !scan_failed && !catalog_ready && bmp_ready &&
                (!REQUIRE_MEMORY_READY || memory_ready)) begin
                scan_start_pulse<=1; scan_kicked<=1; img_found_count<=0;
            end else if (!load_abort) begin
                // 忙的时候也只记 1 次“下一张”请求，不会累积成连跳两张
                if (key_next_press && !switching_effect && scan_done && (img_found_count > 3'd0))
                    next_req_pending <= 1'b1;
                if (key2_event && !switching_effect && scan_done && (img_found_count > 3'd0))
                    prev_req_pending <= 1'b1;

                // 自动播放开/关
                if (key_auto_press && scan_done && (img_found_count > 3'd1)) begin
                    auto_play_en <= ~auto_play_en;
                    auto_cnt     <= 32'd0;
                end

                // 自动播放 1s 计数：只有当前没有写图任务时才计时
                if (!thumbnail_mode && !key_auto_press && scan_done && auto_play_en && display_valid && (FOUR_BANK_MODE || !load_busy) && first_image_committed && (img_found_count > 3'd1)) begin
                    if (auto_tick)
                        auto_cnt <= 32'd0;
                    else
                        auto_cnt <= auto_cnt + 32'd1;
                end else begin
                    auto_cnt <= 32'd0;
                end

                // 首图自动加载到 buffer0
                if(FOUR_BANK_MODE) begin
                    // Load every available file once into its permanent image bank.
                    if(scan_done && bmp_ready && !load_busy &&
                       {1'b0,preload_index}<img_found_count && preload_missing[preload_index]) begin
                        thumbnail_write<=0;
                        retry_count<=0;manual_load<=0;load_idx<=preload_index;
                        load_sector<=sector_lut(preload_index);
                        pending_buf_idx<=preload_index;write_buf_idx<=preload_index;
                        load_start_pulse<=1;load_busy<=1;source_done_seen<=0;write_done_seen<=0;
                    end
                    // Cached selections never initiate an SD read or SDRAM write.
                    if(display_valid && !switch_wait && !switching_effect && |image_complete) begin
                        if(publicity_start && image_complete[0]) begin
                            disp_buf_idx<=0;bank_manual<=0;auto_cnt<=0;
                            next_req_pending<=0;prev_req_pending<=0;publicity_start<=0;
                        end else if(next_req_pending) begin
                            disp_buf_idx<=cached_next;next_req_pending<=0;
                            bank_manual<=cached_next!=img_idx;auto_cnt<=0;
                        end else if(prev_req_pending) begin
                            disp_buf_idx<=cached_prev;prev_req_pending<=0;
                            bank_manual<=cached_prev!=img_idx;auto_cnt<=0;
                        end else if(!thumbnail_mode && !key_auto_press && auto_play_en && auto_tick) begin
                            disp_buf_idx<=cached_next;bank_manual<=0;auto_cnt<=0;
                        end
                    end
                end else begin
                if (scan_done && !first_image_committed && bmp_ready && !load_busy && any_available) begin
                    retry_count <= 0;
                    manual_load      <= 1'b0;
                    load_idx         <= first_available;
                    load_sector      <= sector_lut(first_available);
                    pending_buf_idx  <= 2'd0;
                    write_buf_idx    <= 2'd0;
                    load_start_pulse <= 1'b1;
                    load_busy        <= 1'b1;
                    source_done_seen <= 1'b0;
                    write_done_seen  <= 1'b0;
                    next_req_pending <= 1'b0;
                    auto_cnt         <= 32'd0;
                end
                // 手动下一张优先：写到“非当前显示”的另一块 buffer
                else if (scan_done && bmp_ready && display_valid && !load_busy && next_req_pending && any_available) begin
                    retry_count <= 0;
                    manual_load      <= 1'b1;
                    load_idx         <= next_from_current;
                    load_sector      <= sector_lut(next_from_current);
                    pending_buf_idx  <= next_buf_lut(disp_buf_idx, display_valid);
                    write_buf_idx    <= next_buf_lut(disp_buf_idx, display_valid);
                    load_start_pulse <= 1'b1;
                    load_busy        <= 1'b1;
                    source_done_seen <= 1'b0;
                    write_done_seen  <= 1'b0;
                    next_req_pending <= 1'b0;
                    auto_cnt         <= 32'd0;
                end

                // === 新增：上一张请求处理（跟下一张完全对称，下一张优先）===
                else if (scan_done && bmp_ready && display_valid && !load_busy && prev_req_pending && any_available) begin
                    retry_count <= 0;
                    manual_load <= 1'b1;
                    source_done_seen <= 1'b0;
                    write_done_seen <= 1'b0;
                    // 上一张：只有下一张没有请求时才处理（下一张优先）
                     load_idx <= prev_available;
                     load_sector <= sector_lut(prev_available);
                     pending_buf_idx <= next_buf_lut(disp_buf_idx, display_valid);
                     write_buf_idx <= next_buf_lut(disp_buf_idx, display_valid);
                     load_start_pulse <= 1'b1;
                     load_busy <= 1'b1;
                     prev_req_pending <= 1'b0;
                     auto_cnt <= 32'd0;
                end

                // 自动播放下一张：同样写到“非当前显示”的另一块 buffer
                else if (!key_auto_press && scan_done && bmp_ready && display_valid && !load_busy && auto_play_en && auto_tick && any_available && next_available!=img_idx && (img_found_count > 3'd1)) begin
                    retry_count <= 0;
                    manual_load      <= 1'b0;
                    load_idx         <= next_from_current;
                    load_sector      <= sector_lut(next_from_current);
                    pending_buf_idx  <= next_buf_lut(disp_buf_idx, display_valid);
                    write_buf_idx    <= next_buf_lut(disp_buf_idx, display_valid);
                    load_start_pulse <= 1'b1;
                    load_busy        <= 1'b1;
                    source_done_seen <= 1'b0;
                    write_done_seen  <= 1'b0;
                    auto_cnt         <= 32'd0;
                end
                end // legacy double-buffer selection
            end
        end
        if(MODE_SELECT && publicity!=publicity_q) begin
            auto_play_en<=publicity;auto_cnt<=0;publicity_start<=publicity;
            next_req_pending<=0;prev_req_pending<=0;
        end
        // Passive reader diagnostic: never reset SDRAM or invalidate cached banks.
        if(FOUR_BANK_MODE && read_fault_sync[2:1]==2'b01) begin
            last_bmp_error<=3'd4;last_error_stage<=3'd6;last_error_detail<=0;last_fs_error<=0;
        end
    end
end

fat32_bmp_reader #(.CLK_FREQ_HZ(CLK_FREQ_HZ), .SD_READ_TIMEOUT_MS(SD_READ_TIMEOUT_MS),.AUDIO_ENABLE(WAV_ENABLE),.AUDIO_SEGMENTS(MODE_SELECT),.AUDIO_SEGMENT_BYTES(AUDIO_SEGMENT_SECONDS*96000),.SCALING_ENABLE(SCALING_ENABLE),.PRESERVE_ASPECT(PRESERVE_ASPECT)) bmp_read_m0(
    .audio_enable(MODE_SELECT ? (publicity && !audio_clear) : (scan_done && first_image_committed && !load_busy && preload_missing==0)),
    .audio_index(audio_picture),
    .audio_sector_ready(audio_sector_ready),.audio_byte_valid(audio_byte_valid),.audio_byte(audio_byte),
    .audio_finished(audio_finished),.audio_error(audio_error),
    .clk                    (clk),
    .rst                    (rst),
    .ready                  (bmp_ready),
    .bmp_error(bmp_error), .load_error(load_error), .load_done(reader_load_done),
    .fs_error(reader_fs_error),
    .file_width(file_width),.file_height(file_height),

    .scan_start             (scan_start_pulse),
    .scan_start_sector      (SCAN_START_SECTOR),
    .scan_max_sector        (SCAN_MAX_SECTOR),
    .scan_target_count      (SCAN_TARGET_COUNT),
    .scan_done              (reader_scan_done),
    .scan_found_valid       (scan_found_valid),
    .scan_found_sector      (scan_found_sector),
    .scan_found_total       (scan_found_total),

    .load_start             (load_start_pulse),
    .load_abort             (load_abort),
    .load_sector            (load_sector),

    .sd_init_done           (sd_init_done),
    .state_code             (state_code_i),
    .bmp_width              (bmp_width),
    .bmp_height             (bmp_height),
    .write_req              (write_req),
    .write_req_ack          (write_req_ack),
    .sd_sec_read            (sd_sec_read),
    .sd_sec_read_addr       (sd_sec_read_addr),
    .sd_sec_read_data       (sd_sec_read_data),
    .sd_sec_read_data_valid (sd_sec_read_data_valid),
    .sd_sec_read_end        (sd_sec_read_end),
    .bmp_data_wr_en         (bmp_data_wr_en),
    .bmp_data               (bmp_data)
);

sd_card_top sd_card_top_m0(
    .clk                    (clk),
    .rst                    (rst || recovery_active),
    .SD_nCS                 (SD_nCS),
    .SD_DCLK                (SD_DCLK),
    .SD_MOSI                (SD_MOSI),
    .SD_MISO                (SD_MISO),
    .sd_init_done           (sd_init_done),
    .sd_sec_read            (sd_sec_read),
    .sd_sec_read_addr       (sd_sec_read_addr),
    .sd_sec_read_data       (sd_sec_read_data),
    .sd_sec_read_data_valid (sd_sec_read_data_valid),
    .sd_sec_read_end        (sd_sec_read_end),
    .sd_sec_write           (1'b0),
    .sd_sec_write_addr      (32'd0),
    .sd_sec_write_data      (),
    .sd_sec_write_data_req  (),
    .sd_sec_write_end       ()
);

endmodule

// BMP stream is bottom-up: stream row 3 corresponds to display row 476.
// Sampling rows 3,7,...479 gives display rows 476,472,...0, exactly y*4.
module thumbnail_sample_4x(
 input clk,rst,start,pixel_valid,output sample_valid);
 reg [9:0] x;
 reg [1:0] row_phase;
 assign sample_valid=pixel_valid && x[1:0]==0 && row_phase==3;
 always @(posedge clk or posedge rst) begin
  if(rst) begin x<=0;row_phase<=0;end
  else if(start) begin x<=0;row_phase<=0;end
  else if(pixel_valid) begin
   if(x==639) begin x<=0;row_phase<=row_phase+1'b1;end
   else x<=x+1'b1;
  end
 end
endmodule

module key_press_debounce #(
    parameter integer CLK_FREQ_HZ = 100_000_000,
    parameter integer DEBOUNCE_MS = 20
)(
    input  wire clk,
    input  wire rst,
    input  wire button_in,    // 默认：松开=1，按下=0
    output reg  press_pulse,
// === 新增：消抖后的稳定状态输出（用于长按检测）===
    output reg  button_stable_out
);

localparam integer DEBOUNCE_CYCLES = (CLK_FREQ_HZ / 1000) * DEBOUNCE_MS;

reg button_sync0;
reg button_sync1;
reg button_stable;
reg [31:0] cnt;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        button_sync0  <= 1'b1;
        button_sync1  <= 1'b1;
        button_stable <= 1'b1;
        cnt           <= 32'd0;
        press_pulse   <= 1'b0;
        button_stable_out <= 1'b1;
    end else begin
        button_sync0 <= button_in;
        button_sync1 <= button_sync0;
        press_pulse  <= 1'b0;

        if (button_sync1 == button_stable) begin
            cnt <= 32'd0;
        end else begin
            if (cnt >= DEBOUNCE_CYCLES - 1) begin
                if (button_stable && !button_sync1)
                    press_pulse <= 1'b1;
                button_stable <= button_sync1;
                button_stable_out <= button_sync1;  // 新增：同步输出
                cnt           <= 32'd0;
            end else begin
                cnt <= cnt + 32'd1;
            end
        end
    end
end

endmodule



// Raw active-low buttons. Clicks are reported on debounced release so a
// chord/long hold can consume the gesture without an earlier click.
module dual_key_chord_router #(
    parameter integer CLK_FREQ_HZ = 100_000_000,
    parameter integer DEBOUNCE_MS = 20,
    parameter integer LONG_PRESS_MS = 1000,
    parameter integer CHORD_HOLD_MS = 3000
)(
    input wire clk, rst,
    input wire key2_n, key3_n,
    input wire cancel,
    output wire key2_released,key3_released,
    output reg key2_event,
    output reg key3_event,
    output reg key3_long_event,
    output reg save_event
);
localparam integer LONG_CYCLES_RAW = (CLK_FREQ_HZ / 1000) * LONG_PRESS_MS;
localparam integer CHORD_CYCLES_RAW = (CLK_FREQ_HZ / 1000) * CHORD_HOLD_MS;
localparam integer LONG_CYCLES = LONG_CYCLES_RAW < 1 ? 1 : LONG_CYCLES_RAW;
localparam integer CHORD_CYCLES = CHORD_CYCLES_RAW < 1 ? 1 : CHORD_CYCLES_RAW;
localparam [2:0] IDLE=0, KEY2=1, KEY3=2, CHORD=3, WAIT_RELEASE=4;
wire key2_stable, key3_stable;
assign key2_released=key2_stable;
assign key3_released=key3_stable;
reg [2:0] state;
reg [31:0] hold_count;
reg long_consumed;
key_press_debounce #(.CLK_FREQ_HZ(CLK_FREQ_HZ), .DEBOUNCE_MS(DEBOUNCE_MS)) u_key2 (
    .clk(clk), .rst(rst), .button_in(key2_n), .press_pulse(),
    .button_stable_out(key2_stable)
);
key_press_debounce #(.CLK_FREQ_HZ(CLK_FREQ_HZ), .DEBOUNCE_MS(DEBOUNCE_MS)) u_key3 (
    .clk(clk), .rst(rst), .button_in(key3_n), .press_pulse(),
    .button_stable_out(key3_stable)
);
always @(posedge clk or posedge rst) begin
    if(rst) begin
        state <= IDLE; hold_count <= 0; long_consumed <= 0;
        key2_event <= 0; key3_event <= 0;
        key3_long_event <= 0; save_event <= 0;
    end else begin
        key2_event <= 0; key3_event <= 0;
        key3_long_event <= 0; save_event <= 0;
        if(cancel) begin state<=WAIT_RELEASE;hold_count<=0;long_consumed<=1;end
        else case(state)
            IDLE: begin
                hold_count <= 0; long_consumed <= 0;
                if(!key2_stable && !key3_stable) state <= CHORD;
                else if(!key2_stable) state <= KEY2;
                else if(!key3_stable) state <= KEY3;
            end
            KEY2: begin
                if(!key3_stable) begin
                    hold_count <= 0;
                    state <= !key2_stable ? CHORD : WAIT_RELEASE;
                end else if(key2_stable) begin
                    key2_event <= !long_consumed;
                    state <= IDLE;
                end else if(!long_consumed) begin
                    if(hold_count >= LONG_CYCLES-1) long_consumed <= 1;
                    else hold_count <= hold_count + 1'b1;
                end
            end
            KEY3: begin
                // A second button suppresses the click/brightness gesture.
                if(!key2_stable) begin
                    hold_count <= 0;
                    state <= (!key3_stable && !long_consumed) ? CHORD : WAIT_RELEASE;
                end else if(key3_stable) begin
                    key3_event <= !long_consumed;
                    state <= IDLE;
                end else if(!long_consumed) begin
                    if(hold_count >= LONG_CYCLES-1) begin
                        key3_long_event <= 1;
                        long_consumed <= 1;
                    end else hold_count <= hold_count + 1'b1;
                end
            end
            CHORD: begin
                if(key2_stable || key3_stable) begin
                    hold_count <= 0;
                    state <= WAIT_RELEASE;
                end else if(hold_count >= CHORD_CYCLES-1) begin
                    save_event <= 1;
                    state <= WAIT_RELEASE;
                end else hold_count <= hold_count + 1'b1;
            end
            WAIT_RELEASE: begin
                hold_count <= 0;
                if(key2_stable && key3_stable) state <= IDLE;
            end
            default: state <= IDLE;
        endcase
    end
end
endmodule

// SW2 owns key routing. A 30 ms all-released guard prevents synthetic clicks
// at entry/exit. All counters use the existing SD clock, never a fabric clock.
module settings_menu_control #(parameter integer CLK_FREQ_HZ=100000000)(
 input clk,rst,open,keys_released,rtc_page,
 input prev_event,next_event,adjust_event,other_event,
 output [5:0] settings,output reg [2:0] selection,output reg confirm,
 output adjust,defaults,output guard,output beep_n);
 localparam integer QUARTER_MS=CLK_FREQ_HZ>=4000 ? CLK_FREQ_HZ/4000:1;
 localparam integer DIV_BITS=QUARTER_MS>1 ? $clog2(QUARTER_MS):1;
 reg [DIV_BITS-1:0] divider;
 wire tick=divider==QUARTER_MS-1;
 reg open_q,guarding;
 (* async_reg="true" *) reg [1:0] released_sync;
 assign guard=guarding || open!=open_q;
 reg [6:0] release_ticks;
 reg subtitle_on,waveform_on,beep_on;
 reg [2:0] tone;
 reg [7:0] beep_ticks;
 reg beep_phase;
 assign settings={beep_on,tone,waveform_on,subtitle_on};
 assign adjust=open && !rtc_page && !guard && open==open_q && adjust_event && !prev_event && !next_event;
 assign defaults=adjust && selection==6 && confirm;
 assign beep_n=!(beep_on && beep_ticks!=0 && beep_phase);
 always @(posedge clk or posedge rst) begin
  if(rst) begin
   divider<=0;open_q<=0;release_ticks<=0;guarding<=1;released_sync<=0;
   subtitle_on<=1;waveform_on<=1;beep_on<=0;tone<=0;
   selection<=0;confirm<=0;beep_ticks<=0;beep_phase<=0;
  end else begin
   if(tick) divider<=0;else divider<=divider+1'b1;
   open_q<=open;released_sync<={released_sync[0],keys_released};
   if(open!=open_q) begin
    guarding<=1;release_ticks<=0;confirm<=0;selection<=0;
   end else if(guard) begin
    if(!released_sync[1]) release_ticks<=0;
    else if(tick) begin
     if(release_ticks==119) begin guarding<=0;release_ticks<=0;end
     else release_ticks<=release_ticks+1'b1;
    end
   end
   if(open && !rtc_page && !guard && open==open_q) begin
    if(prev_event || next_event) begin
     confirm<=0;
     if(prev_event) selection<=selection==0 ? 3'd7:selection-1'b1;
     else selection<=selection==7 ? 3'd0:selection+1'b1;
    end else if(adjust) begin
     case(selection)
      2:subtitle_on<=~subtitle_on;
      3:waveform_on<=~waveform_on;
      4:tone<=tone==4 ? 3'd0:tone+1'b1;
      5:beep_on<=~beep_on;
      6:begin
       confirm<=~confirm;
       if(confirm) begin subtitle_on<=1;waveform_on<=1;tone<=0;beep_on<=0;end
      end
      default:confirm<=0;
     endcase
    end
   end
   if(!beep_on && !(adjust && selection==5)) begin beep_ticks<=0;beep_phase<=0;end
   else if(!guard && open==open_q &&
           (prev_event || next_event || adjust_event || (!open && other_event))) begin
    beep_ticks<=160;beep_phase<=0;
   end else if(tick) begin
    if(beep_ticks!=0) begin beep_ticks<=beep_ticks-1'b1;beep_phase<=~beep_phase;end
    else beep_phase<=0;
   end
  end
 end
endmodule
