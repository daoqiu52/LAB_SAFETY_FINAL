# HX4S20C V1.0 / EG4S20BG256, Lesson 5.
# Audio is generated in video_clk; there is no independent audio PLL.
create_clock -name clk -period 20 -waveform {0 10} [get_ports {clk}]
derive_clocks
rename_clock -name {sd_card_clk} -source [get_ports {clk}] -master_clock {clk} [get_pins {sys_pll_m0/pll_inst.clkc[0]}]
rename_clock -name {ext_mem_clk} -source [get_ports {clk}] -master_clock {clk} [get_pins {sys_pll_m0/pll_inst.clkc[1]}]
rename_clock -name {ext_mem_clk_sft} -source [get_ports {clk}] -master_clock {clk} [get_pins {sys_pll_m0/pll_inst.clkc[2]}]
rename_clock -name {video_clk} -source [get_ports {clk}] -master_clock {clk} [get_pins {video_pll_m0/pll_inst.clkc[0]}]
rename_clock -name {hdmi_5x_clk} -source [get_ports {clk}] -master_clock {clk} [get_pins {video_pll_m0/pll_inst.clkc[1]}]

# Button assertion is asynchronous. Do NOT cut synchronized reset release trees.
set_false_path -from [get_ports {rst_n}]
# Cut only first sampling registers of these single-bit level synchronizers.
# No exception on their second stage. FIFO Gray max-delay comes from IP TCLs.
foreach pattern {u_memory_ready_sd/s1 u_display_valid/s1} {
    set first_stage [get_regs -nowarn $pattern]
    if {$first_stage != ""} { set_false_path -to $first_stage }
}
set_clock_uncertainty -setup 0.100 [get_clocks {clk sd_card_clk ext_mem_clk ext_mem_clk_sft video_clk hdmi_5x_clk}]
set_clock_uncertainty -hold 0.050 [get_clocks {clk sd_card_clk ext_mem_clk ext_mem_clk_sft video_clk hdmi_5x_clk}]
set_max_delay -from [get_clocks {hdmi_5x_clk}] -to [get_ports {HDMI_CLK_P HDMI_D0_P HDMI_D1_P HDMI_D2_P}] 8.000 -datapath_only

# The pixel/serial clocks BOTH run and exchange data in lane_lvds_10_1.
# They are NOT exclusive. Keep their related-clock timing checks enabled.
# No blanket asynchronous clock groups: multi-bit payloads must remain checked.
# Do NOT false-path SDRAM shifted clocks without the vendor interface contract.
# External SD IO delays require the board/card electrical timing budget; do not
# invent values. Their absence is documented in FLASH_BOOT_LESSON5.md.

# WAV FIFO: registered Gray pointers must arrive within one SOURCE clock.
# Stable-picture diagnostic counter crosses as registered Gray code at 50 MHz.
set_max_delay -from [get_regs -nowarn {u_stable_integrity/mem_gray*}] -to [get_regs -nowarn {u_stable_integrity/gray_meta*}] 20.000 -datapath_only -nowarn
# Do not false-path these buses: that would override this physical skew bound.
set_max_delay -from [get_regs -nowarn {u_wav_buffer/wr_gray*}] -to [get_regs -nowarn {u_wav_buffer/wr_sync1*}] 10.000 -datapath_only -nowarn
set_max_delay -from [get_regs -nowarn {u_wav_buffer/rd_gray*}] -to [get_regs -nowarn {u_wav_buffer/rd_sync1*}] 40.000 -datapath_only -nowarn

# Mode and playback-end are single-bit levels; cut first capture stages only.
# The sole mode_sw fanout is its first sampler, packed into the C8 IREG.
# A port-started path ends there; the second sampler remains timed.
set_false_path -from [get_ports {mode_sw}]
set_false_path -to [get_regs {u_mode_pixel/s1*}]
set_false_path -to [get_regs {u_audio_end_sd/s1*}]

# SW2 has a single first-stage sampler; second stage remains timed.
set_false_path -from [get_ports {menu_sw}]

# RTC input samplers are packed into the D14/G11 IREGs. Port-started paths
# end at those first samplers; the second synchronization stage stays timed.
set_false_path -from [get_ports {rtc_scl}]
set_false_path -from [get_ports {rtc_sda}]
# RTC bundled data mailbox: bound stable payload flight to one source period.
set_max_delay -from [get_regs {u_rtc_display/payload*}] -to [get_regs {u_rtc_display/pixel_data*}] 10.000 -datapath_only
