# Project modification boundaries

User instruction dated 2026-09-16: strictly follow the module permission table
in `C:/Users/Lenovo/Desktop/22bf90f317a5bdfb9ba263e496dea63c.jpg`.
This is a project-wide engineering constraint, including referenced modules
outside this SD directory. The user explicitly forbids future edits to anything
marked "不要修改" or "绝对不能修改".

## Do not modify

- SD/frame_fifo_read.v and SD/frame_fifo_write.v (frame_fifo_read/write).
- SD/video_timing_data.v.
- SD/video_delay.v.
- SD/sdram.v and include/sdr_*.enc.v (SDRAM controller and encrypted cores).
- hdmi1.4b_transmitter_core/ (HDMI transmitter encrypted core, including
  hdmi_1_4b_transmitter_core_wrapper).
- hdmi_phy_warpper.v (actual project filename; module hdmi_phy_wrapper).

Do not bypass these restrictions by renaming, replacing, shadowing, or editing
copies of the protected modules used by the build. Reading, inspecting and
testing them is allowed. Implement new behavior in independent modules and
connect through existing interfaces from editable integration/application code.
If a requested feature cannot be implemented within these boundaries, explain
the conflict rather than silently changing a protected module.

## Modify cautiously

- SD/sd_card_top.v: SD-card SPI protocol top.
- SD/spi_master.v: SPI driver.

Only modify these when necessary and supported by specific evidence; prefer
application-layer changes and verify protocol behavior when changed.

## Allowed scopes from the table

- SD/sd_card_bmp.v: primary application/control changes.
- SD/bmp_read.v: BMP compatibility improvements.
- SD/frame_read_write.v: frame-buffer control optimization.
- hdmi_audio_tone_i2s_64fs.v: tone and volume changes, if present.
- SD/seg_decoder.v and SD/seg_scan.v: display content changes.

## Existing state

Before this instruction, brightness scaling was added to video_delay.v.
The new restriction applies from now on. Do not automatically revert or migrate
that existing implementation: reverting also changes a now-protected file and
could disrupt current functionality. Be explicit about this history; do not
claim protected files have never been modified.

Board naming observed by user: HX4S20C V1.0, EG4S20BG256; current physical K2
maps to top key1/B2, physical K4 to top key2/C1. Avoid confusing physical button
labels with logical KEY2/KEY3 names in the reference document.

User confirmed physical K3=B1 on 2026-09-16. It is now top key3, connected to
sd_card_bmp.key_play_n, for the independent short-release autoplay toggle.
