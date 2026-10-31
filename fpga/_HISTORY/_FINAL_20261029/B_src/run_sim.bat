@echo off
rem ==========================================================
rem  CSI-2 relay regression suite
rem    - stub_top_deps.v replaces vendor PHY primitives (sim only)
rem    - runs every testbench, prints RESULT / FAIL lines only
rem  Usage: double-click, or run  run_sim.bat  in cmd
rem ==========================================================
cd /d %~dp0
set PATH=D:\modelsim\win64;%PATH%

if not exist sim_work mkdir sim_work
cd sim_work

vlib work >nul 2>&1

vlog -quiet ..\uisrc\02_sim\stub_top_deps.v ..\uisrc\01_rtl\csi2\csi2_ecc.v ..\uisrc\01_rtl\csi2\crc16_csi.v ..\uisrc\01_rtl\csi2\crc16_par2.v ..\uisrc\01_rtl\csi2\csi2_tx_packet.v ..\uisrc\01_rtl\csi2\csi2_rx_parse.v ..\uisrc\01_rtl\csi2\csi2_ptr_fifo.v ..\uisrc\01_rtl\uics500_cfg\uii2c.v ..\uisrc\01_rtl\uics500_cfg\SC500GainTbl.v ..\uisrc\01_rtl\uics500_cfg\uics500reg.v ..\uisrc\01_rtl\uics500_cfg\uics500regAE.v ..\uisrc\01_rtl\uics500_cfg\uicfgcs500.v ..\uisrc\01_rtl\csi2_tx_top.v ..\uisrc\01_rtl\ae\ae_set.v ..\uisrc\02_sim\tb_csi2_ecc_crc.v ..\uisrc\02_sim\tb_crc_par2.v ..\uisrc\02_sim\tb_csi2_tx_packet.v ..\uisrc\02_sim\tb_csi2_rx_parse.v ..\uisrc\02_sim\tb_csi2_pipeline.v ..\uisrc\02_sim\tb_csi2_ptr_fifo.v ..\uisrc\02_sim\tb_csi2_tx_top.v ..\uisrc\02_sim\tb_csi2_full_line.v ..\uisrc\02_sim\tb_csi2_bank_switch.v ..\uisrc\02_sim\tb_csi2_lane_swap.v ..\uisrc\02_sim\tb_cam_cfg_i2c.v
if errorlevel 1 (
  echo [SIM] compile failed
  goto :eof
)

for %%T in (tb_csi2_ecc_crc tb_crc_par2 tb_csi2_tx_packet tb_csi2_rx_parse tb_csi2_pipeline tb_csi2_ptr_fifo tb_csi2_tx_top tb_csi2_full_line tb_csi2_bank_switch tb_csi2_lane_swap tb_cam_cfg_i2c) do (
  echo.
  echo ==== %%T ====
  vsim -c -quiet -do "run -all; quit -f" %%T | findstr /C:"RESULT" /C:"FAIL" /C:"CRC-PAR2"
)
