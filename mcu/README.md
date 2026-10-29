# PH1P35 digit UART MCU firmware

This is a standalone PH1P35 RISC-V firmware project based on the Anlogic SDK
used by `FDwork/anlu_app2`. It builds a TCM/JTAG image and does not write the
shared FPGA/QSPI flash.

The firmware reports a boot banner on USART1 at 115200 8N1, reads newline-
terminated UART lines, and accepts the FPGA result format:

```text
D,xxx,yyy,www,hhh,class
```

It checks the numeric fields and echoes accepted messages as
`MCU,D,xxx,yyy,www,hhh,class`. Malformed lines are reported as
`MCU,REJECT,...`.

Build from PowerShell:

```powershell
cd D:\2026fpga\ph1p35second\mcu
.\build.ps1
```

Override the FD installation or SDK directory if needed:

```powershell
.\build.ps1 -FdRoot 'F:\AnlogicFpga\FD_2026.1' `
  -SdkRoot 'D:\2026fpga\lab_hd_1_mipi_hdmi\FDwork\anlu'
```

Artifacts are written to `build\ph1p35_digit_uart.elf` and
`build\ph1p35_digit_uart.bin`.

## Integration boundary

The current `ph1p35second/fpga` camera top routes the recognition UART to FPGA
pin J3. The F02 board pin list identifies J3 as `FPGA_UART_TX0` for the onboard
USB-UART bridge. It is not MCU USART1 RX. The current FPGA top also does not
wire the recognition stream into the MCU. Therefore this firmware is a
separate USART1 bring-up/receiver image; it will not consume CNN results from
the current FPGA bit until an explicit FPGA-to-MCU path and the MCU boot
configuration are added and validated. For current recognition testing, read
the FPGA result lines directly from the PC COM port connected to J3.

No firmware or FPGA image is flashed by the build script.
