param(
    [string]$FdRoot = "F:\AnlogicFpga\FD_2026.1",
    [string]$SdkRoot = ""
)

$ErrorActionPreference = "Stop"
$FdRoot = [IO.Path]::GetFullPath($FdRoot)
$buildTools = Join-Path $FdRoot "toolchain\build-tools\bin"
$toolchain = Join-Path $FdRoot "toolchain\gcc\riscv\ph1p35\bin"
$make = Join-Path $buildTools "make.exe"
$gcc = Join-Path $toolchain "riscv64-unknown-elf-gcc.exe"

if (-not (Test-Path -LiteralPath $make)) { throw "make.exe not found: $make" }
if (-not (Test-Path -LiteralPath $gcc)) { throw "PH1P35 RISC-V GCC not found: $gcc" }
if (-not $SdkRoot) {
    $SdkRoot = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot "..\..\lab_hd_1_mipi_hdmi\FDwork\anlu"))
}
$SdkRoot = [IO.Path]::GetFullPath($SdkRoot).Replace('\','/')
if (-not (Test-Path -LiteralPath (Join-Path $SdkRoot "tools\make\rules.mk"))) {
    throw "Anlogic PH1P35 SDK not found: $SdkRoot"
}

$oldPath = $env:Path
Push-Location $PSScriptRoot
try {
    $env:Path = "$buildTools;$toolchain;$oldPath"
    & $make -j8 all "SDK_ROOT=$($SdkRoot.Replace('\','/'))" "TARGET_PATH=$PSScriptRoot" "APP_TAR_DIR=$PSScriptRoot\build" "COMPILE_PREFIX=riscv64-unknown-elf-" "DOWNLOAD=tcm"
    if ($LASTEXITCODE -ne 0) { throw "MCU build failed with exit code $LASTEXITCODE" }
} finally {
    $env:Path = $oldPath
    Pop-Location
}

Write-Host "Built: $PSScriptRoot\build\ph1p35_digit_uart.elf"
Write-Host "Built: $PSScriptRoot\build\ph1p35_digit_uart.bin"
