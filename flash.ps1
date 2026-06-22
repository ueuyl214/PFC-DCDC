param(
  [switch]$NoBuild,
  [switch]$UnderReset,
  [switch]$DryRun,
  [string]$Config = "Debug",
  [string]$Elf = "",
  [string]$Programmer = "",
  [string]$Ninja = ""
)

$ErrorActionPreference = "Stop"

$Root = Split-Path -Parent $MyInvocation.MyCommand.Path
$BuildDir = Join-Path $Root ("build\" + $Config)

function Find-Exe {
  param(
    [string]$Name,
    [string]$ExplicitPath,
    [string[]]$Candidates
  )

  if (($ExplicitPath -ne "") -and (Test-Path $ExplicitPath)) {
    return (Resolve-Path $ExplicitPath).Path
  }

  $cmd = Get-Command $Name -ErrorAction SilentlyContinue
  if ($cmd -ne $null) {
    return $cmd.Source
  }

  foreach ($candidate in $Candidates) {
    if (Test-Path $candidate) {
      return (Resolve-Path $candidate).Path
    }
  }

  return ""
}

$ProgrammerCandidates = @(
  "$env:LOCALAPPDATA\stm32cube\bundles\programmer\2.22.0+st.1\bin\STM32_Programmer_CLI.exe",
  "C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe",
  "C:\Program Files (x86)\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"
)

$NinjaCandidates = @(
  "$env:LOCALAPPDATA\stm32cube\bundles\ninja\1.13.2+st.1\bin\ninja.exe"
)

$ProgrammerExe = Find-Exe "STM32_Programmer_CLI.exe" $Programmer $ProgrammerCandidates
if ($ProgrammerExe -eq "") {
  Write-Host "ERROR: STM32_Programmer_CLI.exe not found."
  Write-Host "Install STM32CubeProgrammer or pass -Programmer <path>."
  exit 1
}

if ($NoBuild) {
  Write-Host "Skip build: -NoBuild"
} else {
  $NinjaExe = Find-Exe "ninja.exe" $Ninja $NinjaCandidates
  if ($NinjaExe -eq "") {
    Write-Host "ERROR: ninja.exe not found."
    Write-Host "Pass -NoBuild to flash an existing ELF, or pass -Ninja <path>."
    exit 1
  }

  Write-Host "Building $Config..."
  & $NinjaExe -C $BuildDir
  if ($LASTEXITCODE -ne 0) {
    Write-Host "ERROR: build failed."
    exit $LASTEXITCODE
  }
}

if ($Elf -eq "") {
  $Elf = Join-Path $BuildDir "PFC-DCDC.elf"
}

if (-not (Test-Path $Elf)) {
  Write-Host "ERROR: ELF not found: $Elf"
  exit 1
}

$Elf = (Resolve-Path $Elf).Path

Write-Host "Programmer: $ProgrammerExe"
Write-Host "ELF       : $Elf"
Write-Host "Connect ST-LINK and power the target, then flashing over SWD..."

if ($UnderReset) {
  $connectArgs = @("-c", "port=SWD", "freq=4000", "mode=UR", "reset=HWrst")
} else {
  $connectArgs = @("-c", "port=SWD", "freq=4000")
}

if ($DryRun) {
  Write-Host "Dry run command:"
  Write-Host "  `"$ProgrammerExe`" $($connectArgs -join ' ') -w `"$Elf`" -v -rst"
  exit 0
}

& $ProgrammerExe @connectArgs "-w" $Elf "-v" "-rst"
if ($LASTEXITCODE -ne 0) {
  Write-Host ""
  Write-Host "ERROR: flash failed."
  Write-Host "Tip: if normal SWD connect fails and NRST is wired, retry:"
  Write-Host "  .\flash.ps1 -UnderReset"
  exit $LASTEXITCODE
}

Write-Host ""
Write-Host "Flash OK."
