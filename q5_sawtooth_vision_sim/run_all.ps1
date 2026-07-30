$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $MyInvocation.MyCommand.Path

& (Join-Path $root 'sim\run_modelsim.ps1')
python (Join-Path $root 'vision_sim.py') --output (Join-Path $root 'outputs')
if ($LASTEXITCODE -ne 0) {
    throw "Vision simulation exited with code $LASTEXITCODE"
}
