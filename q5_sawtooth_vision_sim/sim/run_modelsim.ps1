$ErrorActionPreference = 'Stop'
$scriptDirectory = Split-Path -Parent $MyInvocation.MyCommand.Path
Push-Location $scriptDirectory
try {
    vsim -c -do modelsim.do
    if ($LASTEXITCODE -ne 0) {
        throw "ModelSim exited with code $LASTEXITCODE"
    }
    $transcript = Get-Content -Raw transcript
    if ($transcript -notmatch 'MODELSIM_RESULT: PASS') {
        throw 'ModelSim regression did not report PASS'
    }
} finally {
    Pop-Location
}
