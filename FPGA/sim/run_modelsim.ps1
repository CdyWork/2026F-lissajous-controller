$vsim = (Get-Command vsim.exe -ErrorAction Stop).Source
Push-Location $PSScriptRoot
try {
    & $vsim -c -do modelsim.do
    if ($LASTEXITCODE -ne 0) {
        throw "ModelSim tests failed with exit code $LASTEXITCODE"
    }
}
finally {
    Pop-Location
}
