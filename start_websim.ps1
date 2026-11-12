# Lance de websimulator (robotics.dinno.be) via de API, met een zichtbaar Chrome-venster, en de brug naar dit project.
#
#   .\start_websim.ps1                        pelouse van het park (vlak, 12 x 12 m)
#   .\start_websim.ps1 -Plaats "-3.4,0,90"    begin van de epsilon-baan
#
# Eenmalig, de sleutel permanent bewaren (nieuw PowerShell-venster daarna):
#   setx ROBOTSIM_KEY "rsk_jouw_sleutel"
# De sleutel staat nooit in dit bestand of in het project. Daarna rijd je in een tweede venster, bijvoorbeeld:
#   $env:JAGUAR_WIELSTRAAL = "0.135"
#   python -m jaguar.rijden.bediening --ip 127.0.0.1
param([string]$Plaats = "9,4,0", [string]$ApiMap = "C:\Dev\robotSimulatorAPI")

# setx n'agit que sur de nouveaux processus (et VS Code garde l'ancien environnement): lees de waarde dan zelf uit je gebruikersomgeving
if (-not $env:ROBOTSIM_KEY) { $env:ROBOTSIM_KEY = [Environment]::GetEnvironmentVariable("ROBOTSIM_KEY", "User") }
if (-not $env:ROBOTSIM_KEY) {
    Write-Host "ROBOTSIM_KEY ontbreekt. Eenmalig:  setx ROBOTSIM_KEY `"rsk_jouw_sleutel`"  en open een nieuw PowerShell-venster." -ForegroundColor Yellow
    exit 1
}
if (-not (Test-Path "$ApiMap\server.mjs")) {
    Write-Host "robotSimulatorAPI wordt gekloond naar $ApiMap ..."
    git clone https://github.com/mdequanter/robotSimulatorAPI $ApiMap
    if ($LASTEXITCODE -ne 0) { exit 1 }
}
$env:JAGUAR_WIELSTRAAL = "0.135"     # de websimulator rekent met een wiel van 0,135 m

Write-Host "Simulator starten (er opent een Chrome-venster: laat het zichtbaar) ..."
Start-Process powershell -ArgumentList "-NoExit", "-Command", "cd '$ApiMap'; node server.mjs --show"
Start-Sleep -Seconds 12

Write-Host "Brug starten, robot geplaatst op $Plaats ..."
New-Item -ItemType Directory -Force -Path "$PSScriptRoot\ritten" | Out-Null
Start-Process powershell -ArgumentList "-NoExit", "-Command", "cd '$PSScriptRoot'; `$env:JAGUAR_WIELSTRAAL='0.135'; python -m jaguar.testen.api_brug --plaats=$Plaats --spoor ritten\websim_spoor.csv --beeld ritten\websim_einde.jpg"
Write-Host "Klaar. Rij nu in dit venster, bijvoorbeeld:  python -m jaguar.rijden.bediening --ip 127.0.0.1"
