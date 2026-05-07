# Layout-Save-Helfer:
#  - Nimmt das neueste *.json aus dem Downloads-Ordner
#  - Kopiert es nach ws/src/pro_lab_filters/config/foxglove_layout.json
#  - Restarted den foxglove_ui-Container, damit foxglove_init.sh
#    das neue Layout in LICHTBLICK_SUITE_DEFAULT_LAYOUT patcht
#
# Danach im Browser: F12 -> Application -> IndexedDB -> lichtblick loeschen,
# dann F5. Lichtblick laedt dann nicht mehr seinen Cache, sondern den
# frischen Default.

$ErrorActionPreference = "Stop"
$repo     = Split-Path -Parent $PSScriptRoot
$target   = Join-Path $repo "ws\src\pro_lab_filters\config\foxglove_layout.json"
$downloads = Join-Path $env:USERPROFILE "Downloads"

Write-Host "[1/3] Suche neueste .json in $downloads ..." -ForegroundColor Cyan
$latest = Get-ChildItem -Path $downloads -Filter "*.json" -File `
    | Sort-Object LastWriteTime -Descending `
    | Select-Object -First 1

if (-not $latest) {
    Write-Host "ERROR: Keine .json in Downloads gefunden." -ForegroundColor Red
    exit 1
}

$ageMin = [int]((Get-Date) - $latest.LastWriteTime).TotalMinutes
Write-Host "      gefunden: $($latest.Name) (vor $ageMin min)" -ForegroundColor Gray

# Sanity-check: schaut es nach einem Lichtblick/Foxglove-Layout aus?
$content = Get-Content $latest.FullName -Raw
if ($content -notmatch '"configById"|"layout"|"globalVariables"') {
    Write-Host "WARN: Datei sieht nicht nach einem Foxglove-Layout aus." -ForegroundColor Yellow
    $confirm = Read-Host "Trotzdem kopieren? [y/N]"
    if ($confirm -ne 'y') { exit 1 }
}

Write-Host "[2/3] Kopiere -> $target" -ForegroundColor Cyan
Copy-Item -Path $latest.FullName -Destination $target -Force

Write-Host "[3/3] Restart foxglove_ui Container ..." -ForegroundColor Cyan
wsl -- bash -lc "cd /mnt/c/git/PRO-LAB/docker && docker compose restart foxglove_ui" | Out-Null

Write-Host ""
Write-Host "Fertig. Im Browser jetzt:" -ForegroundColor Green
Write-Host "  1) F12 -> Application -> IndexedDB -> 'lichtblick' loeschen" -ForegroundColor Green
Write-Host "  2) F5"  -ForegroundColor Green
Write-Host ""
Write-Host "Oder einfach: neuen Inkognito-Tab oeffnen mit:" -ForegroundColor Gray
Write-Host "  http://localhost:8082/?ds=foxglove-websocket&ds.url=ws://127.0.0.1:8767" -ForegroundColor Gray
