# Launches Chrome with --remote-debugging-port=9222 so the chrome-devtools-mcp
# server can attach. Uses a separate user-data-dir but loads your existing
# Immersive Web Emulator extension so WebXR works.
#
# Run this BEFORE asking Claude to inspect the page.

param(
    [string]$Url = 'https://192.168.1.13:8080',
    [int]$Port = 9222,
    [string]$ExtensionId = 'cgffilbpcibhmcfbgggfhfolhkfbhmik'  # Immersive Web Emulator
)

$chromeCandidates = @(
    "$env:ProgramFiles\Google\Chrome\Application\chrome.exe",
    "${env:ProgramFiles(x86)}\Google\Chrome\Application\chrome.exe",
    "$env:LOCALAPPDATA\Google\Chrome\Application\chrome.exe"
)
$chrome = $chromeCandidates | Where-Object { Test-Path $_ } | Select-Object -First 1
if (-not $chrome) { throw 'chrome.exe not found' }

# Find latest version of the WebXR Emulator extension in your default profile
$extRoot = Join-Path $env:LOCALAPPDATA "Google\Chrome\User Data\Default\Extensions\$ExtensionId"
$loadExt = $null
if (Test-Path $extRoot) {
    $verDir = Get-ChildItem $extRoot -Directory | Sort-Object Name -Descending | Select-Object -First 1
    if ($verDir) { $loadExt = $verDir.FullName }
}
if ($loadExt) {
    Write-Host "Loading WebXR extension from: $loadExt" -ForegroundColor Green
} else {
    Write-Warning "Immersive Web Emulator not found in default profile. WebXR will not work in this Chrome."
}

$profileDir = Join-Path $env:LOCALAPPDATA 'chrome-debug-profile'
New-Item -ItemType Directory -Path $profileDir -Force | Out-Null

$args = @(
    "--remote-debugging-port=$Port",
    "--user-data-dir=$profileDir",
    '--no-first-run',
    '--no-default-browser-check',
    '--ignore-certificate-errors',
    '--enable-features=WebXR,WebXRHandInput,WebXRLayers'
)
if ($loadExt) { $args += "--load-extension=$loadExt" }
$args += $Url

Write-Host "Launching Chrome (debug port $Port)..." -ForegroundColor Cyan
Write-Host "URL: $Url"
Write-Host ""
Write-Host "Once Chrome is open, return to Claude Code and say something like:"
Write-Host '  "list the console messages of the active tab"' -ForegroundColor Yellow
Write-Host '  "what errors did the page log?"' -ForegroundColor Yellow
Write-Host ""

Start-Process $chrome -ArgumentList $args
Start-Sleep 2
try {
    $tabs = Invoke-RestMethod "http://localhost:$Port/json/version" -TimeoutSec 5
    Write-Host "Chrome debug API live at http://localhost:$Port" -ForegroundColor Green
    Write-Host "  Browser: $($tabs.Browser)"
} catch {
    Write-Warning "Could not reach debug API yet - give it a few seconds."
}
