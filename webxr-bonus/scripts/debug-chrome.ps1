# Launches a fresh Chrome instance with --remote-debugging-port and preloads
# the user's existing Immersive Web Emulator extension. Then prints all
# console / errors / network failures from the page to stdout.
# Usage:  powershell -ExecutionPolicy Bypass -File debug-chrome.ps1 [URL]

param(
    [string]$Url = 'https://192.168.1.13:8080',
    [int]$Port   = 9222,
    [string]$ExtensionId = 'cgffilbpcibhmcfbgggfhfolhkfbhmik'
)

$chrome = "$env:ProgramFiles\Google\Chrome\Application\chrome.exe"
if (-not (Test-Path $chrome)) { $chrome = "${env:ProgramFiles(x86)}\Google\Chrome\Application\chrome.exe" }
if (-not (Test-Path $chrome)) { throw 'chrome.exe not found' }

# Locate the WebXR emulator extension in the default Chrome profile
$extRoot = Join-Path $env:LOCALAPPDATA "Google\Chrome\User Data\Default\Extensions\$ExtensionId"
if (-not (Test-Path $extRoot)) {
    Write-Warning "Extension $ExtensionId not found in default profile. Continuing without it."
    $loadExt = $null
} else {
    $verDir = Get-ChildItem $extRoot | Sort-Object Name -Descending | Select-Object -First 1
    $loadExt = $verDir.FullName
    Write-Host "Loading extension from: $loadExt" -ForegroundColor DarkGray
}

$profileDir = Join-Path $env:TEMP 'chrome-webxr-debug'
if (Test-Path $profileDir) { Remove-Item -Recurse -Force $profileDir -ErrorAction SilentlyContinue }
New-Item -ItemType Directory -Path $profileDir | Out-Null

$args = @(
    "--remote-debugging-port=$Port",
    "--user-data-dir=$profileDir",
    '--no-first-run',
    '--no-default-browser-check',
    '--ignore-certificate-errors',
    '--disable-features=ChromeAppsDeprecation',
    '--auto-open-devtools-for-tabs',
    $Url
)
if ($loadExt) { $args = @("--load-extension=$loadExt") + $args }

Write-Host "Launching Chrome on debug port $Port..." -ForegroundColor Cyan
Start-Process $chrome -ArgumentList $args
Start-Sleep -Seconds 3

# Find the tab matching our URL
$tabs = $null
for ($i = 0; $i -lt 20; $i++) {
    try { $tabs = Invoke-RestMethod "http://localhost:$Port/json" -TimeoutSec 2; break } catch { Start-Sleep -Milliseconds 500 }
}
if (-not $tabs) { throw "Could not reach Chrome debug API on port $Port" }
$tab = $tabs | Where-Object { $_.url -like "*$([System.Uri]::new($Url).Host)*" } | Select-Object -First 1
if (-not $tab) { $tab = $tabs | Where-Object { $_.type -eq 'page' } | Select-Object -First 1 }
if (-not $tab) { throw 'No suitable tab found' }

Write-Host "Streaming console from: $($tab.url)`n" -ForegroundColor Cyan
Write-Host "================ CONSOLE ================" -ForegroundColor Yellow

$ws = New-Object System.Net.WebSockets.ClientWebSocket
$uri = [Uri]::new($tab.webSocketDebuggerUrl)
$ws.ConnectAsync($uri, [Threading.CancellationToken]::None).Wait()

function Send-WS([System.Net.WebSockets.ClientWebSocket]$ws, [hashtable]$obj) {
    $json = ($obj | ConvertTo-Json -Compress -Depth 10)
    $bytes = [Text.Encoding]::UTF8.GetBytes($json)
    $seg = [ArraySegment[byte]]::new($bytes)
    $ws.SendAsync($seg, [Net.WebSockets.WebSocketMessageType]::Text, $true, [Threading.CancellationToken]::None).Wait()
}

Send-WS $ws @{ id = 1; method = 'Runtime.enable' }
Send-WS $ws @{ id = 2; method = 'Log.enable' }
Send-WS $ws @{ id = 3; method = 'Network.enable' }
Send-WS $ws @{ id = 4; method = 'Page.enable' }

$buf  = New-Object byte[] (256 * 1024)
$cs   = [Threading.CancellationToken]::None

while ($ws.State -eq 'Open') {
    $sb = New-Object Text.StringBuilder
    do {
        $seg = [ArraySegment[byte]]::new($buf)
        $r = $ws.ReceiveAsync($seg, $cs).Result
        [void]$sb.Append([Text.Encoding]::UTF8.GetString($buf, 0, $r.Count))
    } while (-not $r.EndOfMessage)
    $msg = $sb.ToString() | ConvertFrom-Json
    switch ($msg.method) {
        'Runtime.consoleAPICalled' {
            $type = $msg.params.type
            $vals = $msg.params.args | ForEach-Object {
                if ($_.value -ne $null) { $_.value } elseif ($_.description) { $_.description } else { $_ | ConvertTo-Json -Compress -Depth 4 }
            }
            $color = switch ($type) { 'error' {'Red'} 'warning' {'Yellow'} 'info' {'Cyan'} default {'Gray'} }
            Write-Host ("[{0}] {1}" -f $type, ($vals -join ' ')) -ForegroundColor $color
        }
        'Runtime.exceptionThrown' {
            $e = $msg.params.exceptionDetails
            $txt = "$($e.text) — $($e.exception.description)"
            Write-Host "[exception] $txt" -ForegroundColor Red
        }
        'Log.entryAdded' {
            $e = $msg.params.entry
            Write-Host ("[{0}/{1}] {2}" -f $e.level, $e.source, $e.text) -ForegroundColor Magenta
        }
        'Network.loadingFailed' {
            Write-Host ("[net-fail] {0} {1}" -f $msg.params.errorText, $msg.params.requestId) -ForegroundColor Red
        }
        'Page.frameStartedLoading' { Write-Host '[page] navigating' -ForegroundColor DarkGray }
    }
}
