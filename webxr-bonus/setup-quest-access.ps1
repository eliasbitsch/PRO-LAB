# Run as Administrator. Sets up port forwarding from Windows LAN to WSL2
# so the Quest 3 (or any LAN device) can reach the WebXR app.
#
# Usage:  PowerShell as Admin -> .\setup-quest-access.ps1

if (-not ([Security.Principal.WindowsPrincipal] [Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole([Security.Principal.WindowsBuiltInRole] 'Administrator')) {
    Write-Host 'Must run as Administrator. Restarting elevated...' -ForegroundColor Yellow
    Start-Process powershell -Verb RunAs -ArgumentList "-NoProfile -ExecutionPolicy Bypass -File `"$PSCommandPath`""
    exit
}

$port = 8080
$wslIp = (wsl hostname -I).Trim().Split()[0]

# Refresh portproxy
netsh interface portproxy reset | Out-Null
netsh interface portproxy add v4tov4 listenport=$port listenaddress=0.0.0.0 connectport=$port connectaddress=$wslIp | Out-Null

# Firewall (idempotent)
$ruleName = "WSL WebXR $port"
Get-NetFirewallRule -DisplayName $ruleName -ErrorAction SilentlyContinue | Remove-NetFirewallRule -ErrorAction SilentlyContinue
New-NetFirewallRule -DisplayName $ruleName -Direction Inbound -LocalPort $port -Protocol TCP -Action Allow | Out-Null

Write-Host "WSL IP   : $wslIp"
Write-Host "Forward  : 0.0.0.0:$port -> $wslIp`:$port"
Write-Host ""
Write-Host "LAN URL  :"
Get-NetIPAddress -AddressFamily IPv4 |
    Where-Object { $_.PrefixOrigin -in 'Dhcp', 'Manual' -and $_.InterfaceAlias -notmatch 'Loopback|vEthernet|WSL|Hyper' } |
    ForEach-Object { Write-Host ("    https://{0}:{1}" -f $_.IPAddress, $port) -ForegroundColor Cyan }

Write-Host ""
Write-Host "Open the URL on your Quest 3 Browser. Accept the self-signed cert warning ('Advanced -> Proceed')."
Write-Host "Press any key to exit..."
[void][System.Console]::ReadKey($true)
