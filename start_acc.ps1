# start_acc.ps1 -- right-click -> "Run with PowerShell"

# Self-elevate if not already admin
$isAdmin = ([Security.Principal.WindowsPrincipal][Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)
if (-NOT $isAdmin) {
    Start-Process PowerShell -Verb RunAs -ArgumentList "-ExecutionPolicy Bypass -File `"$PSCommandPath`""
    exit
}

# Wait for Pico W in a loop
$busid = $null
Write-Host "Waiting for Pico W... plug it in now." -ForegroundColor Yellow

while (-not $busid) {
    $line = usbipd list 2>$null | Select-String "2e8a"
    if ($line) {
        $busid = ($line.ToString().Trim() -split '\s+')[0]
    } else {
        Start-Sleep -Seconds 1
        Write-Host "." -NoNewline
    }
}

Write-Host ""
Write-Host "Found Pico W at busid $busid" -ForegroundColor Cyan

usbipd bind --busid $busid 2>$null

Write-Host "Attaching to WSL..."
usbipd attach --wsl --busid $busid
if ($LASTEXITCODE -ne 0) {
    Write-Host "[error] usbipd attach failed." -ForegroundColor Red
    Read-Host "Press Enter to exit"
    exit 1
}

Write-Host "Launching ACC dashboard in WSL..." -ForegroundColor Green

# Resolve the absolute WSL path to the script (avoids ~ expansion issues)
$wslHome = (wsl bash -c "echo `$HOME").Trim()
$scriptPath = "$wslHome/ACC/start_acc.sh"

$wtAvailable = Get-Command wt.exe -ErrorAction SilentlyContinue
if ($wtAvailable) {
    Start-Process wt.exe -ArgumentList @("wsl.exe", "bash", $scriptPath)
} else {
    Start-Process wsl.exe -ArgumentList @("bash", $scriptPath)
}

Read-Host "Press Enter to close this window"
