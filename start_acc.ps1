# start_acc.ps1 -- launched by start_acc.bat (or right-click -> "Run with PowerShell")

# Self-elevate if not already admin
$isAdmin = ([Security.Principal.WindowsPrincipal][Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)
if (-NOT $isAdmin) {
    Start-Process PowerShell -Verb RunAs -ArgumentList "-ExecutionPolicy Bypass -File `"$PSCommandPath`""
    exit
}

Write-Host "=== ACC Launcher ===" -ForegroundColor Cyan

# Wait for the Pico W in normal (CDC) mode.
# If it is detected in BOOTSEL mode ("RP2 Boot") we warn the user and keep waiting.
$busid = $null
$bootselWarned = $false
Write-Host "Waiting for Pico W... plug it in now." -ForegroundColor Yellow

while (-not $busid) {
    $lines = usbipd list 2>$null | Select-String "2e8a"
    if ($lines) {
        $bootselLine = $lines | Where-Object { $_ -match "Boot" }
        $normalLine  = $lines | Where-Object { $_ -notmatch "Boot" }

        if ($normalLine) {
            $busid = ($normalLine[0].ToString().Trim() -split '\s+')[0]
        } elseif ($bootselLine -and -not $bootselWarned) {
            Write-Host ""
            Write-Host "Pico is in BOOTSEL mode -- flash the firmware, then unplug and replug normally." -ForegroundColor Yellow
            $bootselWarned = $true
        } else {
            Start-Sleep -Seconds 1
            Write-Host "." -NoNewline
        }
    } else {
        $bootselWarned = $false
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
