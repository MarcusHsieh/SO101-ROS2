# SO-101 ROS2 Setup Script for Windows (WSL2)
# =============================================
# This script sets up the development environment on Windows with WSL2.
#
# Prerequisites:
#   - Windows 10 version 2004+ or Windows 11
#   - WSL2 with Ubuntu installed
#   - Docker Desktop for Windows
#
# Usage:
#   Run in PowerShell as Administrator:
#   .\setup_windows.ps1

$ErrorActionPreference = "Stop"

Write-Host "==========================================" -ForegroundColor Cyan
Write-Host "  SO-101 ROS2 Setup for Windows (WSL2)"
Write-Host "==========================================" -ForegroundColor Cyan
Write-Host ""

# Check if running as Administrator
$isAdmin = ([Security.Principal.WindowsPrincipal] [Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)
if (-not $isAdmin) {
    Write-Host "Please run this script as Administrator" -ForegroundColor Red
    exit 1
}

# Step 1: Check WSL2
Write-Host ""
Write-Host "Step 1: Checking WSL2 installation..." -ForegroundColor Yellow
try {
    $wslVersion = wsl --version 2>&1
    if ($LASTEXITCODE -eq 0) {
        Write-Host "WSL2 is installed" -ForegroundColor Green
        wsl --version
    } else {
        throw "WSL not found"
    }
} catch {
    Write-Host "WSL2 is not installed or not configured" -ForegroundColor Red
    Write-Host "Please install WSL2 first:"
    Write-Host "  wsl --install"
    Write-Host "  # Restart computer after installation"
    exit 1
}

# Step 2: Check Docker Desktop
Write-Host ""
Write-Host "Step 2: Checking Docker Desktop..." -ForegroundColor Yellow
try {
    $dockerVersion = docker --version
    Write-Host "Docker is installed: $dockerVersion" -ForegroundColor Green

    # Check if Docker Desktop WSL2 backend is enabled
    $dockerInfo = docker info 2>&1
    if ($dockerInfo -match "WSL") {
        Write-Host "Docker Desktop WSL2 backend is enabled" -ForegroundColor Green
    } else {
        Write-Host "Please enable WSL2 backend in Docker Desktop settings" -ForegroundColor Yellow
    }
} catch {
    Write-Host "Docker is not installed or not running" -ForegroundColor Red
    Write-Host "Please install Docker Desktop for Windows:"
    Write-Host "  https://docs.docker.com/desktop/install/windows-install/"
    exit 1
}

# Step 3: Install usbipd-win for USB passthrough
Write-Host ""
Write-Host "Step 3: Checking usbipd-win for USB passthrough..." -ForegroundColor Yellow
try {
    $usbipd = Get-Command usbipd -ErrorAction SilentlyContinue
    if ($usbipd) {
        Write-Host "usbipd-win is installed" -ForegroundColor Green
        usbipd --version
    } else {
        throw "usbipd not found"
    }
} catch {
    Write-Host "usbipd-win is not installed" -ForegroundColor Yellow
    Write-Host "Installing usbipd-win via winget..."

    try {
        winget install --interactive --exact dorssel.usbipd-win
        Write-Host "usbipd-win installed successfully" -ForegroundColor Green
        Write-Host "Please restart this script after installation" -ForegroundColor Yellow
    } catch {
        Write-Host "Failed to install via winget. Please install manually:" -ForegroundColor Red
        Write-Host "  https://github.com/dorssel/usbipd-win/releases"
        Write-Host "  Download and run the .msi installer"
    }
}

# Step 4: List USB devices
Write-Host ""
Write-Host "Step 4: Listing USB devices..." -ForegroundColor Yellow
Write-Host "Connected USB devices:"
try {
    usbipd list
} catch {
    Write-Host "Could not list USB devices. Ensure usbipd-win is installed." -ForegroundColor Yellow
}

# Step 5: Create helper scripts
Write-Host ""
Write-Host "Step 5: Creating helper scripts..." -ForegroundColor Yellow

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path

# Create USB attach script
$attachScript = @'
# attach_usb.ps1 - Attach USB serial device to WSL2
# Run as Administrator
#
# Usage:
#   .\attach_usb.ps1
#   .\attach_usb.ps1 -BusId "1-3"

param(
    [string]$BusId = ""
)

Write-Host "USB Device Attachment for SO-101 Robot" -ForegroundColor Cyan
Write-Host ""

# List devices
Write-Host "Available USB devices:" -ForegroundColor Yellow
usbipd list

if ($BusId -eq "") {
    Write-Host ""
    Write-Host "Looking for common serial adapters (CH340, FTDI, CP210x)..." -ForegroundColor Yellow

    # Try to find common USB serial adapters
    $devices = usbipd list 2>&1
    $serialPatterns = @("CH340", "CH341", "FTDI", "CP210", "USB-SERIAL", "USB Serial")

    foreach ($pattern in $serialPatterns) {
        $match = $devices | Select-String -Pattern $pattern
        if ($match) {
            Write-Host "Found: $match" -ForegroundColor Green
            # Extract bus ID (format: X-X)
            if ($match -match "(\d+-\d+)") {
                $BusId = $Matches[1]
                Write-Host "Detected Bus ID: $BusId" -ForegroundColor Green
                break
            }
        }
    }
}

if ($BusId -eq "") {
    Write-Host ""
    Write-Host "Could not auto-detect serial device." -ForegroundColor Yellow
    $BusId = Read-Host "Enter the Bus ID of your USB serial device (e.g., 1-3)"
}

if ($BusId -ne "") {
    Write-Host ""
    Write-Host "Binding device $BusId..." -ForegroundColor Yellow
    usbipd bind --busid $BusId --force

    Write-Host "Attaching device $BusId to WSL..." -ForegroundColor Yellow
    usbipd attach --wsl --busid $BusId

    Write-Host ""
    Write-Host "Device attached! In WSL, check for /dev/ttyUSB0 or /dev/ttyACM0" -ForegroundColor Green
    Write-Host "Run: ls -la /dev/tty*" -ForegroundColor Cyan
} else {
    Write-Host "No Bus ID provided. Exiting." -ForegroundColor Red
}
'@

$attachScriptPath = Join-Path $scriptDir "attach_usb.ps1"
$attachScript | Out-File -FilePath $attachScriptPath -Encoding UTF8
Write-Host "Created: $attachScriptPath" -ForegroundColor Green

# Create USB detach script
$detachScript = @'
# detach_usb.ps1 - Detach USB serial device from WSL2
# Run as Administrator

Write-Host "Detaching all USB devices from WSL..." -ForegroundColor Yellow
usbipd list | ForEach-Object {
    if ($_ -match "(\d+-\d+).*Attached") {
        $busId = $Matches[1]
        Write-Host "Detaching $busId..." -ForegroundColor Cyan
        usbipd detach --busid $busId
    }
}
Write-Host "Done!" -ForegroundColor Green
'@

$detachScriptPath = Join-Path $scriptDir "detach_usb.ps1"
$detachScript | Out-File -FilePath $detachScriptPath -Encoding UTF8
Write-Host "Created: $detachScriptPath" -ForegroundColor Green

# Step 6: Create WSL setup script
Write-Host ""
Write-Host "Step 6: Creating WSL setup script..." -ForegroundColor Yellow

$wslSetupScript = @'
#!/bin/bash
# setup_wsl.sh - Run this inside WSL2 to complete setup
set -e

echo "=========================================="
echo "  SO-101 Setup inside WSL2"
echo "=========================================="

# Install usbip client tools in WSL
echo "Installing USB/IP tools..."
sudo apt-get update
sudo apt-get install -y linux-tools-generic hwdata
sudo update-alternatives --install /usr/local/bin/usbip usbip /usr/lib/linux-tools/*-generic/usbip 20

# Install Python dependencies
echo "Installing Python dependencies..."
pip3 install --user pyserial numpy pyyaml

# Check for serial devices
echo ""
echo "Checking for serial devices..."
ls -la /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo "No serial devices found yet"

echo ""
echo "=========================================="
echo "  WSL Setup Complete!"
echo "=========================================="
echo ""
echo "After attaching USB with attach_usb.ps1, check:"
echo "  ls -la /dev/ttyUSB*"
echo ""
'@

$wslSetupPath = Join-Path $scriptDir "setup_wsl.sh"
$wslSetupScript | Out-File -FilePath $wslSetupPath -Encoding UTF8 -NoNewline
Write-Host "Created: $wslSetupPath" -ForegroundColor Green

# Step 7: Create environment file for Docker
Write-Host ""
Write-Host "Step 7: Creating Docker environment file..." -ForegroundColor Yellow

$dockerDir = Join-Path (Split-Path $scriptDir -Parent) "docker"
$envContent = @"
# SO-101 Docker Environment Configuration
# Windows/WSL2 configuration

# Use bridge network (host network not available on Windows)
NETWORK_MODE=bridge

# ROS2 Domain ID
ROS_DOMAIN_ID=0

# Serial port inside WSL (after USB attach)
SERIAL_PORT=/dev/ttyUSB0
"@

$envPath = Join-Path $dockerDir ".env"
$envContent | Out-File -FilePath $envPath -Encoding UTF8
Write-Host "Created: $envPath" -ForegroundColor Green

# Print summary
Write-Host ""
Write-Host "==========================================" -ForegroundColor Cyan
Write-Host "  Setup Complete!" -ForegroundColor Green
Write-Host "==========================================" -ForegroundColor Cyan
Write-Host ""
Write-Host "Next Steps:" -ForegroundColor Yellow
Write-Host ""
Write-Host "1. Open WSL terminal and run the setup script:"
Write-Host "   wsl"
Write-Host "   cd /mnt/c/path/to/lerobot_ws/scripts"
Write-Host "   chmod +x setup_wsl.sh && ./setup_wsl.sh"
Write-Host ""
Write-Host "2. Connect your SO-101 robot via USB"
Write-Host ""
Write-Host "3. In PowerShell (Admin), attach USB to WSL:"
Write-Host "   .\attach_usb.ps1"
Write-Host ""
Write-Host "4. In WSL, verify the device appeared:"
Write-Host "   ls -la /dev/ttyUSB*"
Write-Host ""
Write-Host "5. Run the motor bridge (in WSL):"
Write-Host "   python3 motor_bridge.py --port /dev/ttyUSB0 --mode websocket"
Write-Host ""
Write-Host "6. Start Docker containers:"
Write-Host "   cd docker"
Write-Host "   docker compose up -d"
Write-Host ""
Write-Host "7. Open Foxglove Studio and connect to:"
Write-Host "   ws://localhost:8765"
Write-Host ""
