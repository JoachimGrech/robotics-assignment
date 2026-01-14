# ROS 2 Humble Installer Script for Windows
# Run this script as Administrator!

Write-Host "Starting ROS 2 Humble Installation..." -ForegroundColor Cyan

# 1. Create Directories
$installPath = "C:\dev\ros2_humble"
if (!(Test-Path -Path "C:\dev")) { New-Item -ItemType Directory -Path "C:\dev" | Out-Null }
if (Test-Path -Path $installPath) {
    Write-Host "ROS 2 folder already exists at $installPath. Skipping download." -ForegroundColor Yellow
} else {
    # 2. Download ROS 2
    $url = "https://github.com/ros2/ros2/releases/download/release-humble-20230614/ros2-humble-20230614-windows-release-amd64.zip"
    $zipPath = "C:\dev\ros2.zip"
    
    Write-Host "Downloading ROS 2 (approx 500MB)... This may take a while." -ForegroundColor Green
    Invoke-WebRequest -Uri $url -OutFile $zipPath
    
    # 3. Extract
    Write-Host "Extracting files... This may take a few minutes." -ForegroundColor Green
    Expand-Archive -Path $zipPath -DestinationPath "C:\dev" -Force
    
    # Rename folder if needed (zip usually extracts to 'ros2-windows')
    if (Test-Path "C:\dev\ros2-windows") {
        Rename-Item -Path "C:\dev\ros2-windows" -NewName "ros2_humble"
    }
    
    # Cleanup
    Remove-Item $zipPath
}

# 4. Install Dependencies via Winget
Write-Host "Installing System Dependencies..." -ForegroundColor Green
winget install Python.Python.3.10
winget install Microsoft.VisualStudio.2019.BuildTools
winget install Microsoft.VisualCpp.Redistributable.X64

# 5. Environment Variables
Write-Host "Setting Environment Variables..." -ForegroundColor Green
[System.Environment]::SetEnvironmentVariable("ROS_DISTRO", "humble", "User")
[System.Environment]::SetEnvironmentVariable("ROS_PYTHON_VERSION", "3", "User")
[System.Environment]::SetEnvironmentVariable("PYTHONPATH", "", "User") # Clear to avoid conflicts

# Warning: Modifying PATH requires checking current length to avoid truncation
# For simplicity, we create a setup script the user calls, rather than risking PATH corruption in a script.

Write-Host "Installation Files Ready." -ForegroundColor Cyan
Write-Host "To use ROS 2, you must source the environment in every new terminal:"
Write-Host "  call C:\dev\ros2_humble\local_setup.bat" -ForegroundColor Yellow

Write-Host "Setup Complete!"
Pause
