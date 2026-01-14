@echo off
setlocal

REM Try to find ROS 2 installation
set "ROS_PATH="
if exist "C:\dev\ros2_iron\local_setup.bat" set "ROS_PATH=C:\dev\ros2_iron\local_setup.bat"
if exist "C:\dev\ros2_humble\local_setup.bat" set "ROS_PATH=C:\dev\ros2_humble\local_setup.bat"
if exist "C:\opt\ros\humble\x64\setup.bat" set "ROS_PATH=C:\opt\ros\humble\x64\setup.bat"
if exist "C:\ros2_iron\local_setup.bat" set "ROS_PATH=C:\ros2_iron\local_setup.bat"

if "%ROS_PATH%"=="" (
    echo [ERROR] Could not find ROS 2 installation in standard paths.
    echo Please ensure ROS 2 is installed (e.g., in C:\dev\ros2_iron)
    pause
    exit /b 1
)

echo [INFO] Found ROS 2 at: %ROS_PATH%
call "%ROS_PATH%"

REM Source local workspace
if exist "install\setup.bat" (
    echo [INFO] Sourcing local workspace...
    call "install\setup.bat"
) else (
    echo [INFO] Building workspace...
    colcon build --merge-install --packages-select sortabot_description
    if errorlevel 1 (
        echo [ERROR] Build failed.
        pause
        exit /b 1
    )
    call "install\setup.bat"
)

echo [INFO] Launching Visualization...
ros2 launch sortabot_description display.launch.py

pause
