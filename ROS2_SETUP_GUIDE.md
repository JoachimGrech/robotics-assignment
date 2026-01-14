# Installing ROS 2 Humble on Windows

Since the automated script couldn't find ROS 2, you'll need to install it manually. Specifically, we recommend **ROS 2 Humble Hawksbill** (LTS).

## Step 1: Install Dependencies
Open a PowerShell terminal as **Administrator** and run the following commands to install Python and standard tools:

```powershell
winget install Python.Python.3.10
winget install Microsoft.VisualStudio.2019.BuildTools
winget install Microsoft.VisualCpp.Redistributable.X64
```

> **Note:** If you already have Python installed, ensure it is in your system PATH.

## Step 2: Download ROS 2
1.  Go to the official release page: [ROS 2 Humble Releases](https://github.com/ros2/ros2/releases/tag/release-humble-20230614)
2.  Download the **ros2-humble-20230614-windows-release-amd64.zip** file.
3.  **Unblock the zip**: Right-click the downloaded zip file -> Properties -> Check the "Unblock" box at the bottom -> OK.

## Step 3: Extract Files
1.  Create a folder named `C:\dev`.
2.  Extract the contents of the zip file into `C:\dev`.
    *   You should end up with a folder structure like `C:\dev\ros2_humble\local_setup.bat`.

## Step 4: Install Dependencies (OpenSSL & OpenCV)
1.  **OpenSSL**: Download [OpenSSL v1.1.1](https://slproweb.com/products/Win32OpenSSL.html) (Win64 OpenSSL v1.1.1w Light) and install it.
    *   Add `C:\Program Files\OpenSSL-Win64\bin\` to your System PATH environment variable.
2.  **OpenCV**: Download [OpenCV 3.4.6](https://github.com/ros2/ros2/releases/download/opencv-archives/opencv-3.4.6-vc16.VS2019.zip).
    *   Extract to `C:\opencv`.
    *   Add `C:\opencv\x64\vc16\bin` to your System PATH.

## Step 5: Verify & Run
Once everything is installed:
1.  Open a new terminal.
2.  Navigate to your assignment folder:
    ```powershell
    cd "C:\Users\joach\OneDrive\Desktop\Uni\3rdYr\Sem1\Robotics-ARI3215\robotics assignment"
    ```
3.  Run the helper script again:
    ```powershell
    .\visualize_robot.bat
    ```
