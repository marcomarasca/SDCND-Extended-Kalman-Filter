# Extended Kalman Filter
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

[//]: # (Image References)

[dataset_1_gif]: ./images/dataset1.gif "Tracking on the first dataset"
[dataset_2_gif]: ./images/dataset2.gif "Tracking on the second (reversed) dataset"

![alt text][dataset_1_gif]

Overview
---

This repository contains a C++ implementation of the (extended) kalman filter used to estimate the state of a moving object of interest through measurements coming from with both lidar and radar sensors. The measurements comes from a simulated environment thanks to the [Udacity Simulator](https://github.com/udacity/self-driving-car-sim) and are fed to the program through [WebSockets](https://en.wikipedia.org/wiki/WebSocket) messages. The [main](./src/main.cpp) file process the incoming messages and parse the type of measurement that is then processed by the [FusionEKF](./src/FusionEKF.cpp) class.

Each message contains the type of sensor measurement (LASER or RADAR), the measurement data and a timestamp, plus the ground truth values (e.g. real values of the position of the tracked object). The messages are encoded as JSON objects and the "sensor_measurement" attribute will contain the raw data from the sensor, the data is encoded as a space separated values string, whose first character encodes the type of measurement, **L** for Laser and **R** for Radar. The values following are the data plus ground truth.

#### Radar

```sensor_type, rho_measured, phi_measured, rhodot_measured, timestamp, x_groundtruth, y_groundtruth, vx_groundtruth, vy_groundtruth, yaw_groundtruth, yawrate_groundtruth```

sensor_type is **R**

#### Lidar

```sensor_type, x_measured, y_measured, timestamp, x_groundtruth, y_groundtruth, vx_groundtruth, vy_groundtruth, yaw_groundtruth, yawrate_groundtruth```

sensor_type is **L**

The program process this data and sends back a JSON object containing the estimated position and the root mean squared error values for the position and velocity (computed using the estimation and ground truth):

```
["estimate_x"] <= Estimated position x
["estimate_y"] <= Estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]
```

As we can see the coordinates coming from the 2 sensors are different in the type of representation, while the lidar outputs cartesian coordinates, the radar provides polar coordinates. This is where the extended kalman filter comes into place where the non-linear data is used (for the radar), the extended version uses a linear approximation for the computation.

Some (RMSE) results from various runs:

##### Dataset 1, Radar + Lidar

RMSE (x, y, vx, vy): 0.0964, 0.0853, 0.4154, 0.4316

##### Dataset 2, Radar + Lidar

RMSE (x, y, vx, vy): 0.0727, 0.0968, 0.4893, 0.5078

##### Dataset 1, Radar

RMSE (x, y, vx, vy): 0.1915, 0.2798, 0.5221, 0.6609

##### Dataset 1, Lidar

RMSE (x, y, vx, vy): 0.1222, 0.0984, 0.5511, 0.4604

As expected using both sensors together leads to a lower error in the estimation of both position and velocity.

Getting Started
---

In order to run the program you need the simulator provided by [Udacity](https://www.udacity.com/) which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even better [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. The version compatible with the simulator is the uWebSocketIO branch **e94b6e1**.

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. ```mkdir build```
2. ```cd build```
3. ```cmake .. && make```
4. ```./ExtendedKF```

Note that to compile the program with debug symbols you can supply the flag to cmake: ```cmake -DCMAKE_BUILD_TYPE=Debug .. && make```.

Now the Udacity simulator can be run selecting the EKF/UKF project, after the dataset selection press start and see the application in action.

![alt text][dataset_2_gif]

#### Other Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

Environment Setup
---

This project was developed under windows using the windows subsystem for linux ([WSL](https://docs.microsoft.com/en-us/windows/wsl/install-win10)) with Ubuntu Bash 16.04 together with [Visual Studio Code](https://code.visualstudio.com/).

The steps to setup the environment under mac, linux or windows (WSL) are more or less the same:

- Review the above dependencies
- Clone the repo and run the appropriate script (./install-ubuntu.sh under WSL and linux and ./install-mac.sh under mac), this should install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) from the branch **e94b6e1**

Under windows (WSL) and linux you can make a clean installation as follows:

1. ```sudo apt-get update```
2. ```sudo apt-get install git```
3. ```sudo apt-get install cmake```
4. ```sudo apt-get install openssl```
5. ```sudo apt-get install libssl-dev```
6. ```git clone https://github.com/Az4z3l/CarND-Extended-Kalman-Filter```
7. ```sudo rm /usr/lib/libuWS.so```
8. ```./install-ubuntu.sh```

#### Debugging with VS Code

Since I developed this project using WSL and Visual Studio Code it was very useful for me to setup a debugging pipeline. VS Code comes with a official Microsoft cpp extension that can be downloaded directly from the marketplace: https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools. After the installation there are a few things to setup in order to make it work with the subsystem for linux, personally I went with the default Ubuntu distribution.

For the following setup I assume that the repository was cloned in **D:/Dev/CarND-Extended-Kalman-Filter/**.

##### Setup the language server (for IntelliSense)

From the official documentation https://github.com/Microsoft/vscode-cpptools/blob/master/Documentation/LanguageServer/Windows%20Subsystem%20for%20Linux.md: 

Simply Crtl+P and select "C/Cpp: Edit Configurations", this will create a c_cpp_properties.json file that can be configured as follows:

```json
{
    "name": "WSL",
    "intelliSenseMode": "clang-x64",
    "compilerPath": "/usr/bin/gcc",
    "includePath": [
        "${workspaceFolder}"
    ],
    "defines": [],
    "browse": {
        "path": [
            "${workspaceFolder}"
        ],
        "limitSymbolsToIncludedHeaders": true,
        "databaseFilename": ""
    },
    "cStandard": "c11",
    "cppStandard": "c++17"
}
```

##### Setup the Debugger

From the official documenation https://github.com/Microsoft/vscode-cpptools/blob/master/Documentation/Debugger/gdb/Windows%20Subsystem%20for%20Linux.md:

First install gdb in the WSL:

```
sudo apt install gdb
```

Then simply create a lunch configuration from VS Code: "Debug" -> "Add Configuration.." and setup the launch.json as follows:

```json
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "C++ Launch",
            "type": "cppdbg",
            "request": "launch",
            "program": "/mnt/d/Dev/CarND-Extended-Kalman-Filter/build/ExtendedKF",
            "args": ["-fThreading"],
            "stopAtEntry": false,
            "cwd": "/mnt/d/Dev/CarND-Extended-Kalman-Filter/build/",
            "environment": [],
            "externalConsole": true,
            "windows": {
                "MIMode": "gdb",
                "setupCommands": [
                    {
                        "description": "Enable pretty-printing for gdb",
                        "text": "-enable-pretty-printing",
                        "ignoreFailures": true
                    }
                ]
            },
            "pipeTransport": {
                "pipeCwd": "",
                "pipeProgram": "c:\\windows\\sysnative\\bash.exe",
                "pipeArgs": ["-c"],
                "debuggerPath": "/usr/bin/gdb"
            },
            "sourceFileMap": {
                "/mnt/d": "d:\\"
            }
        }
    ]
}
```

Note how the program is mapped directly into the file system of the WSL and piped through bash.exe (the paths are relative to the WSL environment).

Now you are ready to debug the application directly from VS Code, simply compile the application from within the WSL with the debug symbols:

```cmake -DCMAKE_BUILD_TYPE=Debug .. && make```

And run the debugger from VS Code (e.g. F5) :)








