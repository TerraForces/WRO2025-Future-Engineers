# Engineering materials

This repository contains the engineering materials of our self-driven vehicle's model participating in the WRO Future Engineers competition in the season 2025.

## Content

* [`t-photos`](/t-photos/) contains 2 photos of our team (an [official](/t-photos/official.jpg) one and one [funny](/t-photos/funny.jpg) photo with all our team members)
* [`v-photos`](/v-photos/) contains 6 photos of the vehicle (from every side, from top and bottom)
* [`video`](/video/) contains the [video.md](/video/video.md) file with the links to our driving demonstration videos
* [`schemes`](/schemes/) contains the schematic diagrams of our 3 PCBs ([power](/schemes/powerPCB.png), [lights](/schemes/ledPCB.png), [logic](/schemes/logicPCB.png))
* [`src`](/src/) contains the source code for our two controllers ([ESP32](/src/WRO%20ESP32/) and [Raspberry Pi 5](/src/WRO%20RPI5/))

## Code Description

The code of our autonomously driving vehicle mainly runs on the [ESP32](/src/WRO%20ESP32/) microcontroller. Our only other programmed controller, the [Raspberry Pi 5](/src/WRO%20RPI5/), is only used for image processing and object detection and therefore deactivated during open challenge rounds. 

### ESP32

We divided our code for the [ESP32](/src/WRO%20ESP32/) microcontroller into 4 different parts:
- the [main.cpp](/src/WRO%20ESP32/src/main.cpp) file containing the decision making code for both challenge types and test modes
- the [car.hpp](/src/WRO%20ESP32/src/car.hpp) file containing class declarations for all the sensors and actuators that need to be used in the decision making code
- the [car.cpp](/src/WRO%20ESP32/src/car.cpp) file containing the definitions to all the class member functions declared in the car.hpp and additionally background tasks that are required for asynchronous sensor and actuator control
- the [lib](/src/WRO%20ESP32/lib/) folder containing modified external libraries used for communication with sensors and the display

### Raspberry Pi 5
The code for the [Raspberry Pi 5](/src/WRO%20RPI5/) consists mainly of only one small source file called [main.cpp](/src/WRO%20RPI5/Inference/main.cpp). It calls a the command `rpicam-hello` which is automatically installed on the Raspberry Pi 5 with the [`rpicam-apps`](https://github.com/raspberrypi/rpicam-apps/) package. We used the following options because of different reasons:
- `--framerate 50` captures 50 frames per second and sends them through the inference pipeline (more fps are not possible without stability problems because the image inference time can reach up to 18 ms)
- `--hflip --vflip` flips the image in horizontally and vertically (required because we mounted the camera upside down)
- `--post-process-file /home/terra/Documents/WRO\\ RPI5/hailo_yolo11s.json` uses the post processing options requested in the specified file [hailo_yolo11s.json](/src/WRO%20RPI5/Inference/hailo_yolo11s.json) which is located at the path `/home/terra/Documents/WRO\ RPI5/hailo_yolo11s.json` on the Raspberry Pi 5
- `-n` disables the preview window (saves computing power)
- `-t 0` lets the program run indefinitely (disables timed program end)
- `-v` enables verbose mode (outputs object data in the console, required for us to read it)
- `--width 1296 --height 1296` translates object positions to a 1296x1296 pixels image

The file [hailo_yolo11s.json](/src/WRO%20RPI5/Inference/hailo_yolo11s.json) tells the rpicam-hello command to use an image size of 640x640 pixels for postprocessing. Additionally, it specifies the post processing stage as `hailo_yolo_inference` which is used for object detection on a Hailo AI accelerator using [YOLO](https://docs.ultralytics.com/) models like our YOLO11s. The options for this stage specify the location of a HEF file [yolov11s.hef](/src/WRO%20RPI5/Inference/yolov11s.hef) which contains the binary AI model compiled for our specific architecture, limit the detection count to 100 (which will probably never be reached) and set the detection threshold to 0.5 which means that only objects with a confidence value larger than 50% are shown.

The output of the command `rpicam-hello` is captured with help from the single-file [pstream.h](/src/WRO%20RPI5/Inference/pstream.h) header which adds the C++ steam functionality for pipes. This was necessary, because the command outputs the information unconventionally over stderr instead of stdout.

The output is read line by line. Only 2 line types are interesting for us:
- lines starting with `Viewfinder frame` (always followed by the frame number) which mark the beginning of a new frame's object list
- lines starting with `Object:` which each contain the data of a single object

Whenever a line starting with `Object:` is read, the object position, size and detection confidence are extracted and added to the `objects` buffer. When a line starting with `Viewfinder frame` is read, one empty object is added in front of the object buffer which tells the ESP32 to save the received objects and clear the receive buffer. Then, all objects are sent to the ESP32 over UART at a speed of 115200 bit per second and the `objects` buffer is cleared to read the next objects.

### AI model training
The folder [Training](/src/WRO%20RPI5/Training/) contains files used in the process of training the AI model which is done using the `YOLO` python package in the file [train.py](/src/WRO%20RPI5/Training/train.py). The over 2200 images for the training and validation data set were captured using the [capture.script](/src/WRO%20RPI5/Training/capture.script) file which saves images of the size 640x640 pixels every 2 seconds with ascending file names starting at the specified value of i (here: 3000). After capturing, the majority of images was labelled using the program [Labelme](https://labelme.io/) and converted to a COCO dataset using the command labelme2coco; only a few images were deleted.

## Upload instructions

The guides should work equally on Windows and Linux systems, the behavior on other operating systems hasn't been tested. An internet connection might be required to automatically install missing packages.

### ESP32

The code for the ESP32 microcontroller has been developed using [PlatformIO](https://platformio.org/), an extension for Visual Studio Code which is also used for the upload process:

1. make sure that you have Visual Studio Code with the [PlatformIO](https://platformio.org/) extension installed (installation guide can be found [here](https://platformio.org/install/ide?install=vscode))
2. do one of the following things:
	- open the [ESP32](/src/WRO%20ESP32/) folder using Visual Studio Code
	- open the [platformio.ini](/src/WRO%20ESP32/platformio.ini) file using Visual Studio Code
3. wait for PlatformIO to initialize (IDE restart might be required after update or fresh install)
4. plug the ESP32 into your device using an USB cable
5. press the `PlatformIO: Upload` button (arrow pointing to the right in the status bar)

Notes: When the upload process terminates with the error `the port doesn't exist` on Linux systems, it might be required to manually give the permission for execution on this port using the command `sudo chmod 777 /dev/ttyUSB0`. Other errors can usually be fixed by simply un- and replugging the ESP32 into the computer and trying again.

### Raspberry Pi 5

1. place the [Raspberry Pi 5](/src/WRO%20RPI5/) folder on the Raspberry Pi 5
2. make sure that the path in [hailo_yolo11s.json](/src/WRO%20RPI5/Inference/hailo_yolo11s.json) points to the [yolov11s.hef](/src/WRO%20RPI5/Inference/yolov11s.hef) file
3. make sure that the path in [main.cpp](/src/WRO%20RPI5/Inference/main.cpp) points to the [hailo_yolo11s.json](/src/WRO%20RPI5/Inference/hailo_yolo11s.json) file
4. compile the [main.cpp](/src/WRO%20RPI5/Inference/main.cpp) source file on the Raspberry Pi 5 using the command `g++ main.cpp -o main -Ofast` in the folder where the main.cpp file is located
5. add the generated `main` executable to autostart