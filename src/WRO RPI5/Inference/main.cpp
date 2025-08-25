#include <algorithm>
#include <cstdint>
#include <fcntl.h>
#include <iostream>
#include <stdio.h>
#include <string>
#include <termios.h>
#include "pstream.h"

// possible object colors
enum COLORS : uint8_t {
    CLEAR, // sent as first object in each transmission, clears object buffer
    RED,   // red traffic sign
    GREEN, // green traffic sign
    PINK   // pink parking space borders
};

// description of detected objects
struct OBJECT {
    uint16_t l; // left x position of the bounding rect in image pixels (0 - 1295)
    uint16_t r; // right x position of the bounding rect in image pixels (0 - 1295)
    uint16_t t; // top y position of the bounding rect in image pixels (0 - 1295)
    uint16_t b; // bottom y position of the bounding rect in image pixels (0 - 1295)
    uint16_t w; // bounding rect width in image pixels (0 - 1295)
    uint16_t h; // bounding rect height in image pixels (0 - 1295)
    uint16_t c; // object color (see possible object colors above)
    uint16_t p; // detection probability in 0.1% (0 - 1000)
};

// program start
int main() {

    // configure and start UART bus at 115200 bits per second
    int32_t uart = open("/dev/ttyAMA0", O_RDWR);
    termios tty;
    tcgetattr(uart, &tty);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;
    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;
    tty.c_lflag &= ~ECHOE;
    tty.c_lflag &= ~ECHONL;
    tty.c_lflag &= ~ISIG;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;
    tty.c_cc[VTIME] = 10;
    tty.c_cc[VMIN] = 0;
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);
    tcsetattr(uart, TCSANOW, &tty);

    // start camera + inference pipeline and capture output (command is using error output for logging)
    ipstream pipe("rpicam-hello --framerate 50 --hflip --vflip --post-process-file /home/terra/Documents/WRO\\ RPI5/hailo_yolo11s.json -n -t 0 -v --width 1296 --height 1296", pstreams::pstderr);
    
    // read output lines and extract important data
    std::string line;
    std::vector<OBJECT> objects;
    while(getline(pipe.out(), line)) {

        // new frame starts always with "Viewfinder frame" + frame index
        if(line.compare(0, 16, "Viewfinder frame") == 0) {

            // sort objects by relevance (nearest object, not pink)
            if(objects.size() >= 2) {
                std::sort(objects.begin(), objects.end(),
                    [](OBJECT l, OBJECT r) {
                        if((l.c == PINK) != (r.c == PINK)) return r.c == PINK;
                        return l.b > r.b;
                    }
                );
            }

            // start data transfer with clear command object
            objects.insert(objects.begin(), OBJECT{});

            // send objects to ESP32
            write(uart, objects.data(), sizeof(OBJECT) * objects.size());

            // clear object list
            objects.clear();
        }

        // lines starting with "Object:" contain data of new objects
        else if(line.compare(0, 8, "Object: ") == 0) {

            // extract position data
            uint16_t x = strtoul(line.substr(line.find_first_of('@') + 2).c_str(), 0, 10);
            uint16_t y = strtoul(line.substr(line.find_first_of(',') + 1).c_str(), 0, 10);
            int16_t width = strtol(line.substr(line.find_last_of(' ') + 1).c_str(), 0, 10);
            int16_t height = strtol(line.substr(line.find_last_of('x') + 1).c_str(), 0, 10);

            // calculate normalized position data
            OBJECT object = {};
            object.l = width >= 0 ? x : x + width;
            object.r = width >= 0 ? x + width : x;
            object.t = height >= 0 ? y : y + height;
            object.b = height >= 0 ? y + height : y;
            object.w = width >= 0 ? width : -width;
            object.h = height >= 0 ? height : -height;

            // extract object color
            object.c = line[line.find_first_of('[') + 1] - 48u;

            // extract detection probability
            object.p = std::stof(line.substr(line.find_first_of('(') + 1)) * 1000u;

            // add object to list
            objects.push_back(object);
        }
    }

    // stop uart when stream ends
    close(uart);
}