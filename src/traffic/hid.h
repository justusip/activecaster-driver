#pragma once

#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>
#include <optional>
#include <vector>
#include <functional>

#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <linux/types.h>
#include <linux/input.h>
#include <linux/hidraw.h>

#include "../models/vesc_output.h"
#include "../models/vesc_status.h"

using namespace std;

class HidMaster {
public:
    array<MotorOutput, 8> outputs = {};

    std::array<uint8_t, 8> gpio = {};

    bool isConnected() const;

    void run();

    bool writeGpio();

    bool writeMotors();

    void onFeedback(const function<void(const vector<VescStatus> &)> &feedback);

    bool send(const vector<uint8_t> &buf);

private:
    bool connected = false;
    int attempts = 0;
    int fd;
    mutex m;
    function<void(const vector<VescStatus> &)> onFeedbackFunc;

    optional<string> search(const string &clazz, const string &type, const string &productName);
};