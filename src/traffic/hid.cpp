#include "hid.h"

#include <thread>
#include <cstring>

#include "ros/ros.h"

optional<string> HidMaster::search(const std::string &clazz,
                                   const std::string &type,
                                   const std::string &productName) {
    optional<string> device;
    for (const auto &d : filesystem::directory_iterator("/dev")) {
        const auto &o = d.path();

        auto devicePath = filesystem::path("/sys/class/")
                .append(clazz)
                .append(o.filename().string())
                .append("device");
        if (!filesystem::exists(devicePath))
            continue;

        auto canonicalPath = filesystem::canonical(devicePath);
        auto subsystem = filesystem::canonical(canonicalPath.append("subsystem")).filename();
        if (subsystem != type)
            continue;

        ifstream file(canonicalPath.parent_path().parent_path().parent_path().append("product"));
        string product;
        getline(file, product);
        if (product != productName)
            continue;

        return o.string();
    }
    return {};
}

void HidMaster::run() {
    ROS_INFO("Scanning USB devices...");
    while (ros::ok()) {
        if (attempts > 10) {
            ROS_ERROR("Unable to connect to the controller. %s", strerror(errno));
            return;
        }
        std::optional<string> device = search("hidraw", "hid", "Cheetah Controller");
        if (!device)
            continue;

        fd = open(device->c_str(), O_RDWR | O_NOCTTY);
        if (fd < 0) {
            attempts++;
            this_thread::sleep_for(chrono::milliseconds(100));
            continue;
        }

        hidraw_devinfo info = {};
        int res = ioctl(fd, HIDIOCGRAWINFO, &info);
        if (res < 0) {
            attempts++;
            this_thread::sleep_for(chrono::milliseconds(100));
            continue;
        }

        connected = true;
        attempts = 0;
        ROS_INFO("Connected to %s.", device->c_str());

        while (ros::ok()) {
            m.lock();

            writeMotors();

            uint8_t buf[64] = {};
            int rx_len = read(fd, buf, 64);
            if (rx_len < 0)
                break;

            vector<VescStatus> status;
            for (int i = 0; i < 8; i++) {
                VescStatus s = VescStatus(buf + 8 * i);
                // ROS_INFO("ID: %d RPM %d POS: %d", i, s.rpm, s.pos);
                status.push_back(s);
            }

            if (onFeedbackFunc)
                onFeedbackFunc(status);

            m.unlock();
        }
        if (connected) {
            connected = false;
            close(fd);
            m.unlock();
        }
        ROS_INFO("Disconnected.");
    }

}

bool HidMaster::send(const vector<uint8_t> &buf) {
    if (!connected)
        return false;
    m.lock();
    write(fd, &buf[0], buf.size());
    m.unlock();
    return true;
}

bool HidMaster::writeMotors() {
    vector<uint8_t> buf = {
            0x01, //report id?

            static_cast<uint8_t>(outputs[0].mode),
            static_cast<uint8_t>(outputs[0].value[0]),
            static_cast<uint8_t>(outputs[0].value[1]),
            static_cast<uint8_t>(outputs[0].value[2]),
            static_cast<uint8_t>(outputs[0].value[3]),

            static_cast<uint8_t>(outputs[1].mode),
            static_cast<uint8_t>(outputs[1].value[0]),
            static_cast<uint8_t>(outputs[1].value[1]),
            static_cast<uint8_t>(outputs[1].value[2]),
            static_cast<uint8_t>(outputs[1].value[3]),

            static_cast<uint8_t>(outputs[2].mode),
            static_cast<uint8_t>(outputs[2].value[0]),
            static_cast<uint8_t>(outputs[2].value[1]),
            static_cast<uint8_t>(outputs[2].value[2]),
            static_cast<uint8_t>(outputs[2].value[3]),

            static_cast<uint8_t>(outputs[3].mode),
            static_cast<uint8_t>(outputs[3].value[0]),
            static_cast<uint8_t>(outputs[3].value[1]),
            static_cast<uint8_t>(outputs[3].value[2]),
            static_cast<uint8_t>(outputs[3].value[3]),

            static_cast<uint8_t>(outputs[4].mode),
            static_cast<uint8_t>(outputs[4].value[0]),
            static_cast<uint8_t>(outputs[4].value[1]),
            static_cast<uint8_t>(outputs[4].value[2]),
            static_cast<uint8_t>(outputs[4].value[3]),

            static_cast<uint8_t>(outputs[5].mode),
            static_cast<uint8_t>(outputs[5].value[0]),
            static_cast<uint8_t>(outputs[5].value[1]),
            static_cast<uint8_t>(outputs[5].value[2]),
            static_cast<uint8_t>(outputs[5].value[3]),

            static_cast<uint8_t>(outputs[6].mode),
            static_cast<uint8_t>(outputs[6].value[0]),
            static_cast<uint8_t>(outputs[6].value[1]),
            static_cast<uint8_t>(outputs[6].value[2]),
            static_cast<uint8_t>(outputs[6].value[3]),

            static_cast<uint8_t>(outputs[7].mode),
            static_cast<uint8_t>(outputs[7].value[0]),
            static_cast<uint8_t>(outputs[7].value[1]),
            static_cast<uint8_t>(outputs[7].value[2]),
            static_cast<uint8_t>(outputs[7].value[3]),
    };
    write(fd, &buf[0], buf.size());
    return true;
}

bool HidMaster::writeGpio() {
    return send({
                        0x02,
                        gpio[0],
                        gpio[1],
                        gpio[2],
                        gpio[3],
                        gpio[4],
                        gpio[5],
                        gpio[6],
                        gpio[7],
                });
}

void HidMaster::onFeedback(const function<void(const vector<VescStatus> &)> &feedback) {
    onFeedbackFunc = feedback;
}

bool HidMaster::isConnected() const {
    return connected;
}