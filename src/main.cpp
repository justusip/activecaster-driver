#include <thread>

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include "std_msgs/UInt8.h"
#include "cheetah_driver/VescStatus.h"

#include "traffic/hid.h"
#include "models/vesc_output.h"

using namespace std;

vector<ros::Subscriber> sub = {};
array<ros::Publisher, 4> pubFeedback = {};

int main(int argc, char **argv) {
    HidMaster h;
    thread ht(&HidMaster::run, ref(h));

    ros::init(argc, argv, "cheetah");
    ros::NodeHandle n;

    for (int motorId = 0; motorId < 8; motorId++) {
        string motorDisplayId = to_string(motorId);
        sub.push_back(n.subscribe<std_msgs::Int32>(
                "cheetah/" + motorDisplayId + "/current",
                1,
                [&, motorId](const std_msgs::Int32::ConstPtr &msg) {
                    h.outputs[motorId].mode = MotorOutput::MODE_CURRENT;
                    *(int32_t *) h.outputs[motorId].value = msg->data;
                }));
        sub.push_back(n.subscribe<std_msgs::Int32>(
                "cheetah/" + motorDisplayId + "/rpm",
                1,
                [&, motorId](const std_msgs::Int32::ConstPtr &msg) {
                    h.outputs[motorId].mode = MotorOutput::MODE_RPM;
                    *(int32_t *) h.outputs[motorId].value = msg->data;
                }));

        sub.push_back(n.subscribe<std_msgs::Int32>(
                "cheetah/" + motorDisplayId + "/pos",
                1,
                [&, motorId](const std_msgs::Int32::ConstPtr &msg) {
                    h.outputs[motorId].mode = MotorOutput::MODE_POS;
                    *(int32_t *) h.outputs[motorId].value = msg->data;
                }));

        sub.push_back(n.subscribe<std_msgs::Int32>(
                "cheetah/" + motorDisplayId + "/hardbrake",
                1,
                [&, motorId](const std_msgs::Int32::ConstPtr &msg) {
                    h.outputs[motorId].mode = MotorOutput::MODE_HARDBRAKE;
                    *(int32_t *) h.outputs[motorId].value = msg->data;
                }));

        for (int gpioId = 0; gpioId < 2; gpioId++) {
            sub.push_back(n.subscribe<std_msgs::Bool>(
                    "cheetah/" + motorDisplayId + "/output/" + std::to_string(gpioId),
                    1,
                    [&, motorId, gpioId](const std_msgs::Bool::ConstPtr &msg) {
                        if (!h.isConnected())
                            ROS_WARN("Controller not yet connected.");
                        if (msg->data)
                            h.gpio[motorId] |= 1 << gpioId;
                        else
                            h.gpio[motorId] &= ~(1 << gpioId);
                        h.writeGpio();
                    }));
        }
    }

    for (int i = 0; i < 8; i++)
        pubFeedback[i] = n.advertise<cheetah_driver::VescStatus>("cheetah/" + to_string(i) + "/feedback", 1);

    h.onFeedback([&](const vector<VescStatus> &status) {
        for (int i = 0; i < status.size(); i++) {
            const VescStatus &s = status[i];
            cheetah_driver::VescStatus msg = {};
            msg.rpm = s.rpm;
            msg.pos = s.pos;
            pubFeedback[i].publish(msg);
        }
    });

    ros::AsyncSpinner spinner(2);
    spinner.start();
    ht.join();
    return 0;
}