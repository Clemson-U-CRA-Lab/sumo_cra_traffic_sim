#pragma once

#include <string>

namespace v2x_controller_live {

struct V2XStatusPayload {
    bool valid = false;

    int seq = -1;
    double sendTime = 0.0;
    double receiveTime = 0.0;

    double x = 0.0;
    double y = 0.0;
    double speed = 0.0;
    double accel = 0.0;
};

void updateLatestStatus(const V2XStatusPayload& payload);
bool hasLatestStatus();
V2XStatusPayload getLatestStatus();

}
