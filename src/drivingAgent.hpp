#pragma once
#include "interfaces_data_structs.h"
#include "lowLevelDrivingAgent.hpp"

class DrivingAgent {
    double lastMessageTime = 0;
    LowLevelDrivingAgent lowLevelControl;

    public:

    DrivingAgent();

    output_data_str computeManoeuvre(const double time, const input_data_str& scenario_message);

};

