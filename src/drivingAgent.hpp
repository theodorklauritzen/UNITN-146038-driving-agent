#pragma once
#include "interfaces_data_structs.h"
#include "lowLevelDrivingAgent.hpp"

enum class DrivingAgentState {
    Stop,
    Cruise,
    TrafficLight,
};

class DrivingAgent {
    double lastMessageTime = 0;
    LowLevelDrivingAgent lowLevelControl;
    DrivingAgentState state = DrivingAgentState::Stop;

    const double lookaheadDistance(const input_data_str& scenario_message);
    
    void planManoeuvre(const double time, const input_data_str& scenario_message);

    public:

    DrivingAgent();

    output_data_str computeManoeuvre(const double time, const input_data_str& scenario_message);
};

