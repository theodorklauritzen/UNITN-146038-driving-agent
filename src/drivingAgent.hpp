#pragma once
#include "interfaces_data_structs.h"
#include "lowLevelDrivingAgent.hpp"

enum class DrivingAgentState {
    Initial,
    Cruise,
    TrafficLight,
};

class DrivingAgent {
    double lastMessageTime = 0;
    LowLevelDrivingAgent lowLevelControl;
    DrivingAgentState state = DrivingAgentState::Initial;

    const double lookaheadDistance(const input_data_str& scenario_message);

    void enterStateCruise(const input_data_str& scenario_message);
    void enterStateTrafficLight(const input_data_str& scenario_message);

    void planManoeuvre(const input_data_str& scenario_message);

    public:

    DrivingAgent();

    output_data_str computeManoeuvre(const input_data_str& scenario_message);
};

