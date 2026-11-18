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


    constexpr static double safetyDistance = 2;

    const double lookaheadDistance(const input_data_str& scenario_message);

    void planManoeuvre(const input_data_str& scenario_message);

    // Cruise state members
    double cruisingSpeed = 90 / 3.6;
    int nextSpeedLimitHandle = 0;
    void changeSpeed(double speed, double distance, const input_data_str& scenario_message);
    void enterStateCruise(const input_data_str& scenario_message);
    void loopStateCruise(const input_data_str& scenario_message);

    // Traffic light members
    void enterStateTrafficLight(const input_data_str& scenario_message);

    public:

    DrivingAgent();

    output_data_str computeManoeuvre(const input_data_str& scenario_message);
};

