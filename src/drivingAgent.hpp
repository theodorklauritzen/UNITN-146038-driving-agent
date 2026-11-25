#pragma once
#include "interfaces_data_structs.h"
#include "lowLevelDrivingAgent.hpp"

enum class DrivingAgentState {
    Initial,
    Cruise,
    TrafficLight,
};

struct RecommendedSpeedReturn {
    double speed;
    double distance;
};

double removeNear0Value(double value);

class DrivingAgent {
    double lastMessageTime = 0;
    LowLevelDrivingAgent lowLevelControl;
    DrivingAgentState state = DrivingAgentState::Initial;


    constexpr static double safetyDistance = 2;

    const double lookaheadDistance(const input_data_str& scenario_message);

    void planManoeuvre(const input_data_str& scenario_message);
    double lastStateChange = 0;

    int nextSpeedLimitHandle = 0;
    int nextPassingSpeedLimit = 0;
    void perceptSpeedLimit(const input_data_str& scenario_message);
    RecommendedSpeedReturn getRecommendedUpperSpeed(const input_data_str& scenario_message);

    // Cruise state members
    double cruisingSpeed = 0;
    void changeSpeed(double speed, double distance, const input_data_str& scenario_message);
    void enterStateCruise(const input_data_str& scenario_message);
    void loopStateCruise(const input_data_str& scenario_message);

    // Traffic light members
    double nextTrafficLightPosition = 0;
    constexpr static double lengthOfIntersection = 10;
    bool trafficLightIntendToStop = false;
    void enterStateTrafficLight(const input_data_str& scenario_message);
    void loopStateTrafficLight(const input_data_str& scenario_message);

    public:

    DrivingAgent();

    output_data_str computeManoeuvre(const input_data_str& scenario_message);
};

