
#pragma once

#include <array>
#include <span>
#include <iostream>

class VehicleTrajectory {
    std::array<double, 5> m;
    double startTime;
    double duration;

    const double eval(double time, int derivates);

    public:
    VehicleTrajectory(double startTime, double duration, const std::span<double, 5> m);
    VehicleTrajectory() {};

    const double sEval(double time) { return eval(time, 0); };
    const double vEval(double time) { return eval(time, 1); };
    const double aEval(double time) { return eval(time, 2); };
    const double jEval(double time) { return eval(time, 3); };

    const double getEndTime() { return startTime + duration; }

    friend std::ostream& operator<<(std::ostream& os, const VehicleTrajectory& trajectory);
};
