#include "vehicleTrajectory.hpp"
#include <algorithm>
#include <cmath>

VehicleTrajectory::VehicleTrajectory(double startTime, double duration, double startDistance, std::span<double, 5> m) {
    this->startTime = startTime;
    this->duration = duration;
    this->startDistance = startDistance;
    std::ranges::copy(m, this->m.begin());
}

constexpr std::array<double, 5> coeffs = {
    1.0 / 1.0,
    1.0 / 2.0,
    1.0 / 6.0,
    1.0 / 24.0,
    1.0 / 120.0
};

const double VehicleTrajectory::eval(double time, int derivates) {
    // Zeros derivates means the distance, 1 is the velocity 2 is acceleration 3 is jerk
    double timeDiff = std::min(time - startTime, duration);

    double result = 0;
    // This is needed since the distance doesn't have a constant offset
    if (derivates > 0) {
        result += m.at(derivates - 1);
    }

    for (int i = 0; i < 5 - derivates; i++) {
        result += m.at(derivates + i) * std::pow(timeDiff, i + 1) * coeffs.at(i);
    }

    return result;
}

std::ostream& operator<<(std::ostream& os, const VehicleTrajectory& trajectory) {
    os << "VehicleTrajectory(startTime=" << trajectory.startTime << ", duration=" << trajectory.duration <<  ", startDistance=" << trajectory.startDistance << ", m=[";
    for (std::size_t i = 0; i < trajectory.m.size(); ++i)
        os << (i ? ", " : "") << trajectory.m[i];
    return os << "])";
}
