#include "additional_logging.h"
#include <algorithm>

template <typename F>
static void print_array(std::ostream& os, const char* name, int n, F get) {
    os << "  " << name << "[" << n << "]: ";
    for (int i = 0; i < n; ++i) os << (i ? ", " : "") << get(i);
    os << '\n';
}

std::ostream& operator<<(std::ostream& os, const input_data_str& p) {
    // copy counts to locals (packed fields can't bind to std::min's references)
    int nObjs   = p.NrObjs;
    int nCurv   = p.AdasisCurvatureNr;
    int nLimits = p.AdasisSpeedLimitNr;
    nObjs   = std::clamp(nObjs,   0, 20);
    nCurv   = std::clamp(nCurv,   0, 100);
    nLimits = std::clamp(nLimits, 0, 10);

    os << "input_data_str {\n"
       << "  ID=" << p.ID << " Version=" << p.Version
       << " CycleNumber=" << p.CycleNumber << '\n'
       << "  ECUupTime=" << p.ECUupTime << " Status=" << p.Status << '\n'
       << "  VLgtFild=" << p.VLgtFild << " ALgtFild=" << p.ALgtFild
       << " YawRateFild=" << p.YawRateFild << " SteerWhlAg=" << p.SteerWhlAg << '\n'
       << "  VehicleLen=" << p.VehicleLen << " VehicleWidth=" << p.VehicleWidth
       << " RequestedCruisingSpeed=" << p.RequestedCruisingSpeed << '\n'
       << "  AutomationLevel=" << p.AutomationLevel
       << " CurrentLane=" << p.CurrentLane << " NrObjs=" << p.NrObjs << '\n';

    print_array(os, "ObjID",     nObjs, [&](int i) { return p.ObjID[i]; });
    print_array(os, "ObjX",      nObjs, [&](int i) { return p.ObjX[i]; });
    print_array(os, "ObjY",      nObjs, [&](int i) { return p.ObjY[i]; });
    print_array(os, "ObjLen",    nObjs, [&](int i) { return p.ObjLen[i]; });
    print_array(os, "ObjWidth",  nObjs, [&](int i) { return p.ObjWidth[i]; });
    print_array(os, "ObjVel",    nObjs, [&](int i) { return p.ObjVel[i]; });
    print_array(os, "ObjCourse", nObjs, [&](int i) { return p.ObjCourse[i]; });

    os << "  LaneWidth=" << p.LaneWidth << " LatOffsLineR=" << p.LatOffsLineR
       << " LatOffsLineL=" << p.LatOffsLineL << '\n'
       << "  LaneHeading=" << p.LaneHeading << " LaneCrvt=" << p.LaneCrvt << '\n'
       << "  AdasisCurvatureNr=" << p.AdasisCurvatureNr << '\n';

    print_array(os, "AdasisCurvatureDist",   nCurv, [&](int i) { return p.AdasisCurvatureDist[i]; });
    print_array(os, "AdasisCurvatureValues", nCurv, [&](int i) { return p.AdasisCurvatureValues[i]; });

    os << "  AdasisSpeedLimitNr=" << p.AdasisSpeedLimitNr << '\n';
    print_array(os, "AdasisSpeedLimitDist",   nLimits, [&](int i) { return p.AdasisSpeedLimitDist[i]; });
    print_array(os, "AdasisSpeedLimitValues", nLimits, [&](int i) { return p.AdasisSpeedLimitValues[i]; });

    os << "  NrTrfLights=" << p.NrTrfLights << " TrfLightDist=" << p.TrfLightDist
       << " TrfLightCurrState=" << p.TrfLightCurrState << '\n'
       << "  TrfLight next: t1=" << p.TrfLightFirstTimeToChange
       << " -> " << p.TrfLightFirstNextState
       << ", t2=" << p.TrfLightSecondTimeToChange
       << " -> " << p.TrfLightSecondNextState
       << ", t3=" << p.TrfLightThirdTimeToChange << '\n'
       << "}";
    return os;
}
