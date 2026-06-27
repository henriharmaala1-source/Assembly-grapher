#include "world_model.hpp"

#include <sstream>

const char* behavior_name(Behavior b) {
    switch (b) {
        case Behavior::MANUAL:      return "MANUAL";
        case Behavior::IDLE:        return "IDLE";
        case Behavior::NAVIGATE:    return "NAVIGATE";
        case Behavior::ROAD_FOLLOW: return "ROAD_FOLLOW";
        case Behavior::TRACK:       return "TRACK";
        case Behavior::SEARCH:      return "SEARCH";
        case Behavior::EVADE:       return "EVADE";
        case Behavior::HOLD:        return "HOLD";
        case Behavior::RTL:         return "RTL";
    }
    return "?";
}

std::string WorldState::brief() const {
    std::ostringstream os;
    os << "beh=" << behavior_name(behavior);

    os << " trk=";
    if (!targetValid) {
        os << "none";
    } else {
        os << (targetLocked ? (targetCoast ? "COAST" : "LOCK") : "LOST")
           << "(c" << int(targetConf * 100) << "%,age" << targetAge
           << ",lost" << targetLosses << ")";
    }

    os << " nav=";
    if (!corridorValid) {
        os << "none";
    } else {
        os << (corridorDecisive ? "TRAVERSE" : "SCAN")
           << "(open" << int(corridorOpen * 100) << "%,hdg"
           << int(corridorHeading.x) << "," << int(corridorHeading.y) << ")";
    }

    os << " road=";
    if (!roadValid) os << "none";
    else os << "(off" << int(roadOffset * 100) << ",c" << int(roadConf * 100) << "%)";

    os << " det=[";
    for (size_t i = 0; i < detections.size(); ++i) {
        if (i) os << ",";
        os << detections[i].label << ":" << int(detections[i].confidence * 100) << "%";
    }
    os << "]";

    if (estValid) {
        os << " est(" << int(estPe) << "," << int(estPn) << "," << int(estPu) << "m"
           << ",v" << int(estSpeed) << ",e" << int(estEphM);
        if (estGpsDenied) os << ",DENIED";
        if (estFeedingFc) os << ",feed";
        os << ")";
    }

    if (control.valid)
        os << " ctl(r" << int(control.roll * 100) << ",p" << int(control.pitch * 100)
           << ",y" << int(control.yaw * 100) << ",t" << int(control.throttle * 100)
           << (controlActive ? ",LIVE)" : ",dry)");

    os << " bat=" << int(vehBattery * 100) << "% alt=" << int(vehAltM) << "m"
       << (vehLink ? " fc=up" : " fc=down")
       << " fps=" << int(fps);
    return os.str();
}
