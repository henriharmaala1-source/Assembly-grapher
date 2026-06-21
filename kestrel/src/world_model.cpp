#include "world_model.hpp"

#include <sstream>

const char* behavior_name(Behavior b) {
    switch (b) {
        case Behavior::IDLE:     return "IDLE";
        case Behavior::NAVIGATE: return "NAVIGATE";
        case Behavior::TRACK:    return "TRACK";
        case Behavior::SEARCH:   return "SEARCH";
        case Behavior::EVADE:    return "EVADE";
        case Behavior::RTL:      return "RTL";
        case Behavior::HOLD:     return "HOLD";
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

    os << " det=[";
    for (size_t i = 0; i < detections.size(); ++i) {
        if (i) os << ",";
        os << detections[i].label << ":" << int(detections[i].confidence * 100) << "%";
    }
    os << "]";

    os << " bat=" << int(vehBattery * 100) << "% alt=" << int(vehAltM) << "m"
       << " fps=" << int(fps);
    return os.str();
}
