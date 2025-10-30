#pragma once

#include <string>

namespace ugv::config {

struct SerialConfig {
    std::string lidarPort;
    unsigned int lidarBaud{115200};
    std::string motionPort;
    unsigned int motionBaud{115200};
};

struct MotionConfig {
    unsigned int safeDistanceMm{250};
    unsigned int turnDistanceMm{500};
    double speedSlow{0.15};
    double speedNormal{0.20};
    double speedFast{0.25};
    double boostSpeed{0.30};
};

struct Config {
    SerialConfig serial;
    MotionConfig motion;
};

// Loads config from path. If file missing or invalid, returns defaults.
Config loadFromJsonFile(const std::string& path);

}


