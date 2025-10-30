#include "ugv/config/Config.hpp"
#include <jsoncpp/json/json.h>
#include <fstream>

namespace ugv::config {

static unsigned int asUInt(const Json::Value& v, unsigned int def) {
    return v.isUInt() ? v.asUInt() : def;
}

static double asDouble(const Json::Value& v, double def) {
    return v.isDouble() || v.isInt() ? v.asDouble() : def;
}

static std::string asString(const Json::Value& v, const std::string& def) {
    return v.isString() ? v.asString() : def;
}

Config loadFromJsonFile(const std::string& path) {
    Config cfg;
    cfg.serial.lidarPort = "/dev/serial0";
    cfg.serial.motionPort = "/dev/serial0";

    std::ifstream in(path);
    if(!in.good()) {
        return cfg; // defaults
    }

    Json::Value root;
    in >> root;

    const auto& s = root["serial"];
    if(!s.isNull()) {
        cfg.serial.lidarPort = asString(s["lidar_port"], cfg.serial.lidarPort);
        cfg.serial.lidarBaud  = asUInt(s["lidar_baud"], cfg.serial.lidarBaud);
        cfg.serial.motionPort = asString(s["motion_port"], cfg.serial.motionPort);
        cfg.serial.motionBaud = asUInt(s["motion_baud"], cfg.serial.motionBaud);
    }

    const auto& m = root["motion"];
    if(!m.isNull()) {
        cfg.motion.safeDistanceMm = asUInt(m["safe_distance_mm"], cfg.motion.safeDistanceMm);
        cfg.motion.turnDistanceMm = asUInt(m["turn_distance_mm"], cfg.motion.turnDistanceMm);
        cfg.motion.speedSlow      = asDouble(m["speed_slow"], cfg.motion.speedSlow);
        cfg.motion.speedNormal    = asDouble(m["speed_normal"], cfg.motion.speedNormal);
        cfg.motion.speedFast      = asDouble(m["speed_fast"], cfg.motion.speedFast);
        cfg.motion.boostSpeed     = asDouble(m["boost_speed"], cfg.motion.boostSpeed);
    }

    return cfg;
}

}


