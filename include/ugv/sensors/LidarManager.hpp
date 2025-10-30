#pragma once

#include <boost/asio.hpp>
#include <deque>
#include <mutex>
#include <vector>

namespace ugv::sensors {

class LidarManager {
public:
    explicit LidarManager(boost::asio::serial_port& lidarSerial);

    void startReaderThread();
    void setContinuousMode();
    void startSingleMeasurement();
    int16_t getLatestDistance();

private:
    void reader();
    void sendRaw(const std::vector<uint8_t>& cmd);

    boost::asio::serial_port& lidarSerial;
    std::mutex queueMutex;
    std::deque<std::vector<uint8_t>> responseQueue;
};

}


