#include "ugv/sensors/LidarManager.hpp"
#include <thread>

using namespace boost::asio;

namespace ugv::sensors {

LidarManager::LidarManager(serial_port& lidarSerial) : lidarSerial(lidarSerial) {}

void LidarManager::setContinuousMode() {
    std::vector<uint8_t> ENABLE_CONTINUOUS = {0x01,0x06,0x00,0x3D,0x00,0x01,0xD9,0xC6};
    sendRaw(ENABLE_CONTINUOUS);
}

void LidarManager::startSingleMeasurement() {
    std::vector<uint8_t> SINGLE_MEASUREMENT = {0x01,0x06,0x00,0x03,0x00,0x01,0xB8,0x0A};
    sendRaw(SINGLE_MEASUREMENT);
}

void LidarManager::startReaderThread() {
    std::thread(&LidarManager::reader, this).detach();
}

void LidarManager::sendRaw(const std::vector<uint8_t>& cmd) {
    write(lidarSerial, buffer(cmd));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

void LidarManager::reader() {
    std::vector<uint8_t> bufferBytes(64);
    std::vector<uint8_t> leftover;
    while (true) {
        try {
            size_t bytesRead = lidarSerial.read_some(buffer(bufferBytes));
            bufferBytes.resize(bytesRead);
            bufferBytes.insert(bufferBytes.begin(), leftover.begin(), leftover.end());

            size_t i = 0;
            while (i + 9 <= bufferBytes.size()) {
                if (bufferBytes[i] == 0x01) {
                    std::vector<uint8_t> response(bufferBytes.begin() + i, bufferBytes.begin() + i + 9);
                    {
                        std::lock_guard<std::mutex> lock(queueMutex);
                        responseQueue.push_back(response);
                    }
                    i += 9;
                } else {
                    ++i;
                }
            }
            leftover.assign(bufferBytes.begin() + i, bufferBytes.end());
        } catch (...) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
}

int16_t LidarManager::getLatestDistance() {
    std::lock_guard<std::mutex> lock(queueMutex);
    if (!responseQueue.empty()) {
        auto& response = responseQueue.back();
        uint16_t distance = (response[3] << 8) | response[4];
        if (distance >= 30 && distance <= 30000) {
            return distance;
        }
    }
    return -1;
}

}


