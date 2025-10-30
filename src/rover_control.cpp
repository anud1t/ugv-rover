#include <iostream>
#include <boost/asio.hpp>
#include <thread>
#include <vector>
#include <chrono>
#include <string>
#include <cmath>
#include <cstdlib>
#include <unistd.h>
#include <fstream>

#include "ugv/ui/OledViewModel.hpp"
#include "ugv/input/TerminalInput.hpp"
#include "ugv/sensors/LidarManager.hpp"
#include "ugv/config/Config.hpp"

using namespace std;
using namespace boost::asio;

// -------------------------------------------------------------------------
// MotionState & IMUData (same as your autopilot snippet)
// -------------------------------------------------------------------------
enum MotionState {
    MOTION_STOPPED,
    MOTION_FORWARD,
    MOTION_REVERSE,
    MOTION_TURN_LEFT,
    MOTION_TURN_RIGHT,
    MOTION_UNKNOWN
};

struct IMUData {
    double r;
    double p;
    double y;
    double ax, ay, az;
    double gx, gy, gz;
    double mx, my, mz;
    double temp;
    bool valid;
};

// -------------------------------------------------------------------------
// Global objects
// -------------------------------------------------------------------------
static io_service io;
static serial_port lidarSerial(io);    // LIDAR port
static serial_port motionSerial(io);   // Motion port

static string LIDAR_PORT  = "/dev/serial0";
static unsigned int LIDAR_BAUD  = 115200;

static string MOTION_PORT = "/dev/serial0";
static unsigned int MOTION_BAUD = 115200;

// Distances for obstacle logic
static uint16_t SAFE_DISTANCE = 250;   // mm
static uint16_t TURN_DISTANCE = 500;   // mm

// Timing for stuck detection
constexpr auto STUCK_TIME_LIMIT = std::chrono::seconds(3);
constexpr auto RECOVERY_TIME    = std::chrono::milliseconds(500);

static MotionState currentMotion = MOTION_UNKNOWN;

// Movement Commands (JSON). Adjust to your firmware’s protocol.
static const string FORWARD_CMD = R"({"T":1,"L":0.20,"R":0.20})";
static const string STOP_CMD    = R"({"T":1,"L":0.0,"R":0.0})";
static const string REVERSE_CMD = R"({"T":1,"L":-0.12,"R":-0.12})";
static const string RIGHT_CMD   = R"({"T":1,"L":0.25,"R":-0.25})";
static const string LEFT_CMD    = R"({"T":1,"L":-0.25,"R":0.25})";

// Example of a "boost" or high-speed forward command
static const string BOOST_CMD   = R"({"T":1,"L":0.30,"R":0.30})";

// -------------------------------------------------------------------------
// Utility: Initialize serial port
// -------------------------------------------------------------------------
void initSerial(serial_port& sp, const string& port, unsigned int baudRate) {
    try {
        sp.open(port);
        sp.set_option(serial_port_base::baud_rate(baudRate));
        sp.set_option(serial_port_base::character_size(8));
        sp.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
        sp.set_option(serial_port_base::parity(serial_port_base::parity::none));
        sp.set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));

        cout << "Serial connection established on " 
             << port << " at " << baudRate << " baud.\n";
    } catch (boost::system::system_error& e) {
        cerr << "Error opening serial port (" << port << "): " << e.what() << endl;
        exit(1);
    }
}

// -------------------------------------------------------------------------
// Send motion commands over motionSerial
// -------------------------------------------------------------------------
void sendMovementCommand(const string& jsonCmd, MotionState newState) {
    cout << "Sending motion command: " << jsonCmd << endl;
    vector<uint8_t> data(jsonCmd.begin(), jsonCmd.end());
    data.push_back('\n');  // if line-delimited
    try {
        boost::asio::write(motionSerial, boost::asio::buffer(data));
        currentMotion = newState;
    } catch (const boost::system::system_error& e) {
        cerr << "Error writing to motion serial: " << e.what() << endl;
    }
}

// (OLED handled by ugv::ui::OledViewModel)

// -------------------------------------------------------------------------
// Enhanced auto-pilot logic (distance + tilt + simple stuck detection)
// -------------------------------------------------------------------------
void runAutoPilot(ugv::sensors::LidarManager& lidarMgr, ugv::ui::OledViewModel& oled) {
    static auto lastProgressTime = std::chrono::steady_clock::now();
    static int16_t lastDistance = -1;

    // 1. Get current distance
    int16_t distance = lidarMgr.getLatestDistance();

    // 2. Check IMU tilt (placeholder)
    IMUData imu;
    imu.valid = true;
    imu.r = 0.0;
    imu.p = 0.0;
    if (imu.valid && (fabs(imu.r) > 30.0 || fabs(imu.p) > 30.0)) {
        // Tilt => stop & recover
        oled.printLine(1, "Excess Tilt!");
        sendMovementCommand(STOP_CMD, MOTION_STOPPED);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        // Simple recovery: reverse + turn left + forward
        sendMovementCommand(REVERSE_CMD, MOTION_REVERSE);
        std::this_thread::sleep_for(RECOVERY_TIME);
        sendMovementCommand(STOP_CMD, MOTION_STOPPED);
        sendMovementCommand(LEFT_CMD, MOTION_TURN_LEFT);
        std::this_thread::sleep_for(RECOVERY_TIME);
        sendMovementCommand(STOP_CMD, MOTION_STOPPED);
        sendMovementCommand(FORWARD_CMD, MOTION_FORWARD);
        return;
    }

    cout << "[AutoPilot] Distance: " << distance << " mm" << endl;
    oled.printLine(0, "AutoPilot");
    if(distance == -1) {
        oled.printLine(2, "Dist: -1");
        return;
    } else {
        oled.printLine(2, string("Dist:") + to_string(distance));
    }

    // Stuck check
    if (distance == lastDistance) {
        auto now = std::chrono::steady_clock::now();
        if (now - lastProgressTime > STUCK_TIME_LIMIT) {
            oled.printLine(1, "Stuck! Reco...");
            sendMovementCommand(STOP_CMD, MOTION_STOPPED);
            // Recovery
            sendMovementCommand(REVERSE_CMD, MOTION_REVERSE);
            std::this_thread::sleep_for(RECOVERY_TIME);
            sendMovementCommand(STOP_CMD, MOTION_STOPPED);
            sendMovementCommand(LEFT_CMD, MOTION_TURN_LEFT);
            std::this_thread::sleep_for(RECOVERY_TIME);
            sendMovementCommand(STOP_CMD, MOTION_STOPPED);
            sendMovementCommand(FORWARD_CMD, MOTION_FORWARD);
            lastProgressTime = std::chrono::steady_clock::now();
        }
    } else {
        lastProgressTime = std::chrono::steady_clock::now();
        lastDistance = distance;
    }

    // Obstacle avoidance
    if (distance < SAFE_DISTANCE) {
        oled.printLine(1, "Obstacle!");
        sendMovementCommand(STOP_CMD, MOTION_STOPPED);
        // Recovery
        sendMovementCommand(REVERSE_CMD, MOTION_REVERSE);
        std::this_thread::sleep_for(RECOVERY_TIME);
        sendMovementCommand(STOP_CMD, MOTION_STOPPED);
        sendMovementCommand(LEFT_CMD, MOTION_TURN_LEFT);
        std::this_thread::sleep_for(RECOVERY_TIME);
        sendMovementCommand(STOP_CMD, MOTION_STOPPED);
        sendMovementCommand(FORWARD_CMD, MOTION_FORWARD);
    } 
    else if (distance < TURN_DISTANCE) {
        // Turn a bit
        if (currentMotion == MOTION_FORWARD || currentMotion == MOTION_STOPPED) {
            oled.printLine(1, "Turn Left");
            sendMovementCommand(LEFT_CMD, MOTION_TURN_LEFT);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            sendMovementCommand(STOP_CMD, MOTION_STOPPED);

            int16_t distCheck = lidarMgr.getLatestDistance();
            if (distCheck > SAFE_DISTANCE) {
                oled.printLine(1, "Forward");
                sendMovementCommand(FORWARD_CMD, MOTION_FORWARD);
            } else {
                sendMovementCommand(STOP_CMD, MOTION_STOPPED);
            }
        }
    } else {
        // Clear => forward
        if (currentMotion != MOTION_FORWARD) {
            oled.printLine(1, "Forward");
            sendMovementCommand(FORWARD_CMD, MOTION_FORWARD);
        } else {
            // Re-send forward to maintain
            sendMovementCommand(FORWARD_CMD, MOTION_FORWARD);
        }
    }
}

// -------------------------------------------------------------------------
// Main
// -------------------------------------------------------------------------
int main() {
    // 0. Load config
    auto cfg = ugv::config::loadFromJsonFile("config/rover.json");
    LIDAR_PORT = cfg.serial.lidarPort;
    LIDAR_BAUD = cfg.serial.lidarBaud;
    MOTION_PORT = cfg.serial.motionPort;
    MOTION_BAUD = cfg.serial.motionBaud;
    SAFE_DISTANCE = static_cast<uint16_t>(cfg.motion.safeDistanceMm);
    TURN_DISTANCE = static_cast<uint16_t>(cfg.motion.turnDistanceMm);

    // 1. Init serial
    initSerial(lidarSerial, LIDAR_PORT, LIDAR_BAUD);
    initSerial(motionSerial, MOTION_PORT, MOTION_BAUD);

    ugv::ui::OledViewModel oled(motionSerial);
    ugv::sensors::LidarManager lidarMgr(lidarSerial);
    ugv::input::TerminalInput terminal;

    // OLED welcome
    oled.printLine(0, "Manual Mode");
    oled.printLine(1, "Idle");
    oled.printLine(2, "Dist: ---");
    oled.printLine(3, " ");

    // 2. LIDAR config
    cout << "Setting continuous mode...\n";
    lidarMgr.setContinuousMode();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    cout << "Starting measurement...\n";
    lidarMgr.startSingleMeasurement();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // 3. Start LIDAR reading thread
    lidarMgr.startReaderThread();

    // 4. Terminal raw mode for keyboard
    terminal.enableRaw();

    // 5. Variables for auto-pilot toggle
    bool autoPilotEnabled = false;

    // Info
    cout << "Keyboard controls:\n"
         << "  Arrow Up/Down/Left/Right => manual movement\n"
         << "  1 => slow speed\n"
         << "  2 => normal speed\n"
         << "  3 => fast speed\n"
         << "  b => show battery\n"
         << "  Space => stop\n"
         << "  x => boost forward\n"
         << "  p => toggle auto-pilot ON/OFF\n"
         << "  q => quit\n\n";

    double speedFactor = 0.20; // default speed

    // 6. Main loop
    bool running = true;

    while(running) {
        // a) If auto-pilot is enabled, run that logic
        if(autoPilotEnabled) {
            runAutoPilot(lidarMgr, oled);
        }

        // b) Check if a key was pressed (non-blocking)
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(STDIN_FILENO, &fds);

        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 0;

        int ret = select(STDIN_FILENO+1, &fds, NULL, NULL, &tv);
        if(ret > 0 && FD_ISSET(STDIN_FILENO, &fds)) {
            char c;
            if(read(STDIN_FILENO, &c, 1) < 0) {
                perror("read()");
                break;
            }

            // If ANY of these keys pressed, we might disable auto-pilot
            // (unless it's specifically the 'p' to toggle ON).
            bool manualKey = false;

            if(c == 'q') {
                // Quit
                running = false;
                break;
            }
            else if(c == ' ') {
                // Stop
                oled.printLine(1, "Stopped");
                sendMovementCommand(STOP_CMD, MOTION_STOPPED);
                manualKey = true;
            }
            else if(c == 'x') {
                // Boost
                oled.printLine(1, "Boost!");
                sendMovementCommand(BOOST_CMD, MOTION_FORWARD);
                manualKey = true;
            }
            else if(c == 'p') {
                // Toggle autopilot
                autoPilotEnabled = !autoPilotEnabled;
                cout << "[INFO] AutoPilot toggled: " 
                     << (autoPilotEnabled ? "ON" : "OFF") << endl;
                oled.printLine(0, autoPilotEnabled ? "AutoPilot ON" : "Manual Mode");
                if(!autoPilotEnabled) {
                    oled.printLine(1, "Idle");
                }
            }
            else if(c == '1') {
                speedFactor = 0.15;
                cout << "[INFO] Speed set to SLOW (0.15)\n";
                oled.printLine(1, "Speed: SLOW");
                manualKey = true;
            }
            else if(c == '2') {
                speedFactor = 0.20;
                cout << "[INFO] Speed set to NORMAL (0.20)\n";
                oled.printLine(1, "Speed: NORMAL");
                manualKey = true;
            }
            else if(c == '3') {
                speedFactor = 0.25;
                cout << "[INFO] Speed set to FAST (0.25)\n";
                oled.printLine(1, "Speed: FAST");
                manualKey = true;
            }
            else if(c == 'b') {
                float battery = 12.0f; // Placeholder; wire in real read if available
                char buf[32];
                snprintf(buf, sizeof(buf), "Batt: %.2fV", battery);
                oled.printLine(3, buf);
            }
            else if(c == '\033') {
                // Possible arrow key
                char seq[2];
                if(read(STDIN_FILENO, seq, 2) < 2) {
                    continue;
                }
                if(seq[0] == '[') {
                    switch(seq[1]) {
                        case 'A': // Up
                            oled.printLine(1, "Forward");
                            {
                                string cmd = string("{") + "\"T\":1,\"L\":" + to_string(speedFactor) + ",\"R\":" + to_string(speedFactor) + "}";
                                sendMovementCommand(cmd, MOTION_FORWARD);
                            }
                            manualKey = true;
                            break;
                        case 'B': // Down
                            oled.printLine(1, "Reverse");
                            {
                                string cmd = string("{") + "\"T\":1,\"L\":" + to_string(-speedFactor) + ",\"R\":" + to_string(-speedFactor) + "}";
                                sendMovementCommand(cmd, MOTION_REVERSE);
                            }
                            manualKey = true;
                            break;
                        case 'C': // Right
                            oled.printLine(1, "Turning Right");
                            {
                                string cmd = string("{") + "\"T\":1,\"L\":" + to_string(speedFactor) + ",\"R\":" + to_string(-speedFactor) + "}";
                                sendMovementCommand(cmd, MOTION_TURN_RIGHT);
                            }
                            manualKey = true;
                            break;
                        case 'D': // Left
                            oled.printLine(1, "Turning Left");
                            {
                                string cmd = string("{") + "\"T\":1,\"L\":" + to_string(-speedFactor) + ",\"R\":" + to_string(speedFactor) + "}";
                                sendMovementCommand(cmd, MOTION_TURN_LEFT);
                            }
                            manualKey = true;
                            break;
                    }
                }
            }

            // If a "manual key" was pressed, we override auto-pilot
            if(manualKey && autoPilotEnabled) {
                autoPilotEnabled = false;
                cout << "[INFO] AutoPilot disabled by manual key.\n";
                oled.printLine(0, "Manual Mode");
            }
        }

        // c) Sleep a bit
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    // Cleanup
    oled.restoreDefault();
    motionSerial.close();
    lidarSerial.close();

    return 0;
}


