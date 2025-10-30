#include "ugv/ui/OledViewModel.hpp"
#include <jsoncpp/json/json.h>
#include <vector>

using namespace boost::asio;

namespace ugv::ui {

OledViewModel::OledViewModel(serial_port& motionSerial) : motionSerial(motionSerial) {}

void OledViewModel::printLine(int lineNum, const std::string& text) {
    Json::Value root;
    root["T"] = 3;
    root["lineNum"] = lineNum;
    root["Text"] = text;

    Json::FastWriter writer;
    std::string cmd = writer.write(root);
    if(!cmd.empty() && cmd.back() == '\n') {
        cmd.pop_back();
    }
    std::vector<uint8_t> data(cmd.begin(), cmd.end());
    data.push_back('\n');
    write(motionSerial, buffer(data));
}

void OledViewModel::restoreDefault() {
    Json::Value root;
    root["T"] = -3;
    Json::FastWriter writer;
    std::string cmd = writer.write(root);
    if(!cmd.empty() && cmd.back() == '\n') {
        cmd.pop_back();
    }
    std::vector<uint8_t> data(cmd.begin(), cmd.end());
    data.push_back('\n');
    write(motionSerial, buffer(data));
}

}


