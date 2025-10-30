#pragma once

#include <string>
#include <boost/asio.hpp>

namespace ugv::ui {

class OledViewModel {
public:
    explicit OledViewModel(boost::asio::serial_port& motionSerial);

    void printLine(int lineNum, const std::string& text);
    void restoreDefault();

private:
    boost::asio::serial_port& motionSerial;
};

}


