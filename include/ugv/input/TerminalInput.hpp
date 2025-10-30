#pragma once

#include <termios.h>

namespace ugv::input {

class TerminalInput {
public:
    TerminalInput();
    ~TerminalInput();

    void enableRaw();
    void disableRaw();

private:
    struct termios oldt{};
    bool enabled{false};
};

}


