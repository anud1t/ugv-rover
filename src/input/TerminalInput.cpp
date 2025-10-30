#include "ugv/input/TerminalInput.hpp"
#include <unistd.h>

namespace ugv::input {

TerminalInput::TerminalInput() = default;
TerminalInput::~TerminalInput() { disableRaw(); }

void TerminalInput::enableRaw() {
    if (enabled) return;
    struct termios newt{};
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    newt.c_cc[VMIN] = 1;
    newt.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    enabled = true;
}

void TerminalInput::disableRaw() {
    if (!enabled) return;
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    enabled = false;
}

}


