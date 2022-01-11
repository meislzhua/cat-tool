#include <Ticker.h>
bool Ticker::active() {
    return !!this->_timer;
}