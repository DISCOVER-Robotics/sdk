#ifndef WEBSOCKET_HPP
#define WEBSOCKET_HPP
#include <thread>

namespace websocket
{
    void set_port(int p);
    std::thread status_websocket_thread();
}
#endif