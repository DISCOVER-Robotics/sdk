#ifndef SERVER_HPP
#define SERVER_HPP
#include <thread>
namespace httpserver
{
    void set_urdf_path(char *path);
    void set_port(int p);
    std::thread http_server_thread();
}
#endif