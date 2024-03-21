#include "server.hpp"
#include "controller.hpp"
#include "websocket.hpp"
#include "argparse.hpp"

int main(int argc, char **argv)
{
    // // argparse
    // argparse::ArgumentParser program("airbot_play");
    // program.add_argument("-u", "--urdf")
    //     .required()
    //     .default_value(std::getenv("URDF_PATH")
    //                        ? std::string(std::getenv("URDF_PATH"))
    //                        : std::string())
    //     .help(
    //         "URDF file for describing the arm. If not provided, read from the "
    //         "environment variable URDF_PATH");
    // program.add_argument("-m", "--master")
    //     .required()
    //     .default_value("can0")
    //     .help("Can device interface of the master arm. Default: can0");
    // program.add_argument("-p", "--port")
    //     .default_value<int>(8888)
    //     .help("Listening port of HTTP");
    // program.add_argument("-w", "--ws-port")
    //     .default_value<int>(9999)
    //     .help("Listening port of Websocket");

    // try
    // {
    //     program.parse_args(argc, argv);
    // }
    // catch (const std::exception &err)
    // {
    //     std::cerr << err.what() << std::endl;
    //     std::cerr << program;
    //     return 1;
    // // }
    // std::string urdf_path = std::string(argv[1]);
    // std::string master_can = std::string(argv[2]);
    int http_port = std::atoi(argv[3]);
    int ws_port = std::atoi(argv[4]);

    httpserver::set_urdf_path(argv[1]);
    controller::set_can_interface(argv[2]);
    httpserver::set_port(http_port);
    websocket::set_port(ws_port);

    websocket::status_websocket_thread().detach();
    httpserver::http_server_thread().join();
}
