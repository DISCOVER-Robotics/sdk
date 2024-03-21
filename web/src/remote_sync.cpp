#include "websocketpp/client.hpp"
#include "httplib.h"
#include <websocketpp/config/asio_no_tls_client.hpp>
#include "nlohmann/json.hpp"

#include "airbot/modules/controller/ik_chain.hpp"
#include "airbot/modules/controller/fk_chain.hpp"
#include "airbot/modules/controller/id_chain_rne.hpp"
#include "airbot/modules/motors/protocol/motor_driver.hpp"
#include "airbot/modules/command/command_base.hpp"

#include <ncurses.h>

typedef websocketpp::client<websocketpp::config::asio_client> client;

using websocketpp::lib::bind;
using websocketpp::lib::placeholders::_1;
using websocketpp::lib::placeholders::_2;

client c;
arm::Status *instance;

class MyClientHandler
{
public:
    void on_message(websocketpp::connection_hdl, client::message_ptr msg)
    {
        if (instance == NULL)
        {
            return;
        }
        json body = json::parse(msg->get_payload());
        vector<double> data = body["data"];
        double end = body["end"];
        instance->set_target_joint_q(data);
        instance->set_target_end(end);
    }
};

void run()
{
    c.run();
}

int main(int argc, char **argv)
{
    char *urdf_path = argv[1];
    char *can = argv[2];
    char *http_remote = argv[3];
    char *ws_remote = argv[4];

    httplib::Client cli(http_remote);

    // init the master arm
    std::cout << "===================== init master arm =====================" << std::endl;
    cli.Post("/init");
    std::cout << "===================== init master arm =====================" << std::endl;

    // init local can
    std::string can_str(can);
    std::string cmd;
    cmd = "sudo ip link set " + can_str + " up type can bitrate 1000000";
    system(cmd.c_str());

    // init local arm
    instance = new arm::Status(std::make_unique<arm::ChainFKSolver>(urdf_path),
                               std::make_unique<arm::ChainIKSolver>(urdf_path),
                               std::make_unique<arm::ChainIDSolver>(urdf_path),
                               can, 3.14159265, "gripper");

    // init local websocket client
    c.set_reuse_addr(true);
    c.clear_access_channels(websocketpp::log::alevel::all);
    c.init_asio();
    MyClientHandler handler;
    c.set_message_handler(bind(&MyClientHandler::on_message, &handler, _1, _2));
    websocketpp::lib::error_code ec;
    auto con = c.get_connection(ws_remote, ec);

    if (ec)
    {
        std::cout << "Error creating connection: " << ec.message() << std::endl;
        return 1;
    }

    // start master gravity compensation
    std::cout << "===================== start master gravity compensation =====================" << std::endl;
    cli.Post("/start_gravity_compensation");
    std::cout << "===================== start master gravity compensation =====================" << std::endl;

    c.connect(con);
    std::thread(run).detach();

    initscr();
    cbreak();
    noecho();
    keypad(stdscr, TRUE);

    char ch;
    std::cout << "Press a key (Q to quit):" << std::endl;

    do
    {
        ch = getch();
        switch (ch)
        {
        case 'Q':
        case 'q':
            std::cout << "Exiting..." << std::endl;
            cli.Post("/erase");
            //c.close(con->get_handle(), websocketpp::close::status::normal, "Closing connection");
	    delete (instance);
	    instance = NULL;
            break;
        }
    } while (ch != 'Q' && ch != 'q');

    endwin();

    return 0;
}
