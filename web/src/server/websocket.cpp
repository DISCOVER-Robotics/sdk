#include <iostream>
#include <utility>
#include <atomic>
#include <thread>

#include "websocketpp/config/asio_no_tls.hpp"
#include "websocketpp/server.hpp"
#include "nlohmann/json.hpp"

#include "websocket.hpp"
#include "service.hpp"

typedef websocketpp::server<websocketpp::config::asio> websocketsvr;
typedef websocketsvr::message_ptr message_ptr;

using websocketpp::lib::bind;
using websocketpp::lib::placeholders::_1;
using websocketpp::lib::placeholders::_2;

namespace websocket
{
  int port = 9999;
  void set_port(int p)
  {
    port = p;
  }
  // status resp data definition
  nlohmann::json status_resp_data;

  // status functions
  void status_ticker(websocketsvr *server, websocketpp::connection_hdl hdl)
  {
    auto con = server->get_con_from_hdl(hdl);
    while (con->get_state() == websocketpp::session::state::open)
    {
      vector<double> data = service::get_current_joint_q();
      if (data.size() == 0)
      {
        continue;
      }
      double end = service::get_current_end();

      status_resp_data["data"] = data;
      status_resp_data["end"] = end;
      server->send(hdl, status_resp_data.dump(), websocketpp::frame::opcode::text);
      this_thread::sleep_for(chrono::milliseconds(2));
    }
  }
  void OnStatusOpen(websocketsvr *server, websocketpp::connection_hdl hdl)
  {
    std::cout << "have client connected" << std::endl;
    std::thread t(status_ticker, server, hdl);
    t.detach();
  }

  void OnStatusClose(websocketsvr *server, websocketpp::connection_hdl hdl)
  {
    std::cout << "have client closed" << std::endl;
  }

  // socket entry
  void start_status_websocket()
  {
    websocketsvr server;
    server.set_reuse_addr(true);
    server.clear_access_channels(websocketpp::log::alevel::all);
    server.init_asio();

    server.set_open_handler(bind(&OnStatusOpen, &server, ::_1));
    server.set_close_handler(bind(&OnStatusClose, &server, _1));

    server.listen(port);
    server.start_accept();
    server.run();
  }

  // thread
  std::thread status_websocket_thread()
  {
    return std::thread(start_status_websocket);
  }
}