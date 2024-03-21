#include <thread>

#include "httplib.h"
#include "server.hpp"
#include "controller.hpp"

namespace httpserver
{
    int port = 8888;
    void set_port(int p)
    {
        port = p;
    }
    void start_serve_http()
    {
        // HTTP
        httplib::Server svr;

        svr.set_default_headers({{"Access-Control-Expose-Headers", "Content-Length, Access-Control-Allow-Origin, Access-Control-Allow-Headers, Content-Type"},
                                 {"Access-Control-Allow-Origin", "*"},
                                 {"Access-Control-Allow-Headers", "Content-Type,AccessToken,X-CSRF-Token, Authorization, Token, x-token"},
                                 {"Access-Control-Allow-Methods", "POST, GET, OPTIONS, DELETE, PATCH, PUT"},
                                 {"Access-Control-Allow-Credentials", "true"}});
        svr.Options(R"(.*)", [](const httplib::Request &req, httplib::Response &res)
                    { res.status = 200; });

        svr.Post("/init", controller::init);
        svr.Post("/erase", controller::erase);

        svr.Get("/get_target_pose", controller::get_target_pose);
        svr.Get("/get_target_joint_q", controller::get_target_joint_q);
        svr.Get("/get_target_joint_v", controller::get_target_joint_v);
        svr.Get("/get_target_joint_t", controller::get_target_joint_t);
        svr.Get("/get_target_translation", controller::get_target_translation);
        svr.Get("/get_target_rotation", controller::get_target_rotation);
        svr.Get("/get_current_pose", controller::get_current_pose);
        svr.Get("/get_current_joint_q", controller::get_current_joint_q);
        svr.Get("/get_current_joint_v", controller::get_current_joint_v);
        svr.Get("/get_current_joint_t", controller::get_current_joint_t);
        svr.Get("/get_current_translation", controller::get_current_translation);
        svr.Get("/get_current_rotation", controller::get_current_rotation);
        svr.Get("/get_current_end", controller::get_current_end);

        svr.Post("/set_target_pose", controller::set_target_pose);
        svr.Post("/set_target_end", controller::set_target_end);
        svr.Post("/set_target_joint_q", controller::set_target_joint_q);
        svr.Post("/set_target_joint_v", controller::set_target_joint_v);
        svr.Post("/set_target_joint_q_v", controller::set_target_joint_q_v);
        svr.Post("/set_target_joint_t", controller::set_target_joint_t);
        svr.Post("/set_target_translation", controller::set_target_translation);
        svr.Post("/set_target_rotation", controller::set_target_rotation);
        svr.Post("/add_target_joint_q", controller::add_target_joint_q);
        svr.Post("/add_target_joint_v", controller::add_target_joint_v);
        svr.Post("/add_target_translation", controller::add_target_translation);
        svr.Post("/add_target_relative_rotation", controller::add_target_relative_rotation);

        svr.Post("/start_gravity_compensation", controller::start_gravity_compensation);

        svr.Post("/record_start", controller::record_start);
        svr.Post("/record_stop", controller::record_stop);
        svr.Post("/record_replay", controller::record_replay);

        svr.listen("0.0.0.0", port);
    }

    std::thread http_server_thread()
    {
        return std::thread(start_serve_http);
    }

    void set_urdf_path(char *path)
    {
        controller::set_urdf_path(path);
    }
}