#include "httplib.h"
#include "service.hpp"
#include "nlohmann/json.hpp"

using json = nlohmann::json;

namespace controller
{
    const std::string content_type_json = "application/json";
    char *urdf_path = "";
    char *can_interface = "can0";

    void set_urdf_path(char *path)
    {
        urdf_path = path;
    }
    void set_can_interface(char *interface)
    {
        can_interface = interface;
    }

    void init(const httplib::Request &req, httplib::Response &resp)
    {
        service::init(urdf_path, can_interface);
        resp.set_content("OK", content_type_json);
    }

    void erase(const httplib::Request &req, httplib::Response &resp)
    {
        service::erase();

        resp.set_content("OK", content_type_json);
    }

    void get_target_pose(const httplib::Request &req, httplib::Response &resp)
    {
        vector<vector<double>> data = service::get_target_pose();
        json result;
        result["data"] = data;

        resp.set_content(result.dump(), content_type_json);
    }

    void get_target_joint_q(const httplib::Request &req, httplib::Response &resp)
    {
        vector<double> data = service::get_target_joint_q();
        json result;
        result["data"] = data;

        resp.set_content(result.dump(), content_type_json);
    }

    void get_target_joint_v(const httplib::Request &req, httplib::Response &resp)
    {
        vector<double> data = service::get_target_joint_v();
        json result;
        result["data"] = data;

        resp.set_content(result.dump(), content_type_json);
    }

    void get_target_joint_t(const httplib::Request &req, httplib::Response &resp)
    {
        vector<double> data = service::get_target_joint_t();
        json result;
        result["data"] = data;

        resp.set_content(result.dump(), content_type_json);
    }

    void get_target_translation(const httplib::Request &req, httplib::Response &resp)
    {
        vector<double> data = service::get_target_translation();
        json result;
        result["data"] = data;

        resp.set_content(result.dump(), content_type_json);
    }

    void get_target_rotation(const httplib::Request &req, httplib::Response &resp)
    {
        vector<double> data = service::get_target_rotation();
        json result;
        result["data"] = data;

        resp.set_content(result.dump(), content_type_json);
    }

    void get_current_pose(const httplib::Request &req, httplib::Response &resp)
    {
        vector<vector<double>> data = service::get_current_pose();
        json result;
        result["data"] = data;

        resp.set_content(result.dump(), content_type_json);
    }

    void get_current_joint_q(const httplib::Request &req, httplib::Response &resp)
    {
        vector<double> data = service::get_current_joint_q();
        json result;
        result["data"] = data;

        resp.set_content(result.dump(), content_type_json);
    }

    void get_current_joint_v(const httplib::Request &req, httplib::Response &resp)
    {
        vector<double> data = service::get_current_joint_v();
        json result;
        result["data"] = data;

        resp.set_content(result.dump(), content_type_json);
    }

    void get_current_joint_t(const httplib::Request &req, httplib::Response &resp)
    {
        vector<double> data = service::get_current_joint_t();
        json result;
        result["data"] = data;

        resp.set_content(result.dump(), content_type_json);
    }

    void get_current_translation(const httplib::Request &req, httplib::Response &resp)
    {
        vector<double> data = service::get_current_translation();
        json result;
        result["data"] = data;

        resp.set_content(result.dump(), content_type_json);
    }

    void get_current_rotation(const httplib::Request &req, httplib::Response &resp)
    {
        vector<double> data = service::get_current_rotation();
        json result;
        result["data"] = data;

        resp.set_content(result.dump(), content_type_json);
    }

    void get_current_end(const httplib::Request &req, httplib::Response &resp)
    {
        double data = service::get_current_end();
        json result;
        result["data"] = data;

        resp.set_content(result.dump(), content_type_json);
    }

    void set_target_pose(const httplib::Request &req, httplib::Response &resp)
    {
        json body = json::parse(req.body);
        if (body["data"].is_array() && body["data"].size() == 2)
        {
            vector<vector<double>> req_data = body["data"];
            service::set_target_pose(req_data);
            resp.set_content("OK", content_type_json);
        }
        else
        {
            resp.status = 400;
        }
    }

    void set_target_end(const httplib::Request &req, httplib::Response &resp)
    {
        json body = json::parse(req.body);
        if (body["data"].is_array())
        {
            vector<double> req_data = body["data"];
            service::set_target_joint_q(req_data);
            resp.set_content("OK", content_type_json);
        }
        else
        {
            resp.status = 400;
        }
        if (body["end"].is_number())
        {
            service::set_target_end(double(body["end"]));
            resp.set_content("OK", content_type_json);
        }
        else
        {
            resp.status = 400;
        }
    }

    void set_target_joint_q(const httplib::Request &req, httplib::Response &resp)
    {
        json body = json::parse(req.body);
        if (body["data"].is_array())
        {
            vector<double> req_data = body["data"];
            service::set_target_joint_q(req_data);
            resp.set_content("OK", content_type_json);
        }
        else
        {
            resp.status = 400;
        }
    }

    void set_target_joint_v(const httplib::Request &req, httplib::Response &resp)
    {
        json body = json::parse(req.body);
        if (body["data"].is_array())
        {
            vector<double> req_data = body["data"];
            service::set_target_joint_v(req_data);
            resp.set_content("OK", content_type_json);
        }
        else
        {
            resp.status = 400;
        }
    }
    void set_target_joint_q_v(const httplib::Request &req, httplib::Response &resp)
    {
        json body = json::parse(req.body);
        std::vector<std::vector<double>> vecs;
        if (body["data"].is_array() && body["data2"].is_array())
        {
            vector<double> req_data1 = body["data"];
            vecs.push_back(req_data1);
            vector<double> req_data2 = body["data2"];
            vecs.push_back(req_data2);
            service::set_target_joint_q_v(vecs);
            resp.set_content("OK", content_type_json);
        }
        else
        {
            resp.status = 400;
        }
    }

    void set_target_joint_t(const httplib::Request &req, httplib::Response &resp)
    {
        json body = json::parse(req.body);
        if (body["data"].is_array())
        {
            vector<double> req_data = body["data"];
            service::set_target_joint_t(req_data);
            resp.set_content("OK", content_type_json);
        }
        else
        {
            resp.status = 400;
        }
    }

    void set_target_translation(const httplib::Request &req, httplib::Response &resp)
    {
        json body = json::parse(req.body);
        if (body["data"].is_array())
        {
            vector<double> req_data = body["data"];
            service::set_target_translation(req_data);
            resp.set_content("OK", content_type_json);
        }
        else
        {
            resp.status = 400;
        }
    }

    void set_target_rotation(const httplib::Request &req, httplib::Response &resp)
    {
        json body = json::parse(req.body);
        if (body["data"].is_array())
        {
            vector<double> req_data = body["data"];
            service::set_target_rotation(req_data);
            resp.set_content("OK", content_type_json);
        }
        else
        {
            resp.status = 400;
        }
    }

    void add_target_joint_q(const httplib::Request &req, httplib::Response &resp)
    {
        json body = json::parse(req.body);
        if (body["data"].is_array())
        {
            vector<double> req_data = body["data"];
            service::add_target_joint_q(req_data);
            resp.set_content("OK", content_type_json);
        }
        else
        {
            resp.status = 400;
        }
    }

    void add_target_joint_v(const httplib::Request &req, httplib::Response &resp)
    {
        json body = json::parse(req.body);
        if (body["data"].is_array())
        {
            vector<double> req_data = body["data"];
            service::add_target_joint_v(req_data);
            resp.set_content("OK", content_type_json);
        }
        else
        {
            resp.status = 400;
        }
    }

    void add_target_translation(const httplib::Request &req, httplib::Response &resp)
    {
        json body = json::parse(req.body);
        if (body["data"].is_array())
        {
            vector<double> req_data = body["data"];
            service::add_target_translation(req_data);
            resp.set_content("OK", content_type_json);
        }
        else
        {
            resp.status = 400;
        }
    }

    void add_target_relative_rotation(const httplib::Request &req, httplib::Response &resp)
    {
        json body = json::parse(req.body);
        if (body["data"].is_array())
        {
            vector<double> req_data = body["data"];
            service::add_target_relative_rotation(req_data);
            resp.set_content("OK", content_type_json);
        }
        else
        {
            resp.status = 400;
        }
    }

    void start_gravity_compensation(const httplib::Request &req, httplib::Response &resp)
    {
        service::gravity_compensation();
        resp.set_content("OK", content_type_json);
    }

    void record_start(const httplib::Request &req, httplib::Response &resp)
    {
        service::record_start();
        resp.set_content("OK", content_type_json);
    }

    void record_stop(const httplib::Request &req, httplib::Response &resp)
    {
        service::record_stop();
        resp.set_content("OK", content_type_json);
    }

    void record_replay(const httplib::Request &req, httplib::Response &resp)
    {
        service::record_replay();
        resp.set_content("OK", content_type_json);
    }
}