#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include "httplib.h"

namespace controller
{
    void set_urdf_path(char *);
    void set_can_interface(char *interface);
    void init(const httplib::Request &req, httplib::Response &resp);
    void erase(const httplib::Request &req, httplib::Response &resp);
    void get_target_pose(const httplib::Request &req, httplib::Response &resp);
    void get_target_joint_q(const httplib::Request &req, httplib::Response &resp);
    void get_target_joint_v(const httplib::Request &req, httplib::Response &resp);
    void get_target_joint_t(const httplib::Request &req, httplib::Response &resp);
    void get_target_translation(const httplib::Request &req, httplib::Response &resp);
    void get_target_rotation(const httplib::Request &req, httplib::Response &resp);
    void get_current_pose(const httplib::Request &req, httplib::Response &resp);
    void get_current_joint_q(const httplib::Request &req, httplib::Response &resp);
    void get_current_joint_v(const httplib::Request &req, httplib::Response &resp);
    void get_current_joint_t(const httplib::Request &req, httplib::Response &resp);
    void get_current_translation(const httplib::Request &req, httplib::Response &resp);
    void get_current_rotation(const httplib::Request &req, httplib::Response &resp);
    void get_current_end(const httplib::Request &req, httplib::Response &resp);
    void set_target_pose(const httplib::Request &req, httplib::Response &resp);
    void set_target_end(const httplib::Request &req, httplib::Response &resp);
    void set_target_joint_q(const httplib::Request &req, httplib::Response &resp);
    void set_target_joint_q_v(const httplib::Request &req, httplib::Response &resp);
    void set_target_joint_v(const httplib::Request &req, httplib::Response &resp);
    void set_target_joint_t(const httplib::Request &req, httplib::Response &resp);
    void set_target_translation(const httplib::Request &req, httplib::Response &resp);
    void set_target_rotation(const httplib::Request &req, httplib::Response &resp);
    void add_target_joint_q(const httplib::Request &req, httplib::Response &resp);
    void add_target_joint_v(const httplib::Request &req, httplib::Response &resp);
    void add_target_translation(const httplib::Request &req, httplib::Response &resp);
    void add_target_relative_rotation(const httplib::Request &req, httplib::Response &resp);
    void start_gravity_compensation(const httplib::Request &req, httplib::Response &resp);
    void record_start(const httplib::Request &req, httplib::Response &resp);
    void record_stop(const httplib::Request &req, httplib::Response &resp);
    void record_replay(const httplib::Request &req, httplib::Response &resp);
}
#endif