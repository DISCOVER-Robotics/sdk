#include <vector>
#include <cstdlib>
#include <cstring>

#include "service.hpp"
#include "arm.hpp"
#include "airbot/modules/command/command_base.hpp"

namespace service
{
    void init(const char *urdf_path, const char *can)
    {
        if (Instance::get_instance() == nullptr)
        {
            std::string can_str(can);
            std::string cmd;
            cmd = "sudo ip link set " + can_str + " up type can bitrate 1000000";
            system(cmd.c_str());
            Instance::init(urdf_path, can);
        }
    }

    void erase()
    {
        if (Instance::get_instance() != nullptr)
        {
            Instance::erase();
        }
    }

    std::vector<std::vector<double>> get_target_pose()
    {
        if (Instance::get_instance() != nullptr)
        {
            return Instance::get_instance()->get_target_pose();
        }
        else
        {
            return std::vector<std::vector<double>>();
        }
    }

    vector<double> get_target_joint_q()
    {
        if (Instance::get_instance() != nullptr)
        {
            return Instance::get_instance()->get_target_joint_q();
        }
        else
        {
            return vector<double>();
        }
    }

    vector<double> get_target_joint_v()
    {
        if (Instance::get_instance() != nullptr)
        {
            return Instance::get_instance()->get_target_joint_v();
        }
        else
        {
            return vector<double>();
        }
    }

    vector<double> get_target_joint_t()
    {
        if (Instance::get_instance() != nullptr)
        {
            return Instance::get_instance()->get_target_joint_t();
        }
        else
        {
            return vector<double>();
        }
    }

    vector<double> get_target_translation()
    {
        if (Instance::get_instance() != nullptr)
        {
            return Instance::get_instance()->get_target_translation();
        }
        else
        {
            return vector<double>();
        }
    }

    vector<double> get_target_rotation()
    {
        if (Instance::get_instance() != nullptr)
        {
            return Instance::get_instance()->get_target_rotation();
        }
        else
        {
            return vector<double>();
        }
    }

    vector<vector<double>> get_current_pose()
    {
        if (Instance::get_instance() != nullptr)
        {
            return Instance::get_instance()->get_current_pose();
        }
        else
        {
            return vector<vector<double>>();
        }
    }

    vector<double> get_current_joint_q()
    {
        if (Instance::get_instance() != nullptr)
        {
            return Instance::get_instance()->get_current_joint_q();
        }
        else
        {
            return vector<double>();
        }
    }

    vector<double> get_current_joint_v()
    {
        if (Instance::get_instance() != nullptr)
        {
            return Instance::get_instance()->get_current_joint_v();
        }
        else
        {
            return vector<double>();
        }
    }

    vector<double> get_current_joint_t()
    {
        if (Instance::get_instance() != nullptr)
        {
            return Instance::get_instance()->get_current_joint_t();
        }
        else
        {
            return vector<double>();
        }
    }

    vector<double> get_current_translation()
    {
        if (Instance::get_instance() != nullptr)
        {
            return Instance::get_instance()->get_current_translation();
        }
        else
        {
            return vector<double>();
        }
    }

    vector<double> get_current_rotation()
    {
        if (Instance::get_instance() != nullptr)
        {
            return Instance::get_instance()->get_current_rotation();
        }
        else
        {
            return vector<double>();
        }
    }

    double get_current_end()
    {
        if (Instance::get_instance() != nullptr)
        {
            return Instance::get_instance()->get_current_end();
        }
        else
        {
            return 0.0;
        }
    }

    void set_target_end(double angle)
    {
        if (Instance::get_instance() != nullptr)
        {
            Instance::get_instance()->set_target_end(angle);
        }
    }

    void set_target_pose(const vector<vector<double>> &target_pose)
    {
        if (Instance::get_instance() != nullptr)
        {
            Instance::get_instance()->set_target_pose(target_pose);
        }
    }

    void set_target_pose(const vector<double> &target_translation,
                         const vector<double> &target_rotation)
    {
        if (Instance::get_instance() != nullptr)
        {
            Instance::get_instance()->set_target_pose(target_translation, target_rotation);
        }
    }

    void set_target_joint_q(const vector<double> &target_joint_q)
    {
        if (Instance::get_instance() != nullptr)
        {
            Instance::get_instance()->set_target_joint_q(target_joint_q);
        }
    }

    void set_target_joint_v(const vector<double> &target_joint_v)
    {
        if (Instance::get_instance() != nullptr)
        {
            Instance::get_instance()->set_target_joint_v(target_joint_v);
        }
    }

    void set_target_joint_q_v(const vector<vector<double>> &target_joint_q_v)
    {
        if (Instance::get_instance() != nullptr)
        {
            Instance::get_instance()->set_target_joint_q_v(target_joint_q_v);
        }
    }

    void set_target_joint_t(const vector<double> &target_joint_t)
    {
        if (Instance::get_instance() != nullptr)
        {
            Instance::get_instance()->set_target_joint_t(target_joint_t);
        }
    }

    void set_target_translation(const vector<double> &target_translation)
    {
        if (Instance::get_instance() != nullptr)
        {
            Instance::get_instance()->set_target_translation(target_translation);
        }
    }

    void set_target_rotation(const vector<double> &target_rotation)
    {
        if (Instance::get_instance() != nullptr)
        {
            Instance::get_instance()->set_target_rotation(target_rotation);
        }
    }

    void add_target_joint_q(const vector<double> &target_d_joint_q)
    {
        if (Instance::get_instance() != nullptr)
        {
            Instance::get_instance()->add_target_joint_q(target_d_joint_q);
        }
    }

    void add_target_joint_v(const vector<double> &target_d_joint_v)
    {
        if (Instance::get_instance() != nullptr)
        {
            Instance::get_instance()->add_target_joint_v(target_d_joint_v);
        }
    }

    void add_target_translation(const vector<double> &target_d_translation)
    {
        if (Instance::get_instance() != nullptr)
        {
            Instance::get_instance()->add_target_translation(target_d_translation);
        }
    }

    void add_target_relative_rotation(const vector<double> &target_d_rotation)
    {
        if (Instance::get_instance() != nullptr)
        {
            Instance::get_instance()->add_target_relative_rotation(target_d_rotation);
        }
    }

    void gravity_compensation()
    {
        if (Instance::get_instance() != nullptr)
        {
            Instance::get_instance()->gravity_compensation();
        }
    }

    void record_start()
    {
        if (Instance::get_instance() != nullptr)
        {
            Instance::get_instance()->record_start();
        }
    }

    void record_stop()
    {
        if (Instance::get_instance() != nullptr)
        {
            Instance::get_instance()->record_stop();
        }
    }

    void record_replay()
    {
        if (Instance::get_instance() != nullptr)
        {
            Instance::get_instance()->record_replay();
        }
    }
}