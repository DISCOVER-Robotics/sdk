#include <vector>
#include <boost/thread/shared_mutex.hpp>

#include "airbot/modules/command/command_base.hpp"
#include "arm.hpp"

namespace Instance
{
    boost::shared_mutex mtx;
    arm::Status *arm = nullptr;

    void init(const char *urdf_path, const char *can)
    {
        mtx.lock();
        arm = new arm::Status(std::make_unique<arm::ChainFKSolver>(urdf_path),
                              std::make_unique<arm::ChainIKSolver>(urdf_path),
                              std::make_unique<arm::ChainIDSolver>(urdf_path),
                              can, 3.14159265, "gripper");
        mtx.unlock();
    }

    arm::Status *get_instance()
    {
        mtx.lock_shared();
        auto _arm = arm;
        mtx.unlock_shared();
        return _arm;
    }

    void erase()
    {
        mtx.lock();
        delete arm;
        arm = nullptr;
        mtx.unlock();
    }
}