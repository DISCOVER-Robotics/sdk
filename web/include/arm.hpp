#ifndef ARM_HPP
#define ARM_HPP
#include <vector>
#include <thread>
#include <mutex>
#include <unordered_map>

#include "airbot/modules/controller/ik_chain.hpp"
#include "airbot/modules/controller/fk_chain.hpp"
#include "airbot/modules/controller/id_chain_rne.hpp"
#include "airbot/modules/motors/protocol/motor_driver.hpp"
#include "airbot/modules/command/command_base.hpp"

namespace Instance
{
    void init(const char *urdf_path, const char *can);
    arm::Status *get_instance();
    void erase();
};
#endif