#include <airbot/airbot.hpp>

#include "argparse/argparse.hpp"

int main(int argc, char **argv) {
  // argparse
  argparse::ArgumentParser program("airbot_packing", AIRBOT_VERSION);
  program.add_description("A simple program to set AIRBOT Play to packing pose.");
  program.add_argument("-m", "--master")
      .required()
      .default_value("can0")
      .help("Can device interface of the master arm.");
  program.add_argument("-u", "--urdf")
      .default_value(std::string())
      .help("Manually provided URDF path to override default paths.");
  program.add_argument("--bigarm-type")
      .default_value("OD")
      .choices("DM", "OD", "encoder")
      .help(
          "The type of big arm. Available choices: \"DM\": Damiao motor, \"OD\": Self-developed motors, \"encoder\": "
          "Self-developed encoders");
  program.add_argument("--forearm-type")
      .default_value("DM")
      .choices("DM", "OD")
      .help("The type of forearm. Available choices: \"DM\": Damiao motor, \"OD\": Self-developed motors");
  try {
    program.parse_args(argc, argv);
  } catch (const std::exception &err) {
    std::cerr << err.what() << std::endl;
    std::cerr << program;
    return 1;
  }
  std::string urdf_path = program.get<std::string>("--urdf");
  std::string master_can = program.get<std::string>("--master");
  std::string bigarm_type = program.get<std::string>("--bigarm-type");
  std::string forearm_type = program.get<std::string>("--forearm-type");
  if (urdf_path == "") urdf_path = URDF_INSTALL_PATH + "airbot_play_v2_1_with_gripper.urdf";

  // Synchronization
  std::shared_ptr<arm::Robot<6>> robot;
  try {
    robot = std::make_shared<arm::Robot<6>>(urdf_path, master_can, "down", M_PI / 2, "none", bigarm_type, forearm_type);
  } catch (const std::runtime_error &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }

  robot->set_target_joint_q({-1.590562, -1.521678, -0.007057, -0.027276, -1.544404, -0.232128}, true, 0.2, true);

  std::exit(0);
}
