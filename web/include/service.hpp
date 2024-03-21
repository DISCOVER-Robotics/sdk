#ifndef SERVICE_HPP
#define SERVICE_HPP
#include <vector>
#include <thread>

#include "airbot/modules/controller/ik_chain.hpp"
#include "airbot/modules/controller/fk_chain.hpp"
#include "airbot/modules/motors/protocol/motor_driver.hpp"
#include "airbot/modules/command/command_base.hpp"

using namespace std;

namespace service
{
  void init(const char *, const char *);

  void erase();
  /**
   * @brief Get target end pose
   * @return `vector<vector<double>>`: target position (x, y, z in base
   * coordinates) and target direction (quaternion, the rotation between target
   * direction and (1, 0, 0))
   */
  vector<vector<double>> get_target_pose();
  /**
   * @brief Get target joint positions
   * @return `vector<double>`: containing angles of the motors from base to end
   */
  vector<double> get_target_joint_q();
  /**
   * @brief Get target joint velocities
   * @return `vector<double>`: containing angular velocities of the motors from
   * base to end
   */
  vector<double> get_target_joint_v();
  /**
   * @brief Get target joint torques
   * @return `vector<double>`: containing torques of the motors from base to end
   */
  vector<double> get_target_joint_t();

  /**
   * @brief Get target end position
   * @return `vector<double>`: target position (x, y, z in base
   * coordinates)
   */
  vector<double> get_target_translation();

  /**
   * @brief Get target end direction
   * @return `vector<double>`: target direction (quaternion, the rotation
   * between target direction and (1, 0, 0))
   */
  vector<double> get_target_rotation();

  /**
   * @brief Get current end pose
   * @return `vector<vector<double>>`: current position (x, y, z in base
   * coordinates) and current direction (quaternion, the rotation between
   * current direction and (1, 0, 0))
   */
  vector<vector<double>> get_current_pose();
  /**
   * @brief Get current joint positions
   * @return `vector<double>`: containing angles of the motors from base to end
   */
  vector<double> get_current_joint_q();
  /**
   * @brief Get current joint velocities
   * @return `vector<double>`: containing angular velocities of the motors from
   * base to end
   */
  vector<double> get_current_joint_v();
  /**
   * @brief Get current joint torques
   * @return `vector<double>`: containing angular torques of the motors from
   * base to end
   */
  vector<double> get_current_joint_t();

  /**
   * @brief Get current end position
   * @return `vector<double>`: current position (x, y, z in base
   * coordinates)
   */
  vector<double> get_current_translation();
  /**
   * @brief Get current end direction
   * @return `vector<double>`: current direction (quaternion, the rotation
   * between current direction and (1, 0, 0))
   */
  vector<double> get_current_rotation();

  double get_current_end();
  void set_target_end(double end);

  /**
   * @brief Set target end pose
   * @param target_pose `vector<vector<double>>`: target position (x, y, z in
   * base coordinates) and target direction (quaternion, the rotation between
   * target direction and (1, 0, 0))
   */
  void set_target_pose(const vector<vector<double>> &target_pose);
  /**
   * @brief Set target end pose
   * @param target_translation `vector<double>`: target position (x, y, z in
   * base coordinates)
   * @param target_rotation `vector<double>`: target direction (quaternion, the
   * rotation between target direction and (1, 0, 0))
   */
  void set_target_pose(const vector<double> &target_translation,
                       const vector<double> &target_rotation);
  /**
   * @brief Set target joint positions
   * @param target_joint_q `vector<double>`: containing angles of the motors
   * from base to end
   */
  void set_target_joint_q(const vector<double> &target_joint_q);
  /**
   * @brief Set target joint velocities
   * @param target_joint_v `vector<double>`: containing angular velocities of
   * the motors from base to end
   */
  void set_target_joint_v(const vector<double> &target_joint_v);
  /**
   * @brief Set target joint torques
   * @param target_joint_t `vector<double>`: containing angular torques of the
   * motors from base to end
   */

  void set_target_joint_q_v(const vector<vector<double>> &target_joint_q_v);
  void set_target_joint_t(const vector<double> &target_joint_t);

  /**
   * @brief Set target end position
   * @param target_translation `vector<double>`: target position (x, y, z in
   * base coordinates)
   */
  void set_target_translation(const vector<double> &target_translation);
  /**
   * @brief Set target end direction
   * @param target_rotation `vector<double>`: target direction (quaternion, the
   * rotation between target direction and (1, 0, 0))
   */
  void set_target_rotation(const vector<double> &target_rotation);

  /**
   * @brief Add target joint positions by delta
   * @param target_d_joint_q `vector<double>`: containing delta angles of the
   * motors from base to end
   */
  void add_target_joint_q(const vector<double> &target_d_joint_q);
  /**
   * @brief Add target joint velocities by delta
   * @param target_d_joint_v `vector<double>`: containing delta angular
   * velocities of the motors from base to end
   */
  void add_target_joint_v(const vector<double> &target_d_joint_v);
  /**
   * @brief Add target joint torques by delta
   * @param target_d_joint_t `vector<double>`: containing delta angular torques
   * of the motors from base to end
   */
  void add_target_translation(const vector<double> &target_d_translation);

  /**
   * @brief Add target translation from end
   * @param target_d_rotation `vector<double>`: target rotation (quaternion in
   * end link frame)
   */
  void add_target_relative_rotation(const vector<double> &target_d_rotation);

  void gravity_compensation();

  void record_start();

  void record_stop();

  void record_replay();
};

#endif