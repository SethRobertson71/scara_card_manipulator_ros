#pragma once
#include <string>
#include <memory>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <m1pro_bringup/commander.h>
#include <m1pro_bringup/srv/enable_robot.hpp>
#include <m1pro_bringup/srv/disable_robot.hpp>
#include <m1pro_bringup/srv/clear_error.hpp>
#include <m1pro_bringup/srv/reset_robot.hpp>
#include <m1pro_bringup/srv/speed_factor.hpp>
#include <m1pro_bringup/srv/user.hpp>
#include <m1pro_bringup/srv/tool.hpp>
#include <m1pro_bringup/srv/robot_mode.hpp>
#include <m1pro_bringup/srv/pay_load.hpp>
#include <m1pro_bringup/srv/do.hpp>
#include <m1pro_bringup/srv/do_execute.hpp>
#include <m1pro_bringup/srv/tool_do.hpp>
#include <m1pro_bringup/srv/tool_do_execute.hpp>
#include <m1pro_bringup/srv/acc_j.hpp>
#include <m1pro_bringup/srv/acc_l.hpp>
#include <m1pro_bringup/srv/speed_j.hpp>
#include <m1pro_bringup/srv/speed_l.hpp>
#include <m1pro_bringup/srv/arch.hpp>
#include <m1pro_bringup/srv/cp.hpp>
#include <m1pro_bringup/srv/lim_z.hpp>
#include <m1pro_bringup/srv/set_arm_orientation.hpp>
#include <m1pro_bringup/srv/run_script.hpp>
#include <m1pro_bringup/srv/stop_script.hpp>
#include <m1pro_bringup/srv/pause_script.hpp>
#include <m1pro_bringup/srv/continue_script.hpp>
#include <m1pro_bringup/srv/get_hold_regs.hpp>
#include <m1pro_bringup/srv/set_hold_regs.hpp>
#include <m1pro_bringup/srv/set_obstacle_avoid.hpp>
#include <m1pro_bringup/srv/set_collision_level.hpp>
#include <m1pro_bringup/srv/emergency_stop.hpp>
#include <m1pro_bringup/srv/mov_j.hpp>
#include <m1pro_bringup/srv/mov_l.hpp>
#include <m1pro_bringup/srv/joint_mov_j.hpp>
#include <m1pro_bringup/srv/jump.hpp>
#include <m1pro_bringup/srv/rel_mov_j.hpp>
#include <m1pro_bringup/srv/rel_mov_l.hpp>
#include <m1pro_bringup/srv/mov_lio.hpp>
#include <m1pro_bringup/srv/mov_jio.hpp>
#include <m1pro_bringup/srv/arc.hpp>
#include <m1pro_bringup/srv/circle.hpp>
#include <m1pro_bringup/srv/rel_mov_j_user.hpp>
#include <m1pro_bringup/srv/rel_mov_l_user.hpp>
#include <m1pro_bringup/srv/rel_joint_mov_j.hpp>
#include <m1pro_bringup/srv/mov_j_ext.hpp>
#include <m1pro_bringup/srv/sync.hpp>
#include <m1pro_bringup/srv/sync_all.hpp>
#include <m1pro_bringup/srv/move_jog.hpp>
#include <m1pro_bringup/srv/stopmove_jog.hpp>
#include <m1pro_bringup/srv/wait.hpp>
#include <m1pro_bringup/srv/continues.hpp>
#include <m1pro_bringup/srv/pause.hpp>
#include <m1pro_bringup/srv/modbus_create.hpp>
#include <m1pro_bringup/srv/modbus_close.hpp>
#include <m1pro_bringup/srv/get_in_bits.hpp>
#include <m1pro_bringup/srv/get_in_regs.hpp>
#include <m1pro_bringup/srv/get_coils.hpp>
#include <m1pro_bringup/srv/set_coils.hpp>
#include <m1pro_bringup/srv/di.hpp>
#include <m1pro_bringup/srv/tool_di.hpp>
#include <m1pro_bringup/srv/do_group.hpp>
#include <m1pro_bringup/srv/brake_control.hpp>
#include <m1pro_bringup/srv/start_drag.hpp>
#include <m1pro_bringup/srv/stop_drag.hpp>
#include <m1pro_bringup/srv/load_switch.hpp>
#include <m1pro_bringup/srv/get_angle.hpp>
#include <m1pro_bringup/srv/get_pose.hpp>
#include <m1pro_bringup/srv/get_error_id.hpp>
#include <m1pro_bringup/srv/set_payload.hpp>
#include <m1pro_bringup/srv/positive_solution.hpp>
#include <m1pro_bringup/srv/inverse_solution.hpp>

class M1ProRobot : public rclcpp::Node {
public:
    explicit M1ProRobot(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~M1ProRobot() override = default;
    void init();
    void getJointState(double* point);
    void getToolVectorActual(double* val);
    bool isEnable() const;
    bool isConnected() const;

private:
    std::shared_ptr<CR5Commander> commander_;
    std::vector<rclcpp::ServiceBase::SharedPtr> server_tbl_;

    void enableRobot(const std::shared_ptr<m1pro_bringup::srv::EnableRobot::Request> req, std::shared_ptr<m1pro_bringup::srv::EnableRobot::Response> res);
    void disableRobot(const std::shared_ptr<m1pro_bringup::srv::DisableRobot::Request> req, std::shared_ptr<m1pro_bringup::srv::DisableRobot::Response> res);
    void clearError(const std::shared_ptr<m1pro_bringup::srv::ClearError::Request> req, std::shared_ptr<m1pro_bringup::srv::ClearError::Response> res);
    void resetRobot(const std::shared_ptr<m1pro_bringup::srv::ResetRobot::Request> req, std::shared_ptr<m1pro_bringup::srv::ResetRobot::Response> res);
    void speedFactor(const std::shared_ptr<m1pro_bringup::srv::SpeedFactor::Request> req, std::shared_ptr<m1pro_bringup::srv::SpeedFactor::Response> res);
    void getErrorID(const std::shared_ptr<m1pro_bringup::srv::GetErrorID::Request> req, std::shared_ptr<m1pro_bringup::srv::GetErrorID::Response> res);
    void robotMode(const std::shared_ptr<m1pro_bringup::srv::RobotMode::Request> req, std::shared_ptr<m1pro_bringup::srv::RobotMode::Response> res);
    void emergencyStop(const std::shared_ptr<m1pro_bringup::srv::EmergencyStop::Request> req, std::shared_ptr<m1pro_bringup::srv::EmergencyStop::Response> res);
    void getAngle(const std::shared_ptr<m1pro_bringup::srv::GetAngle::Request> req, std::shared_ptr<m1pro_bringup::srv::GetAngle::Response> res);
    void getPose(const std::shared_ptr<m1pro_bringup::srv::GetPose::Request> req, std::shared_ptr<m1pro_bringup::srv::GetPose::Response> res);
    void movJ(const std::shared_ptr<m1pro_bringup::srv::MovJ::Request> req, std::shared_ptr<m1pro_bringup::srv::MovJ::Response> res);
    void movL(const std::shared_ptr<m1pro_bringup::srv::MovL::Request> req, std::shared_ptr<m1pro_bringup::srv::MovL::Response> res);
    void jointMovJ(const std::shared_ptr<m1pro_bringup::srv::JointMovJ::Request> req, std::shared_ptr<m1pro_bringup::srv::JointMovJ::Response> res);
    void relMovJ(const std::shared_ptr<m1pro_bringup::srv::RelMovJ::Request> req, std::shared_ptr<m1pro_bringup::srv::RelMovJ::Response> res);
    void relMovL(const std::shared_ptr<m1pro_bringup::srv::RelMovL::Request> req, std::shared_ptr<m1pro_bringup::srv::RelMovL::Response> res);
    void DO(const std::shared_ptr<m1pro_bringup::srv::DO::Request> req, std::shared_ptr<m1pro_bringup::srv::DO::Response> res);
    void DOExecute(const std::shared_ptr<m1pro_bringup::srv::DOExecute::Request> req, std::shared_ptr<m1pro_bringup::srv::DOExecute::Response> res);
    void toolDO(const std::shared_ptr<m1pro_bringup::srv::ToolDO::Request> req, std::shared_ptr<m1pro_bringup::srv::ToolDO::Response> res);
    void toolDOExecute(const std::shared_ptr<m1pro_bringup::srv::ToolDOExecute::Request> req, std::shared_ptr<m1pro_bringup::srv::ToolDOExecute::Response> res);
    void DOGroup(const std::shared_ptr<m1pro_bringup::srv::DOGroup::Request> req, std::shared_ptr<m1pro_bringup::srv::DOGroup::Response> res);
    void moveJog(const std::shared_ptr<m1pro_bringup::srv::MoveJog::Request> req, std::shared_ptr<m1pro_bringup::srv::MoveJog::Response> res);
    void stopmoveJog(const std::shared_ptr<m1pro_bringup::srv::StopmoveJog::Request> req, std::shared_ptr<m1pro_bringup::srv::StopmoveJog::Response> res);
    void sync(const std::shared_ptr<m1pro_bringup::srv::Sync::Request> req, std::shared_ptr<m1pro_bringup::srv::Sync::Response> res);
    void syncAll(const std::shared_ptr<m1pro_bringup::srv::SyncAll::Request> req, std::shared_ptr<m1pro_bringup::srv::SyncAll::Response> res);
    void relJointMovJ(const std::shared_ptr<m1pro_bringup::srv::RelJointMovJ::Request> req, std::shared_ptr<m1pro_bringup::srv::RelJointMovJ::Response> res);

    static int str2Int(const char* val);
};
