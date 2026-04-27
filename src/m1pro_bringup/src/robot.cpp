#include <m1pro_bringup/robot.h>
#include <cstdio>
#include <cctype>
#include <cstdlib>
#include <cstring>
#include <stdexcept>

namespace {

int32_t first_active_alarm_id(const std::vector<std::string>& tokens)
{
    for (const auto& token : tokens) {
        const char* p = token.c_str();
        while (*p != '\0') {
            while (*p != '\0' && !std::isdigit(static_cast<unsigned char>(*p)) && *p != '-') {
                ++p;
            }
            if (*p == '\0') {
                break;
            }

            char* end = nullptr;
            long value = std::strtol(p, &end, 10);
            if (end == p) {
                ++p;
                continue;
            }
            if (value > 0) {
                return static_cast<int32_t>(value);
            }
            p = end;
        }
    }
    return 0;
}

}  // namespace

M1ProRobot::M1ProRobot(const rclcpp::NodeOptions& options)
    : rclcpp::Node("m1pro_driver", options) {}

void M1ProRobot::init()
{
    std::string ip = this->declare_parameter<std::string>("robot_ip_address", "192.168.1.6");
    RCLCPP_INFO(this->get_logger(), "Connecting to Dobot M1 Pro at %s", ip.c_str());
    commander_ = std::make_shared<CR5Commander>(ip);
    commander_->init();

    #define REG_SRV(SrvType, path, handler) \
        server_tbl_.push_back(this->create_service<m1pro_bringup::srv::SrvType>( \
            "/bringup/srv/" #path, \
            std::bind(&M1ProRobot::handler, this, std::placeholders::_1, std::placeholders::_2)))

    REG_SRV(EnableRobot,   EnableRobot,   enableRobot);
    REG_SRV(DisableRobot,  DisableRobot,  disableRobot);
    REG_SRV(ClearError,    ClearError,    clearError);
    REG_SRV(ResetRobot,    ResetRobot,    resetRobot);
    REG_SRV(SpeedFactor,   SpeedFactor,   speedFactor);
    REG_SRV(GetErrorID,    GetErrorID,    getErrorID);
    REG_SRV(RobotMode,     RobotMode,     robotMode);
    REG_SRV(EmergencyStop, EmergencyStop, emergencyStop);
    REG_SRV(GetAngle,      GetAngle,      getAngle);
    REG_SRV(GetPose,       GetPose,       getPose);
    REG_SRV(DO,            DO,            DO);
    REG_SRV(DOExecute,     DOExecute,     DOExecute);
    REG_SRV(ToolDO,        ToolDO,        toolDO);
    REG_SRV(ToolDOExecute, ToolDOExecute, toolDOExecute);
    REG_SRV(DOGroup,       DOGroup,       DOGroup);
    REG_SRV(MovJ,          MovJ,          movJ);
    REG_SRV(MovL,          MovL,          movL);
    REG_SRV(JointMovJ,     JointMovJ,     jointMovJ);
    REG_SRV(RelMovJ,       RelMovJ,       relMovJ);
    REG_SRV(RelMovL,       RelMovL,       relMovL);
    REG_SRV(MoveJog,       MoveJog,       moveJog);
    REG_SRV(StopmoveJog,   StopmoveJog,   stopmoveJog);
    REG_SRV(Sync,          Sync,          sync);
    REG_SRV(SyncAll,       SyncAll,       syncAll);
    REG_SRV(RelJointMovJ,  RelJointMovJ,  relJointMovJ);
    REG_SRV(SetArmOrientation, SetArmOrientation, setArmOrientation);
    
    #undef REG_SRV

    RCLCPP_INFO(this->get_logger(), "M1Pro driver ready — %zu services registered", server_tbl_.size());
}

void M1ProRobot::getJointState(double* point) { commander_->getCurrentJointStatus(point); }
bool M1ProRobot::isEnable() const { return commander_->isEnable(); }
bool M1ProRobot::isConnected() const { return commander_->isConnected(); }
void M1ProRobot::getToolVectorActual(double* val) { commander_->getToolVectorActual(val); }

#define DASH_CMD(name, cmd_str) \
    void M1ProRobot::name( \
        const std::shared_ptr<m1pro_bringup::srv::name##Request>, \
        std::shared_ptr<m1pro_bringup::srv::name##Response> res) \
    { try { commander_->dashboardDoCmd(cmd_str, res->res); } \
      catch (const std::exception& e) { RCLCPP_ERROR(this->get_logger(), #name ": %s", e.what()); res->res = -1; } }

void M1ProRobot::enableRobot(
    const std::shared_ptr<m1pro_bringup::srv::EnableRobot::Request> req,
    std::shared_ptr<m1pro_bringup::srv::EnableRobot::Response> res)
{
    try {
        std::string str = "EnableRobot(";
        for (size_t i = 0; i < req->args.size(); i++) {
            str += std::to_string(req->args[i]);
            if (i < req->args.size() - 1) str += ",";
        }
        str += ")";
        commander_->dashboardDoCmd(str.c_str(), res->res);
    } catch (const std::exception& e) { RCLCPP_ERROR(this->get_logger(), "enableRobot: %s", e.what()); res->res = -1; }
}

void M1ProRobot::disableRobot(const std::shared_ptr<m1pro_bringup::srv::DisableRobot::Request>, std::shared_ptr<m1pro_bringup::srv::DisableRobot::Response> res)
{ try { commander_->dashboardDoCmd("DisableRobot()", res->res); } catch (const std::exception& e) { RCLCPP_ERROR(this->get_logger(), "disableRobot: %s", e.what()); res->res = -1; } }

void M1ProRobot::clearError(const std::shared_ptr<m1pro_bringup::srv::ClearError::Request>, std::shared_ptr<m1pro_bringup::srv::ClearError::Response> res)
{ try { commander_->dashboardDoCmd("ClearError()", res->res); } catch (const std::exception& e) { RCLCPP_ERROR(this->get_logger(), "clearError: %s", e.what()); res->res = -1; } }

void M1ProRobot::resetRobot(const std::shared_ptr<m1pro_bringup::srv::ResetRobot::Request>, std::shared_ptr<m1pro_bringup::srv::ResetRobot::Response> res)
{ try { commander_->dashboardDoCmd("ResetRobot()", res->res); } catch (const std::exception& e) { RCLCPP_ERROR(this->get_logger(), "resetRobot: %s", e.what()); res->res = -1; } }

void M1ProRobot::speedFactor(const std::shared_ptr<m1pro_bringup::srv::SpeedFactor::Request> req, std::shared_ptr<m1pro_bringup::srv::SpeedFactor::Response> res)
{ try { char cmd[64]; snprintf(cmd, sizeof(cmd), "SpeedFactor(%d)", req->ratio); commander_->dashboardDoCmd(cmd, res->res); } catch (const std::exception& e) { RCLCPP_ERROR(this->get_logger(), "speedFactor: %s", e.what()); res->res = -1; } }

void M1ProRobot::getErrorID(const std::shared_ptr<m1pro_bringup::srv::GetErrorID::Request>, std::shared_ptr<m1pro_bringup::srv::GetErrorID::Response> res)
{
    try {
        int32_t cmd_status = 0;
        std::vector<std::string> result;
        commander_->dashboardDoCmd("GetErrorID()", cmd_status, result);

        if (cmd_status != 0) {
            // If the command itself fails, keep legacy behavior by returning transport/protocol status.
            res->res = cmd_status;
            return;
        }

        // When the command succeeds, surface the first active alarm id (0 means no active alarms).
        res->res = first_active_alarm_id(result);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "getErrorID: %s", e.what());
        res->res = -1;
    }
}

void M1ProRobot::robotMode(const std::shared_ptr<m1pro_bringup::srv::RobotMode::Request>, std::shared_ptr<m1pro_bringup::srv::RobotMode::Response> res)
{
    try {
        std::vector<std::string> result;
        commander_->dashboardDoCmd("RobotMode()", res->res, result);
        if (!result.empty()) res->mode = str2Int(result[0].c_str());
    } catch (const std::exception& e) { RCLCPP_ERROR(this->get_logger(), "robotMode: %s", e.what()); res->res = -1; }
}

void M1ProRobot::emergencyStop(const std::shared_ptr<m1pro_bringup::srv::EmergencyStop::Request>, std::shared_ptr<m1pro_bringup::srv::EmergencyStop::Response> res)
{ try { commander_->dashboardDoCmd("EmergencyStop()", res->res); } catch (const std::exception& e) { RCLCPP_ERROR(this->get_logger(), "emergencyStop: %s", e.what()); res->res = -1; } }

void M1ProRobot::getAngle(const std::shared_ptr<m1pro_bringup::srv::GetAngle::Request>, std::shared_ptr<m1pro_bringup::srv::GetAngle::Response> res)
{ try { commander_->dashboardDoCmd("GetAngle()", res->res); } catch (const std::exception& e) { RCLCPP_ERROR(this->get_logger(), "getAngle: %s", e.what()); res->res = -1; } }

void M1ProRobot::getPose(const std::shared_ptr<m1pro_bringup::srv::GetPose::Request>, std::shared_ptr<m1pro_bringup::srv::GetPose::Response> res)
{ try { commander_->dashboardDoCmd("GetPose()", res->res); } catch (const std::exception& e) { RCLCPP_ERROR(this->get_logger(), "getPose: %s", e.what()); res->res = -1; } }

void M1ProRobot::DO(const std::shared_ptr<m1pro_bringup::srv::DO::Request> req, std::shared_ptr<m1pro_bringup::srv::DO::Response> res)
{ try { char cmd[64]; snprintf(cmd, sizeof(cmd), "DO(%d, %d)", req->index, req->status); commander_->dashboardDoCmd(cmd, res->res); } catch (const std::exception& e) { RCLCPP_ERROR(this->get_logger(), "DO: %s", e.what()); res->res = -1; } }

void M1ProRobot::DOExecute(const std::shared_ptr<m1pro_bringup::srv::DOExecute::Request> req, std::shared_ptr<m1pro_bringup::srv::DOExecute::Response> res)
{ try { char cmd[64]; snprintf(cmd, sizeof(cmd), "DOExecute(%d, %d)", req->index, req->status); commander_->dashboardDoCmd(cmd, res->res); } catch (const std::exception& e) { RCLCPP_ERROR(this->get_logger(), "DOExecute: %s", e.what()); res->res = -1; } }

void M1ProRobot::toolDO(const std::shared_ptr<m1pro_bringup::srv::ToolDO::Request> req, std::shared_ptr<m1pro_bringup::srv::ToolDO::Response> res)
{ try { char cmd[64]; snprintf(cmd, sizeof(cmd), "ToolDO(%d, %d)", req->index, req->status); commander_->dashboardDoCmd(cmd, res->res); } catch (const std::exception& e) { RCLCPP_ERROR(this->get_logger(), "toolDO: %s", e.what()); res->res = -1; } }

void M1ProRobot::toolDOExecute(const std::shared_ptr<m1pro_bringup::srv::ToolDOExecute::Request> req, std::shared_ptr<m1pro_bringup::srv::ToolDOExecute::Response> res)
{ try { char cmd[64]; snprintf(cmd, sizeof(cmd), "ToolDOExecute(%d, %d)", req->index, req->status); commander_->dashboardDoCmd(cmd, res->res); } catch (const std::exception& e) { RCLCPP_ERROR(this->get_logger(), "toolDOExecute: %s", e.what()); res->res = -1; } }

void M1ProRobot::DOGroup(const std::shared_ptr<m1pro_bringup::srv::DOGroup::Request> req, std::shared_ptr<m1pro_bringup::srv::DOGroup::Response> res)
{ try { std::string cmd = "DOGroup("; for (size_t i = 0; i < req->args.size(); ++i) { cmd += std::to_string(req->args[i]); if (i + 1 < req->args.size()) cmd += ","; } cmd += ")"; commander_->dashboardDoCmd(cmd.c_str(), res->res); } catch (const std::exception& e) { RCLCPP_ERROR(this->get_logger(), "DOGroup: %s", e.what()); res->res = -1; } }

void M1ProRobot::movJ(const std::shared_ptr<m1pro_bringup::srv::MovJ::Request> req, std::shared_ptr<m1pro_bringup::srv::MovJ::Response> res)
{
    try {
        char cmd[512];
        snprintf(cmd, sizeof(cmd), "MovJ(%.3f,%.3f,%.3f,%.3f,AccJ=60)", req->x, req->y, req->z, req->r);
        std::string full = std::string(cmd);
        for (const auto& p : req->param_value) full += "," + p;
        full += ")";
        if (full.size() >= 2 && full.substr(full.size()-2) == "))") full = full.substr(0, full.size()-1);
        commander_->motionDoCmd(full.c_str(), res->res);
    } catch (const std::exception& e) { RCLCPP_ERROR(this->get_logger(), "movJ: %s", e.what()); res->res = -1; }
}

void M1ProRobot::movL(const std::shared_ptr<m1pro_bringup::srv::MovL::Request> req, std::shared_ptr<m1pro_bringup::srv::MovL::Response> res)
{ try { char cmd[64]; snprintf(cmd, sizeof(cmd), "MovL(%.3f,%.3f,%.3f,%.3f)", req->x, req->y, req->z, req->r); commander_->motionDoCmd(cmd, res->res); } catch (const std::exception& e) { RCLCPP_ERROR(this->get_logger(), "movL: %s", e.what()); res->res = -1; } }

void M1ProRobot::jointMovJ(const std::shared_ptr<m1pro_bringup::srv::JointMovJ::Request> req, std::shared_ptr<m1pro_bringup::srv::JointMovJ::Response> res)
{ try { char cmd[64]; snprintf(cmd, sizeof(cmd), "JointMovJ(%.3f,%.3f,%.3f,%.3f)", req->j1, req->j2, req->j3, req->j4); commander_->motionDoCmd(cmd, res->res); } catch (const std::exception& e) { RCLCPP_ERROR(this->get_logger(), "jointMovJ: %s", e.what()); res->res = -1; } }

void M1ProRobot::relMovJ(const std::shared_ptr<m1pro_bringup::srv::RelMovJ::Request> req, std::shared_ptr<m1pro_bringup::srv::RelMovJ::Response> res)
{ try { char cmd[64]; snprintf(cmd, sizeof(cmd), "RelMovJ(%.3f,%.3f,%.3f,%.3f)", req->offset1, req->offset2, req->offset3, req->offset4); commander_->motionDoCmd(cmd, res->res); } catch (const std::exception& e) { RCLCPP_ERROR(this->get_logger(), "relMovJ: %s", e.what()); res->res = -1; } }

void M1ProRobot::relMovL(const std::shared_ptr<m1pro_bringup::srv::RelMovL::Request> req, std::shared_ptr<m1pro_bringup::srv::RelMovL::Response> res)
{ try { char cmd[64]; snprintf(cmd, sizeof(cmd), "RelMovL(%.3f,%.3f,%.3f)", req->x, req->y, req->z); commander_->motionDoCmd(cmd, res->res); } catch (const std::exception& e) { RCLCPP_ERROR(this->get_logger(), "relMovL: %s", e.what()); res->res = -1; } }

void M1ProRobot::moveJog(const std::shared_ptr<m1pro_bringup::srv::MoveJog::Request> req, std::shared_ptr<m1pro_bringup::srv::MoveJog::Response> res)
{ try { std::string cmd = "MoveJog(" + req->axis_id + ")"; commander_->motionDoCmd(cmd.c_str(), res->res); } catch (const std::exception& e) { RCLCPP_ERROR(this->get_logger(), "moveJog: %s", e.what()); res->res = -1; } }

void M1ProRobot::stopmoveJog(const std::shared_ptr<m1pro_bringup::srv::StopmoveJog::Request>, std::shared_ptr<m1pro_bringup::srv::StopmoveJog::Response> res)
{ try { commander_->motionDoCmd("MoveJog()", res->res); } catch (const std::exception& e) { RCLCPP_ERROR(this->get_logger(), "stopmoveJog: %s", e.what()); res->res = -1; } }

void M1ProRobot::sync(const std::shared_ptr<m1pro_bringup::srv::Sync::Request>, std::shared_ptr<m1pro_bringup::srv::Sync::Response> res)
{ try { commander_->motionDoCmd("Sync()", res->res); } catch (const std::exception& e) { RCLCPP_ERROR(this->get_logger(), "sync: %s", e.what()); res->res = -1; } }

void M1ProRobot::syncAll(const std::shared_ptr<m1pro_bringup::srv::SyncAll::Request>, std::shared_ptr<m1pro_bringup::srv::SyncAll::Response> res)
{ try { commander_->motionDoCmd("SyncAll()", res->res); } catch (const std::exception& e) { RCLCPP_ERROR(this->get_logger(), "syncAll: %s", e.what()); res->res = -1; } }

void M1ProRobot::relJointMovJ(const std::shared_ptr<m1pro_bringup::srv::RelJointMovJ::Request> req, std::shared_ptr<m1pro_bringup::srv::RelJointMovJ::Response> res)
{ try { char cmd[64]; snprintf(cmd, sizeof(cmd), "RelJointMovJ(%.3f,%.3f,%.3f,%.3f)", req->offset1, req->offset2, req->offset3, req->offset4); commander_->motionDoCmd(cmd, res->res); } catch (const std::exception& e) { RCLCPP_ERROR(this->get_logger(), "relJointMovJ: %s", e.what()); res->res = -1; } }

void M1ProRobot::setArmOrientation(const std::shared_ptr<m1pro_bringup::srv::SetArmOrientation::Request> req, std::shared_ptr<m1pro_bringup::srv::SetArmOrientation::Response> res)
{
    try {
        char cmd[128];
        snprintf(cmd, sizeof(cmd), "SetArmOrientation(%d)", req->l_or_r);
        commander_->motionDoCmd(cmd, res->res);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "setArmOrientation: %s", e.what());
        res->res = -1;
    }
}

int M1ProRobot::str2Int(const char* val)
{
    char* end;
    int v = (int)strtol(val, &end, 10);
    if (*end != '\0') throw std::logic_error(std::string("Invalid value: ") + val);
    return v;
}
