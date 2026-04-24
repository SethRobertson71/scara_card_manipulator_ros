#pragma once
#include <vector>
#include <string>
#include <memory>
#include <thread>
#include <mutex>
#include <atomic>
#include <cstring>
#include <cassert>
#include <sstream>
#include <stdexcept>
#include <rclcpp/rclcpp.hpp>
#include <m1pro_bringup/tcp_socket.h>

#pragma pack(push, 1)
struct RealTimeData {
    uint16_t len;
    uint16_t Reserve[3];
    uint64_t digital_input_bits;
    uint64_t digital_outputs;
    uint64_t robot_mode;
    uint64_t controller_timer;
    uint64_t run_time;
    uint64_t test_value;
    double safety_mode;
    double speed_scaling;
    double linear_momentum_norm;
    double v_main;
    double v_robot;
    double i_robot;
    double program_state;
    double safety_status;
    double tool_accelerometer_values[3];
    double elbow_position[3];
    double elbow_velocity[3];
    double q_target[6];
    double qd_target[6];
    double qdd_target[6];
    double i_target[6];
    double m_target[6];
    double q_actual[6];      // joint positions (degrees) at offset 432
    double qd_actual[6];
    double i_actual[6];
    double i_control[6];
    double tool_vector_actual[6];
    double TCP_speed_actual[6];
    double TCP_force[6];
    double Tool_vector_target[6];
    double TCP_speed_target[6];
    double motor_temperatures[6];
    double joint_modes[6];
    double v_actual[6];
    double dummy[9][6];      // pads to 1440 bytes total
};
#pragma pack(pop)

class CR5Commander {
protected:
    static constexpr double PI = 3.14159265358979323846;
private:
    std::mutex mutex_;
    double current_joint_[6];
    double tool_vector_[6];
    RealTimeData real_time_data_;
    std::atomic<bool> is_running_;
    std::unique_ptr<std::thread> thread_;
    std::shared_ptr<TcpClient> motion_cmd_tcp_;
    std::shared_ptr<TcpClient> real_time_tcp_;
    std::shared_ptr<TcpClient> dash_board_tcp_;
    rclcpp::Logger logger_;

public:
    explicit CR5Commander(const std::string& ip)
        : current_joint_{}, tool_vector_{}, real_time_data_{}
        , is_running_(false)
        , logger_(rclcpp::get_logger("CR5Commander"))
    {
        motion_cmd_tcp_ = std::make_shared<TcpClient>(ip, 30003);
        real_time_tcp_  = std::make_shared<TcpClient>(ip, 30004);
        dash_board_tcp_ = std::make_shared<TcpClient>(ip, 29999);
    }

    ~CR5Commander() {
        is_running_ = false;
        if (thread_ && thread_->joinable()) thread_->join();
    }

    void getCurrentJointStatus(double* joint) {
        std::lock_guard<std::mutex> lock(mutex_);
        memcpy(joint, current_joint_, sizeof(current_joint_));
    }

    void getToolVectorActual(double* val) {
        std::lock_guard<std::mutex> lock(mutex_);
        memcpy(val, tool_vector_, sizeof(tool_vector_));
    }

    void recvTask() {
        uint32_t has_read;
        while (is_running_) {
            if (real_time_tcp_->isConnect()) {
                try {
                    if (real_time_tcp_->tcpRecv(&real_time_data_, sizeof(real_time_data_), has_read, 5000)) {
                        if (real_time_data_.len != 1440) continue;
                        std::lock_guard<std::mutex> lock(mutex_);
                        for (uint32_t i = 0; i < 6; i++)
                            current_joint_[i] = deg2Rad(real_time_data_.q_actual[i]);
                        memcpy(tool_vector_, real_time_data_.tool_vector_actual, sizeof(tool_vector_));
                    }
                } catch (const TcpClientException& err) {
                    real_time_tcp_->disConnect();
                    RCLCPP_ERROR(logger_, "real time tcp recv error: %s", err.what());
                }
            } else {
                try { real_time_tcp_->connect(); }
                catch (const TcpClientException& err) {
                    RCLCPP_ERROR(logger_, "real time tcp connect error: %s", err.what());
                    std::this_thread::sleep_for(std::chrono::seconds(3));
                }
            }
            if (!dash_board_tcp_->isConnect()) {
                try { dash_board_tcp_->connect(); }
                catch (const TcpClientException& err) {
                    RCLCPP_ERROR(logger_, "dashboard tcp connect error: %s", err.what());
                    std::this_thread::sleep_for(std::chrono::seconds(3));
                }
            }
            if (!motion_cmd_tcp_->isConnect()) {
                try { motion_cmd_tcp_->connect(); }
                catch (const TcpClientException& err) {
                    RCLCPP_ERROR(logger_, "motion tcp connect error: %s", err.what());
                    std::this_thread::sleep_for(std::chrono::seconds(3));
                }
            }
        }
    }

    void init() {
        is_running_ = true;
        thread_ = std::make_unique<std::thread>(&CR5Commander::recvTask, this);
        RCLCPP_INFO(logger_, "CR5Commander initialized — connecting to robot...");
    }

    bool isEnable() const { return real_time_data_.robot_mode == 5; }
    bool isConnected() const { return dash_board_tcp_->isConnect() && motion_cmd_tcp_->isConnect(); }

    void dashboardDoCmd(const char* cmd, int32_t& err_id) {
        std::vector<std::string> result;
        tcpDoCmd(dash_board_tcp_, cmd, err_id, result, logger_);
    }
    void dashboardDoCmd(const char* cmd, int32_t& err_id, std::vector<std::string>& result) {
        tcpDoCmd(dash_board_tcp_, cmd, err_id, result, logger_);
    }
    void motionDoCmd(const char* cmd, int32_t& err_id) {
        std::vector<std::string> result;
        tcpDoCmd(motion_cmd_tcp_, cmd, err_id, result, logger_);
    }
    void motionDoCmd(const char* cmd, int32_t& err_id, std::vector<std::string>& result) {
        tcpDoCmd(motion_cmd_tcp_, cmd, err_id, result, logger_);
    }

    static void parseString(const std::string& str, const std::string& send_cmd,
                            int32_t& err, std::vector<std::string>& result) {
        if (str.find(send_cmd) == std::string::npos)
            throw std::logic_error(std::string("Invalid string: ") + str);
        std::size_t pos = str.find(',');
        if (pos == std::string::npos)
            throw std::logic_error(std::string("No ',' found: ") + str);
        char buf[200];
        assert(pos < sizeof(buf));
        str.copy(buf, pos, 0); buf[pos] = 0;
        char* end;
        err = (int32_t)strtol(buf, &end, 10);
        if (*end != '\0') throw std::logic_error(std::string("Invalid err id: ") + str);
        std::size_t start_pos = str.find('{');
        if (start_pos == std::string::npos) throw std::logic_error(std::string("No '{': ") + str);
        std::size_t end_pos = str.find('}');
        if (end_pos == std::string::npos) throw std::logic_error(std::string("No '}': ") + str);
        assert(end_pos > start_pos);
        std::string inner = str.substr(start_pos + 1, end_pos - start_pos - 1);
        std::stringstream ss(inner);
        std::string token;
        while (std::getline(ss, token, ',')) result.push_back(token);
    }

private:
    static void tcpDoCmd(std::shared_ptr<TcpClient>& tcp, const char* cmd,
                         int32_t& err_id, std::vector<std::string>& result,
                         const rclcpp::Logger& logger) {
        try {
            uint32_t has_read;
            char buf[1024];
            memset(buf, 0, sizeof(buf));
            RCLCPP_INFO(logger, "tcp send: %s", cmd);
            tcp->tcpSend(cmd, strlen(cmd));
            char* recv_ptr = buf;
            while (true) {
                bool ok = tcp->tcpRecv(recv_ptr, 1, has_read, 0);
                if (!ok) { RCLCPP_ERROR(logger, "tcpDoCmd: recv timeout"); return; }
                if (*recv_ptr == ';') break;
                recv_ptr++;
            }
            RCLCPP_INFO(logger, "tcp recv: %s", buf);
            std::string cmd_str(cmd);
            std::size_t paren = cmd_str.find('(');
            std::string cmd_name = (paren != std::string::npos) ? cmd_str.substr(0, paren) : cmd_str;
            parseString(std::string(buf), cmd_name, err_id, result);
        } catch (const std::logic_error& err) {
            RCLCPP_ERROR(logger, "tcpDoCmd failed: %s", err.what());
        }
    }
    static inline double rad2Deg(double rad) { return rad * 180.0 / PI; }
    static inline double deg2Rad(double deg) { return deg * PI / 180.0; }
};
