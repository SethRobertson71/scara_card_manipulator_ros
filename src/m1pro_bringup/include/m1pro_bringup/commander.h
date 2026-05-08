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
#include <chrono>
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
    double q_actual[6];
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
    double dummy[9][6];
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
            // --- real-time feedback (30004) ---
            if (real_time_tcp_->isConnect()) {
                try {
                    if (real_time_tcp_->tcpRecv(&real_time_data_, sizeof(real_time_data_), has_read, 5000)) {
                        if (real_time_data_.len != 1440) continue;
                        std::lock_guard<std::mutex> lock(mutex_);
                        // Firmware q_actual: [J1=base, J2=elbow, J3=zslide_mm, J4=eef]
                        // ROS joint order:  [joint1=zslide_m, joint2=base, joint3=elbow, joint4=eef]
                        current_joint_[0] = real_time_data_.q_actual[2] / 1000.0;
                        current_joint_[1] = deg2Rad(real_time_data_.q_actual[0]);
                        current_joint_[2] = deg2Rad(real_time_data_.q_actual[1]);
                        current_joint_[3] = deg2Rad(real_time_data_.q_actual[3]);
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
            // --- dashboard reconnect (29999) ---
            if (!dash_board_tcp_->isConnect()) {
                try { dash_board_tcp_->connect(); }
                catch (const TcpClientException& err) {
                    RCLCPP_ERROR(logger_, "dashboard tcp reconnect error: %s", err.what());
                    std::this_thread::sleep_for(std::chrono::seconds(3));
                }
            }
            // --- motion reconnect (30003) ---
            if (!motion_cmd_tcp_->isConnect()) {
                try { motion_cmd_tcp_->connect(); }
                catch (const TcpClientException& err) {
                    RCLCPP_ERROR(logger_, "motion tcp reconnect error: %s", err.what());
                    std::this_thread::sleep_for(std::chrono::seconds(3));
                }
            }
        }
    }

    void init() {
        is_running_ = true;
        RCLCPP_INFO(logger_, "CR5Commander initialized — polling robot TCP ports...");

        // Poll dashboard (29999) and motion (30003) directly until both accept
        // connections — up to 90 seconds to cover full robot boot sequence.
        // The feedback thread (30004) is started only after both are confirmed up.
        auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(90);
        while (std::chrono::steady_clock::now() < deadline) {
            bool db_ok = false, mo_ok = false;

            if (!dash_board_tcp_->isConnect()) {
                try { dash_board_tcp_->connect(); db_ok = true; }
                catch (const TcpClientException&) { dash_board_tcp_->disConnect(); }
            } else {
                db_ok = true;
            }

            if (!motion_cmd_tcp_->isConnect()) {
                try { motion_cmd_tcp_->connect(); mo_ok = true; }
                catch (const TcpClientException&) { motion_cmd_tcp_->disConnect(); }
            } else {
                mo_ok = true;
            }

            if (db_ok && mo_ok) {
                RCLCPP_INFO(logger_, "Robot TCP connections established — starting feedback thread");
                thread_ = std::make_unique<std::thread>(&CR5Commander::recvTask, this);
                return;
            }

            RCLCPP_INFO(logger_, "Waiting for robot TCP ports (dashboard:%s motion:%s)...",
                        db_ok ? "UP" : "DOWN", mo_ok ? "UP" : "DOWN");
            std::this_thread::sleep_for(std::chrono::seconds(3));
        }
        throw std::runtime_error("Timed out waiting for robot TCP connections after 90s");
    }

    bool isEnable() const { return real_time_data_.robot_mode == 5; }
    bool isConnected() const {
        return dash_board_tcp_->isConnect() && motion_cmd_tcp_->isConnect();
    }

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
        err_id = -1;
        // Wait up to 10s for socket to be connected before sending
        auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
        while (!tcp->isConnect()) {
            if (std::chrono::steady_clock::now() > deadline) {
                RCLCPP_ERROR(logger, "tcpDoCmd: socket not connected, dropping cmd: %s", cmd);
                return;
            }
            RCLCPP_WARN(logger, "tcpDoCmd: waiting for socket reconnect...");
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
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
            return;
        } catch (const std::logic_error& err) {
            RCLCPP_ERROR(logger, "tcpDoCmd failed: %s", err.what());
            err_id = -1;
        } catch (const std::exception& err) {
            RCLCPP_ERROR(logger, "tcpDoCmd unexpected error: %s", err.what());
            err_id = -1;
        }
    }

    static inline double rad2Deg(double rad) { return rad * 180.0 / PI; }
    static inline double deg2Rad(double deg) { return deg * PI / 180.0; }
};
