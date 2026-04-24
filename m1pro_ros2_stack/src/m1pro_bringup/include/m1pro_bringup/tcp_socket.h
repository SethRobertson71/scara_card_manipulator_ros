#pragma once
#include <atomic>
#include <stdint.h>
#include <exception>
#include <stdexcept>
#include <string>

class TcpClientException : public std::logic_error {
public:
    explicit TcpClientException(const std::string& what) : std::logic_error(what) {}
};

class TcpClient {
private:
    int fd_;
    uint16_t port_;
    std::string ip_;
    std::atomic<bool> is_connected_;
public:
    TcpClient(std::string ip, uint16_t port);
    ~TcpClient();
    void close();
    void connect();
    void disConnect();
    bool isConnect() const;
    void tcpSend(const void* buf, uint32_t len);
    bool tcpRecv(void* buf, uint32_t len, uint32_t& has_read, uint32_t timeout);
    std::string toString();
};
