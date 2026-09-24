#include <rclcpp/rclcpp.hpp>
#include "ros2_comm/msg/referee_data.hpp"
#include "interfaces/msg/chassis_command.hpp"

#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstring>
#include <vector>
#include <stdexcept>
#include <string>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>

constexpr uint8_t SEND_FRAME_HEADER = 0xAA; // 宿主机 -> ROS2
constexpr uint8_t SEND_FRAME_TAIL   = 0x5A;
constexpr uint8_t RECV_FRAME_HEADER = 0xBB; // ROS2 -> 宿主机
constexpr uint8_t RECV_FRAME_TAIL   = 0x5B;

// 端口配置
constexpr int LOCAL_PORT = 8888;   // Docker 监听端口
const char* TARGET_IP = "127.0.0.1"; // 宿主机 IP (host模式)
constexpr int TARGET_PORT = 8889;  // 宿主机监听端口

// 发送节拍：宿主机每轮只读一个数据报，发送速率必须显著低于它的排空速率，否则内核接收队列会积压成
// 陈旧指令 FIFO（队列满时丢新包、保留旧包），造成恒定的秒级延迟。两条发送路径都要求距上次发送
// >= MIN_SEND_INTERVAL，故速率硬上限 50 Hz、保活下限 20 Hz；用 steady_clock 避免 NTP 步进误判。
constexpr auto MIN_SEND_INTERVAL = std::chrono::milliseconds(20);   // 速率上限 50 Hz
constexpr auto KEEPALIVE_INTERVAL = std::chrono::milliseconds(50);  // 无新指令时的补发周期
constexpr auto KEEPALIVE_TICK = std::chrono::milliseconds(20);      // 保活定时器查询周期
constexpr int RECV_POLL_TIMEOUT_MS = 20;                           // 接收线程阻塞等待上限

// 对应宿主机的 ROS2_SEND_PACKET
struct __attribute__((packed)) RawRefereePacket
{
    uint16_t projectile_allowance_17mm;
    uint8_t  power_management_shooter_output;
    uint16_t current_hp;
    uint16_t outpost_HP;
    uint16_t base_HP;
    uint8_t  game_progess;
};

// 对应宿主机的 ROS2_RECV_PACKET
struct __attribute__((packed)) RawControlPacket
{
    float vx;
    float vy;
    uint8_t mode;
    uint8_t nav_state;
};
static_assert(sizeof(RawControlPacket) == 10, "Control payload must be 10 bytes");
static_assert(offsetof(RawControlPacket, mode) == 8 &&
              offsetof(RawControlPacket, nav_state) == 9,
              "Control mode must precede nav_state");

// ROS2 节点类 
class UdpBridgeNode : public rclcpp::Node
{
public:
    UdpBridgeNode() : Node("udp_bridge_node"), m_running(false), m_sock_fd(-1)
    {
        const int normal_mode = declare_parameter<int>("normal_mode", 4);
        if (normal_mode < 0 || normal_mode > 255) {
            throw std::invalid_argument("normal_mode must be in [0,255]");
        }
        normal_mode_ = static_cast<uint8_t>(normal_mode);
        // 初始化 ROS2 接口
        pub_referee_ = this->create_publisher<ros2_comm::msg::RefereeData>("referee_data", 10);
        
        // 深度 1：指令是最新值优先的量，积压的旧指令没有价值，不该在恢复后被补发
        sub_chassis_cmd_ = this->create_subscription<interfaces::msg::ChassisCommand>(
            "/nav_executor/chassis_cmd", 1,
            std::bind(&UdpBridgeNode::chassis_cmd_callback, this, std::placeholders::_1));

        // 初始化控制数据
        {
            std::lock_guard<std::mutex> lock(m_ctrl_mutex);
            m_latest_ctrl.vx = 0.0f;
            m_latest_ctrl.vy = 0.0f;
            m_latest_ctrl.mode = normal_mode_;
            m_latest_ctrl.nav_state = 0;
            m_last_cmd_time = this->now();
            m_last_send_time = std::chrono::steady_clock::now() - KEEPALIVE_INTERVAL;
        }

        // 初始化 UDP Socket
        if (init_udp())
        {
            // 接收线程只负责收，发送由底盘命令回调和保活定时器触发
            m_running = true;
            m_comm_thread = std::thread(&UdpBridgeNode::comm_thread_loop, this);
            m_keepalive_timer = this->create_wall_timer(
                KEEPALIVE_TICK, [this]() { maybe_send(KEEPALIVE_INTERVAL); });
            RCLCPP_INFO(this->get_logger(), "UDP Bridge Node started. Listening on port %d", LOCAL_PORT);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize UDP");
        }
    }

    ~UdpBridgeNode()
    {
        m_running = false;
        if (m_comm_thread.joinable())
        {
            m_comm_thread.join();
        }
        close_udp();
    }

private:
    // ROS2 接口
    rclcpp::Publisher<ros2_comm::msg::RefereeData>::SharedPtr pub_referee_;
    rclcpp::Subscription<interfaces::msg::ChassisCommand>::SharedPtr sub_chassis_cmd_;

    // 线程与状态
    std::thread m_comm_thread;
    std::atomic<bool> m_running;
    std::mutex m_ctrl_mutex;
    rclcpp::TimerBase::SharedPtr m_keepalive_timer;

    RawControlPacket m_latest_ctrl;
    uint8_t normal_mode_{4};
    rclcpp::Time m_last_cmd_time;
    std::chrono::steady_clock::time_point m_last_send_time;
    // 0.3 s 超时：0.4 m/s 下失联最多多滑 12 cm；不得放宽，滑行距离与超时成正比
    static constexpr double CMD_TIMEOUT_SEC = 0.3;

    // UDP
    int m_sock_fd;
    sockaddr_in m_target_addr; // 宿主机地址

    // 初始化 UDP
    bool init_udp()
    {
        m_sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
        if (m_sock_fd < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to create socket: %s", strerror(errno));
            return false;
        }

        // 设置非阻塞
        int flags = fcntl(m_sock_fd, F_GETFL, 0);
        fcntl(m_sock_fd, F_SETFL, flags | O_NONBLOCK);

        // 绑定本地端口
        sockaddr_in local_addr;
        memset(&local_addr, 0, sizeof(local_addr));
        local_addr.sin_family = AF_INET;
        local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
        local_addr.sin_port = htons(LOCAL_PORT);

        if (bind(m_sock_fd, (sockaddr*)&local_addr, sizeof(local_addr)) < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to bind port %d: %s", LOCAL_PORT, strerror(errno));
            close(m_sock_fd);
            return false;
        }

        // 设置目标地址 (宿主机)
        memset(&m_target_addr, 0, sizeof(m_target_addr));
        m_target_addr.sin_family = AF_INET;
        m_target_addr.sin_port = htons(TARGET_PORT);
        inet_pton(AF_INET, TARGET_IP, &m_target_addr.sin_addr);

        RCLCPP_INFO(this->get_logger(), "UDP initialized. Target: %s:%d", TARGET_IP, TARGET_PORT);
        return true;
    }

    void close_udp()
    {
        if (m_sock_fd >= 0)
        {
            close(m_sock_fd);
            m_sock_fd = -1;
        }
    }

    // 底盘命令回调：更新最新值后立即尝试发送，新指令不必等定时器
    void chassis_cmd_callback(const interfaces::msg::ChassisCommand::SharedPtr msg)
    {
        {
            std::lock_guard<std::mutex> lock(m_ctrl_mutex);
            m_latest_ctrl.vx = msg->vx;
            m_latest_ctrl.vy = msg->vy;
            m_latest_ctrl.mode = msg->mode;
            if (msg->vx == 0 && msg->vy == 0) {
                m_latest_ctrl.nav_state = 0;
            } else {
                m_latest_ctrl.nav_state = 1;
            }
            m_last_cmd_time = this->now();
        }
        maybe_send(MIN_SEND_INTERVAL);
    }

    // 唯一的发送入口。距上次发送不足 min_gap 就跳过，本次的值留给后续触发带出去
    // （最新值优先：跳过不丢信息，因为发出去的总是当前最新的 m_latest_ctrl）
    void maybe_send(const std::chrono::steady_clock::duration min_gap)
    {
        RawControlPacket pkt;
        bool timed_out = false;
        {
            std::lock_guard<std::mutex> lock(m_ctrl_mutex);
            const auto now = std::chrono::steady_clock::now();
            if (now - m_last_send_time < min_gap) {
                return;
            }
            m_last_send_time = now;
            timed_out = (this->now() - m_last_cmd_time).seconds() > CMD_TIMEOUT_SEC;
            pkt = timed_out ? RawControlPacket {0.0f, 0.0f, m_latest_ctrl.mode, 0} : m_latest_ctrl;
        }
        // 系统调用放在锁外，避免阻塞 /cmd_vel 回调
        send_to_host(pkt);
        if (timed_out) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "chassis command timeout, sending zero velocity");
        }
    }

    // 发送数据给宿主机
    bool send_to_host(const RawControlPacket& pkt)
    {
        if (m_sock_fd < 0) return false;
        const size_t payload_size = sizeof(pkt);
        uint8_t frame[sizeof(RawControlPacket) + 3];
        frame[0] = RECV_FRAME_HEADER;
        frame[1] = static_cast<uint8_t>(payload_size);
        memcpy(frame + 2, &pkt, payload_size);
        frame[2 + payload_size] = RECV_FRAME_TAIL;

        const size_t frame_size = payload_size + 3;
        const ssize_t sent = sendto(m_sock_fd, frame, frame_size, 0,
                                    (sockaddr*)&m_target_addr, sizeof(m_target_addr));
        if (sent != static_cast<ssize_t>(frame_size))
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "sendto failed: %s", strerror(errno));
            return false;
        }
        return true;
    }

    // 接收线程。只负责收：阻塞等待到有数据为止，不再空转，也不再驱动发送
    void comm_thread_loop()
    {
        RCLCPP_INFO(this->get_logger(), "Receive thread running");

        while (m_running)
        {
            pollfd pfd {m_sock_fd, POLLIN, 0};
            if (poll(&pfd, 1, RECV_POLL_TIMEOUT_MS) <= 0)
            {
                continue; // 超时或被信号打断，回去检查 m_running
            }

            // 排空接收队列，只保留最后一帧。裁判数据是状态量而非事件，旧帧无价值；
            // 每轮只读一个的话，一旦对端发得比这里读得快，队列就会累积成陈旧数据
            uint8_t buffer[1024];
            uint8_t latest[1024];
            ssize_t latest_n = -1;
            while (true)
            {
                const ssize_t n = recvfrom(m_sock_fd, buffer, sizeof(buffer), 0, nullptr, nullptr);
                if (n <= 0)
                {
                    break; // EAGAIN：队列已空
                }
                memcpy(latest, buffer, static_cast<size_t>(n));
                latest_n = n;
            }
            if (latest_n > 0)
            {
                handle_referee_frame(latest, latest_n);
            }
        }

        RCLCPP_INFO(this->get_logger(), "Receive thread stopped");
    }

    // 校验并发布一帧裁判系统数据
    void handle_referee_frame(const uint8_t* buffer, const ssize_t n)
    {
        if (n < 3 || buffer[0] != SEND_FRAME_HEADER) return;
        const uint8_t data_len = buffer[1];
        if (n < static_cast<ssize_t>(2 + data_len + 1)) return;
        if (buffer[2 + data_len] != SEND_FRAME_TAIL) return;
        if (data_len != sizeof(RawRefereePacket)) return;

        RawRefereePacket raw;
        memcpy(&raw, &buffer[2], sizeof(raw));

        auto msg = ros2_comm::msg::RefereeData();
        msg.projectile_allowance_17mm = raw.projectile_allowance_17mm;
        msg.power_management_shooter_output = raw.power_management_shooter_output;
        msg.current_hp = raw.current_hp;
        msg.outpost_hp = raw.outpost_HP;
        msg.base_hp = raw.base_HP;
        msg.game_progess = raw.game_progess;
        pub_referee_->publish(msg);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UdpBridgeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
