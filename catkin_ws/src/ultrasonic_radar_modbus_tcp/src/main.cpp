// 超声波雷达 Modbus TCP 节点入口。
//
// 职责边界：准备 ROS 参数、组装服务器列表、构造节点、创建发布器，并在
// 事件循环里直观地完成四个大步骤：queryRadars()（问询/读取）、发布聚合话题、
// 发布每台服务器的在线状态、处理回调。单台服务器的重连、寄存器读取、距离换算
// 等细节封装在 UltrasonicRadarNode 内。
//
// 话题：
//   /perception_sensors/ultrasonic_radar_datas                    —— 数据，每轮一条
//   /perception_sensors/ultrasonic_radar_status/<服务器 name>     —— 状态，跳变时一条
//
// 参数来源分两类：
//   - 可由 launch 覆盖：网关 IP、两个端口、轮询周期、超时、重连间隔、打印开关；
//   - 固定在本文件：从机地址、寄存器地址/数量、距离阈值、话题名与发布器。
// 参数默认值以字面量直接写在这里，现场接法一目了然。

#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <clocale>
#include <string>
#include <vector>

#include <ultrasonic_radar_modbus_tcp/ultrasonic_radar_datas.h>

#include "ultrasonic_radar_modbus_tcp/ultrasonic_radar_node.h"

int main(int argc, char** argv) {
    std::setlocale(LC_ALL, "");
    ros::init(argc, argv, "ultrasonic_radar_modbus_tcp");
    // 只用一个主句柄：发布话题，同时读取 launch 中定义在全局命名空间的参数。
    ros::NodeHandle nh;

    // —— 一、串口服务器 ——
    // 固定两个 Modbus 网关端口，均在同一台 M04 上；读不到的端口启动时跳过。
    // 端口与 HC 通道对应关系：502→HC0，503→HC1，504→HC2，505→HC3。
    std::string gateway_ip = "192.168.1.12";  // M04 串口服务器 IP（现场值，launch 可覆盖）
    int port_1 = 503;                          // 网关端口，对应 HC1
    int port_2 = 502;                          // 网关端口，对应 HC0
    const int slave_id = 1;  // RS485 侧 DYP 电应普超声波雷达模组从机地址，固定值（0 为广播地址）

    // launch 在全局命名空间设置了同名参数时覆盖上面的默认值。
    nh.param("gateway_ip", gateway_ip, gateway_ip);
    nh.param("port_1", port_1, port_1);
    nh.param("port_2", port_2, port_2);

    // 两个端口必须不同：对同一端口建两条连接会让两路请求互相抢答、
    // 事务号错乱，表现为随机超时或数据串路。
    if (port_1 == port_2) {
        ROS_FATAL("port_1 与 port_2 相同（%d），不能对同一端口建两条连接",
                port_1);
        return 1;
    }

    // 每个端口对应一组 4 路探头，现场接线固定为：
    //   502 → 车体左侧四个探头，name = left_radar_502；
    //   503 → 车体右侧四个探头，name = right_radar_503。
    // 发布到话题后订阅方据此区分左右。若 launch 把端口改成 502/503 之外的值，
    // 只保留 radar_<端口>，宁可不标左右也不标错。
    std::vector<ultrasonic_radar_modbus_tcp::ServerConfig> servers;
    for (const int port : {port_1, port_2}) {
        ultrasonic_radar_modbus_tcp::ServerConfig server;
        std::string side;
        if (port == 502) {
            side = "left_";
        } else if (port == 503) {
            side = "right_";
        }
        server.name = side + "radar_" + std::to_string(port);
        server.host = gateway_ip;
        server.port = port;
        server.slave_id = slave_id;
        servers.push_back(server);
    }

    // —— 二、话题 ——
    // 只发布一个聚合话题：每轮一条，radars[] 包含本轮所有读取成功的服务器，
    // 每条数据的 name（如 left_radar_502）区分来源。话题名固定，不通过 launch 配置。
    const std::string array_topic = "/perception_sensors/ultrasonic_radar_datas";

    ultrasonic_radar_modbus_tcp::RadarNodeConfig config;

    // —— 三、寄存器（固定值，不通过 launch 配置）——
    // DYP 电应普手册中 1~4 号探头对应 0x0106~0x0109，因此从 0x0106 起连续读 4 个。
    config.start_register = 0x0106;  // 1 号探头寄存器（十进制 262）
    config.probe_count = 4;          // 连续读取 4 路：前/侧1/侧2/后

    // —— 四、轮询与超时 ——
    // poll_interval 是一整轮问询的周期；timeout 同时用于 TCP 连接和单次应答。
    config.poll_interval = 0.3;       // 秒
    config.timeout = 0.5;             // 秒
    config.reconnect_interval = 1.0;  // 已探测到的服务器断线后的重连间隔（秒）
    config.reprobe_interval = 30.0;   // 启动探测失败端口的重探间隔（秒）；0=关闭
    nh.param("poll_interval", config.poll_interval, config.poll_interval);
    nh.param("timeout", config.timeout, config.timeout);
    nh.param("reconnect_interval", config.reconnect_interval,
            config.reconnect_interval);
    nh.param("reprobe_interval", config.reprobe_interval,
            config.reprobe_interval);

    // —— 五、距离换算（固定值）——
    // 原始读数单位 mm；≤ 0.1 m 视为无有效回波（含 0 mm），统一发布 -1.0 m。
    // -1.0 是哨兵值而非真实距离，订阅方应使用 < 0 判断"该路无目标"。
    config.min_valid_distance = 0.1;  // 米
    config.invalid_distance = -1.0;   // 米

    // —— 六、调试开关 ——
    // print_data：每轮打印 4 路原始 mm 和换算后的距离，现场默认打开；
    // debug_frames：打印 Modbus TCP 收发字节，仅在排查协议问题时打开。
    config.print_data = true;
    config.debug_frames = false;
    nh.param("print_data", config.print_data, config.print_data);
    nh.param("debug_frames", config.debug_frames, config.debug_frames);

    // —— 七、构造超声波雷达的tcp通讯节点 ——
    // 把超声波服务器列表servers与运行参数config交给节点类，完成初始化
    ultrasonic_radar_modbus_tcp::UltrasonicRadarNode node(servers, config);
    // start() 校验参数并探测服务器，失败则退出。
    std::string error;
    if (!node.start(&error)) {
        ROS_FATAL("节点启动失败: %s", error.c_str());
        return 1;
    }

    // —— 八、发布器 ——
    // 聚合话题：每轮发布一条 ultrasonic_radar_datas，radars[] 内每个元素对应
    ros::Publisher array_pub =
        nh.advertise<ultrasonic_radar_modbus_tcp::ultrasonic_radar_datas>(array_topic, 10);

    // 每台服务器一个在线状态话题，话题名直接用服务器 name 区分子命名空间：
    //   /perception_sensors/ultrasonic_radar_status/left_radar_502
    //   /perception_sensors/ultrasonic_radar_status/right_radar_503
    // 类型 std_msgs/Bool：true=本轮读到数据，false=超时/离线/尚未上线。
    // latched=true：后启动的订阅者也能立即拿到最近一次状态，不必等下一次跳变。
    // 发布器顺序与 servers 一致，主循环按同样下标对应 serverStatuses()。
    const std::string status_topic_prefix =
        "/perception_sensors/ultrasonic_radar_status/";
    std::vector<ros::Publisher> status_pubs;
    status_pubs.reserve(servers.size());
    for (const auto& server : servers) {
        status_pubs.push_back(nh.advertise<std_msgs::Bool>(
            status_topic_prefix + server.name, 10, true));
    }

    // 上一轮已发布的状态与「是否已发过首条」。首轮无论状态如何都发一次，
    // 让 latched 话题从一开始就有值；之后只在跳变时发布。
    std::vector<bool> last_online(servers.size(), false);
    std::vector<bool> status_published(servers.size(), false);

    // —— 九、事件循环 ——
    // 单线程循环，每轮四个大步骤；周期由 poll_interval 控制。
    // ros::Rate 按固定频率休眠；若某轮读取耗时超过周期，sleep() 会立即返回
    // 进入下一轮，不补发旧周期，避免消息堆积。
    ros::Rate rate(1.0 / config.poll_interval);
    while (ros::ok()) {
        // 1. 问询 + 读取所有可用服务器；内部按服务器重连或跳过，失败不抛出。
        const auto batch = node.queryRadars();

        // 2. 发布聚合话题：每轮必发一条，radars[] 为空表示本轮没有读到任何服务器。
        array_pub.publish(batch);

        // 3. 发布每台服务器的在线状态：首轮必发，之后只在 true/false 跳变时发一条。
        //    online 由上面的 queryRadars() 按本轮读取结果刷新；启动探测失败、
        //    后台重探中、读超时、网关忙都会是 false。
        const auto statuses = node.serverStatuses();
        for (size_t i = 0; i < statuses.size(); ++i) {
            if (status_published[i] && statuses[i].online == last_online[i]) {
                continue;
            }

            std_msgs::Bool status_msg;
            status_msg.data = statuses[i].online;
            status_pubs[i].publish(status_msg);

            if (statuses[i].online) {
                ROS_INFO("%s 在线", statuses[i].name.c_str());
            } else {
                ROS_WARN("%s 超时/离线", statuses[i].name.c_str());
            }

            last_online[i] = statuses[i].online;
            status_published[i] = true;
        }

        // 4. 处理 ROS 回调并休眠到下一周期。本节点当前没有订阅者，
        //    spinOnce() 保留是为了后续扩展回调时无需改动循环结构。
        ros::spinOnce();
        rate.sleep();
    }

    // 退出时 node 析构，ModbusTcpClient 会关闭所有 socket。
    return 0;
}
