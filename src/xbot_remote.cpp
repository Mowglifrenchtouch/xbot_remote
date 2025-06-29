#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nlohmann/json.hpp>
#include <websocketpp/server.hpp>
#include <websocketpp/config/asio_no_tls.hpp>
#include <thread>
#include <memory>
#include <mutex>
#include <iostream>

using json = nlohmann::json;
using websocket_server = websocketpp::server<websocketpp::config::asio>;
using connection_hdl = websocketpp::connection_hdl;
using message_ptr = websocket_server::message_ptr;

class XBotRemoteNode {
public:
    XBotRemoteNode()
        : vx_max_(1.0), vz_max_(3.2), vx_scale_(0.4), vz_scale_(3.2), nh_("~") {
        // Load parameters
        nh_.param("VxMax", vx_max_, vx_max_);
        nh_.param("VzMax", vz_max_, vz_max_);
        nh_.param("VxScale", vx_scale_, vx_scale_);
        nh_.param("VzScale", vz_scale_, vz_scale_);

        // Create ROS publisher
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("xbot_remote/cmd_vel", 1);

        // Set up WebSocket server
        ws_server_.set_access_channels(websocketpp::log::alevel::none);
        ws_server_.init_asio();
        ws_server_.set_reuse_addr(true);
        ws_server_.set_message_handler(
            std::bind(&XBotRemoteNode::onMessage, this, std::placeholders::_1, std::placeholders::_2)
        );
    }

    void start() {
        // Start server thread
        ws_thread_ = std::thread([this]() {
            try {
                ws_server_.listen(9002);
                ws_server_.start_accept();
                while (ros::ok()) {
                    ws_server_.run_one();
                }
            } catch (const std::exception& e) {
                ROS_ERROR_STREAM("WebSocket error: " << e.what());
            }
        });
    }

    void spin() {
        ros::spin();
        shutdown();
    }

    ~XBotRemoteNode() {
        shutdown();
    }

private:
    void shutdown() {
        if (ws_server_.is_listening()) {
            ROS_INFO("Stopping WebSocket server...");
            ws_server_.stop_listening();
        }
        if (ws_thread_.joinable()) {
            ws_thread_.join();
            ROS_INFO("WebSocket thread shut down.");
        }
    }

    void onMessage(connection_hdl hdl, message_ptr msg) {
        try {
            auto payload = json::from_bson(msg->get_payload());
            double vx = payload["vx"].get<double>();
            double vz = payload["vz"].get<double>();

            ROS_INFO_STREAM_THROTTLE(0.5, "vx: " << vx << " vz: " << vz);

            geometry_msgs::Twist twist;

            // Normalize and shape curve
            twist.linear.x = clampAndShape(vx / vx_max_);
            twist.angular.z = clampAndShape(vz / vz_max_);

            cmd_vel_pub_.publish(twist);
        } catch (const std::exception& e) {
            ROS_ERROR_STREAM("JSON decode error: " << e.what());
        }
    }

    static double clampAndShape(double value) {
        value *= std::abs(value); // Square response
        return std::max(-1.0, std::min(1.0, value)); // Clamp [-1, 1]
    }

    // ROS
    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub_;

    // Parameters
    double vx_max_, vz_max_;
    double vx_scale_, vz_scale_;

    // WebSocket server
    websocket_server ws_server_;
    std::thread ws_thread_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "xbot_remote");
    XBotRemoteNode node;
    node.start();
    node.spin();
    return 0;
}
