#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_interfaces/srv/i2c_command.hpp"
#include "geometry_msgs/msg/vector3.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

// The node is implemented as a class that inherits from rclcpp::Node [4]
class VectorLogicClient : public rclcpp::Node {
public:
    VectorLogicClient() : Node("vector_logic_client"), is_initialized_(false) {
        // Create publisher for the final processed vector [5]
        pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/sensor/accel/vector", 10);
        
        // Create the client to telepathically communicate with the Raspberry Pi
        client_ = this->create_client<sensor_interfaces::srv::I2cCommand>("/sensor/i2c_command");
        
        // Control loop executing at 10 Hz (100ms) [1]
        timer_ = this->create_wall_timer(100ms, std::bind(&VectorLogicClient::control_cycle, this));
    }

private:
    bool is_initialized_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_;
    rclcpp::Client<sensor_interfaces::srv::I2cCommand>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;

    // The FSM control logic [2]
    void control_cycle() {
        if (!client_->wait_for_service(1s)) {
            RCLCPP_WARN(this->get_logger(), "Waiting for Shoggoth (Raspberry Pi) to awaken..."); //maybe tirar
            return;
        }

        auto request = std::make_shared<sensor_interfaces::srv::I2cCommand::Request>();
        request->device_addr = 0x1D; // MMA7455 I2C Address

        if (!is_initialized_) {
            // INIT STATE: Send Write command (Wakes up sensor to +-2g mode)
            request->is_read = false;
            request->reg_addr = 0x16;
            request->write_data = {0x25};
            
            client_->async_send_request(request, [this](rclcpp::Client<sensor_interfaces::srv::I2cCommand>::SharedFuture future) {
                if (future.get()->success) {
                    RCLCPP_INFO(this->get_logger(), "Sensor initialized successfully.");
                    this->is_initialized_ = true;
                }
            });
        } else {
            // READ STATE: Request 3 bytes starting from X-axis register (0x06)
            request->is_read = true;
            request->reg_addr = 0x06;
            request->length = 3;

            client_->async_send_request(request, std::bind(&VectorLogicClient::response_callback, this, _1));
        }
    }

    void response_callback(rclcpp::Client<sensor_interfaces::srv::I2cCommand>::SharedFuture future) {
        auto response = future.get();
        if (!response->success || response->read_data.size() < 3) return;

        // Extract raw bytes returned by the Raspberry Pi
        int x_raw = response->read_data[0];
        int y_raw = response->read_data[1];
        int z_raw = response->read_data[2];

        // Process two's complement mathematically on the Workstation
        if (x_raw > 127) x_raw -= 256;
        if (y_raw > 127) y_raw -= 256;
        if (z_raw > 127) z_raw -= 256;

        auto vector_msg = geometry_msgs::msg::Vector3();
        vector_msg.x = x_raw / 64.0;
        vector_msg.y = y_raw / 64.0;
        vector_msg.z = z_raw / 64.0;

        pub_->publish(vector_msg);
        RCLCPP_INFO(this->get_logger(), "Vector -> X: %+.3f g, Y: %+.3f g, Z: %+.3f g", 
                    vector_msg.x, vector_msg.y, vector_msg.z);
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VectorLogicClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}