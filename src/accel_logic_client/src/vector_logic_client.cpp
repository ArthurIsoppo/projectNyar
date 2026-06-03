#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "sensor_interfaces/srv/i2c_command.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "geometry_msgs/msg/vector3.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

enum class RobotState {
    INITIALIZING,
    RUNNING,
    ERROR
};

class VectorLogicClient : public rclcpp::Node {
public:
    VectorLogicClient() : Node("vector_logic_client"), current_state_(RobotState::INITIALIZING) {
        pub_    = this->create_publisher<geometry_msgs::msg::Vector3>("/sensor/accel/vector", 10);
        client_ = this->create_client<sensor_interfaces::srv::I2cCommand>("/sensor/i2c_command");

        // 20 Hz loop drives the entire I2C transaction cycle
        fsm_timer_ = this->create_wall_timer(50ms, std::bind(&VectorLogicClient::fsm_cycle, this));
    }

private:
    RobotState current_state_;
    int  error_counter_       = 0;
    bool waiting_for_response_ = false;

    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_;
    rclcpp::Client<sensor_interfaces::srv::I2cCommand>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr fsm_timer_;

    void fsm_cycle() {
        if (waiting_for_response_) return;

        if (!client_->wait_for_service(0s)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Procurando Raspberry Pi na rede...");
            return;
        }

        auto request = std::make_shared<sensor_interfaces::srv::I2cCommand::Request>();
        request->device_addr = 0x1D; // MMA7455

        switch (current_state_) {
            case RobotState::INITIALIZING:
                // Mode Control (0x16): acorda o sensor em modo de medição 8-bit
                request->write_data = {0x16, 0x01};
                request->length = 0;

                waiting_for_response_ = true;
                client_->async_send_request(request,
                    [this](rclcpp::Client<sensor_interfaces::srv::I2cCommand>::SharedFuture future) {
                        waiting_for_response_ = false;
                        if (future.get()->success) {
                            RCLCPP_INFO(this->get_logger(), "Acelerômetro inicializado.");
                            current_state_ = RobotState::RUNNING;
                        }
                    });
                break;

            case RobotState::RUNNING:
                // Lê 3 bytes a partir de XOUT8 (0x06)
                request->write_data = {0x06};
                request->length = 3;

                waiting_for_response_ = true;
                client_->async_send_request(request,
                    [this](rclcpp::Client<sensor_interfaces::srv::I2cCommand>::SharedFuture future) {
                        waiting_for_response_ = false;
                        auto response = future.get();
                        if (response->success && response->read_data.size() == 3) {
                            error_counter_ = 0;
                            process_and_publish(response->read_data);
                        } else {
                            if (++error_counter_ >= 10) {
                                RCLCPP_ERROR(this->get_logger(),
                                    "Falha persistente na leitura. Sensor desconectado?");
                                current_state_ = RobotState::ERROR;
                            } else {
                                RCLCPP_WARN(this->get_logger(),
                                    "Falha na leitura (%d/10)", error_counter_);
                            }
                        }
                    });
                break;

            case RobotState::ERROR:
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "FSM em estado de ERRO.");
                break;
        }
    }

    void process_and_publish(const std::vector<uint8_t>& data) {
        int x_raw = data[0];
        int y_raw = data[1];
        int z_raw = data[2];

        // Complemento de 2 para valores signed de 8 bits
        if (x_raw > 127) x_raw -= 256;
        if (y_raw > 127) y_raw -= 256;
        if (z_raw > 127) z_raw -= 256;

        auto msg = geometry_msgs::msg::Vector3();
        msg.x = x_raw / 64.0;
        msg.y = y_raw / 64.0;
        msg.z = z_raw / 64.0;

        RCLCPP_INFO(this->get_logger(), "[g] X: %.2f | Y: %.2f | Z: %.2f", msg.x, msg.y, msg.z);
        pub_->publish(msg);
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VectorLogicClient>());
    rclcpp::shutdown();
    return 0;
}
