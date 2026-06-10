#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "sensor_interfaces/srv/i2c_command.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "geometry_msgs/msg/vector3.hpp"

//For statistics
#include <numeric>
#include <cmath>
#include <algorithm>
#include <vector>
// ---

using namespace std::chrono_literals;
using std::placeholders::_1;

enum class STATE {
    INITIALIZING,
    RUNNING,
    ERROR
};

class VectorLogicClient : public rclcpp::Node {
public:
    VectorLogicClient() : Node("vector_logic_client"), current_state_(STATE::INITIALIZING) {
        pub_    = this->create_publisher<geometry_msgs::msg::Vector3>("/sensor/accel/vector", 10);
        client_ = this->create_client<sensor_interfaces::srv::I2cCommand>("/sensor/i2c_command");

        // 20 Hz loop drives the entire I2C transaction cycle
        fsm_timer_ = this->create_wall_timer(50ms, std::bind(&VectorLogicClient::fsm_cycle, this));
    }

private:
    STATE current_state_;
    int  error_counter_       = 0;
    bool waiting_for_response_ = false;

    // statistics
    double request_time_ = 0.0;
    std::vector<double> latencies_;
    // ---

    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_;
    rclcpp::Client<sensor_interfaces::srv::I2cCommand>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr fsm_timer_;

    void fsm_cycle() {
        if (waiting_for_response_) return;

        if (!client_->wait_for_service(0s)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "searching for Shoggoth on the network...");
            return;
        }

        auto request = std::make_shared<sensor_interfaces::srv::I2cCommand::Request>();
        request->device_addr = 0x1D; // MMA7455

        switch (current_state_) {
            case STATE::INITIALIZING:
                // Mode Control (0x16): wake up sensor
                request->write_data = {0x16, 0x01};
                request->length = 0;

                waiting_for_response_ = true;
                client_->async_send_request(request,
                    [this](rclcpp::Client<sensor_interfaces::srv::I2cCommand>::SharedFuture future) {
                        waiting_for_response_ = false;
                        if (future.get()->success) {
                            RCLCPP_INFO(this->get_logger(), "accelerometer initialized");
                            current_state_ = STATE::RUNNING;
                        }
                    });
                break;

            case STATE::RUNNING:
                // read 3 bytes starting on XOUT8 (0x06)
                request->write_data = {0x06};
                request->length = 3;

                // statistics
                request_time_ = this->now().seconds();
                // ---

                waiting_for_response_ = true;
                client_->async_send_request(request,
                    [this](rclcpp::Client<sensor_interfaces::srv::I2cCommand>::SharedFuture future) {
                        waiting_for_response_ = false;

                        // statistics
                        double arrival_time = this->now().seconds();
                        double rtt = arrival_time - request_time_;
                        double latencia_ida_ms = (rtt / 2.0) * 1000.0;

                        latencies_.push_back(latencia_ida_ms);

                        double soma = std::accumulate(latencies_.begin(), latencies_.end(), 0.0);
                        double media = soma / latencies_.size();

                        double soma_diff_quadrado = 0.0;
                        for (double lat : latencies_) {
                            soma_diff_quadrado += (lat - media) * (lat - media);
                        }
                        double desvio = std::sqrt(soma_diff_quadrado / latencies_.size());

                        double p99 = 0.0;
                        if (!latencies_.empty()) {
                            std::vector<double> lat_ordenadas = latencies_;
                            std::sort(lat_ordenadas.begin(), lat_ordenadas.end());
                            size_t index = std::floor(0.99 * lat_ordenadas.size());
                            if(index >= lat_ordenadas.size()) index = lat_ordenadas.size() - 1;
                            p99 = lat_ordenadas[index];
                        }

                        RCLCPP_INFO(this->get_logger(),
                            "Lat: %.2f ms | Média: %.2f ms | Desvio (Jitter): %.2f ms | Cauda (P99): %.2f ms",
                            latencia_ida_ms, media, desvio, p99);
                        // --------------------------------------

                        auto response = future.get();
                        if (response->success && response->read_data.size() == 3) {
                            error_counter_ = 0;
                            process_and_publish(response->read_data);
                            RCLCPP_INFO(this->get_logger(), "[MARK] Timestamp - Data recived from Rasp");
                        } else {
                            if (++error_counter_ >= 10) {
                                RCLCPP_ERROR(this->get_logger(),
                                    "persistent read failure. sensor disconnected?");
                                current_state_ = STATE::ERROR;
                            } else {
                                RCLCPP_WARN(this->get_logger(),
                                    "read failure (%d/10)", error_counter_);
                            }
                        }
                    });
                break;

            case STATE::ERROR:
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "FSM in ERROR state");
                break;
        }
    }

    void process_and_publish(const std::vector<uint8_t>& data) {
        int x_raw = data[0];
        int y_raw = data[1];
        int z_raw = data[2];

        if (x_raw > 127) x_raw -= 256;
        if (y_raw > 127) y_raw -= 256;
        if (z_raw > 127) z_raw -= 256;

        auto msg = geometry_msgs::msg::Vector3();
        msg.x = x_raw / 64.0;
        msg.y = y_raw / 64.0;
        msg.z = z_raw / 64.0;

        //RCLCPP_INFO(this->get_logger(), "[g] X: %.2f | Y: %.2f | Z: %.2f", msg.x, msg.y, msg.z);
        pub_->publish(msg);
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VectorLogicClient>());
    rclcpp::shutdown();
    return 0;
}
