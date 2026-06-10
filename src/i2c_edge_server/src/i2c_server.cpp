#include "rclcpp/rclcpp.hpp"
#include <sensor_interfaces/srv/i2c_command.hpp>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <unistd.h>
#include <vector>

using std::placeholders::_1;
using std::placeholders::_2;

class I2cEdgeServer : public rclcpp::Node {
public:
    I2cEdgeServer() : Node("i2c_edge_server") {
        i2c_fd_ = open("/dev/i2c-1", O_RDWR);
        if (i2c_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "failed to open");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "I2C open");

        srv_ = this->create_service<sensor_interfaces::srv::I2cCommand>(
            "/sensor/i2c_command",
            std::bind(&I2cEdgeServer::handle_service_command, this, _1, _2)
        );
    }

    ~I2cEdgeServer() {
        if (i2c_fd_ >= 0) close(i2c_fd_);
    }

private:
    int i2c_fd_;
    rclcpp::Service<sensor_interfaces::srv::I2cCommand>::SharedPtr srv_;

    void handle_service_command(
        const std::shared_ptr<sensor_interfaces::srv::I2cCommand::Request> request,
        std::shared_ptr<sensor_interfaces::srv::I2cCommand::Response> response)
    {
        if (ioctl(i2c_fd_, I2C_SLAVE, request->device_addr) < 0) {
            RCLCPP_ERROR(this->get_logger(), "failed to access addr 0x%X", request->device_addr);
            response->success = false;
            return;
        }

        response->success = true;

        if (!request->write_data.empty()) {
            if (write(i2c_fd_, request->write_data.data(), request->write_data.size())
                    != (ssize_t)request->write_data.size()) {
                RCLCPP_ERROR(this->get_logger(), "error on I2C write");
                response->success = false;
                return;
            }
        }

        if (request->length > 0) {
            response->read_data.resize(request->length);
            if (read(i2c_fd_, response->read_data.data(), request->length) != request->length) {
                RCLCPP_ERROR(this->get_logger(), "error on I2C read");
                response->success = false;
            } else {
                RCLCPP_INFO(this->get_logger(), "[MARK] Timestamp - Data going to the workstation");
            }
        } else {
            // to stop buffer spam
            response->read_data = {0x00};
        }
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<I2cEdgeServer>());
    rclcpp::shutdown();
    return 0;
}
