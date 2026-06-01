#include "rclcpp/rclcpp.hpp"
#include <sensor_interfaces/srv/i2c_command.hpp>

// Linux I2C headers (Outside of source material)
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <unistd.h>

using std::placeholders::_1;
using std::placeholders::_2;

class I2cServer : public rclcpp::Node {
public:
    I2cServer() : Node("i2c_server") {
        // Create the service server
        srv_ = this->create_service<sensor_interfaces::srv::I2cCommand>(
            "/sensor/i2c_command", 
            std::bind(&GenericI2cServer::i2c_callback, this, _1, _2)
        );

        // Open the I2C bus 1
        i2c_fd_ = open("/dev/i2c-1", O_RDWR);
        if (i2c_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "failed to open I2C bus");
        } else {
            RCLCPP_INFO(this->get_logger(), "Shoggoth awake: I2C Server Ready."); //maybe tirar isso
        }
    }

    ~I2cServer() {
        if (i2c_fd_ >= 0) close(i2c_fd_);
    }

private:
    int i2c_fd_;
    rclcpp::Service<sensor_interfaces::srv::I2cCommand>::SharedPtr srv_;

    void i2c_callback(const std::shared_ptr<sensor_interfaces::srv::I2cCommand::Request> request, std::shared_ptr<sensor_interfaces::srv::I2cCommand::Response> response) {
        
        // Target the specific I2C device address requested
        if (ioctl(i2c_fd_, I2C_SLAVE, request->device_addr) < 0) {
            response->success = false;
            return;
        }

        // Handle the Read/Write opcode
        if (request->is_read) {
            // Write the register address we want to read from
            write(i2c_fd_, &request->reg_addr, 1);
            
            // Read the specified length into our response buffer
            response->read_data.resize(request->length);
            if (read(i2c_fd_, response->read_data.data(), request->length) == request->length) {
                response->success = true;
            } else {
                response->success = false;
            }
        } else {
            // Write Mode: Construct buffer [register_address, data...]
            std::vector<uint8_t> buffer;
            buffer.push_back(request->reg_addr);
            buffer.insert(buffer.end(), request->write_data.begin(), request->write_data.end());
            
            if (write(i2c_fd_, buffer.data(), buffer.size()) == (ssize_t)buffer.size()) {
                response->success = true;
            } else {
                response->success = false;
            }
        }
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<I2cServer>());
    rclcpp::shutdown();
    return 0;
}