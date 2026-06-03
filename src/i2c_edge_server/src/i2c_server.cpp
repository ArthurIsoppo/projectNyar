#include "rclcpp/rclcpp.hpp"
#include <sensor_interfaces/srv/i2c_command.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

// Headers do Linux I2C
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <unistd.h>
#include <vector>

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

class I2cEdgeServer : public rclcpp::Node {
public:
    I2cEdgeServer() : Node("i2c_edge_server") {
        // 1. Declaração de Parâmetros
        this->declare_parameter<int>("device_addr", 0x1D);     // Endereço do MMA7455
        this->declare_parameter<int>("reg_start_addr", 0x06);  // Registo do Eixo X (XOUT8)
        this->declare_parameter<int>("read_length", 3);        // Ler 3 bytes (X, Y, Z)
        this->declare_parameter<double>("pub_rate_hz", 20.0);  // Frequência de leitura

        // Obter valores iniciais dos parâmetros
        device_addr_ = this->get_parameter("device_addr").as_int();
        reg_start_addr_ = this->get_parameter("reg_start_addr").as_int();
        read_length_ = this->get_parameter("read_length").as_int();
        double rate = this->get_parameter("pub_rate_hz").as_double();

        // 2. Abrir o barramento I2C físico
        i2c_fd_ = open("/dev/i2c-1", O_RDWR);
        if (i2c_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Falha ao abrir o barramento I2C");
            return;
        } else {
            RCLCPP_INFO(this->get_logger(), "Shoggoth awake: Barramento I2C Aberto.");
        }

        // 3. Inicializar o Publisher (Telemetria Contínua)
        raw_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("/sensor/raw_telemetry", 10);

        // 4. Inicializar o Timer para leitura cíclica
        std::chrono::duration<double> period(1.0 / rate);
        timer_ = this->create_wall_timer(period, std::bind(&I2cEdgeServer::read_cycle, this));

        // 5. Manter o Serviço (Para comandos de ESCRITA pontuais)
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
    int device_addr_;
    int reg_start_addr_;
    int read_length_;

    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr raw_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<sensor_interfaces::srv::I2cCommand>::SharedPtr srv_;

    // --- O MÚSCULO DE LEITURA (Automático e Contínuo) ---
    void read_cycle() {
        if (i2c_fd_ < 0) return;

        // Focar no dispositivo I2C alvo
        if (ioctl(i2c_fd_, I2C_SLAVE, device_addr_) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Falha ao contactar o escravo I2C");
            return;
        }

        // Escrever o registo de onde queremos começar a ler (ex: 0x06)
        uint8_t reg = reg_start_addr_;
        if (write(i2c_fd_, &reg, 1) != 1) return;

        // Ler a quantidade de bytes definida nos parâmetros
        std::vector<uint8_t> buffer(read_length_);
        if (read(i2c_fd_, buffer.data(), read_length_) == read_length_) {
            
            // Publicar os dados lidos no Tópico
            auto msg = std_msgs::msg::UInt8MultiArray();
            msg.data = buffer;
            raw_pub_->publish(msg);
        }
    }

    // --- O MÚSCULO DE ESCRITA (Pontual, acionado pela rede) ---
    void handle_service_command(
        const std::shared_ptr<sensor_interfaces::srv::I2cCommand::Request> request,
        std::shared_ptr<sensor_interfaces::srv::I2cCommand::Response> response) {
        
        // Focar no dispositivo específico pedido pelo serviço
        if (ioctl(i2c_fd_, I2C_SLAVE, request->device_addr) < 0) {
            response->success = false;
            return;
        }

        // Lógica original de escrita do seu código
        if (!request->is_read) {
            std::vector<uint8_t> buffer;
            buffer.push_back(request->reg_addr);
            buffer.insert(buffer.end(), request->write_data.begin(), request->write_data.end());
            
            if (write(i2c_fd_, buffer.data(), buffer.size()) == (ssize_t)buffer.size()) {
                response->success = true;
                RCLCPP_INFO(this->get_logger(), "Comando de escrita I2C executado com sucesso.");
            } else {
                response->success = false;
            }
        } 
        // Mantive a opção de leitura no serviço caso queira ler apenas 1 byte esporádico (ex: ler um ID de registo)
        else {
            write(i2c_fd_, &request->reg_addr, 1);
            response->read_data.resize(request->length);
            if (read(i2c_fd_, response->read_data.data(), request->length) == request->length) {
                response->success = true;
            } else {
                response->success = false;
            }
        }
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<I2cEdgeServer>());
    rclcpp::shutdown();
    return 0;
}