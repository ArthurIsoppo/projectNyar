#include "rclcpp/rclcpp.hpp"
#include <sensor_interfaces/srv/i2c_command.hpp>

// Headers do Linux I2C
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
        // 1. Abrir o barramento I2C físico
        i2c_fd_ = open("/dev/i2c-1", O_RDWR);
        if (i2c_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Falha ao abrir o barramento I2C");
            return;
        } 
        RCLCPP_INFO(this->get_logger(), "Shoggoth awake: Barramento I2C Aberto no Modo Passivo.");

        // 2. Iniciar o ÚNICO Serviço de comunicação genérica
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

    // --- A TRANSAÇÃO ATÔMICA (Acionada apenas pela rede) ---
    void handle_service_command(
        const std::shared_ptr<sensor_interfaces::srv::I2cCommand::Request> request,
        std::shared_ptr<sensor_interfaces::srv::I2cCommand::Response> response) {
        
        // Focar no dispositivo específico pedido pela Workstation
        if (ioctl(i2c_fd_, I2C_SLAVE, request->device_addr) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Falha ao contactar o endereço 0x%X", request->device_addr);
            response->success = false;
            return;
        }

        response->success = true; // Assume sucesso até prova em contrário

        // PASSO 1: ESCRITA (Injeção de Registradores ou Dados Puros)
        if (request->write_data.size() > 0) {
            if (write(i2c_fd_, request->write_data.data(), request->write_data.size()) != (ssize_t)request->write_data.size()) {
                RCLCPP_ERROR(this->get_logger(), "Erro elétrico na Escrita I2C.");
                response->success = false;
                return; // Se a escrita falhar, abortamos antes de ler
            }
        }

        // PASSO 2: LEITURA (Extração de Dados)
// PASSO 2: LEITURA (Extração de Dados)
        if (request->length > 0) {
            response->read_data.resize(request->length);
            if (read(i2c_fd_, response->read_data.data(), request->length) != request->length) {
                RCLCPP_ERROR(this->get_logger(), "Erro elétrico na Leitura I2C.");
                response->success = false;
            }
        } else {
            // FIX DO CYCLONEDDS: Nunca devolver um array de tamanho ZERO na rede
            response->read_data = {0x00}; 
        }
        
        // O ROS 2 bloqueia outros nós enquanto isso roda, garantindo que ninguém 
        // intercepte o barramento entre a Fase 1 e a Fase 2!
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<I2cEdgeServer>());
    rclcpp::shutdown();
    return 0;
}