#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "sensor_interfaces/srv/i2c_command.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "geometry_msgs/msg/vector3.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

// Definição dos estados da FSM do Robô
enum class RobotState {
    INITIALIZING,
    RUNNING,
    ERROR
};

class VectorLogicClient : public rclcpp::Node {
public:
    VectorLogicClient() : Node("vector_logic_client"), current_state_(RobotState::INITIALIZING) {
        // 1. Publisher para a gravidade final calculada
        pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/sensor/accel/vector", 10);

        // 2. Cliente do único Serviço I2C existente na rede
        client_ = this->create_client<sensor_interfaces::srv::I2cCommand>("/sensor/i2c_command");

        // 3. O Mestre assume o ritmo! Timer de 20Hz (50ms) controlando toda a operação.
        fsm_timer_ = this->create_wall_timer(50ms, std::bind(&VectorLogicClient::fsm_cycle, this));
    }

private:
    RobotState current_state_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_;
    rclcpp::Client<sensor_interfaces::srv::I2cCommand>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr fsm_timer_;
    
    // Flag de segurança para não inundar a rede se a resposta atrasar
    bool waiting_for_response_ = false; 

    // --- SUPERVISOR DA FSM (Roda a 20Hz ditando o barramento) ---
    void fsm_cycle() {
        if (waiting_for_response_) return; // Aguarda a Rasp responder antes de pedir de novo

        if (!client_->wait_for_service(0s)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Procurando Raspberry Pi na rede...");
            return;
        }

        auto request = std::make_shared<sensor_interfaces::srv::I2cCommand::Request>();
        request->device_addr = 0x1D; // O Endereço do MMA7455

        switch (current_state_) {
            case RobotState::INITIALIZING:
                // PASSO 1: ACORDAR O SENSOR (Transação Atômica de Escrita)
                // Lembra da remoção do reg_addr? Ele entra como o 1º byte dos dados!
                // 0x16 (Reg Mode Control) + 0x01 (Acordar em 8-bit)
                request->write_data = {0x16, 0x01}; 
                request->length = 0; // Não pedimos dados de volta

                waiting_for_response_ = true;
                client_->async_send_request(request, [this](rclcpp::Client<sensor_interfaces::srv::I2cCommand>::SharedFuture future) {
                    waiting_for_response_ = false;
                    if (future.get()->success) {
                        RCLCPP_INFO(this->get_logger(), "Acelerômetro acordado! Transicionando para RUNNING.");
                        current_state_ = RobotState::RUNNING;
                    }
                });
                break;

            case RobotState::RUNNING:
                // PASSO 2: TELEMETRIA CONTÍNUA (Transação Atômica Escreve + Lê)
                // Aponta para o registrador 0x06 (Eixo X) e puxa 3 bytes sequenciais
                request->write_data = {0x06}; 
                request->length = 3;

                waiting_for_response_ = true;
                client_->async_send_request(request, [this](rclcpp::Client<sensor_interfaces::srv::I2cCommand>::SharedFuture future) {
                    waiting_for_response_ = false;
                    auto response = future.get();
                    if (response->success && response->read_data.size() == 3) {
                        process_and_publish(response->read_data);
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "Falha na leitura elétrica! Sensor desconectou?");
                        current_state_ = RobotState::ERROR;
                    }
                });
                break;

            case RobotState::ERROR:
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "FSM travada em estado de ERRO.");
                break;
        }
    }

    // --- CÉREBRO: Lógica Física e Conversões Isoladas ---
    void process_and_publish(const std::vector<uint8_t>& data) {
        int x_raw = data[0];
        int y_raw = data[1];
        int z_raw = data[2];

        // Complemento de 2 (Preservado do seu código original)
        if (x_raw > 127) x_raw -= 256;
        if (y_raw > 127) y_raw -= 256;
        if (z_raw > 127) z_raw -= 256;

        // Cálculo da física real (g)
        auto vector_msg = geometry_msgs::msg::Vector3();
        vector_msg.x = x_raw / 64.0;
        vector_msg.y = y_raw / 64.0;
        vector_msg.z = z_raw / 64.0;

        RCLCPP_INFO(this->get_logger(), "Física Real [g] -> X: %.2f | Y: %.2f | Z: %.2f", vector_msg.x, vector_msg.y, vector_msg.z);

        // Finalmente, publica no Tópico do sistema para a Navegação usar
        pub_->publish(vector_msg);
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VectorLogicClient>());
    rclcpp::shutdown();
    return 0;
}