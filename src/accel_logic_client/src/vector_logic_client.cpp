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
        // 1. Publisher para os dados processados (física real)
        pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/sensor/accel/vector", 10);
        
        // 2. Cliente de Serviço para enviar comandos pontuais (ex: acordar o sensor)
        client_ = this->create_client<sensor_interfaces::srv::I2cCommand>("/sensor/i2c_command");
        
        // 3. Subscriber para escutar a telemetria bruta vinda da Raspberry Pi
        sub_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
            "/sensor/raw_telemetry", 
            10, 
            std::bind(&VectorLogicClient::telemetry_callback, this, _1)
        );
        
        // Timer lento (ex: 1Hz) apenas para gerenciar a inicialização e supervisão da FSM
        fsm_timer_ = this->create_wall_timer(1s, std::bind(&VectorLogicClient::fsm_cycle, this));
    }

private:
    RobotState current_state_;
    
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_;
    rclcpp::Client<sensor_interfaces::srv::I2cCommand>::SharedPtr client_;
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr fsm_timer_;

    // --- SUPERVISOR DA FSM (Roda em background de forma lenta) ---
    void fsm_cycle() {
        switch (current_state_) {
            case RobotState::INITIALIZING:
                RCLCPP_INFO(this->get_logger(), "Estado FSM: INICIALIZANDO. Tentando acordar o acelerômetro...");
                send_wakeup_command();
                break;
                
            case RobotState::RUNNING:
                // O fluxo principal agora é guiado pela callback do Subscriber.
                // O timer aqui pode ser usado para checar timeouts de segurança se necessário.
                break;
                
            case RobotState::ERROR:
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Estado FSM: ERRO CRÍTICO!");
                break;
        }
    }

    // --- ESCRITA PONTUAL: Acorda o sensor através do serviço ---
    void send_wakeup_command() {
        if (!client_->wait_for_service(1s)) {
            RCLCPP_WARN(this->get_logger(), "Aguardando o servidor I2C na Raspberry Pi ficar online...");
            return;
        }

        auto request = std::make_shared<sensor_interfaces::srv::I2cCommand::Request>();
        request->device_addr = 0x1D;       // Endereço do MMA7455
        request->reg_addr = 0x16;          // Registrador Mode Control
        request->is_read = false;          // Operação de ESCRITA
        request->write_data = {0x01};      // Valor 0x01: Ativa o "Measurement Mode" (8-bit)

        client_->async_send_request(request, [this](rclcpp::Client<sensor_interfaces::srv::I2cCommand>::SharedFuture future) {
            auto response = future.get();
            if (response->success) {
                RCLCPP_INFO(this->get_logger(), "Acelerômetro acordado com sucesso! Transicionando para RUNNING.");
                this->current_state_ = RobotState::RUNNING;
            } else {
                RCLCPP_ERROR(this->get_logger(), "Falha ao enviar comando de inicialização. Retentando...");
            }
        });
    }

    // --- PROCESSAMENTO DA TELEMETRIA (O Cérebro Lógico) ---
    void telemetry_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
        // Segurança: só processa dados se o sensor já foi devidamente inicializado
        if (current_state_ != RobotState::RUNNING) {
            return;
        }

        if (msg->data.size() < 3) {
            RCLCPP_WARN(this->get_logger(), "Dados incompletos recebidos do barramento.");
            return;
        }

        // Extração dos bytes brutos recebidos da rede
        int x_raw = msg->data[0];
        int y_raw = msg->data[1];
        int z_raw = msg->data[2];

        // Conversão de Complemento de 2 (Lógica original preservada)
        if (x_raw > 127) x_raw -= 256;
        if (y_raw > 127) y_raw -= 256;
        if (z_raw > 127) z_raw -= 256;

        // Cálculo da física real (g)
        auto vector_msg = geometry_msgs::msg::Vector3();
        vector_msg.x = x_raw / 64.0;
        vector_msg.y = y_raw / 64.0;
        vector_msg.z = z_raw / 64.0;

        // Publica o vetor final calculado para o restante do ecossistema do robô
        pub_->publish(vector_msg);

        // Exemplo de como a FSM usaria esses dados para tomar decisões robóticas:
        // Se a inclinação em X for absurda (ex: robô capotando), muda o estado da FSM
        if (std::abs(vector_msg.x) > 1.5) { 
            RCLCPP_WARN(this->get_logger(), "Anomalia detectada no eixo X! Mudando para estado de ERRO.");
            current_state_ = RobotState::ERROR;
        }
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VectorLogicClient>());
    rclcpp::shutdown();
    return 0;
}