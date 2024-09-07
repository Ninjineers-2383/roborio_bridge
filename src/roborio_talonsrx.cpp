#include "roborio_bridge/roborio_talonsrx.hpp"
#include <iostream>

#include "fms_status.pb.h"
#include "motor_init.pb.h"
#include "motor_control.pb.h"

using namespace roborio_bridge;

RoboRIOTalonSRX::RoboRIOTalonSRX()
    : logger_{rclcpp::get_logger("RoboRIOTalonSRX")}
{
}

void RoboRIOTalonSRX::process_status_messages(std::shared_ptr<zmq::context_t> ctx)
{
    zmq::socket_t motorSub(*ctx, ZMQ_SUB);
    zmq::socket_t motorReq(*ctx, ZMQ_REQ);
    zmq::socket_t controlRep(*ctx, ZMQ_REP);

    controlRep.connect("inproc://control");

    zmq::pollitem_t items[] = {
        {controlRep, 0, ZMQ_POLLIN, 0},
        {motorSub, 0, ZMQ_POLLIN, 0}};

    rclcpp::Logger logger = rclcpp::get_logger("talonsrx_message_thread");

    while (true)
    {
        zmq::message_t message;
        zmq::poll(&items[0], 2, std::chrono::seconds(-1));

        if (items[0].revents & ZMQ_POLLIN)
        {
            RCLCPP_INFO(logger, "Got an inproc request");
            controlRep.recv(message);

            if (*((unsigned char *)message.data()) == 1)
            {
                RCLCPP_INFO(logger, "Got an inproc configure request");

                controlRep.recv(message);
                auto connectStr = message.to_string();
                controlRep.recv(message);
                auto connectStr2 = message.to_string();
                controlRep.recv(message);

                RCLCPP_INFO_STREAM(logger, "Connecting to " << connectStr << " and " << connectStr2);

                motorSub.connect(connectStr.c_str());
                motorReq.connect(connectStr2.c_str());

                RCLCPP_INFO_STREAM(logger, "Connected");

                zmq::message_t header(std::string("INIT_MOTOR"));
                motorReq.send(header, ZMQ_SNDMORE);
                motorReq.send(message);

                RCLCPP_INFO_STREAM(logger, "Sent waiting for response");

                motorReq.recv(message);

                messages::motor::Motor_Init_Rep motor_rep;
                motor_rep.ParseFromArray(message.data(), message.size());

                motorSub.set(zmq::sockopt::subscribe, motor_rep.motor_topic());
                motorSub.set(zmq::sockopt::subscribe, "FMS");

                RCLCPP_INFO_STREAM(logger, "Subscribing to" << motor_rep.motor_topic());

                zmq::message_t response(1);
                *(bool *)response.data() = motor_rep.success();

                controlRep.send(response);
            }
            else if (*((unsigned char *)message.data()) == 2)
            {
                controlRep.recv(message);
                auto connectStr = message.to_string();

                motorSub.disconnect(connectStr);
            }
            else if (*(unsigned char *)message.data() == 3)
            {
                controlRep.recv(message);

                zmq::message_t header(std::string("MOTOR_CONTROL"));
                motorReq.send(header, ZMQ_SNDMORE);
                motorReq.send(message);

                motorReq.recv(message);

                messages::motor::Motor_Control_Rep motor_rep;
                motor_rep.ParseFromArray(message.data(), message.size());

                zmq::message_t response(1);
                *(bool *)response.data() = motor_rep.success();

                controlRep.send(response);
            }
            else
            {
                std::cerr << "Control request unknown" << std::endl;
            }
        }

        if (items[1].revents & ZMQ_POLLIN)
        {
            zmq::message_t topic;
            motorSub.recv(topic);
            motorSub.recv(message);

            std::string str_topic = topic.to_string();

            if (str_topic == "MOTOR1")
            {

                motorDataBuffer.writeFromNonRT(*(double *)message.data());
                // RCLCPP_INFO(logger, "motor data");
            }
            else if (str_topic == "FMS")
            {
                messages::FMS_Status fms_status;
                fms_status.ParseFromArray(message.data(), message.size());
                // RCLCPP_INFO_STREAM(logger, "is_auton: " << fms_status.is_auto() << " is_teleop: " << fms_status.is_teleop());
            }
            else
                RCLCPP_ERROR(logger, "unknown data");
        }
    }
}

hardware_interface::CallbackReturn RoboRIOTalonSRX::on_init(
    const hardware_interface::HardwareInfo &info)
{
    if (hardware_interface::ActuatorInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    context = std::make_shared<zmq::context_t>();
    controlReq = std::make_unique<zmq::socket_t>(*context.get(), ZMQ_REQ);

    controlReq->bind("inproc://control");

    message_thread = std::thread{&RoboRIOTalonSRX::process_status_messages, this, context};

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RoboRIOTalonSRX::on_configure(
    const rclcpp_lifecycle::State &previous_state)
{

    messages::motor::Motor_Init_Req motor_init_rq;
    motor_init_rq.set_motor_type(messages::motor::MotorType::Motor_TalonSRX);
    motor_init_rq.set_can_bus("canivore");
    motor_init_rq.set_can_id(1);

    std::string mess;
    motor_init_rq.SerializeToString(&mess);

    zmq::message_t option(1);
    *(unsigned char *)option.data() = 1;
    controlReq->send(option, ZMQ_SNDMORE);
    zmq::message_t connection(std::string("tcp://127.0.0.1:8090"));
    controlReq->send(connection, ZMQ_SNDMORE);
    zmq::message_t connection2(std::string("tcp://127.0.0.1:8091"));
    controlReq->send(connection2, ZMQ_SNDMORE);
    zmq::message_t motor_init(mess);
    controlReq->send(motor_init);

    RCLCPP_INFO(this->get_logger(), "Sent connection info");

    zmq::message_t res;

    controlReq->recv(res);

    RCLCPP_INFO(this->get_logger(), "Received response");

    if (!*(bool *)res.data())
    {
        RCLCPP_ERROR(this->get_logger(), "Control response not success");
        return hardware_interface::CallbackReturn::ERROR;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RoboRIOTalonSRX::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> stateInterfaces;

    stateInterfaces.emplace_back(hardware_interface::StateInterface(info_.joints[0].name, "position", &position));
    stateInterfaces.emplace_back(hardware_interface::StateInterface(info_.joints[0].name, "velocity", &velocity));
    stateInterfaces.emplace_back(hardware_interface::StateInterface(info_.joints[0].name, "acceleration", &acceleration));

    return stateInterfaces;
}

std::vector<hardware_interface::CommandInterface> RoboRIOTalonSRX::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> commandInterfaces;

    commandInterfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[0].name, "effort", &voltage_command));

    return commandInterfaces;
}

hardware_interface::return_type RoboRIOTalonSRX::read(
    const rclcpp::Time &, const rclcpp::Duration &)
{
    position = *motorDataBuffer.readFromRT();

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type RoboRIOTalonSRX::write(
    const rclcpp::Time &, const rclcpp::Duration &)
{
    messages::motor::Motor_Control_Req motor_control_req;
    motor_control_req.set_motor_id(0);
    motor_control_req.set_control(messages::motor::CONTROL_VOLTAGE);
    motor_control_req.set_voltage(voltage_command);

    std::string mess;
    motor_control_req.SerializeToString(&mess);

    zmq::message_t option(1);
    *(unsigned char *)option.data() = 3;
    controlReq->send(option, ZMQ_SNDMORE);
    zmq::message_t motor_init(mess);
    controlReq->send(motor_init);

    zmq::message_t res;

    controlReq->recv(res);

    if (!*(bool *)res.data())
    {
        RCLCPP_ERROR(this->get_logger(), "Control response not success");
        return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(RoboRIOTalonSRX, hardware_interface::ActuatorInterface)