#include "roborio_bridge/roborio_talonsrx.hpp"
#include <iostream>

using namespace roborio_bridge;

RoboRIOTalonSRX::RoboRIOTalonSRX()
    : logger_{rclcpp::get_logger("RoboRIOTalonSRX")}
{
}

void RoboRIOTalonSRX::process_status_messages(std::shared_ptr<zmq::context_t> ctx)
{
    zmq::socket_t motorSub(*ctx, ZMQ_SUB);
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
                auto motorTopic = message.to_string();

                RCLCPP_INFO_STREAM(logger, "Connecting to " << connectStr.c_str() << " on topic " << motorTopic.c_str());

                motorSub.connect(connectStr.c_str());

                motorSub.setsockopt(ZMQ_SUBSCRIBE, motorTopic.c_str(), motorTopic.length());

                zmq::message_t response(1);
                *(unsigned char *)response.data() = 0;

                controlRep.send(response);
            }

            else if (*((unsigned char *)message.data()) == 2)
            {
                controlRep.recv(message);
                auto connectStr = message.to_string();

                motorSub.disconnect(connectStr);
            }
            else
            {
                std::cerr << "Control request unknown" << std::endl;
            }
        }

        if (items[1].revents & ZMQ_POLLIN)
        {
            motorSub.recv(message);
            motorSub.recv(message);

            motorDataBuffer.writeFromNonRT(*(double *)message.data());
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
    zmq::message_t option(1);
    *(unsigned char *)option.data() = 1;
    controlReq->send(option, ZMQ_SNDMORE);
    zmq::message_t connection("tcp://127.0.0.1:8090");
    controlReq->send(connection, ZMQ_SNDMORE);
    zmq::message_t topic(6);
    memcpy(topic.data(), "MOTOR1", 6);
    controlReq->send(topic);

    RCLCPP_INFO(this->get_logger(), "Sent connection info");

    zmq::message_t res;

    controlReq->recv(res);

    RCLCPP_INFO(this->get_logger(), "Received response");

    if (*(unsigned char *)res.data())
    {
        RCLCPP_ERROR(this->get_logger(), "Control response non 0");
        return hardware_interface::CallbackReturn::ERROR;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RoboRIOTalonSRX::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> stateInterfaces;

    stateInterfaces.emplace_back(hardware_interface::StateInterface("joint1", "position", &position));
    stateInterfaces.emplace_back(hardware_interface::StateInterface("joint1", "command", &command));

    return stateInterfaces;
}

std::vector<hardware_interface::CommandInterface> RoboRIOTalonSRX::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> commandInterfaces;

    commandInterfaces.emplace_back(hardware_interface::CommandInterface("joint1", "position", &command));

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
    return hardware_interface::return_type::OK;
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(RoboRIOTalonSRX, hardware_interface::ActuatorInterface)