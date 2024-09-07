#ifndef ROBORIO_TALONSRX_HPP
#define ROBORIO_TALONSRX_HPP

#include "hardware_interface/actuator_interface.hpp"
#include "realtime_tools/realtime_buffer.h"
#include <zmq.hpp>

#include <thread>

#include <memory>

namespace roborio_bridge
{
    class RoboRIOTalonSRX : public hardware_interface::ActuatorInterface
    {
    public:
        RoboRIOTalonSRX();

        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareInfo &info) override;

        hardware_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &previous_state) override;

        // hardware_interface::CallbackReturn on_cleanup(
        //     const rclcpp_lifecycle::State &previous_state) override;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        // hardware_interface::CallbackReturn on_activate(
        //     const rclcpp_lifecycle::State &previous_state) override;

        // hardware_interface::CallbackReturn on_deactivate(
        //     const rclcpp_lifecycle::State &previous_state) override;

        // hardware_interface::CallbackReturn on_shutdown(
        //     const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::return_type read(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        hardware_interface::return_type write(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

    protected:
        void process_status_messages(std::shared_ptr<zmq::context_t> ctx);

        std::thread message_thread;

        std::shared_ptr<zmq::context_t>
            context{nullptr};
        std::unique_ptr<zmq::socket_t> controlReq{nullptr};
        std::unique_ptr<zmq::socket_t> motorPub{nullptr};

        realtime_tools::RealtimeBuffer<double> motorDataBuffer;

        rclcpp::Logger logger_;

        double position;
        double velocity;
        double acceleration;
        double voltage_command;
    };
}
#endif /* ROBORIO_TALONSRX_HPP */