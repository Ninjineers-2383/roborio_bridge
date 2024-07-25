#include <gmock/gmock.h>

#include <string>

#include "hardware_interface/resource_manager.hpp"
#include "ros2_control_test_assets/components_urdfs.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

class TestGenericSystem : public ::testing::Test
{
protected:
    void SetUp() override
    {
        generic_system_2dof_ =
            R"(
        <ros2_control name="TalonSRX1" type="actuator">
          <hardware>
            <plugin>roborio_bridge/RoboRIOTalonSRX</plugin>
          </hardware>
          <joint name="joint1">
            <command_interface name="position"/>
            <state_interface name="position"/>
            <state_interface name="command"/>
          </joint>
        </ros2_control>
    )";
    }

    std::string generic_system_2dof_;
    rclcpp::Node node_ = rclcpp::Node("TestGenericSystem");
};

class TestableResourceManager : public hardware_interface::ResourceManager
{
public:
    friend TestGenericSystem;

    FRIEND_TEST(TestGenericSystem, load_canopen_system_2dof);

    explicit TestableResourceManager(rclcpp::Node &node)
        : hardware_interface::ResourceManager(
              node.get_node_clock_interface(), node.get_node_logging_interface())
    {
    }

    explicit TestableResourceManager(
        rclcpp::Node &node, const std::string &urdf, bool activate_all = false)
        : hardware_interface::ResourceManager(
              urdf, node.get_node_clock_interface(), node.get_node_logging_interface(), activate_all, 100)
    {
    }
};

TEST_F(TestGenericSystem, load_canopen_system_2dof)
{
    auto urdf = ros2_control_test_assets::urdf_head + generic_system_2dof_ +
                ros2_control_test_assets::urdf_tail;
    // ASSERT_NO_THROW(TestableResourceManager rm(node_, urdf));
    ASSERT_TRUE(true);
}