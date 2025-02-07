#include <gtest/gtest.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

class TestDemo : public ::testing::Test {
protected:
    ros::NodeHandle nh_;
    ros::Subscriber cmd_vel_sub_;
    geometry_msgs::Twist last_cmd_;
    bool received_msg_;
    
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
        last_cmd_ = *msg;
        received_msg_ = true;
    }
    
    void SetUp() override {
        received_msg_ = false;
        cmd_vel_sub_ = nh_.subscribe("/cmd_vel", 1, &TestDemo::cmdVelCallback, this);
    }
};

TEST_F(TestDemo, TestCmdVelPublishing) {
    ros::Rate rate(10);
    int count = 0;
    
    while (!received_msg_ && count < 50) {
        ros::spinOnce();
        rate.sleep();
        count++;
    }
    
    ASSERT_TRUE(received_msg_) << "No cmd_vel message received within timeout";
    EXPECT_NEAR(last_cmd_.linear.x, 0.5, 0.01) 
        << "Linear velocity does not match expected value";
    EXPECT_NEAR(last_cmd_.angular.z, 0.2, 0.01)
        << "Angular velocity does not match expected value";
}

TEST_F(TestDemo, TestCmdVelBounds) {
    ros::Rate rate(10);
    int count = 0;
    
    while (!received_msg_ && count < 50) {
        ros::spinOnce();
        rate.sleep();
        count++;
    }
    
    ASSERT_TRUE(received_msg_) << "No cmd_vel message received";
    
    // Check velocity bounds
    EXPECT_GE(last_cmd_.linear.x, 0.0) << "Linear velocity cannot be negative";
    EXPECT_LE(last_cmd_.linear.x, 1.0) << "Linear velocity exceeds maximum";
    
    EXPECT_GE(last_cmd_.angular.z, -1.0) << "Angular velocity below minimum";
    EXPECT_LE(last_cmd_.angular.z, 1.0) << "Angular velocity exceeds maximum";
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_demo");
    return RUN_ALL_TESTS();
}
