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
    
    TestDemo() : received_msg_(false) {
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
    
    EXPECT_TRUE(received_msg_);
    EXPECT_NEAR(last_cmd_.linear.x, 0.5, 0.01);
    EXPECT_NEAR(last_cmd_.angular.z, 0.2, 0.01);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_demo");
    return RUN_ALL_TESTS();
}
