#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

class DemoNode {
private:
    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub_;
    ros::Timer timer_;
    
public:
    DemoNode() : nh_("~") {
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        timer_ = nh_.createTimer(ros::Duration(1.0), &DemoNode::timerCallback, this);
    }
    
    void timerCallback(const ros::TimerEvent&) {
        geometry_msgs::Twist msg;
        msg.linear.x = 0.5;
        msg.angular.z = 0.2;
        cmd_vel_pub_.publish(msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "demo_node");
    DemoNode node;
    ros::spin();
    return 0;
}
