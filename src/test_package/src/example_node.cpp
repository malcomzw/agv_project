#include <ros/ros.h>
#include <std_msgs/String.h>

class ExampleNode {
public:
    ExampleNode() : count_(0) {}

    int getCount() const { return count_; }
    void increment() { count_++; }
    
    std::string processMessage(const std::string& input) {
        return "Processed: " + input;
    }

private:
    int count_;
};
