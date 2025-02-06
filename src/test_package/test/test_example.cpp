#include <gtest/gtest.h>
#include "../src/example_node.cpp"

class ExampleNodeTest : public ::testing::Test {
protected:
    ExampleNode node;
};

TEST_F(ExampleNodeTest, TestInitialCount) {
    EXPECT_EQ(node.getCount(), 0);
}

TEST_F(ExampleNodeTest, TestIncrement) {
    node.increment();
    EXPECT_EQ(node.getCount(), 1);
    node.increment();
    EXPECT_EQ(node.getCount(), 2);
}

TEST_F(ExampleNodeTest, TestMessageProcessing) {
    std::string input = "test message";
    std::string expected = "Processed: test message";
    EXPECT_EQ(node.processMessage(input), expected);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
