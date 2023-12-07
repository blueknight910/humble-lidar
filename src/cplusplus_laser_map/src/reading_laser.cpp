#include <memory>
#include <vector>
#include <cmath>

#include "matplotlibcpp.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace plt = matplotlibcpp; // Alias namespace for matplotlibcpp for easier usage
using std::placeholders::_1;   // For std::bind - allows placeholders in callback function

// Class definition for ReadingLaser
class ReadingLaser : public rclcpp::Node
{
public:
    // Constructor for the ReadingLaser class
    ReadingLaser()
    : Node("reading_laser"), range_index_(0)
    {
        // Subscribing to the LaserScan topic. 
        // "scan" is the topic name, 10 is the queue size, 
        // and the callback function is bound to this class instance
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&ReadingLaser::topic_callback, this, _1));
    }

    // Destructor for the ReadingLaser class
    ~ReadingLaser() {
        // Call plotData function to plot the data when the object is destroyed
        plotData();
    }

private:
    // Callback function for LaserScan messages
    void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr _msg) {
        // Check if range_index_ exceeds the size of the ranges array
        if (range_index_ >= _msg->ranges.size()) {
            range_index_ = 0; // Reset to 0 if it does
        }

        // Log the live data being collected to the console
        RCLCPP_INFO(this->get_logger(), "Received scan data: Angle %d°, Distance %f meters", range_index_, _msg->ranges[range_index_]);

        // Store the angle and distance data in collected_data_
        collected_data_.push_back(std::make_pair(range_index_, _msg->ranges[range_index_]));

        // Increment the range index, reset to 0 after completing a full circle (360 degrees)
        range_index_ = (range_index_ + 1) % 360;

        // Shutdown the node after collecting data for 3 full rotations
        if (collected_data_.size() >= 360 * 3) {
            rclcpp::shutdown();
        }
    }

    // Function to plot the collected data
    void plotData() {
        std::vector<double> x, y; // Vectors to store Cartesian coordinates

        // Convert polar coordinates to Cartesian coordinates
        for (const auto& data : collected_data_) {
            double angle = data.first * M_PI / 180.0; // Convert angle to radians
            double radius = data.second; // Radius (distance)
            x.push_back(radius * cos(angle)); // Convert to Cartesian x-coordinate
            y.push_back(radius * sin(angle)); // Convert to Cartesian y-coordinate
        }

        // Plotting the data using matplotlibcpp
        plt::figure(); // Create a new figure
        plt::plot(x, y, "ro"); // Plot x and y as red dots
        plt::title("Laser Scan Data in Cartesian Coordinates"); // Set the title of the plot
        plt::xlabel("X axis"); // Label for x-axis
        plt::ylabel("Y axis"); // Label for y-axis
        plt::save("/home/prez/git/humble-lidar/output/cartesian_plot.png"); // Save the plot as an image
    }

    // Member variables
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_; // Subscription object
    size_t range_index_; // Current index in the laser scan array
    std::vector<std::pair<int, float>> collected_data_; // Vector to store collected angle and distance data
};

// Main function
int main(int argc, char * argv[])
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Spin the ReadingLaser node
    rclcpp::spin(std::make_shared<ReadingLaser>());

    // Shutdown ROS 2
    rclcpp::shutdown();

    return 0;
}
