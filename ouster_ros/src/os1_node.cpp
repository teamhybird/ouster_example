// -----------------------------------------------------------------------------
/**
 * Example node to publish OS1 output on ROS topics
 *
 * Additionally, this node can be used to record and replay raw sesnor output
 * by publishing and listening to PacketMsg topics
 *
 * ROS Parameters
 * decimate_mask: 0 means send all points. 1 means send every second point.
 * rotate_using_imu: 1 means rotate the points according to the IMU data
 * scan_dur_ns: nanoseconds to batch lidar packets before publishing a cloud
 * os1_hostname: hostname or IP in dotted decimal form of the sensor
 * os1_udp_dest: hostname or IP where the sensor will send data packets
 * os1_lidar_port: port to which the sensor should send lidar data
 * os1_imu_port: port to which the sensor should send imu data
 * replay_mode: when true, the node will listen on ~/lidar_packets and
 *   ~/imu_packets for data instead of attempting to connect to a sensor
 */
// -----------------------------------------------------------------------------

#include <chrono>
#include <functional>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include <pcl_conversions/pcl_conversions.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include "ouster_ros/PacketMsg.h"
#include "ouster_ros/os1_ros.h"
#include <std_msgs/Float64.h>

using ns = std::chrono::nanoseconds;
using PacketMsg = ouster_ros::PacketMsg;

// -----------------------------------------------------------------------------
// Main entry point for this ROS node
int main(int argc, char** argv) {
    // Initialise this ROS node
    ros::init(argc, argv, "liveouster_node");
    ros::NodeHandle nh("~");

    // Parse the input ROS parameters
    ouster_ros::OS1::rotate_using_imu = nh.param("rotate_using_imu", 1);
    ouster_ros::OS1::decimate_mask = nh.param("decimate_mask", 64);
    auto scan_dur = ns(nh.param("scan_dur_ns", 100000000)); // Process 100ms of data at a time
    auto os1_hostname = nh.param("os1_hostname", std::string("localhost"));
    auto os1_udp_dest = nh.param("os1_udp_dest", std::string("192.168.1.1"));
    auto os1_lidar_port = nh.param("os1_lidar_port", -1);
    auto os1_imu_port = nh.param("os1_imu_port", -1);
    auto replay_mode = nh.param("replay", true);

    // We publish processed data
    auto lidar_pub = nh.advertise<sensor_msgs::PointCloud2>("points", 10);
    auto imu_pub = nh.advertise<sensor_msgs::Imu>("imu", 10);

    // Handler for point cloud messages from the LIDAR via UDP packets
    auto lidar_handler = ouster_ros::OS1::batch_packets(
        scan_dur, [&](ns scan_ts, const ouster_ros::OS1::CloudOS1& cloud) {
            lidar_pub.publish(
                ouster_ros::OS1::cloud_to_cloud_msg(cloud, scan_ts));
        });

    // Handler for IMU messages from the LIDAR via UDP packets
    // This callback handler is done as a lambda rather than a standalone function, meaning imu_pub is in scope
    auto imu_handler = [&](const PacketMsg& p) {
        imu_pub.publish(ouster_ros::OS1::packet_to_imu_msg(p));
    };

    // Handler for IMU messages from the PX4 via ROS
    auto px4_handler = [&](const std_msgs::Float64& p) {
        float h = p.data;
        ouster_ros::OS1::set_yaw(h);
    };

    // Subscribe to the PX4 IMU data
    auto px4_packet_sub = nh.subscribe<std_msgs::Float64, const std_msgs::Float64& >(
            "/mavros/global_position/compass_hdg", 10, px4_handler);
    
    if (replay_mode) { // Replay mode - subscribe to raw packets from elsewhere
        auto lidar_packet_sub = nh.subscribe<PacketMsg, const PacketMsg&>(
            "lidar_packets", 500, lidar_handler);
        auto imu_packet_sub = nh.subscribe<PacketMsg, const PacketMsg&>(
            "imu_packets", 500, imu_handler);
        ros::spin();
    } else { // Non-replay mode - send raw and processed packets
        // Send raw packets
        auto lidar_packet_pub = nh.advertise<PacketMsg>("lidar_packets", 500);
        auto imu_packet_pub = nh.advertise<PacketMsg>("imu_packets", 500);

        // Connect to the ouster hardware
        auto cli = ouster::OS1::init_client(os1_hostname, os1_udp_dest,
            os1_lidar_port, os1_imu_port);
        if (!cli) {
            ROS_ERROR("Failed to initialize sensor at: %s", os1_hostname.c_str());
            return 1;
        }

        // Main spin code
        ouster_ros::OS1::spin(*cli,
                              [&](const PacketMsg& pm) {
                                  lidar_packet_pub.publish(pm);
                                  lidar_handler(pm);
                              },
                              [&](const PacketMsg& pm) {
                                  imu_packet_pub.publish(pm);
                                  imu_handler(pm);
                              });
    }
    return 0;
}
