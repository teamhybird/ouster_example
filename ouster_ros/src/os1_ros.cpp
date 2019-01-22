// -----------------------------------------------------------------------------
// Ouster OS1 ROS input node.
// -----------------------------------------------------------------------------

#include <pcl_conversions/pcl_conversions.h>

#include "ouster/os1.h"
#include "ouster/os1_packet.h"
#include "ouster_ros/os1_ros.h"

// Maths libraries
#include <Eigen/Eigen>
#include <Eigen/Geometry>

namespace ouster_ros {
namespace OS1 {

using namespace ouster::OS1;

Eigen::Matrix3f imuRotate;
int rotate_using_imu = 0;
int decimate_mask = 0;
ros::Time last_log_time(0.0);

// -----------------------------------------------------------------------------
ns timestamp_of_imu_packet(const PacketMsg& pm) {
    return ns(imu_gyro_ts(pm.buf.data()));
}

// -----------------------------------------------------------------------------
ns timestamp_of_lidar_packet(const PacketMsg& pm) {
    return ns(col_timestamp(nth_col(0, pm.buf.data())));
}

// -----------------------------------------------------------------------------
bool read_imu_packet(const client& cli, PacketMsg& m) {
    m.buf.resize(imu_packet_bytes + 1);
    return read_imu_packet(cli, m.buf.data());
}

// -----------------------------------------------------------------------------
bool read_lidar_packet(const client& cli, PacketMsg& m) {
    m.buf.resize(lidar_packet_bytes + 1);
    return read_lidar_packet(cli, m.buf.data());
}

// -----------------------------------------------------------------------------
// Convert a packet from the IMU in OS1 format into a packet in ROS format
sensor_msgs::Imu packet_to_imu_msg(const PacketMsg& p) {
    const double standard_g = 9.80665;
    sensor_msgs::Imu m;
    const uint8_t* buf = p.buf.data();

    m.header.stamp.fromNSec(imu_gyro_ts(buf));
    m.header.frame_id = "os1_imu";

    m.orientation.x = 0;
    m.orientation.y = 0;
    m.orientation.z = 0;
    m.orientation.w = 0;

    m.linear_acceleration.x = imu_la_x(buf) * standard_g;
    m.linear_acceleration.y = imu_la_y(buf) * standard_g;
    m.linear_acceleration.z = imu_la_z(buf) * standard_g;

    m.angular_velocity.x = imu_av_x(buf) * M_PI / 180.0;
    m.angular_velocity.y = imu_av_y(buf) * M_PI / 180.0;
    m.angular_velocity.z = imu_av_z(buf) * M_PI / 180.0;

    for (int i = 0; i < 9; i++) {
        m.orientation_covariance[i] = -1;
        m.angular_velocity_covariance[i] = 0;
        m.linear_acceleration_covariance[i] = 0;
    }
    for (int i = 0; i < 9; i += 4) {
        m.linear_acceleration_covariance[i] = 0.01;
        m.angular_velocity_covariance[i] = 6e-4;
    }
    
    if (rotate_using_imu)
    {
        // to do: convert the gravity vector into a one-dimensional rotation around the gimbal axis
        // which counteracts the motor rotation
        float angle = atan2(-m.linear_acceleration.y, -m.linear_acceleration.z);
        // If angle is - then we are rolling left, else rolling right
        imuRotate = Eigen::AngleAxis<float>(angle, Eigen::Vector3f::UnitX());
        if ((m.header.stamp - last_log_time).toSec() > 1.0f)
        {
            last_log_time = m.header.stamp;
            int angle_deg = angle * 180 / M_PI;
            printf("(%f,%f,%f)=%d ",
                (float)m.linear_acceleration.x,
                (float)m.linear_acceleration.y,
                (float)m.linear_acceleration.z,
                angle_deg);
            fflush(stdout);
        }
    }
    return m;
}

// -----------------------------------------------------------------------------
sensor_msgs::PointCloud2 cloud_to_cloud_msg(const CloudOS1& cloud, ns timestamp,
                                            const std::string& frame) {
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);
    msg.header.frame_id = frame;
    msg.header.stamp.fromNSec(timestamp.count());
    return msg;
}

// -----------------------------------------------------------------------------
// Get the (transformed) point from a LIDAR buffer in converted form
static PointOS1 nth_point(int ind, const uint8_t* col_buf) {
    float h_angle_0 = col_h_angle(col_buf);
    auto tte = trig_table[ind];
    const uint8_t* px_buf = nth_px(ind, col_buf);
    float r = px_range(px_buf) / 1000.0;
    float h_angle = tte.beam_azimuth_angles + h_angle_0;

    PointOS1 point;
    point.reflectivity = px_reflectivity(px_buf);
    point.intensity = px_signal_photons(px_buf);
    if (rotate_using_imu)
    {
        auto pt1 = Eigen::Vector3f(
            -r * tte.cos_beam_altitude_angles * cosf(h_angle),
            r * tte.cos_beam_altitude_angles * sinf(h_angle),
            r * tte.sin_beam_altitude_angles);
        auto pt2 = imuRotate * pt1;
        
        point.x = pt2.x();
        point.y = pt2.y();
        point.z = pt2.z();
    }
    else
    {
        point.x = -r * tte.cos_beam_altitude_angles * cosf(h_angle);
        point.y = r * tte.cos_beam_altitude_angles * sinf(h_angle);
        point.z = r * tte.sin_beam_altitude_angles;
    }
    point.ring = ind;
    return point;
}

// -----------------------------------------------------------------------------
// Process a lidar packet and send the generated points to the ros cloud (up to 1024 of)
void add_packet_to_cloud(ns scan_start_ts, ns scan_duration,
                         const PacketMsg& pm, CloudOS1& cloud) {
    const uint8_t* buf = pm.buf.data();
    
    static int idecimatetoggle = 0;
    idecimatetoggle++;
    int idecimate = (idecimatetoggle & 64) ? 64-1 : 0-1;
    for (int icol = 0; icol < columns_per_buffer; icol++) {
        const uint8_t* col_buf = nth_col(icol, buf);
        float ts = (col_timestamp(col_buf) - scan_start_ts.count()) /
                   (float)scan_duration.count();

        for (int ipx = 0; ipx < pixels_per_column; ipx++) {
            if ((++idecimate & ouster_ros::OS1::decimate_mask) == 0) {
                auto p = nth_point(ipx, col_buf); // Get the (transformed) point
                p.t = ts;
                cloud.push_back(p); // Add the transformed point to the ROS cloud
            }
        }
    }
}

// -----------------------------------------------------------------------------
// Main spin loop in non-replay mode
void spin(const client& cli,
          const std::function<void(const PacketMsg& pm)>& lidar_handler, // Callback for LIDAR packets
          const std::function<void(const PacketMsg& pm)>& imu_handler) { // Callback for IMU packets
    // Prepare (single buffer) of input packets
    PacketMsg lidar_packet, imu_packet;
    lidar_packet.buf.resize(lidar_packet_bytes + 1);
    imu_packet.buf.resize(imu_packet_bytes + 1);

    // This loop customises ros::spin()
    while (ros::ok()) {
        // Poll the UDP sockets for valid data. This code is in ouster_client rather than ouster_ros
        auto state = poll_client(cli);
        if (state & ERROR) {
            ROS_ERROR("spin: poll_client returned error");
            return;
        }
        // Read and process LIDAR data if valid
        if (state & LIDAR_DATA) {
            if (read_lidar_packet(cli, lidar_packet.buf.data()))
                lidar_handler(lidar_packet);
        }
        // Read and process IMU data 
        if (state & IMU_DATA) {
            if (read_imu_packet(cli, imu_packet.buf.data()))
                imu_handler(imu_packet);
        }
        ros::spinOnce();
    }
}

// -----------------------------------------------------------------------------
static ns nearest_scan_dur(ns scan_dur, ns ts) {
    return ns((ts.count() / scan_dur.count()) * scan_dur.count());
};

// -----------------------------------------------------------------------------
// Create a functor (lambda) that is the lidar handler.
// It processes a lidar packet and adds many points to the cloud
std::function<void(const PacketMsg&)> batch_packets(
    ns scan_dur, const std::function<void(ns, const CloudOS1&)>& f) {
    auto cloud = std::make_shared<OS1::CloudOS1>();
    auto scan_ts = ns(-1L);

    return [=](const PacketMsg& pm) mutable {
        ns packet_ts = OS1::timestamp_of_lidar_packet(pm);
        if (scan_ts.count() == -1L)
            scan_ts = nearest_scan_dur(scan_dur, packet_ts);

        OS1::add_packet_to_cloud(scan_ts, scan_dur, pm, *cloud);

        auto batch_dur = packet_ts - scan_ts;
        if (batch_dur >= scan_dur || batch_dur < ns(0)) {
            f(scan_ts, *cloud);

            cloud->clear();
            scan_ts = ns(-1L);
        }
    };
}
}
}
