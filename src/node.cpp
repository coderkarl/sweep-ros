
/*The MIT License (MIT)
 *
 * Copyright (c) 2017, Scanse, LLC
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "rclcpp/rclcpp.hpp"
#include <pcl/point_types.h>
#include "sensor_msgs/msg/point_cloud2.h"
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sweep/sweep.hpp>

class SweepNode : public rclcpp::Node
{
public:
	SweepNode() : Node("sweep_node")
    {
		//Setup Publisher
		m_pc2_pub = create_publisher<sensor_msgs::msg::PointCloud2>("pc2", 1000);

		m_serial_port = declare_parameter("serial_port", "/dev/ttyUSB0");
        m_serial_baudrate = declare_parameter("serial_baudrate", 115200);
        m_rotation_speed = declare_parameter("rotation_speed", 5);
        m_sample_rate = declare_parameter("sample_rate", 500);
        m_frame_id = declare_parameter("frame_id", "laser");

        RCLCPP_INFO(this->get_logger(), "Sweep Node (port = %s) Initialized", m_serial_port.c_str());

        //Create Sweep Driver Object
        m_device_ptr = new sweep::sweep(m_serial_port.c_str() );

        //Send Rotation Speed
        m_device_ptr->set_motor_speed(m_rotation_speed);

        //Send Sample Rate
        m_device_ptr->set_sample_rate(m_sample_rate);

        RCLCPP_INFO(this->get_logger(), "expected rotation frequency: %d (Hz)", m_rotation_speed);

        //Start Scan
        m_device_ptr->start_scanning();
    }

	~SweepNode()
	{
		try
		{
			m_device_ptr->stop_scanning();
		}
		catch(const sweep::device_error& e)
		{
			std::cerr << "Error: " << e.what() << std::endl;
		}
	}

	void update()
	{
		const sweep::scan scan = m_device_ptr->get_scan();
		publish_scan(&scan, m_frame_id);
	}

	void publish_scan(const sweep::scan *scan, std::string frame_id)
	{
	    pcl::PointCloud<pcl::PointXYZ> cloud;
	    sensor_msgs::msg::PointCloud2 cloud_msg;
	    rclcpp::Time ros_now = now();

	    float angle;
	    int32_t range;
	    float x;
	    float y;
	    int i = 0;

	    cloud.height = 1;
	    cloud.width = scan->samples.size();
	    cloud.points.resize(cloud.width * cloud.height);

	    for (const sweep::sample& sample : scan->samples)
	    {
	        range = sample.distance;
	        angle = ((float)sample.angle / 1000); //millidegrees to degrees

	        //Polar to Cartesian Conversion
	        x = (range * cos(DEG2RAD(angle))) / 100;
	        y = (range * sin(DEG2RAD(angle))) / 100;

	        cloud.points[i].x = x;
	        cloud.points[i].y = y;
	        i++;
	    }

	    //Convert pcl PC to ROS PC2
	    pcl::toROSMsg(cloud, cloud_msg);
	    cloud_msg.header.frame_id = frame_id;
	    cloud_msg.header.stamp = ros_now;

	    RCLCPP_DEBUG(this->get_logger(), "Publishing a full scan");
	    m_pc2_pub->publish(cloud_msg);
	}

    void stop()
    {
    	m_device_ptr->stop_scanning();
    }

private:
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2> > m_pc2_pub;

    std::string m_serial_port;
    int m_serial_baudrate;
    int m_rotation_speed;
    int m_sample_rate;
    std::string m_frame_id;

    sweep::sweep* m_device_ptr;
};


int main(int argc, char *argv[]) try
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<SweepNode>();

    while (rclcpp::ok())
    {
        //Grab Full Scan
        node->update();
        rclcpp::spin_some(node);
    }

    //Stop Scanning & Destroy Driver
    node->stop();
}

    catch (const sweep::device_error& e) {
      std::cerr << "Error: " << e.what() << std::endl;
}
