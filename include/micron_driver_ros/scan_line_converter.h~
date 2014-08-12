/*******************************************************************************
 *
 *      scan_line_converter
 * 
 *      Copyright (c) 2011, Edward T. Kaszubski ( ekaszubski@gmail.com )
 *      All rights reserved.
 *
 *      Redistribution and use in source and binary forms, with or without
 *      modification, are permitted provided that the following conditions are
 *      met:
 *      
 *      * Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above
 *        copyright notice, this list of conditions and the following disclaimer
 *        in the documentation and/or other materials provided with the
 *        distribution.
 *      * Neither the name of "tritech_micron-RelWithDebInfo@tritech_micron" nor the names of its
 *        contributors may be used to endorse or promote products derived from
 *        this software without specific prior written permission.
 *      
 *      THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *      "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *      LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *      A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *      OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *      SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *      LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *      DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *      THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *      (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *      OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************/

#ifndef SCAN_LINE_CONVERTER_H_
#define SCAN_LINE_CONVERTER_H_


#include <micron_driver_ros/math.h>
#include <micron_driver_ros/ScanLine.h>


#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include "ros/ros.h"


//Service includes 
#include <micron_driver_ros/scanline_parser_reconfig.h>


typedef micron_driver_ros::ScanLine _ScanLineMsgType;
typedef micron_driver_ros::IntensityBin _IntensityBinMsgType;

typedef sensor_msgs::LaserScan _LaserScanMsgType;
typedef sensor_msgs::PointCloud _PointCloudMsgType;

typedef float _StepType;
typedef float _AngleType;
typedef std::vector<unsigned char> _IntensityBinsRawType;

class ScanLineConverter
{
public:
	//Subscribers
	ros::Subscriber scan_line_sub_;
	
	//Publishers
	ros::Publisher laser_scan_pub_;
	ros::Publisher point_cloud_pub_;
	ros::Publisher image_pub_;

	//Services
	ros::ServiceServer reconfigserver;
	
	double min_laser_intensity_threshold, min_distance_threshold,min_point_cloud_intensity_threshold;
	bool use_point_cloud_threshold,only_first_point;


	float min_laser_distance_, max_laser_distance_, angular_distance_, last_scan_angle_;
	int last_angular_direction_;

	_LaserScanMsgType laser_scan_msg_;

	ScanLineConverter( ros::NodeHandle & nh )
					
	{
		//Subscribers
		scan_line_sub_ = nh.subscribe( "/micron_driver/scan_line",1,&ScanLineConverter::scanLineCB, this );
		//Publishers
		laser_scan_pub_ = nh.advertise<_LaserScanMsgType>( "laser_scan",1 );
		point_cloud_pub_ = nh.advertise<_PointCloudMsgType>( "point_cloud",1 );
		//Service
		reconfigserver = nh.advertiseService("Scanline_Reconfiguration", &ScanLineConverter::reconfig, this);
		
		getparams(nh);
		clearLaserStats();

	}

	bool reconfig (micron_driver_ros::scanline_parser_reconfig::Request &req,micron_driver_ros::scanline_parser_reconfig::Response &resp )
	{
		min_laser_intensity_threshold = req.min_laser_intensity_threshold;
		min_distance_threshold = req.min_distance_threshold;
		min_point_cloud_intensity_threshold = req.min_point_cloud_intensity_threshold;
		use_point_cloud_threshold =req.use_point_cloud_threshold; 
		only_first_point = req.only_first_point;

		resp.result = true; 

	return true; 
	}


	bool getparams(ros::NodeHandle &nh) 
	{
		
		if (nh.hasParam("/scan_line_parser/use_point_cloud_threshold")) 
			nh.getParam("/scan_line_parser/use_point_cloud_threshold", use_point_cloud_threshold);
		else {
	 		use_point_cloud_threshold = true;
			ROS_WARN("Did not find 'use_point_cloud_threshold' on the parameter Server, using default value instead"); 
		} 
		if (nh.hasParam("/scan_line_parser/min_distance_threshold")) 
			nh.getParam("/scan_line_parser/min_distance_threshold", min_distance_threshold);
		else {
	 		min_distance_threshold = 0.0;
			ROS_WARN("Did not find 'min_distance_threshold' on the parameter Server, using default value instead"); 
		}
		if (nh.hasParam("/scan_line_parser/min_point_cloud_intensity_threshold")) 
			nh.getParam("/scan_line_parser/min_point_cloud_intensity_threshold", min_point_cloud_intensity_threshold);
		else {
	 		min_point_cloud_intensity_threshold = 50;
			ROS_WARN("Did not find 'min_point_cloud_intensity_threshold' on the parameter Server, using default value instead"); 
		}
		if (nh.hasParam("/scan_line_parser/min_laser_intensity_threshold")) 
			nh.getParam("/scan_line_parser/min_laser_intensity_threshold", min_laser_intensity_threshold);
		else {
	 		min_laser_intensity_threshold = 50;
			ROS_WARN("Did not find 'min_laser_intensity_threshold' on the parameter Server, using default value instead"); 
		}
		if (nh.hasParam("/scan_line_parser/only_first_point")) 
			nh.getParam("/scan_line_parser/only_first_point", only_first_point);
		else {
	 		only_first_point = true;
			ROS_WARN("Did not find 'only_first_point' on the parameter Server, using default value instead"); 
		}
	}
	
	void clearLaserStats()
	{
		laser_scan_msg_.ranges.clear();
		laser_scan_msg_.intensities.clear();
		min_laser_distance_ = std::numeric_limits<float>::max();
		max_laser_distance_ = std::numeric_limits<float>::min();
		angular_distance_ = 0;
		last_scan_angle_ = 0;
		last_angular_direction_ = 0;
	}

	//Callback when a scanline is received
	void scanLineCB( const _ScanLineMsgType::ConstPtr & scan_line_msg )
	{
				
		publishLaserScan( scan_line_msg );

		publishPointCloud( scan_line_msg );


	}

	

	void publishLaserScan( const _ScanLineMsgType::ConstPtr & scan_line_msg )
	{
		_IntensityBinMsgType bin = getThresholdedScanLine( scan_line_msg );

		if( laser_scan_msg_.ranges.size() == 0 )
		{
			laser_scan_msg_.angle_min = math_utils::degToRad( scan_line_msg->angle );
			laser_scan_msg_.header = scan_line_msg->header;
			last_scan_angle_ = scan_line_msg->angle;
			last_angular_direction_ = 0;
		}

		// 170 -> -170 = 20
		// -170 -> 170 = -20

		float angular_distance_inc;

		if( last_scan_angle_ > 90 && scan_line_msg->angle < -90 ) angular_distance_inc = ( 180 - last_scan_angle_ ) + ( scan_line_msg->angle + 180 );
		else if( last_scan_angle_ < -90 && scan_line_msg->angle > 90 ) angular_distance_inc = ( -180 - last_scan_angle_ ) + ( scan_line_msg->angle - 180 );
		else angular_distance_inc = scan_line_msg->angle - last_scan_angle_;

		int angular_direction = angular_distance_inc > 0 ? 1 : angular_distance_inc < 0 ? -1 : 0;

		angular_distance_ += angular_distance_inc;

		printf( "%f %f %f %f %d\n", laser_scan_msg_.angle_min, scan_line_msg->angle, last_scan_angle_, angular_distance_, angular_direction );

		last_scan_angle_ = scan_line_msg->angle;

		laser_scan_msg_.intensities.push_back( bin.intensity / 255.0 );
		laser_scan_msg_.ranges.push_back( bin.distance );

		if( scan_line_msg->bins.front().distance < min_laser_distance_ ) min_laser_distance_ = scan_line_msg->bins.front().distance;
		if( scan_line_msg->bins.back().distance > max_laser_distance_ ) max_laser_distance_ = scan_line_msg->bins.back().distance;

		if( fabs( angular_distance_ ) >= 300 || ( angular_direction != last_angular_direction_ && angular_direction != 0 && last_angular_direction_ != 0 ) )
		{
			// gather statistics
			laser_scan_msg_.angle_max = math_utils::degToRad( scan_line_msg->angle );
			laser_scan_msg_.angle_increment = laser_scan_msg_.ranges.size() > 1 ? math_utils::degToRad( angular_distance_ / ( laser_scan_msg_.ranges.size() - 1 ) ) : 0;
			laser_scan_msg_.range_min = min_laser_distance_;
			laser_scan_msg_.range_max = max_laser_distance_;

			laser_scan_msg_.scan_time = ( scan_line_msg->header.stamp - laser_scan_msg_.header.stamp ).toSec();
			//laser_scan_msg_.header.stamp = ros::Time::now() - ( scan_line_msg->header.stamp - laser_scan_msg_.header.stamp );
			laser_scan_msg_.time_increment = laser_scan_msg_.ranges.size() > 1 ? laser_scan_msg_.scan_time / ( laser_scan_msg_.ranges.size() - 1 ) : 0;

			// publish
			if ( laser_scan_msg_.ranges.size() > 0 ) laser_scan_pub_.publish( _LaserScanMsgType::Ptr( new _LaserScanMsgType( laser_scan_msg_ ) ) );
			clearLaserStats();
		}

		if( angular_direction != 0 ) last_angular_direction_ = angular_direction;
	}


	//This function loops through the received scanline until it finds a intensity greater then the defined threshold. 
	//It then returns the intensity and the corresponding distance 
	_IntensityBinMsgType getThresholdedScanLine( const _ScanLineMsgType::ConstPtr & scan_line_msg )
	{
		for (int i = 0; i < scan_line_msg->bins.size(); ++i )
		{
			if ( scan_line_msg->bins[i].distance < min_distance_threshold ) continue;

			/*if ( !reconfigure_params_.use_laser_threshold || scan_line_msg->bins[i].intensity >= reconfigure_params_.min_laser_intensity_threshold )
			{
				laser_scan_msg->intensities.push_back( scan_line_msg->bins[i].intensity / 255.0 );
				laser_scan_msg->ranges.push_back( scan_line_msg->bins[i].distance );
			}*/

			if ( scan_line_msg->bins[i].intensity >= min_laser_intensity_threshold ) return scan_line_msg->bins[i];
		}

		return _IntensityBinMsgType();
	}



	void publishPointCloud( const _ScanLineMsgType::ConstPtr & scan_line_msg )
	{
		_PointCloudMsgType::Ptr point_cloud_msg( new _PointCloudMsgType );
		point_cloud_msg->header = scan_line_msg->header;

		point_cloud_msg->points.reserve( scan_line_msg->bins.size() );

		sensor_msgs::ChannelFloat32 channel;
		channel.name = "intensity";
		channel.values.reserve( scan_line_msg->bins.size() );

		for ( int i = 0; i < scan_line_msg->bins.size(); ++i )
		{
			if ( scan_line_msg->bins[i].distance < min_distance_threshold ) continue;
			
			if ( !use_point_cloud_threshold  || scan_line_msg->bins[i].intensity >= min_point_cloud_intensity_threshold )
			{
				geometry_msgs::Point32 point;
				point.x = scan_line_msg->bins[i].distance * cos( math_utils::degToRad( scan_line_msg->angle ) );
				point.y = scan_line_msg->bins[i].distance * sin( math_utils::degToRad( scan_line_msg->angle ) );
				point.z = 0;
				point_cloud_msg->points.push_back( point );

				channel.values.push_back( scan_line_msg->bins[i].intensity / 255.0 );
				if (only_first_point) break; 
			}
		}

		point_cloud_msg->channels.push_back( channel );

		point_cloud_pub_.publish( point_cloud_msg );
	}
};

#endif /* SCAN_LINE_CONVERTER_H_ */
