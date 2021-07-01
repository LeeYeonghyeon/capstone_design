#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <cmath>
#include <pcl/segmentation/extract_clusters.h>
#include <vector>
#include <algorithm>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/voxel_grid.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <string>
#include <std_msgs/Float64.h>
#define PI 3.14159265

using namespace std;
class sys{
protected:
	ros::NodeHandle nh;

	ros::Publisher pub, pub_, pub__, pub___, pub____;
	ros::Subscriber sub_, sub2_;

	std_msgs::Float64 vel_msg, rad_msg;
	sensor_msgs::PointCloud msgCloud;
	sensor_msgs::PointCloud2 msgCloud2;
	sensor_msgs::PointCloud msgCloud3;
	sensor_msgs::PointCloud msgCloud4;
	sensor_msgs::PointCloud msgCloud5;
	pcl::PointCloud<pcl::PointXYZ> inputCloud;
	pcl::PointCloud<pcl::PointXYZ> filteredCloud;
	pcl::PassThrough<pcl::PointXYZ> pass;	
	float f_maxrange, f_minrange;
	
	
	float threshold = 0.97;
	string mode = "load...";
	bool double_lane = false;
	double st_offset = 0.7;
	double cr_offset = 1.2;
	double speed = 0;
	double angle = 0;
	
	double min_range = -1.3;
	double max_range = 1.3;
	int xx;
	float fnum, rnum, lnum, rdnum, ldnum;
	int flagsss = 0;
	double a, b, x1, y1, x2, y2, aa, bb, sum;
	double old_sum = 100000000;
	double aa1 = 0;
	double bb1 = 0;
	double aa2 = 0;
	double bb2 = 0;
//
	geometry_msgs::Point p, q;

public:

	sys(){
		initsetup();}
	~sys(){}
	
	void initsetup(){
		////////////////////////////////get param zone///////////////////////////////////////////
		nh.getParam("/warm_capstone_node/f_maxrange",f_maxrange);
		nh.getParam("/warm_capstone_node/f_minrange",f_minrange);
		/////////////////////////////////////////////////////////////////////////////////////////
		pub = nh.advertise<std_msgs::Float64>("/vel",1);
		pub_ = nh.advertise<std_msgs::Float64>("/rad",1);
		pub__ = nh.advertise<sensor_msgs::PointCloud>("warm_capstone_",1);
		pub___ = nh.advertise<sensor_msgs::PointCloud>("warm_capstone__",1);
		pub____ = nh.advertise<sensor_msgs::PointCloud>("warm_capstone___",1);
		sub_ = nh.subscribe("/scan", 1000, &sys::Callback, this);
		sub2_ = nh.subscribe("/Laser2PointCloud", 1, &sys::Callback2, this);


	}
	float rad2deg(float radian){
		return radian * 180/PI;
	}

	void corner_drive(int size, auto ranges, float angle_increment){
			
			if  (double_lane == true){
				float check_angle = atan((aa1+aa2)/2)*(180/PI);//*st_offset;
				if (check_angle < 19 && check_angle > -19){
				      	mode="st";	
					if(((ranges[size*0.25] + (ranges[size*0.75]-0.09))/2)*threshold > ranges[size*0.25]) 
						angle = -8;
					
					else if(((ranges[size*0.25] + (ranges[size*0.75]-0.09))/2)*threshold > ranges[size*0.75]) 
						angle = 8;
					else
						angle = check_angle * st_offset;
					speed = -11.5;
				}
				else{	
					mode = "free";
					angle = check_angle*cr_offset;
					speed = -12.33;
				}
			}
			else{
				float check_angle = atan(aa1)*(180/PI);//* st_offset;
				if(check_angle < 19 && check_angle > -19){
					mode="st";
					if(((ranges[size*0.25] + (ranges[size*0.75]-0.09))/2)*threshold > ranges[size*0.25]) 
						angle = -8;
					
					else if(((ranges[size*0.25] + (ranges[size*0.75]-0.09))/2)*threshold > ranges[size*0.75]) 
						angle = 8;
					else
						angle = check_angle * st_offset;
					speed = -11.5;

				}
				else{
					mode ="free";
					angle = check_angle*cr_offset;
					speed = -12.33;
				}


		}
	}


	
	void ransac(){
		old_sum = 100000000;
		for(int _ =0; _ < (msgCloud.points.size()*0.7); _++){ 
		int k = rand()%msgCloud.points.size();
		x1 = msgCloud.points[k].x;
		y1 = msgCloud.points[k].y;
		int l = rand()%(msgCloud.points.size()-1);
		if(l >= k)
			
		{
			l+=1;
			x2 = msgCloud.points[l].x;
			y2 = msgCloud.points[l].y;
		}

		try{	
			
			
			a = (y1-y2)/(x1-x2);
			b = (x1*y2-x2*y1)/(x1-x2);
			
		}catch(...){
			if ((x1-x2) == 0)
				continue;
		}

		

		sum = 0;
		for(int j=0; j<msgCloud.points.size(); j++)
		{
			double c = msgCloud.points[j].x;
			double d = msgCloud.points[j].y;
			sum = sum + pow((d-a*c-b),2);
		}

		if(sum < (old_sum))
		{
			old_sum = sum;
			aa = a;
			bb = b;
		}
		}
	}
		
	void Callback2(const sensor_msgs::PointCloud2::ConstPtr &pt2){
		pcl::fromROSMsg(*pt2, inputCloud);
		pcl::PointCloud<pcl::PointXYZ> tmpcloud;
		pcl::VoxelGrid<pcl::PointXYZ> vox;
    		vox.setInputCloud (inputCloud.makeShared());
    		vox.setLeafSize (0.1f, 0.1f, 0.1f); // set Grid Size(0.1m)
    		vox.filter (filteredCloud);	
		
		
		pass.setInputCloud (filteredCloud.makeShared());
		pass.setFilterFieldName ("x");
		pass.setFilterLimits(-1.7, -0.2);
		pass.setFilterLimitsNegative (false);
		pass.filter (tmpcloud);
		pass.setInputCloud (tmpcloud.makeShared());
		pass.setFilterFieldName ("y");
		pass.setFilterLimits(min_range, max_range);
		pass.setFilterLimitsNegative (false);
		pass.filter (filteredCloud);
		
		pcl::toROSMsg(filteredCloud, msgCloud2);

		sensor_msgs::convertPointCloud2ToPointCloud(msgCloud2, msgCloud5);
		pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZ>);
		kdtree -> setInputCloud (filteredCloud.makeShared());
		std::vector<pcl::PointIndices> clusterIndices;
		
		pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
		ec.setClusterTolerance(0.25); // set distance threshold = 1.5m
		ec.setMinClusterSize (8); //set Minimum Cluster Size
		ec.setMaxClusterSize (1000); //set Maximum Cluster Size
		ec.setSearchMethod (kdtree);
		ec.setInputCloud(filteredCloud.makeShared());
		ec.extract(clusterIndices);
		
		std::vector<pcl::PointIndices>::const_iterator it;
	
		for (it=clusterIndices.begin(); it!=clusterIndices.end(); ++it){
			pcl::PointCloud<pcl::PointXYZ> filteredCloud_;

			for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
				filteredCloud_.push_back(filteredCloud[*pit]);
				

	
			pcl::toROSMsg(filteredCloud_, msgCloud2);
			sensor_msgs::convertPointCloud2ToPointCloud(msgCloud2, msgCloud);
			if (flagsss == 0){
				flagsss = 1;	
				ransac();
				aa1 = aa;
				bb1 = bb;
				msgCloud3 = msgCloud;
				double_lane = false;
				}
			else if(flagsss == 1){
				flagsss = 2;
				ransac();
				aa2 = aa;
				bb2 = bb;
				double_lane = true;
				msgCloud4 = msgCloud;
				}
		}
			



	}
	
	void Callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg){
	
		int size = scan_msg->ranges.size();
		
		corner_drive(size, scan_msg->ranges, scan_msg->angle_increment);	
		
		//////////////////display imformation/////////////////////////	
		cout<<"******************************************"<<endl<<endl;
		cout<<"aa1 "<<atan(aa1)*180/PI<<" aa2 "<<atan(aa2)*180/PI<<endl;
		cout<<"double_lane -> "<<boolalpha<<double_lane<<endl;
		cout<<"track mode -> "<<mode<<endl<<endl;
		cout<<"angle "<<angle<<endl;
		cout<<"speed "<<speed<<endl<<endl;
		cout<<"******************************************"<<endl<<endl;
		for(int k=0; k < 7; k++)
			cout<<endl;
		/////////////////////////////////////////////////////////////

		vel_msg.data = speed;
		rad_msg.data = (PI/180)*angle;
	
		std_msgs::Header h = scan_msg->header;
		msgCloud3.header.frame_id="/map";
		msgCloud4.header.frame_id="/map";
		msgCloud3.header.stamp=h.stamp;
		msgCloud4.header.stamp=h.stamp;
		msgCloud5.header.frame_id="/map";
		msgCloud5.header.stamp=h.stamp;
		pub____.publish(msgCloud5);
		pub___.publish(msgCloud4);
		pub__.publish(msgCloud3);

		if (flagsss > 0){
			pub.publish(vel_msg);
			pub_.publish(rad_msg);
		}

		flagsss = 0;
	}
};

int main(int argc, char **argv){
	ros::init(argc, argv, "warm_capstone_node");
	sys ss;

	ros::spin();

	return 0;
}	
