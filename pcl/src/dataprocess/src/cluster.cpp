#include <pcl/ModelCoefficients.h>
#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include "dataprocess/ClusterArray.h"

#include <std_msgs/String.h>
#include <pcl_ros/point_cloud.h>
#include <iostream>
#include <pcl/point_types.h>
//#include <pcl/filters/passthrough.h>
//#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
//#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
//#include <pcl/sample_consensus/method_types.h>
//#include <pcl/sample_consensus/model_types.h>
//#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>

const int region_max_ =7;
int regions_[100];
std::string sensor_model_;
std::string frame_id_;
bool print_fps_;
float z_axis_min_;
float z_axis_max_;
int cluster_size_min_;
int cluster_size_max_;
uint32_t cluster_array_seq_ = 0;
uint32_t pose_array_seq_ = 0;

Eigen::Vector4f min_, max_;

int frames; clock_t start_time; bool reset = true;//fps


typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;
//typedef pcl::PointCloud<pcl::PointXYZRGB> PointType;

class SubscribeAndPublish{
    public:
        SubscribeAndPublish(){
            
            /*** Publishers ***/
            ros::NodeHandle private_nh("~");
            cluster_array_pub_ = private_nh.advertise<dataprocess::ClusterArray>("clusters", 100);
            cloud_filtered_pub_ = private_nh.advertise<sensor_msgs::PointCloud2>("cloud_filtered", 100);
            pose_array_pub_ = private_nh.advertise<geometry_msgs::PoseArray>("poses", 100);
            marker_array_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>("markers", 100);
            
            /*** Parameters ***/
            private_nh.param<std::string>("sensor_model", sensor_model_, "HDL-32E"); // VLP-16, HDL-32E, HDL-64E
            private_nh.param<std::string>("frame_id", frame_id_, "velodyne2");
            private_nh.param<bool>("print_fps", print_fps_, false);
            private_nh.param<float>("z_axis_min", z_axis_min_, -2.4);
            private_nh.param<float>("z_axis_max", z_axis_max_, 0.5);
            private_nh.param<int>("cluster_size_min", cluster_size_min_, 5);
            private_nh.param<int>("cluster_size_max", cluster_size_max_, 700000);

 
            pub = n.advertise<PointCloud>("clustered",1);
            sub = n.subscribe("removed", 1, &SubscribeAndPublish::callback, this);

        }
        
        void callback(const PointCloud input){
            //.... do something with the input and generate the output...
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
            pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZI>);
            cloud->header.frame_id="clustered";
            cloud->width = input.width;
            cloud->height= input.height;
            cloud->points.resize(cloud->width * cloud->height);
            cloud->points = input.points;  
            //std::cout << cloud->height <<" "<< cloud->width<<" "; 
            
            
            boost::array<std::vector<int>, region_max_> indices_array;
            // Creating the KdTree object for the search method of the extraction
            std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZI>::Ptr > > clusters;
            float tolerance = 0.0;
            for(int i = 0; i < region_max_; i++) {
               tolerance += 0.1;
                //if();
                boost::shared_ptr<std::vector<int> > indices_array_ptr(new std::vector<int>(indices_array[i]));



            pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
            tree->setInputCloud (cloud);

            std::vector<pcl::PointIndices> cluster_indices;
            pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
            ec.setClusterTolerance (0.02); // 2cm
            ec.setMinClusterSize (100);
            ec.setMaxClusterSize (25000);
            ec.setSearchMethod (tree);
            ec.setInputCloud (cloud);
            ec.setIndices(indices_array_ptr);
            ec.extract (cluster_indices);
            
            for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();it !=cluster_indices.end();it++){
      	        pcl::PointCloud<pcl::PointXYZI>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZI>);
      	        for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
      	            cluster->points.push_back(cloud->points[*pit]);
  	            }
      	        cluster->width = cluster->size();
      	        cluster->height = 1;
      	        cluster->is_dense = true;
	            clusters.push_back(cluster); 
            }
            

            //pub.publish("hi"); 
            //int j = 0;
            //for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
            //{
            //    std::cout << " j= "<< j;
            //    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);
            //    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            //    {
            //        cloud_cluster->points.push_back (cloud->points[*pit]); //*
            //        //std::cout << " hi ";
            //    }
            //    cloud_cluster->width = cloud_cluster->points.size ();
            //    cloud_cluster->height = 1;
            //    cloud_cluster->is_dense = true;
            //    cloud_cluster->header.frame_id="clustered"; 
            //    pub.publish(cloud_cluster);
            //    j++; 
                //std::cout << "clustered = "<< cloud_cluster->height <<" "<< cloud_cluster->width << " ";
                //std::cout << "clustered= "<< cloud_cluster->height <<" ";
            //    std::cout << " " << cloud_cluster->points[1].z << " "<< std::endl;
            //}


            }
            dataprocess::ClusterArray cluster_array;
            geometry_msgs::PoseArray pose_array;
            visualization_msgs::MarkerArray marker_array;
            
            for(int i = 0; i < clusters.size(); i++) {
                if(cluster_array_pub_.getNumSubscribers() > 0) {
                    sensor_msgs::PointCloud2 ros_pc2_out;
                    pcl::toROSMsg(*clusters[i], ros_pc2_out);
                    cluster_array.clusters.push_back(ros_pc2_out);
                }
                
                 if(pose_array_pub_.getNumSubscribers() > 0) {
                    Eigen::Vector4f centroid;
                    pcl::compute3DCentroid(*clusters[i], centroid);
      
                    geometry_msgs::Pose pose;
                    pose.position.x = centroid[0];
                    pose.position.y = centroid[1];
                    pose.position.z = centroid[2];
                    pose.orientation.w = 1;
                    pose_array.poses.push_back(pose);
                }
    
                if(marker_array_pub_.getNumSubscribers() > 0) {
                    Eigen::Vector4f min, max;
                    pcl::getMinMax3D(*clusters[i], min, max);
      
                    visualization_msgs::Marker marker;
                    marker.header.stamp = ros::Time::now();
                    marker.header.frame_id = frame_id_;
                    marker.ns = "adaptive_clustering";
                    marker.id = i;
                    marker.type = visualization_msgs::Marker::LINE_LIST;
      
                    geometry_msgs::Point p[24];
                    p[0].x = max[0];  p[0].y = max[1];  p[0].z = max[2];
                    p[1].x = min[0];  p[1].y = max[1];  p[1].z = max[2];
                    p[2].x = max[0];  p[2].y = max[1];  p[2].z = max[2];
                    p[3].x = max[0];  p[3].y = min[1];  p[3].z = max[2];
                    p[4].x = max[0];  p[4].y = max[1];  p[4].z = max[2];
                    p[5].x = max[0];  p[5].y = max[1];  p[5].z = min[2];
                    p[6].x = min[0];  p[6].y = min[1];  p[6].z = min[2];
                    p[7].x = max[0];  p[7].y = min[1];  p[7].z = min[2];
                    p[8].x = min[0];  p[8].y = min[1];  p[8].z = min[2];
                    p[9].x = min[0];  p[9].y = max[1];  p[9].z = min[2];
                    p[10].x = min[0]; p[10].y = min[1]; p[10].z = min[2];
                    p[11].x = min[0]; p[11].y = min[1]; p[11].z = max[2];
                    p[12].x = min[0]; p[12].y = max[1]; p[12].z = max[2];
                    p[13].x = min[0]; p[13].y = max[1]; p[13].z = min[2];
                    p[14].x = min[0]; p[14].y = max[1]; p[14].z = max[2];
                    p[15].x = min[0]; p[15].y = min[1]; p[15].z = max[2];
                    p[16].x = max[0]; p[16].y = min[1]; p[16].z = max[2];
                    p[17].x = max[0]; p[17].y = min[1]; p[17].z = min[2];
                    p[18].x = max[0]; p[18].y = min[1]; p[18].z = max[2];
                    p[19].x = min[0]; p[19].y = min[1]; p[19].z = max[2];
                    p[20].x = max[0]; p[20].y = max[1]; p[20].z = min[2];
                    p[21].x = min[0]; p[21].y = max[1]; p[21].z = min[2];
                    p[22].x = max[0]; p[22].y = max[1]; p[22].z = min[2];
                    p[23].x = max[0]; p[23].y = min[1]; p[23].z = min[2];
                    for(int i = 0; i < 24; i++) {
  	                    marker.points.push_back(p[i]);
                    }
      
                    marker.scale.x = 0.02;
                    marker.color.a = 1.0;
                    marker.color.r = 0.0;
                    marker.color.g = 1.0;
                    marker.color.b = 0.5;
                    marker.lifetime = ros::Duration(0.1);
                    marker_array.markers.push_back(marker);
                    }

                    if(cluster_array.clusters.size()) {
                        cluster_array.header.seq = ++cluster_array_seq_;
                        cluster_array.header.stamp = ros::Time::now();
                        cluster_array.header.frame_id = frame_id_;
                        cluster_array_pub_.publish(cluster_array);
                    }

                    if(pose_array.poses.size()) {
                        pose_array.header.seq = ++pose_array_seq_;
                        pose_array.header.stamp = ros::Time::now();
                        pose_array.header.frame_id = frame_id_;
                        pose_array_pub_.publish(pose_array);
                    }
  
                    if(marker_array.markers.size()) {
                        marker_array_pub_.publish(marker_array);
                    }
  
                    if(print_fps_)if(++frames>10){std::cerr<<"[adaptive_clustering] fps = "<<float(frames)/(float(clock()-start_time)/CLOCKS_PER_SEC)<<", timestamp = "<<clock()/CLOCKS_PER_SEC<<std::endl;reset = true;}//fps


            } 
    

            
            /*** Parameters ***/
    //        private_nh.param<std::string>("sensor_model", sensor_model_, "HDL-32E"); // VLP-16, HDL-32E, HDL-64E
    //        private_nh.param<std::string>("frame_id", frame_id_, "velodyne2");
    //        private_nh.param<bool>("print_fps", print_fps_, false);
    //        private_nh.param<float>("z_axis_min", z_axis_min_, -0.5);
    //        private_nh.param<float>("z_axis_max", z_axis_max_, 5.0);
    //        private_nh.param<int>("cluster_size_min", cluster_size_min_, 5);
    //        private_nh.param<int>("cluster_size_max", cluster_size_max_, 700000);

        }
    
    
    
    private:
        ros::NodeHandle n;
         
        //ros::NodeHandle private_nh("~");

        ros::Publisher pub;
        
        ros::Publisher cluster_array_pub_;
        ros::Publisher cloud_filtered_pub_;
        ros::Publisher pose_array_pub_;
        ros::Publisher marker_array_pub_;
        
        ros::Subscriber sub;
};

int main(int argc, char **argv){
    ros::init(argc, argv, "clustering");
    SubscribeAndPublish SAPObject;
    ros::spin();
    return 0;
}

