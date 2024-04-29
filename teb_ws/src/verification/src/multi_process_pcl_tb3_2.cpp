/////////////////////////////////////////////***imports***////////////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <monitoring/Boundingbox.h>
// #include <testrobots/newBoundingbox.h>
#include <monitoring/Plot.h>
#include <monitoring/yolodepth.h>

#include <string>
#include <vector>
#include <iostream>
#include <algorithm>
#include <stack>
#include <boost/lexical_cast.hpp>
#include <chrono>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>

// ROS Topics
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h> 
#include <pcl_ros/point_cloud.h>  
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>


// pcl
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_painter2D.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/impl/centroid.hpp>
#include <pcl/common/pca.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/point_types.h>

// std
#include <sstream>
#include <fstream>
#include <vector>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <vector>

using namespace std;
using namespace std::chrono;
using namespace std::chrono_literals;
int start;
std::string cp_flag;

//****************************************************declarations************************************************

//publisher declarations
ros::Publisher tf_pub;
ros::Publisher organizer;
ros::Publisher marker_pub;
ros::Publisher cloud_for_poly;
ros::Publisher rad_ros_pub;
ros::Publisher pub_cropped_cloud;
ros::Publisher pub_extracted_cloud;
ros::Publisher pub_projected_cloud;
ros::Publisher passthrough_filtered;
ros::Publisher passthrough_filtered_again;

//object declarations
pcl::PCDReader reader; 
pcl::PCDWriter writer;
pcl::VoxelGrid<pcl::PCLPointCloud2> vg;
pcl::PassThrough<pcl::PointXYZ> pass_filter;
pcl::PassThrough<pcl::PointXYZ> pass_filter2;

//msg declarations
sensor_msgs::PointCloud2 rad_ros;
sensor_msgs::PointCloud2 obj_msg;
sensor_msgs::PointCloud2 proj_msg;
visualization_msgs::Marker marker;
sensor_msgs::PointCloud2 crop_cloud_msg;
sensor_msgs::PointCloud2 passfiltered_ros;
sensor_msgs::PointCloud2 passfiltered_ros_again;


// variables and pointers
Eigen::Matrix<float,4,1> mean;
Eigen::Matrix<float, 3,3> cov_matrix;
pcl::PCLPointCloud2 passfiltered_pcl2;
pcl::PointCloud<pcl::PointXYZ> final_cloud;
pcl::PCLPointCloud2 passfiltered_pcl2_again;
std::string frame_id="tb3_2_tf/camera_rgb_optical_frame";
pcl::PointCloud<pcl::PointXYZ> pass_filtered_cloud;
static const std::string PCL_TOPIC = "/tb3_2/camera/depth/points";
pcl::PCLPointCloud2::Ptr inputCloud (new pcl::PCLPointCloud2());
pcl::PCLPointCloud2::Ptr outputCloud (new pcl::PCLPointCloud2());
pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
pcl::PointCloud<pcl::PointXYZ>::Ptr output_ptr(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr no_plane_cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr passfiltered_again (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr passfiltered_pclXYZ (new pcl::PointCloud<pcl::PointXYZ>);   

//function declarations:
void cp_flag_callback(std_msgs::String msg);
void extractObject(pcl::PointCloud<pcl::PointXYZ>::Ptr crop_cloud_ptr);
int counter = 0 ;



//////////////////////////////////////*****function definitions**********////////////////////////////////////////////////

void save_pcd(sensor_msgs::PointCloud2 ros_msg, int counter,string file_name ){
   pcl::PointCloud<pcl::PointXYZ> save_cloud;
   pcl::fromROSMsg(ros_msg,save_cloud);
   pcl::io::savePCDFileASCII (file_name + std::to_string(counter)+".pcd", save_cloud);

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) { 
   

   if (cp_flag == "yes")//if human is present 
   {
      ROS_INFO("Processing Cloud now....");
      //start timer
      auto start1 = high_resolution_clock::now(); 
      pcl_conversions::toPCL(*cloud_msg, *inputCloud);
      counter++;

      // ----------save to pcd--------------------  
      
   
      // pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      // pcl::PCLPointCloud2 pcl_pc2;
      // pcl_conversions::toPCL(*cloud_msg,pcl_pc2);
      // pcl::fromPCLPointCloud2(pcl_pc2,*m_cloud);
      // std::stringstream ss;
      // ss << "original"<< counter <<".pcd";
      // writer.write<pcl::PointXYZ>(ss.str(), *m_cloud, false);

   
   // ----------------------------------------

      //do voxel filtering and save to pcd   

      vg.setInputCloud(inputCloud);
      vg.setLeafSize(0.07,0.0,0.07);
      vg.filter(*outputCloud);
      std::cerr << "PointCloud after filtering: " << outputCloud->width * outputCloud->height << " data points (" << pcl::getFieldsList (*outputCloud) << ").\n" << std::endl;    
      pcl::fromPCLPointCloud2(*outputCloud, *output_ptr);

       //passthrough filtering z axis
      // pcl::PointCloud<pcl::PointXYZ>::Ptr passfiltered_pclXYZ (new pcl::PointCloud<pcl::PointXYZ>);
      // pcl::PassThrough<pcl::PointXYZ> pass_filter;

      pass_filter.setInputCloud (output_ptr);
      pass_filter.setFilterFieldName ("z");
      pass_filter.setFilterLimits (0, 4.0); //1.25,3.0
      pass_filter.setFilterLimitsNegative (false);  // try this with false
      pass_filter.filter (*passfiltered_pclXYZ);

      pcl::PCLPointCloud2 passfiltered_pcl2;
      sensor_msgs::PointCloud2 passfiltered_ros;
      pcl::toPCLPointCloud2(*passfiltered_pclXYZ, passfiltered_pcl2);
      pcl_conversions::fromPCL(passfiltered_pcl2, passfiltered_ros);
      passthrough_filtered.publish(passfiltered_ros);


      //passthrough filtering x axis
      // pcl::PointCloud<pcl::PointXYZ>::Ptr passfiltered_again (new pcl::PointCloud<pcl::PointXYZ>);
      // pcl::PassThrough<pcl::PointXYZ> pass_filter2;

      pass_filter2.setInputCloud (passfiltered_pclXYZ);
      pass_filter2.setFilterFieldName ("x");
      pass_filter2.setFilterLimits (-1.0, 1.0);
      pass_filter2.setFilterLimitsNegative (false);  // try this with false
      pass_filter2.filter (*passfiltered_again);

      pcl::PCLPointCloud2 passfiltered_pcl2_again;

      pcl::toPCLPointCloud2(*passfiltered_again, passfiltered_pcl2_again);
      pcl_conversions::fromPCL(passfiltered_pcl2_again, passfiltered_ros_again);
      passthrough_filtered_again.publish(passfiltered_ros_again);


      //call extract function and convert to ros msg and publish 

      std::cout<<"extracting object...\n"<< std::endl; 
      extractObject(passfiltered_again); //output_ptr
      pcl::toROSMsg(*no_plane_cloud.get(),obj_msg );
      pub_extracted_cloud.publish(obj_msg);


      //save extracted cloud to pcd   
      //save_pcd(obj_msg,counter, "extracted");


      // //call extract function and convert to ros msg and publish 

      // std::cout<<"extracting object...\n"<< std::endl; 
      // extractObject(output_ptr);
      // pcl::toROSMsg(*no_plane_cloud.get(),obj_msg );
      // pub_extracted_cloud.publish(obj_msg);


      // //save extracted cloud to pcd   
      // //save_pcd(obj_msg,counter, "extracted");


      // project points on XZ plane:
      pcl::ProjectInliers<pcl::PointXYZ> proj;
      coefficients->values.resize (4);
      coefficients->values[0] = 0;
      coefficients->values[1] = 1;
      coefficients->values[2] = 0.0;
      coefficients->values[3] = 0;

      proj.setModelType (pcl::SACMODEL_PLANE);
      proj.setInputCloud (no_plane_cloud);
      proj.setModelCoefficients (coefficients);
      proj.filter (*cloud_projected);
   

      pcl::toROSMsg(*cloud_projected.get(),proj_msg);
      pub_projected_cloud.publish(proj_msg); 
      pcl::fromROSMsg(proj_msg,final_cloud );

      // ----------save to pcd--------------------
      // std::stringstream bb;
      // bb << "final"<< counter <<".pcd";
      // writer.write<pcl::PointXYZ>(bb.str(), *cloud_projected, false);



      //calculate computation time

      auto stop1 = high_resolution_clock::now();
      auto duration1 = duration_cast<microseconds>(stop1 - start1);
      std::cout << "total time: "<< duration1.count()/1000000.0 << " s\n" << std::endl;
      std::cout << "**************************\n"<<std::endl;
    
   }
   else
   {
      ROS_INFO("tb3_2: NO human found");
   }
   

}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void extractObject(pcl::PointCloud<pcl::PointXYZ>::Ptr crop_cloud_ptr)
{
    no_plane_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);    
    pcl::PointIndices::Ptr planar_inliers (new pcl::PointIndices);// Cloud indices representing planar components inliers   
    pcl::ModelCoefficients::Ptr planar_coefficients (new pcl::ModelCoefficients); // Cloud coefficients for planar components inliers
    pcl::SACSegmentation<pcl::PointXYZ> SAC_filter;// Segmentation object
    pcl::ExtractIndices<pcl::PointXYZ> planar_inliers_extraction;// Euclidean Cluster Extraction object


    // Segmentation object initialization
    SAC_filter.setOptimizeCoefficients (true);
    SAC_filter.setModelType(pcl::SACMODEL_PLANE);
    SAC_filter.setMethodType (pcl::SAC_RANSAC);
    SAC_filter.setMaxIterations (200);
    SAC_filter.setDistanceThreshold (0.001);
   

    // Segment the dominant plane cluster
    SAC_filter.setInputCloud (crop_cloud_ptr);
    SAC_filter.segment (*planar_inliers, *planar_coefficients);

    if (planar_inliers->indices.size () == 0)
    {
       return ;
    }

    // Remove the planar cluster from the input cloud
    planar_inliers_extraction.setInputCloud (crop_cloud_ptr);
    planar_inliers_extraction.setIndices (planar_inliers);
    planar_inliers_extraction.setNegative (true);
    planar_inliers_extraction.filter (*no_plane_cloud);
    std::vector<int> no_Nan_vector;
    pcl::removeNaNFromPointCloud(*no_plane_cloud,*no_plane_cloud,no_Nan_vector);

  
   
}


void cp_flag_callback(std_msgs::String str){
   cp_flag = str.data;
   
}

////////////////////////***************main function**********************///////////////////////////////////////////////

int main(int argc, char **argv)
{
   
   ros::init (argc, argv, "tb3_2_cloud_processing");
   ros::NodeHandle nh;

   //subscribe
   
   ros::Subscriber PCLsub = nh.subscribe(PCL_TOPIC, 10, callback);
   ros::Subscriber cp_flag_sub = nh.subscribe("tb3_2/cp_flag", 10, cp_flag_callback);

   //set frame_id
   
   crop_cloud_msg.header.frame_id=frame_id;
   obj_msg.header.frame_id = frame_id;
   proj_msg.header.frame_id = frame_id;
   passfiltered_ros.header.frame_id = frame_id;
   passfiltered_ros_again.header.frame_id = frame_id;
   marker.header.frame_id = frame_id;

   
   
   //publish
   pub_cropped_cloud=nh.advertise<sensor_msgs::PointCloud2>("tb3_2/cropped_cloud",1);
   pub_extracted_cloud=nh.advertise<sensor_msgs::PointCloud2>("tb3_2/extracted_cloud",1);
   pub_projected_cloud=nh.advertise<sensor_msgs::PointCloud2>("tb3_2/projected",1);
   passthrough_filtered=nh.advertise<sensor_msgs::PointCloud2>("tb3_2/passfiltered",1);
   passthrough_filtered_again=nh.advertise<sensor_msgs::PointCloud2>("tb3_2/passfiltered_again",1);
   
   
   ros::spin();
   return 0;
   

}
