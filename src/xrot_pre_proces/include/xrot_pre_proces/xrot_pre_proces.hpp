//cpp libraries
#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <time.h>

//ros libraries
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

//pcl libreries
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/console/time.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/point_cloud2_iterator.h>


#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/features/moment_of_inertia_estimation.h>

#include <dynamic_reconfigure/server.h>

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include <chrono>

typedef pcl::PointXYZI pointType;
typedef pcl::PointCloud<pointType> pointCloud;
typedef pcl::PointXYZINormal pointTypeNormal;

//define usefull parameters
#define PI 3.14159265
#define TODEG 57.2957
#define TORAD 0.0174

using namespace std;
using namespace Eigen;

//change NodeName , TopicName , TopicType 
namespace xrot_pre_proces
{
    class xrot_pre_proces
    {

    	protected:
        ros::NodeHandle nh_;
        ros::Subscriber sub_cloudPoints,
						sub_CostCloud,
            sub_Local,
            sub_EulAngles,
            sub_RowEntryGoal,
            sub_RowLooka,
            sub_PathEgo,
            sub_PathAngle;

        ros::Publisher  pub_GroundRemoved,
                        pub_RowCloud,
                        pub_CleanCloud,
						pub_CloudTemp1,
						pub_CloudTemp2,
                        pub_InsideRowCloud,
                        pub_LeftRow,
                        pub_RightRow,
                        pub_Debug,
                        pub_RegionCloud,
                        pub_InsideRow;
        ros::Publisher  debug_Point1, debug_Point2, debug_Point3;
    	public:
        xrot_pre_proces(ros::NodeHandle nh);
        virtual ~xrot_pre_proces();
        pcl::visualization::PCLVisualizer::Ptr viewer;

        bool vox_filt, pass_filt,record_log,pub_clean_cloud,activate_filtering,ransac_check_prev_plane;
        int normal_k_search;

		struct single_row {
			pointCloud cluster_filt;
			pcl::ModelCoefficients coeff_ransac;
			pcl::PointIndices inliers_ransac;
			pointType x_min;
			pointType x_max;
			float dist_y;
		};
		struct row {
			pointCloud cluster1;
			pointCloud cluster2;
			pcl::ModelCoefficients coefficients1;
			pcl::ModelCoefficients coefficients2;
			pointType x_min_1;
			pointType x_max_1;
			pointType x_min_2;
			pointType x_max_2;
			float dist_y1;
			float dist_y2;
		};

		vector<pointCloud> found_clusters;
		vector<single_row> half_row;
		vector<row> detected_rows;

        tf::TransformListener listener;
		tf::StampedTransform transform_base_odom;
        double refresh_rate,
               prev_time,
      			   ROI_box_x,
      			   ROI_box_y,
               ROI_box_y_inside,
               ROI_box_y_outside,
      			   ROI_box_z,
      			   ROI_box_x_back,
      			   vox_leaf,
      			   ground_cluster_normal_th,
      			   euclidian_cluster_min_size,
      			   euclidian_cluster_max_size,
      			   euclidian_cluster_tollerance,
               euclidian_cluster_min_size_insiderow,
               euclidian_cluster_max_size_insiderow,
               euclidian_cluster_tollerance_insiderow,
      			   ransac_ground_distance_th,
      			   ransac_ground_angle_th,
               ransac_ground_cut_z_th,
               ransac_row_distance_th,
               ransac_row_angle_th,
               ransac_row_angle_th_inside,
               ransac_max_iterations,
               ransac_min_dist,
               ransac_line_angle_th,
               ransac_line_distance_th,
               ransac_line_angle_th_cluster,
               ransac_line_distance_th_cluster,
               row_width_limit,
               filter_cloud_divide_min_y,
               filter_cloud_divide_max_y;

        ofstream logfile;
        double left_y,
               right_y,
               left_ang,
               right_ang,
               line_step,
               line_inliers_dist_th;

        double ex_time;

        double left_y_prev,
               right_y_prev,
               left_ang_prev,
               right_ang_prev,
               prev_gain;

        double left_y_new,
               right_y_new,
               left_ang_new,
               right_ang_new;

        bool InsideRow,InsideRowPrev,CTRL_row_navigation,use_lines,use_layers,CTRL_polimi_row_navigation,use_parallels,use_groundremoval,use_fitline;

        double dist_from_next_change,
               dist_from_prev_change,
               dist_from_next_goal,
               out_row_height,
               in_home_height,
               second_layer_dz,
               third_layer_dz,
               dist_from_path_th,
               ang_cmd_th;

        string navigation_state;

        geometry_msgs::PoseStamped row_entry_goal_lidar,row_entry_goal,row_looka,path_ego; 

        double vehicle_orientation, vehicle_head, path_orientation,vehicle_x, vehicle_y,x , row_entry_x, row_entry_y;
        bool got_row_enter , use_other_segment, use_cut_row, use_cluster_row,use_path_orientation, prev_condition;
        Matrix4f mat_base_odom;

        double left_max_x,
               left_min_x,
               right_max_x,
               right_min_x,
               pcl_save_period,
               clean_cloud_period,
               region_limit_x,
               region_limit_y,
               region_limit_x_shift,
               region_discretization_N,
               region_limit_x_offset,
               region_limit_y_offset,
               inside_row_counter_time;
               
        unsigned int environment_state;
        std_msgs::Bool inside_row; 

        pointCloud pcl_to_save_left,pcl_to_save_right;
        double left_cluster_length, right_cluster_length,inside_row_length_th, outside_row_length_th, use_inliers_th,prev_gain_memory;

        void init();
        void callback_cloudPoints(const sensor_msgs::PointCloud2 current_cloud2);
        void callback_CostCloud(const sensor_msgs::PointCloud2 current_cloud2);
        void initialProcessing(pointCloud::Ptr);
        void removeGround(pointCloud::Ptr,pointCloud::Ptr);
        void filterInsideRows(pointCloud::Ptr,pointCloud::Ptr,pointCloud::Ptr,pointCloud::Ptr,pointCloud::Ptr);
        void findRows_clusters(pointCloud::Ptr,pointCloud::Ptr,pointType,pointType);
        void getLineInliers(pointCloud::Ptr, pointCloud::Ptr ,Eigen::Vector4f ,double,double);
        void log_to_file();
        void clusterRow(pointCloud::Ptr , pointCloud::Ptr , pointCloud::Ptr );
        void cutRow(pointCloud::Ptr , pointCloud::Ptr , pointCloud::Ptr );
        void fitPlane(pointCloud::Ptr , pointCloud::Ptr ,pointCloud::Ptr , pointCloud::Ptr );
        void fitLine(pointCloud::Ptr , pointCloud::Ptr ,pointCloud::Ptr , pointCloud::Ptr );
        void fitPlane_parallels(pointCloud::Ptr , pointCloud::Ptr ,pointCloud::Ptr , pointCloud::Ptr );
        void clusterRowInsideRow(pointCloud::Ptr , pointCloud::Ptr , pointCloud::Ptr );
        void divideRows(pointCloud::Ptr cloud_in,pointCloud::Ptr left_cloud ,pointCloud::Ptr right_cloud);

        void callback_Local(const nav_msgs::Odometry data);
        void callback_EulAngles(const std_msgs::Float32MultiArray data);
        void callback_RowEntryGoal(const geometry_msgs::PoseStamped data);
        void callback_RowLooka(const geometry_msgs::PoseStamped data);
        void callback_PathEgo(const geometry_msgs::PoseStamped data);

        void setTransMatrixFrames();

        void calFrontWallDist(pointCloud::Ptr);
        void countPointsIn4Reagions(pointCloud::Ptr);
        void cloud_distribution(pointCloud::Ptr left_cloud);
        void pclSaveCallback(const ros::TimerEvent&);
        void cleanCloudCallback(const ros::TimerEvent&);
        double unwrap(double x, double y);

        void get_cluster(pointCloud::Ptr cloud_in,pointCloud::Ptr cloud_out);
        void updateInsideRows(pointCloud::Ptr cloud);
        double get_cloud_length(pointCloud::Ptr cloud_in);

        ros::Timer timer_pcl_save,timer_clean_cloud;
    };
}

