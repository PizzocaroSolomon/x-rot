#include <xrot_pre_proces/xrot_pre_proces.hpp>
#include <thread>

namespace xrot_pre_proces
{
	xrot_pre_proces::xrot_pre_proces(ros::NodeHandle nh)
	:nh_(nh)
    {
        ros::NodeHandle private_nh("~");

        private_nh.param<double>("ROI_box_x", ROI_box_x, 6.0);
		private_nh.param<double>("ROI_box_y", ROI_box_y_outside, 4.0);
		private_nh.param<double>("ROI_box_x_back", ROI_box_x_back, 4.0);
		// private_nh.param<double>("ROI_box_z", ROI_box_z, 5.0);
		private_nh.param<double>("vox_leaf", vox_leaf, 0.07);

		private_nh.param<bool>("vox_filt", vox_filt, true);
		private_nh.param<bool>("pass_filt", pass_filt, true);
        private_nh.param<bool>("use_parallels", use_parallels, false);
        private_nh.param<bool>("use_fitline", use_fitline, false);
        private_nh.param<bool>("use_groundremoval", use_groundremoval, true);

		private_nh.param<double>("ground_cluster_normal_th", ground_cluster_normal_th, 0.05);
		private_nh.param<double>("euclidian_cluster_tollerance", euclidian_cluster_tollerance, 0.3);
		private_nh.param<double>("euclidian_cluster_min_size", euclidian_cluster_min_size, 200);
		private_nh.param<double>("euclidian_cluster_max_size", euclidian_cluster_max_size, 2000);

        private_nh.param<double>("euclidian_cluster_tollerance_insiderow", euclidian_cluster_tollerance_insiderow, 0.3);
        private_nh.param<double>("euclidian_cluster_min_size_insiderow", euclidian_cluster_min_size_insiderow, 200);
        private_nh.param<double>("euclidian_cluster_max_size_insiderow", euclidian_cluster_max_size_insiderow, 2000);

		private_nh.param<int>("normal_k_search", normal_k_search, 100);

		private_nh.param<double>("ransac_ground_distance_th", ransac_ground_distance_th, 0.35);
		private_nh.param<double>("ransac_ground_angle_th", ransac_ground_angle_th, 20);
        private_nh.param<double>("ransac_ground_cut_z_th", ransac_ground_cut_z_th, 1);

        private_nh.param<double>("ransac_row_distance_th", ransac_row_distance_th, 0.35);
        private_nh.param<double>("ransac_row_angle_th", ransac_row_angle_th, 20);
        private_nh.param<double>("ransac_row_angle_th_inside", ransac_row_angle_th_inside, 20);

		private_nh.param<double>("ransac_line_angle_th_cluster", ransac_line_angle_th_cluster, 3);
		private_nh.param<double>("ransac_line_distance_th_cluster", ransac_line_distance_th_cluster, 0.1);
		private_nh.param<double>("ransac_line_angle_th", ransac_line_angle_th, 10.0);
		private_nh.param<double>("ransac_line_distance_th", ransac_line_distance_th, 0.8); 
        private_nh.param<double>("ransac_max_iterations", ransac_max_iterations,50);
		private_nh.param<double>("row_width_limit",row_width_limit, 2);

        private_nh.param<double>("filter_cloud_divide_min_y",filter_cloud_divide_min_y,0.3);
        private_nh.param<double>("filter_cloud_divide_max_y",filter_cloud_divide_max_y,2.5);

        private_nh.param<double>("refresh_rate",refresh_rate,5);
        private_nh.param<bool>("record_log",record_log,false);
        private_nh.param<double>("line_step",line_step,0.5);
        private_nh.param<double>("line_inliers_dist_th",line_inliers_dist_th,0.5);

        private_nh.param<double>("prev_gain",prev_gain,0.8);
        private_nh.param<bool>("pub_clean_cloud",pub_clean_cloud,false); 
        private_nh.param<bool>("use_lines",use_lines,false); 
        private_nh.param<bool>("use_layers",use_layers,false); 

        private_nh.param<double>("in_home_height",in_home_height,1.5);
        private_nh.param<double>("out_row_height",out_row_height,1.2);

        private_nh.param<double>("second_layer_dz",second_layer_dz,0.2);
        private_nh.param<double>("third_layer_dz",third_layer_dz,0.7); 
        private_nh.param<double>("dist_from_path_th",dist_from_path_th,2.5);

        private_nh.param<double>("pcl_save_period",pcl_save_period,0);
        private_nh.param<double>("clean_cloud_period",clean_cloud_period,0);

        private_nh.param<double>("ransac_min_dist", ransac_min_dist,0.5);
        private_nh.param<bool>("ransac_check_prev_plane", ransac_check_prev_plane,false);

        private_nh.param<double>("region_limit_x", region_limit_x,2);
        private_nh.param<double>("region_limit_y", region_limit_y,2);
        private_nh.param<double>("region_limit_x_shift", region_limit_x_shift,0.5);
        private_nh.param<double>("region_limit_x_offset", region_limit_x_offset,0.3);
        private_nh.param<double>("region_limit_y_offset", region_limit_y_offset,0.1);
        private_nh.param<double>("region_discretization_N", region_discretization_N,6);
        private_nh.param<bool>("use_cut_row",use_cut_row,false);
        private_nh.param<bool>("use_cluster_row",use_cluster_row,false);
        private_nh.param<bool>("use_path_orientation",use_path_orientation,false);

	private_nh.param<double>("prev_gain", prev_gain_memory,0.8);

    }
    /****************************************************************
     *
     */
	xrot_pre_proces::~xrot_pre_proces()
    {
    }
    /*****************************************************************************************************************
     * Initialization
     */
    void xrot_pre_proces::init()
    {
    	//subscribers
    	sub_cloudPoints = nh_.subscribe<sensor_msgs::PointCloud2>("/point_raw" ,1 , &xrot_pre_proces::callback_cloudPoints, this);
    	//publishers
    	pub_GroundRemoved = nh_.advertise<sensor_msgs::PointCloud2> ("/cloud_no_ground", 1);

        if(record_log)
        {
          time_t     now = time(0);
          struct tm  tstruct;
          char       buf[80];
          tstruct = *localtime(&now);
          stringstream filename;
          strftime(buf, sizeof(buf), "%Y%m%d_%H%M", &tstruct);

          std::string path_ ;
          path_=ros::package::getPath("xrot_pre_proces");

          filename << path_<< "/logs/" << buf << "_"<<int(ROI_box_x)<<".txt";

          logfile.open(filename.str().c_str());

          logfile<<"time"<<";"<<
               "left_y_raw"<<";"<<
               "right_y_raw"<<";"<<
               "left_ang_raw"<<";"<<
               "right_ang_raw"<<";"<<
               "left_y_filt"<<";"<<
               "right_y_filt"<<";"<<
               "left_ang_filt"<<";"<<
               "right_ang_filt"<<";"<<
               "path_orientation"<<";"<<
               "vehicle_orientation"<<";"<<
               "vehicle_x"<<";"<<
               "vehicle_y"<<";"<<
               "left_min_x"<<";"<<
               "left_max_x"<<";"<<
               "right_min_x"<<";"<<
               "right_max_x"<<";"<<
               "ex_time"<<";"<<
               "InsideRow"<<'\n';
               
        }

        // init for complementary filter
        left_y_prev = 1.2;
        right_y_prev = -1.2;
        left_ang_prev = PI/2;
        right_ang_prev = PI/2;

        // init filtering param
        activate_filtering = false;
        got_row_enter = false;

        environment_state = 0;
        inside_row_counter_time = ros::Time::now().toSec();

    }

    void xrot_pre_proces::callback_cloudPoints(const sensor_msgs::PointCloud2 current_cloud2)
    {
    	// transform to base_footprint frame
    	pointCloud::Ptr cloud_in(new pointCloud);
        pointCloud::Ptr cloud_no_ground(new pointCloud);

    	sensor_msgs::PointCloud2 cloud2_in;
        sensor_msgs::PointCloud2 cloud2_no_ground;

        pcl::fromROSMsg(current_cloud2,*cloud_in);

        removeGround(cloud_in,cloud_no_ground);
        
		pcl::toROSMsg(*cloud_no_ground,cloud2_no_ground);
        cloud2_no_ground.header.frame_id = "lidar";
    	cloud2_no_ground.header.stamp = ros::Time::now();
    	pub_GroundRemoved.publish(cloud2_no_ground);


    }

    void xrot_pre_proces::removeGround(pointCloud::Ptr cloud_in,pointCloud::Ptr cloud_no_ground)
    {

		pcl::PassThrough<pointType> pass (true);
		pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients), coefficients_temp (new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices), inliers_temp (new pcl::PointIndices);
		pcl::PointIndices::Ptr ground_indices (new pcl::PointIndices), ground_indices_temp (new pcl::PointIndices);
		pcl::PointIndices::Ptr indices_rem (new pcl::PointIndices), indices_rem_temp (new pcl::PointIndices);
		pcl::SACSegmentation<pointType> seg;

		pass.setInputCloud(cloud_in);
		pass.setFilterFieldName("z");
		pass.setFilterLimits (-20,0.7);
		pass.filter(*cloud_no_ground);
		pass.filter(indices_rem->indices);

		seg.setOptimizeCoefficients (true);
		seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
		seg.setMethodType (pcl::SAC_RANSAC);

		seg.setDistanceThreshold (ransac_ground_distance_th);
		seg.setAxis(Eigen::Vector3f(0, 0, 1));
		seg.setEpsAngle(ransac_ground_angle_th*TORAD);
		seg.setInputCloud (cloud_no_ground);
		seg.setMaxIterations(200);
		seg.segment (*inliers, *coefficients);

		for(int i=0 ; i<inliers->indices.size();i++)
			ground_indices->indices.push_back(indices_rem->indices[ inliers->indices[i] ]);

		pcl::ExtractIndices<pointType> extract (true);
		extract.setInputCloud (cloud_in);
		extract.setIndices (ground_indices);
		extract.setNegative (true); 				//Set true to remove  inliers, false to mainatin only inliers
		extract.filter(*cloud_no_ground);

    }


}


int main (int argc, char** argv)
{
	// Initialize ROS
    ros::init (argc, argv, "xrot_pre_proces");
    ros::NodeHandle nh;
    //for registering the node
    xrot_pre_proces::xrot_pre_proces xrot_pre_proces_handler(nh);
    xrot_pre_proces_handler.init();

    ros::Rate loop_rate(xrot_pre_proces_handler.refresh_rate);
    
    while(ros::ok() )
    {
        xrot_pre_proces_handler.prev_time = ros::Time::now().toSec();
        ros::spinOnce();
//        ROS_INFO("row filter : total computation time %f ", ros::Time::now().toSec()-xrot_pre_proces_handler.prev_time );
        loop_rate.sleep();
    }
    
    xrot_pre_proces_handler.logfile.close();
    return 0;
}
