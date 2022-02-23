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

		private_nh.param<double>("k_search_th", k_search_th, 0.05);

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
        private_nh.param<bool>("use_zed_pointcloud", use_zed_pointcloud,false);

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
        cout<<"tyh "<<use_zed_pointcloud<<endl;
        if(use_zed_pointcloud)
        {
            cloud_frame_id = "zed2_left_camera_frame";
            sub_cloudPoints = nh_.subscribe<sensor_msgs::PointCloud2>("/zed2/zed_node/point_cloud/cloud_registered" ,1 , &xrot_pre_proces::callback_cloudPoints, this);
        }
        else
        {
            cloud_frame_id = "lidar";
    	    sub_cloudPoints = nh_.subscribe<sensor_msgs::PointCloud2>("/point_raw" ,1 , &xrot_pre_proces::callback_cloudPoints, this);
        }
    	//publishers
    	pub_BackGroundRemoved = nh_.advertise<sensor_msgs::PointCloud2> ("/cloud_no_background", 1);
        pub_GroundRemoved = nh_.advertise<sensor_msgs::PointCloud2> ("/cloud_no_ground", 1);
        pub_CleanCloud = nh_.advertise<sensor_msgs::PointCloud2> ("/cloud", 1);

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
        while(ros::Time::now().toSec()==0)
            prev_time = ros::Time::now().toSec();

    }

    void xrot_pre_proces::initialProcessing(pointCloud::Ptr cloud_in)
    {
    	pcl::PassThrough<pointType> pass (true);
		pcl::VoxelGrid<pointType> vox;

		if(pass_filt)
		{
            pointCloud::Ptr cloud_inside_x(new pointCloud), cloud_outside_x(new pointCloud);
            if(!InsideRow)
                ROI_box_y = ROI_box_y_outside;
            else
                ROI_box_y = ROI_box_y_inside;

			pass.setInputCloud(cloud_in);
			pass.setFilterFieldName("x");
			pass.setFilterLimits ( -ROI_box_x_back , ROI_box_x);
			pass.filter(*cloud_in);

			pass.setInputCloud(cloud_in);
			pass.setFilterFieldName("y");
			pass.setFilterLimits ( -ROI_box_y , ROI_box_y);
			pass.filter(*cloud_in);

            // remove point inside footprint robot
            pass.setInputCloud(cloud_in);
            pass.setFilterFieldName("x");
            pass.setFilterLimits(-1, 1);
            pass.filter(*cloud_inside_x);
            pass.setFilterLimitsNegative(true);
            pass.filter(*cloud_outside_x);
            pass.setInputCloud(cloud_inside_x);
            pass.setFilterFieldName("y");
            pass.setFilterLimits(-0.6, 0.6);
            pass.setFilterLimitsNegative(true);
            pass.filter(*cloud_in);

            *cloud_in += *cloud_outside_x;

		}

		if(vox_filt)
		{
			pointCloud::Ptr vox_cloud(new pointCloud);
			vox.setInputCloud (cloud_in);
			vox.setLeafSize ((float)vox_leaf, (float)vox_leaf, (float)vox_leaf);
			vox.filter (*vox_cloud);

			*cloud_in = *vox_cloud;

		}
    }

    void xrot_pre_proces::callback_cloudPoints(const sensor_msgs::PointCloud2 current_cloud2)
    {
        double curr_time = ros::Time::now().toSec();

    	// transform to base_footprint frame
    	pointCloud::Ptr cloud_init(new pointCloud);
        pointCloud::Ptr cloud_in(new pointCloud);
        pointCloud::Ptr cloud_no_ground(new pointCloud);

    	sensor_msgs::PointCloud2 cloud2_in;
        sensor_msgs::PointCloud2 cloud2_no_ground;

        pcl::fromROSMsg(current_cloud2,*cloud_in);
        
        pcl::toROSMsg(*cloud_in,cloud2_no_ground);
        cloud2_no_ground.header.frame_id = cloud_frame_id;
    	cloud2_no_ground.header.stamp = ros::Time::now();
    	pub_CleanCloud.publish(cloud2_no_ground);

        initialProcessing(cloud_in);
        removeGround(cloud_in,cloud_no_ground);

        pcl::toROSMsg(*cloud_no_ground,cloud2_no_ground);
        cloud2_no_ground.header.frame_id = cloud_frame_id;
    	cloud2_no_ground.header.stamp = ros::Time::now();
    	pub_GroundRemoved.publish(cloud2_no_ground);
        
        if(( curr_time - prev_time)<5)
        {
            initial_cloud+=*cloud_no_ground;
        }

        *cloud_init = initial_cloud;
        removeBackGround(cloud_no_ground,cloud_init);
        // *cloud_no_ground-=initial_cloud;

        pcl::toROSMsg(*cloud_no_ground,cloud2_no_ground);
        cloud2_no_ground.header.frame_id = cloud_frame_id;
    	cloud2_no_ground.header.stamp = ros::Time::now();
    	pub_BackGroundRemoved.publish(cloud2_no_ground);

    }

    void xrot_pre_proces::removeBackGround(pointCloud::Ptr cloud_in, pointCloud::Ptr cloud_back_ground)
    {
        // search inliers within radius
        pcl::KdTreeFLANN<pointType> kdtree;
        kdtree.setInputCloud (cloud_back_ground);
        pointType minPt, maxPt,Pt;

        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        pcl::ExtractIndices<pointType> extract (true);
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

        for(float i = 0 ; i< cloud_in->points.size() ; i+= 1){

            Pt = cloud_in->points[i];

            if ( kdtree.radiusSearch (Pt, k_search_th , pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 ){
                inliers->indices.push_back(i);
            }
        }

        extract.setInputCloud (cloud_in);
        extract.setIndices (inliers);
        extract.setNegative (true);
        extract.filter(*cloud_in);
            
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
        ros::spinOnce();
//        ROS_INFO("row filter : total computation time %f ", ros::Time::now().toSec()-xrot_pre_proces_handler.prev_time );
        loop_rate.sleep();
    }
    
    xrot_pre_proces_handler.logfile.close();
    return 0;
}
