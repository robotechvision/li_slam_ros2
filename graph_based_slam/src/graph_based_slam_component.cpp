#include "graph_based_slam/graph_based_slam_component.h"
#include <chrono>
#include "tf2_ros/create_timer_ros.h"
#include "rclcpp/serialization.hpp"
#include "fast_gicp/gicp/fast_gicp.hpp"

using namespace std::chrono_literals;

namespace graphslam
{
GraphBasedSlamComponent::GraphBasedSlamComponent(const rclcpp::NodeOptions & options)
: rtv_lifecycle::LifecycleNode("graph_based_slam", "", false, options)
{
  RCLCPP_INFO(get_logger(), "initialization start");

  declare_parameter("registration_method", "NDT");
  declare_parameter("voxel_leaf_size", 0.2);
  declare_parameter("ndt_resolution", 5.0);
  declare_parameter("ndt_num_threads", 0);
  declare_parameter("loop_detection_period", 1000);
  declare_parameter("threshold_loop_closure_score", 1.0);
  declare_parameter("distance_loop_closure", 20.0);
  declare_parameter("range_of_searching_loop_closure", 20.0);
  declare_parameter("search_submap_num", 3);
  declare_parameter("num_adjacent_pose_cnstraints", 5);
  declare_parameter("use_save_map_in_loop", true);
  declare_parameter("pose_graph_path", std::string("pose_graph.g2o"));
  declare_parameter("loaded_map_static", false);
  declare_parameter("rotation_epsilon", 2e-3);
  declare_parameter("transformation_epsilon", 5e-4);
  declare_parameter("max_num_iterations", 64);
  declare_parameter("num_threads", 10);
  declare_parameter("loam_frame_id", "laser_odom");
  declare_parameter("map_frame_id", "map");
}

GraphBasedSlamComponent::LifecycleCallbackReturn GraphBasedSlamComponent::on_configure(const rclcpp_lifecycle::State &) {
  tfbuffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      get_node_base_interface(), get_node_timers_interface());
  tfbuffer_->setCreateTimerInterface(timer_interface);
  listener_ = std::make_shared<tf2_ros::TransformListener>(*tfbuffer_);
  broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

  std::string registration_method;
  double voxel_leaf_size;
  double ndt_resolution;
  int ndt_num_threads;
  double rotaton_epsilon;
  double transformation_epsilon;
  int max_num_iterations;
  int num_threads;

  get_parameter("registration_method", registration_method);
  get_parameter("voxel_leaf_size", voxel_leaf_size);
  get_parameter("ndt_resolution", ndt_resolution);
  get_parameter("ndt_num_threads", ndt_num_threads);
  get_parameter("loop_detection_period", loop_detection_period_);
  get_parameter("threshold_loop_closure_score", threshold_loop_closure_score_);
  get_parameter("distance_loop_closure", distance_loop_closure_);
  get_parameter("range_of_searching_loop_closure", range_of_searching_loop_closure_);
  get_parameter("search_submap_num", search_submap_num_);
  get_parameter("num_adjacent_pose_cnstraints", num_adjacent_pose_cnstraints_);
  get_parameter("use_save_map_in_loop", use_save_map_in_loop_);
  get_parameter("pose_graph_path", pose_graph_path_);
  get_parameter("loaded_map_static", loaded_map_static_);
  get_parameter("rotaton_epsilon", rotaton_epsilon);
  get_parameter("transformation_epsilon", transformation_epsilon);
  get_parameter("max_num_iterations", max_num_iterations);
  get_parameter("num_threads", num_threads);
  get_parameter("loam_frame_id", loam_frame_id_);
  get_parameter("map_frame_id", map_frame_id_);

  std::cout << "registration_method:" << registration_method << std::endl;
  std::cout << "voxel_leaf_size[m]:" << voxel_leaf_size << std::endl;
  std::cout << "ndt_resolution[m]:" << ndt_resolution << std::endl;
  std::cout << "ndt_num_threads:" << ndt_num_threads << std::endl;
  std::cout << "loop_detection_period[Hz]:" << loop_detection_period_ << std::endl;
  std::cout << "threshold_loop_closure_score:" << threshold_loop_closure_score_ << std::endl;
  std::cout << "distance_loop_closure[m]:" << distance_loop_closure_ << std::endl;
  std::cout << "range_of_searching_loop_closure[m]:" << range_of_searching_loop_closure_ <<
    std::endl;
  std::cout << "search_submap_num:" << search_submap_num_ << std::endl;
  std::cout << "num_adjacent_pose_cnstraints:" << num_adjacent_pose_cnstraints_ << std::endl;
  std::cout << "use_save_map_in_loop:" << std::boolalpha << use_save_map_in_loop_ << std::endl;
  std::cout << "------------------" << std::endl;

  voxelgrid_.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);

  if (registration_method == "NDT") {
    boost::shared_ptr<pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>>
      ndt(new pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
    ndt->setMaximumIterations(100);
    ndt->setResolution(ndt_resolution);
    // ndt->setTransformationEpsilon(0.01);
    ndt->setTransformationRotationEpsilon(rotaton_epsilon);
    ndt->setTransformationEpsilon( transformation_epsilon);
    ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
    if (ndt_num_threads > 0) {ndt->setNumThreads(ndt_num_threads);}
    registration_ = ndt;
  } else if (registration_method == "GICP") {
    boost::shared_ptr<pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>>
      gicp(new pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>());
    gicp->setMaxCorrespondenceDistance(30);
    gicp->setMaximumIterations(100);
    //gicp->setCorrespondenceRandomness(20);
    gicp->setRotationEpsilon(rotaton_epsilon);
    gicp->setTransformationEpsilon(transformation_epsilon);
    gicp->setEuclideanFitnessEpsilon(1e-6);
    gicp->setRANSACIterations(0);
    registration_ = gicp;
  } else if (registration_method == "FastGICP") {
    boost::shared_ptr<fast_gicp::FastGICP<pcl::PointXYZI, pcl::PointXYZI>>
        gicp(new fast_gicp::FastGICP<pcl::PointXYZI, pcl::PointXYZI>());
    gicp->setNumThreads(num_threads);
    gicp->setDebugPrint(true);
    gicp->setTransformationEpsilon(transformation_epsilon);  //
    gicp->setMaximumIterations(max_num_iterations);
    gicp->setRANSACIterations(50);
    //gicp->setMaxCorrespondenceDistance(20);
    registration_ = gicp;
  } else {
    throw std::runtime_error("registration method " + registration_method + " does not exist");
  }

  initializePub();

  map_array_msg_.submaps.clear();
  loaded_submaps_cnt_ = 0;
  if (loadSubmaps(map_array_msg_, loop_edges_)) {
    loaded_submaps_cnt_ = map_array_msg_.submaps.size();
    RCLCPP_INFO(get_logger(), "Loaded reference map with %d submaps", static_cast<int>(loaded_submaps_cnt_));
    modified_map_pub_->on_activate();
    modified_map_array_pub_->on_activate();
    modified_path_pub_->on_activate();
    saved_path_pub_->on_activate();
    doPoseAdjustment(map_array_msg_, false, true);  // publish initial map
    modified_map_pub_->on_deactivate();
    modified_map_array_pub_->on_deactivate();
    modified_path_pub_->on_deactivate();
    saved_path_pub_->on_deactivate();
  }

  auto map_save_callback =
    [this](const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<std_srvs::srv::Empty::Request> request,
      const std::shared_ptr<std_srvs::srv::Empty::Response> response) -> void
    {
      std::cout << "Received an request to save the map" << std::endl;
      if (initial_map_array_received_ == false) {
        std::cout << "initial map is not received" << std::endl;
        return;
      }
      doPoseAdjustment(map_array_msg_, true);
    };

  map_save_srv_ = create_service<std_srvs::srv::Empty>("map_save", map_save_callback);

  // publish zero transform between loam and map frame
  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.frame_id = loam_frame_id_;
  transform_stamped.child_frame_id = map_frame_id_;
  broadcaster_->sendTransform(transform_stamped);

  auto initial_pose_callback =
      [this](const typename geometry_msgs::msg::PoseStamped::SharedPtr msg) -> void
      {
        if (msg->header.frame_id != loam_frame_id_ && msg->header.frame_id != map_frame_id_) {
          RCLCPP_WARN(get_logger(), "This initial_pose is not in the global frame");
          return;
        }
        RCLCPP_INFO(get_logger(), "initial_pose is received");

        // reset global transform to display initial pose correctly, although it will be set in loam frame by scanmatcher node
        if (msg->header.frame_id == map_frame_id_) {
          geometry_msgs::msg::TransformStamped transform_stamped;
          transform_stamped.header.frame_id = loam_frame_id_;
          transform_stamped.child_frame_id = map_frame_id_;
          broadcaster_->sendTransform(transform_stamped);
        }
      };

  initial_pose_sub_ =
      create_subscription<geometry_msgs::msg::PoseStamped>(
          "initial_pose", rclcpp::QoS(1), initial_pose_callback);

  return LifecycleCallbackReturn::SUCCESS;
}

GraphBasedSlamComponent::LifecycleCallbackReturn GraphBasedSlamComponent::on_activate(const rclcpp_lifecycle::State &) {
  initializeSub();

  modified_map_pub_->on_activate();
  modified_map_array_pub_->on_activate();
  modified_path_pub_->on_activate();
  saved_path_pub_->on_activate();
  input_path_pub_->on_activate();
  registration_pub_->on_activate();

  createBond();
  return LifecycleCallbackReturn::SUCCESS;
}
GraphBasedSlamComponent::LifecycleCallbackReturn GraphBasedSlamComponent::on_deactivate(const rclcpp_lifecycle::State &) {
  map_array_sub_.reset();
  loop_detect_timer_.reset();

  modified_map_pub_->on_deactivate();
  modified_map_array_pub_->on_deactivate();
  modified_path_pub_->on_deactivate();
  saved_path_pub_->on_deactivate();
  input_path_pub_->on_deactivate();
  registration_pub_->on_deactivate();

  destroyBond();
  return LifecycleCallbackReturn::SUCCESS;
}
GraphBasedSlamComponent::LifecycleCallbackReturn GraphBasedSlamComponent::on_cleanup(const rclcpp_lifecycle::State &) {
  modified_map_array_pub_.reset();
  modified_map_array_pub_.reset();
  modified_path_pub_.reset();
  saved_path_pub_.reset();
  input_path_pub_.reset();
  registration_pub_.reset();
  map_save_srv_.reset();

  broadcaster_.reset();
  listener_.reset();
  tfbuffer_.reset();
  return LifecycleCallbackReturn::SUCCESS;
}

void GraphBasedSlamComponent::initializePub()
{
  RCLCPP_INFO(get_logger(), "initialize Publishers");

  modified_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "modified_map",
    rclcpp::QoS(1).transient_local());

  registration_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "registration_cloud",
      rclcpp::QoS(1).transient_local());

  modified_map_array_pub_ = create_publisher<lidarslam_msgs::msg::MapArray>(
    "modified_map_array", rclcpp::QoS(1).transient_local());

  modified_path_pub_ = create_publisher<nav_msgs::msg::Path>(
    "modified_path",
    rclcpp::QoS(1).transient_local());

  saved_path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "saved_path",
      rclcpp::QoS(1).transient_local());

  input_path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "input_path",
      rclcpp::QoS(1).transient_local());

  RCLCPP_INFO(get_logger(), "initialization end");

}

void GraphBasedSlamComponent::initializeSub()
{
  RCLCPP_INFO(get_logger(), "initialize Subscribers");

  auto map_array_callback =
    [this](const typename lidarslam_msgs::msg::MapArray::SharedPtr msg_ptr) -> void
    {
      std::lock_guard<std::mutex> lock(mtx_);
      map_array_msg_.cloud_coordinate = msg_ptr->cloud_coordinate;
      map_array_msg_.header = msg_ptr->header;
      size_t existing_submaps_cnt = map_array_msg_.submaps.size();
      map_array_msg_.submaps.insert(map_array_msg_.submaps.end(), msg_ptr->submaps.begin() + (existing_submaps_cnt - loaded_submaps_cnt_), msg_ptr->submaps.end());

      // assign adjacency transform and adjust poses based on currently optimized graph
      for (size_t i = existing_submaps_cnt; i < map_array_msg_.submaps.size(); i++) {
        auto &submap = map_array_msg_.submaps[i];
        if (std::isfinite(submap.adjacency_transform.rotation.w)) {  // no adjacency breakage requested
          size_t msg_i = i - loaded_submaps_cnt_;
          Eigen::Isometry3d prev_pose, cur_pose, prev_pose_map, adjac_trans;
          tf2::fromMsg(msg_ptr->submaps[msg_i - 1].pose, prev_pose);
          tf2::fromMsg(submap.pose, cur_pose);
          tf2::fromMsg(map_array_msg_.submaps[i - 1].pose, prev_pose_map);
          adjac_trans = prev_pose.inverse()*cur_pose;
          submap.adjacency_transform = tf2::eigenToTransform(adjac_trans).transform;
          submap.pose = tf2::toMsg(prev_pose_map*adjac_trans);
        }
        else {  // break adjacency between previous and current submap
          // storing first pose as adjacency_transform to be able to reconstruct original poses (for visualization etc.)
          Eigen::Isometry3d pose;
          tf2::fromMsg(submap.pose, pose);
          submap.adjacency_transform = tf2::eigenToTransform(pose).transform;
          auto &w = submap.adjacency_transform.rotation.w;
          w = std::numeric_limits<double>::infinity() * (w >= 0 ? 1 : -1);  // storing sign to be able to reconstruct original value (||rot|| = 1)
        }
      }
      initial_map_array_received_ = true;
      is_map_array_updated_ = true;
    };

  map_array_sub_ =
    create_subscription<lidarslam_msgs::msg::MapArray>(
    "map_array", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(), map_array_callback);

  std::chrono::milliseconds period(loop_detection_period_);
  loop_detect_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&GraphBasedSlamComponent::searchLoop, this)
  );

  RCLCPP_INFO(get_logger(), "initialization end");
}

void GraphBasedSlamComponent::searchLoop()
{

  if (initial_map_array_received_ == false) {return;}
  if (is_map_array_updated_ == false) {return;}
  if (map_array_msg_.cloud_coordinate != map_array_msg_.LOCAL) {
    RCLCPP_WARN(get_logger(), "cloud_coordinate should be local, but it's not local.");
  }
  is_map_array_updated_ = false;

  lidarslam_msgs::msg::MapArray &map_array_msg = map_array_msg_;
  std::lock_guard<std::mutex> lock(mtx_);
  int num_submaps = map_array_msg.submaps.size();
  std::cout << "----------------------------" << std::endl;
  std::cout << "searching Loop, num_submaps:" << num_submaps << std::endl;

  double min_fitness_score = std::numeric_limits<double>::max();
  double distance_min_fitness_score = 0;
  bool is_candidate = false;

  pcl::console::setVerbosityLevel(pcl::console::VERBOSITY_LEVEL::L_VERBOSE);
  const lidarslam_msgs::msg::SubMap &latest_submap = map_array_msg.submaps[num_submaps - 1];
  Eigen::Affine3d latest_submap_affine;
  tf2::fromMsg(latest_submap.pose, latest_submap_affine);
  pcl::PointCloud<pcl::PointXYZI>::Ptr latest_submap_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(latest_submap.cloud, *latest_submap_cloud_ptr);
  // pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_latest_submap_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  Eigen::Affine3d latest_affine;
  tf2::fromMsg(latest_submap.pose, latest_affine);
  // pcl::transformPointCloud(*latest_submap_cloud_ptr, *transformed_latest_submap_cloud_ptr, latest_affine.matrix().cast<float>());
  registration_->setInputSource(latest_submap_cloud_ptr);
  double latest_moving_distance = latest_submap.distance;
  Eigen::Vector3d latest_submap_pos{
    latest_submap.pose.position.x,
    latest_submap.pose.position.y,
    latest_submap.pose.position.z};
  int id_min = 0;
  double min_dist = std::numeric_limits<double>::max();
  lidarslam_msgs::msg::SubMap const *min_submap = nullptr;
  for (int i = 0; i < num_submaps; i++) {
    const auto &submap = map_array_msg.submaps[i];
    Eigen::Vector3d submap_pos{submap.pose.position.x, submap.pose.position.y,
      submap.pose.position.z};
    double dist = (latest_submap_pos - submap_pos).norm();
    if (latest_moving_distance - submap.distance > distance_loop_closure_ &&
      dist < range_of_searching_loop_closure_)
    {
      if(dist < min_dist) {
        is_candidate = true;
        id_min = i;
        min_dist = dist;
        min_submap = &submap;
      }
    }
  }

  if (is_candidate) {
      RCLCPP_INFO(get_logger(), "Matching %d against %d", num_submaps - 1, id_min);
      pcl::PointCloud<pcl::PointXYZI>::Ptr submap_clouds_ptr(new pcl::PointCloud<pcl::PointXYZI>);
      for (int j = id_min - search_submap_num_; j <= id_min + search_submap_num_; ++j) {
        if (j < 0) {continue;}
        if (j >= num_submaps - 1) {break;}
        auto near_submap = map_array_msg.submaps[j];  //
        pcl::PointCloud<pcl::PointXYZI>::Ptr submap_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(near_submap.cloud, *submap_cloud_ptr);
        pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_submap_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
        Eigen::Affine3d affine;
        tf2::fromMsg(near_submap.pose, affine);
        affine = latest_affine.inverse()*affine;
        pcl::transformPointCloud(*submap_cloud_ptr, *transformed_submap_cloud_ptr, affine.matrix().cast<float>());
        *submap_clouds_ptr += *transformed_submap_cloud_ptr;
      }

      pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_clouds_ptr(new pcl::PointCloud<pcl::PointXYZI>());
      voxelgrid_.setInputCloud(submap_clouds_ptr);
      voxelgrid_.filter(*filtered_clouds_ptr);
      registration_->setInputTarget(filtered_clouds_ptr);

      pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
      registration_->align(*output_cloud_ptr);
      double fitness_score = registration_->getFitnessScore();

      Eigen::Isometry3d registration_trans(registration_->getFinalTransformation().cast<double>());

      if (registration_->hasConverged() && fitness_score < threshold_loop_closure_score_) {

        Eigen::Affine3d reference_affine;
        tf2::fromMsg(min_submap->pose, reference_affine);

        LoopEdge loop_edge;
        loop_edge.pair_id = std::pair<int, int>(num_submaps - 1, id_min);

        Eigen::Isometry3d latest_iso(latest_affine.matrix());
        Eigen::Isometry3d reference_iso(reference_affine.matrix());
        loop_edge.relative_pose = registration_trans.inverse() * latest_iso.inverse() * reference_iso;
        loop_edges_.push_back(loop_edge);

        std::cout << "---" << std::endl;
        std::cout << "PoseAdjustment" << std::endl;
        std::cout << "distance:" << min_submap->distance << ", score:" << fitness_score << std::endl;
        std::cout << "id_loop_point 1:" << num_submaps - 1 << std::endl;
        std::cout << "id_loop_point 2:" << id_min << std::endl;
        std::cout << "final transformation:" << std::endl;
        std::cout << registration_->getFinalTransformation() << std::endl;
        std::cout << "relative pose:" << std::endl;
        std::cout << loop_edge.relative_pose.matrix() << std::endl;
        doPoseAdjustment(map_array_msg, use_save_map_in_loop_);
      }

    pcl::PointCloud<pcl::PointXYZI>::Ptr registration_clouds_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    for (auto &p : latest_submap_cloud_ptr->points)
      p.intensity = 0;
    *registration_clouds_ptr += *latest_submap_cloud_ptr;
    for (auto &p : submap_clouds_ptr->points)
      p.intensity = 1;
    *registration_clouds_ptr += *submap_clouds_ptr;
    for (auto &p : latest_submap_cloud_ptr->points) {
      auto pe = p.getVector4fMap();
      pe = (registration_trans*pe.cast<double>()).eval().cast<float>();
      p.intensity = 2;
    }
    *registration_clouds_ptr += *latest_submap_cloud_ptr;
    sensor_msgs::msg::PointCloud2 registration_clouds_msg;
    pcl::toROSMsg(*registration_clouds_ptr, registration_clouds_msg);
    registration_clouds_msg.header.frame_id = map_frame_id_;
    registration_pub_->publish(registration_clouds_msg);

    std::cout << "-" << std::endl;
    std::cout << "min_submap_distance:" << min_submap->distance << std::endl;
    std::cout << "min_fitness_score:" << fitness_score << std::endl;
  }

  nav_msgs::msg::Path path;
  nav_msgs::msg::Path modified_path;
  path.header.frame_id = map_frame_id_;
  modified_path.header.frame_id = map_frame_id_;
  Eigen::Isometry3d submap_pose = Eigen::Isometry3d::Identity();
  for (int i = 0; i < num_submaps; i++) {
    /* path */
    const auto &submap = map_array_msg.submaps[i];
    if (!std::isfinite(submap.adjacency_transform.rotation.w)) {
      // reconstruct pose previously stored to unused adjacency_transform of a disconnected vertex
      auto pose = submap.adjacency_transform;
      auto &rot = pose.rotation;
      double w = sqrt(1 - rot.x*rot.x - rot.y*rot.y - rot.z*rot.z);
      rot.w = rot.w < 0 ? -w : w;
      submap_pose = tf2::transformToEigen(pose);
    }
    else {
      Eigen::Isometry3d trans;
      trans = tf2::transformToEigen(submap.adjacency_transform);
      submap_pose = submap_pose*trans;
    }

    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header = submap.header;
    pose_stamped.pose = tf2::toMsg(submap_pose);
    path.poses.push_back(pose_stamped);

    pose_stamped.pose = submap.pose;
    modified_path.poses.push_back(pose_stamped);
  }
  if (num_submaps > 0) {
    Eigen::Isometry3d modified_submap_pose;
    tf2::fromMsg(modified_path.poses.back().pose, modified_submap_pose);
    Eigen::Isometry3d map_loam_trans = modified_submap_pose*submap_pose.inverse();

    // publish zero transform between loam and map frame
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.frame_id = loam_frame_id_;
    transform_stamped.child_frame_id = map_frame_id_;
    transform_stamped.transform = tf2::eigenToTransform(map_loam_trans.inverse()).transform;
    broadcaster_->sendTransform(transform_stamped);
  }

  input_path_pub_->publish(path);
  modified_path_pub_->publish(modified_path);
}

void GraphBasedSlamComponent::doPoseAdjustment(
  lidarslam_msgs::msg::MapArray &map_array_msg,
  bool do_save_map, bool just_loaded)
{

  g2o::SparseOptimizer optimizer;
  optimizer.setVerbose(false);
  std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linear_solver =
    std::make_unique<g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>>();
  g2o::OptimizationAlgorithmLevenberg * solver = new g2o::OptimizationAlgorithmLevenberg(
    std::make_unique<g2o::BlockSolver_6_3>(std::move(linear_solver)));

  optimizer.setAlgorithm(solver);

  int submaps_size = map_array_msg.submaps.size();
  Eigen::Matrix<double, 6, 6> info_mat = Eigen::Matrix<double, 6, 6>::Identity();
  for (int i = 0; i < submaps_size; i++) {
    Eigen::Affine3d affine;
    Eigen::fromMsg(map_array_msg.submaps[i].pose, affine);
    Eigen::Isometry3d pose(affine.matrix());

    g2o::VertexSE3 * vertex_se3 = new g2o::VertexSE3();
    vertex_se3->setId(i);
    vertex_se3->setEstimate(pose);
    if (i == 0 || (loaded_map_static_ && i < loaded_submaps_cnt_)) {vertex_se3->setFixed(true);}
    optimizer.addVertex(vertex_se3);

    if (std::isfinite(map_array_msg.submaps[i].adjacency_transform.rotation.w)) {
      Eigen::Isometry3d relative_pose = Eigen::Isometry3d::Identity();
      for (int j = i - 1; j >= i - num_adjacent_pose_cnstraints_; j--) {
        Eigen::Isometry3d adjacency_trans;
        adjacency_trans = tf2::transformToEigen(map_array_msg.submaps[j + 1].adjacency_transform);
        relative_pose = adjacency_trans*relative_pose;
        g2o::EdgeSE3 *edge_se3 = new g2o::EdgeSE3();
        edge_se3->setMeasurement(relative_pose);
        edge_se3->setInformation(info_mat);
        edge_se3->vertices()[0] = optimizer.vertex(j);
        edge_se3->vertices()[1] = optimizer.vertex(i);
        optimizer.addEdge(edge_se3);
        // if (i >= loaded_submaps_cnt_)
        //   RCLCPP_INFO_STREAM(get_logger(), j << " -> " << i << ", trans:\n" << relative_pose.translation().transpose() << ", rot:\n" <<
        //                      Eigen::Quaterniond(relative_pose.rotation()) << "\n");
        if (!std::isfinite(map_array_msg.submaps[j].adjacency_transform.rotation.w))
          break;
      }
    }

  }
  /* loop edge */
  for (auto loop_edge : loop_edges_) {
    g2o::EdgeSE3 * edge_se3 = new g2o::EdgeSE3();
    edge_se3->setMeasurement(loop_edge.relative_pose);
    edge_se3->setInformation(info_mat);
    edge_se3->vertices()[0] = optimizer.vertex(loop_edge.pair_id.first);
    edge_se3->vertices()[1] = optimizer.vertex(loop_edge.pair_id.second);

    if (loop_edge.pair_id.first >= loaded_submaps_cnt_ || loop_edge.pair_id.second >= loaded_submaps_cnt_) {
      Eigen::Affine3d pose1, pose2;
      Eigen::fromMsg(map_array_msg.submaps[loop_edge.pair_id.first].pose, pose1);
      Eigen::fromMsg(map_array_msg.submaps[loop_edge.pair_id.second].pose, pose2);
      Eigen::Isometry3d correction = (Eigen::Isometry3d(pose1.matrix()).inverse()*Eigen::Isometry3d(pose2.matrix())).inverse()*loop_edge.relative_pose;
      RCLCPP_INFO_STREAM(get_logger(),
                         loop_edge.pair_id.first << " LLL> " << loop_edge.pair_id.second << ", trans:\n" <<
                                                 correction.translation().transpose() << ", rot:\n"
                                                 << Eigen::Quaterniond(correction.rotation()) << "\n");
    }
    optimizer.addEdge(edge_se3);
  }

  optimizer.initializeOptimization();
  optimizer.optimize(10);
  optimizer.save(pose_graph_path_.c_str());

  /* modified_map publish */
  std::cout << "modified_map publish" << std::endl;
  nav_msgs::msg::Path path;
  path.header.frame_id = map_frame_id_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  for (int i = 0; i < submaps_size; i++) {
    g2o::VertexSE3 * vertex_se3 = static_cast<g2o::VertexSE3 *>(optimizer.vertex(i));
    Eigen::Affine3d se3 = vertex_se3->estimate();
    geometry_msgs::msg::Pose pose = tf2::toMsg(se3);

    /* map */
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud_ptr(
      new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(map_array_msg.submaps[i].cloud, *cloud_ptr);

    pcl::transformPointCloud(*cloud_ptr, *transformed_cloud_ptr, se3.matrix().cast<float>());
    for (auto &p : transformed_cloud_ptr->points)
      p.intensity = i;
    *map_ptr += *transformed_cloud_ptr;

    /* submap */
    auto &submap = map_array_msg.submaps[i];
    submap.pose = pose;

    /* path */
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header = submap.header;
    pose_stamped.pose = submap.pose;
    path.poses.push_back(pose_stamped);

  }

  modified_map_array_pub_->publish(map_array_msg);
  if (do_save_map || just_loaded)
    saved_path_pub_->publish(path);

  sensor_msgs::msg::PointCloud2::SharedPtr map_msg_ptr(new sensor_msgs::msg::PointCloud2);
  pcl::toROSMsg(*map_ptr, *map_msg_ptr);
  map_msg_ptr->header.frame_id = map_frame_id_;
  modified_map_pub_->publish(*map_msg_ptr);
  if (do_save_map) {
    pcl::io::savePCDFileASCII("map.pcd", *map_ptr); // too heavy
    saveSubmaps(map_array_msg, loop_edges_);
  }
}

void GraphBasedSlamComponent::saveSubmaps(const lidarslam_msgs::msg::MapArray &submaps, const std::vector<LoopEdge> &loop_edges) const
{
  {  // Submaps
    RCLCPP_DEBUG(get_logger(), "serializing submaps");
    rclcpp::Serialization<lidarslam_msgs::msg::MapArray> serializer;
    std::shared_ptr<rclcpp::SerializedMessage> serialized_msg = std::make_shared<rclcpp::SerializedMessage>();
    serializer.serialize_message(&submaps, serialized_msg.get());
    rcl_serialized_message_t rcl_serialized_msg = serialized_msg->get_rcl_serialized_message();
    RCLCPP_DEBUG(get_logger(), "message serialized");

    std::string submaps_path = pose_graph_path_ + ".submaps";
    std::ofstream file(submaps_path, std::ios::binary);
    if (file.is_open()) {
      file.write(reinterpret_cast<const char *>(rcl_serialized_msg.buffer), rcl_serialized_msg.buffer_length);
      file.close();
      RCLCPP_INFO(get_logger(), "Submaps saved successfully to file %s", submaps_path.c_str());
    } else {
      RCLCPP_ERROR(get_logger(), "Unable to open submaps file %s for writing", submaps_path.c_str());
    }
  }

  {  // Loop edges
    std::string loop_edges_path = pose_graph_path_ + ".loop_edges";
    std::ofstream loop_edges_file(loop_edges_path, std::ios::binary);
    if (loop_edges_file.is_open()) {
      loop_edges_file.write(reinterpret_cast<const char *>(loop_edges.data()),
                            loop_edges.size()*sizeof(LoopEdge));
      loop_edges_file.close();
      RCLCPP_DEBUG(get_logger(), "Loop edges saved successfully to file %s", loop_edges_path.c_str());
    } else {
      RCLCPP_ERROR(get_logger(), "Unable to open loop edges file %s for writing", loop_edges_path.c_str());
    }
  }
}

bool GraphBasedSlamComponent::loadSubmaps(lidarslam_msgs::msg::MapArray &submaps, std::vector<LoopEdge> &loop_edges) const
{
  {  // Submaps
    std::string submaps_path = pose_graph_path_ + ".submaps";
    std::ifstream file(submaps_path, std::ios::binary);
    if (file.is_open()) {
      // get length of file:
      file.seekg(0, file.end);
      int length = file.tellg();
      file.seekg(0, file.beg);

      // Allocate memory for the buffer
      rcl_serialized_message_t rcl_msg = rmw_get_zero_initialized_serialized_message();
      rcl_allocator_t allocator = rcl_get_default_allocator();
      rmw_serialized_message_init(&rcl_msg, length, &allocator);
      //std::memcpy(rcl_msg.buffer, str.data(), length);

      // Set the size
      rcl_msg.buffer_length = length;
      //rcl_serialized_msg_2.buffer_length = file.end - file.beg;

      //rcl_msg.buffer = new uint8_t[rcl_msg.buffer_length];
      file.read(reinterpret_cast<char *>(rcl_msg.buffer), rcl_msg.buffer_length);
      file.close();

      RCLCPP_DEBUG(get_logger(), "Submaps file read successfully!");

      RCLCPP_DEBUG(get_logger(), "de-serializing message");
      const rclcpp::SerializedMessage serialized_message(rcl_msg);
      rclcpp::Serialization<lidarslam_msgs::msg::MapArray> serializer = rclcpp::Serialization<lidarslam_msgs::msg::MapArray>();
      serializer.deserialize_message(&serialized_message, &submaps);
      RCLCPP_DEBUG(get_logger(), "message de-serialized");

      RCLCPP_INFO(get_logger(), "Loaded submaps from file %s", submaps_path.c_str());
    } else {
      RCLCPP_ERROR(get_logger(), "Unable to open submaps file %s", submaps_path.c_str());
      return false;
    }
  }
  // make all submaps loop-closable to new trajectory (by setting their distance to at least -(distance_loop_closure_ + 1.0))
  double max_distance = -std::numeric_limits<double>::infinity();
  for (auto &submap : submaps.submaps)
    max_distance = std::max(max_distance, submap.distance);
  for (auto &submap : submaps.submaps)
    submap.distance -= max_distance + distance_loop_closure_ + 1.0;

  {  // Loop edges
    std::string loop_edges_path = pose_graph_path_ + ".loop_edges";
    std::ifstream file(loop_edges_path, std::ios::binary);
    if (file.is_open()) {
      // get length of file:
      file.seekg(0, file.end);
      int length = file.tellg();
      file.seekg(0, file.beg);

      // Allocate memory for the buffer
      loop_edges.resize(length/sizeof(LoopEdge));
      file.read(reinterpret_cast<char *>(loop_edges.data()), length);
      file.close();

      RCLCPP_INFO(get_logger(), "Loaded loop edges from file %s", loop_edges_path.c_str());
    } else {
      RCLCPP_ERROR(get_logger(), "Unable to open loop edges file %s", loop_edges_path.c_str());
      return false;
    }
  }
  return true;
}
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(graphslam::GraphBasedSlamComponent)
