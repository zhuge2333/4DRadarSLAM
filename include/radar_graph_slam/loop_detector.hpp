// SPDX-License-Identifier: BSD-2-Clause

#ifndef LOOP_DETECTOR_HPP
#define LOOP_DETECTOR_HPP

#include <math.h>

#include <boost/format.hpp>

#include <radar_graph_slam/keyframe.hpp>
#include <radar_graph_slam/registrations.hpp>
#include <radar_graph_slam/graph_slam.hpp>
#include <radar_graph_slam/information_matrix_calculator.hpp>
#include <scan_context/Scancontext.h>

#include <g2o/types/slam3d/vertex_se3.h>

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std;

namespace radar_graph_slam {

struct Loop {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = std::shared_ptr<Loop>;

  Loop(const KeyFrame::Ptr& key1, const KeyFrame::Ptr& key2, const Eigen::Matrix4f& relpose) : key1(key1), key2(key2), relative_pose(relpose) {}

public:
  KeyFrame::Ptr key1;
  KeyFrame::Ptr key2;
  Eigen::Matrix4f relative_pose;
};

/**
 * @brief this class finds loops by scam matching and adds them to the pose graph
 */
class LoopDetector {
public:
  typedef pcl::PointXYZI PointT;

  /**
   * @brief constructor
   * @param pnh
   */
  LoopDetector() {}
  LoopDetector(ros::NodeHandle& pnh);
  ~LoopDetector();

  std::vector<Loop::Ptr> detect(const std::vector<KeyFrame::Ptr>& keyframes, const std::deque<KeyFrame::Ptr>& new_keyframes, radar_graph_slam::GraphSLAM& graph_slam);

  double get_distance_thresh() const;

  Loop::Ptr performScanContextLoopClosure(const std::vector<KeyFrame::Ptr>& keyframes, const std::vector<KeyFrame::Ptr>& candidate_keyframes, const KeyFrame::Ptr& new_keyframe);

private:
  std::vector<KeyFrame::Ptr> find_candidates(const std::vector<KeyFrame::Ptr>& keyframes, const KeyFrame::Ptr& new_keyframe) const;
  cv::Mat getColorImage(cv::Mat &desc);
  typedef std::tuple<u_char, u_char, u_char> Color;
  std::map<uint32_t, Color> _argmax_to_rgb;
  cv::Mat makeSCImage(Eigen::MatrixXd src_mat);

private:
  double distance_thresh;                 // estimated distance between keyframes consisting a loop must be less than this distance
  double accum_distance_thresh;           // Minimum distance beteen two edges of the loop
  double min_loop_interval_dist;  // Minimum distance between a new loop edge and the last one

  double odom_drift_xy;     // Odometry drift along X and Y axis
  double odom_drift_z;      // Odometry drift along Z axis
  double drift_scale_xy;       // Odometry drift scale
  double drift_scale_z;

  double max_baro_difference;
  double max_yaw_difference;
  
  double fitness_score_max_range;  // maximum allowable distance between corresponding points
  double fitness_score_thresh;     // threshold for scan matching

  double last_loop_edge_accum_distance = 0.0;
  size_t last_loop_edge_index = 0;

  pcl::Registration<PointT, PointT>::Ptr registration;

public:
  // Loop closure
  std::unique_ptr<SCManager> scManager;  // loop detector
  std::unique_ptr<InformationMatrixCalculator> inf_calclator;
  int historyKeyframeSearchNum;
  float historyKeyframeFitnessScore;
  std::multimap<int, int> loopIndexContainer; // from new to old // giseop 
  std::vector<pair<int, int>> loopIndexQueue;
  std::vector<Eigen::Isometry3d> loopPoseQueue;
  std::vector<Eigen::MatrixXd> loopInfoQueue;
  std::mutex mtx;
  double odom_check_trans_thresh;
  double odom_check_rot_thresh;
  double pairwise_check_trans_thresh;
  double pairwise_check_rot_thresh;

  bool enable_pf;
  bool enable_odom_check;

  std::vector<double> pf_time;
  std::vector<double> sc_time;
  std::vector<double> oc_time;
  clock_t start_ms_pf;
  clock_t end_ms_pf;
  clock_t start_ms_sc;
  clock_t end_ms_sc;
  clock_t start_ms_oc;
  clock_t end_ms_oc;
  std::unique_ptr<image_transport::ImageTransport> image_transport;
  image_transport::Publisher pub_cur_sc;
  image_transport::Publisher pub_pre_sc;
};

}  // namespace radar_graph_slam

#endif  // LOOP_DETECTOR_HPP
