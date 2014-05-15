#include "laser_icp_alg_node.h"

LaserIcpAlgNode::LaserIcpAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<LaserIcpAlgorithm>()
{
  ROS_INFO("Laser ICP: Warming up..");
  //init class attributes if necessary
  //this->loop_rate_ = 2;//in [Hz]

  // [init publishers]

  // [init subscribers]

  // [init services]
  this->get_relative_pose_server_ = this->public_node_handle_.advertiseService("get_relative_pose", &LaserIcpAlgNode::get_relative_poseCallback, this);
  pthread_mutex_init(&this->get_relative_pose_mutex_,NULL);

  // [init clients]

  // [init action servers]

  // [init action clients]

  // **** init parameters

  initParams();

  // **** state variables

  input_.laser[0] = 0.0;
  input_.laser[1] = 0.0;
  input_.laser[2] = 0.0;

}

LaserIcpAlgNode::~LaserIcpAlgNode(void)
{
  // [free dynamic memory]
  pthread_mutex_destroy(&this->get_relative_pose_mutex_);
}

void LaserIcpAlgNode::mainNodeThread(void)
{
  // [fill msg structures]

  // [fill srv structure and make request to the server]

  // [fill action structure and make request to the action server]

  // [publish messages]
}

/*  [subscriber callbacks] */

/*  [service callbacks] */
bool LaserIcpAlgNode::get_relative_poseCallback(
                    iri_laser_icp::GetRelativePose::Request &req,
                    iri_laser_icp::GetRelativePose::Response &res)
{
  ROS_DEBUG("Laser ICP: New Request");

  //use appropiate mutex to shared variables if necessary
  //this->alg_.lock();
  //this->get_relative_pose_mutex_.enter();

//   if(this->alg_.isRunning())
//   {
    //ROS_INFO("LaserIcpAlgNode::get_relative_poseCallback: Processin New Request!");

    //do operations with req and output on res
    input_.min_reading = std::max(float(min_laser_range_), std::min(req.scan_ref.range_min,req.scan_sens.range_min));
    input_.max_reading = std::min(float(max_laser_range_), std::max(req.scan_ref.range_max,req.scan_sens.range_max));
    laserScanToLDP(req.scan_ref, ref_ldp_scan_);
    laserScanToLDP(req.scan_sens, sens_ldp_scan_);
    
    // first guess given
    input_.first_guess[0] = req.prior_d.position.x;
    input_.first_guess[1] = req.prior_d.position.y;
    if (req.prior_d.orientation.x != 0 || req.prior_d.orientation.y != 0 || req.prior_d.orientation.z != 0 || req.prior_d.orientation.w != 0)
      input_.first_guess[2] = tf::getYaw(req.prior_d.orientation);
    else
      input_.first_guess[2] = 0;
      
    processScans();

    // output_

    //       struct sm_result {
    //   /** 1 if the result is valid */
    //   int valid;
      
    //   /** Scan matching result (x,y,theta) */
    //   double x[3];
      
    //   /** Number of iterations done */
    //   int iterations;
    //   * Number of valid correspondence in the end 
    //   int nvalid;
    //   /** Total correspondence error */
    //   double error;
      
    //   /** Fields used for covariance computation */
    //   #ifndef RUBY
    //     gsl_matrix *cov_x_m;  
    //     gsl_matrix *dx_dy1_m;
    //     gsl_matrix *dx_dy2_m;
    //   #endif
    // };

//     #include <gsl/gsl_matrix.h>
 
// int print_matrix(FILE *f, const gsl_matrix *m)
// {
//         int status, n = 0;
 
//         for (size_t i = 0; i < m->size1; i++) {
//                 for (size_t j = 0; j < m->size2; j++) {
//                         if ((status = fprintf(f, "%g ", gsl_matrix_get(m, i, j))) < 0)
//                                 return -1;
//                         n += status;
//                 }
 
//                 if ((status = fprintf(f, "\n")) < 0)
//                         return -1;
//                 n += status;
//         }
 
//         return n;
// }


    if (output_.valid)
    {
      res.pose_rel.pose.pose.position.x = output_.x[0];
      res.pose_rel.pose.pose.position.y = output_.x[1];
      res.pose_rel.pose.pose.orientation = tf::createQuaternionMsgFromYaw(output_.x[2]);

      res.pose_rel.pose.covariance[0]  = gsl_matrix_get(output_.cov_x_m, 0, 0);
      res.pose_rel.pose.covariance[1]  = gsl_matrix_get(output_.cov_x_m, 0, 1);
      res.pose_rel.pose.covariance[5]  = gsl_matrix_get(output_.cov_x_m, 0, 2);
      res.pose_rel.pose.covariance[6]  = gsl_matrix_get(output_.cov_x_m, 1, 0);
      res.pose_rel.pose.covariance[7]  = gsl_matrix_get(output_.cov_x_m, 1, 1);
      res.pose_rel.pose.covariance[11] = gsl_matrix_get(output_.cov_x_m, 1, 2);
      res.pose_rel.pose.covariance[30] = gsl_matrix_get(output_.cov_x_m, 2, 0);
      res.pose_rel.pose.covariance[31] = gsl_matrix_get(output_.cov_x_m, 2, 1);
      res.pose_rel.pose.covariance[35] = gsl_matrix_get(output_.cov_x_m, 2, 2);

      res.success = true;
    }
    else
    {
      res.pose_rel.pose.pose.position.x = 0;
      res.pose_rel.pose.pose.position.y = 0;
      res.pose_rel.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);

      res.pose_rel.pose.covariance[0]  = 0;
      res.pose_rel.pose.covariance[1]  = 0;
      res.pose_rel.pose.covariance[5]  = 0;
      res.pose_rel.pose.covariance[6]  = 0;
      res.pose_rel.pose.covariance[7]  = 0;
      res.pose_rel.pose.covariance[11] = 0;
      res.pose_rel.pose.covariance[30] = 0;
      res.pose_rel.pose.covariance[31] = 0;
      res.pose_rel.pose.covariance[35] = 0;

      res.success = false;
    }

//   }
//   else
//   {
//     ROS_INFO("LaserIcpAlgNode::get_relative_poseCallback: ERROR: alg is not on run mode yet.");
//   }

  //unlock previously blocked shared variables
  //this->alg_.unlock();
  //this->get_relative_pose_mutex_.exit();

  return true;
}

void LaserIcpAlgNode::get_relative_pose_mutex_enter(void) 
{ 
  pthread_mutex_lock(&this->get_relative_pose_mutex_); 
} 

void LaserIcpAlgNode::get_relative_pose_mutex_exit(void) 
{ 
  pthread_mutex_unlock(&this->get_relative_pose_mutex_); 
}

/*  [action callbacks] */

/*  [action requests] */

void LaserIcpAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();

  this->alg_.unlock();
}

void LaserIcpAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<LaserIcpAlgNode>(argc, argv, "laser_icp_alg_node");
}


void LaserIcpAlgNode::initParams()
{

  if (!private_node_handle_.getParam ("min_laser_range", min_laser_range_))
    min_laser_range_ = 0.0;

  if (!private_node_handle_.getParam ("max_laser_range", max_laser_range_))
    max_laser_range_ = 1.0e10;
  
  if (!private_node_handle_.getParam ("use_alpha_beta", use_alpha_beta_))
    use_alpha_beta_ = false;

  if (!private_node_handle_.getParam ("alpha", alpha_))
    alpha_ = 1.0;
  
  if (!private_node_handle_.getParam ("beta", beta_))
    beta_ = 0.8;

 // **** CSM parameters - comments copied from algos.h (by Andrea Censi)

  // Maximum angular displacement between scans
  if (!private_node_handle_.getParam ("max_angular_correction_deg", input_.max_angular_correction_deg))
    input_.max_angular_correction_deg = 45.0;

  // Maximum translation between scans (m)
  if (!private_node_handle_.getParam ("max_linear_correction", input_.max_linear_correction))
    input_.max_linear_correction = 0.50;

  // Maximum ICP cycle iterations
  if (!private_node_handle_.getParam ("max_iterations", input_.max_iterations))
    input_.max_iterations = 10;

  // A threshold for stopping (m)
  if (!private_node_handle_.getParam ("epsilon_xy", input_.epsilon_xy))
    input_.epsilon_xy = 0.000001;

  // A threshold for stopping (rad)
  if (!private_node_handle_.getParam ("epsilon_theta", input_.epsilon_theta))
    input_.epsilon_theta = 0.000001;

  // Maximum distance for a correspondence to be valid
  if (!private_node_handle_.getParam ("max_correspondence_dist", input_.max_correspondence_dist))
    input_.max_correspondence_dist = 0.3;

  // Noise in the scan (m)
  if (!private_node_handle_.getParam ("sigma", input_.sigma))
    input_.sigma = 0.010;

  // Use smart tricks for finding correspondences.cov
  if (!private_node_handle_.getParam ("use_corr_tricks", input_.use_corr_tricks))
    input_.use_corr_tricks = 1;

  // Restart: Restart if error is over threshold
  if (!private_node_handle_.getParam ("restart", input_.restart))
    input_.restart = 0;

  // Restart: Threshold for restarting
  if (!private_node_handle_.getParam ("restart_threshold_mean_error", input_.restart_threshold_mean_error))
    input_.restart_threshold_mean_error = 0.01;

  // Restart: displacement for restarting. (m)
  if (!private_node_handle_.getParam ("restart_dt", input_.restart_dt))
    input_.restart_dt = 1.0;

  // Restart: displacement for restarting. (rad)
  if (!private_node_handle_.getParam ("restart_dtheta", input_.restart_dtheta))
    input_.restart_dtheta = 0.1;

  // Max distance for staying in the same clustering
  if (!private_node_handle_.getParam ("clustering_threshold", input_.clustering_threshold))
    input_.clustering_threshold = 0.25;

  // Number of neighbour rays used to estimate the orientation
  if (!private_node_handle_.getParam ("orientation_neighbourhood", input_.orientation_neighbourhood))
    input_.orientation_neighbourhood = 20;

  // If 0, it's vanilla ICP
  if (!private_node_handle_.getParam ("use_point_to_line_distance", input_.use_point_to_line_distance))
    input_.use_point_to_line_distance = 1;

  // Discard correspondences based on the angles
  if (!private_node_handle_.getParam ("do_alpha_test", input_.do_alpha_test))
    input_.do_alpha_test = 0;

  // Discard correspondences based on the angles - threshold angle, in degrees
  if (!private_node_handle_.getParam ("do_alpha_test_thresholdDeg", input_.do_alpha_test_thresholdDeg))
    input_.do_alpha_test_thresholdDeg = 20.0;

  // Percentage of correspondences to consider: if 0.9,
  // always discard the top 10% of correspondences with more error
  if (!private_node_handle_.getParam ("outliers_maxPerc", input_.outliers_maxPerc))
    input_.outliers_maxPerc = 0.90;

  // Parameters describing a simple adaptive algorithm for discarding.
  //  1) Order the errors.
  //  2) Choose the percentile according to outliers_adaptive_order.
  //     (if it is 0.7, get the 70% percentile)
  //  3) Define an adaptive threshold multiplying outliers_adaptive_mult
  //     with the value of the error at the chosen percentile.
  //  4) Discard correspondences over the threshold.
  //  This is useful to be conservative; yet remove the biggest errors.
  if (!private_node_handle_.getParam ("outliers_adaptive_order", input_.outliers_adaptive_order))
    input_.outliers_adaptive_order = 0.7;

  if (!private_node_handle_.getParam ("outliers_adaptive_mult", input_.outliers_adaptive_mult))
    input_.outliers_adaptive_mult = 2.0;

  //If you already have a guess of the solution, you can compute the polar angle
  //  of the points of one scan in the new position. If the polar angle is not a monotone
  //  function of the readings index, it means that the surface is not visible in the
  //  next position. If it is not visible, then we don't use it for matching.
  if (!private_node_handle_.getParam ("do_visibility_test", input_.do_visibility_test))
    input_.do_visibility_test = 0;

  // no two points in laser_sens can have the same corr.
  if (!private_node_handle_.getParam ("outliers_remove_doubles", input_.outliers_remove_doubles))
    input_.outliers_remove_doubles = 1;

  // If 1, computes the covariance of ICP using the method http://purl.org/censi/2006/icpcov
  if (!private_node_handle_.getParam ("do_compute_covariance", input_.do_compute_covariance))
    input_.do_compute_covariance = 0;

  // Checks that find_correspondences_tricks gives the right answer
  if (!private_node_handle_.getParam ("debug_verify_tricks", input_.debug_verify_tricks))
    input_.debug_verify_tricks = 0;

  // If 1, the field 'true_alpha' (or 'alpha') in the first scan is used to compute the
  // incidence beta, and the factor (1/cos^2(beta)) used to weight the correspondence.");
  if (!private_node_handle_.getParam ("use_ml_weights", input_.use_ml_weights))
    input_.use_ml_weights = 0;

  // If 1, the field 'readings_sigma' in the second scan is used to weight the
  // correspondence by 1/sigma^2
  if (!private_node_handle_.getParam ("use_sigma_weights", input_.use_sigma_weights))
    input_.use_sigma_weights = 0;
}


void LaserIcpAlgNode::processScans()
{
  struct timeval start, end;    // used for timing
  gettimeofday(&start, NULL);

  //ROS_INFO("ProcessScans");

  ref_ldp_scan_->odometry[0] = 0;
  ref_ldp_scan_->odometry[1] = 0;
  ref_ldp_scan_->odometry[2] = 0;

  ref_ldp_scan_->estimate[0] = 0;
  ref_ldp_scan_->estimate[1] = 0;
  ref_ldp_scan_->estimate[2] = 0;

  ref_ldp_scan_->true_pose[0] = 0;
  ref_ldp_scan_->true_pose[1] = 0;
  ref_ldp_scan_->true_pose[2] = 0;

  input_.laser_ref  = ref_ldp_scan_;
  input_.laser_sens = sens_ldp_scan_;

  // *** scan match - using point to line icp from CSM

  sm_icp(&input_, &output_);

  ROS_DEBUG("Laser ICP:ProcessScans: output valid? %d", output_.valid);

  if (output_.valid)
  {
    // the correction of the laser's position, in the laser frame

    ROS_INFO_XYT("", output_.x[0], output_.x[1], output_.x[2]);
    //ROS_DEBUG("Laser ICP: it: %d, valid: %d, error: %f", output_.iterations, output_.nvalid, output_.error);
    //ROS_INFO("Laser ICP: Covariance matrix X (%lu,%lu)",output_.cov_x_m->size1,output_.cov_x_m->size2);
    //gsl_matrix_fprintf(stdout,output_.cov_x_m,"%f");
    //ROS_INFO("Laser ICP: Covariance matrix DX DY1 (%lu,%lu)",output_.dx_dy1_m->size1,output_.dx_dy1_m->size2);
    //gsl_matrix_fprintf(stdout,output_.dx_dy1_m,"%f");
    //ROS_INFO("Laser ICP: Covariance matrix DX DY2 (%lu,%lu)",output_.dx_dy2_m->size1,output_.dx_dy2_m->size2);
    //gsl_matrix_fprintf(stdout,output_.dx_dy2_m,"%f");
    //ROS_INFO("Laser ICP END");

  }
  else
  {
    ROS_ERROR("Laser ICP: Output invalid. Error in scan matching");
  }

  // **** swap old and new

  ld_free(ref_ldp_scan_);
  ld_free(sens_ldp_scan_);

  // **** statistics

  gettimeofday(&end, NULL);
  double dur_total = ((end.tv_sec   * 1000000 + end.tv_usec  ) -
                      (start.tv_sec * 1000000 + start.tv_usec)) / 1000.0;
  ROS_DEBUG("Laser ICP: matching duration: %.1f ms", dur_total);
}

void LaserIcpAlgNode::laserScanToLDP(const sensor_msgs::LaserScan& scan_msg,
                                            LDP& ldp)
{
  unsigned int n = scan_msg.ranges.size();
  ldp = ld_alloc_new(n);

  for (unsigned int i = 0; i < n; i++)
  {
    // calculate position in laser frame

    double r = scan_msg.ranges[i];

    if (r > scan_msg.range_min && r < scan_msg.range_max)
    {
      // fill in laser scan data

      ldp->valid[i] = 1;
      ldp->readings[i] = r;
    }
    else
    {
      ldp->valid[i] = 0;
      ldp->readings[i] = -1;  // for invalid range
      //ROS_INFO("ICP: scan %i is invalid (range)",i);
    }

    ldp->theta[i]    = scan_msg.angle_min + i * scan_msg.angle_increment;

    ldp->cluster[i]  = -1;
  }

  ldp->min_theta = ldp->theta[0];
  ldp->max_theta = ldp->theta[n-1];

  ldp->odometry[0] = 0.0;
  ldp->odometry[1] = 0.0;
  ldp->odometry[2] = 0.0;

  ldp->true_pose[0] = 0.0;
  ldp->true_pose[1] = 0.0;
  ldp->true_pose[2] = 0.0;
}

void LaserIcpAlgNode::ROS_INFO_XYT(const std::string & str,const float & x,const float & y,const float & th)
{
  ROS_DEBUG("Laser ICP: %s",str.c_str());
  ROS_DEBUG("\033[31mx:\033[0m%f \033[32my:\033[0m%f \033[34mth:\033[0m%f ",
           x,y,th);
}

