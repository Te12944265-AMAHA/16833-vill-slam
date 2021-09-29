#include <vector>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <ros/package.h>
#include <mutex>
#include <queue>
#include <thread>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include "keyframe.h"
#include "utility/tic_toc.h"
#include "pose_graph.h"
#include "utility/CameraPoseVisualization.h"
#include "map.h"
#include "parameters.h"
#include <tf/transform_broadcaster.h>

#define SKIP_FIRST_CNT 10
using namespace std;

queue<sensor_msgs::ImageConstPtr> image_buf;
queue<sensor_msgs::PointCloudConstPtr> point_buf;
queue<nav_msgs::Odometry::ConstPtr> pose_buf;
queue<Eigen::Vector3d> odometry_buf;
queue<sensor_msgs::PointCloudConstPtr> margin_point_buf;
std::mutex m_buf;
std::mutex m_process;
int frame_index = 0;
int sequence = 1;
PoseGraph posegraph;
int skip_first_cnt = 0;
int SKIP_CNT;
int skip_cnt = 0;
bool load_flag = 0;
bool start_flag = 0;
double SKIP_DIS = 0;

int VISUALIZATION_SHIFT_X;
int VISUALIZATION_SHIFT_Y;
int ROW;
int COL;
int DEBUG_IMAGE;
//int DEBUG_LOOP_CLOSURE;
int VISUALIZE_IMU_FORWARD;
int LOOP_CLOSURE;
int FAST_RELOCALIZATION;

camodocal::CameraPtr m_camera;
Eigen::Vector3d tic;
Eigen::Matrix3d qic;
ros::Publisher pub_match_img;
ros::Publisher pub_match_points;
ros::Publisher pub_camera_pose_visual;
ros::Publisher pub_key_odometrys;
ros::Publisher pub_vio_path;
nav_msgs::Path no_loop_path;

std::string BRIEF_PATTERN_FILE;
std::string POSE_GRAPH_SAVE_PATH;
std::string VINS_RESULT_PATH;
CameraPoseVisualization cameraposevisual(1, 0, 0, 1);
Eigen::Vector3d last_t(-100, -100, -100);
double last_image_time = -1;

LandmarkMap lm_map;

int g_kf_idx_offset = -1;

void pubTF(const KeyFrame *mid_kf, const Eigen::Matrix3d &R_w_i,
           const Eigen::Vector3d &t_w_i)
{
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;

  Eigen::Matrix3d R_w_c = R_w_i * qic;
  Eigen::Vector3d t_w_c = R_w_i * tic + t_w_i;

  transform.setOrigin(tf::Vector3(t_w_c.x(), t_w_c.y(), t_w_c.z()));

  Quaterniond q_w_c(R_w_c);
  q.setW(q_w_c.w());
  q.setX(q_w_c.x());
  q.setY(q_w_c.y());
  q.setZ(q_w_c.z());
  transform.setRotation(q);

  br.sendTransform(
      tf::StampedTransform(transform, ros::Time(mid_kf->time_stamp), "world",
                           "camera_pg_undrift"));
}

void new_sequence()
{
  printf("new sequence\n");
  sequence++;
  printf("sequence cnt %d \n", sequence);
  if (sequence > 5)
  {
    ROS_WARN(
        "only support 5 sequences since it's boring to copy code for more sequences.");
    ROS_BREAK();
  }
  posegraph.posegraph_visualization->reset();
  posegraph.publish();
  m_buf.lock();
  while (!image_buf.empty())
    image_buf.pop();
  while (!point_buf.empty())
    point_buf.pop();
  while (!pose_buf.empty())
    pose_buf.pop();
  while (!odometry_buf.empty())
    odometry_buf.pop();
  m_buf.unlock();
}

void image_callback(const sensor_msgs::ImageConstPtr &image_msg)
{
  //ROS_INFO("image_callback!");
  if (!LOOP_CLOSURE)
    return;
  m_buf.lock();
  image_buf.push(image_msg);
  m_buf.unlock();
  //printf(" image time %f \n", image_msg->header.stamp.toSec());

  // detect unstable camera stream
  if (last_image_time == -1)
    last_image_time = image_msg->header.stamp.toSec();
  else if (image_msg->header.stamp.toSec() - last_image_time > 1.0 ||
           image_msg->header.stamp.toSec() < last_image_time)
  {
    ROS_WARN("image discontinue! detect a new sequence!");
    new_sequence();
  }
  last_image_time = image_msg->header.stamp.toSec();
}

void point_callback(const sensor_msgs::PointCloudConstPtr &point_msg)
{
  //ROS_INFO("point_callback!");
  if (!LOOP_CLOSURE)
    return;
  m_buf.lock();
  point_buf.push(point_msg);
  m_buf.unlock();
  /*
  for (unsigned int i = 0; i < point_msg->points.size(); i++)
  {
      printf("%d, 3D point: %f, %f, %f 2D point %f, %f \n",i , point_msg->points[i].x,
                                                   point_msg->points[i].y,
                                                   point_msg->points[i].z,
                                                   point_msg->channels[i].values[0],
                                                   point_msg->channels[i].values[1]);
  }
  */
}

void
margin_point_callback(const sensor_msgs::PointCloudConstPtr &margin_point_msg)
{
  if (!LOOP_CLOSURE)
    return;
  if (g_kf_idx_offset == -1)
    return;
  m_buf.lock();
  margin_point_buf.push(margin_point_msg);
  m_buf.unlock();
}

void pose_callback(const nav_msgs::Odometry::ConstPtr &pose_msg)
{
  //ROS_INFO("pose_callback!");
  if (!LOOP_CLOSURE)
    return;
  m_buf.lock();
  pose_buf.push(pose_msg);
  m_buf.unlock();
  /*
  printf("pose t: %f, %f, %f   q: %f, %f, %f %f \n", pose_msg->pose.pose.position.x,
                                                     pose_msg->pose.pose.position.y,
                                                     pose_msg->pose.pose.position.z,
                                                     pose_msg->pose.pose.orientation.w,
                                                     pose_msg->pose.pose.orientation.x,
                                                     pose_msg->pose.pose.orientation.y,
                                                     pose_msg->pose.pose.orientation.z);
  */
}

void imu_forward_callback(const nav_msgs::Odometry::ConstPtr &forward_msg)
{
  // imu message is only used for forward visualization
  if (VISUALIZE_IMU_FORWARD)
  {
    Vector3d vio_t(forward_msg->pose.pose.position.x,
                   forward_msg->pose.pose.position.y,
                   forward_msg->pose.pose.position.z);
    Quaterniond vio_q;
    vio_q.w() = forward_msg->pose.pose.orientation.w;
    vio_q.x() = forward_msg->pose.pose.orientation.x;
    vio_q.y() = forward_msg->pose.pose.orientation.y;
    vio_q.z() = forward_msg->pose.pose.orientation.z;

    vio_t = posegraph.w_r_vio * vio_t + posegraph.w_t_vio;
    vio_q = posegraph.w_r_vio * vio_q;

    vio_t = posegraph.r_drift * vio_t + posegraph.t_drift;
    vio_q = posegraph.r_drift * vio_q;

    Vector3d vio_t_cam;
    Quaterniond vio_q_cam;
    vio_t_cam = vio_t + vio_q * tic;
    vio_q_cam = vio_q * qic;

    cameraposevisual.reset();
    cameraposevisual.add_pose(vio_t_cam, vio_q_cam);
    cameraposevisual.publish_by(pub_camera_pose_visual, forward_msg->header);
  }
}

void relo_relative_pose_callback(const nav_msgs::Odometry::ConstPtr &pose_msg)
{
  Vector3d relative_t = Vector3d(pose_msg->pose.pose.position.x,
                                 pose_msg->pose.pose.position.y,
                                 pose_msg->pose.pose.position.z);
  Quaterniond relative_q;
  relative_q.w() = pose_msg->pose.pose.orientation.w;
  relative_q.x() = pose_msg->pose.pose.orientation.x;
  relative_q.y() = pose_msg->pose.pose.orientation.y;
  relative_q.z() = pose_msg->pose.pose.orientation.z;
  double relative_yaw = pose_msg->twist.twist.linear.x;
  int index = pose_msg->twist.twist.linear.y;
  //printf("receive index %d \n", index );
  Eigen::Matrix<double, 8, 1> loop_info;
  loop_info << relative_t.x(), relative_t.y(), relative_t.z(),
      relative_q.w(), relative_q.x(), relative_q.y(), relative_q.z(),
      relative_yaw;
  posegraph.updateKeyFrameLoop(index, loop_info);

}

void vio_callback(const nav_msgs::Odometry::ConstPtr &pose_msg)
{
  //ROS_INFO("vio_callback!");
  // takes in the latest odometry data.
  Vector3d vio_t(pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y,
                 pose_msg->pose.pose.position.z);
  Quaterniond vio_q;
  vio_q.w() = pose_msg->pose.pose.orientation.w;
  vio_q.x() = pose_msg->pose.pose.orientation.x;
  vio_q.y() = pose_msg->pose.pose.orientation.y;
  vio_q.z() = pose_msg->pose.pose.orientation.z;

  // compensate the transformation between the world frame and the current sequence frame
  vio_t = posegraph.w_r_vio * vio_t + posegraph.w_t_vio;
  vio_q = posegraph.w_r_vio * vio_q;

  // compensate for the drift (difference between current pose of VIO and pose graph)
  //cout << "***** Debug drift info *****" << endl
  //     << "yaw drift: " << posegraph.yaw_drift << endl
  //     << "t drift: " << posegraph.t_drift.transpose() << endl;
  vio_t = posegraph.r_drift * vio_t + posegraph.t_drift;
  vio_q = posegraph.r_drift * vio_q;

  Vector3d vio_t_cam;
  Quaterniond vio_q_cam;
  vio_t_cam = vio_t + vio_q * tic;
  vio_q_cam = vio_q * qic;

  if (!VISUALIZE_IMU_FORWARD)
  {
    cameraposevisual.reset();
    cameraposevisual.add_pose(vio_t_cam, vio_q_cam);
    cameraposevisual.publish_by(pub_camera_pose_visual, pose_msg->header);
  }

  odometry_buf.push(vio_t_cam);
  if (odometry_buf.size() > 10)
  {
    odometry_buf.pop();
  }

  // looks like key_odometrys are just 10 latest odometry outputs
  visualization_msgs::Marker key_odometrys;
  key_odometrys.header = pose_msg->header;
  key_odometrys.header.frame_id = "world";
  key_odometrys.ns = "key_odometrys";
  key_odometrys.type = visualization_msgs::Marker::SPHERE_LIST;
  key_odometrys.action = visualization_msgs::Marker::ADD;
  key_odometrys.pose.orientation.w = 1.0;
  key_odometrys.lifetime = ros::Duration();

  //static int key_odometrys_id = 0;
  key_odometrys.id = 0; //key_odometrys_id++;
  key_odometrys.scale.x = 0.1;
  key_odometrys.scale.y = 0.1;
  key_odometrys.scale.z = 0.1;
  key_odometrys.color.r = 1.0;
  key_odometrys.color.a = 1.0;

  for (unsigned int i = 0; i < odometry_buf.size(); i++)
  {
    geometry_msgs::Point pose_marker;
    Vector3d vio_t;
    vio_t = odometry_buf.front();
    odometry_buf.pop();
    pose_marker.x = vio_t.x();
    pose_marker.y = vio_t.y();
    pose_marker.z = vio_t.z();
    key_odometrys.points.push_back(pose_marker);
    odometry_buf.push(vio_t);
  }
  pub_key_odometrys.publish(key_odometrys);

  if (!LOOP_CLOSURE)
  {
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = pose_msg->header;
    pose_stamped.header.frame_id = "world";
    pose_stamped.pose.position.x = vio_t.x();
    pose_stamped.pose.position.y = vio_t.y();
    pose_stamped.pose.position.z = vio_t.z();
    no_loop_path.header = pose_msg->header;
    no_loop_path.header.frame_id = "world";
    no_loop_path.poses.push_back(pose_stamped);
    pub_vio_path.publish(no_loop_path);
  }
}

void extrinsic_callback(const nav_msgs::Odometry::ConstPtr &pose_msg)
{
  m_process.lock();
  tic = Vector3d(pose_msg->pose.pose.position.x,
                 pose_msg->pose.pose.position.y,
                 pose_msg->pose.pose.position.z);
  qic = Quaterniond(pose_msg->pose.pose.orientation.w,
                    pose_msg->pose.pose.orientation.x,
                    pose_msg->pose.pose.orientation.y,
                    pose_msg->pose.pose.orientation.z).toRotationMatrix();
  m_process.unlock();
}

void process()
{
  if (!LOOP_CLOSURE)
    return;
  while (ros::ok)
  {
    //cout << "flag 1" << endl;
    sensor_msgs::ImageConstPtr image_msg = nullptr;
    sensor_msgs::PointCloudConstPtr point_msg = nullptr;
    sensor_msgs::PointCloudConstPtr margin_point_msg = nullptr;
    nav_msgs::Odometry::ConstPtr pose_msg = nullptr;


    // find out the messages with same time stamp
    m_buf.lock();
    if (!image_buf.empty() && !point_buf.empty() && !pose_buf.empty())
    {
      if (false)
      {
        std::cout << "image buf front: "
                  << image_buf.front()->header.stamp.toSec() << std::endl
                  << "pose  buf front: "
                  << pose_buf.front()->header.stamp.toSec() << std::endl
                  << "point buf front: "
                  << point_buf.front()->header.stamp.toSec() << std::endl
                  << "image buf front: "
                  << image_buf.front()->header.stamp.toSec() << std::endl
                  << "image buf back : "
                  << image_buf.back()->header.stamp.toSec() << std::endl
                  << "point buf back : "
                  << point_buf.back()->header.stamp.toSec() << std::endl;
      }
      if (image_buf.front()->header.stamp.toSec() >
          pose_buf.front()->header.stamp.toSec())
      {
        pose_buf.pop();
        printf("throw pose at beginning\n");
      } else if (image_buf.front()->header.stamp.toSec() >
                 point_buf.front()->header.stamp.toSec())
      {
        point_buf.pop();
        printf("throw point at beginning\n");
      } else if (image_buf.back()->header.stamp.toSec() >=
                 pose_buf.front()->header.stamp.toSec()
                 && point_buf.back()->header.stamp.toSec() >=
                    pose_buf.front()->header.stamp.toSec())
      {
        pose_msg = pose_buf.front();
        pose_buf.pop();
        //while (!pose_buf.empty())
        //    pose_buf.pop();
        while (image_buf.front()->header.stamp.toSec() <
               pose_msg->header.stamp.toSec())
          image_buf.pop();
        image_msg = image_buf.front();
        image_buf.pop();

        while (point_buf.front()->header.stamp.toSec() <
               pose_msg->header.stamp.toSec())
          point_buf.pop();
        point_msg = point_buf.front();
        point_buf.pop();
      }
    }

    m_buf.unlock();

    if (pose_msg != NULL)
    {
      //printf(" pose time %f \n", pose_msg->header.stamp.toSec());
      //printf(" point time %f \n", point_msg->header.stamp.toSec());
      //printf(" image time %f \n", image_msg->header.stamp.toSec());
      // skip fisrt few
      //cout << "flag 4" << endl;
      if (skip_first_cnt < SKIP_FIRST_CNT)
      {
        skip_first_cnt++;
        continue;
      }

      if (skip_cnt < SKIP_CNT)
      {
        skip_cnt++;
        continue;
      } else
      {
        skip_cnt = 0;
      }
      //cout << "flag 5" << endl;
      cv_bridge::CvImageConstPtr ptr;
      if (image_msg->encoding == "8UC1")
      {
        sensor_msgs::Image img;
        img.header = image_msg->header;
        img.height = image_msg->height;
        img.width = image_msg->width;
        img.is_bigendian = image_msg->is_bigendian;
        img.step = image_msg->step;
        img.data = image_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
      } else
        ptr = cv_bridge::toCvCopy(image_msg,
                                  sensor_msgs::image_encodings::MONO8);

      cv::Mat image = ptr->image;
      // build keyframe
      Vector3d T = Vector3d(pose_msg->pose.pose.position.x,
                            pose_msg->pose.pose.position.y,
                            pose_msg->pose.pose.position.z);
      Matrix3d R = Quaterniond(pose_msg->pose.pose.orientation.w,
                               pose_msg->pose.pose.orientation.x,
                               pose_msg->pose.pose.orientation.y,
                               pose_msg->pose.pose.orientation.z).toRotationMatrix();
      if ((T - last_t).norm() > SKIP_DIS)
      {
        vector<cv::Point3f> point_3d;
        vector<cv::Point2f> point_2d_uv;
        vector<cv::Point2f> point_2d_normal;
        vector<double> point_id;

        for (unsigned int i = 0; i < point_msg->points.size(); i++)
        {
          cv::Point3f p_3d;
          p_3d.x = point_msg->points[i].x;
          p_3d.y = point_msg->points[i].y;
          p_3d.z = point_msg->points[i].z;
          point_3d.push_back(p_3d);

          cv::Point2f p_2d_uv, p_2d_normal;
          double p_id;
          p_2d_normal.x = point_msg->channels[i].values[0];
          p_2d_normal.y = point_msg->channels[i].values[1];
          p_2d_uv.x = point_msg->channels[i].values[2];
          p_2d_uv.y = point_msg->channels[i].values[3];
          p_id = point_msg->channels[i].values[4];
          point_2d_normal.push_back(p_2d_normal);
          point_2d_uv.push_back(p_2d_uv);
          point_id.push_back(p_id);

          //printf("u %f, v %f \n", p_2d_uv.x, p_2d_uv.y);
        }
        //cout << "flag 6" << endl;

        //KeyFrame* keyframe = new KeyFrame(pose_msg->header.stamp.toSec(), frame_index, T, R, image,
        //                   point_3d, point_2d_uv, point_2d_normal, point_id, sequence);

        // use the sequence as the keyframe index.
        static bool is_first_kf = true;
        if (is_first_kf)
        {
          is_first_kf = false;
          g_kf_idx_offset = pose_msg->header.seq;
        }
        KeyFrame *keyframe = new KeyFrame(pose_msg->header.stamp.toSec(),
                                          pose_msg->header.seq -
                                          g_kf_idx_offset,
                                          T, R, image, point_3d, point_2d_uv,
                                          point_2d_normal, point_id, sequence);
        m_process.lock();
        start_flag = 1;
        bool is_global_lp = posegraph.addKeyFrame(keyframe, 1);
        if (!is_global_lp) // disable local loop closure for now
        {
          //! 1. Perform active map registration
          Matrix3f R_ai_f;
          Vector3f t_ai_f;
          size_t inact_kf_ind, act_kf_ind;
          if (lm_map.registerActive(R_ai_f, t_ai_f, inact_kf_ind, act_kf_ind))
          {
            Matrix3d R_ai = R_ai_f.cast<double>();
            Vector3d t_ai = t_ai_f.cast<double>();


            //! 2. set up drift information
            Matrix3d R_ia = R_ai.transpose(), prev_drift_R = posegraph.r_drift;
            Vector3d t_ia = -R_ia * t_ai, prev_drift_t = posegraph.t_drift;
            Quaterniond q_ia(R_ia);
            double yaw_ia = Utility::R2ypr(
                R_ia).x(), prev_drift_yaw = posegraph.yaw_drift;
            cout << "***** Debug drift correction *****" << endl;
            cout << "new trans from act to inact: " << endl
                 << "t_ia: " << t_ia.transpose() << endl
                 << "R_ia: \n" << R_ia << endl;
            posegraph.yaw_drift = yaw_ia + prev_drift_yaw;
            //posegraph.r_drift = Utility::ypr2R(Vector3d(posegraph.yaw_drift, 0, 0)); // only correct yaw.
            posegraph.r_drift = R_ia * prev_drift_R;
            posegraph.t_drift = R_ia * prev_drift_t + t_ia;
            cout << "new drift: " << endl
                 << "t_drift: " << posegraph.t_drift.transpose() << endl
                 << "yaw_drift: " << posegraph.yaw_drift << endl;

            //! 3. publish active map middle kf undrifted odom
            nav_msgs::Odometry mid_kf_odom;
            mid_kf_odom.header.frame_id = "world";

            KeyFrame *mid_act_kf = posegraph.getKeyFrame(act_kf_ind);
            mid_kf_odom.header.stamp = ros::Time(mid_act_kf->time_stamp);
            Matrix3d mid_vio_R;
            Vector3d mid_vio_t;
            mid_act_kf->getVioPose(mid_vio_t, mid_vio_R);
            Matrix3d R_undrift = posegraph.r_drift * mid_vio_R;
            Vector3d t_undrift =
                posegraph.r_drift * mid_vio_t + posegraph.t_drift;
            mid_kf_odom.pose.pose.position.x = t_undrift[0];
            mid_kf_odom.pose.pose.position.y = t_undrift[1];
            mid_kf_odom.pose.pose.position.z = t_undrift[2];
            Quaterniond q_undrift(R_undrift);
            mid_kf_odom.pose.pose.orientation.x = q_undrift.x();
            mid_kf_odom.pose.pose.orientation.y = q_undrift.y();
            mid_kf_odom.pose.pose.orientation.z = q_undrift.z();
            mid_kf_odom.pose.pose.orientation.w = q_undrift.w();

            posegraph.pub_undrift_odom.publish(mid_kf_odom);
            pubTF(mid_act_kf, mid_vio_R, mid_vio_t);

            //! 4. Form pose graph constraint
            // forget about pose graph for now!

            // todo check if need to set earliest loop index. If so, set it. Could be triggering seg fault.

            auto p_act_kf = posegraph.getKeyFrame(act_kf_ind);
            auto p_inact_kf = posegraph.getKeyFrame(inact_kf_ind);
            Eigen::Matrix3d R_old_corrected = R_ia * p_act_kf->origin_vio_R;
            double rel_yaw = Utility::normalizeAngle(
                Utility::R2ypr(p_act_kf->origin_vio_R).x() -
                Utility::R2ypr(R_old_corrected).x());

            p_act_kf->has_loop = true;
            p_act_kf->loop_index = p_inact_kf->index;
            p_act_kf->loop_info << t_ia[0], t_ia[1], t_ia[2],
                q_ia.w(), q_ia.x(), q_ia.y(), q_ia.z(),
                rel_yaw;
            posegraph.setEarliestLoopIndex(
                p_inact_kf->index); // todo just added this line, without testing

            //! 5. Publish transformation constraint
            // todo: add this when need fast relocalization. for now we don't need it.

            //! 6. Add to optimization buffer
            posegraph.addLoopClosureKFIdx(act_kf_ind);

          }
        }
        m_process.unlock();
        frame_index++;
        last_t = T;
      }
    }

    // add margin cloud
    if (!margin_point_buf.empty() && !posegraph.isKFEmpty())
    {
      while (!margin_point_buf.empty()
             && margin_point_buf.front()->channels[3].values.front() -
                g_kf_idx_offset < 0)
        margin_point_buf.pop();

      if (!margin_point_buf.empty())
      {
        int max_kf_ind = int(*(std::max_element(
            margin_point_buf.front()->channels[3].values.begin(),
            margin_point_buf.front()->channels[3].values.end())))
                         - g_kf_idx_offset;
        if (max_kf_ind <= posegraph.getLastKFIdx())
        {
          margin_point_msg = margin_point_buf.front();
          margin_point_buf.pop();
          //! process marginalized cloud here
          // construct new map points
          size_t n_margin_points = margin_point_msg->channels[0].values.size();
          std::vector<MapPoint *> new_margin_frame(n_margin_points);
          size_t obs_start_it = 0;
          for (size_t i = 0; i < n_margin_points; i++)
          {
            size_t n_observations = margin_point_msg->channels[0].values[i];
            Vector3d mp_pos(margin_point_msg->points[i].x,
                            margin_point_msg->points[i].y,
                            margin_point_msg->points[i].z);
            std::vector<int> kf_ind(n_observations);
            for (auto it = obs_start_it;
                 it < obs_start_it + n_observations; it++)
            {
              // todo add the +1 in the end
              kf_ind[it - obs_start_it] =
                  int(margin_point_msg->channels[3].values[it]) -
                  g_kf_idx_offset + 1;
            }

            std::vector<float> u_in_image(n_observations), v_in_image(
                n_observations);
            std::copy(
                margin_point_msg->channels[1].values.begin() + obs_start_it,
                margin_point_msg->channels[1].values.begin() + obs_start_it +
                n_observations,
                u_in_image.begin());
            std::copy(
                margin_point_msg->channels[2].values.begin() + obs_start_it,
                margin_point_msg->channels[2].values.begin() + obs_start_it +
                n_observations,
                v_in_image.begin());

            std::vector<Eigen::Vector3d> pos_in_image(n_observations);
            for (size_t j = 0; j < n_observations; j++)
            {
              pos_in_image[j][0] = margin_point_msg->channels[4].values[
                  obs_start_it + j];
              pos_in_image[j][1] = margin_point_msg->channels[5].values[
                  obs_start_it + j];
              pos_in_image[j][2] = margin_point_msg->channels[6].values[
                  obs_start_it + j];
            }

            // undrift map point position
            mp_pos = posegraph.r_drift * mp_pos + posegraph.t_drift;

            MapPoint *pmp = new MapPoint(&posegraph, mp_pos, kf_ind, u_in_image,
                                         v_in_image, pos_in_image);
            new_margin_frame[i] = pmp;

            obs_start_it += n_observations;
          }
          lm_map.addMapPoints(new_margin_frame);

          //lm_map.pubActiveInactiveCloud();
        }
      }
    }

    std::chrono::milliseconds dura(5);
    std::this_thread::sleep_for(dura);
  }
}

void command()
{
  if (!LOOP_CLOSURE)
    return;
  while (ros::ok())
  {
    char c = getchar();
    if (c == 's')
    {
      m_process.lock();
      posegraph.savePoseGraph();
      m_process.unlock();
      printf(
          "save pose graph finish\nyou can set 'load_previous_pose_graph' to 1 in the config file to reuse it next time\n");
      // printf("program shutting down...\n");
      // ros::shutdown();
    }
    if (c == 'n')
      new_sequence();

    std::chrono::milliseconds dura(5);
    std::this_thread::sleep_for(dura);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pose_graph");
  ros::NodeHandle n("~");
  posegraph.registerPub(n);
  lm_map.registerPub(n);

  posegraph.setLMMap(&lm_map);

  // read param
  n.getParam("visualization_shift_x", VISUALIZATION_SHIFT_X);
  n.getParam("visualization_shift_y", VISUALIZATION_SHIFT_Y);
  n.getParam("skip_cnt", SKIP_CNT);
  n.getParam("skip_dis", SKIP_DIS);
  std::string config_file;
  n.getParam("config_file", config_file);
  cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
  if (!fsSettings.isOpened())
  {
    std::cerr << "ERROR: Wrong path to settings" << std::endl;
  }

  double camera_visual_size = fsSettings["visualize_camera_size"];
  cameraposevisual.setScale(camera_visual_size);
  cameraposevisual.setLineWidth(camera_visual_size / 10.0);


  LOOP_CLOSURE = fsSettings["loop_closure"];
  std::string IMAGE_TOPIC;
  int LOAD_PREVIOUS_POSE_GRAPH;
  if (LOOP_CLOSURE)
  {
    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    std::string pkg_path = ros::package::getPath("pose_graph");
    string vocabulary_file = pkg_path + "/../support_files/brief_k10L6.bin";
    cout << "vocabulary_file" << vocabulary_file << endl;
    posegraph.loadVocabulary(vocabulary_file);

    BRIEF_PATTERN_FILE = pkg_path + "/../support_files/brief_pattern.yml";
    cout << "BRIEF_PATTERN_FILE" << BRIEF_PATTERN_FILE << endl;
    m_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(
        config_file.c_str());

    fsSettings["image_topic"] >> IMAGE_TOPIC;
    fsSettings["pose_graph_save_path"] >> POSE_GRAPH_SAVE_PATH;
    fsSettings["output_path"] >> VINS_RESULT_PATH;
    fsSettings["save_image"] >> DEBUG_IMAGE;
    //fsSettings["debug_loop_closure"] >> DEBUG_LOOP_CLOSURE;

    // create folder if not exists
    FileSystemHelper::createDirectoryIfNotExists(POSE_GRAPH_SAVE_PATH.c_str());
    FileSystemHelper::createDirectoryIfNotExists(VINS_RESULT_PATH.c_str());

    VISUALIZE_IMU_FORWARD = fsSettings["visualize_imu_forward"];
    LOAD_PREVIOUS_POSE_GRAPH = fsSettings["load_previous_pose_graph"];
    FAST_RELOCALIZATION = fsSettings["fast_relocalization"];
    VINS_RESULT_PATH = VINS_RESULT_PATH + "/vins_result_loop.csv";
    std::ofstream fout(VINS_RESULT_PATH, std::ios::out);
    fout.close();
    fsSettings.release();

    if (LOAD_PREVIOUS_POSE_GRAPH)
    {
      printf("load pose graph\n");
      m_process.lock();
      posegraph.loadPoseGraph();
      m_process.unlock();
      printf("load pose graph finish\n");
      load_flag = 1;
    } else
    {
      printf("no previous pose graph\n");
      load_flag = 1;
    }
  }

  fsSettings.release();

  ros::Subscriber sub_imu_forward = n.subscribe("/vins_estimator/imu_propagate",
                                                2000, imu_forward_callback);
  ros::Subscriber sub_vio = n.subscribe("/vins_estimator/odometry", 2000,
                                        vio_callback);
  ros::Subscriber sub_image = n.subscribe(IMAGE_TOPIC, 2000, image_callback);
  ros::Subscriber sub_pose = n.subscribe("/vins_estimator/keyframe_pose", 2000,
                                         pose_callback);
  ros::Subscriber sub_extrinsic = n.subscribe("/vins_estimator/extrinsic", 2000,
                                              extrinsic_callback);
  ros::Subscriber sub_point = n.subscribe("/vins_estimator/keyframe_point",
                                          2000, point_callback);
  ros::Subscriber sub_margin_point = n.subscribe(
      "/vins_estimator/history_cloud", 2000, margin_point_callback);
  ros::Subscriber sub_relo_relative_pose = n.subscribe(
      "/vins_estimator/relo_relative_pose", 2000, relo_relative_pose_callback);

  pub_match_img = n.advertise<sensor_msgs::Image>("match_image", 1000);
  pub_camera_pose_visual = n.advertise<visualization_msgs::MarkerArray>(
      "camera_pose_visual", 1000);
  pub_key_odometrys = n.advertise<visualization_msgs::Marker>("key_odometrys",
                                                              1000);
  pub_vio_path = n.advertise<nav_msgs::Path>("no_loop_path", 1000);
  pub_match_points = n.advertise<sensor_msgs::PointCloud>("match_points", 100);

  std::thread measurement_process;
  std::thread keyboard_command_process;

  measurement_process = std::thread(process);
  keyboard_command_process = std::thread(command);


  ros::spin();

  return 0;
}
