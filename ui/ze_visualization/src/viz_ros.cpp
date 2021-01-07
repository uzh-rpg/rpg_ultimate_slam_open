// Copyright (c) 2015-2016, ETH Zurich, Wyss Zurich, Zurich Eye
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the ETH Zurich, Wyss Zurich, Zurich Eye nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL ETH Zurich, Wyss Zurich, Zurich Eye BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Modified: Robotics and Perception Group

#include <ze/visualization/viz_ros.hpp>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <ze/ros/tf_bridge.hpp>
#include <ze/visualization/viz_ros_utils.hpp>

namespace ze {

VisualizerRos::VisualizerRos()
{
  // Inititialize ROS if it was not initialized before.
  if(!ros::isInitialized())
  {
    VLOG(1) << "Initializting ROS";
    int argc = 0;
    ros::init(argc, nullptr, std::string("ze_visualization"));
  }

  // Create node and subscribe.
  nh_.reset(new ros::NodeHandle("~"));
  pub_marker_.reset(new ros::Publisher(nh_->advertise<visualization_msgs::Marker>("markers", 100)));
  tf_broadcaster_.reset(new tf::TransformBroadcaster());
}

VisualizerRos::VisualizerRos(const std::string& frame)
  : VisualizerRos::VisualizerRos()
{
  world_frame = frame;
}

void VisualizerRos::drawPoint(
    const std::string& ns,
    const size_t id,
    const Position& point,
    const Color& color,
    const real_t size)
{
  if(pub_marker_->getNumSubscribers() == 0)
    return;

  visualization_msgs::Marker m;
  m.header.frame_id = world_frame;
  m.header.stamp = ros::Time::now();
  m.ns = ns;
  m.id = id;
  m.type = visualization_msgs::Marker::CUBE;
  m.action = 0; // add/modify
  real_t marker_scale = size * viz_scale_;
  m.scale.x = marker_scale;
  m.scale.y = marker_scale;
  m.scale.z = marker_scale;
  m.color = getRosColor(color);
  m.pose.position = getRosPoint(point);
  pub_marker_->publish(m);
}

void VisualizerRos::drawLine(
    const std::string& ns,
    const size_t id,
    const Position& line_from,
    const Position& line_to,
    const Color& color,
    const real_t size)
{
  if(pub_marker_->getNumSubscribers() == 0)
    return;

  visualization_msgs::Marker m;
  m.header.frame_id = world_frame;
  m.header.stamp = ros::Time::now();
  m.ns = ns;
  m.id = id;
  m.type = visualization_msgs::Marker::LINE_STRIP;
  m.action = 0; // 0 = add/modify
  m.scale.x = size * viz_scale_;
  m.scale.y = size * viz_scale_;
  m.scale.z = size * viz_scale_;
  m.color = getRosColor(color);
  m.points.reserve(2);
  m.points.push_back(getRosPoint(line_from));
  m.points.push_back(getRosPoint(line_to));
  pub_marker_->publish(m);
}

void VisualizerRos::drawCoordinateFrame(
    const std::string& ns,
    const size_t id,
    const Transformation& pose, // T_W_B
    const real_t size)
{
  if(pub_marker_->getNumSubscribers() == 0)
    return;

  const Vector3& p = pose.getPosition();

  visualization_msgs::Marker m;
  m.header.frame_id = world_frame;
  m.header.stamp = ros::Time::now();
  m.ns = ns;
  m.id = id;
  m.type = visualization_msgs::Marker::LINE_LIST;
  m.action = 0; // 0 = add/modify
  m.scale.x = size * viz_scale_ * 0.05;
  m.colors.reserve(6);
  m.points.reserve(6);
  real_t length = size * viz_scale_;
  m.points.push_back(getRosPoint(Vector3::Zero()));
  m.colors.push_back(getRosColor(Colors::Red));
  m.points.push_back(getRosPoint(Vector3::UnitX() * length));
  m.colors.push_back(getRosColor(Colors::Red));
  m.points.push_back(getRosPoint(Vector3::Zero()));
  m.colors.push_back(getRosColor(Colors::Green));
  m.points.push_back(getRosPoint(Vector3::UnitY() * length));
  m.colors.push_back(getRosColor(Colors::Green));
  m.points.push_back(getRosPoint(Vector3::Zero()));
  m.colors.push_back(getRosColor(Colors::Blue));
  m.points.push_back(getRosPoint(Vector3::UnitZ() * length));
  m.colors.push_back(getRosColor(Colors::Blue));
  m.pose = getRosPose(pose);
  pub_marker_->publish(m);
}

void VisualizerRos::drawRobot(const std::string& name,
    const Transformation& T_W_B,
    const ros::Time &stamp)
{
  tf::StampedTransform tf;
  tf.stamp_ = (stamp == ros::Time(0)) ?
              ros::Time::now() : stamp;
  tf.frame_id_ = world_frame;
  tf.child_frame_id_ = name;
  tf.setData(transformationToTF(T_W_B));
  tf_broadcaster_->sendTransform(tf);
}

void VisualizerRos::drawPoints(
    const std::string& ns,
    const size_t id,
    const Positions& points,
    const Color& color,
    const real_t size)
{
  if(pub_marker_->getNumSubscribers() == 0)
    return;

  visualization_msgs::Marker m;
  m.header.frame_id = world_frame;
  m.header.stamp = ros::Time::now();
  m.ns = ns;
  m.id = id;
  m.type = visualization_msgs::Marker::POINTS;
  m.action = 0; // add/modify
  real_t marker_scale = size * viz_scale_;
  m.scale.x = marker_scale;
  m.scale.y = marker_scale;
  m.scale.z = marker_scale;
  m.color = getRosColor(color);
  m.points.reserve(points.cols());
  for(int i = 0; i < points.cols(); ++i)
  {
    if(points.col(i).norm() > 1e4)
    {
      continue;
    }
    m.points.push_back(getRosPoint(points.col(i)));
  }
  pub_marker_->publish(m);
}

void VisualizerRos::drawLines(
    const std::string& ns,
    const size_t id,
    const LineMarkers& lines,
    const Color& color,
    const real_t size)
{
  if(pub_marker_->getNumSubscribers() == 0)
    return;

  visualization_msgs::Marker m;
  m.header.frame_id = world_frame;
  m.header.stamp = ros::Time::now();
  m.ns = ns;
  m.id = id;
  m.type = visualization_msgs::Marker::LINE_LIST;
  m.action = 0; // 0 = add/modify
  m.scale.x = size * viz_scale_;
  m.color = getRosColor(color);
  m.points.reserve(lines.size() * 2);
  for(size_t i = 0; i < lines.size(); ++i)
  {
    m.points.push_back(getRosPoint(lines[i].first));
    m.points.push_back(getRosPoint(lines[i].second));
  }
  pub_marker_->publish(m);
}

void VisualizerRos::drawCoordinateFrames(
    const std::string& ns,
    const size_t id,
    const TransformationVector& poses,
    const real_t size)
{
  if(pub_marker_->getNumSubscribers() == 0)
    return;

  visualization_msgs::Marker m;
  m.header.frame_id = world_frame;
  m.header.stamp = ros::Time::now();
  m.ns = ns;
  m.id = id;
  m.type = visualization_msgs::Marker::LINE_LIST;
  m.action = 0; // 0 = add/modify
  m.scale.x = size * viz_scale_ * 0.05;
  m.colors.reserve(poses.size() * 3);
  m.points.reserve(poses.size() * 3);
  real_t length = size * viz_scale_;
  for(const Transformation& T : poses)
  {
    const Vector3& p = T.getPosition();
    const Matrix3 R = T.getRotationMatrix();
    m.points.push_back(getRosPoint(p));
    m.colors.push_back(getRosColor(Colors::Red));
    m.points.push_back(getRosPoint(p + R.col(0) * length));
    m.colors.push_back(getRosColor(Colors::Red));
    m.points.push_back(getRosPoint(p));
    m.colors.push_back(getRosColor(Colors::Green));
    m.points.push_back(getRosPoint(p + R.col(1) * length));
    m.colors.push_back(getRosColor(Colors::Green));
    m.points.push_back(getRosPoint(p));
    m.colors.push_back(getRosColor(Colors::Blue));
    m.points.push_back(getRosPoint(p + R.col(2) * length));
    m.colors.push_back(getRosColor(Colors::Blue));
  }
  pub_marker_->publish(m);
}

void VisualizerRos::drawTrajectory(
    const std::string& topic,
    const size_t id,
    const std::vector<Position>& points,
    const Color& color,
    const real_t size)
{
  if(pub_marker_->getNumSubscribers() == 0)
    return;

  visualization_msgs::Marker m;
  m.header.frame_id = world_frame;
  m.header.stamp = ros::Time::now();
  m.ns = topic;
  m.id = id;
  m.type = visualization_msgs::Marker::LINE_STRIP;
  m.action = 0; // 0 = add/modify
  m.scale.x = size * viz_scale_ * 0.01;
  m.color = getRosColor(color);
  m.points.reserve(points.size());
  for (size_t i = 0u; i < points.size(); ++i)
  {
    m.points.push_back(getRosPoint(points[i]));
  }
  pub_marker_->publish(m);
}

} // namespace ze
