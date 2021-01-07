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

#include <ros/ros.h>
#include <ze/common/logging.hpp>
#include <gflags/gflags.h>

#include <ze/visualization/viz_ros.hpp>

int main(int argc, char** argv)
{
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  // Visualizer internally initializes ROS and creates a node handle.
  ze::VisualizerRos visualizer;

  for(int i = 0; i < 100; ++i)
  {
    visualizer.drawPoint(
          "point", i, ze::Vector3(0, 0, i), ze::Colors::DarkBlue);

    visualizer.drawLine(
          "line", i, ze::Vector3(i, 0, 0), ze::Vector3(i, 0.4, 0), ze::Colors::DarkRed);

    visualizer.drawCoordinateFrame(
          "frame", i, ze::Transformation(ze::Quaternion(), ze::Vector3(i, i, i)*0.2));

    ze::Positions points(3, 4);
    points << 0, 1, 2, 3,
              1, 1, 1, 1,
              i, i, i, i;
    visualizer.drawPoints("points", 0, points, ze::Colors::Blue);

    ze::LineMarkers lines;
    lines.push_back(std::make_pair(ze::Position(i, 0, 1), ze::Position(i, 0, 0)));
    lines.push_back(std::make_pair(ze::Position(i, 1, 1), ze::Position(i, 1, 0)));
    lines.push_back(std::make_pair(ze::Position(i, 2, 1), ze::Position(i, 2, 0)));
    lines.push_back(std::make_pair(ze::Position(i, 3, 1), ze::Position(i, 3, 0)));
    visualizer.drawLines("lines", 0, lines, ze::Colors::Green);

    ze::TransformationVector poses;
    poses.push_back(ze::Transformation(ze::Quaternion(ze::Vector3(0.1, 0, 0)), ze::Position(i, 0, 0)));
    poses.push_back(ze::Transformation(ze::Quaternion(ze::Vector3(0.2, 0, 0)), ze::Position(i, 1, 0)));
    poses.push_back(ze::Transformation(ze::Quaternion(ze::Vector3(0.3, 0, 0)), ze::Position(i, 2, 0)));
    poses.push_back(ze::Transformation(ze::Quaternion(ze::Vector3(0.4, 0, 0)), ze::Position(i, 3, 0)));
    visualizer.drawCoordinateFrames("poses", 0, poses);

    sleep(1);
  }
}
