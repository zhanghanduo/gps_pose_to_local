/**
 * This file is part of S-SLAM.
 *
 * Copyright (C) 2013-2017 Taihú Pire
 * Copyright (C) 2014-2017 Thomas Fischer
 * Copyright (C) 2016-2017 Gastón Castro
 * Copyright (C) 2017 Matias Nitsche
 * Copyright (C) 2018 Zhang Handuo
 * For more information see <git@gitlab.com:handuo/sslam.git>
 *
 * S-SLAM is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * S-SLAM is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with S-SLAM. If not, see <http://www.gnu.org/licenses/>.
 *
 * Authors:  Zhang Handuo
 *           Taihú Pire
 *           Thomas Fischer
 *           Gastón Castro
 *           Matías Nitsche
 *
 * Laboratory of ST Corporation Lab of Robotics
 * Department of Electrical and Electronics Engineering
 * Nanyang Technological University
 */

#include "include/utils/CameraPose.hpp"

CameraPose::CameraPose()
  : position_( Eigen::Vector3d::Zero() ), orientation_( Eigen::Quaterniond::Identity() )
  , orientationMatrix_( Eigen::Matrix3d::Identity() ), covariance_( Eigen::Matrix6d::Identity() )
{}

CameraPose::CameraPose(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation, const Eigen::Matrix6d& covariance)
  : position_( position ), orientation_( orientation ), covariance_( covariance )
{
  orientationMatrix_ = orientation_.toRotationMatrix();
}

std::ostream& operator << ( std::ostream& os, const CameraPose& cameraPose)
{
  const Eigen::Quaterniond& orientation = cameraPose.GetOrientationQuaternion();
  return os << cameraPose.GetPosition() << " [" << orientation.x() << ", " << orientation.y() << ", " << orientation.z() << ", " << orientation.w() << "]";
}
