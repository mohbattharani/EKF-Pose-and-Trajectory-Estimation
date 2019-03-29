/*
 *  QUT cyphy ardrone position estimation code
 *  Copyright (C) 2012, CYPHY lab
 *  Inkyu Sa <i.sa@qut.edu.au>
 *
 *  http://wiki.qut.edu.au/display/cyphy
 *
 *
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


#include "ardrone_pose_estimation.h"

int main(int argc, char** argv)
{
  ROS_INFO("Ardrone pose estimation node");
  ros::init(argc, argv, "ardrone_pose_estimation");
  ros::NodeHandle nh;
  pose_esti pose_esti(nh);  
  ros::spin();
  return 0;
}
