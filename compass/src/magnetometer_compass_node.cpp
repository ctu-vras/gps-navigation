/**
 * \file
 * \brief Compute various azimuth values based on a magnetometer, IMU orientation and possibly also GPS coordinates.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <ros/datatypes.h>
#include <ros/init.h>
#include <ros/this_node.h>

#include <nodelet/detail/callback_queue.h>
#include <nodelet/detail/callback_queue_manager.h>

#include <compass/magnetometer_compass_nodelet.h>


int main(int argc, char** argv)
{
  // Remove the program name from argv because the nodelet handling code does not expect it
  argc -= 1;
  argv += 1;
  ros::V_string my_argv;
  ros::removeROSArgs(argc, argv, my_argv);
  ros::init(argc, argv, "magnetometer_compass");

  ros::NodeHandle pnh("~");
  nodelet::detail::CallbackQueueManager manager(pnh.param("num_worker_threads", 1));

  // Declaration order of these variables is important to make sure they can be properly stopped and destroyed.
  auto nodelet = boost::make_shared<compass::MagnetometerCompassNodelet>();
  auto stQueue = boost::make_shared<nodelet::detail::CallbackQueue>(&manager, nodelet);
  auto mtQueue = boost::make_shared<nodelet::detail::CallbackQueue>(&manager, nodelet);
  manager.addQueue(stQueue, false);
  manager.addQueue(mtQueue, true);

  nodelet->init(ros::this_node::getName(), {}, my_argv, stQueue.get(), mtQueue.get());

  ros::spin();

  // Destroying the nodelet helps stopping the queues.
  nodelet.reset();
  manager.removeQueue(stQueue);
  manager.removeQueue(mtQueue);
  manager.stop();
}