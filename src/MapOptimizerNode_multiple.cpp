/*
Copyright (c) 2010-2014, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <ros/ros.h>
#include "rtabmap_ros/MapData.h"
#include "rtabmap_ros/MapGraph.h"
#include "rtabmap_ros/MsgConversion.h"
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/Graph.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UConversion.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <tf2_ros/transform_broadcaster.h>
#include <boost/thread.hpp>
#include "rtabmap/core/Compression.h"
#include <fstream>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

using namespace rtabmap;

class MapOptimizer
{

public:
	MapOptimizer() :
		mapFrameId_("map"),
		odomFrameId_("odom"),
		globalOptimization_(true),
		optimizeFromLastNode_(false),
		mapToOdom_(rtabmap::Transform::getIdentity()),
		transformThread_(0)
	{
		ros::NodeHandle nh;
		ros::NodeHandle pnh("~");

		double epsilon = 0.0;
		bool robust = true;
		bool slam2d =false;
		int strategy = 1; // 0=TORO, 1=g2o, 2=GTSAM
		int iterations = 5;
		bool ignoreVariance = false;

		pnh.param("map_frame_id", mapFrameId_, mapFrameId_);
		pnh.param("odom_frame_id", odomFrameId_, odomFrameId_);
		pnh.param("iterations", iterations, iterations);
		pnh.param("ignore_variance", ignoreVariance, ignoreVariance);
		pnh.param("global_optimization", globalOptimization_, globalOptimization_);
		pnh.param("optimize_from_last_node", optimizeFromLastNode_, optimizeFromLastNode_);
		pnh.param("epsilon", epsilon, epsilon);
		pnh.param("robust", robust, robust);
		pnh.param("slam_2d", slam2d, slam2d);
		pnh.param("strategy", strategy, strategy);


		UASSERT(iterations > 0);

		ParametersMap parameters;
		parameters.insert(ParametersPair(Parameters::kRGBDOptimizeStrategy(), uNumber2Str(strategy)));
		parameters.insert(ParametersPair(Parameters::kRGBDOptimizeEpsilon(), uNumber2Str(epsilon)));
		parameters.insert(ParametersPair(Parameters::kRGBDOptimizeIterations(), uNumber2Str(iterations)));
		parameters.insert(ParametersPair(Parameters::kRGBDOptimizeRobust(), uBool2Str(robust)));
		parameters.insert(ParametersPair(Parameters::kRGBDOptimizeSlam2D(), uBool2Str(slam2d)));
		parameters.insert(ParametersPair(Parameters::kRGBDOptimizeVarianceIgnored(), uBool2Str(ignoreVariance)));
		optimizer_ = graph::Optimizer::create(parameters);

		double tfDelay = 0.05; // 20 Hz
		bool publishTf = true;
		pnh.param("publish_tf", publishTf, publishTf);
		pnh.param("tf_delay", tfDelay, tfDelay);

		mapDataTopic_ = nh.subscribe("/rtabmap/mapData", 100, &MapOptimizer::mapDataReceivedCallback, this);
		mapDataPub_ = nh.advertise<rtabmap_ros::MapData>(nh.resolveName("mapData")+"_optimized", 10);
		mapGraphPub_ = nh.advertise<rtabmap_ros::MapGraph>(nh.resolveName("mapData")+"Graph_optimized", 10);
		landmarkPosePub_ = nh.advertise<visualization_msgs::Marker>("LandmarkOptimized", 1);
		robotPosePub_= nh.advertise<visualization_msgs::Marker>("RobotOptimized", 1);
		if(publishTf)
		{
			ROS_INFO("map_optimizer will publish tf between frames \"%s\" and \"%s\"", mapFrameId_.c_str(), odomFrameId_.c_str());
			ROS_INFO("map_optimizer: map_frame_id = %s", mapFrameId_.c_str());
			ROS_INFO("map_optimizer: odom_frame_id = %s", odomFrameId_.c_str());
			ROS_INFO("map_optimizer: tf_delay = %f", tfDelay);
			transformThread_ = new boost::thread(boost::bind(&MapOptimizer::publishLoop, this, tfDelay));
		}
		landmarks.open("../optimization.txt");
		//landmarks<<"file initialized"<<std::endl;
	}

	~MapOptimizer()
	{
		if(transformThread_)
		{
			transformThread_->join();
			delete transformThread_;
		}
	}

	void publishLoop(double tfDelay)
	{
		if(tfDelay == 0)
			return;
		ros::Rate r(1.0 / tfDelay);
		while(ros::ok())
		{
			mapToOdomMutex_.lock();
			ros::Time tfExpiration = ros::Time::now() + ros::Duration(tfDelay);
			geometry_msgs::TransformStamped msg;
			msg.child_frame_id = odomFrameId_;
			msg.header.frame_id = mapFrameId_;
			msg.header.stamp = tfExpiration;
			rtabmap_ros::transformToGeometryMsg(mapToOdom_, msg.transform);
			tfBroadcaster_.sendTransform(msg);
			mapToOdomMutex_.unlock();
			r.sleep();
		}
	}

	/*Dina Youakim*/
	void initializeLandmarkCounter(rtabmap_ros::MapDataConstPtr & msg)
	{
		int maxNodeID = -1;
		for(unsigned int i=0; i<msg->nodes.size(); ++i)
		{
			rtabmap_ros::NodeData currentNode = msg->nodes[i];
			if(maxNodeID<currentNode.id)
				maxNodeID = currentNode.id;
		}
		landmarkCounter = maxNodeID;
	}

	/* Dina Youakim */
	void fillOldLandmarks(rtabmap_ros::MapDataConstPtr & msg,std::map<int, rtabmap::Transform> &posesOut, std::multimap<int, rtabmap::Link> &linksOut)
	{
		
		if(!landmarksCache_.empty())
		{
			//landmarks<<"start inserting old landmark nodes "<<std::endl;
			for(std::map<int, std::vector<rtabmap::Transform> >::iterator iter = landmarksCache_.begin(); iter != landmarksCache_.end(); ++iter)
			{
				std::vector<rtabmap::Transform> currentList = iter->second;
				//ROS_INFO_STREAM("new vector fetched from the map with size " <<currentList.size());
				for(int i=0; i< currentList.size();i++)
				{
					rtabmap::Transform landmarkObservation = currentList[i];
					rtabmap::Transform landmarkToRobotPose ;
					//ROS_INFO_STREAM("stored mat at "<<i<<" fetched from vetor");
					//ROS_INFO_STREAM("stored data: "<<landmarkObservation);
					landmarkCounter++;
					for (unsigned int i=0; i<msg->nodes.size(); ++i)
					{
						if(msg->nodes[i].id == iter->first)
						{
							double roll, pitch, yaw;
							tf::Quaternion quat;
			    			tf::quaternionMsgToTF(msg->nodes[i].pose.orientation, quat);
							tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
							rtabmap::Transform robotPose(msg->nodes[i].pose.position.x,msg->nodes[i].pose.position.y,msg->nodes[i].pose.position.z,roll,pitch,yaw);
							landmarkToRobotPose =  (robotPose)*landmarkObservation;
							posesOut.insert(std::make_pair(landmarkCounter,landmarkObservation));
							break;
						}
					}

					/* Create landmark link*/
					rtabmap::Link link;
					link.setFrom(iter->first);
					link.setTo (landmarkCounter);
					link.setType (rtabmap::Link::kLandmark);
					link.setTransform(landmarkToRobotPose);
					linksOut.insert(std::make_pair(link.from(),link));
					//landmarks<<"landmark node created between "<<iter->first<<","<<landmarkCounter<<std::endl;
					//landmarks<<"the landmark pose is: "<<landmarkObservation.translation().x()<<","<<landmarkObservation.translation().y()<<","<<landmarkObservation.theta()<<std::endl;
					//landmarks<<"the landmark constraint is: "<<landmarkToRobotPose.translation().x()<<","<<landmarkToRobotPose.translation().y()<<","<<landmarkToRobotPose.translation().theta()<<std::endl;
				}
			}
		//landmarks<<"end inserting old landmark nodes "<<std::endl;
		}

	}

    /* Dina Youakim*/
    void createLandmarks(rtabmap_ros::MapDataConstPtr & msg, std::map<int, rtabmap::Transform> &posesOut, std::multimap<int, rtabmap::Link> & linksOut)
	{
		for(unsigned int i=0; i<msg->nodes.size(); ++i)
		{
			rtabmap_ros::NodeData currentNode = msg->nodes[i];
			
			//landmarks<<"current node stamp: "<<currentNode.stamp<<std::endl;
			if(!currentNode.userData.empty())
			{
				//landmarks<<"user data not empty"<<std::endl;
				landmarkCounter++;
				cv::Mat landmark = rtabmap::uncompressData(currentNode.userData);

				rtabmap::Transform landmarkObservation(landmark.at<double>(0,0),landmark.at<double>(0,1),landmark.at<double>(0,2),
					landmark.at<double>(1,0),landmark.at<double>(1,1),landmark.at<double>(1,2));

				double roll, pitch, yaw;
				tf::Quaternion quat;
    			tf::quaternionMsgToTF(currentNode.pose.orientation, quat);
				tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
				rtabmap::Transform robotPose(currentNode.pose.position.x,currentNode.pose.position.y,currentNode.pose.position.z,roll,pitch,yaw);
				//rtabmap::Transform landmarkPose = (robotPose)*landmarkObservation;
				rtabmap::Transform landmarkToRobotPose =  (robotPose)*landmarkObservation;
				/*rtabmap::Transform t2 = landmarkObservation*robotPose;
				rtabmap::Transform t3 = landmarkObservation*robotPose.inverse();
				rtabmap::Transform t4 = landmarkObservation.inverse();*/
				posesOut.insert(std::make_pair(landmarkCounter,landmarkObservation));

				//landmarks<<"observed at time: "<<currentNode.stamp<<", for node: "<<currentNode.id<<std::endl;
				/*landmarks<<"(1) "<<landmarkToRobotPose.translation().x()<<","<<landmarkToRobotPose.translation().y()<<","<<landmarkToRobotPose.theta()<<std::endl;
				landmarks<<"(2) "<<landmarkPose.translation().x()<<","<<landmarkPose.translation().y()<<","<<landmarkPose.theta()<<std::endl;
				landmarks<<"(3) "<<t2.translation().x()<<","<<t2.translation().y()<<","<<t2.theta()<<std::endl;
				landmarks<<"(4) "<<t3.translation().x()<<","<<t3.translation().y()<<","<<t3.theta()<<std::endl;
				landmarks<<"(5) "<<landmarkObservation.translation().x()<<","<<landmarkObservation.translation().y()<<","<<landmarkObservation.theta()<<std::endl;
				landmarks<<"(6) "<<t4.translation().x()<<","<<t4.translation().y()<<","<<t4.theta()<<std::endl;*/
				//landmarks<<"landmark trans: "<<landmarkObservation.translation().x()<<","<<landmarkObservation.translation().y()<<","<<landmarkObservation.translation().z()<<std::endl; //landmarkObservation.translation()
				float r,p,y;
				landmarkObservation.getEulerAngles(r,p,y);
				//landmarks<<"landmark rot: "<<r<<","<<p<<","<<y<<","<<landmarkObservation.theta()<<std::endl; //landmarkObservation.rotation()

				/* Create landmark link*/
				rtabmap::Link link;
				link.setFrom(currentNode.id);
				link.setTo (landmarkCounter);
				link.setType (rtabmap::Link::kLandmark);
				link.setTransform(landmarkToRobotPose);
				//link.setInfMatrix() ???;
				//link.setVariance() ???;
				linksOut.insert(std::make_pair(link.from(),link));
				//landmarks<<"landmark node created between "<<currentNode.id<<","<<landmarkCounter<<std::endl;
				landmarks<<landmarkObservation.translation().x()<<","<<landmarkObservation.translation().y()<<","<<landmarkObservation.theta()<<std::endl;
			    //landmarks<<"the landmark constraint is: "<<landmarkToRobotPose.translation().x()<<","<<landmarkToRobotPose.translation().y()<<","<<landmarkToRobotPose.theta()<<std::endl;
				std::map<int,std::vector< rtabmap::Transform > >::iterator  it ; 
				if(!landmarksCache_.empty())
				{
					it = landmarksCache_.find(currentNode.id);
					if(it != landmarksCache_.end())
					{
						it->second.push_back(landmarkObservation);
					}
					else
					{
						std::vector< rtabmap::Transform > currentList;
						currentList.push_back(landmarkObservation);
						landmarksCache_.insert(std::make_pair<int, std::vector <rtabmap::Transform> >(currentNode.id,currentList));
					}
					
				}
				else 
				{
					std::vector< rtabmap::Transform > currentList;
					currentList.push_back(landmarkObservation);
					landmarksCache_.insert(std::make_pair<int, std::vector <rtabmap::Transform> >(currentNode.id,currentList));
				}
			}
			else
			{
				//landmarks<<"user data empty"<<std::endl;
			}
		}
		//landmarks<<"current landmark cache size is: "<<landmarksCache_.size()<<std::endl;
	}


	void mapDataReceivedCallback(const rtabmap_ros::MapDataConstPtr & msg)
	{
		// save new poses and constraints
		// Assuming that nodes/constraints are all linked together
		//landmarks<<"new map data received"<<std::endl;
		UASSERT(msg->graph.posesId.size() == msg->graph.poses.size());

		bool dataChanged = false;

		//Dina Youakim
		rtabmap_ros::MapDataConstPtr modifiedMapData = msg; 
		//createLandmarks(modifiedMapData);
		
		std::multimap<int, Link> newConstraints;
		for(unsigned int i=0; i<modifiedMapData->graph.links.size(); ++i)
		{
			Link link = rtabmap_ros::linkFromROS(modifiedMapData->graph.links[i]);
			newConstraints.insert(std::make_pair(link.from(), link));
			bool edgeAlreadyAdded = false;
			for(std::multimap<int, Link>::iterator iter = cachedConstraints_.lower_bound(link.from());
					iter != cachedConstraints_.end() && iter->first == link.from();
					++iter)
			{
				if(iter->second.to() == link.to())
				{
					edgeAlreadyAdded = true;
					if(iter->second.transform().getDistanceSquared(link.transform()) > 0.0001)
					{
						ROS_WARN("%d ->%d (%s vs %s)",iter->second.from(), iter->second.to(), iter->second.transform().prettyPrint().c_str(),
								link.transform().prettyPrint().c_str());
						dataChanged = true;
					}
				}
			}
			if(!edgeAlreadyAdded)
			{
				//landmarks<<"new constraint from-To:"<<link.from()<<","<<link.to()<<" with values: "<<link.transform().x()<<","<<link.transform().y()<<","<<link.transform().theta()<<std::endl;
			
				cachedConstraints_.insert(std::make_pair(link.from(), link));
			}
		}

		std::map<int, Signature> newNodeInfos;
		// add new odometry poses
		for(unsigned int i=0; i<modifiedMapData->nodes.size(); ++i)
		{
			int id = modifiedMapData->nodes[i].id;
			Transform pose = rtabmap_ros::transformFromPoseMsg(modifiedMapData->nodes[i].pose);
			Signature s = rtabmap_ros::nodeInfoFromROS(modifiedMapData->nodes[i]);
			newNodeInfos.insert(std::make_pair(id, s));

			std::pair<std::map<int, Signature>::iterator, bool> p = cachedNodeInfos_.insert(std::make_pair(id, s));
			if(!p.second && pose.getDistanceSquared(cachedNodeInfos_.at(id).getPose()) > 0.0001)
			{
				dataChanged = true;
			}
			//landmarks<<"Pose: "<<s.id()<<","<<s.getStamp()<<","<<pose.x()<<","<<pose.y()<<","<<pose.z()<<","<<pose.theta()<<std::endl;
		
		}

		if(dataChanged)
		{
			ROS_WARN("Graph data has changed! Reset cache...");
			cachedConstraints_ = newConstraints;
			cachedNodeInfos_ = newNodeInfos;
		}

		//match poses in the graph
		std::multimap<int, Link> constraints;
		std::map<int, Signature> nodeInfos;
		
		if(globalOptimization_)
		{
			//landmarks<<"globalOptimization_"<<std::endl;
			constraints = cachedConstraints_;
			nodeInfos = cachedNodeInfos_;
		}
		else
		{
			//landmarks<<"NOT globalOptimization_"<<std::endl;
			constraints = newConstraints;
			for(unsigned int i=0; i<modifiedMapData->graph.posesId.size(); ++i)
			{
				std::map<int, Signature>::iterator iter = cachedNodeInfos_.find(modifiedMapData->graph.posesId[i]);
				if(iter != cachedNodeInfos_.end())
				{
					nodeInfos.insert(*iter);
				}
				else
				{
					ROS_ERROR("Odometry pose of node %d not found in cache!", modifiedMapData->graph.posesId[i]);
					return;
				}
			}
		}

		std::map<int, Transform> poses;
		for(std::map<int, Signature>::iterator iter=nodeInfos.begin(); iter!=nodeInfos.end(); ++iter)
		{
			poses.insert(std::make_pair(iter->first, iter->second.getPose()));
			//landmarks<<"SensorData is: "<<iter->second.sensorData().id()<<","<<iter->second.sensorData().stamp()<<","<<iter->second.sensorData().userDataRaw().empty()<<std::endl;
			}
        
		Transform landmarkPose, currentRobotPose;
		//landmarks<<constraints<<std::endl;
		// Optimize only if there is a subscriber
		if(mapDataPub_.getNumSubscribers() || mapGraphPub_.getNumSubscribers())
		{
			ROS_INFO_STREAM("Optimization enableddd!!!!!");
			UTimer timer;
			std::map<int, Transform> optimizedPoses, landmarkOptimizedPoses, robotOptimizedPoses;
			Transform mapCorrection = Transform::getIdentity();
			Transform landmarkCorrection = Transform::getIdentity();
			std::map<int, rtabmap::Transform> posesOut, posesOutNoLandmarks, posesOutLandmarks;
			std::multimap<int, rtabmap::Link> linksOut;
			if(poses.size() > 1 && constraints.size() > 0)
			{
				int fromId = optimizeFromLastNode_?poses.rbegin()->first:poses.begin()->first;
				optimizer_->getConnectedGraph(
						fromId,
						poses,
						constraints,
						posesOut,
						linksOut);

				posesOutNoLandmarks = posesOut;
				ROS_INFO_STREAM("connected graph computed");
				//Dina Youakim: since landmark nodes IDs are dynamic and change everytime given the max ID we have in the robot pose nodes
				//we need to compute this max and start creating landmark nodes from max+1
				initializeLandmarkCounter(modifiedMapData);
				ROS_INFO_STREAM("counter initialized");
				//Dina Youakim: before qdding new landmarks; process old ones from the cache and add them to poses and links with new IDs 
				//given the max used IDs for the normal nodes
				fillOldLandmarks(modifiedMapData,posesOutLandmarks, linksOut);
				ROS_INFO_STREAM("Old Landmarks filled");
				//Dina Youakim create new landmarks here add to poses and links
				createLandmarks(modifiedMapData, posesOutLandmarks,linksOut);
				ROS_INFO_STREAM("New Landmarks created");
				posesOut.insert(posesOutLandmarks.begin(), posesOutLandmarks.end());
				optimizedPoses = optimizer_->optimizeWithLandmarks(fromId, posesOut, linksOut);//optimizeWithLandmarks
				/*for (std::map<int, Transform>::iterator iter = posesOutNoLandmarks.begin(); iter != posesOutNoLandmarks.end(); ++iter)
				{
					if(optimizedPoses.find(iter->first))
						robotOptimizedPoses.insert(std::make_pair<int,Transform>(iter->first,iter->second));
				}*/
				//landmarks<<"optimization result:"<<std::endl;
				
				for (std::map<int, Transform>::iterator iter = optimizedPoses.begin(); iter != optimizedPoses.end(); ++iter)
				{
					landmarks<<iter->first<<","<<iter->second.x()<<","<<iter->second.y()<<","<<iter->second.theta()<<std::endl;
					if(iter->first == landmarkCounter)
					{
						landmarkPose = iter->second;
						//landmarkPose = optimizedPoses.at(posesOutLandmarks.begin()->first);
						//landmarks<<"last landmark pose chosen: "<<landmarkPose.x()<<","<<landmarkPose.y()<<","<<landmarkPose.theta()<<std::endl;
						break;
					}
				}
				mapToOdomMutex_.lock();

				currentRobotPose = optimizedPoses.at(posesOutNoLandmarks.rbegin()->first);
				landmarks<<currentRobotPose.x()<<","<<currentRobotPose.y()<<","<<currentRobotPose.theta()<<std::endl;
				mapCorrection = currentRobotPose * posesOutNoLandmarks.rbegin()->second.inverse();
				landmarkCorrection = optimizedPoses.at(posesOutLandmarks.rbegin()->first) * posesOutLandmarks.rbegin()->second.inverse();
				landmarkPose.x() -= landmarkCorrection.x();
				landmarkPose.y() -= landmarkCorrection.y();
				//landmarkPose.z() -= landmarkCorrection.z();
				//landmarks<<"Map correction: "<<mapCorrection.x()<<","<<mapCorrection.y()<<","<<mapCorrection.z()<<","<<mapCorrection.theta()<<std::endl;
				//landmarks<<"Landmark correction: "<<landmarkCorrection.x()<<","<<landmarkCorrection.y()<<","<<landmarkCorrection.z()<<","<<landmarkCorrection.theta()<<std::endl;
				mapToOdom_ = mapCorrection;
				mapToOdomMutex_.unlock();
			}
			else if(poses.size() == 1 && constraints.size() == 0)
			{
				optimizedPoses = poses;
			}
			else if(poses.size() || constraints.size())
			{
				ROS_ERROR("map_optimizer: Poses=%d and edges=%d (poses must "
					   "not be null if there are edges, and edges must be null if poses <= 1)",
					  (int)poses.size(), (int)constraints.size());
			}

			rtabmap_ros::MapData outputDataMsg;
			rtabmap_ros::MapGraph outputGraphMsg;
			rtabmap_ros::mapGraphToROS(optimizedPoses,
					linksOut,
					mapCorrection,
					outputGraphMsg);

			if(mapGraphPub_.getNumSubscribers())
			{
				outputGraphMsg.header = modifiedMapData->header;
				mapGraphPub_.publish(outputGraphMsg);
			}

			if(mapDataPub_.getNumSubscribers())
			{
				outputDataMsg.header = modifiedMapData->header;
				outputDataMsg.graph = outputGraphMsg;
				outputDataMsg.nodes = modifiedMapData->nodes;
				if(posesOutNoLandmarks.size() > modifiedMapData->nodes.size())
				{
					std::set<int> addedNodes;
					for(unsigned int i=0; i<modifiedMapData->nodes.size(); ++i)
					{
						addedNodes.insert(modifiedMapData->nodes[i].id);
					}
					std::list<int> toAdd;
					for(std::map<int, Transform>::iterator iter=posesOut.begin(); iter!=posesOut.end(); ++iter)
					{
						if(addedNodes.find(iter->first) == addedNodes.end())
						{
							toAdd.push_back(iter->first);
						}
					}
					if(toAdd.size())
					{
						int oi = outputDataMsg.nodes.size();
						outputDataMsg.nodes.resize(outputDataMsg.nodes.size()+toAdd.size());
						for(std::list<int>::iterator iter=toAdd.begin(); iter!=toAdd.end(); ++iter)
						{
							UASSERT(cachedNodeInfos_.find(*iter) != cachedNodeInfos_.end());
							rtabmap_ros::nodeDataToROS(cachedNodeInfos_.at(*iter), outputDataMsg.nodes[oi]);
							++oi;
						}
					}
				}
				mapDataPub_.publish(outputDataMsg);
			}

			
			/*geometry_msgs::PoseStamped landmarkPoseStamped;
			landmarkPoseStamped.header.stamp = ros::Time::now();
			rtabmap_ros::transformToPoseMsg(landmarkPose,landmarkPoseStamped.pose);    	
    		geometry_msgs::PoseStamped out;
    		out.header.stamp = landmarkPoseStamped.header.stamp;
    		out.header.frame_id = "odom";
    		if(tfListener_.waitForTransform("base_link", "odom",landmarkPoseStamped.header.stamp, ros::Duration(0.1))){
        		tfListener_.transformPose("odom",landmarkPoseStamped,out); 
        		
        	}*/
        	//Transform landmarkTransformToOdom = rtabmap_ros::transformFromPoseMsg(out.pose);

			//landmarkPose = mapCorrection*landmarkPose;
			visualization_msgs::Marker marker;
			marker.header.frame_id = mapFrameId_;
			marker.header.stamp = ros::Time::now();
			marker.ns = "landmark";
			marker.id = landmarkCounter;
			marker.action = visualization_msgs::Marker::ADD;
			marker.pose.position.x = landmarkPose.x();
			marker.pose.position.y = landmarkPose.y();
			marker.pose.position.z = landmarkPose.z();
			geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(landmarkPose.theta());
			marker.pose.orientation = q;
			
			marker.scale.x = 0.08;
			marker.scale.y = 0.08;
			marker.scale.z = 0.08;
			marker.color.a = 1;
			marker.color.r = 1.0;
			marker.color.g = 0.0;
			marker.color.b = 0.0;

			marker.type = visualization_msgs::Marker::CUBE;
			landmarkPosePub_.publish(marker);

			
			visualization_msgs::Marker marker2;
			marker2.header.frame_id = mapFrameId_;
			marker2.header.stamp = ros::Time::now();
			marker2.ns = "robot";
			marker2.id = landmarkCounter;
			marker2.action = visualization_msgs::Marker::ADD;
			marker2.pose.position.x = currentRobotPose.x();
			marker2.pose.position.y = currentRobotPose.y();
			marker2.pose.position.z = currentRobotPose.z();
			geometry_msgs::Quaternion q2 = tf::createQuaternionMsgFromYaw(currentRobotPose.theta());
			marker2.pose.orientation = q2;
			
			marker2.scale.x = 0.08;
			marker2.scale.y = 0.08;
			marker2.scale.z = 0.08;
			marker2.color.a = 1;
			marker2.color.r = 0.0;
			marker2.color.g = 1.0;
			marker2.color.b = 0.0;

			marker2.type = visualization_msgs::Marker::CUBE;
			robotPosePub_.publish(marker2);

			geometry_msgs::TransformStamped msg;
			msg.child_frame_id = "landmark";
			msg.header.frame_id = mapFrameId_;
			msg.header.stamp = ros::Time::now() ;
			rtabmap_ros::transformToGeometryMsg(landmarkPose, msg.transform);
			tfBroadcaster_.sendTransform(msg);

			ROS_INFO_STREAM("Marker published with pose "<<landmarkPose.x()<<","<<landmarkPose.y()<<","<<landmarkPose.theta());
			landmarks<<landmarkPose.x()<<","<<landmarkPose.y()<<","<<landmarkPose.theta()<<std::endl;
			ROS_INFO("Time graph optimization = %f s", timer.ticks());
		}
	}

private:
	std::string mapFrameId_;
	std::string odomFrameId_;
	bool globalOptimization_;
	bool optimizeFromLastNode_;
	graph::Optimizer * optimizer_;

	rtabmap::Transform mapToOdom_;
	boost::mutex mapToOdomMutex_;

	ros::Subscriber mapDataTopic_;

	ros::Publisher mapDataPub_;
	ros::Publisher mapGraphPub_;
	ros::Publisher landmarkPosePub_, robotPosePub_;

	std::multimap<int, Link> cachedConstraints_;
	std::map<int, Signature> cachedNodeInfos_;
	//Dina Youakim: add this map to act like a chache for landmarks observed from previous times
	//As the landmarks are not saved in rtabmap_ros memory; they need to be saved here for next optimization cycles
	//The ID of a landmark is not so representative as it will change every cycle (could not use -ve IDs so every time ze compute the max of nodes IDs and start assigining to
	//landmark starting from max + 1). So the key of the map is the ID of the node to which it is attached and the value is a list of landmark nodes or cv::Mat
	std::map< int,std::vector<rtabmap::Transform> > landmarksCache_;

	tf2_ros::TransformBroadcaster tfBroadcaster_;
	tf::TransformListener tfListener_;

	boost::thread* transformThread_;
	std::ofstream landmarks;
	int landmarkCounter;
};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "map_optimizer");
	MapOptimizer optimizer;
	ros::spin();
	return 0;
}
