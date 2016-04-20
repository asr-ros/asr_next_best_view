/*
 * MapTranslator.hpp
 *
 *  Created on: Oct 15, 2014
 *      Author: ralfschleicher
 */

#pragma once

#include <ros/ros.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GetPlan.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>
#include <vector>
#include <boost/tuple/tuple.hpp>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf/transform_listener.h>
#include "next_best_view/helper/DebugHelper.hpp"
#include "next_best_view/helper/MathHelper.hpp"
#include "next_best_view/helper/TypeHelper.hpp"
#include "typedef.hpp"

namespace next_best_view {
	struct RayTracingIndex {
		int32_t x;
		int32_t y;
		int8_t occupancy;
	public:
		static bool topologicalCompare(const RayTracingIndex &source, const RayTracingIndex &firstIndex, const RayTracingIndex &secondIndex) {
			SimpleVector2 sourceVector(source.x, source.y);
			SimpleVector2 firstIndexVector(firstIndex.x, firstIndex.y);
			SimpleVector2 secondIndexVector(secondIndex.x, secondIndex.y);

			return (firstIndexVector - sourceVector).squaredNorm() < (secondIndexVector - sourceVector).squaredNorm();
		}

		static bool equals(const RayTracingIndex &firstIndex, const RayTracingIndex &secondIndex) {
			SimpleVector2 firstIndexVector(firstIndex.x, firstIndex.y);
			SimpleVector2 secondIndexVector(secondIndex.x, secondIndex.y);
			return firstIndexVector == secondIndexVector;
		}
	};
	typedef boost::shared_ptr<RayTracingIndex> RayTracingIndexPtr;
	typedef std::vector<RayTracingIndex> Ray;


	class MapHelper {
	private:
        DebugHelperPtr mDebugHelperPtr;

        bool mMapReceived;
		int8_t mCollisionThreshold;
		ros::NodeHandle mGlobalNodeHandle;
		ros::ServiceClient mGetPlanServiceClient;
		nav_msgs::OccupancyGrid mMap;
        costmap_2d::Costmap2D mCostmap;
        nav_msgs::OccupancyGrid mRaytracingMap;
        // table to translate from values 0-255 in Costmap2D to values -1 to 100 in OccupancyGrid
        int8_t mCostTranslationTable[256];
	public:
        MapHelper(const std::string &mapTopicName = "map", const std::string &getPlanServiceName = "move_base/make_plan") : mMapReceived(false), mCollisionThreshold(45) {
            mDebugHelperPtr = DebugHelper::getInstance();

            // set cost translation table
            mCostTranslationTable[0] = 0;  // NO obstacle
            mCostTranslationTable[253] = 99;  // INSCRIBED obstacle
            mCostTranslationTable[254] = 100;  // LETHAL obstacle
            mCostTranslationTable[255] = -1;  // UNKNOWN

            for (int i = 1; i < 253; i++) {
                mCostTranslationTable[i] = int8_t(1 + (97 * (i - 1)) / 251);
            }

            ros::Subscriber mapSubscriber = mGlobalNodeHandle.subscribe<nav_msgs::OccupancyGrid>(mapTopicName, 1, &MapHelper::mapReceived, this);
            while(ros::ok() && !mMapReceived) {
                mDebugHelperPtr->write(std::stringstream() << "Waiting for map to arrive on topic '" << mapSubscriber.getTopic() << "'",
                            DebugHelper::MAP);

				ros::spinOnce();
				ros::Duration(0.5).sleep();
			}

            this->setCostmap();

			mGetPlanServiceClient = mGlobalNodeHandle.serviceClient<nav_msgs::GetPlan>(getPlanServiceName, true);
			while(ros::ok() && !mGetPlanServiceClient.exists()) {
                mDebugHelperPtr->write(std::stringstream() << "Waiting for planning server to start on service '"
                                        << mGetPlanServiceClient.getService(), DebugHelper::MAP);
				ros::spinOnce();
				ros::Duration(0.5).sleep();
			}

            this->aggregateRaytracingMap();
		}

		virtual ~MapHelper() { }
	private:
		void mapReceived(nav_msgs::OccupancyGrid map) {
            mDebugHelperPtr->write("Map received", DebugHelper::MAP);
			mMap = map;
			mMapReceived = true;
		}

        void setCostmap() {
            tf::TransformListener tf(ros::Duration(10));
            std::string name = "global_costmap";
            costmap_2d::Costmap2DROS costmapRos(name, tf);
            costmapRos.start();
            costmapRos.stop();

            mCostmap = *(costmapRos.getCostmap());

            // get output path
            std::string path;
            mGlobalNodeHandle.getParam("/nbv/mMapsImagePath", path);

            // output costmap
            std::string costmapPath = path + "/costmap.pgm";
            mDebugHelperPtr->write("Outputting calculated costmap to " + costmapPath,
                        DebugHelper::MAP);
            mCostmap.saveMap(costmapPath);
        }

        void aggregateRaytracingMap() {
            mDebugHelperPtr->write("Aggregating raytracing map.", DebugHelper::MAP);
            if (mMap.info.width != mCostmap.getSizeInCellsX() || mMap.info.height != mCostmap.getSizeInCellsY()) {
				ROS_ERROR("Cannot aggregate raytracing map. Dimensions of map and costmap do not match!");
                assert(mMap.info.width == mCostmap.getSizeInCellsX() && mMap.info.height == mCostmap.getSizeInCellsY());
            }

            mRaytracingMap.info = mMap.info;
            mRaytracingMap.data.reserve(mMap.data.size());
            for (unsigned int y = 0; y < mMap.info.height; y++) {
                for (unsigned int x = 0; x < mMap.info.width; x++) {
                    unsigned int index = y * mMap.info.width + x;
                    int8_t mapOccupancyValue = mMap.data[index];
                    int8_t costmapOccupancyValue = mCostTranslationTable[mCostmap.getCost(x, y)];
                    int8_t aggregatedOccupancyValue = mapOccupancyValue == -1 ? -1 : costmapOccupancyValue;

                    mRaytracingMap.data[index] = aggregatedOccupancyValue;
                }
            }
            mDebugHelperPtr->write("Aggregation done.", DebugHelper::MAP);
        }

		int8_t getOccupancyValue(const nav_msgs::OccupancyGrid &map, const SimpleVector3 &position) {
			int32_t mapX, mapY;
			this->worldToMapCoordinates(position, mapX, mapY);
			return this->getOccupancyValue(map, mapX, mapY);
		}

        int8_t getOccupancyValue(const costmap_2d::Costmap2D &costmap, const SimpleVector3 &position) {
            int32_t mapX, mapY;
            this->worldToMapCoordinates(position, mapX, mapY);
            return this->getOccupancyValue(costmap, mapX, mapY);
        }

		int8_t getOccupancyValue(const nav_msgs::OccupancyGrid &map, const int32_t &mapX, const int32_t &mapY) {
			uint32_t width = map.info.width;
			uint32_t height = map.info.height;

            if (mapX < 0 || mapY < 0 || mapX >= (int32_t)width || mapY >= (int32_t)height) {
				return -1;
			}

			return map.data[mapX + mapY * width];
		}

        int8_t getOccupancyValue(const costmap_2d::Costmap2D &costmap, const int32_t &mapX, const int32_t &mapY) {
            uint32_t width = costmap.getSizeInCellsX();
            uint32_t height = costmap.getSizeInCellsY();

            if (mapX < 0 || mapY < 0 || mapX >= (int32_t)width || mapY >= (int32_t)height) {
                return -1;
            }

            unsigned char cost = costmap.getCost(mapX, mapY);
            return mCostTranslationTable[cost];
        }

	public:
		int8_t getMapOccupancyValue(const SimpleVector3 &position) {
			return this->getOccupancyValue(mMap, position);
		}

		int8_t getMapOccupancyValue(const int32_t &mapX, const int32_t &mapY) {
			return this->getOccupancyValue(mMap, mapX, mapY);
		}


		int8_t getCostmapOccupancyValue(const SimpleVector3 &position) {
			return this->getOccupancyValue(mCostmap, position);
		}

		int8_t getCostmapOccupancyValue(const int32_t &mapX, const int32_t &mapY) {
			return this->getOccupancyValue(mCostmap, mapX, mapY);
		}


		int8_t getRaytracingMapOccupancyValue(const SimpleVector3 &position) {
			return this->getOccupancyValue(mRaytracingMap, position);
		}

		int8_t getRaytracingMapOccupancyValue(const int32_t &mapX, const int32_t &mapY) {
			return this->getOccupancyValue(mRaytracingMap, mapX, mapY);
		}

		bool doRaytracing(const SimpleVector3 &fromPoint, const SimpleVector3 &toPoint) {
			Ray ray;
			bool retVal = this->doRaytracing(fromPoint, toPoint, ray);
			return retVal;
		}

		bool doRaytracing(const SimpleVector3 &fromPoint, const SimpleVector3 &toPoint, Ray &ray) {
			// clear ray
			ray.clear();

			// get the discrete coordinates from world coordinates
			int32_t fromMapX, fromMapY;
			this->worldToMapCoordinates(fromPoint, fromMapX, fromMapY);

			int32_t toMapX, toMapY;
			this->worldToMapCoordinates(toPoint, toMapX, toMapY);

			// calculate the distances
			int32_t distanceX = toMapX - fromMapX;
			int32_t distanceY = toMapY - fromMapY;

			// return the result of a zero length raytracing
			if (distanceX == 0 && distanceY == 0) {
				int8_t occupancyValue = this->getRaytracingMapOccupancyValue(fromMapX, fromMapY);
				return this->isOccupancyValueAcceptable(occupancyValue);
			}

			// get the bounds - needed to prevent non feasible results
			int32_t minX = std::min(fromMapX, toMapX);
			int32_t maxX = std::max(fromMapX, toMapX);
			int32_t minY = std::min(fromMapY, toMapY);
			int32_t maxY = std::max(fromMapY, toMapY);

			// get the resolution
			float resolution = mRaytracingMap.info.resolution;

			// get the signum of the distance
			int32_t signumX = distanceX < 0 ? - 1 : 1;
			int32_t signumY = distanceY < 0 ? - 1 : 1;

			// calculate the difference between the two points
			SimpleVector2 diff = (toPoint - fromPoint).block<2, 1>(0, 0);

			// contains the value about success of raytracing
			bool rayTracingSucceeded = true;

			// tracing along the x-axis
			for (int32_t offsetX = 0; offsetX <= abs(distanceX) + 1 && diff[0] != 0; offsetX += 1) {
				double x = (fromMapX + signumX * offsetX) * resolution;
				double t = (x - fromPoint[0]) / diff[0];
				double y = fromPoint[1] + t * diff[1];

				RayTracingIndex idx;
				idx.x = (int32_t) (x / resolution);
				idx.y = (int32_t) (y / resolution);
				idx.occupancy = this->getRaytracingMapOccupancyValue(idx.x, idx.y);

				if (idx.x < minX || idx.x > maxX || idx.y < minY || idx.y > maxY) {
					continue;
				}

				if (!this->isOccupancyValueAcceptable(idx.occupancy)) {
					rayTracingSucceeded = false;
				}

				ray.push_back(idx);
			}

			// tracing along the y axis.
			for (int32_t offsetY = 0; offsetY <= abs(distanceY) + 1 && diff[1] != 0; offsetY += 1) {
				double y = (fromMapY + signumY * offsetY) * resolution;
				double t = (y - fromPoint[1]) / diff[1];
				double x = fromPoint[0] + t * diff[0];

				RayTracingIndex idx;
				idx.x = (int32_t) (x / resolution);
				idx.y = (int32_t) (y / resolution);
				idx.occupancy = this->getRaytracingMapOccupancyValue(idx.x, idx.y);

				if (idx.x < minX || idx.x > maxX || idx.y < minY || idx.y > maxY) {
					continue;
				}

				if (!this->isOccupancyValueAcceptable(idx.occupancy)) {
					rayTracingSucceeded = false;
				}

				ray.push_back(idx);
			}

			// do a topological sort by distance to the fromPoint
			RayTracingIndex source;
			source.x = fromMapX;
			source.y = fromMapY;
			std::stable_sort(ray.begin(), ray.end(), boost::bind(&RayTracingIndex::topologicalCompare, source, _1, _2));

			// remove duplicate values
			Ray::iterator endIterator = std::unique(ray.begin(), ray.end(), &RayTracingIndex::equals);
			ray.resize(std::distance(ray.begin(), endIterator));

			return rayTracingSucceeded;
		}

		float getMetricWidth() {
			return mMap.info.width * mMap.info.resolution;
		}

		float getMetricHeight() {
			return mMap.info.height * mMap.info.resolution;
		}

		void worldToMapCoordinates(const SimpleVector3 &position, int32_t &x, int32_t &y) {
			SimpleVector3 translation = this->getTranslation();
			SimpleQuaternion orientation = this->getOrientation();
			SimpleMatrix3 rotationMatrix = orientation.toRotationMatrix();

			// the map is rotated by rotation matrix, for back transform do a transpose.
			SimpleVector3 continuousMapCoords = (rotationMatrix.transpose() * position - translation) / mMap.info.resolution;

			// get the map coords
			x = (int32_t) floor(continuousMapCoords[0]);
			y = (int32_t) floor(continuousMapCoords[1]);
		}

		void mapToWorldCoordinates(const int32_t &x, const int32_t &y, SimpleVector3 &result) {
			SimpleVector3 translation = this->getTranslation();
			SimpleQuaternion orientation = this->getOrientation();
			result = orientation.toRotationMatrix() * (SimpleVector3(x, y, 0.0) + translation) * mMap.info.resolution;
			result += SimpleVector3(mMap.info.resolution / 2.0, mMap.info.resolution / 2.0, 0);
		}

		SimpleVector3 getTranslation() {
			return SimpleVector3(mMap.info.origin.position.x, mMap.info.origin.position.y, mMap.info.origin.position.z);
		}

		SimpleQuaternion getOrientation() {
			return TypeHelper::getSimpleQuaternion(mMap.info.origin.orientation);
		}

		bool isOccupancyValueAcceptable(const int8_t &occupancyValue) {
			return occupancyValue >= 0 && occupancyValue <= mCollisionThreshold;
		}

		void setCollisionThreshold(int8_t thresholdValue) {
			mCollisionThreshold = thresholdValue;
		}

		int8_t getCollisionThreshold () {
			return mCollisionThreshold;
		}

		float getSquaredDistance(const SimpleVector3 &start, const SimpleVector3 &goal) {
			return (goal - start).squaredNorm();
		}
	};

	typedef boost::shared_ptr<MapHelper> MapHelperPtr;
}
