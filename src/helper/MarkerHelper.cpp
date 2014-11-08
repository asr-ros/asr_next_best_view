/*
 * MarkerHelper.hpp
 *
 *  Created on: Aug 12, 2014
 *      Author: ralfschleicher
 */

#include "next_best_view/helper/MarkerHelper.hpp"
#include "next_best_view/helper/MathHelper.hpp"
#include "next_best_view/helper/TypeHelper.hpp"

namespace next_best_view {
	MarkerHelper::MarkerHelper() { }

	MarkerHelper::~MarkerHelper() { }

	visualization_msgs::Marker MarkerHelper::getBasicMarker(int id) {
		visualization_msgs::Marker marker;
		marker.header.frame_id = "/map";
		marker.header.stamp = ros::Time();
		marker.ns = "my_namespace";
		marker.id = id;
		marker.action = visualization_msgs::Marker::ADD;

		return marker;
	}

	visualization_msgs::Marker MarkerHelper::getDeleteMarker(int id) {
		visualization_msgs::Marker marker = getBasicMarker(id);
		marker.action = visualization_msgs::Marker::DELETE;

		return marker;
	}

	visualization_msgs::Marker MarkerHelper::getMeshMarker(int id, std::string mesh_resource, SimpleVector3 centroid, SimpleQuaternion quaternion) {
		geometry_msgs::Pose pose;
		pose.orientation.w = quaternion.w();
		pose.orientation.x = quaternion.x();
		pose.orientation.y = quaternion.y();
		pose.orientation.z = quaternion.z();
		pose.position.x = centroid[0];
		pose.position.y = centroid[1];
		pose.position.z = centroid[2];

		return getMeshMarker(id, mesh_resource, pose);
	}

	visualization_msgs::Marker MarkerHelper::getMeshMarker(int id, std::string mesh_resource, geometry_msgs::Pose pose) {
		visualization_msgs::Marker marker = getBasicMarker(id);
		marker.type = visualization_msgs::Marker::MESH_RESOURCE;
		marker.mesh_resource = mesh_resource;
		marker.mesh_use_embedded_materials = true;
		marker.pose = pose;
		marker.scale.x = 0.001;
		marker.scale.y = 0.001;
		marker.scale.z = 0.001;

		return marker;
	}


	visualization_msgs::Marker MarkerHelper::getArrowMarker(int id, SimpleVector3 startPoint, SimpleVector3 endPoint, SimpleVector4 color) {
		visualization_msgs::Marker lmarker = getBasicMarker(id);

		lmarker.type = visualization_msgs::Marker::ARROW;
		lmarker.pose.position.x = 0;
		lmarker.pose.position.y = 0;
		lmarker.pose.position.z = 0;
		lmarker.pose.orientation.x = 0.0;
		lmarker.pose.orientation.y = 0.0;
		lmarker.pose.orientation.z = 0.0;
		lmarker.pose.orientation.w = 1.0;
		// the model size unit is mm
		lmarker.scale.x = .025;
		lmarker.scale.y = .05;
		lmarker.scale.z = .05;

		lmarker.color.r = color[0];
		lmarker.color.g = color[1];
		lmarker.color.b = color[2];
		lmarker.color.a = color[3];

		lmarker.points.push_back(TypeHelper::getGeometryMsgsPoint(startPoint));
		lmarker.points.push_back(TypeHelper::getGeometryMsgsPoint(endPoint));

		return lmarker;
	}

	void MarkerHelper::getRainbowColor(visualization_msgs::Marker &marker, double x, double alpha) {
		// clamping value to [0.0, 1.0)
		if (x >= 1.0) {
			while(x >= 1.0) {
				x -= 1.0;
			}
		} else if (x < 0.0) {
			while(x < 0.0) {
				x += 1.0;
			}
		}

		double red = 0.0;
		double green = 0.0;
		double blue = 0.0;

		x = x * 6.0;
		if (x < 1.0) {
			red = 1.0;
			green = x;
			blue = 0.0;
		} else if (x < 2.0) {
			x -= 1.0;
			red = 1 - x;
			green = 1.0;
			blue = 0.0;
		} else if (x < 3.0) {
			x -= 2.0;
			red = 0.0;
			green = 1.0;
			blue = x;
		} else if (x < 4.0) {
			x -= 3.0;
			red = 0.0;
			green = 1.0 - x;
			blue = 1.0;
		} else if (x < 5.0) {
			x -= 4.0;
			red = x;
			green = 0.0;
			blue = 1.0;
		} else if (x < 6.0) {
			x -= 5.0;
			red = 1.0;
			green = 0.0;
			blue = 1.0 - x;
		}

		marker.color.r = red;
		marker.color.g = green;
		marker.color.b = blue;
		marker.color.a = alpha;
	}
}

