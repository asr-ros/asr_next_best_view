/**

Copyright (c) 2016, Allgeyer Tobias, Aumann Florian, Borella Jocelyn, Braun Kai, Heller Florian, Hutmacher Robin, Karrenbauer Oliver, Marek Felix, Mayr Matthias, Mehlhaus Jonas, Meißner Pascal, Schleicher Ralf, Stöckle Patrick, Stroh Daniel, Trautmann Jeremias, Walter Milena
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

4. The use is explicitly not permitted to any application which deliberately try to kill or do harm to any living creature.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "next_best_view/helper/MarkerHelper.hpp"

namespace next_best_view {
	MarkerHelper::MarkerHelper() { }

	MarkerHelper::~MarkerHelper() { }

    visualization_msgs::Marker MarkerHelper::getBasicMarker(int id, std::string ns) {
		visualization_msgs::Marker marker;
		marker.header.frame_id = "/map";
		marker.header.stamp = ros::Time();
        marker.lifetime = ros::Duration();
        marker.ns = ns;
		marker.id = id;
		marker.action = visualization_msgs::Marker::ADD;
		return marker;
	}

    visualization_msgs::Marker MarkerHelper::getBasicMarker(int id, SimpleVector3 position, SimpleQuaternion orientation,
                                                SimpleVector3 scale, SimpleVector4 color, std::string ns) {
        visualization_msgs::Marker marker = getBasicMarker(id, ns);

        marker.pose.position = TypeHelper::getPointMSG(position);
        marker.pose.orientation = TypeHelper::getQuaternionMSG(orientation);
        marker.scale = TypeHelper::getVector3(scale);
        marker.color = TypeHelper::getColor(color);

        return marker;
    }

    visualization_msgs::Marker MarkerHelper::getTextMarker(int id, std::string text, geometry_msgs::Pose pose, std::string ns)
    {
        visualization_msgs::Marker textMarker = getBasicMarker(id, ns);
        textMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        textMarker.color.a = 1;
        textMarker.color.r = 1;
        textMarker.color.g = 1;
        textMarker.color.b = 1;
        textMarker.text = text;
        textMarker.pose = pose;
        textMarker.scale.z = 0.25;
        return textMarker;
    }

    visualization_msgs::Marker MarkerHelper::getDeleteMarker(int id, std::string ns) {
        visualization_msgs::Marker marker = getBasicMarker(id, ns);

        marker.action = visualization_msgs::Marker::DELETE;

        return marker;
    }

    visualization_msgs::Marker MarkerHelper::getMeshMarker(int id, std::string meshResource, SimpleVector3 centroid, SimpleQuaternion quaternion,
                                                            SimpleVector3 scale, std::string ns) {
        visualization_msgs::Marker marker = getBasicMarker(id, ns);

        marker.type = visualization_msgs::Marker::MESH_RESOURCE;
        marker.mesh_resource = meshResource;
        marker.mesh_use_embedded_materials = true;
        marker.pose.position = TypeHelper::getPointMSG(centroid);
        marker.pose.orientation = TypeHelper::getQuaternionMSG(quaternion);
        marker.scale = TypeHelper::getVector3(scale);

        return marker;
	}

    visualization_msgs::Marker MarkerHelper::getArrowMarker(int id, SimpleVector3 startPoint, SimpleVector3 endPoint, SimpleVector3 scale,
                                                                SimpleVector4 color, std::string ns) {
        visualization_msgs::Marker lmarker = getBasicMarker(id, ns);

		lmarker.type = visualization_msgs::Marker::ARROW;
		lmarker.pose.position.x = 0;
		lmarker.pose.position.y = 0;
		lmarker.pose.position.z = 0;
		lmarker.pose.orientation.x = 0.0;
		lmarker.pose.orientation.y = 0.0;
		lmarker.pose.orientation.z = 0.0;
		lmarker.pose.orientation.w = 1.0;
		// the model size unit is mm
        lmarker.scale.x = scale[0];
        lmarker.scale.y = scale[1];
        lmarker.scale.z = scale[2];

		lmarker.color.r = color[0];
		lmarker.color.g = color[1];
		lmarker.color.b = color[2];
		lmarker.color.a = color[3];

		lmarker.points.push_back(TypeHelper::getPointMSG(startPoint));
		lmarker.points.push_back(TypeHelper::getPointMSG(endPoint));

		return lmarker;
	}

    visualization_msgs::Marker MarkerHelper::getArrowMarker(int id, SimpleVector3 position, SimpleQuaternion orientation,
                                                                SimpleVector3 scale, SimpleVector4 color, std::string ns) {
        visualization_msgs::Marker marker = getBasicMarker(id, position, orientation, scale, color, ns);

        marker.type = visualization_msgs::Marker::ARROW;

        return marker;
    }

    visualization_msgs::Marker MarkerHelper::getCubeMarker(int id, SimpleVector3 position, SimpleQuaternion orientation,
                                                                SimpleVector3 scale, SimpleVector4 color, std::string ns) {
        visualization_msgs::Marker marker = getBasicMarker(id, position, orientation, scale, color, ns);

        marker.type = visualization_msgs::Marker::CUBE;

        return marker;
    }

    visualization_msgs::Marker MarkerHelper::getSphereMarker(int id, SimpleVector3 position, SimpleVector3 scale,
                                                                SimpleVector4 color, std::string ns) {
        visualization_msgs::Marker marker = getBasicMarker(id, position, SimpleQuaternion(), scale, color, ns);

        marker.type = visualization_msgs::Marker::SPHERE;

        return marker;
    }

    visualization_msgs::Marker MarkerHelper::getLineListMarker(int id, std::vector<SimpleVector3> points, double scale,
                                                                    SimpleVector4 color, std::string ns) {
        visualization_msgs::Marker marker = getBasicMarker(id, ns);

        marker.type = visualization_msgs::Marker::LINE_LIST;

        BOOST_FOREACH(SimpleVector3 point, points) {
            marker.points.push_back(TypeHelper::getPointMSG(point));
        }

        marker.scale.x = scale;
        marker.color = TypeHelper::getColor(color);

        return marker;
    }

    visualization_msgs::Marker MarkerHelper::getCylinderMarker(int id, SimpleVector3 position, double w, SimpleVector3 scale,
                                                                    SimpleVector4 color, std::string ns) {
        visualization_msgs::Marker marker = getBasicMarker(id, position, SimpleQuaternion(0, 0, 0, w), scale, color, ns);

        marker.type = visualization_msgs::Marker::CYLINDER;

        return marker;
    }

    visualization_msgs::Marker MarkerHelper::getTriangleListMarker(int id, std::vector<SimpleVector3> vertices,
                                                                    std::vector<SimpleVector4> colors, string ns)
    {
        visualization_msgs::Marker marker = getBasicMarker(id, SimpleVector3(0,0,0), SimpleQuaternion(1,0,0,0),
                                                                SimpleVector3(1,1,1), SimpleVector4(0,0,0,1), ns);

        marker.type = visualization_msgs::Marker::TRIANGLE_LIST;

        for (unsigned int i = 0; i < vertices.size(); i++)
        {
            marker.points.push_back(TypeHelper::getPointMSG(vertices[i]));
            marker.colors.push_back(TypeHelper::getColor(colors[i]));
        }

        return marker;
    }

	void MarkerHelper::getRainbowColor(visualization_msgs::Marker &marker, double x, double alpha) {
        // clamping value to [0.0, 1.0):125
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

