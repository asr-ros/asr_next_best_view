/**

Copyright (c) 2016, Aumann Florian, Borella Jocelyn, Heller Florian, Meißner Pascal, Schleicher Ralf, Stöckle Patrick, Stroh Daniel, Trautmann Jeremias, Walter Milena, Wittenbeck Valerij
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

4. The use is explicitly not permitted to any application which deliberately try to kill or do harm to any living creature.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "next_best_view/helper/TypeHelper.hpp"

namespace next_best_view {
	geometry_msgs::Point TypeHelper::getPointMSG(const SimpleVector3 &vector) {
		geometry_msgs::Point point;

        point.x = vector[0];
		point.y = vector[1];
		point.z = vector[2];

        return point;
	}

    geometry_msgs::Quaternion TypeHelper::getQuaternionMSG(const SimpleQuaternion &quaternion) {
        geometry_msgs::Quaternion result;

        result.w = quaternion.w();
        result.x = quaternion.x();
        result.y = quaternion.y();
        result.z = quaternion.z();

        return result;
    }

	SimpleVector3 TypeHelper::getSimpleVector3(const geometry_msgs::Pose &pose) {
		return TypeHelper::getSimpleVector3(pose.position);
	}

	SimpleVector3 TypeHelper::getSimpleVector3(const geometry_msgs::Point &point) {
		return SimpleVector3(point.x, point.y, point.z);
	}

    SimpleVector3 TypeHelper::getSimpleVector3(const std::vector<double> &vector) {
        return SimpleVector3(vector.at(0), vector.at(1), vector.at(2));
    }

    SimpleVector4 TypeHelper::getSimpleVector4(const std::vector<double> &vector) {
        return SimpleVector4(vector[0], vector[1], vector[2], vector[3]);
    }

    SimpleVector4 TypeHelper::getSimpleVector4(const std_msgs::ColorRGBA &color) {
        return SimpleVector4(color.r, color.g, color.b, color.a);
    }

	SimpleQuaternion TypeHelper::getSimpleQuaternion(const geometry_msgs::Pose &pose) {
		return TypeHelper::getSimpleQuaternion(pose.orientation);
	}

	SimpleQuaternion TypeHelper::getSimpleQuaternion(const geometry_msgs::Quaternion &quaternion) {
        return SimpleQuaternion(quaternion.w, quaternion.x, quaternion.y, quaternion.z);
    }

    SimpleQuaternion TypeHelper::getSimpleQuaternion(const std::vector<double> &vector)
    {
        return SimpleQuaternion(vector.at(0), vector.at(1), vector.at(2), vector.at(3));
    }

    geometry_msgs::Vector3 TypeHelper::getVector3(const SimpleVector3 &vector) {
        geometry_msgs::Vector3 result;

        result.x = vector[0];
        result.y = vector[1];
        result.z = vector[2];

        return result;
    }

    std_msgs::ColorRGBA TypeHelper::getColor(const SimpleVector4 &vector) {
        std_msgs::ColorRGBA result;

        result.r = vector[0];
        result.g = vector[1];
        result.b = vector[2];
        result.a = vector[3];

        return result;
    }

}
