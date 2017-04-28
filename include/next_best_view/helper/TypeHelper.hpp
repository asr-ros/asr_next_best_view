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

#ifndef TYPEHELPER_HPP_
#define TYPEHELPER_HPP_

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/ColorRGBA.h>
#include <vector>
#include "typedef.hpp"

namespace next_best_view {
	class TypeHelper {
	public:
		static geometry_msgs::Point getPointMSG(const SimpleVector3 &vector);
        static geometry_msgs::Quaternion getQuaternionMSG(const SimpleQuaternion &quaternion);
		static SimpleVector3 getSimpleVector3(const geometry_msgs::Pose &pose);
		static SimpleVector3 getSimpleVector3(const geometry_msgs::Point &point);
        static SimpleVector3 getSimpleVector3(const std::vector<double> &vector);
        static SimpleVector4 getSimpleVector4(const std::vector<double> &vector);
        static SimpleVector4 getSimpleVector4(const std_msgs::ColorRGBA &color);
		static SimpleQuaternion getSimpleQuaternion(const geometry_msgs::Pose &pose);
		static SimpleQuaternion getSimpleQuaternion(const geometry_msgs::Quaternion &quaternion);
        static SimpleQuaternion getSimpleQuaternion(const std::vector<double> &vector);
        static geometry_msgs::Vector3 getVector3(const SimpleVector3 &vector);
        static std_msgs::ColorRGBA getColor(const SimpleVector4 &vector);
	};
}


#endif /* TYPEHELPER_HPP_ */
