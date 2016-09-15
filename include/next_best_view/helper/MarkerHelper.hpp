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

#ifndef MARKER_HELPER_HPP_
#define MARKER_HELPER_HPP_

#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include "typedef.hpp"
#include "next_best_view/helper/TypeHelper.hpp"

namespace next_best_view {
	/*!
	 * \brief MarkerHelper is a convenience class to reduce the mess created if marker initialization is used by hand.
	 * \author Ralf Schleicher
	 * \date 2014
	 * \version 1.0
	 * \copyright GNU Public License
	 */
    class MarkerHelper {
	private:
		MarkerHelper();
		~MarkerHelper();
	public:
		/*!
         * \brief returns a marker with basic settings
         * \param id the id of the marker
         * \param ns the namespace of the marker
         * \return a basic marker with the given settings
		 */
        static visualization_msgs::Marker getBasicMarker(int id, std::string ns = "my_namespace");

        /*!
         * \brief returns a marker with basic settings
         * \param id the id of the marker
         * \param position the position of the marker
         * \param orientation the orientation of the marker
         * \param scale the scale of the marker
         * \param color the color of the marker
         * \param ns the namespace of the marker
         * \return a basic marker with the given settings
         */
        static visualization_msgs::Marker getBasicMarker(int id, SimpleVector3 position, SimpleQuaternion orientation,
                                                            SimpleVector3 scale, SimpleVector4 color, std::string ns = "my_namespace");

        /*!
         * \brief returns a text marker with the given settings
         * \param id the id of the marker
         * \param text the text of the marker
         * \param pose the pose of the marker
         * \param ns the namespace of the marker
         * \return a text marker with the given settings
         */
        static visualization_msgs::Marker getTextMarker(int id, std::string text, geometry_msgs::Pose pose, std::string ns = "my_namespace");

		/*!
         * \brief returns a marker which will delete the corresponding marker with the same id and namespace
         * \param id the id of the marker
         * \param ns the namespace of the marker
         * \return a marker which will delete the corresponding marker with the same id and namespace
		 */
        static visualization_msgs::Marker getDeleteMarker(int id, std::string ns = "my_namespace");

		/*!
         * \brief returns a mesh marker with the given settings
         * \param id the id of the marker
         * \param meshResource a uri to the mesh's resource
		 * \param centroid the centroid of the mesh object
		 * \param quaternion the orientation of the mesh object
         * \param ns the namespace of the marker
         * \return a mesh marker with the given settings
		 */
        static visualization_msgs::Marker getMeshMarker(int id, std::string meshResource, SimpleVector3 centroid, SimpleQuaternion quaternion,
                                                            SimpleVector3 scale = SimpleVector3(0.001, 0.001, 0.001), std::string ns = "my_namespace");

		/*!
         * \brief returns an arrow marker with the given settings
         * \param id the id of the marker
         * \param startPoint the point of start
         * \param endPoint the point of end
		 * \param color a 4d point which encodes color in RGBA
         * \param ns the namespace of the marker
         * \return an arrow marker with the given settings
		 */
        static visualization_msgs::Marker getArrowMarker(int id, SimpleVector3 startPoint, SimpleVector3 endPoint,
                                                            SimpleVector3 scale = SimpleVector3(0.025, 0.05, 0.05),
                                                            SimpleVector4 color = SimpleVector4(1.0, 0.0, 0.0, 1.0),
                                                            std::string ns = "my_namespace");

        /*!
         * \brief returns an arrow marker with the given settings
         * \param id the id of the marker
         * \param position the position of the arrow
         * \param orientation the orientation of the arrow
         * \param scale the scale of the arrow
         * \param color the color of the arrow
         * \param ns the namespace of the marker
         * \return an arrow marker with the given settings
         */
        static visualization_msgs::Marker getArrowMarker(int id, SimpleVector3 position, SimpleQuaternion orientation,
                                                            SimpleVector3 scale, SimpleVector4 color, std::string ns = "my_namespace");

        /*!
         * \brief returns a cube marker with the given settings
         * \param id the id of the marker
         * \param position the position of the cube
         * \param orientation the orientation of the cube
         * \param scale the scale of the cube
         * \param color the color of the cube
         * \param ns the namespace of the marker
         * \return a cube marker with the given settings
         */
        static visualization_msgs::Marker getCubeMarker(int id, SimpleVector3 position, SimpleQuaternion orientation,
                                                            SimpleVector3 scale, SimpleVector4 color, std::string ns = "my_namespace");

        /*!
         * \brief returns a sphere marker with the given settings
         * \param id the id of the marker
         * \param position the position of the sphere
         * \param scale the scale of the sphere
         * \param color the color of the sphere
         * \param ns the namespace of the sphere
         * \return a sphere marker with the given settings
         */
        static visualization_msgs::Marker getSphereMarker(int id, SimpleVector3 position, SimpleVector3 scale,
                                                            SimpleVector4 color, std::string ns = "my_namespace");

        /*!
         * \brief returns a line list marker with the given settings
         * \param id the id of the marker
         * \param points the points of the line
         * \param scale the scale of the line
         * \param color the color of the line
         * \param ns the namespace of the marker
         * \return a line list marker with the given settings
         */
        static visualization_msgs::Marker getLineListMarker(int id, std::vector<SimpleVector3> points, double scale,
                                                                SimpleVector4 color, std::string ns = "my_namespace");

        /*!
         * \brief returns a cylinder marker with the given settings
         * \param id the id of the marker
         * \param position the position of the cylinder
         * \param w the w-coordinate of the quaternion representing the orientation of the cylinder
         * \param scale the scale of the cylinder
         * \param color the color of the cylinder
         * \param ns the namespace of the marker
         * \return a cylinder marker with the given settings
         */
        static visualization_msgs::Marker getCylinderMarker(int id, SimpleVector3 position, double w, SimpleVector3 scale,
                                                                SimpleVector4 color, std::string ns = "my_namespace");

		/*!
         * \param marker [in / out] the marker whose color gets set
		 * \param x a value between 0 and 1
		 * \param alpha the alpha value between 0 and 1
		 */
		static void getRainbowColor(visualization_msgs::Marker &marker, double x, double alpha = 1.0);
	};
}


#endif /* MARKER_HELPER_HPP_ */
