/*
 * Helper.hpp
 *
 *  Created on: Aug 12, 2014
 *      Author: ralfschleicher
 */

#ifndef MARKER_HELPER_HPP_
#define MARKER_HELPER_HPP_

#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include "typedef.hpp"

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
         * \param id the id of the marker
         * \param ns the namespace of the marker
		 * \return a marker with all basic settings
		 */
        static visualization_msgs::Marker getBasicMarker(int id, std::string ns = "my_namespace");

        static visualization_msgs::Marker getBasicMarker(int id, SimpleVector3 position, SimpleQuaternion orientation,
                                                            SimpleVector3 scale, SimpleVector4 color, std::string ns = "my_namespace");

        /*!
         * \brief getTextMarker
         * \param id the id of the marker
         * \param text the text of the marker
         * \param pose the pose of the marker
         * \param ns the namespace of the marker
         * \return a marker with all basic settings
         */
        static visualization_msgs::Marker getTextMarker(int id, std::string text, geometry_msgs::Pose pose, std::string ns = "my_namespace");

		/*!
         * \param id the id of the marker
         * \param ns the namespace of the marker
		 * \return a marker which will delete the corresponding marker with the same id
		 */
        static visualization_msgs::Marker getDeleteMarker(int id, std::string ns = "my_namespace");

		/*!
		 * \param id the id of the marker
         * \param meshResource a uri to the mesh's resource
		 * \param centroid the centroid of the mesh object
		 * \param quaternion the orientation of the mesh object
         * \param ns the namespace of the marker
		 * \return a marker with all settings for a visualization of a mesh
		 */
        static visualization_msgs::Marker getMeshMarker(int id, std::string meshResource, SimpleVector3 centroid, SimpleQuaternion quaternion,
                                                            SimpleVector3 scale = SimpleVector3(0.001, 0.001, 0.001), std::string ns = "my_namespace");

		/*!
		 * \param id the id of the marker
		 * \param startPoint the 3d point of start
		 * \param endPoint the 3d point of end
		 * \param color a 4d point which encodes color in RGBA
         * \param ns the namespace of the marker
		 * \return a marker with all settings for a visualization of an arrow
		 */
        static visualization_msgs::Marker getArrowMarker(int id, SimpleVector3 startPoint, SimpleVector3 endPoint,
                                                            SimpleVector3 scale = SimpleVector3(0.025, 0.05, 0.05),
                                                            SimpleVector4 color = SimpleVector4(1.0, 0.0, 0.0, 1.0),
                                                            std::string ns = "my_namespace");

        static visualization_msgs::Marker getArrowMarker(int id, SimpleVector3 position, SimpleQuaternion orientation,
                                                            SimpleVector3 scale, SimpleVector4 color, std::string ns = "my_namespace");

        static visualization_msgs::Marker getCubeMarker(int id, SimpleVector3 position, SimpleQuaternion orientation,
                                                            SimpleVector3 scale, SimpleVector4 color, std::string ns = "my_namespace");

        static visualization_msgs::Marker getSphereMarker(int id, SimpleVector3 position, SimpleVector3 scale,
                                                            SimpleVector4 color, std::string ns = "my_namespace");

        static visualization_msgs::Marker getLineListMarker(int id, vector<SimpleVector3> points, double scale,
                                                                SimpleVector4 color, std::string ns = "my_namespace");

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
