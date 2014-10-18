/*
 * Helper.hpp
 *
 *  Created on: Aug 12, 2014
 *      Author: ralfschleicher
 */

#ifndef MARKER_HELPER_HPP_
#define MARKER_HELPER_HPP_

#include "typedef.hpp"
#include <visualization_msgs/Marker.h>

namespace next_best_view {
	/*!
	 * \brief MarkerHelper is a convenience class to reduce the mess created if marker initialization is used by hand.
	 * \author Ralf Schleicher
	 * \date 2014
	 * \version 1.0
	 * \copyright GNU Public License
	 */
	class MarkerHelper {
	public:
		/*!
		 * \param vector a 3d vector
		 * \return the corresponding geometry_msgs::Point object
		 */
		static geometry_msgs::Point getPoint(SimpleVector3 vector);

		/*!
		 * \param id the id of the marker
		 * \return a marker with all basic settings
		 */
		static visualization_msgs::Marker getBasicMarker(int id);

		/*!
		 * \param id the id of the marker
		 * \return a marker which will delete the corresponding marker with the same id
		 */
		static visualization_msgs::Marker getDeleteMarker(int id);

		/*!
		 * \param id the id of the marker
		 * \param mesh_resource a uri to the mesh's resource
		 * \param centroid the centroid of the mesh object
		 * \param quaternion the orientation of the mesh object
		 * \return a marker with all settings for a visualization of a mesh
		 */
		static visualization_msgs::Marker getMeshMarker(int id, std::string mesh_resource, SimpleVector3 centroid, SimpleQuaternion quaternion);

		/*!
		 * \param id the id of the marker
		 * \param mesh_resource a uri to the mesh's resource
		 * \param pose the pose of the mesh object
		 * \return a marker with all settings for a visualization of a mesh
		 */
		static visualization_msgs::Marker getMeshMarker(int id, std::string mesh_resource, geometry_msgs::Pose pose);

		/*!
		 * \param id the id of the marker
		 * \param startPoint the 3d point of start
		 * \param endPoint the 3d point of end
		 * \param color a 4d point which encodes color in RGBA
		 * \return a marker with all settings for a visualization of an arrow
		 */
		static visualization_msgs::Marker getArrowMarker(int id, SimpleVector3 startPoint, SimpleVector3 endPoint, SimpleVector4 color = SimpleVector4(1.0, 0.0, 0.0, 1.0));

		/*!
		 * \param marker [in / out] the marker which color gets set
		 * \param x a value between 0 and 1
		 * \param alpha the alpha value between 0 and 1
		 */
		static void getRainbowColor(visualization_msgs::Marker &marker, double x, double alpha = 1.0);
	};
}


#endif /* MARKER_HELPER_HPP_ */
