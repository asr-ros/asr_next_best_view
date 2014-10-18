/*
 * SingleCameraModelFilter.hpp
 *
 *  Created on: Sep 25, 2014
 *      Author: ralfschleicher
 */

#include "next_best_view/camera_model_filter/impl/SingleCameraModelFilter.hpp"
#include "next_best_view/helper/MathHelper.hpp"
#include "next_best_view/helper/MarkerHelper.hpp"

namespace next_best_view {
	SingleCameraModelFilter::SingleCameraModelFilter(const SimpleVector3 pivotPointOffset) : CameraModelFilter(), mPivotPointOffset(pivotPointOffset), mFrustumCullingPtr(new FrustumCulling()) {
	}

	SingleCameraModelFilter::~SingleCameraModelFilter() { }

	void SingleCameraModelFilter::filter(IndicesPtr &indicesPtr) {
		indicesPtr = IndicesPtr(new Indices());

		// get the settings of the pivot point
		SimpleVector3 pivotPointPosition = this->getPivotPointPosition();
		SimpleQuaternion pivotPointOrientation = this->getOrientation();

		// calculate the real position of the camera
		SimpleVector3 cameraPosition = pivotPointPosition + pivotPointOrientation.toRotationMatrix() * mPivotPointOffset;

		// set the settings
		mFrustumCullingPtr->setHorizontalFOV(this->getVerticalFOV());
		mFrustumCullingPtr->setVerticalFOV(this->getHorizontalFOV());
		mFrustumCullingPtr->setNearPlaneDistance(this->getNearClippingPlane());
		mFrustumCullingPtr->setFarPlaneDistance(this->getFarClippingPlane());

		// orientation and position
		mFrustumCullingPtr->setCameraPose(getCameraPoseMatrix(cameraPosition, pivotPointOrientation));
		mFrustumCullingPtr->setIndices(this->getIndices());
		mFrustumCullingPtr->setInputCloud(this->getInputCloud());

		// start filtering.
		mFrustumCullingPtr->filter(*indicesPtr);
	}

	void SingleCameraModelFilter::setPivotPointOffset(const SimpleVector3 &pivotPointOffset) {
		mPivotPointOffset = pivotPointOffset;
	}

	SimpleVector3 SingleCameraModelFilter::getPivotPointOffset() {
		return mPivotPointOffset;
	}

	viz::MarkerArrayPtr SingleCameraModelFilter::getVisualizationMarkerArray(uint32_t &sequence, double lifetime) {
		viz::MarkerArrayPtr markerArrayPtr(new viz::MarkerArray());

		// get the position of the pivot point.
		SimpleVector3 pivotPointPosition = this->getPivotPointPosition();
		// the orientation
		SimpleQuaternion orientation = this->getOrientation();
		SimpleMatrix3 rotationMatrix = orientation.toRotationMatrix();

		// get the camera position
		SimpleVector3 cameraPosition = pivotPointPosition + rotationMatrix * this->getPivotPointOffset();

		// setting the camera params
		double fovx = MathHelper::degToRad(this->getHorizontalFOV());
		double fovy = MathHelper::degToRad(this->getVerticalFOV());
		double ncp = this->getNearClippingPlane();
		double fcp = this->getFarClippingPlane();

		// Mark Frustum
		viz::Marker axisX = MarkerHelper::getArrowMarker(sequence++, cameraPosition, cameraPosition + rotationMatrix.block<3,1>(0, 0));
		MarkerHelper::getRainbowColor(axisX, 0.0, 0.8);
		axisX.lifetime = ros::Duration(lifetime);
		markerArrayPtr->markers.push_back(axisX);

		viz::Marker axisY = MarkerHelper::getArrowMarker(sequence++, cameraPosition, cameraPosition + rotationMatrix.block<3,1>(0, 1));
		MarkerHelper::getRainbowColor(axisY, 2.0 / 6.0, 0.8);
		axisY.lifetime = ros::Duration(lifetime);
		markerArrayPtr->markers.push_back(axisY);

		viz::Marker axisZ = MarkerHelper::getArrowMarker(sequence++, cameraPosition, cameraPosition + rotationMatrix.block<3,1>(0, 2));
		MarkerHelper::getRainbowColor(axisZ, 4.0 / 6.0, 0.8);
		axisZ.lifetime = ros::Duration(lifetime);
		markerArrayPtr->markers.push_back(axisZ);

		Eigen::Matrix<Precision, 3, 4> frustumMatrix;
		frustumMatrix << 	1,					1,					1, 					1,
							tan(fovx / 2.0),	-tan(fovx / 2.0),	-tan(fovx / 2.0),	tan(fovx / 2.0),
							tan(fovy / 2.0),	tan(fovy / 2.0),	-tan(fovy / 2.0),	-tan(fovy / 2.0);

		Eigen::Matrix<Precision, 3, 4> frustumCornerMatrix = rotationMatrix * frustumMatrix;

		Eigen::Matrix<Precision, 3, 8> completeFrustumCornerMatrix;
		completeFrustumCornerMatrix.block<3, 4>(0, 0) = ncp * frustumCornerMatrix;
		completeFrustumCornerMatrix.block<3, 4>(0, 4) = fcp * frustumCornerMatrix;

		// Create the frustum
		int indicesList[] = {0,1,2, 0,2,3, 0,4,7, 0,3,7 ,0,4,5, 0,1,5, 2,6,7, 2,3,7, 2,5,6, 1,2,5, 4,5,7, 5,6,7};
		std::size_t sz = sizeof(indicesList) / sizeof(int);
		std::vector<int> triangleList(indicesList, indicesList + sz);

		viz::Marker triangleListMarker = MarkerHelper::getBasicMarker(sequence++);
		triangleListMarker.type = viz::Marker::TRIANGLE_LIST;
		triangleListMarker.lifetime = ros::Duration(lifetime);
		triangleListMarker.scale.x = triangleListMarker.scale.y = triangleListMarker.scale.z = 1.0;
		triangleListMarker.pose.position = MarkerHelper::getPoint(cameraPosition);
		MarkerHelper::getRainbowColor(triangleListMarker, 1.0 / 6.0, 0.2);

		BOOST_FOREACH(int index, triangleList) {
			SimpleVector3 frustumPoint = completeFrustumCornerMatrix.block<3, 1>(0, index);
			geometry_msgs::Point pt = MarkerHelper::getPoint(frustumPoint);
			triangleListMarker.points.push_back(pt);
		}
		markerArrayPtr->markers.push_back(triangleListMarker);

		// create the line list
		int lineIndicesList[] = {0,1, 0,3, 0,4, 1,2, 1,5, 2,3, 2,6, 3,7, 4,5, 4,7, 5,6, 6,7};
		sz = sizeof(lineIndicesList) / sizeof(int);
		std::vector<int> lineList(lineIndicesList, lineIndicesList + sz);

		viz::Marker lineListMarker = MarkerHelper::getBasicMarker(sequence++);
		lineListMarker.type = viz::Marker::LINE_LIST;
		lineListMarker.lifetime = ros::Duration(lifetime);
		lineListMarker.scale.x = 0.05;
		lineListMarker.pose.position = MarkerHelper::getPoint(cameraPosition);
		MarkerHelper::getRainbowColor(lineListMarker, 1.0 / 6.0, 0.2);

		BOOST_FOREACH(int index, lineList) {
			SimpleVector3 frustumPoint = completeFrustumCornerMatrix.block<3, 1>(0, index);
			geometry_msgs::Point pt = MarkerHelper::getPoint(frustumPoint);
			lineListMarker.points.push_back(pt);
		}
		markerArrayPtr->markers.push_back(lineListMarker);

		if (pivotPointPosition != cameraPosition) {
			viz::Marker offsetArrow = MarkerHelper::getArrowMarker(sequence++, pivotPointPosition, cameraPosition);
			offsetArrow.lifetime = ros::Duration(lifetime);
			MarkerHelper::getRainbowColor(offsetArrow, 1.0 / 6.0, 1.0);
			markerArrayPtr->markers.push_back(offsetArrow);
		}

		return markerArrayPtr;
	}
}

