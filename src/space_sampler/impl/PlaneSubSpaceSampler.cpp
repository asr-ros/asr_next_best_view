/*
 * PlaneSubSpaceSampler.cpp
 *
 *  Created on: Sep 29, 2014
 *      Author: ralfschleicher
 */

#include "next_best_view/space_sampler/impl/PlaneSubSpaceSampler.hpp"
#include "next_best_view/helper/MathHelper.hpp"

namespace next_best_view {
	PlaneSubSpaceSampler::PlaneSubSpaceSampler() : SpaceSampler() {

	}

	PlaneSubSpaceSampler::~PlaneSubSpaceSampler() {

	}

	SamplePointCloudPtr PlaneSubSpaceSampler::getSampledSpacePointCloud(SimpleVector3 currentSpacePosition, float contractor) {
		SamplePointCloudPtr sampledSpacePointCloudPtr = SamplePointCloudPtr(new SamplePointCloud());

		// Calculate maximum span
		SimpleVector4 minVector, maxVector;
		pcl::getMinMax3D(*this->getInputCloud(), minVector, maxVector);
		// the surrounding cube.
		SimpleVector3 spanCube = maxVector.block<3,1>(0,0) - minVector.block<3,1>(0,0);

		double maximumSpan = spanCube.lpNorm<Eigen::Infinity>() * contractor;

		uint32_t samples = this->getSamples();

		// add current viewport
		SamplePoint currentSamplePoint;
		currentSamplePoint.x = currentSpacePosition[0];
		currentSamplePoint.y = currentSpacePosition[1];
		currentSamplePoint.z = currentSpacePosition[2];
		sampledSpacePointCloudPtr->push_back(currentSamplePoint);

		SimpleQuaternionCollectionPtr spherePointsPtr = MathHelper::getOrientationsOnUnitSphere(samples);
		BOOST_FOREACH(SimpleQuaternion orientation, *spherePointsPtr) {
			SimpleVector3 vector = maximumSpan * (orientation.toRotationMatrix() * SimpleVector3::UnitX());
			vector[2] = 0.0;
			vector += currentSpacePosition;

			SamplePoint samplePoint;
			samplePoint.x = vector[0];
			samplePoint.y = vector[1];
			samplePoint.z = vector[2];
			sampledSpacePointCloudPtr->push_back(samplePoint);
		}

		return sampledSpacePointCloudPtr;
	}
}

