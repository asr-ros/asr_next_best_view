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
		sampledSpacePointCloudPtr->push_back(currentSamplePoint);

		SimpleQuaternionCollectionPtr spherePointsPtr = MathHelper::getOrientationsOnUnitSphere(samples);
		BOOST_FOREACH(SimpleQuaternion orientation, *spherePointsPtr) {
            SimpleVector3 vector3 = orientation.toRotationMatrix() * SimpleVector3::UnitX();

            SimpleVector2 vector2(vector3[0], vector3[1]);
            vector2 *= maximumSpan;
            vector2[0] += currentSpacePosition[0];
            vector2[1] += currentSpacePosition[1];

            SamplePoint samplePoint;
            samplePoint.x = vector2[0];
            samplePoint.y = vector2[1];
            sampledSpacePointCloudPtr->push_back(samplePoint);
		}

		return sampledSpacePointCloudPtr;
	}
}

