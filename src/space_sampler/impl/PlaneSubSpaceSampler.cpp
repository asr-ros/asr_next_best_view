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
		pcl::getMinMax3D(*this->getObjectPointCloud(), minVector, maxVector);
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

