/*
 * SpaceSampler.hpp
 *
 *  Created on: Sep 29, 2014
 *      Author: ralfschleicher
 */

#ifndef SPACESAMPLER_HPP_
#define SPACESAMPLER_HPP_

#include "next_best_view/common/CommonClass.hpp"

namespace next_best_view {
	/*!
	 * \brief SpaceSampler class generalizes the sampling of the space.
	 * \author Ralf Schleicher
	 * \date 2014
	 * \version 1.0
	 * \copyright GNU Public License
	 */
	class SpaceSampler : public CommonClass {
	private:
		/*!
		 * \brief sample count
		 */
		uint32_t mSamples;
	public:
		/*!
		 * \brief constructor for SpaceSampler object
		 */
		SpaceSampler();

		/*!
		 * \brief destructor for SpaceSampler object
		 */
		virtual ~SpaceSampler();

		/*!
		 * \return a point cloud containing a set of points in the space.
		 */
		virtual SamplePointCloudPtr getSampledSpacePointCloud(SimpleVector3 currentSpacePosition, float contractor) = 0;

		/*!
		 * \param samples the count of samples to be created
		 */
		void setSamples(const uint32_t &samples);

		/*!
		 * \return count of samples to be created.
		 */
		uint32_t getSamples();
	};

	/*!
	 * \brief Definition for the shared pointer type of the class.
	 */
	typedef boost::shared_ptr<SpaceSampler> SpaceSamplerPtr;
}


#endif /* SPACESAMPLER_HPP_ */
