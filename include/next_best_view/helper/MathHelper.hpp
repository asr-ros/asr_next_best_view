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

#ifndef MATHHELPER_HPP_
#define MATHHELPER_HPP_

#include "typedef.hpp"
#include <boost/math/special_functions/binomial.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>
#include <ros/ros.h>
#include <set>
#include <vector>
#include <geometry_msgs/Quaternion.h>

namespace next_best_view {
	/*!
	 * \brief MathHelper unites the generally needed math operations.
	 * \author Ralf Schleicher
	 * \date 2014
	 * \version 1.0
	 * \copyright GNU Public License
	 */
	class MathHelper {
	private:
		/*!
		 * \return the randomness generator.
		 */
		static boost::mt19937& getRandomnessGenerator();
	public:
		/*!
		 * \brief converts cartesian coordinates to sphere coordinates
		 * \param cartestian any cartesian coordinates
		 * \return sphere coordinates (radius,inclinate,azimuth)
		 */
		static SimpleSphereCoordinates convertC2S(const SimpleVector3 &cartesian);

		/*!
		 * \brief converts cartesian coordinates to sphere coordinates (lightweight)
		 * \param cartestian any cartesian coordinates
		 * \param sphere (out) sphere coordinates (radius,inclinate,azimuth)
		 */
		static void convertC2S(const SimpleVector3 &cartesian, SimpleSphereCoordinates &sphere);

		/*!
		 * \brief converts sphere coordinates to cartesian coordinates (lightweight)
		 * \param sphere any sphere coordinates (radius,inclinate,azimuth)
		 * \return cartesian coordinates
		 */
		static SimpleVector3 convertS2C(const SimpleSphereCoordinates &sphere);

		/*!
		 * \brief converts sphere coordinates to cartesian coordinates (lightweight)
		 * \param sphere any sphere coordinates (radius,inclinate,azimuth)
		 * \param cartesian (out) cartesian coordinates
		 */
		static void convertS2C(const SimpleSphereCoordinates &sphere, SimpleVector3 &cartesian);

		static SimpleVector3 getVisualAxis(const SimpleQuaternion &orientation);

		static void getVisualAxis(const SimpleQuaternion &orientation, SimpleVector3 &resultAxis);

		/*!
		 * \param value [in]
		 * \return the signum of value
		 */
		static double getSignum(const double &value);

		/*!
		 * \param idx [in] the index to project to 0
		 * \param X [in] the 3d vector used for this action.
		 * \return the 3d vector after projection
		 */
		static SimpleVector3 getProjection(const std::size_t &idx, const SimpleVector3 &X);

		/*!
		 * \param X [in] the first 3d vector
		 * \param Y [in] the second 3d vector
         * \return the cosinus between the two vectors.
		 */
		static Precision getCosinus(const SimpleVector3 &X, const SimpleVector3 &Y);

        /*!
         * \param X [in] the first 3d vector
         * \param Y [in] the second 3d vector
         * \return the angle between the two vectors
         */
        static Precision getAngle(const SimpleVector3 &X, const SimpleVector3 &Y);

        /*!
		 * \param firstAngle [in] the first angle
		 * \param secondAngle [in] the second angle
		 * \return the minimum angle difference.
		 */
		static Precision getMinimumAngleDifference(const Precision &firstAngle, const Precision &secondAngle);

		/*!
		 * \param min [in] the minumum integer
		 * \param max [in] the maximum integer
		 * \return a random integer in the range [min, max]
		 */
		static int getRandomInteger(const int &min, const int &max);

		/*!
		 * \param mean [in] the mean value
		 * \param standardDeviation [in] the standard deviation to use.
		 * \return a normal distributed random number.
		 */
		static Precision getRandomNumber(const Precision &mean, const Precision &standardDeviation);

		/*!
		 * \param mean [in] the mean value in X dimensions
		 * \param standardDeviation [in] the standard deviation to use in X dimensions.
		 * \return a normal distributed random vector.
		 */
		static SimpleVectorX getRandomVector(const SimpleVectorX &mean, const SimpleVectorX &standardDeviation);

		/*!
		 * \return a random rotation quaternion
		 */
		static SimpleQuaternion getRandomQuaternion();

		/*!
		 * \param heading [in] the heading angle
		 * \param attitude [in] the attitude angle
		 * \param bank [in] the bank angle
		 * \return the corresponding quaternion
		 */
		static SimpleQuaternion getQuaternionByAngles(const Precision &heading, const Precision &attitude, const Precision &bank);

		/*!
		 * \param numberOfPoints [in] the number of points to use.
		 * \return an array of orientations.
		 */
		static SimpleQuaternionCollectionPtr getOrientationsOnUnitSphere(const int &numberOfPoints);

		/*!
		 * \param input in radians
		 * \return input in degrees
		 */
		static double radToDeg(double input);

		/*!
		 * \param input in degrees
		 * \return input in radians
		 */
		static double degToRad(double input);

		static double getDotProduct(SimpleVector3 v1, SimpleVector3 v2);

        /**
         * @brief isSubSetOf opposite of isSuperSetOf
         * @param indexSetSub
         * @param indexSetSuper
         * @return
         */
        static bool isSubSetOf(const Indices& indexSetSub, const Indices& indexSetSuper);

        /**
         * @brief isSuperSet
         * @param indexSetSuper
         * @param indexSetSub
         * @return true if indexSetSuper is a superSet of indexSetSub,
         *         false if indexSetSub contains an index that indexSetSuper does not contain.
         */
        static bool isSuperSetOf(const Indices& indexSetSuper, const Indices& indexSetSub);

		template<typename Set> static void printSet(boost::shared_ptr<Set> &setPtr) {
			std::cout << "\t{ ";
			BOOST_FOREACH(typename Set::value_type value, *setPtr) {
				std::cout << value << ", ";
			}
			std::cout << "}" << std::endl;
		}

		template<typename Set> static void printPowerSet(boost::shared_ptr<std::set<boost::shared_ptr<Set> > > &powerSetPtr) {
			std::cout << "{ " << std::endl;
			BOOST_FOREACH(boost::shared_ptr<Set> subSetPtr, *powerSetPtr) {
				printSet(subSetPtr);
			}
			std::cout << "} " << std::endl;
			std::cout << powerSetPtr->size() << " Items" << std::endl;
		}

		template<typename Set> static boost::shared_ptr<std::set<boost::shared_ptr<Set> > > powerSet(const boost::shared_ptr<Set> &setPtr) {
			boost::shared_ptr<std::set<boost::shared_ptr<Set> > > powerSetPtr(new std::set<boost::shared_ptr<Set> >);

			std::size_t value = 0;
			assert(setPtr->size() <= 31);


			std::size_t limit = (1 << setPtr->size());
			for (std::size_t counter = 0; counter < limit; ++counter) {
				boost::shared_ptr<Set> subSetPtr(new Set);
				std::size_t idx = 0;
				for (typename Set::iterator setIter = setPtr->begin(); setIter != setPtr->end();  ++setIter, ++idx) {
					if ( (value & (1 << idx)) != 0 ) {
						subSetPtr->insert(*setIter);
					}
				}
				powerSetPtr->insert(subSetPtr);

				// the value
				++value;
			}

			return powerSetPtr;
		}


		template<typename PowerSet> static boost::shared_ptr<PowerSet> filterCardinalityPowerSet(const boost::shared_ptr<PowerSet> &powerSetPtr, const std::size_t min, const std::size_t max) {
			boost::shared_ptr<PowerSet> resultPowerSetPtr(new PowerSet);

			for (typename PowerSet::iterator powerSetIter = powerSetPtr->begin(); powerSetIter != powerSetPtr->end(); ++powerSetIter) {
				if ((*powerSetIter)->size() < min || (*powerSetIter)->size() > max) {
					continue;
				}

				resultPowerSetPtr->insert(*powerSetIter);
			}

			return resultPowerSetPtr;
		}

		template<typename PowerSet> static boost::shared_ptr<PowerSet> filterCardinalityPowerSet(const boost::shared_ptr<PowerSet> &setPtr, const std::size_t min) {
			return filterCardinalityPowerSet<PowerSet>(setPtr, min, setPtr->size());
		}
	};
}



#endif /* MATHHELPER_HPP_ */
