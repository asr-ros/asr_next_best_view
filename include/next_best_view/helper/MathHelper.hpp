/*
 * LinearAlgebraHelper.hpp
 *
 *  Created on: Aug 12, 2014
 *      Author: ralfschleicher
 */

#ifndef MATHHELPER_HPP_
#define MATHHELPER_HPP_

#include "typedef.hpp"
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>
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
		 * \return the cosinus between the both vectors.
		 */
		static Precision getCosinus(const SimpleVector3 &X, const SimpleVector3 &Y);

		/*!
		 * \param angle [in] the angle
		 * \param angleThreshold [in] the threshold to which the rating function shall rate.
		 * \return the rating
		 */
		static Precision getRatingFunction(const Precision &angle, const Precision &angleThreshold);

		/*!
		 * \param firstAngle [in] the first angle
		 * \param secondAngle [in] the second angle
		 * \return the minimum angle difference.
		 */
		static Precision getMinimumAngleDifference(const Precision &firstAngle, const Precision &secondAngle);

		/*!
		 * \param X [in] the subjected cartesian coord vector.
		 * \return the spherical coords 3d vector (r, phi, theta)
		 */
		static SimpleVector3 getSphericalCoords(const SimpleVector3 &X);

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
		 * \param quat quaternion geometry_msg
		 * \return simple quaternion
		 */
		static SimpleQuaternion getSimpleQuaternion(const geometry_msgs::Quaternion &quat);

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
	};
}



#endif /* MATHHELPER_HPP_ */
