/*
 * LinearAlgebraHelper.hpp
 *
 *  Created on: Aug 12, 2014
 *      Author: ralfschleicher
 */

#include "next_best_view/helper/MathHelper.hpp"

namespace next_best_view {

	boost::mt19937& MathHelper::getRandomnessGenerator() {
		static boost::mt19937 gen(std::time(0));
		return gen;
	}

	double MathHelper::getSignum(const double &value) {
		return (value == 0 ? 0.0 : (value < 0 ? -1.0 : 1.0));
	};

	SimpleVector3 MathHelper::getProjection(const std::size_t &idx, const SimpleVector3 &X) {
		return SimpleVector3((idx == 0 ? 0 : X[0]), (idx == 1 ? 0 : X[1]), (idx == 2 ? 0 : X[2]));
	}

	Precision MathHelper::getCosinus(const SimpleVector3 &X, const SimpleVector3 &Y) {
		Precision xNorm = X.lpNorm<2>();
		Precision yNorm = Y.lpNorm<2>();

		if (xNorm == 0 || yNorm == 0) {
			return 0.0;
		}
		return X.dot(Y) / xNorm / yNorm;
	}

	Precision MathHelper::getRatingFunction(const Precision &angle, const Precision &angleThreshold) {
		if (angle < angleThreshold) {
			return .5 + .5 * cos(angle * M_PI / angleThreshold);
		}
		return 0.0;
	}

	Precision MathHelper::getMinimumAngleDifference(const Precision &firstAngle, const Precision &secondAngle) {
		Precision angleDiff = fabs(firstAngle - secondAngle);
		return fmin(angleDiff, 2 * M_PI - angleDiff);
	}

	SimpleVector3 MathHelper::getSphericalCoords(const SimpleVector3 &X) {
		SimpleVector3 e1(1, 0, 0);
		SimpleVector3 proj = getProjection(2, X);

		Precision acosPhi = acos(getCosinus(e1, proj));
		acosPhi = getSignum(X[1]) == -1.0 ? 2 * M_PI - acosPhi : acosPhi;

		Precision acosTheta = getSignum(X[2]) * acos(getCosinus(proj, X));

		return SimpleVector3(X.lpNorm<2>(), acosPhi, acosTheta);
	}

	int MathHelper::getRandomInteger(const int &min, const int &max) {
		boost::uniform_int<> uniform(min, max);
		boost::variate_generator<boost::mt19937&, boost::uniform_int<> > random(getRandomnessGenerator(), uniform);
		return random();
	}

	Precision MathHelper::getRandomNumber(const Precision &mean, const Precision &standardDeviation) {
		boost::normal_distribution<Precision> uniform(mean, standardDeviation);
		boost::variate_generator<boost::mt19937&, boost::normal_distribution<Precision> > random(getRandomnessGenerator(), uniform);
		return random();
	}

	SimpleVectorX MathHelper::getRandomVector(const SimpleVectorX &mean, const SimpleVectorX &standardDeviation) {
		SimpleVectorX result(mean.rows());

		for (int i = 0; i < mean.rows(); i++) {
			result[i] = getRandomNumber(mean[i], standardDeviation[i]);
		}
		return result;
	}

	SimpleQuaternion MathHelper::getSimpleQuaternion(const geometry_msgs::Quaternion &quat) {
		SimpleQuaternion result;
		result.w() = quat.w;
		result.x() = quat.x;
		result.y() = quat.y;
		result.z() = quat.z;

		return result;
	}

	SimpleQuaternion MathHelper::getRandomQuaternion() {
		SimpleVector3 randomAngles = getRandomVector(SimpleVector3(0.0, 0.0, 0.0), SimpleVector3(2 * M_PI, 2 * M_PI, 1.0));
		return getQuaternionByAngles(randomAngles[0], randomAngles[1], 0);
	}

	SimpleQuaternion MathHelper::getQuaternionByAngles(const Precision &heading, const Precision &attitude, const Precision &bank) {
		double c1 = cos(heading / 2);
		double c2 = cos(attitude  / 2);
		double c3 = cos(bank / 2);
		double s1 = sin(heading / 2);
		double s2 = sin(attitude / 2);
		double s3 = sin(bank / 2);

		SimpleQuaternion quatHeading(c1, 0.0, 0.0, s1);
		SimpleQuaternion quatAttitude(c2, 0.0, s2, 0.0);
		SimpleQuaternion quatBank(c3, s3, 0.0, 0.0);

		//SimpleQuaternion result(c1 * c2 * c3, c1 * c2 * s3, c1 * s2 * c3, s1 * c2 * c3);
		SimpleQuaternion result = quatHeading * quatAttitude * quatBank;

		return result.normalized();
	}

	SimpleQuaternionCollectionPtr MathHelper::getOrientationsOnUnitSphere(const int &numberOfPoints) {
		SimpleQuaternionCollectionPtr oritentationCollectionPtr(new SimpleQuaternionCollection());

		float dlong = M_PI * (3.0 - sqrt(5.0));
		float dz = 2.0 / numberOfPoints;
		float longitude = 0.0;
		float z = 1.0 - dz / 2.0;

		for (int k = 0; k < numberOfPoints; k += 1) {
			float r = sqrt(1.0 - pow(z, 2));

			SimpleVector3 point;
			point[0] = cos(longitude) * r;
			point[1] = sin(longitude) * r;
			point[2] = z;

			SimpleVector3 sphereCoords = getSphericalCoords(point);
			SimpleQuaternion orientation = MathHelper::getQuaternionByAngles(sphereCoords[1], sphereCoords[2], 0.0);
			oritentationCollectionPtr->push_back(orientation);

			z -= dz;
			longitude += dlong;
			if (z < -1) {
				break;
			}
		}

		return oritentationCollectionPtr;
	}


	double MathHelper::radToDeg(double input) {
		return fmod(input / M_PI * 180.0, 360);
	}

	double MathHelper::degToRad(double input) {
		return fmod(input / 180.0 * M_PI, 2 * M_PI);
	}
}
