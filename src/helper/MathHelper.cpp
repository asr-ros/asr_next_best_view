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

#include "next_best_view/helper/MathHelper.hpp"

namespace next_best_view {

	boost::mt19937& MathHelper::getRandomnessGenerator() {
		static boost::mt19937 gen(5);
		return gen;
	}

	SimpleSphereCoordinates MathHelper::convertC2S(const SimpleVector3 &cartesian) {
		SimpleSphereCoordinates ssc;
		convertC2S(cartesian, ssc);
		return ssc;
	}

	void MathHelper::convertC2S(const SimpleVector3 &cartesian, SimpleSphereCoordinates &sphere) {
		sphere[0] = cartesian.lpNorm<2>();
		sphere[1] = asin(cartesian[2]);
		sphere[2] = atan2(cartesian[1], cartesian[0]);
	}

	SimpleVector3 MathHelper::convertS2C(const SimpleSphereCoordinates &sphere) {
		SimpleVector3 cartesian;
		convertS2C(sphere, cartesian);
		return cartesian;
	}

	void MathHelper::convertS2C(const SimpleSphereCoordinates &sphere, SimpleVector3 &cartesian) {
		cartesian[0] = sphere[0] * cos(sphere[1]) * cos(sphere[2]);
		cartesian[1] = sphere[0] * cos(sphere[1]) * sin(sphere[2]);
		cartesian[2] = sphere[0] * sin(sphere[1]);
	}

	SimpleVector3 MathHelper::getVisualAxis(const SimpleQuaternion &orientation) {
		SimpleVector3 resultAxis;
		getVisualAxis(orientation, resultAxis);
		return resultAxis;
	}

	void MathHelper::getVisualAxis(const SimpleQuaternion &orientation, SimpleVector3 &resultAxis) {
		resultAxis = orientation.toRotationMatrix() * SimpleVector3::UnitX();
	}

	double MathHelper::getSignum(const double &value) {
		return (value == 0 ? 0.0 : (value < 0 ? -1.0 : 1.0));
    }

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

    Precision MathHelper::getAngle(const SimpleVector3 &X, const SimpleVector3 &Y) {
        float cosinus = getCosinus(X, Y);
        float angle = acos(cosinus);
        return angle;
    }

	Precision MathHelper::getMinimumAngleDifference(const Precision &firstAngle, const Precision &secondAngle) {
                Precision angleDiff = std::fabs(firstAngle - secondAngle);
		return fmin(angleDiff, 2 * M_PI - angleDiff);
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

		//Comment: Black magic.
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

			SimpleSphereCoordinates sphereCoords = convertC2S(point);
			SimpleQuaternion orientation = MathHelper::getQuaternionByAngles(sphereCoords[2], sphereCoords[1], 0.0);
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

       double MathHelper::getDotProduct(SimpleVector3 v1, SimpleVector3 v2)
	{
		return v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2]; 
	}

    bool MathHelper::vector3Equal(SimpleVector3 v1, SimpleVector3 v2) {
        return std::abs(v1[0] - v2[0]) < 0.0001 &&
                std::abs(v1[1] - v2[1]) < 0.0001 &&
                std::abs(v1[2] - v2[2]) < 0.0001;
    }
}
