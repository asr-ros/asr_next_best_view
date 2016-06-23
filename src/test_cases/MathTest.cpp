
/*
 *  MathTest.cpp
 *
 *  Created on: Jun 15, 2016
 *      Author: Daniel Stroh
 */
#define BOOST_TEST_STATIC_LINK

#include <boost/test/included/unit_test.hpp>
#include "next_best_view/test_cases/BaseTest.h"

using namespace next_best_view;
using namespace boost::unit_test;

class MathTest : public BaseTest {
public:
    MathTest() : BaseTest(false, true) {}

    virtual ~MathTest() {}

    /*!
     * \brief evaluates the correctness of the Sphere To Cartesian and Cartesian To Sphere Methods.
     */
    void evaluateS2CandC2S() {
        ROS_INFO("Running Test for S2C and C2S");

        // tolerance
        double tolerance = 2E-7;

        // Unit Sphere Tests
        int thetaDivisor = 32;
        double thetaStepSize = M_PI / (double) thetaDivisor;
        int phiDivisor = 64;
        double phiStepSize = M_2_PI / (double) phiDivisor;
        for (int thetaFac = 0; thetaFac < thetaDivisor; thetaFac++) {
            double theta = - M_PI_2 + thetaFac * thetaStepSize;
            for (int phiFac = 0; phiFac < phiDivisor; phiFac++) {
                double phi = - M_PI + phiFac * phiStepSize;

                // create coordinates
                SimpleSphereCoordinates scoords(1, theta, phi);
                // convert to cartesian
                SimpleVector3 ccoords = MathHelper::convertS2C(scoords);
                // convert to sphere again
                SimpleSphereCoordinates rescoords = MathHelper::convertC2S(ccoords);
                // convert to cartesian again
                SimpleVector3 reccoords = MathHelper::convertS2C(rescoords);

                double cerror = (ccoords - reccoords).lpNorm<2>();

                BOOST_CHECK_MESSAGE(cerror < tolerance, "error has to be minimal.");
            }
        }
    }

    void iterationTest() {
      evaluateS2CandC2S();
    }
};

test_suite* init_unit_test_suite( int argc, char* argv[] ) {
    test_suite* evaluation = BOOST_TEST_SUITE("Evaluation NBV");

    boost::shared_ptr<MathTest> testPtr(new MathTest());

    evaluation->add(BOOST_CLASS_TEST_CASE(&MathTest::iterationTest, testPtr));

    framework::master_test_suite().add(evaluation);

    return 0;
}
