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
