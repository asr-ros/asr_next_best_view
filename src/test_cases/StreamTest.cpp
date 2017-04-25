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
#include <robot_model_services/robot_model/impl/MILDRobotState.hpp>
#include "next_best_view/rating/impl/DefaultScoreContainer.hpp"
#include <sstream>
#include <string>

using namespace next_best_view;
using namespace boost::unit_test;

class StreamTest : public BaseTest {
public:
    StreamTest() : BaseTest(false, true) {}

    virtual ~StreamTest() {}

    void testString(std::string stringToTest, std::vector<std::string> occurrencesInString) {
        for (auto occurrence : occurrencesInString) {
            BOOST_CHECK_MESSAGE(stringToTest.find(occurrence) != std::string::npos, "attribute " + occurrence + " not found");
        }
    }

    void iterationTest() {
        ObjectPoint objectPoint = ObjectPoint(SimpleVector3(1, 2, 3));
        SamplePoint samplePoint = SamplePoint(SimpleVector3(1, 2, 3));
        ViewportPoint viewportPoint = ViewportPoint();
        robot_model_services::MILDRobotState mildRobotState = robot_model_services::MILDRobotState(1.0, 2.0, 3.0, 4.0, 5.0);
        DefaultScoreContainer defaultScoreContainer = DefaultScoreContainer();
        defaultScoreContainer.setUnweightedInverseMovementCostsBaseRotation(1.7);
        defaultScoreContainer.setUnweightedUnnormalizedObjectUtilitiy("PlateDeep", 0.1);
        defaultScoreContainer.setUtilityNormalization(42);

        std::stringstream objectPointSs;
        std::stringstream samplePointSs;
        std::stringstream viewportPointSs;
        std::stringstream mildRobotStateSs;
        std::stringstream defaultScoreContainerSs;

        objectPointSs << objectPoint;
        samplePointSs << samplePoint;
        viewportPointSs <<viewportPoint;
        mildRobotStateSs << mildRobotState;
        defaultScoreContainerSs << defaultScoreContainer;

        std::string objectPointStr = objectPointSs.str();
        std::string samplePointStr = samplePointSs.str();
        std::string viewportPointStr = viewportPointSs.str();
        std::string mildRobotStateStr = mildRobotStateSs.str();
        std::string defaultScoreContainerStr = defaultScoreContainerSs.str();

        std::vector<std::string> objectPointAttrs = {
            "x: 1",
            "w:",
            "r:",
            "intermediate_object_weight:"
        };

        std::vector<std::string>  samplePointAttrs = {
            "x: 1",
            "child_indices:"
        };

        std::vector<std::string>  viewportPointAttrs = {
            "x: 0",
            "w: 0",
            "score:",
            "object_type_set:"
        };

        std::vector<std::string>  mildRobotStateAttrs = {
            "pan:",
            "tilt:",
            "x:",
            "rotation:"
        };

        std::vector<std::string>  defaultScoreContainerAttrs = {
            "utilityNormalization: 42",
            "unweightedInverseMovementCostsBaseRotation: 1.7",
            "objectUtility:",
            "weightedInverseCosts:",
            "objectUtility:",
            "PlateDeep -> 0.1"
        };

        testString(objectPointStr, objectPointAttrs);
        testString(samplePointStr, samplePointAttrs);
        testString(viewportPointStr, viewportPointAttrs);
        testString(mildRobotStateStr, mildRobotStateAttrs);
        testString(defaultScoreContainerStr, defaultScoreContainerAttrs);

        std::cout << "ObjectPoint: " << std::endl << objectPoint << std::endl;
        std::cout << "SamplePoint: " << std::endl << samplePoint << std::endl;
        std::cout << "ViewportPoint: " << std::endl << viewportPoint << std::endl;
        std::cout << "MILDRobotState: " << std::endl << mildRobotState << std::endl;
        std::cout << "DefaultScoreContainer: " << std::endl << defaultScoreContainer << std::endl;
    }
};

test_suite* init_unit_test_suite( int argc, char* argv[] ) {
    ros::init(argc, argv, "nbv_test");
    ros::start();

    test_suite* evaluation = BOOST_TEST_SUITE("Evaluation NBV");

    boost::shared_ptr<StreamTest> testPtr(new StreamTest());

    evaluation->add(BOOST_CLASS_TEST_CASE(&StreamTest::iterationTest, testPtr));

    framework::master_test_suite().add(evaluation);

    return 0;
}
