/*
 * MILDRobotState.cpp
 *
 *  Created on: Nov 2, 2014
 *      Author: ralfschleicher
 */

#include "next_best_view/robot_model/impl/MILDRobotState.hpp"
#include <ostream>

namespace next_best_view {
    MILDRobotState::MILDRobotState(float pan, float tilt, float rotation, float x, float y): RobotState(), pan(pan), tilt(tilt), rotation(rotation), x(x), y(y) { }
	MILDRobotState::MILDRobotState() : RobotState(), pan(0), tilt(0), rotation(0), x(0), y(0) { }
	MILDRobotState::~MILDRobotState()  { }

    std::ostream& operator<<(std::ostream &strm, const next_best_view::MILDRobotState &r) {
        using std::endl;
        return strm << "pan: " << r.pan << endl
                    << "tilt: " << r.tilt << endl
                    << "rotation: " << r.rotation << endl
                    << "x: " << r.x << endl
                    << "y: " << r.y << endl;
    }

    std::ostream& operator<<(std::ostream &strm, const next_best_view::MILDRobotStatePtr &r) {
        return strm << *r;
    }
}
