/**

Copyright (c) 2016, Allgeyer Tobias, Aumann Florian, Borella Jocelyn, Braun Kai, Heller Florian, Hutmacher Robin, Karrenbauer Oliver, Marek Felix, Mayr Matthias, Mehlhaus Jonas, Meißner Pascal, Schleicher Ralf, Stöckle Patrick, Stroh Daniel, Trautmann Jeremias, Walter Milena
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

4. The use is explicitly not permitted to any application which deliberately try to kill or do harm to any living creature.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "next_best_view/helper/DebugHelper.hpp"

namespace next_best_view {

boost::shared_ptr<DebugHelper> DebugHelper::instancePtr;

boost::shared_ptr<DebugHelper> DebugHelper::getInstance() {
    if (!instancePtr)
        instancePtr = boost::shared_ptr<DebugHelper>(new DebugHelper());
    return instancePtr;
}

DebugHelper::DebugHelper() {
    mNodeHandle = ros::NodeHandle("/nbv");
    this->setLevels();
}

void DebugHelper::write(const char *text, DebugHelper::DebugLevel level)
{
    if (this->checkLevel(level)) {
        ROS_DEBUG("%s", text);
    }
}

void DebugHelper::write(const std::string& text, DebugLevel level) {
    const char* cText = text.c_str();

    this->write(cText, level);
}

void DebugHelper::write(const std::ostream& text, DebugLevel level) {
    std::stringstream ss;
    ss << text.rdbuf();
    std::string s = ss.str();

    this->write(s, level);
}

void DebugHelper::writeNoticeably(const char *text, DebugHelper::DebugLevel level)
{
    if (this->checkLevel(level)) {
        ROS_DEBUG(" ");
        ROS_DEBUG("%s", text);
        ROS_DEBUG(" ");
    }
}

void DebugHelper::writeNoticeably(const std::string &text, DebugHelper::DebugLevel level)
{
    const char* cText = text.c_str();

    this->writeNoticeably(cText, level);
}

void DebugHelper::writeNoticeably(const std::ostream &text, DebugHelper::DebugLevel level)
{
    std::stringstream ss;
    ss << text.rdbuf();
    std::string s = ss.str();

    this->writeNoticeably(s, level);
}

unsigned int DebugHelper::getLevel()
{
    return mLevels;
}

std::string DebugHelper::getLevelString()
{
    if (mLevels == ALL)
        return "ALL";
    if (mLevels == NONE)
        return "NONE";

    std::string level = "";

    if (mLevels & PARAMETERS)
        level += "PARAMETERS";
    if (mLevels & SERVICE_CALLS) {
        addToString(level, "SERVICE_CALLS");
    }
    if (mLevels & VISUALIZATION) {
        addToString(level, "VISUALIZATION");
    }
    if (mLevels & CALCULATION) {
        addToString(level, "CALCULATION");
    }
    if (mLevels & RATING) {
        addToString(level, "RATING");
    }
    if (mLevels & ROBOT_MODEL) {
        addToString(level, "ROBOT_MODEL");
    }
    if (mLevels & MAP) {
        addToString(level, "MAP");
    }
    if (mLevels & FILTER) {
        addToString(level, "FILTER");
    }
    if (mLevels & IK_RATING) {
        addToString(level, "IK_RATING");
    }
    if (mLevels & SPACE_SAMPLER) {
        addToString(level, "SPACE_SAMPLER");
    }
    if (mLevels & HYPOTHESIS_UPDATER) {
        addToString(level, "HYPOTHESIS_UPDATER");
    }

    return level;
}

bool DebugHelper::checkLevel(DebugLevel level) {
    return level & mLevels;
}

void DebugHelper::setLevels() {
    // TODO add parameter levels from dyn config
    std::vector<std::string> levels;
    mNodeHandle.param("debugLevels", levels, std::vector<std::string>());
    mLevels = parseLevels(levels);
}

int DebugHelper::parseLevels(std::vector<std::string> levels) {
    if (levels.size() == 0)
        return ALL;
    if (levels.size() == 1) {
        if (levels.at(0).compare("ALL") == 0)
            return ALL;
        if (levels.at(0).compare("NONE") == 0)
            return NONE;
    }

    int level = 0;
    for (unsigned int i = 0; i < levels.size(); i++) {
        if (levels.at(i).compare("PARAMETERS") == 0) {
            level += PARAMETERS;
        }
        else if (levels.at(i).compare("SERVICE_CALLS") == 0) {
            level += SERVICE_CALLS;
        }
        else if (levels.at(i).compare("VISUALIZATION") == 0) {
            level += VISUALIZATION;
        }
        else if (levels.at(i).compare("CALCULATION") == 0) {
            level += CALCULATION;
        }
        else if (levels.at(i).compare("RATING") == 0) {
            level += RATING;
        }
        else if (levels.at(i).compare("ROBOT_MODEL") == 0) {
            level += ROBOT_MODEL;
        }
        else if (levels.at(i).compare("MAP") == 0) {
            level += MAP;
        }
        else if (levels.at(i).compare("FILTER") == 0) {
            level += FILTER;
        }
        else if (levels.at(i).compare("IK_RATING") == 0) {
            level += IK_RATING;
        }
        else if (levels.at(i).compare("SPACE_SAMPLER") == 0) {
            level += SPACE_SAMPLER;
        }
        else if (levels.at(i).compare("HYPOTHESIS_UPDATER") == 0) {
            level += HYPOTHESIS_UPDATER;
        }
        else {
            ROS_ERROR_STREAM("Invalid debug level: " << levels.at(i));
            throw "Invalid debug level";
        }
    }

    return level;
}

void DebugHelper::addToString(std::string& s, const std::string& add)
{
    if (s.size() != 0)
        s += ", ";
    s += add;
}

}
