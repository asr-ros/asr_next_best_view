#pragma once

#include <ros/ros.h>
#include <string>
#include <sstream>
#include <vector>

namespace next_best_view {

/*!
 * \brief The DebugHelper class is responsible for debug output.
 */
class DebugHelper {

public:

    enum DebugLevel {
        PARAMETERS = 1,
        SERVICE_CALLS = 2,
        VISUALIZATION = 4,
        CALCULATION = 8,
        RATING = 16,
        ROBOT_MODEL = 32,
        MAP = 64,
        FILTER = 128,
        IK_RATING = 256,
        SPACE_SAMPLER = 512,
        HYPOTHESIS_UPDATER = 1024
    };

private:

    static boost::shared_ptr<DebugHelper> instancePtr;
    static const int ALL = PARAMETERS + SERVICE_CALLS + VISUALIZATION + CALCULATION
                            + RATING + ROBOT_MODEL + MAP + FILTER + IK_RATING + SPACE_SAMPLER
                            + HYPOTHESIS_UPDATER;
    static const int NONE = 0;

    ros::NodeHandle mNodeHandle;
    unsigned int mLevels;

public:

    static boost::shared_ptr<DebugHelper> getInstance();

    /*!
     * \brief writes the text to the console if it has a level that allows it
     * \param text the text
     * \param level the debug level of the text
     */
    void write(const char * text, DebugLevel level);

    /*!
     * \brief writes the text to the console if it has a level that allows it
     * \param text the text
     * \param level the debug level of the text
     */
    void write(const std::string &text, DebugLevel level);

    /*!
     * \brief writes the text to the console if it has a level that allows it
     * \param text the text
     * \param level the debug level of the text
     */
    void write(const std::ostream &text, DebugLevel level);

    /*!
     * \brief writes the text noticeably to the console if it has a level that allows it
     * \param text the text
     * \param level the debug level of the text
     */
    void writeNoticeably(const char * text, DebugLevel level);

    /*!
     * \brief writes the text noticeably to the console if it has a level that allows it
     * \param text the text
     * \param level the debug level of the text
     */
    void writeNoticeably(const std::string &text, DebugLevel level);

    /*!
     * \brief writes the text noticeably to the console if it has a level that allows it
     * \param text the text
     * \param level the debug level of the text
     */
    void writeNoticeably(const std::ostream &text, DebugLevel level);

    /*!
     * \brief returns the debug level that is set
     * \return the debug level that is set.
     */
    unsigned int getLevel();

    /*!
     * \brief returns the debug levels that are set as string
     * \return the debug levels that are set as string.
     */
    std::string getLevelString();


    /*!
     * \brief sets the allowed debug levels
     */
    void setLevels();

private:

    DebugHelper();

    /*!
     * \brief parses the level list to the corresponding integer
     * \param levels the list of level strings
     * \return the corresponding integer
     */
    static int parseLevels(std::vector<std::string> levels);

    /*!
     * \brief adds a string to a given string s. Puts a comma between them if the string s has a size bigger than 0.
     * \param s [in,out] the string
     * \param add [in] the string to add
     */
    static void addToString(std::string& s, const std::string& add);

    /*!
     * \brief checks whether the given level is allowed
     * \param level the level
     * \return whether the given level is allowed.
     * true if it is allowed and false if it is not.
     */
    bool checkLevel(DebugLevel level);

};

typedef boost::shared_ptr<DebugHelper> DebugHelperPtr;
}
