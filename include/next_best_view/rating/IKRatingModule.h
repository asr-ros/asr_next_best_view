/*
 * IKRatingModule.h
 *
 *  Created on: Dez 23, 2015
 *      Author: florianaumann
 */

#ifndef IKRATINGMODULE_H
#define IKRATINGMODULE_H

#include "next_best_view/common/CommonClass.hpp"
#include "nav_msgs/Path.h"

namespace next_best_view {
    /*!
     * \brief IKRatingModule is a generalization for the rating of the inverse kinematic parameters
     * \author Florian Aumann
     * \date 2015
     * \version 1.0
     * \copyright GNU Public License
     * \sa DefaultIKRatingModule
     */
    class IKRatingModule : public CommonClass {
    public:
        /*!
         * \brief constructor of the IKRatingModule object
         */
        IKRatingModule();

        /*!
         * \brief destructor of the IKRatingModule object.
         */
        virtual ~IKRatingModule();

        /*!
         * \brief
         * \param panJointFrame [in] the current frame of the pan joint
         * \param panAngle [in] the angle of the pan joint for which the rating should be computed
         * \param navigationPath [in] the path to the robot base frame with respect to the current panAngle
         * \return the rating angle (between 0.0 and 1.0)
         */
        virtual double getPanAngleRating(Eigen::Affine3d &panJointFrame, double panAngle, nav_msgs::Path &navigationPath) = 0;
    };

    /*!
     * \brief Definition for the shared pointer type of the class.
     */
    typedef boost::shared_ptr<IKRatingModule> IKRatingModulePtr;
}

#endif /* IKRATINGMODULE_H */
