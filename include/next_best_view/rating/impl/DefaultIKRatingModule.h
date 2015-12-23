/*
 * IKRatingModule.h
 *
 *  Created on: Dez 23, 2015
 *      Author: florianaumann
 */

#ifndef DEFFAULTIKRATINGMODULE_H
#define DEFFAULTIKRATINGMODULE_H

#include "next_best_view/rating/IKRatingModule.h"



namespace next_best_view {
    /*!
     * \brief DefaultIKRatingModule implements the functionlities offered by IKRatingModule.
     * \author Florian Aumann
     * \date 2015
     * \version 1.0
     * \copyright GNU Public License
     * \sa DefaultIKRatingModule
     */
    class DefaultIKRatingModule : public IKRatingModule {
    public:
        /*!
         * \brief constructor of the DefaultIKRatingModule object
         */
        DefaultIKRatingModule();

        /*!
         * \brief destructor of the DefaultIKRatingModule object.
         */
        virtual ~DefaultIKRatingModule();

        /*!
         * \brief
         * \param panJointFrame [in] the current frame of the pan joint
         * \param panAngle [in] the angle of the pan joint for which the rating should be computed
         * \param navigationPath [in] the path to the robot base frame with respect to the current panAngle
         * \return the rating angle (between 0.0 and 1.0)
         */
        double getPanAngleRating(Eigen::Affine3d &panJointFrame, double panAngle, nav_msgs::Path &navigationPath);
    };

    /*!
     * \brief Definition for the shared pointer type of the class.
     */
    typedef boost::shared_ptr<DefaultIKRatingModule> DefaultIKRatingModulePtr;
}

#endif /* DEFFAULTIKRATINGMODULE_H */
