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
         * \param sourcePosition [in] the MILD's current position
         * \param targetPosition [in] the MILD's target position
         * \param sourceRotationBase [in] the MILD's current rotation
         * \param targetRotationBase [in] the MILD's target rotation
         * \return the rating angle (between 0.0 and 1.0)
         */
        virtual double getPanAngleRating(const geometry_msgs::Point &sourcePosition, const geometry_msgs::Point &targetPosition, double sourceRotationBase, double targetRotationBase) = 0;
    };

    /*!
     * \brief Definition for the shared pointer type of the class.
     */
    typedef boost::shared_ptr<IKRatingModule> IKRatingModulePtr;
}

#endif /* IKRATINGMODULE_H */
