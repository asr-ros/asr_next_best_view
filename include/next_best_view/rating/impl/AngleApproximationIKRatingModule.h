/*
 * AngleApproximationIKRatingModule.h
 *
 *  Created on: Jul 05, 2016
 *      Author: florianaumann
 */

#ifndef ANGLEAPPROXIMATIONIKRATINGMODULE_H
#define ANGLEAPPROXIMATIONIKRATINGMODULE_H

#include "next_best_view/rating/IKRatingModule.h"



namespace next_best_view {
    /*!
     * \brief AngleApproximationIKRatingModule uses an approximation of the total angle delta during the mild movement for the rating
     * \author Florian Aumann
     * \date 2016
     * \version 1.0
     * \copyright GNU Public License
     * \sa AngleApproximationIKRatingModule
     */
    class AngleApproximationIKRatingModule : public IKRatingModule {
    public:
        /*!
         * \brief constructor of the AngleApproximationIKRatingModule object
         */
        AngleApproximationIKRatingModule();

        /*!
         * \brief destructor of the AngleApproximationIKRatingModule object.
         */
        virtual ~AngleApproximationIKRatingModule();

        /*!
         * \brief
         * \param sourcePosition [in] the MILD's current position
         * \param targetPosition [in] the MILD's target position
         * \param sourceRotationBase [in] the MILD's current rotation
         * \param targetRotationBase [in] the MILD's target rotation
         * \return the rating angle (between 0.0 and 1.0)
         */
        double getPanAngleRating(const geometry_msgs::Point &sourcePosition, const geometry_msgs::Point &targetPosition, double sourceRotationBase, double targetRotationBase);
    };

    /*!
     * \brief Definition for the shared pointer type of the class.
     */
    typedef boost::shared_ptr<AngleApproximationIKRatingModule> AngleApproximationIKRatingModulePtr;
}

#endif /* ANGLEAPPROXIMATIONIKRATINGMODULE_H */
