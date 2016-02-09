/*
 * IKRatingModule.h
 *
 *  Created on: Dez 23, 2015
 *      Author: florianaumann
 */

#ifndef SIMPLEIKRATINGMODULE_H
#define SIMPLEIKRATINGMODULE_H

#include "next_best_view/rating/IKRatingModule.h"



namespace next_best_view {
    /*!
     * \brief SimpleIKRatingModule implements the functionlities offered by IKRatingModule.
     * \author Florian Aumann
     * \date 2016
     * \version 1.0
     * \copyright GNU Public License
     * \sa DefaultIKRatingModule
     */
    class SimpleIKRatingModule : public IKRatingModule {
    public:
        /*!
         * \brief constructor of the SimpleDefaultIKRatingModule object
         */
        SimpleIKRatingModule();

        /*!
         * \brief destructor of the SimpleDefaultIKRatingModule object.
         */
        virtual ~SimpleIKRatingModule();

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
    typedef boost::shared_ptr<SimpleIKRatingModule> SimpleIKRatingModulePtr;
}

#endif /* SIMPLEIKRATINGMODULE_H */
