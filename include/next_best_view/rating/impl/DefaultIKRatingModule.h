/*
 * IKRatingModule.h
 *
 *  Created on: Dez 23, 2015
 *      Author: florianaumann
 */

#ifndef DEFFAULTIKRATINGMODULE_H
#define DEFFAULTIKRATINGMODULE_H

#include "next_best_view/rating/IKRatingModule.h"
#include "next_best_view/robot_model/RobotModel.hpp"



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
        DefaultIKRatingModule(RobotModelPtr robotModel);

        /*!
         * \brief destructor of the DefaultIKRatingModule object.
         */
        virtual ~DefaultIKRatingModule();

        /*!
         * \brief
         * \param sourcePosition [in] the MILD's current position
         * \param targetPosition [in] the MILD's target position
         * \param sourceRotationBase [in] the MILD's current rotation
         * \param targetRotationBase [in] the MILD's target rotation
         * \return the rating angle (between 0.0 and 1.0)
         */
        double getPanAngleRating(const geometry_msgs::Point &sourcePosition, const geometry_msgs::Point &targetPosition, double sourceRotationBase, double targetRotationBase);
    private:
        RobotModelPtr mRobotModel;
    };

    /*!
     * \brief Definition for the shared pointer type of the class.
     */
    typedef boost::shared_ptr<DefaultIKRatingModule> DefaultIKRatingModulePtr;
}

#endif /* DEFFAULTIKRATINGMODULE_H */
