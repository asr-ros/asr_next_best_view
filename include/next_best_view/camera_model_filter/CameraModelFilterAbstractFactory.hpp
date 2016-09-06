/*
 * CameraModelFilterAbstractFactory.hpp
 *
 * Created on: Aug 29, 2016
 *      Author: Daniel Stroh
 */

#pragma once

#include "next_best_view/camera_model_filter/CameraModelFilter.hpp"

namespace next_best_view {

    class CameraModelFilterAbstractFactory {

    public:
        virtual CameraModelFilterPtr createCameraModelFilter() = 0;
    };
    typedef boost::shared_ptr<CameraModelFilterAbstractFactory> CameraModelFilterAbstractFactoryPtr;
}
