 /*
 * CropBox.hpp
 *
 *  Created on: Jun 29, 2016
 *      Author: Daniel Stroh
 */
#pragma once

#include "pcl-1.7/pcl/filters/impl/crop_box.hpp"
#include "typedef.hpp"

namespace next_best_view {

    class CropBoxWrapper;
    typedef boost::shared_ptr<CropBoxWrapper> CropBoxWrapperPtr;

    class CropBoxWrapper {
    private:
        CropBoxPtr mCropBoxPtr;
        boost::shared_ptr<std::vector<SimpleVector3>> mCropBoxNormalsListPtr;
    public:
        CropBoxWrapper(CropBoxPtr cropBoxPtr, boost::shared_ptr<std::vector<SimpleVector3>> cropBoxNormalsListPtr);

        CropBoxPtr getCropBox();
        boost::shared_ptr<std::vector<SimpleVector3>> getCropBoxNormalsList();

        void setCropBox(CropBoxPtr cropBoxPtr);
        void setCropBoxNormalsList(boost::shared_ptr<std::vector<SimpleVector3>> cropBoxNormalsListPtr);


        /*!
         * \brief readCropBoxDataFromXMLFile reads the given xml file and returns a list of CropBoxWrapper.
         * \param xml_path path to a xml file containing crop box information (name, poisition, size).
         * \return
         */
        static boost::shared_ptr<std::vector<CropBoxWrapperPtr>> readCropBoxDataFromXMLFile(const std::string &xml_path);
    };

}
