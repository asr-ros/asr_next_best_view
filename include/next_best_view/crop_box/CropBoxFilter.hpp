/*
 * CropBoxFilter.hpp
 *
 *  Created on: Jun 29, 2016
 *      Author: Daniel Stroh
 */
#pragma once

#include "next_best_view/common/GeneralFilter.hpp"
#include "pcl-1.7/pcl/filters/impl/crop_box.hpp"

namespace next_best_view {

    /*!
     * \brief The CropBoxFilter class, used to filter objects in crop boxes.
     */
    class CropBoxFilter : public GeneralFilter {
    private:
        boost::shared_ptr<std::vector<CropBoxPtr>> mCropBoxPtrList;

        /*!
         * \brief readCropBoxDataFromXMLFile reads the given xml file into mCropBoxPtrList.
         * \param mCropBoxListFilePath path to a xml file containing crop box information (name, poisition, size).
         */
        void readCropBoxDataFromXMLFile(const std::string &mCropBoxListFilePath);

    public:
        /*!
         * \brief CropBoxFilter
         * \param xml_path path to a xml file containing crop box information (name, poisition, size).
         */
        CropBoxFilter(const std::string &xml_path);

        /*!
         * \brief doFiltering Does the actual filtering. setInputCloud(...) must be called before.
         * \param indicesPtr the filtered objects indices.
         */
        void doFiltering(IndicesPtr &indicesPtr);

        /*!
         * \brief getCropBoxPtrList
         * \return returns mCropBoxPtrList for visualization.
         */
        boost::shared_ptr<std::vector<CropBoxPtr>> getCropBoxPtrList();
    };

    /*!
     * \brief CropBoxFilterPtr typedef for consistency/less code to create boost::shared_ptr to CropBoxFilter.
     */
    typedef boost::shared_ptr<CropBoxFilter> CropBoxFilterPtr;
}
