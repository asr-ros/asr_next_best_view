 /*
 * CropBoxFilter.hpp
 *
 *  Created on: Jun 29, 2016
 *      Author: Daniel Stroh
 */

#include "next_best_view/crop_box/CropBoxFilter.hpp"

#include <vector>
#include <set>

namespace next_best_view {

    CropBoxFilter::CropBoxFilter(const std::string &xml_path) : GeneralFilter() {
        mCropBoxPtrList = CropBoxWrapper::readCropBoxDataFromXMLFile(xml_path);
    }

    void CropBoxFilter::doFiltering(IndicesPtr &indicesPtr) {

        //Filter the point cloud
        auto result = boost::make_shared<std::set<int>>();
        for (CropBoxWrapperPtr cropBoxWrapper : *mCropBoxPtrList) {
            IndicesPtr filteredObjectIndices = IndicesPtr(new Indices());
            CropBoxPtr cropBoxPtr = cropBoxWrapper->getCropBox();
            cropBoxPtr->setInputCloud(this->getInputCloud());
            cropBoxPtr->filter(*filteredObjectIndices);
            // put all filtered objects into the result set
            result->insert(filteredObjectIndices->begin(), filteredObjectIndices->end());
        }

        // put result set into indicesPtr vector
        std::copy(result->begin(), result->end(), std::back_inserter(*indicesPtr));
    }

    boost::shared_ptr<std::vector<CropBoxWrapperPtr>> CropBoxFilter::getCropBoxWrapperPtrList() {
        return mCropBoxPtrList;
    }
}
