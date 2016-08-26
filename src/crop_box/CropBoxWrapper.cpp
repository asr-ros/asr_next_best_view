/**

Copyright (c) 2016, Allgeyer Tobias, Aumann Florian, Borella Jocelyn, Braun Kai, Heller Florian, Hutmacher Robin, Karrenbauer Oliver, Marek Felix, Mayr Matthias, Mehlhaus Jonas, Meißner Pascal, Schleicher Ralf, Stöckle Patrick, Stroh Daniel, Trautmann Jeremias, Walter Milena
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

4. The use is explicitly not permitted to any application which deliberately try to kill or do harm to any living creature.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "next_best_view/crop_box/CropBoxWrapper.hpp"
#include <rapidxml.hpp>
#include <rapidxml_utils.hpp>

namespace next_best_view {

    CropBoxWrapper::CropBoxWrapper(CropBoxPtr cropBoxPtr, boost::shared_ptr<std::vector<SimpleVector3>> cropBoxNormalsListPtr) {
        this->mCropBoxPtr = cropBoxPtr;
        this->mCropBoxNormalsListPtr = cropBoxNormalsListPtr;
    }

    CropBoxPtr CropBoxWrapper::getCropBox() {
        return mCropBoxPtr;
    }

    boost::shared_ptr<std::vector<SimpleVector3>> CropBoxWrapper::getCropBoxNormalsList() {
        return mCropBoxNormalsListPtr;
    }

    void CropBoxWrapper::setCropBox(CropBoxPtr cropBoxPtr) {
        this->mCropBoxPtr = cropBoxPtr;
    }

    void CropBoxWrapper::setCropBoxNormalsList(boost::shared_ptr<std::vector<SimpleVector3>> cropBoxNormalsListPtr) {
        this->mCropBoxNormalsListPtr = cropBoxNormalsListPtr;
    }

    boost::shared_ptr<std::vector<CropBoxWrapperPtr>> CropBoxWrapper::readCropBoxDataFromXMLFile(const std::string &xml_path) {
        auto cropBoxPtrList = boost::make_shared<std::vector<CropBoxWrapperPtr>>();
        DebugHelperPtr debugHelperPtr = DebugHelper::getInstance();
        debugHelperPtr->write(std::stringstream() << "Path to CropBoxList xml file: " << xml_path,
                  DebugHelper::CALCULATION);
        try {
            rapidxml::file<> xmlFile(xml_path.c_str());
            rapidxml::xml_document<> doc;
            doc.parse<0>(xmlFile.data());

            rapidxml::xml_node<> *root_node = doc.first_node();
            if (root_node) {
                rapidxml::xml_node<> *child_node = root_node->first_node();

                while (child_node)
                {
                    CropBoxPtr bufferCropBoxPtr = CropBoxPtr(new CropBox);
                    rapidxml::xml_node<> * min_pt = child_node->first_node("min_pt");
                    rapidxml::xml_attribute<> *x = min_pt->first_attribute("x");
                    rapidxml::xml_attribute<> *y = min_pt->first_attribute("y");
                    rapidxml::xml_attribute<> *z = min_pt->first_attribute("z");
                    if (x && y && z)
                    {
                        double x_ = boost::lexical_cast<double>(x->value());
                        double y_ = boost::lexical_cast<double>(y->value());
                        double z_ = boost::lexical_cast<double>(z->value());
                        Eigen::Vector4f pt_min(x_,y_,z_,1);
                        bufferCropBoxPtr->setMin(pt_min);
                    }

                    rapidxml::xml_node<> * max_pt = child_node->first_node("max_pt");
                    x = max_pt->first_attribute("x");
                    y = max_pt->first_attribute("y");
                    z = max_pt->first_attribute("z");
                    if (x && y && z)
                    {
                        double x_ = boost::lexical_cast<double>(x->value());
                        double y_ = boost::lexical_cast<double>(y->value());
                        double z_ = boost::lexical_cast<double>(z->value());
                        Eigen::Vector4f pt_max(x_,y_,z_,1);
                        bufferCropBoxPtr->setMax(pt_max);
                    }

                    rapidxml::xml_node<> * rotation = child_node->first_node("rotation");
                    x = rotation->first_attribute("x");
                    y = rotation->first_attribute("y");
                    z = rotation->first_attribute("z");
                    if (x && y && z)
                    {
                        double x_ = boost::lexical_cast<double>(x->value());
                        double y_ = boost::lexical_cast<double>(y->value());
                        double z_ = boost::lexical_cast<double>(z->value());
                        Eigen::Vector3f rotation(x_,y_,z_);
                        bufferCropBoxPtr->setRotation(rotation);
                    }

                    rapidxml::xml_node<> * translation = child_node->first_node("translation");
                    x = translation->first_attribute("x");
                    y = translation->first_attribute("y");
                    z = translation->first_attribute("z");
                    if (x && y && z)
                    {
                        double x_ = boost::lexical_cast<double>(x->value());
                        double y_ = boost::lexical_cast<double>(y->value());
                        double z_ = boost::lexical_cast<double>(z->value());
                        Eigen::Vector3f translation(x_,y_,z_);
                        bufferCropBoxPtr->setTranslation(translation);
                    }

                    int normalsCount = 0;
                    auto normals = boost::make_shared<std::vector<SimpleVector3>>();
                    char search[100];
                    sprintf(search, "normal_%d", normalsCount);
                    rapidxml::xml_node<> * normalsXML = child_node->first_node(search);
                    while (normalsXML != nullptr) {
                        x = normalsXML->first_attribute("x");
                        y = normalsXML->first_attribute("y");
                        z = normalsXML->first_attribute("z");
                        if (x && y && z)
                        {
                            double x_ = boost::lexical_cast<double>(x->value());
                            double y_ = boost::lexical_cast<double>(y->value());
                            double z_ = boost::lexical_cast<double>(z->value());
                            SimpleVector3 normal(x_, y_, z_);
                            normals->push_back(normal);
                        }
                        ++normalsCount;
                        std::fill(search, search + sizeof(search)/sizeof(search[0]), 0);
                        sprintf(search, "normal_%d", normalsCount);
                        normalsXML = child_node->first_node(search);
                    }
                    CropBoxWrapperPtr cropBoxWrapperPtr = CropBoxWrapperPtr(new CropBoxWrapper(bufferCropBoxPtr, normals));

                    cropBoxPtrList->push_back(cropBoxWrapperPtr);
                    child_node = child_node->next_sibling();
                }
            }
        } catch(std::runtime_error err) {
          ROS_ERROR_STREAM("Can't parse xml-file. Runtime error: " << err.what());
        } catch (rapidxml::parse_error err) {
          ROS_ERROR_STREAM("Can't parse xml-file Parse error: " << err.what());
        }
        return cropBoxPtrList;
    }
}
