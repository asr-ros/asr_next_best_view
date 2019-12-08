/**

Copyright (c) 2016, Aumann Florian, Borella Jocelyn, Heller Florian, Meißner Pascal, Schleicher Ralf, Stöckle Patrick, Stroh Daniel, Trautmann Jeremias, Walter Milena, Wittenbeck Valerij
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

4. The use is explicitly not permitted to any application which deliberately try to kill or do harm to any living creature.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#pragma once

#include "pcl/filters/impl/crop_box.hpp"
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
