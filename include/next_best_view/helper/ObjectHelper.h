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

#include <ros/ros.h>
#include <string>
#include <map>
#include "asr_object_database/ObjectMetaData.h"
#include "asr_world_model/GetRecognizerName.h"
#include "asr_world_model/GetIntermediateObjectWeight.h"

namespace next_best_view {
    typedef asr_object_database::ObjectMetaData::Response ObjectMetaDataResponse;
    typedef ObjectMetaDataResponse::Ptr ObjectMetaDataResponsePtr;
    typedef asr_world_model::GetIntermediateObjectWeight::Response IntermediateObjectWeightResponse;
    typedef IntermediateObjectWeightResponse::Ptr IntermediateObjectWeightResponsePtr;

    /**
     * The ObjectManager class delivers a possibility to handle request to the asr_object_database and cache their responses in a map.
     */
    class ObjectHelper {
    private:
        struct State {
        private:
            ros::NodeHandle mNodeHandle;
        public:
            ros::ServiceClient object_metadata_service_client;
            ros::ServiceClient recognizer_name_service_client;
            ros::ServiceClient intermediate_object_weight_service_client;

            std::map<std::string, ObjectMetaDataResponsePtr> object_metadata_cache;
            std::map<std::string, IntermediateObjectWeightResponsePtr> intermediate_object_cache;

            State() : mNodeHandle() {
                object_metadata_service_client = mNodeHandle.serviceClient
                        <asr_object_database::ObjectMetaData>("/asr_object_database/object_meta_data");
                recognizer_name_service_client = mNodeHandle.serviceClient
                        <asr_world_model::GetRecognizerName>("/env/asr_world_model/get_recognizer_name");
                intermediate_object_weight_service_client = mNodeHandle.serviceClient
                        <asr_world_model::GetIntermediateObjectWeight>("/env/asr_world_model/get_intermediate_object_weight");
            }
        };

        static boost::shared_ptr<State> InstancePtr() {
            static const boost::shared_ptr<State> statePtr = boost::shared_ptr<State>(new State());
            return statePtr;
        }


    public:
        ObjectHelper() {}

        ObjectMetaDataResponsePtr getObjectMetaData(std::string objectTypeName) {
            boost::shared_ptr<State> statePtr = InstancePtr();

            ObjectMetaDataResponsePtr responsePtr = statePtr->object_metadata_cache[objectTypeName];

            if (!responsePtr) {
                asr_world_model::GetRecognizerName getRecognizerName;
                getRecognizerName.request.object_type = objectTypeName;

                if (!statePtr->recognizer_name_service_client.exists()) {
                    ROS_ERROR_STREAM("/env/asr_world_model/get_recognizer_name service is not available");
                }
                statePtr->recognizer_name_service_client.call(getRecognizerName);

                std::string recognizer = getRecognizerName.response.recognizer_name;

                if (recognizer.compare("") == 0) {
                    ROS_ERROR_STREAM("No recognizer name received from world_model for object type " << objectTypeName << ".");
                }

                asr_object_database::ObjectMetaData objectMetaData;
                objectMetaData.request.object_type = objectTypeName;
                objectMetaData.request.recognizer = recognizer;

                if (!statePtr->object_metadata_service_client.exists()) {
                    ROS_ERROR_STREAM("/asr_object_database/object_meta_data service is not available");
                    return ObjectMetaDataResponsePtr();
                }
                statePtr->object_metadata_service_client.call(objectMetaData);

                if (!objectMetaData.response.is_valid) {
                    ROS_ERROR_STREAM("objectMetadata response is not valid for object type " << objectTypeName);
                    return ObjectMetaDataResponsePtr();
                }

                responsePtr = ObjectMetaDataResponsePtr(new ObjectMetaDataResponse(objectMetaData.response));
                statePtr->object_metadata_cache[objectTypeName] = responsePtr;
            }
            return responsePtr;
        }

        //Comment !
        IntermediateObjectWeightResponsePtr getIntermediateObjectValue(std::string objectTypeName) {
            boost::shared_ptr<State> statePtr = InstancePtr();

            IntermediateObjectWeightResponsePtr responsePtr = statePtr->intermediate_object_cache[objectTypeName];

            if (!responsePtr) {
                asr_world_model::GetIntermediateObjectWeight getIntermediateObjectWeight;
                getIntermediateObjectWeight.request.object_type = objectTypeName;

                if (!statePtr->intermediate_object_weight_service_client.exists()) {
                    ROS_ERROR_STREAM("/env/asr_world_model/get_intermediate_object_weight service is not available");
                    return IntermediateObjectWeightResponsePtr();
                }
                statePtr->intermediate_object_weight_service_client.call(getIntermediateObjectWeight);

                ROS_ERROR_STREAM("GOT WEIGHT FROM WORLD MODEL " << getIntermediateObjectWeight.response.value);

                responsePtr = IntermediateObjectWeightResponsePtr(new IntermediateObjectWeightResponse(getIntermediateObjectWeight.response));

                statePtr->intermediate_object_cache[objectTypeName] = responsePtr;
            }

            return responsePtr;
        }
    };
}
