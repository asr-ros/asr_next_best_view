#pragma once

#include <ros/ros.h>
#include <string>
#include <map>
#include "object_database/ObjectMetaData.h"
#include "world_model/GetRecognizerName.h"
#include "world_model/GetIntermediateObjectWeight.h"

namespace next_best_view {
    typedef object_database::ObjectMetaData::Response ObjectMetaDataResponse;
    typedef ObjectMetaDataResponse::Ptr ObjectMetaDataResponsePtr;
    typedef world_model::GetIntermediateObjectWeight::Response IntermediateObjectWeightResponse;
    typedef IntermediateObjectWeightResponse::Ptr IntermediateObjectWeightResponsePtr;

    /**
     * The ObjectManager class delivers a possibility to handle request to the object_database and cache their responses in a map.
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
                        <object_database::ObjectMetaData>("/object_database/object_meta_data");
                recognizer_name_service_client = mNodeHandle.serviceClient
                        <world_model::GetRecognizerName>("/env/world_model/get_recognizer_name");
                intermediate_object_weight_service_client = mNodeHandle.serviceClient
                        <world_model::GetIntermediateObjectWeight>("/env/world_model/get_intermediate_object_weight");
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
                world_model::GetRecognizerName getRecognizerName;
                getRecognizerName.request.object_type = objectTypeName;

                if (!statePtr->recognizer_name_service_client.exists()) {
                    ROS_ERROR_STREAM("/env/world_model/get_recognizer_name service is not available");
                }
                statePtr->recognizer_name_service_client.call(getRecognizerName);

                std::string recognizer = getRecognizerName.response.recognizer_name;

                if (recognizer.compare("") == 0) {
                    ROS_ERROR_STREAM("No recognizer name received from world_model for object type " << objectTypeName << ".");
                }

                object_database::ObjectMetaData objectMetaData;
                objectMetaData.request.object_type = objectTypeName;
                objectMetaData.request.recognizer = recognizer;

                if (!statePtr->object_metadata_service_client.exists()) {
                    ROS_ERROR_STREAM("/object_database/object_meta_data service is not available");
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
                world_model::GetIntermediateObjectWeight getIntermediateObjectWeight;
                getIntermediateObjectWeight.request.object_type = objectTypeName;

                if (!statePtr->intermediate_object_weight_service_client.exists()) {
                    ROS_ERROR_STREAM("/env/world_model/get_intermediate_object_weight service is not available");
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
