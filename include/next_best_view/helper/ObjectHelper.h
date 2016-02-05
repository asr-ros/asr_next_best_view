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
            std::map<std::string, double> intermediate_object_cache;

            State() : mNodeHandle() {
                object_metadata_service_client = mNodeHandle.serviceClient
                        <object_database::ObjectMetaData>("/object_database/object_meta_data");
                recognizer_name_service_client = mNodeHandle.serviceClient
                        <world_model::GetRecognizerName>("/env/world_model/get_recognizer_name");
                intermediate_object_weight_service_client = mNodeHandle.serviceClient
                        <world_model::GetRecognizerName>("/env/world_model/get_intermediate_object_weight");
            }
        };

        static boost::shared_ptr<State> InstancePtr() {
            static const boost::shared_ptr<State> statePtr = boost::shared_ptr<State>(new State());
            return statePtr;
        }


    public:
        ObjectHelper() {}

        ObjectMetaDataResponsePtr get(std::string objectTypeName) {
            boost::shared_ptr<State> statePtr = InstancePtr();

            if (!statePtr->object_metadata_service_client.exists()) {
                return ObjectMetaDataResponsePtr();
            }

            ObjectMetaDataResponsePtr responsePtr = statePtr->object_metadata_cache[objectTypeName];

            if (!responsePtr) {
                world_model::GetRecognizerName getRecognizerName;
                getRecognizerName.request.object_type = objectTypeName;
                statePtr->recognizer_name_service_client.call(getRecognizerName);

                std::string recognizer = getRecognizerName.response.recognizer_name;

                object_database::ObjectMetaData objectMetaData;
                objectMetaData.request.object_type = objectTypeName;
                objectMetaData.request.recognizer = recognizer;
                statePtr->object_metadata_service_client.call(objectMetaData);
                if (!objectMetaData.response.is_valid) {
                    return ObjectMetaDataResponsePtr();
                }

                responsePtr = ObjectMetaDataResponsePtr(new object_database::ObjectMetaData::Response(objectMetaData.response));
                statePtr->object_metadata_cache[objectTypeName] = responsePtr;
            }
            return responsePtr;
        }

        double getIntermediateObjectValue(std::string objectTypeName) {
            boost::shared_ptr<State> statePtr = InstancePtr();
            double weight;

            if (!statePtr->intermediate_object_weight_service_client.exists()) {
                return 0;
            }

            std::map<std::string, double>::iterator response_it = statePtr->
                    intermediate_object_cache.find(objectTypeName);

            if (response_it == statePtr->intermediate_object_cache.end()) {
                world_model::GetIntermediateObjectWeight getIntermediateObjectWeight;
                getIntermediateObjectWeight.request.object_type = objectTypeName;
                statePtr->intermediate_object_weight_service_client.call(getIntermediateObjectWeight);

                weight = getIntermediateObjectWeight.response.value;

                statePtr->intermediate_object_cache[objectTypeName] = weight;
            }
            else
            {
                weight = response_it->second;
            }

            return weight;
        }
    };
}
