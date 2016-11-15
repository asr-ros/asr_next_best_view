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

#include "typedef.hpp"
#include <vector>
#include <map>
#include <chrono>

#include <boost/foreach.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/range/algorithm_ext/iota.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/range/irange.hpp>

#include "next_best_view/NextBestViewCache.hpp"
#include "next_best_view/NextBestViewPrediction.hpp"
#include "next_best_view/GeneticAlgorithm.hpp"

#include "next_best_view/hypothesis_updater/HypothesisUpdater.hpp"
#include "next_best_view/robot_model/RobotModel.hpp"
#include "next_best_view/crop_box/CropBoxFilter.hpp"
#include "next_best_view/camera_model_filter/CameraModelFilter.hpp"
#include "next_best_view/camera_model_filter/CameraModelFilterAbstractFactory.hpp"
#include "next_best_view/unit_sphere_sampler/UnitSphereSampler.hpp"
#include "next_best_view/space_sampler/SpaceSampler.hpp"
#include "next_best_view/rating/RatingModule.hpp"
#include "next_best_view/rating/RatingModuleAbstractFactory.hpp"
#include "next_best_view/rating/impl/DefaultScoreContainer.hpp"
#include "next_best_view/cluster/ClusterExtraction.hpp"
#include "next_best_view/filter/sample_point/HypothesisClusterSpaceSampleFilter.hpp"
#include "next_best_view/filter/sample_point/HypothesisKDTreeSpaceSampleFilter.hpp"
#include "next_best_view/filter/sample_point/MapSpaceSampleFilter.hpp"
#include "asr_msgs/AsrAttributedPointCloud.h"
#include "asr_msgs/AsrAttributedPoint.h"
#include "next_best_view/helper/DebugHelper.hpp"
#include "next_best_view/helper/MapHelper.hpp"
#include "next_best_view/helper/ObjectHelper.h"
#include "next_best_view/helper/VisualizationsHelper.hpp"

namespace next_best_view {

class GeneticAlgorithm;
typedef boost::shared_ptr<GeneticAlgorithm> GeneticAlgorithmPtr;
class NextBestViewCache;
typedef boost::shared_ptr<NextBestViewCache> NextBestViewCachePtr;
class NextBestViewPrediction;
typedef boost::shared_ptr<NextBestViewPrediction> NextBestViewPredictionPtr;
class NextBestViewCalculator;
typedef boost::shared_ptr<NextBestViewCalculator> NextBestViewCalculatorPtr;

class NextBestViewCalculator : public boost::enable_shared_from_this<NextBestViewCalculator> {
private:
    ObjectPointCloudPtr mPointCloudPtr;
    // this is basically unused, but "might" be used to remove objects from mPointCloudPtr
    IndicesPtr mActiveIndicesPtr;
    KdTreePtr mKdTreePtr;
    std::map<std::string, std::string> objectsResources;

    // modules
    UnitSphereSamplerPtr mUnitSphereSamplerPtr;
    MapHelperPtr mMapHelperPtr;
    SpaceSamplerPtr mSpaceSamplerPtr;
    RobotModelPtr mRobotModelPtr;
    CameraModelFilterPtr mCameraModelFilterPtr;
    RatingModulePtr mRatingModulePtr;
    HypothesisUpdaterPtr mHypothesisUpdaterPtr;
    ClusterExtractionPtr mClusterExtractionPtr;

    // factories for parallel rating
    RatingModuleAbstractFactoryPtr mRatingModuleAbstractFactoryPtr;
    CameraModelFilterAbstractFactoryPtr mCameraModelFilterAbstractFactoryPtr;

    // filters
    GeneralFilterPtr<SamplePoint> mSpaceSamplingFilterChainPtr;
    GeneralFilterPtr<SamplePoint> mHypothesisClusterSpaceSampleFilterPtr;
    GeneralFilterPtr<SamplePoint> mHypothesisKDTreeSpaceSampleFilterPtr;
    GeneralFilterPtr<SamplePoint> mMapSpaceSampleFilterPtr;
    bool mEnableClusterFilter;
    bool mEnableMapFilter;
    bool mEnableKDTreeFilter;

    CropBoxFilterPtr mCropBoxFilterPtr;
    float mEpsilon;
    ObjectTypeSetPtr mObjectTypeSetPtr;
    DebugHelperPtr mDebugHelperPtr;
    VisualizationHelperPtr mVisHelperPtr;

    bool mEnableCropBoxFiltering;
    bool mEnableIntermediateObjectWeighting;

    int mMaxIterationSteps;
    int mMinIterationSteps;

    int mNumberOfThreads;
    std::vector<CameraModelFilterPtr> mThreadCameraModels;
    std::vector<RatingModulePtr> mThreadRatingModules;

    bool mFirstNBVCallIsRunning;

    double mMinUtility;
    bool mRequireMinUtility;

    bool mRemoveInvalidNormals;
    bool mEnableClustering;
    bool mEnablePrediction;
    bool mEnableGA;

    // 2d grid which contains best viewport (utility) per element
    // wheter results should be cached for next nbvs
    bool mCacheResults;
    NextBestViewCachePtr mNBVCachePtr;
    NextBestViewPredictionPtr mNBVPredictionPtr;

    int mMinIterationGA;
    GeneticAlgorithmPtr mGeneticAlgorithmPtr;
    ViewportPointCloudPtr mRatedSortedViewportsPreIteration;

public:

    NextBestViewCalculator(const UnitSphereSamplerPtr & unitSphereSamplerPtr = UnitSphereSamplerPtr(),
                           const MapHelperPtr &mapHelperPtr = MapHelperPtr(),
                           const SpaceSamplerPtr &spaceSamplerPtr = SpaceSamplerPtr(),
                           const RobotModelPtr &robotModelPtr = RobotModelPtr(),
                           const CameraModelFilterPtr &cameraModelFilterPtr = CameraModelFilterPtr(),
                           const RatingModulePtr &ratingModulePtr = RatingModulePtr());

    /**
     * Calculates the next best view. Starting point of iterative calculations for getNextBestView() service call.
     */
    bool calculateNextBestView(const ViewportPoint &currentCameraViewport, ViewportPoint &resultViewport);

    /**
     * @brief initializeRobotState initializes the robotstate to the given viewport
     * @param currentCameraViewport the camera viewport of the robot
     */
    void initializeRobotState(const ViewportPoint &currentCameraViewport);

    void getFeasibleSamplePoints(const SamplePointCloudPtr &sampledSpacePointCloudPtr, IndicesPtr &resultIndicesPtr);

    /**
     * @brief getFeasibleViewports returns a vector of indices/sampleViewports which contain nearby (hypothesis)
     * @param sampleViewportsPtr viewports which may contain nearby object hypothesis
     * @param resultIndicesPtr
     */
    void getFeasibleViewports(const ViewportPointCloudPtr &sampleViewportsPtr, IndicesPtr &resultIndicesPtr);


    /**
     * @brief getFeasibleHypothesis finds object hypothesis near samplePoint
     * @param samplePoint the point to find nearby hypothesis
     * @param resultIndicesPtr nearby object hypothesis
     * @return true if more than 0 object hypothesis are in range of samplePoint
     */
    bool getFeasibleHypothesis(SimpleVector3 samplePoint, IndicesPtr &resultIndicesPtr);

    /**
     * @brief rates given viewports, which must contain child_indices
     * @param sampleNextBestViewports [in] viewports to rate
     * @param currentCameraViewport [in] current camera viewport to rate
     * @param resultViewport [out] viewport with best rating
     * @param objectTypeSetIsKnown
     * @return if valid result
     */
    bool rateViewports(const ViewportPointCloudPtr &sampleNextBestViewportsPtr, const ViewportPoint &currentCameraViewport, ViewportPoint &resultViewport, bool objectTypeSetIsKnown = false);

    /**
     * @brief rates given viewports, which must contain child_indices
     * @param sampleNextBestViewports [in] viewports to rate
     * @param currentCameraViewport [in] current camera viewport to rate
     * @param ratedNextBestViewports [out] rated viewports, which might be fewer
     * @param objectTypeSetIsKnown
     * @return if valid result
     */
    bool rateViewports(const ViewportPointCloudPtr &sampleNextBestViewports, const ViewportPoint &currentCameraViewport, ViewportPointCloudPtr &ratedNextBestViewportsPtr, bool objectTypeSetIsKnown = false);

private:

    /**
     * @brief rates viewports and returns vector with rated wiewports and the best rated viewport
     * @param sampleNextBestViewports [in] viewports to rate
     * @param currentCameraViewport [in] current camera viewport to rate
     * @param ratedNextBestViewports [out] rated (unsorted in most cases) vector of viewports
     * @param resultViewport [out] best viewport
     * @param objectTypeSetIsKnown
     * @return if valid result
     */
    bool rateViewportsInternal(const ViewportPointCloudPtr &sampleNextBestViewports, const ViewportPoint &currentCameraViewport,
                               ViewportPointCloudPtr &ratedNextBestViewports, ViewportPoint &resultViewport, bool objectTypeSetIsKnown);

    /**
     * @brief ratingThread
     * @param threadId
     * @param mutex
     * @param sampleNextBestViewports viewport sampling, which contains a good estimation for nearby objects per viewport
     * @param currentCameraViewport current camera viewport, used to rate
     * @param ratedNextBestViewports results
     * @param objectTypeSetIsKnown if object_types_set is set fixed and hould not be optimized
     */
    void ratingThread(int threadId, boost::mutex &mutex,
                      const ViewportPointCloudPtr &sampleNextBestViewports,
                      const ViewportPoint &currentCameraViewport,
                      const ViewportPointCloudPtr &ratedNextBestViewports,
                      bool objectTypeSetIsKnown);

    /**
     * @brief uses setBestScoreContainer to optimize which objects should be recognized
     * @param ratingModulePtr [in] used to rate
     * @param currentCameraViewport [in] used to rate
     * @param fullViewportPoint [out] resulting rated viewport
     * @return
     */
    bool rateSingleViewportOptimizeObjectTypes(const RatingModulePtr &ratingModulePtr, const ViewportPoint &currentCameraViewport, ViewportPoint &fullViewportPoint);

    /**
     * @brief uses setSingleScoreContainer to use object_type_set of fullViewportPoint to rate.
     * @param ratingModulePtr [in] used to rate
     * @param currentCameraViewport [in] used to rate
     * @param fullViewportPoint [out] resulting rated viewport
     * @return
     */
    bool rateSingleViewportFixedObjectTypes(const RatingModulePtr &ratingModulePtr, const ViewportPoint &currentCameraViewport, ViewportPoint &fullViewportPoint);

    bool getNBVFromCache(const ViewportPoint &currentCameraViewport, ViewportPoint &resultViewport);

    bool doIteration(const ViewportPoint &currentCameraViewport, const SimpleQuaternionCollectionPtr &sampledOrientationsPtr, ViewportPoint &resultViewport);

    bool doIterationStep(const ViewportPoint &currentCameraViewport, const ViewportPoint &currentBestViewport,
                         const SimpleQuaternionCollectionPtr &sampledOrientationsPtr, float contractor,
                         ViewportPoint &resultViewport, int iterationStep);

public:
    void setHeight(SamplePointCloudPtr pointCloudPtr, double height);

    /**
     * @brief creates a new camera viewport with the given data
     * @param resultViewportPoint
     * @return whether there are objects in the resulting viewport
     */
    bool doFrustumCulling(ViewportPoint &resultViewportPoint);

    /**
     * @brief creates a new camera viewport with the given data
     * @param cameraModelFilterPtr [in] the cameraModel used to filter objects
     * @param resultViewportPoint [out|in] the resulting camera viewport containg the camera position and orientation and the object point indices to be used
     * @return whether there are objects in the resulting viewport
     */
    bool doFrustumCulling(const CameraModelFilterPtr &cameraModelFilterPtr, ViewportPoint &resultViewportPoint);

    /**
     * @brief updates point cloud with external viewport point list
     * @param viewportPointList the list of viewport points
     * @return the number of deactivated normals
     */
    void updateFromExternalViewportPointList(const std::vector<ViewportPoint> &viewportPointList);

    /*!
     * \brief Updates the point cloud under the assumption that the given viewport was chosen.
     * \param objectTypeSetPtr the object type names that shall be updated.
     * \param viewportPoint the viewport that was chosen
     * \return the number of deactivated normals
     */
    unsigned int updateObjectPointCloud(const ObjectTypeSetPtr &objectTypeSetPtr, const ViewportPoint &viewportPoint);

    /**
      * Sets the point cloud points from point cloud message
      * @param message - message containing the point cloud
      */
    bool setPointCloudFromMessage(const asr_msgs::AsrAttributedPointCloud &msg);

private:

    /**
     * @brief filterUnrechableNormals this method really removes them so active_normals_vectors.size() == normal_vectors.size() is true.
     */
    void filterUnrechableNormals();

    /**
     * @brief generateSampleViewports
     * @param spaceSampleStartVector
     * @param contractor
     * @param pointCloudHeight
     * @return
     */
    ViewportPointCloudPtr generateSampleViewports(SimpleVector3 spaceSampleStartVector, double contractor, double pointCloudHeight);

    /**
     * @brief generateOrientationSamples
     * @return
     */
    SimpleQuaternionCollectionPtr generateOrientationSamples();

    /**
     * @brief generateSpaceSamples
     * @param spaceSampleStartVector
     * @param contractor
     * @param pointCloudHeight
     * @return
     */
    SamplePointCloudPtr generateSpaceSamples(SimpleVector3 spaceSampleStartVector, double contractor, double pointCloudHeight);

    /**
     * @brief combines space samples and orientation samples to viewport samples
     * @param sampledSpacePointCloudPtr
     * @param sampledOrientationsPtr
     * @param feasibleIndicesPtr
     * @return
     */
    ViewportPointCloudPtr combineSamples(SamplePointCloudPtr sampledSpacePointCloudPtr, SimpleQuaternionCollectionPtr sampledOrientationsPtr);

    bool setNormals(const ObjectPoint& pointCloudPoint);

    bool setNormalsInCropBoxMode(const ObjectPoint& pointCloudPoint);

    bool isPointInCropbox(const SimpleVector3& position, const Eigen::Vector3f& translation, const Eigen::Vector4f& max) const;

    bool isPointInRange(float point, float min, float lenght) const;

    /////
    ///
    // GETTER AND SETTER
    ///
    //////
public:
    /**
     * Returns the path to a meshs resource file
     */
    std::string getMeshPathByName(std::string objectType);

    /**
     * Sets the point cloud ptr.
     */
    void setPointCloudPtr(const ObjectPointCloudPtr &pointCloudPtr);

    void loadCropBoxListFromFile(const std::string mCropBoxListFilePath);

    void setEnableCropBoxFiltering(const bool mEnableCropBoxFiltering);

    void setEnableIntermediateObjectWeighting(const bool mEnableIntermediateObjectWeighting);

    /**
     * @return the point cloud pointer
     */
    ObjectPointCloudPtr getPointCloudPtr();

    /*!
     * \brief sets the active indices.
     */
    void setActiveIndices(const IndicesPtr &activeIndicesPtr);

    /*!
     * \return the active Indices.
     */
    IndicesPtr getActiveIndices();

    int getNumberActiveNormals();

    unsigned int getNumberTotalNormals(std::string type, std::string identifier);

    unsigned int getNumberActiveNormals(std::string type, std::string identifier);

    std::vector<std::pair<std::string, std::string>> getTypeAndIds();

    bool removeObjects(std::string type, std::string identifier);

    /**
     * @return the point cloud pointer
     */
    KdTreePtr getKdTreePtr();

    /**
     * Sets the unit sphere sampler.
     * @param unitSphereSamplerPtr - the pointer to the unit sphere sampler
     */
    void setUnitSphereSampler(const UnitSphereSamplerPtr &unitSphereSamplerPtr);

    /**
     * @return the unit sphere sampler.
     */
    UnitSphereSamplerPtr getUnitSphereSampler();

    /**
     * Sets the space sampler.
     * @param spaceSamplerPtr - the pointer to the space sampler
     */
    void setSpaceSampler(const SpaceSamplerPtr &spaceSamplerPtr);

    /**
     * @return the pointer to the space sampler.
     */
    SpaceSamplerPtr getSpaceSampler();

    /**
     * sets the robot model
     * @param robotModelPtr - the pointer to robot model.
     */
    void setRobotModel(const RobotModelPtr &robotModelPtr);

    /**
     *@return the robot ptr
     */
    RobotModelPtr getRobotModel();

    /**
     * Sets the camera model filter.
     * @param cameraModelFilterPtr - the pointer to the camera model filter.
     */
    void setCameraModelFilter(const CameraModelFilterPtr &cameraModelFilterPtr);

    /**
     * @return the pointer to the camera model filter
     */
    CameraModelFilterPtr getCameraModelFilter();

    /**
     * Sets the rating module.
     * @param ratingModulePtr - the pointer to the rating module
     */
    void setRatingModule(const RatingModulePtr &ratingModulePtr);

    /**
     * @return the pointer to the rating module.
     */
    RatingModulePtr getRatingModule();

    /**
      * sets the map helper
      * @param mapHelperPtr - the pointer to map helper.
      */
    void setMapHelper(const MapHelperPtr &mapHelperPtr);

    /**
      *@return the map helper ptr
      */
    MapHelperPtr getMapHelper();

    void setHypothesisUpdater(const HypothesisUpdaterPtr &hypothesisUpdaterPtr);

    HypothesisUpdaterPtr getHypothesisUpdater();

    ClusterExtractionPtr getClusterExtractionPtr() const;

    void setClusterExtractionPtr(const ClusterExtractionPtr &value);

    /**
     * @param epsilon - error threshold
     */
    void setEpsilon(float epsilon);

    /**
     * @return epsilon
     */
    float getEpsilon();

    /**
     * Max number of hierarchical iterations when calculating a next-best-view.
     * @param maxIterationSteps  - max number of iteration steps
     */
    void setMaxIterationSteps(int maxIterationSteps);

    void setMinIterationSteps(int minIterationSteps);

    /**
     * @brief setRatingModuleAbstractFactoryPtr used to generate rating modules per thread.
     * @param ratingModuleAbstractFactoryPtr
     */
    void setRatingModuleAbstractFactoryPtr(const RatingModuleAbstractFactoryPtr &ratingModuleAbstractFactoryPtr);

    /**
     * @brief getRatingModuleAbstractFactoryPtr
     * @return
     */
    RatingModuleAbstractFactoryPtr getRatingModuleAbstractFactoryPtr();

    /**
     * @brief setCameraModelFilterAbstractFactoryPtr used to generate camera models per thread.
     * @param cameraModelFilterAbstractFactoryPtr
     */
    void setCameraModelFilterAbstractFactoryPtr(const CameraModelFilterAbstractFactoryPtr &cameraModelFilterAbstractFactoryPtr);

    /**
     * @brief getCameraModelFilterAbstractFactoryPtr
     * @return
     */
    CameraModelFilterAbstractFactoryPtr getCameraModelFilterAbstractFactoryPtr();

    /**
     * @brief getNumberOfThreads
     * @return
     */
    int getNumberOfThreads() const;

    /**
     * @brief sets number of threads used to rate samples, negative numbers use boost::thread::hardware_concurrency() as number of threads.
     * @param value
     */
    void setNumberOfThreads(int value);

    double getMinUtility() const;

    void setMinUtility(double minUtility);

    bool getRemoveInvalidNormals() const;

    void setRemoveInvalidNormals(bool removeInvalidNormals);

    bool getCacheResults() const;

    /**
    * @brief setCacheResults true if nbv sampling results should be cached for following nbv calls with the same PC.
    * @param value cacheResults
    */
    void setCacheResults(bool cacheResults);

    bool getRequireMinUtility() const;

    void setRequireMinUtility(bool requireMinUtility);

    bool getEnableClustering() const;

    void setEnableClustering(bool enableClustering);

    bool getEnablePrediction() const;

    void setEnablePrediction(bool enablePrediction);

    // filters en/disable
    bool getEnableClusterFilter() const;

    void setEnableClusterFilter(bool enableClusterFilter);

    bool getEnableMapFilter() const;

    void setEnableMapFilter(bool enableMapFilter);

    bool getEnableKDTreeFilter() const;

    void setEnableKDTreeFilter(bool enableKDTreeFilter);

    GeneralFilterPtr<SamplePoint> getMapSpaceSampleFilterPtr() const;

    void setMapSpaceSampleFilterPtr(const GeneralFilterPtr<SamplePoint> &mapSpaceSampleFilterPtr);

    GeneralFilterPtr<SamplePoint> getHypothesisKDTreeSpaceSampleFilterPtr() const;

    void setHypothesisKDTreeSpaceSampleFilterPtr(const GeneralFilterPtr<SamplePoint> &hypothesisKDTreeSpaceSampleFilterPtr);

    GeneralFilterPtr<SamplePoint> getHypothesisClusterSpaceSampleFilterPtr() const;

    void setHypothesisClusterSpaceSampleFilterPtr(const GeneralFilterPtr<SamplePoint> &hypothesisClusterSpaceSampleFilterPtr);

    GeneralFilterPtr<SamplePoint> getSpaceSamlpingFilterChainPtr() const;

    void setSpaceSamplingFilterChainPtr();

    int getMinIterationGA() const;

    void setMinIterationGA(int minIterationGA);

    GeneticAlgorithmPtr getGeneticAlgorithmPtr() const;

    void setGeneticAlgorithmPtr(const GeneticAlgorithmPtr &geneticAlgorithmPtr);

    bool getEnableGA() const;

    void setEnableGA(bool enableGA);
};

}
