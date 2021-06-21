/**
 * @copyright Copyright (c) 2017 B-com http://www.b-com.com/
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "SolARStereoFeatureExtractionAndDepthEstimation.h"
#include "core/Log.h"
#include <thread>

namespace xpcf = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::TOOLS::SolARStereoFeatureExtractionAndDepthEstimation);

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace TOOLS {

SolARStereoFeatureExtractionAndDepthEstimation::SolARStereoFeatureExtractionAndDepthEstimation() :ConfigurableBase(xpcf::toUUID<SolARStereoFeatureExtractionAndDepthEstimation>())
{
    addInterface<api::stereo::IStereoFeatureExtractionAndDepthEstimation>(this);
    m_keypointsDetector.resize(2);
    m_descriptorExtractor.resize(2);
    m_undistortPoints.resize(2);
    m_stereoRectificator.resize(2);
    for (int i = 0; i < 2; ++i) {
		std::string name = std::to_string(i);
        declareInjectable<api::features::IKeypointDetector>(m_keypointsDetector[i], name.c_str());
        declareInjectable<api::features::IDescriptorsExtractor>(m_descriptorExtractor[i], name.c_str());
        declareInjectable<api::geom::IUndistortPoints>(m_undistortPoints[i], name.c_str());
        declareInjectable<api::stereo::IStereoRectification>(m_stereoRectificator[i], name.c_str());
    }
    declareInjectable<api::stereo::IStereoDescriptorMatcher>(m_stereoMatcher);
    declareInjectable<api::stereo::IStereoDepthEstimation>(m_stereoDepthEstimator);
    LOG_DEBUG("SolARStereoFeatureExtractionAndDepthEstimation constructor");
}

SolARStereoFeatureExtractionAndDepthEstimation::~SolARStereoFeatureExtractionAndDepthEstimation()
{
    LOG_DEBUG("SolARStereoFeatureExtractionAndDepthEstimation destructor");
}

void SolARStereoFeatureExtractionAndDepthEstimation::setCameraParameters(const SolAR::datastructure::CamCalibration & intrinsicParams, const SolAR::datastructure::CamDistortion & distortionParams)
{
    for (int i = 0; i < 2; ++i)
        m_undistortPoints[i]->setCameraParameters(intrinsicParams, distortionParams);
    m_focal = intrinsicParams(0, 0);
}

FrameworkReturnCode SolARStereoFeatureExtractionAndDepthEstimation::compute(SRef<SolAR::datastructure::Image> image1, SRef<SolAR::datastructure::Image> image2, SRef<SolAR::datastructure::Frame>& frame1, SRef<SolAR::datastructure::Frame>& frame2)
{
    // feature extraction and rectification two images
    std::vector<std::vector<Keypoint>> keypoints(2);
    std::vector<std::vector<Keypoint>> undistortedKeypoints(2);
    std::vector<std::vector<Keypoint>> undistortedRectifiedKeypoints(2);
    std::vector<SRef<DescriptorBuffer>> descriptors(2);
    std::thread thread1(&SolARStereoFeatureExtractionAndDepthEstimation::extractAndRectify, this, 0, 
		image1, std::ref(keypoints[0]), std::ref(undistortedKeypoints[0]), 
		std::ref(undistortedRectifiedKeypoints[0]), std::ref(descriptors[0]));
    std::thread thread2(&SolARStereoFeatureExtractionAndDepthEstimation::extractAndRectify, this, 1, 
		image2, std::ref(keypoints[1]), std::ref(undistortedKeypoints[1]), 
		std::ref(undistortedRectifiedKeypoints[1]), std::ref(descriptors[1]));
    thread1.join();
    thread2.join();

    // stereo feature matching
    std::vector<DescriptorMatch> matches;
    m_stereoMatcher->match(descriptors[0], descriptors[1], undistortedRectifiedKeypoints[0],
        undistortedRectifiedKeypoints[1], m_stereoRectificator[0]->getType(), matches);
    LOG_DEBUG("Number of matches: {}", matches.size());

    // depth estimation
    m_stereoDepthEstimator->estimate(undistortedRectifiedKeypoints[0], undistortedRectifiedKeypoints[1], matches,
        m_focal, m_stereoRectificator[0]->getBaseline(), m_stereoRectificator[0]->getType());
    m_stereoDepthEstimator->estimate(undistortedRectifiedKeypoints[0], undistortedKeypoints[0],
        m_stereoRectificator[0]->getRectificationParamters(0));
    m_stereoDepthEstimator->estimate(undistortedRectifiedKeypoints[1], undistortedKeypoints[1],
        m_stereoRectificator[0]->getRectificationParamters(1));

    // make frames
    frame1 = xpcf::utils::make_shared<Frame>(keypoints[0], undistortedKeypoints[0], descriptors[0], image1);
    frame2 = xpcf::utils::make_shared<Frame>(keypoints[1], undistortedKeypoints[1], descriptors[1], image2);

    return FrameworkReturnCode::_SUCCESS;
}

void SolARStereoFeatureExtractionAndDepthEstimation::extractAndRectify(int indexCamera, SRef<datastructure::Image> image, std::vector<datastructure::Keypoint>& keypoints, std::vector<datastructure::Keypoint>& undistortedKeypoints, std::vector<datastructure::Keypoint>& undistortedRectifiedKeypoints, SRef<datastructure::DescriptorBuffer>& descriptors)
{
    m_keypointsDetector[indexCamera]->detect(image, keypoints);
    m_undistortPoints[indexCamera]->undistort(keypoints, undistortedKeypoints);
    m_descriptorExtractor[indexCamera]->extract(image, keypoints, descriptors);
    m_stereoRectificator[indexCamera]->rectify(undistortedKeypoints, undistortedRectifiedKeypoints, indexCamera);
}

}
}
}
