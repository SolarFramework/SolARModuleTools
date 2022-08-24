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
    addInterface<api::features::IFeatureWithDepthFromStereo>(this);
    m_descriptorExtractor.resize(2);
    m_undistortPoints.resize(2);
    m_stereoRectificator.resize(2);
    for (int i = 0; i < 2; ++i) {
		std::string name = std::to_string(i);
        declareInjectable<api::features::IDescriptorsExtractorFromImage>(m_descriptorExtractor[i], name.c_str());
        declareInjectable<api::geom::IUndistortPoints>(m_undistortPoints[i], name.c_str());
        declareInjectable<api::geom::I2DPointsRectification>(m_stereoRectificator[i], name.c_str());
    }
    declareInjectable<api::features::IDescriptorMatcherStereo>(m_stereoMatcher);
    declareInjectable<api::geom::IDepthEstimation>(m_stereoDepthEstimator);
    declareInjectable<api::geom::IReprojectionStereo>(m_stereoReprojector);
    declareProperty("isMultithreading", m_isMultithreading);
    LOG_DEBUG("SolARStereoFeatureExtractionAndDepthEstimation constructor");
}

SolARStereoFeatureExtractionAndDepthEstimation::~SolARStereoFeatureExtractionAndDepthEstimation()
{
    LOG_DEBUG("SolARStereoFeatureExtractionAndDepthEstimation destructor");
}

void SolARStereoFeatureExtractionAndDepthEstimation::setRectificationParameters(const SolAR::datastructure::CameraParameters & camParams1, const SolAR::datastructure::CameraParameters & camParams2, const SolAR::datastructure::RectificationParameters & rectParams1, const SolAR::datastructure::RectificationParameters & rectParams2)
{
	m_rectParams.resize(2);
	m_camParams.resize(2);
	m_rectParams[0] = rectParams1;
	m_rectParams[1] = rectParams2;
	m_camParams[0] = camParams1;
	m_camParams[1] = camParams2;
	m_isSetParams = true;
	m_isPassRectify.resize(2, false);
	// No need rectify if rectification rotation parameter is identity
	for (int i = 0; i < 2; ++i)
		if (m_rectParams[i].rotation.isIdentity())
			m_isPassRectify[i] = true;
}

FrameworkReturnCode SolARStereoFeatureExtractionAndDepthEstimation::compute(SRef<SolAR::datastructure::Image> image1, SRef<SolAR::datastructure::Image> image2, SRef<SolAR::datastructure::Frame>& frame1, SRef<SolAR::datastructure::Frame>& frame2)
{
	if (!m_isSetParams) {
		LOG_ERROR("Must set rectification parameters before");
		return FrameworkReturnCode::_ERROR_;
	}
    // feature extraction and rectification two images
    std::vector<std::vector<Keypoint>> keypoints(2);
    std::vector<std::vector<Keypoint>> undistortedKeypoints(2);
    std::vector<std::vector<Keypoint>> undistortedRectifiedKeypoints(2);
    std::vector<SRef<DescriptorBuffer>> descriptors(2);
    if (m_isMultithreading){
        std::thread thread1(&SolARStereoFeatureExtractionAndDepthEstimation::extractAndRectify, this, 0,
            image1, std::ref(keypoints[0]), std::ref(undistortedKeypoints[0]),
            std::ref(undistortedRectifiedKeypoints[0]), std::ref(descriptors[0]));
        std::thread thread2(&SolARStereoFeatureExtractionAndDepthEstimation::extractAndRectify, this, 1,
            image2, std::ref(keypoints[1]), std::ref(undistortedKeypoints[1]),
            std::ref(undistortedRectifiedKeypoints[1]), std::ref(descriptors[1]));
        thread1.join();
        thread2.join();
    }
    else {
        extractAndRectify(0, image1, keypoints[0], undistortedKeypoints[0], undistortedRectifiedKeypoints[0], descriptors[0]);
        extractAndRectify(1, image2, keypoints[1], undistortedKeypoints[1], undistortedRectifiedKeypoints[1], descriptors[1]);
    }
	if ((keypoints[0].size() == 0) || (keypoints[1].size() == 0))
		return FrameworkReturnCode::_ERROR_;
    // stereo feature matching
    std::vector<DescriptorMatch> matches;
    m_stereoMatcher->match(descriptors[0], descriptors[1], undistortedRectifiedKeypoints[0],
        undistortedRectifiedKeypoints[1], m_rectParams[0].type, matches);
    LOG_DEBUG("Number of matches: {}", matches.size());

    // depth estimation
	if (m_isPassRectify[0] && m_isPassRectify[1]) {
		m_stereoDepthEstimator->estimate(undistortedKeypoints[0], undistortedKeypoints[1], matches,
			m_rectParams[0].projection(0, 0), m_rectParams[0].baseline, m_rectParams[0].type);
	}
	else {
		m_stereoDepthEstimator->estimate(undistortedRectifiedKeypoints[0], undistortedRectifiedKeypoints[1], matches,
			m_rectParams[0].projection(0, 0), m_rectParams[0].baseline, m_rectParams[0].type);
		m_stereoReprojector->reprojectToUnrectification(undistortedRectifiedKeypoints[0], m_rectParams[0], undistortedKeypoints[0]);
		m_stereoReprojector->reprojectToUnrectification(undistortedRectifiedKeypoints[1], m_rectParams[1], undistortedKeypoints[1]);
	}

    // make frames
    frame1 = xpcf::utils::make_shared<Frame>(keypoints[0], undistortedKeypoints[0], descriptors[0], image1);
    frame2 = xpcf::utils::make_shared<Frame>(keypoints[1], undistortedKeypoints[1], descriptors[1], image2);
    frame1->setCameraParameters(m_camParams[0]);
    frame2->setCameraParameters(m_camParams[1]);

    return FrameworkReturnCode::_SUCCESS;
}

void SolARStereoFeatureExtractionAndDepthEstimation::extractAndRectify(int indexCamera, SRef<datastructure::Image> image, std::vector<datastructure::Keypoint>& keypoints, std::vector<datastructure::Keypoint>& undistortedKeypoints, std::vector<datastructure::Keypoint>& undistortedRectifiedKeypoints, SRef<datastructure::DescriptorBuffer>& descriptors)
{
    if (m_descriptorExtractor[indexCamera]->extract(image, keypoints, descriptors) != FrameworkReturnCode::_SUCCESS)
        return;
    m_undistortPoints[indexCamera]->undistort(keypoints, m_camParams[indexCamera], undistortedKeypoints);
	if (!m_isPassRectify[indexCamera])
		m_stereoRectificator[indexCamera]->rectify(undistortedKeypoints, m_camParams[indexCamera], m_rectParams[indexCamera], undistortedRectifiedKeypoints);
	else
		undistortedRectifiedKeypoints = undistortedKeypoints;
}

}
}
}
