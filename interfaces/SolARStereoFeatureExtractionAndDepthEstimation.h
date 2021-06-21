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

#ifndef SOLARSTEREOFEATUREEXTRACTIONANDDEPTHESTIMATION_H
#define SOLARSTEREOFEATUREEXTRACTIONANDDEPTHESTIMATION_H
#include "api/stereo/IStereoFeatureExtractionAndDepthEstimation.h"
#include "api/features/IKeypointDetector.h"
#include "api/features/IDescriptorsExtractor.h"
#include "api/geom/IUndistortPoints.h"
#include "api/stereo/IStereoRectification.h"
#include "api/stereo/IStereoDescriptorMatcher.h"
#include "api/stereo/IStereoDepthEstimation.h"
#include "SolARToolsAPI.h"
#include "xpcf/component/ConfigurableBase.h"

namespace SolAR {
namespace MODULES {
namespace TOOLS {

/**
* @class SolARStereoFeatureExtractionAndDepthEstimation
* @brief <B> Perform feature extraction and keypoint depth estimation from each stereo images.</B>
* <TT>UUID: d015129a-6dff-448c-bf02-66f461ff401e</TT>
*
* @SolARComponentInjectablesBegin
* @SolARComponentInjectable{SolAR::api::features::IKeypointDetector}
* @SolARComponentInjectable{SolAR::api::features::IDescriptorsExtractor}
* @SolARComponentInjectable{SolAR::api::geom::IUndistortPoints}
* @SolARComponentInjectable{SolAR::stereo::IStereoRectification}
* @SolARComponentInjectable{SolAR::stereo::IStereoDescriptorMatcher}
* @SolARComponentInjectable{SolAR::stereo::IStereoDepthEstimation}
* @SolARComponentInjectablesEnd
*
*/

class SOLAR_TOOLS_EXPORT_API SolARStereoFeatureExtractionAndDepthEstimation : public org::bcom::xpcf::ConfigurableBase,
	public api::stereo::IStereoFeatureExtractionAndDepthEstimation
{
public:
	///@brief SolARStereoFeatureExtractionAndDepthEstimation constructor;
	SolARStereoFeatureExtractionAndDepthEstimation();

	///@brief SolARStereoFeatureExtractionAndDepthEstimation destructor;
	~SolARStereoFeatureExtractionAndDepthEstimation() override;

	/// @brief this method is used to set intrinsic parameters and distorsion of the camera
	/// @param[in] Camera calibration matrix parameters.
	/// @param[in] Camera distorsion parameters.
	void setCameraParameters(const SolAR::datastructure::CamCalibration & intrinsicParams,
							const SolAR::datastructure::CamDistortion & distortionParams) override;

	/// @brief Perform feature extraction and keypoint depth estimation
	/// @param[in] image1 The first image.
	/// @param[in] image2 The second image.
	/// @param[out] frame1 The first frame that contains features and estimated depths of the first image.
	/// @param[out] frame2 The second frame that contains features and estimated depths of the second image.
	FrameworkReturnCode compute(SRef<SolAR::datastructure::Image> image1,
								SRef<SolAR::datastructure::Image> image2,
								SRef<SolAR::datastructure::Frame> &frame1,
								SRef<SolAR::datastructure::Frame> &frame2) override;
	
	void unloadComponent() override final;

private:
	/// @brief Extract feature and rectify keypoints
	void extractAndRectify(int indexCamera,
						SRef<datastructure::Image> image, 						 
						std::vector<datastructure::Keypoint>& keypoints,
						std::vector<datastructure::Keypoint>& undistortedKeypoints,
						std::vector<datastructure::Keypoint>& undistortedRectifiedKeypoints, 
						SRef<datastructure::DescriptorBuffer>& descriptors);

private:
	std::vector<SRef<SolAR::api::features::IKeypointDetector>>		m_keypointsDetector;
	std::vector<SRef<SolAR::api::features::IDescriptorsExtractor>>	m_descriptorExtractor;
	std::vector<SRef<SolAR::api::geom::IUndistortPoints>>			m_undistortPoints;
	std::vector<SRef<SolAR::api::stereo::IStereoRectification>>		m_stereoRectificator;
	SRef<SolAR::api::stereo::IStereoDescriptorMatcher>				m_stereoMatcher;
	SRef<SolAR::api::stereo::IStereoDepthEstimation>				m_stereoDepthEstimator;
	float															m_focal;
};

}
}
}

#endif // SOLARSTEREOFEATUREEXTRACTIONANDDEPTHESTIMATION_H
