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
#include "api/features/IFeatureWithDepthFromStereo.h"
#include "api/features/IDescriptorsExtractorFromImage.h"
#include "api/geom/IUndistortPoints.h"
#include "api/geom/I2DPointsRectification.h"
#include "api/features/IDescriptorMatcherStereo.h"
#include "api/geom/IDepthEstimation.h"
#include "api/geom/IReprojectionStereo.h"
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
* @SolARComponentInjectable{SolAR::api::features::IDescriptorsExtractorFromImage}
* @SolARComponentInjectable{SolAR::api::geom::IUndistortPoints}
* @SolARComponentInjectable{SolAR::geom::I2DPointsRectification}
* @SolARComponentInjectable{SolAR::features::IDescriptorMatcherStereo}
* @SolARComponentInjectable{SolAR::geom::IDepthEstimation}
* @SolARComponentInjectable{SolAR::geom::IReprojectionStereo}
* @SolARComponentInjectablesEnd
*
*/

class SOLAR_TOOLS_EXPORT_API SolARStereoFeatureExtractionAndDepthEstimation : public org::bcom::xpcf::ConfigurableBase,
    public api::features::IFeatureWithDepthFromStereo
{
public:
	///@brief SolARStereoFeatureExtractionAndDepthEstimation constructor;
	SolARStereoFeatureExtractionAndDepthEstimation();

	///@brief SolARStereoFeatureExtractionAndDepthEstimation destructor;
	~SolARStereoFeatureExtractionAndDepthEstimation() override;

    /// @brief this method is used to set rectification parameters of the stereo camera
    /// @param[in] camParams1 Camera parameters of the first camera.
    /// @param[in] camParams2 Camera parameters of the second camera.
    /// @param[in] rectParams1 Rectification parameters of the first camera.
    /// @param[in] rectParams2 Rectification parameters of the second camera.
    void setRectificationParameters(const SolAR::datastructure::CameraParameters& camParams1,
                                    const SolAR::datastructure::CameraParameters& camParams2,
                                    const SolAR::datastructure::RectificationParameters & rectParams1,
                                    const SolAR::datastructure::RectificationParameters & rectParams2) override;

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
    std::vector<SRef<SolAR::api::features::IDescriptorsExtractorFromImage>>	m_descriptorExtractor;
	std::vector<SRef<SolAR::api::geom::IUndistortPoints>>			m_undistortPoints;
    std::vector<SRef<SolAR::api::geom::I2DPointsRectification>>     m_stereoRectificator;
    SRef<SolAR::api::features::IDescriptorMatcherStereo>            m_stereoMatcher;
    SRef<SolAR::api::geom::IDepthEstimation>                        m_stereoDepthEstimator;
    SRef<SolAR::api::geom::IReprojectionStereo>                     m_stereoReprojector;
	std::vector<SolAR::datastructure::CameraParameters>				m_camParams;
	std::vector<SolAR::datastructure::RectificationParameters>		m_rectParams;
	bool															m_isSetParams = false;
	std::vector<bool>												m_isPassRectify;
    int                                                             m_isMultithreading = 1;
};

}
}
}

#endif // SOLARSTEREOFEATUREEXTRACTIONANDDEPTHESTIMATION_H
