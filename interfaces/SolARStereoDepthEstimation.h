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

#ifndef SOLARSTEREODEPTHESTIMATION_H
#define SOLARSTEREODEPTHESTIMATION_H
#include "api/stereo/IStereoDepthEstimation.h"
#include "SolARToolsAPI.h"
#include "xpcf/component/ConfigurableBase.h"

namespace SolAR {
namespace MODULES {
namespace TOOLS {

/**
* @class SolARStereoDepthEstimation
* @brief <B> Depth estimation based on disparity of matched features.</B>
* <TT>UUID: 153ffeaf-7583-44a7-bb7a-3474ba7d99cb</TT>
*/

class SOLAR_TOOLS_EXPORT_API SolARStereoDepthEstimation : public org::bcom::xpcf::ConfigurableBase,
	public api::stereo::IStereoDepthEstimation
{
public:
	///@brief SolARStereoDepthEstimation constructor;
	SolARStereoDepthEstimation();

	///@brief SolARStereoDepthEstimation destructor;
	~SolARStereoDepthEstimation() override;

	/// @brief Depth estimation based on disparity of matched keypoints
	/// @param[in,out] keypoints1 Keypoints of the first image.
	/// @param[in,out] keypoints2 Keypoints of the second image.
	/// @param[in] matches A vector of matches representing pairs of indices relatively to the first and second set of descriptors.
	/// @param[in] focal The common focal of the camera.
	/// @param[in] baseline The baseline distance of two cameras.
	/// @param[in] type Stereo type
	void estimate(std::vector<SolAR::datastructure::Keypoint>& keypoints1,
				std::vector<SolAR::datastructure::Keypoint>& keypoints2,
				const std::vector<SolAR::datastructure::DescriptorMatch>& matches,
				const float& focal,
				const float& baseline,
				const SolAR::datastructure::StereoType& type) override;

	/// @brief Depth estimation of unrectified keypoints based on depths of rectified keypoints
	/// @param[in] rectifiedKeypoints The rectified keypoints containing depth information.
	/// @param[in,out] unrectifiedKeypoints The unrectified keypoints for estimating depth information.
	/// @param[in] rectParams The rectification parameters.
	void estimate(const std::vector<SolAR::datastructure::Keypoint>& rectifiedKeypoints,
				std::vector<SolAR::datastructure::Keypoint>& unrectifiedKeypoints,
				const SolAR::datastructure::RectificationParameters& rectParams) override;

	/// @brief Reproject 2D points with depths of a frame to 3D cloud points in the world coordinate system
	/// @param[in] frame The frame.
	/// @param[in] intrinsicParams The intrinsic parameters of the camera.
	/// @param[out] cloudPoints The output cloud points.
	void reprojectToCloudPoints(SRef<SolAR::datastructure::Frame> frame,
								const SolAR::datastructure::CamCalibration& intrinsicParams,
								std::vector<SRef<SolAR::datastructure::CloudPoint>>& cloudPoints) override;

	void unloadComponent() override final;

private:
	float m_ratioNear = 1.f;
	float m_ratioFar = 0.02f;
};

}
}
}

#endif // SOLARSTEREODEPTHESTIMATION_H
