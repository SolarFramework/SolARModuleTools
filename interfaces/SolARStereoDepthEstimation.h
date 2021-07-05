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
#include "api/geom/IDepthEstimation.h"
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
	public api::geom::IDepthEstimation
{
public:
	///@brief SolARStereoDepthEstimation constructor;
	SolARStereoDepthEstimation();

	///@brief SolARStereoDepthEstimation destructor;
	~SolARStereoDepthEstimation() override;

	/// @brief Depth estimation based on disparity of matched rectified keypoints in a stereo camera
	/// @param[in,out] rectKeypoints1 Rectified keypoints of the first image and the depth estimation is stored in the keypoints.
	/// @param[in,out] rectKeypoints2 Rectified keypoints of the second image and the depth estimation is stored in the keypoints.
	/// @param[in] matches A vector of matches representing pairs of indices relatively to the first and second set of descriptors.
	/// @param[in] focal The common focal of the camera.
	/// @param[in] baseline The baseline distance of two cameras.
	/// @param[in] type Stereo type
	/// @return FrameworkReturnCode::_SUCCESS if estimating succeed, else FrameworkReturnCode::_ERROR_
    FrameworkReturnCode estimate(std::vector<SolAR::datastructure::Keypoint>& rectKeypoints1,
                                 std::vector<SolAR::datastructure::Keypoint>& rectKeypoints2,
                                 const std::vector<SolAR::datastructure::DescriptorMatch>& matches,
                                 const float& focal,
                                 const float& baseline,
                                 const SolAR::datastructure::StereoType& type) override;
	
	void unloadComponent() override final;

private:
	float m_ratioNear = 1.f;
	float m_ratioFar = 0.02f;
};

}
}
}

#endif // SOLARSTEREODEPTHESTIMATION_H
