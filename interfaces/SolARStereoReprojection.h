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

#ifndef SOLARSTEREOREPROJECTION_H
#define SOLARSTEREOREPROJECTION_H
#include "base/geom/AReprojectionStereo.h"
#include "SolARToolsAPI.h"

namespace SolAR {
namespace MODULES {
namespace TOOLS {

/**
* @class SolARStereoReprojection
* @brief <B>Reproject keypoints with estimating depth to 3D cloud points.</B>
* <TT>UUID: 6f0c5373-1b00-41ce-ab1b-a845b83f65b3</TT>
*/

class SOLAR_TOOLS_EXPORT_API SolARStereoReprojection : public base::geom::AReprojectionStereo
{
public:
    ///@brief SolARStereoReprojection constructor;
    SolARStereoReprojection();

    ///@brief SolARStereoReprojection destructor;
    ~SolARStereoReprojection() override;

    /// @brief Reproject depth of rectified keypoints to unrectified keypoints
    /// @param[in] rectifiedKeypoints The rectified keypoints containing depth information.
    /// @param[in] rectParams The rectification parameters.
    /// @param[out] unrectifiedKeypoints The unrectified keypoints for estimating depth information.
    /// @return FrameworkReturnCode::_SUCCESS if reprojecting succeed, else FrameworkReturnCode::_ERROR_
    FrameworkReturnCode reprojectToUnrectification(const std::vector<SolAR::datastructure::Keypoint>& rectifiedKeypoints,
                                                   const SolAR::datastructure::RectificationParameters& rectParams,
                                                   std::vector<SolAR::datastructure::Keypoint>& unrectifiedKeypoints) override;

    /// @brief Reproject 2D keypoints with depths to 3D cloud points in the world coordinate system
    /// @param[in] undistortedkeypoints The undistorted keypoints of image.
    /// @param[in] descriptors The descriptors corresponding to the keypoints.
    /// @param[in] pose The pose of image.
    /// @param[in] intrinsicParams The intrinsic parameters of the camera.
    /// @param[out] cloudPoints The output cloud points.
    /// @return FrameworkReturnCode::_SUCCESS if reprojecting succeed, else FrameworkReturnCode::_ERROR_
    FrameworkReturnCode reprojectToCloudPoints(const std::vector<SolAR::datastructure::Keypoint>& undistortedKeypoints,
                                               const SRef<SolAR::datastructure::DescriptorBuffer> descriptors,
                                               const SolAR::datastructure::Transform3Df& pose,
                                               const SolAR::datastructure::CamCalibration& intrinsicParams,
                                               std::vector<SRef<SolAR::datastructure::CloudPoint>>& cloudPoints) override;
	
	void unloadComponent() override final;
};

}
}
}

#endif // SOLARSTEREOREPROJECTION_H
