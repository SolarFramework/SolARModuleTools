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

#ifndef SOLAR3DTRANSFORMESTIMATIONSACFROM3D3D_H
#define SOLAR3DTRANSFORMESTIMATIONSACFROM3D3D_H
#include <vector>
#include <string>
#include "api/solver/pose/I3DTransformSACFinderFrom3D3D.h"
#include "api/geom/I3DTransform.h"
#include "api/geom/IProject.h"
#include "datastructure/Image.h"
#include "api/solver/map/IBundler.h"
#include "SolARToolsAPI.h"
#include "xpcf/component/ConfigurableBase.h"

namespace SolAR {
namespace MODULES {
namespace TOOLS {

/**
* @class SolAR3DTransformEstimationSACFrom3D3D
* @brief <B>Finds the 3D transform of 3D-3D points correspondences with a SAmple Consensus.</B>
* <TT>UUID: 3b7a1117-8b59-46b1-8e0c-6e76a8377ab4</TT>
*
* @SolARComponentInjectablesBegin
* @SolARComponentInjectable{SolAR::api::geom::I3DTransform}
* @SolARComponentInjectable{SolAR::api::geom::IProject}
* @SolARComponentInjectablesEnd
*
* @SolARComponentPropertiesBegin
* @SolARComponentProperty{ iterationsCount,
*                          number of iterations,
*                          @SolARComponentPropertyDescNum{ int, [0..MAX INT], 500 }}
* @SolARComponentProperty{ reprojError,
*                          inlier threshold value used by the RANSAC procedure.<br>
*                           The parameter value is the maximum allowed distance between 
*                           the observed and computed point projections to consider it an inlier,
*                          @SolARComponentPropertyDescNum{ float, [0..MAX FLOAT], 4.f }}
* @SolARComponentProperty{ distanceError,
*                          inlier threshold value based on 3d distance error,
*                          @SolARComponentPropertyDescNum{ float, [0..MAX FLOAT], 0.1f }}
* @SolARComponentProperty{ confidence,
*                          the probability that the algorithm produces a useful result,
*                          @SolARComponentPropertyDescNum{ float, [0..1], 0.99f }}
* @SolARComponentProperty{ minNbInliers,
*                          the minimum of number of inliers to valid a good pose estimation,
*                          @SolARComponentPropertyDescNum{ int, [0..MAX INT], 10 }}
* @SolARComponentPropertiesEnd
*
*/
class SOLAR_TOOLS_EXPORT_API SolAR3DTransformEstimationSACFrom3D3D : public org::bcom::xpcf::ConfigurableBase,
    public SolAR::api::solver::pose::I3DTransformSACFinderFrom3D3D
{
public:
    ///@brief SolAR3DTransformEstimationFrom3D3D constructor;
	SolAR3DTransformEstimationSACFrom3D3D();
    ///@brief SolAR3DTransformEstimationFrom3D3D destructor;
    ~SolAR3DTransformEstimationSACFrom3D3D() = default;

	/// @brief Estimates camera pose from a set of 3D-3D point correspondences.
	/// @param[in] firstPoints3D: first set of 3D points.
	/// @param[in] secondPoints3D: second set of 3D points.
	/// @param[out] pose: 3D transformation maps the first set of 3D points to the second one.
	/// @param[out] inliers: indices of inlier correspondences.
	FrameworkReturnCode estimate(const std::vector<SolAR::datastructure::Point3Df> & firstPoints3D,
								const std::vector<SolAR::datastructure::Point3Df> & secondPoints3D,
                                SolAR::datastructure::Transform3Df & pose,
								std::vector<int> &inliers) override;

	/// @brief Estimates camera pose from a set of 3D-3D point correspondences.
	/// @param[in] firstKeyframe: first keyframe.
	/// @param[in] secondKeyframe: second keyframe.
	/// @param[in] matches: matches between two keyframes.
	/// @param[in] firstPoints3D: first set of 3D points.
	/// @param[in] secondPoints3D: second set of 3D points.
	/// @param[out] pose: 3D transformation maps the first set of 3D points to the second one.
	/// @param[out] inliers: indices of inlier correspondences.
    FrameworkReturnCode estimate(const SRef<SolAR::datastructure::Keyframe> firstKeyframe,
                                const SRef<SolAR::datastructure::Keyframe> secondKeyframe,
								const std::vector<SolAR::datastructure::DescriptorMatch> &matches,
								const std::vector<SolAR::datastructure::Point3Df> & firstPoints3D,
								const std::vector<SolAR::datastructure::Point3Df> & secondPoints3D,
                                SolAR::datastructure::Transform3Df & pose,
								std::vector<int> &inliers) override;

    void unloadComponent () override final;

private:
	/// @brief Number of iterations
	int m_iterationsCount = 500;

	/// @brief Inlier threshold value used by the RANSAC procedure. The parameter value is the maximum allowed distance between the observed and computed point projections to consider it an inlier.
    float m_reprojError = 4.0f;

	/// @brief Inlier threshold value based on 3d distance error
    float m_distanceError = 0.1f;

	/// @brief The probability that the algorithm produces a useful result.
	float m_confidence = 0.99f;

	/// @brief The minimum of number of inliers to valid a good pose estimation
	/// @brief Does optimize sim3 using graph optimization
	int m_optimizeSim3 = 0;	
	int m_NbInliersToValidPose = 10;

	/// @brief Transform 3D
    SRef<SolAR::api::geom::I3DTransform> m_transform3D;

	/// @brief Projector
	SRef<SolAR::api::geom::IProject> m_projector;
	
	/// @brief Bundler
    SRef<api::solver::map::IBundler> m_bundler;
};

}
}
}

#endif // SOLAR3DTRANSFORMESTIMATIONSACFROM3D3D_H
