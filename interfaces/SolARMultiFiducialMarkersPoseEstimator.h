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

#ifndef SOLARMULTIFIDUCIALMARKERSPOSEESTIMATOR_H
#define SOLARMULTIFIDUCIALMARKERSPOSEESTIMATOR_H
#include "api/solver/pose/IMultiTrackablesPose.h"
#include "api/features/I2DTrackablesDetector.h"
#include "api/geom/IProject.h"
#include "api/solver/pose/I3DTransformFinderFrom2D3D.h"
#include "datastructure/FiducialMarker.h"
#include "SolARToolsAPI.h"
#include "xpcf/component/ConfigurableBase.h"

namespace SolAR {
namespace MODULES {
namespace TOOLS {

/**
* @class SolARMultiFiducialMarkersPoseEstimator
* @brief <B>Estimate camera pose based on a set of fiducial markers.</B>
* <TT>UUID: 9a4521de-2ea5-48f4-97ba-7e698a426076</TT>
*
* @SolARComponentInjectablesBegin
* @SolARComponentInjectable{SolAR::api::solver::pose::I3DTransformFinderFrom2D3D}
* @SolARComponentInjectable{SolAR::api::geom::IProject}
* @SolARComponentInjectable{SolAR::api::features::I2DTrackablesDetector}
* @SolARComponentInjectablesEnd
*
* @SolARComponentPropertiesBegin
* @SolARComponentProperty{ m_maxReprojError,
*                         ,
*                         @SolARComponentPropertyDescNum{ float, [0..MAX FLOAT], 1.0 }}
* @SolARComponentPropertiesEnd
*
*/

class SOLAR_TOOLS_EXPORT_API SolARMultiFiducialMarkersPoseEstimator : public org::bcom::xpcf::ConfigurableBase,
    public SolAR::api::solver::pose::IMultiTrackablesPose
{
public:
    ///@brief SolARMultiFiducialMarkersPoseEstimator constructor;
    SolARMultiFiducialMarkersPoseEstimator();
    ///@brief SolARMultiFiducialMarkersPoseEstimator destructor;
    ~SolARMultiFiducialMarkersPoseEstimator() = default;

	/// @brief this method is used to set intrinsic parameters and distorsion of the camera
    /// @param[in] intrinsicParams Camera calibration matrix parameters.
    /// @param[in] distorsionParams Camera distorsion parameters.
	void setCameraParameters(const SolAR::datastructure::CamCalibration & intrinsicParams, const SolAR::datastructure::CamDistortion & distorsionParams) override;;

    /// @brief this method is used to set the trackable used to estimate the pose.
    /// @param[in] trackables the set of trackables used to estimate the pose.
    FrameworkReturnCode setTrackables(const std::vector<SRef<SolAR::datastructure::Trackable>> trackables) override;

    /// @brief Estimates camera pose based on a fiducial marker.
    /// @param[in] image input image.
    /// @param[out] pose camera pose.
	/// @return FrameworkReturnCode::_SUCCESS if the estimation succeed, else FrameworkReturnCode::_ERROR_
    FrameworkReturnCode estimate(const SRef<SolAR::datastructure::Image> image, SolAR::datastructure::Transform3Df & pose) override;

	void unloadComponent() override final;

private:
    SolAR::datastructure::CamCalibration						m_camMatrix;
    SolAR::datastructure::CamDistortion                         m_camDistortion;
    SRef<SolAR::api::solver::pose::I3DTransformFinderFrom2D3D>	m_pnp;
    SRef<SolAR::api::features::I2DTrackablesDetector>           m_markersDetector;
    SRef<SolAR::api::geom::IProject>							m_projector;
    std::vector<std::vector<SolAR::datastructure::Point3Df>>	m_pattern3DPoints;
    int															m_nbMarkers;
    float                                                       m_maxReprojError = 1.0f;
};

}
}
}

#endif // SOLARMULTIFIDUCIALMARKERSPOSEESTIMATOR_H
