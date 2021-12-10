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
#include "api/image/IImageFilter.h"
#include "api/image/IImageConvertor.h"
#include "api/features/IContoursExtractor.h"
#include "api/features/IContoursFilter.h"
#include "api/image/IPerspectiveController.h"
#include "api/features/IDescriptorsExtractorSBPattern.h"
#include "api/features/ISBPatternReIndexer.h"
#include "api/geom/IProject.h"
#include "api/solver/pose/I3DTransformFinderFrom2D3D.h"
#include "api/features/ICornerRefinement.h"
#include "datastructure/Image.h"
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
* @SolARComponentInjectable{SolAR::api::image::IImageFilter, optional}
* @SolARComponentInjectable{SolAR::api::image::IImageConvertor}
* @SolARComponentInjectable{SolAR::api::features::IContoursExtractor}
* @SolARComponentInjectable{SolAR::api::features::IContoursFilter}
* @SolARComponentInjectable{SolAR::api::image::IPerspectiveController}
* @SolARComponentInjectable{SolAR::api::features::IDescriptorsExtractorSBPattern}
* @SolARComponentInjectable{SolAR::api::features::IDescriptorMatcher}
* @SolARComponentInjectable{SolAR::api::features::ISBPatternReIndexer}
* @SolARComponentInjectable{SolAR::api::solver::pose::I3DTransformFinderFrom2D3D}
* @SolARComponentInjectable{SolAR::api::features::ICornerRefinement}
* @SolARComponentInjectablesEnd
*
* @SolARComponentPropertiesBegin
* @SolARComponentProperty{ nbThreshold,
*                         ,
*                         @SolARComponentPropertyDescNum{ int, [0..MAX INT], 3 }}
* @SolARComponentProperty{ minThreshold,
*                          ,
*                          @SolARComponentPropertyDescNum{ int, [-1..MAX INT], -1 }}
* @SolARComponentProperty{ maxThreshold,
*                          ,
*                          @SolARComponentPropertyDescNum{ int, [0..MAX INT], 220 }}
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
    SRef<SolAR::api::image::IImageFilter>						m_imageFilterBinary;
    SRef<SolAR::api::image::IImageConvertor>					m_imageConvertor;
    SRef<api::features::IContoursExtractor>                     m_contoursExtractor;
    SRef<SolAR::api::features::IContoursFilter>                 m_contoursFilter;
    SRef<SolAR::api::image::IPerspectiveController>             m_perspectiveController;
    SRef<SolAR::api::features::IDescriptorsExtractorSBPattern>	m_patternDescriptorExtractor;
    SRef<SolAR::api::features::IDescriptorMatcher>				m_patternMatcher;
    SRef<SolAR::api::features::ISBPatternReIndexer>             m_patternReIndexer;
    SRef<SolAR::api::solver::pose::I3DTransformFinderFrom2D3D>	m_pnp;
    SRef<SolAR::api::features::ICornerRefinement>				m_cornerRefinement;
    SRef<SolAR::api::geom::IProject>							m_projector;
	std::map<int, std::vector<SRef<SolAR::datastructure::DescriptorBuffer>>> m_markerPatternDescriptors;
	std::map<int, std::vector<std::vector<SolAR::datastructure::Point3Df>>>	 m_pattern3DPoints;
	int															m_nbMarkers;
    int                                                         m_nbThreshold = 3;
    int                                                         m_minThreshold = -1;
    int                                                         m_maxThreshold = 220;
    float                                                       m_maxReprojError = 0.5f;
};

}
}
}

#endif // SOLARMULTIFIDUCIALMARKERSPOSEESTIMATOR_H
