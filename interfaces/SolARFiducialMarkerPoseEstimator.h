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

#ifndef SOLARFIDUCIALMARKERPOSEESTIMATOR_H
#define SOLARFIDUCIALMARKERPOSEESTIMATOR_H
#include "api/solver/pose/ITrackablePose.h"
#include "api/image/IImageFilter.h"
#include "api/image/IImageConvertor.h"
#include "api/features/IContoursExtractor.h"
#include "api/features/IContoursFilter.h"
#include "api/image/IPerspectiveController.h"
#include "api/features/IDescriptorsExtractorSBPattern.h"
#include "api/features/ISBPatternReIndexer.h"
#include "api/geom/IImage2WorldMapper.h"
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
* @class SolARFiducialMarkerPoseEstimator
* @brief <B>Estimate camera pose based on a fiducial marker.</B>
* <TT>UUID: cddd23c4-da4e-4c5c-b3f9-7d095d097c97</TT>
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
* @SolARComponentInjectable{SolAR::api::geom::IImage2WorldMapper}
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

class SOLAR_TOOLS_EXPORT_API SolARFiducialMarkerPoseEstimator : public org::bcom::xpcf::ConfigurableBase,
    public SolAR::api::solver::pose::ITrackablePose
{
public:
	///@brief SolAR3DTransformEstimationFrom3D3D constructor;
	SolARFiducialMarkerPoseEstimator();

	///@brief SolAR3DTransformEstimationFrom3D3D destructor;
    ~SolARFiducialMarkerPoseEstimator() = default;

    /// @brief this method is used to set the trackable used to estimate the pose.
    /// @param[in] the trackable used to estimate the pose.
    FrameworkReturnCode setTrackable(const SRef<SolAR::datastructure::Trackable> trackable) override;

    /// @brief Estimates camera pose based on a fiducial marker.
    /// @param[in] image input image.
    /// @param[in] camParams the camera parameters.
    /// @param[out] pose camera pose.
    /// @return FrameworkReturnCode::_SUCCESS if the estimation succeed, else FrameworkReturnCode::_ERROR_
    FrameworkReturnCode estimate(const SRef<SolAR::datastructure::Image> image,
                                 const SolAR::datastructure::CameraParameters & camParams,
                                 SolAR::datastructure::Transform3Df & pose) override;

	void unloadComponent() override final;

private:
    SRef<SolAR::datastructure::FiducialMarker>                  m_fiducialMarker;
    SRef<SolAR::api::image::IImageFilter>						m_imageFilterBinary;
    SRef<SolAR::api::image::IImageConvertor>					m_imageConvertor;
    SRef<api::features::IContoursExtractor>                     m_contoursExtractor;
    SRef<SolAR::api::features::IContoursFilter>                 m_contoursFilter;
    SRef<SolAR::api::image::IPerspectiveController>             m_perspectiveController;
    SRef<SolAR::api::features::IDescriptorsExtractorSBPattern>	m_patternDescriptorExtractor;
    SRef<SolAR::api::features::IDescriptorMatcher>				m_patternMatcher;
    SRef<SolAR::api::features::ISBPatternReIndexer>             m_patternReIndexer;
    SRef<SolAR::api::geom::IImage2WorldMapper>					m_img2worldMapper;
    SRef<SolAR::api::solver::pose::I3DTransformFinderFrom2D3D>	m_pnp;
    SRef<SolAR::api::features::ICornerRefinement>				m_cornerRefinement;
    SRef<SolAR::api::geom::IProject>							m_projector;
	SRef<SolAR::datastructure::DescriptorBuffer>				m_markerPatternDescriptor;
    int                                                         m_nbThreshold = 3;
    int                                                         m_minThreshold = -1;
    int                                                         m_maxThreshold = 220;
    float                                                       m_maxReprojError = 0.5f;
};

}
}
}

#endif // SOLARFIDUCIALMARKERPOSEESTIMATOR_H
