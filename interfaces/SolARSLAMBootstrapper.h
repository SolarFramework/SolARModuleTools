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

#ifndef SOLARSLAMBOOTSTRAPPER_H
#define SOLARSLAMBOOTSTRAPPER_H
#include "api/slam/IBootstrapper.h"
#include "datastructure/Image.h"
#include "api/storage/IMapManager.h"
#include "api/image/IImageFilter.h"
#include "api/features/IKeypointDetector.h"
#include "api/features/IDescriptorsExtractor.h"
#include "api/features/IDescriptorMatcher.h"
#include "api/features/IMatchesFilter.h"
#include "api/solver/map/ITriangulator.h"
#include "api/solver/map/IMapFilter.h"
#include "api/solver/map/IKeyframeSelector.h"
#include "api/display/IMatchesOverlay.h"
#include "api/solver/pose/I3DTransformFinderFrom2D2D.h"
#include "api/geom/IUndistortPoints.h"
#include "SolARToolsAPI.h"
#include "xpcf/component/ConfigurableBase.h"

namespace SolAR {
namespace MODULES {
namespace TOOLS {

/**
* @class SolARSLAMBootstrapper
* @brief <B>Initialization SLAM using an image stream of a camera.</B>
* <TT>UUID: 8f43eed0-1a2e-4c47-83f0-8dd5b259cdb0</TT>
*
* @SolARComponentInjectablesBegin
* @SolARComponentInjectable{SolAR::api::storage::IMapManager}
* @SolARComponentInjectable{SolAR::api::image::IImageFilter}
* @SolARComponentInjectable{SolAR::api::features::IKeypointDetector}
* @SolARComponentInjectable{SolAR::api::features::IDescriptorsExtractor}
* @SolARComponentInjectable{SolAR::api::features::IDescriptorMatcher}
* @SolARComponentInjectable{SolAR::api::features::IMatchesFilter}
* @SolARComponentInjectable{SolAR::api::solver::map::ITriangulator}
* @SolARComponentInjectable{SolAR::api::solver::map::IMapFilter}
* @SolARComponentInjectable{SolAR::api::solver::map::IKeyframeSelector}
* @SolARComponentInjectable{SolAR::api::solver::pose::I3DTransformFinderFrom2D2D}
* @SolARComponentInjectable{SolAR::api::display::IMatchesOverlay}
* @SolARComponentInjectablesEnd
*
* @SolARComponentPropertiesBegin
* @SolARComponentProperty{ hasPose,
*                          (0 = false\, 1 = true),
*                          @SolARComponentPropertyDescNum{ int, [0\, 1], 1 }}
* @SolARComponentProperty{ nbMinInitPointCloud,
*                          ,
*                          @SolARComponentPropertyDescNum{ int, [0..MAX INT], 50 }}
* @SolARComponentProperty{ angleThres,
*                          ,
*                          @SolARComponentPropertyDescNum{ float, [0..2*PI], 0.1f }}
* @SolARComponentPropertiesEnd
*
*/

class SOLAR_TOOLS_EXPORT_API SolARSLAMBootstrapper : public org::bcom::xpcf::ConfigurableBase,
    public SolAR::api::slam::IBootstrapper
{
public:
	///@brief SolAR3DTransformEstimationFrom3D3D constructor;
	SolARSLAMBootstrapper();
	///@brief SolAR3DTransformEstimationFrom3D3D destructor;
	~SolARSLAMBootstrapper() = default;
	
	/// @brief this method is used to set intrinsic parameters and distorsion of the camera
	/// @param[in] Camera calibration matrix parameters.
	/// @param[in] Camera distorsion parameters.
	void setCameraParameters(const SolAR::datastructure::CamCalibration & intrinsicParams, const SolAR::datastructure::CamDistortion & distorsionParams) override;

	/// @brief This method uses images to boostrap
	/// @param[in] image: input image to process
	/// @param[out] view: output image to visualize
	/// @param[in] pose: the pose of the input image
	/// @return FrameworkReturnCode::_SUCCESS_ if initialization succeed, else FrameworkReturnCode::_ERROR.
    FrameworkReturnCode process(const SRef<SolAR::datastructure::Image> image, SRef<SolAR::datastructure::Image> & view, const SolAR::datastructure::Transform3Df & pose = SolAR::datastructure::Transform3Df::Identity()) override;

	void unloadComponent() override final;
	org::bcom::xpcf::XPCFErrorCode onConfigured() override final;

private:
	/// bootstrap uses marker
	FrameworkReturnCode initFiducialMarker();
	/// bootstrap doesn't use marker
	FrameworkReturnCode initMarkerLess();

private:
    int                                                         m_hasPose = 1;
    int                                                         m_nbMinInitPointCloud = 50;
    float                                                       m_angleThres = 0.1f;
    float                                                       m_ratioDistanceIsKeyframe = 0.05f;
    bool                                                        m_bootstrapOk = false;
    bool                                                        m_initKeyframe1 = false;
    SRef<SolAR::datastructure::Keyframe>                        m_keyframe1, m_keyframe2;
    SolAR::datastructure::CamCalibration                        m_camMatrix;
    SolAR::datastructure::CamDistortion                         m_camDistortion;
    SRef<SolAR::api::storage::IMapManager>						m_mapManager;
    SRef<SolAR::api::image::IImageFilter>                       m_imageFilter;
    SRef<SolAR::api::features::IKeypointDetector>				m_keypointsDetector;
    SRef<SolAR::api::features::IDescriptorsExtractor>			m_descriptorExtractor;
    SRef<SolAR::api::features::IDescriptorMatcher>				m_matcher;
    SRef<SolAR::api::features::IMatchesFilter>					m_matchesFilter;
    SRef<SolAR::api::solver::map::ITriangulator>				m_triangulator;
    SRef<SolAR::api::solver::map::IMapFilter>					m_mapFilter;
    SRef<SolAR::api::solver::map::IKeyframeSelector>			m_keyframeSelector;
    SRef<SolAR::api::solver::pose::I3DTransformFinderFrom2D2D>	m_poseFinderFrom2D2D;
    SRef<SolAR::api::geom::IUndistortPoints>					m_undistortPoints;
    SRef<SolAR::api::display::IMatchesOverlay>					m_matchesOverlay;
};

}
}
}

#endif // SOLARSLAMBOOTSTRAPPER_H
