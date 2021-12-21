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

#ifndef SOLARSLAMTRACKING_H
#define SOLARSLAMTRACKING_H
#include "api/slam/ITracking.h"
#include "datastructure/Image.h"
#include "api/storage/IKeyframesManager.h"
#include "api/storage/IMapManager.h"
#include "api/features/IDescriptorMatcher.h"
#include "api/features/IDescriptorMatcherRegion.h"
#include "api/features/IMatchesFilter.h"
#include "api/display/I2DOverlay.h"
#include "api/solver/pose/I2D3DCorrespondencesFinder.h"
#include "api/solver/pose/I3DTransformFinderFrom2D3D.h"
#include "api/solver/pose/I3DTransformSACFinderFrom2D3D.h"
#include "api/geom/IProject.h"
#include "api/reloc/IKeyframeRetriever.h"
#include "SolARToolsAPI.h"
#include "xpcf/component/ConfigurableBase.h"

namespace SolAR {
namespace MODULES {
namespace TOOLS {

/**
* @class SolARSLAMTracking
* @brief <B> SLAM tracking task.</B>
* <TT>UUID: c45da19d-9637-48b6-ab52-33d3f0af6f72</TT>
*
* @SolARComponentInjectablesBegin
* @SolARComponentInjectable{SolAR::api::storage::IMapManager}
* @SolARComponentInjectable{SolAR::api::storage::IKeyframesManager}
* @SolARComponentInjectable{SolAR::api::features::IDescriptorMatcher}
* @SolARComponentInjectable{SolAR::api::features::IDescriptorMatcherRegion}
* @SolARComponentInjectable{SolAR::api::features::IMatchesFilter}
* @SolARComponentInjectable{SolAR::api::solver::pose::I2D3DCorrespondencesFinder}
* @SolARComponentInjectable{SolAR::api::solver::pose::I3DTransformFinderFrom2D3D}
* @SolARComponentInjectable{SolAR::api::solver::pose::I3DTransformSACFinderFrom2D3D}
* @SolARComponentInjectable{SolAR::api::geom::IProject}
* @SolARComponentInjectable{SolAR::api::reloc::IKeyframeRetriever}
* @SolARComponentInjectable{SolAR::api::display::I2DOverlay}
* @SolARComponentInjectablesEnd
*
* @SolARComponentPropertiesBegin
* @SolARComponentProperty{ minWeightNeighbor,
*                          ,
*                          @SolARComponentPropertyDescNum{ float, [0..MAX FLOAT], 10.f }}
* @SolARComponentProperty{ thresAngleViewDirection,
*                          ,
*                          @SolARComponentPropertyDescNum{ float, [0..MAX FLOAT], 0.7f }}
* @SolARComponentProperty{ displayTrackedPoints,
*                          ,
*                          @SolARComponentPropertyDescNum{ int, [0..MAX INT], 1 }}
* @SolARComponentPropertiesEnd
*
*/

class SOLAR_TOOLS_EXPORT_API SolARSLAMTracking : public org::bcom::xpcf::ConfigurableBase,
    public SolAR::api::slam::ITracking
{
public:
	///@brief SolARSLAMTracking constructor;
	SolARSLAMTracking();

	///@brief SolARSLAMTracking destructor;
	~SolARSLAMTracking() = default;

	org::bcom::xpcf::XPCFErrorCode onConfigured() override final;

	/// @brief this method is used to set intrinsic parameters and distorsion of the camera
	/// @param[in] Camera calibration matrix parameters.
	/// @param[in] Camera distorsion parameters.
	void setCameraParameters(const SolAR::datastructure::CamCalibration & intrinsicParams, const SolAR::datastructure::CamDistortion & distorsionParams) override;

	/// @brief this method is used to update reference keyframe to track
	/// @param[in] refKeyframe: the new reference keyframe.
    void updateReferenceKeyframe(const SRef<SolAR::datastructure::Keyframe> refKeyframe) override;

	/// @brief this method is used to process tracking
	/// @param[in] frame: the input frame.
	/// @param[out] displayImage: the image to display.
	/// @return FrameworkReturnCode::_SUCCESS if tracking succeed, else FrameworkReturnCode::_ERROR_
    FrameworkReturnCode process(const SRef<SolAR::datastructure::Frame> frame, SRef<SolAR::datastructure::Image> &displayImage) override;

	void unloadComponent() override final;

private:
	void updateLocalMap();

private:
    SRef<SolAR::datastructure::Keyframe>                            m_referenceKeyframe;
    SolAR::datastructure::Transform3Df                              m_lastPose = SolAR::datastructure::Transform3Df::Identity();
    std::vector<SRef<SolAR::datastructure::CloudPoint>>             m_localMap;
    bool                                                            m_isLostTrack = false;
    float                                                           m_minWeightNeighbor = 10.f;
    float                                                           m_thresAngleViewDirection = 0.7f;
    float                                                           m_reprojErrorThreshold;
    float                                                           m_thresConfidence;
    int                                                             m_displayTrackedPoints = 1;
    int                                                             m_estimatedPose = 0;
    bool                                                            m_isUpdateReferenceKeyframe = false;
    std::mutex                                                      m_refKeyframeMutex;
    SolAR::datastructure::CamCalibration                            m_camMatrix;
    SolAR::datastructure::CamDistortion                             m_camDistortion;
    SRef<SolAR::api::storage::IMapManager>                          m_mapManager;
    SRef<SolAR::api::features::IDescriptorMatcher>                  m_matcher;
    SRef<SolAR::api::features::IDescriptorMatcherRegion>            m_matcherRegion;
    SRef<SolAR::api::features::IMatchesFilter>                      m_matchesFilter;
    SRef<api::display::I2DOverlay>                                  m_overlay2DGreen, m_overlay2DRed;
    SRef<SolAR::api::solver::pose::I2D3DCorrespondencesFinder>      m_corr2D3DFinder;
    SRef<SolAR::api::solver::pose::I3DTransformSACFinderFrom2D3D>	m_pnpRansac;
    SRef<SolAR::api::solver::pose::I3DTransformFinderFrom2D3D>      m_pnp;
    SRef<SolAR::api::geom::IProject>                                m_projector;
    SRef<SolAR::api::reloc::IKeyframeRetriever>                     m_keyframeRetriever;
    SRef<SolAR::api::storage::IKeyframesManager>                    m_keyframesManager;
    SRef<SolAR::api::storage::IPointCloudManager>                   m_pointCloudManager;
};

}
}
}

#endif // SOLARSLAMTRACKING_H
