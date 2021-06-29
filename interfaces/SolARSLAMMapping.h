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

#ifndef SOLARSLAMMAPPING_H
#define SOLARSLAMMAPPING_H
#include "api/slam/IMapping.h"
#include "api/storage/IMapManager.h"
#include "api/storage/ICovisibilityGraphManager.h"
#include "api/storage/IKeyframesManager.h"
#include "api/storage/IPointCloudManager.h"
#include "api/solver/map/IBundler.h"
#include "api/reloc/IKeyframeRetriever.h"
#include "api/features/IMatchesFilter.h"
#include "api/solver/map/ITriangulator.h"
#include "api/solver/map/IMapFilter.h"
#include "api/geom/IProject.h"
#include "api/features/IDescriptorMatcher.h"
#include "api/solver/pose/I2D3DCorrespondencesFinder.h"
#include "SolARToolsAPI.h"
#include "xpcf/component/ConfigurableBase.h"

namespace SolAR {
namespace MODULES {
namespace TOOLS {

/**
* @class SolARSLAMMapping
* @brief <B> SLAM mapping.</B>
* <TT>UUID: c276bcb1-2ac8-42f2-806d-d4fe0ce7d4be</TT>
*
* @SolARComponentInjectablesBegin
* @SolARComponentInjectable{SolAR::api::storage::IMapManager}
* @SolARComponentInjectable{SolAR::api::storage::IPointCloudManager}
* @SolARComponentInjectable{SolAR::api::storage::IKeyframesManager}
* @SolARComponentInjectable{SolAR::api::storage::ICovisibilityGraphManager}
* @SolARComponentInjectable{SolAR::api::solver::map::IBundler}
* @SolARComponentInjectable{SolAR::api::reloc::IKeyframeRetriever}
* @SolARComponentInjectable{SolAR::api::features::IMatchesFilter}
* @SolARComponentInjectable{SolAR::api::solver::map::ITriangulator}
* @SolARComponentInjectable{SolAR::solver::map::IMapFilter}
* @SolARComponentInjectable{SolAR::api::geom::IProject}
* @SolARComponentInjectable{SolAR::api::features::IDescriptorMatcher}
* @SolARComponentInjectable{SolAR::api::solver::pose::I2D3DCorrespondencesFinder}
* @SolARComponentInjectablesEnd
*
* @SolARComponentPropertiesBegin
* @SolARComponentProperty{ minWeightNeighbor,
*                          ,
*                          @SolARComponentPropertyDescNum{ float, [0..MAX FLOAT], 1.f }}
* @SolARComponentProperty{ maxNbNeighborKfs,
*                          ,
*                          @SolARComponentPropertyDescNum{ int, [0..MAX INT], 5 }}
* @SolARComponentProperty{ minTrackedPoints,
*                          ,
*                          @SolARComponentPropertyDescNum{ int, [0..MAX INT], 100 }}
* @SolARComponentPropertiesEnd
*
*
*/

class SOLAR_TOOLS_EXPORT_API SolARSLAMMapping : public org::bcom::xpcf::ConfigurableBase,
    public SolAR::api::slam::IMapping
{
public:
	///@brief SolARSLAMMapping constructor;
	SolARSLAMMapping();

	///@brief SolARSLAMMapping destructor;
	~SolARSLAMMapping() = default;

	/// @brief this method is used to set intrinsic parameters and distorsion of the camera
	/// @param[in] Camera calibration matrix parameters.
	/// @param[in] Camera distorsion parameters.
	void setCameraParameters(const SolAR::datastructure::CamCalibration & intrinsicParams, const SolAR::datastructure::CamDistortion & distorsionParams) override;

	/// @brief this method is used to process mapping task.
	/// @param[in] frame: the input frame.
    /// @param[out] keyframe: new keyframe or new reference keyframe found.
    FrameworkReturnCode process(const SRef<SolAR::datastructure::Frame> frame, SRef<SolAR::datastructure::Keyframe> & keyframe) override;

	void unloadComponent() override final;

private:
	SRef<SolAR::datastructure::Keyframe> processNewKeyframe(const SRef<SolAR::datastructure::Frame> &frame);
	bool checkNeedNewKeyframeInLocalMap(const SRef<SolAR::datastructure::Frame> &frame);
	void updateAssociateCloudPoint(const SRef<SolAR::datastructure::Keyframe> &keyframe);
	void findMatchesAndTriangulation(const SRef<SolAR::datastructure::Keyframe> & keyframe, const std::vector<uint32_t> &idxBestNeighborKfs, std::vector<SRef<SolAR::datastructure::CloudPoint>> &cloudPoint);
	void fuseCloudPoint(const SRef<SolAR::datastructure::Keyframe> &keyframe, const std::vector<uint32_t> &idxNeigborKfs, std::vector<SRef<SolAR::datastructure::CloudPoint>> &newCloudPoint);
	void cloudPointsCulling(const SRef<SolAR::datastructure::Keyframe> &keyframe);

private:
	float																		m_minWeightNeighbor = 1.f;
	int																			m_minTrackedPoints = 200;
	int																			m_maxNbNeighborKfs = 5;
	int																			m_nbPassedFrames = 0;
	int																			m_nbVisibilityAtLeast = 30;
	int																			m_nbPassedFrameAtLeast = 5;
	float																		m_ratioCPRefKeyframe = 0.5;
    SRef<SolAR::datastructure::Keyframe>										m_updatedReferenceKeyframe;
    SolAR::datastructure::CamCalibration										m_camMatrix;
    SolAR::datastructure::CamDistortion											m_camDistortion;
    SRef<SolAR::api::storage::ICovisibilityGraphManager>                        m_covisibilityGraphManager;
    SRef<SolAR::api::storage::IKeyframesManager>								m_keyframesManager;
    SRef<SolAR::api::solver::map::IBundler>										m_bundler;
    SRef<SolAR::api::reloc::IKeyframeRetriever>									m_keyframeRetriever;
    SRef<SolAR::api::storage::IMapManager>                                      m_mapManager;
    SRef<SolAR::api::storage::IPointCloudManager>								m_pointCloudManager;
    SRef<SolAR::api::features::IMatchesFilter>									m_matchesFilter;
    SRef<SolAR::api::solver::map::ITriangulator>								m_triangulator;
    SRef<SolAR::api::solver::map::IMapFilter>									m_mapFilter;
    SRef<SolAR::api::geom::IProject>											m_projector;
    SRef<SolAR::api::features::IDescriptorMatcher>								m_matcher;
    SRef<SolAR::api::solver::pose::I2D3DCorrespondencesFinder>					m_corr2D3DFinder;
	std::map<uint32_t, std::pair<SRef<SolAR::datastructure::CloudPoint>, uint32_t>>	m_recentAddedCloudPoints;
};

}
}
}

#endif // SOLARSLAMMAPPING_H
