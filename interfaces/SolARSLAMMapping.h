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
#include "api/reloc/IKeyframeRetriever.h"
#include "api/solver/map/ITriangulator.h"
#include "api/solver/map/IMapFilter.h"
#include "api/features/IDescriptorMatcherGeometric.h"
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
* @SolARComponentInjectable{SolAR::api::reloc::IKeyframeRetriever}
* @SolARComponentInjectable{SolAR::api::solver::map::ITriangulator}
* @SolARComponentInjectable{SolAR::solver::map::IMapFilter}
* @SolARComponentInjectable{SolAR::api::features::IDescriptorMatcherGeometric}
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

    /// @brief check the mapping process is idle
    /// @return true if the mapping process is idle, else false
    bool idle() override;

	/// @brief this method is used to process mapping task.
	/// @param[in] frame: the input frame.
    /// @param[out] keyframe: new keyframe or new reference keyframe found.
    FrameworkReturnCode process(const SRef<SolAR::datastructure::Frame> frame, SRef<SolAR::datastructure::Keyframe> & keyframe) override;

	void unloadComponent() override final;

private:
	void updateAssociateCloudPoint(const SRef<SolAR::datastructure::Keyframe> &keyframe);
	void findMatchesAndTriangulation(const SRef<SolAR::datastructure::Keyframe> & keyframe, const std::vector<uint32_t> &idxBestNeighborKfs, std::vector<SRef<SolAR::datastructure::CloudPoint>> &cloudPoint);
	void cloudPointsCulling(const SRef<SolAR::datastructure::Keyframe> &keyframe);
    void setIdle(bool flag);

private:
	float																		m_minWeightNeighbor = 1.f;
	int																			m_maxNbNeighborKfs = 5;	
	int																			m_isSaveImage = 0;
    bool                                                                        m_idle = true;
    std::mutex                                                                  m_mutexIdle;
    SRef<SolAR::api::storage::ICovisibilityGraphManager>                        m_covisibilityGraphManager;
    SRef<SolAR::api::storage::IKeyframesManager>								m_keyframesManager;
    SRef<SolAR::api::reloc::IKeyframeRetriever>									m_keyframeRetriever;
    SRef<SolAR::api::storage::IMapManager>                                      m_mapManager;
    SRef<SolAR::api::storage::IPointCloudManager>								m_pointCloudManager;
    SRef<SolAR::api::solver::map::ITriangulator>								m_triangulator;
    SRef<SolAR::api::solver::map::IMapFilter>									m_mapFilter;
    SRef<SolAR::api::features::IDescriptorMatcherGeometric>						m_matcher;
	std::map<uint32_t, std::pair<SRef<SolAR::datastructure::CloudPoint>, uint32_t>>	m_recentAddedCloudPoints;
};

}
}
}

#endif // SOLARSLAMMAPPING_H
