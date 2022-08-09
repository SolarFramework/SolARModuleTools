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

#ifndef SOLAROVERLAPDETECTOR_H
#define SOLAROVERLAPDETECTOR_H

#include "api/reloc/IKeyframeRetriever.h"
#include "api/loop/ILoopClosureDetector.h"
#include "api/solver/pose/I3DTransformSACFinderFrom3D3D.h"
#include "api/geom/I3DTransform.h"
#include "api/features/IDescriptorMatcher.h"
#include "api/solver/pose/I3D3DCorrespondencesFinder.h"
#include "api/features/IMatchesFilter.h"
#include "api/loop/IOverlapDetector.h"
#include "xpcf/component/ConfigurableBase.h"
#include "SolARToolsAPI.h"
#include <fstream>
#include <mutex>

namespace SolAR {
namespace MODULES {
namespace TOOLS {
/**
* @class SolAROverlapDetector
* @brief Detect a loop closure from a given keyframe.
* <TT>UUID: 58087630-1376-11eb-adc1-0242ac120002</TT>
*
* @SolARComponentInjectablesBegin
* @SolARComponentInjectable{SolAR::api::solver::pose::I3DTransformSACFinderFrom3D3D}
* @SolARComponentInjectable{SolAR::api::features::IDescriptorMatcher}
* @SolARComponentInjectable{SolAR::api::features::IMatchesFilter}
* @SolARComponentInjectable{SolAR::api::solver::pose::I3D3DCorrespondencesFinder}
* @SolARComponentInjectable{SolAR::api::geom::I3DTransform}
* @SolARComponentInjectablesEnd
*
* @SolARComponentPropertiesBegin
* @SolARComponentProperty{ minNbInliers,
*                          ,
*                          @SolARComponentPropertyDescNum{ int, [0..MAX INT], 0 }}
* @SolARComponentPropertiesEnd
*
*/

class SOLAR_TOOLS_EXPORT_API SolAROverlapDetector : public org::bcom::xpcf::ConfigurableBase,
        public SolAR::api::loop::IOverlapDetector {
public:
	SolAROverlapDetector();
	~SolAROverlapDetector() = default;

	/// @brief Detect overlap between two floating maps with different refences.
	/// @param[in] globalMap global map as reference.
	/// @param[in] floatingMap floating map as the map to merge.
	/// @param[out] sim3Transform 3D similarity transformation (Sim(3)) from the floating map to the global map.
	/// @param[out] cpOverlapIndices pairs of detected overlap cloud points indices of floating map and global map.
	/// @return FrameworkReturnCode::_SUCCESS if detect a loop closure, else FrameworkReturnCode::_ERROR_
    FrameworkReturnCode detect(const SRef<SolAR::datastructure::Map> globalMap,
                               const SRef<SolAR::datastructure::Map> floatingMap,
                               SolAR::datastructure::Transform3Df & sim3Transform,
                               std::vector<std::pair<uint32_t, uint32_t>> & cpOverlapIndices) const override;

	/// @brief Detect overlap between two floating maps with different refences.
	/// @param[in] globalMap global map as reference.
	/// @param[in] floatingMap floating map as the map to merge.
	/// @param[out] sim3Transform 3D similarity transformation (Sim(3)) from query keyframe from the floating map to the detected overlaped keyframe in global map.
	/// @param[out] overlapIndices pairs of detected overlap keyframe indices of floating map and global map.
	/// @param[out] scores : represent scores of overlap candidates.
	/// @return FrameworkReturnCode::_SUCCESS if detect a loop closure, else FrameworkReturnCode::_ERROR_
    FrameworkReturnCode detect(const SRef<SolAR::datastructure::Map> globalMap,
                               const SRef<SolAR::datastructure::Map> floatingMap,
                               std::vector<SolAR::datastructure::Transform3Df> & sim3Transform,
                               std::vector<std::pair<uint32_t, uint32_t>> & overlapIndices,
                               std::vector<double>&scores) const override;


	void unloadComponent () override final;

 private:
	SRef<SolAR::api::reloc::IKeyframeRetriever>						m_globalKeyframeRetriever;
	SRef<SolAR::api::solver::pose::I3DTransformSACFinderFrom3D3D>	m_estimator3D;
	SRef<SolAR::api::features::IDescriptorMatcher>					m_matcher;
	SRef<SolAR::api::features::IMatchesFilter>						m_matchesFilter;
	SRef<SolAR::api::solver::pose::I3D3DCorrespondencesFinder>		m_corr3D3DFinder;
	SRef<SolAR::api::geom::I3DTransform>							m_transform3D;
	int																m_NbMinInliers;
};

}
}
}

#endif // SOLARLOOPCLOSUREDETECTOR_H
