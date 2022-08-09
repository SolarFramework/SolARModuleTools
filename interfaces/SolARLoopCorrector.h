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

#ifndef SOLARLOOPCORRECTOR_H
#define SOLARLOOPCORRECTOR_H

#include "api/loop/ILoopCorrector.h"
#include "api/storage/IPointCloudManager.h"
#include "api/storage/ICovisibilityGraphManager.h"
#include "api/storage/IKeyframesManager.h"
#include "api/geom/I3DTransform.h"
#include "api/features/IDescriptorMatcherRegion.h"
#include "api/geom/IProject.h"
#include "xpcf/component/ConfigurableBase.h"
#include "SolARToolsAPI.h"
#include <fstream>
#include <mutex>

namespace SolAR {
namespace MODULES {
namespace TOOLS {


/**
 * @class SolARLoopCorrector
 * @brief Corrects a loop of camera poses and updates associated geometry.
 * <TT>UUID: 1007b588-c1f2-11ea-b3de-0242ac130004</TT>
 *
 *@SolARComponentInjectablesBegin
 * @SolARComponentInjectable{SolAR::api::storage::IKeyframesManager}
 * @SolARComponentInjectable{SolAR::api::storage::IPointCloudManager}
 * @SolARComponentInjectable{SolAR::api::storage::ICovisibilityGraph}
 * @SolARComponentInjectable{SolAR::api::features::IDescriptorMatcherRegion}
 * @SolARComponentInjectable{SolAR::api::geom::I3DTransform}
 * @SolARComponentInjectable{SolAR::api::geom::IProject}
 * @SolARComponentInjectablesEnd
 *
 */

class SOLAR_TOOLS_EXPORT_API SolARLoopCorrector : public org::bcom::xpcf::ConfigurableBase,
        public SolAR::api::loop::ILoopCorrector {
public:

    SolARLoopCorrector();
    ~SolARLoopCorrector() = default;

    /// @brief corrects a loop of keyframes and their associated point clouds from a loop detection result.
    /// @param[in] queryKeyframe: the query keyframe.
    /// @param[in] detectedLoopKeyframe: the detected loop keyframe.
	/// @param[in] S_wl_wc : 3D similarity transformation (Sim(3)) from world c.s of the query keyframe to world c.s of the loop detected keyframe
    /// @param[in] duplicatedPointsIndices: indices of duplicated cloud points. The first index is the id of point cloud seen from the detected loop keyframe. The second one is id of point cloud seen from the query keyframe
    /// @return FrameworkReturnCode::_SUCCESS if loop closure is correctly corrected, else FrameworkReturnCode::_ERROR_
    FrameworkReturnCode correct(const SRef<SolAR::datastructure::Keyframe> queryKeyframe,
                                const SRef<SolAR::datastructure::Keyframe> detectedLoopKeyframe,
                                const SolAR::datastructure::Transform3Df & S_wl_wc,
                                const std::vector<std::pair<uint32_t, uint32_t>> &duplicatedPointsIndices) override;
    
	void unloadComponent () override final;

private:
	// get cloud points seen from keyframes
	void getLocalMapPoints(const std::map<uint32_t, SRef<SolAR::datastructure::Keyframe> > &connectedKfs, std::vector<SRef<SolAR::datastructure::CloudPoint>>& localMapPoints);

 private:
    SRef<SolAR::api::storage::IKeyframesManager>			m_keyframesManager;
    SRef<SolAR::api::storage::ICovisibilityGraphManager>    m_covisibilityGraphManager;
    SRef<SolAR::api::storage::IPointCloudManager>			m_pointCloudManager;
    SRef<SolAR::api::features::IDescriptorMatcherRegion>	m_matcher;
    SRef<SolAR::api::geom::I3DTransform>					m_transform3D;
    SRef<SolAR::api::geom::IProject>						m_projector;
};

}
}
}

#endif // SOLARLOOPCORRECTOR_H
