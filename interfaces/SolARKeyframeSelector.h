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

#ifndef SOLARKEYFRAMESELECTOR_H
#define SOLARKEYFRAMESELECTOR_H

#include "api/solver/map/IKeyframeSelector.h"
#include "xpcf/component/ConfigurableBase.h"
#include "SolARToolsAPI.h"

#include <vector>

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace TOOLS {

class SOLAR_TOOLS_EXPORT_API SolARKeyframeSelector : public org::bcom::xpcf::ConfigurableBase,
        public api::solver::map::IKeyframeSelector {
public:

    SolARKeyframeSelector();
    ///
    ///@brief ~SolARKeyframeSelector
    ///
    ~SolARKeyframeSelector() override = default;

    /// @brief  Select if a frame can be considered as a keyframe
    /// @param[in] frame: the frame tested to know if it could be a Keyframe
    /// @param[in] matches: the matches between the frame and its keyframe of reference.
    /// @return true if the frame can be considered as a new keyframe, false otherwise.
    bool select(const SRef<Frame> & frame, const std::vector<DescriptorMatch> & matches) override;

    void unloadComponent () final;

 private:
    // Minimum number of matches for a frame to be a keyframe
    int m_minNbMatchesIsKeyframe = 50;

    // Minimum mean distance for a frame to be a keyframe
    float m_minMeanDistanceIsKeyframe = 20.0;

};

}
}
}
#endif // SOLARKEYFRAMESELECTOR_H
