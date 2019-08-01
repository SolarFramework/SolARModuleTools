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

#ifndef SOLARKEYPOINTSREINDEXER_H
#define SOLARKEYPOINTSREINDEXER_H

#include "api/features/IKeypointsReIndexer.h"
#include "xpcf/component/ComponentBase.h"
#include "SolARToolsAPI.h"

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace TOOLS {

class SOLAR_TOOLS_EXPORT_API SolARKeypointsReIndexer : public org::bcom::xpcf::ComponentBase,
        public api::features::IKeypointsReIndexer {
public:
    SolARKeypointsReIndexer();
    ~SolARKeypointsReIndexer() override = default;

    FrameworkReturnCode reindex(const std::vector<Keypoint>& refKeypoints,
                                const std::vector<Keypoint> & imgKeypoints,
                                const std::vector<DescriptorMatch> & matches,
                                std::vector<Point2Df> & matchedRefKeypoints,
                                std::vector<Point2Df> & matchedImgKeypoints) override;

    void unloadComponent () final;

    };

}
}
}  // end of namespace Solar

#endif // SOLARKEYPOINTSREINDEXER_H
