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

#include "SolARKeypointsReIndexer.h"
#include "xpcf/component/ComponentFactory.h"

namespace xpcf = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::TOOLS::SolARKeypointsReIndexer);

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace TOOLS {

    SolARKeypointsReIndexer::SolARKeypointsReIndexer():ComponentBase(xpcf::toUUID<SolARKeypointsReIndexer>())
    {
        declareInterface<api::features::IKeypointsReIndexer>(this);
    }

  FrameworkReturnCode SolARKeypointsReIndexer::reindex(const std::vector<Keypoint> & refKeypoints,
                                                       const std::vector<Keypoint> & imgKeypoints,
                                                       const std::vector<DescriptorMatch> & matches,
                                                       std::vector<Point2Df>& matchedRefKeypoints,
                                                       std::vector<Point2Df>& matchedImgKeypoints)
  {
      matchedRefKeypoints.clear();
      matchedImgKeypoints.clear();

      matchedRefKeypoints.reserve(matches.size());
      matchedImgKeypoints.reserve(matches.size());
      for(const auto & match : matches)
      {
          matchedRefKeypoints.emplace_back(refKeypoints[match.getIndexInDescriptorA()].x(),refKeypoints[match.getIndexInDescriptorA()].y());
          matchedImgKeypoints.emplace_back(imgKeypoints[match.getIndexInDescriptorB()].x(),imgKeypoints[match.getIndexInDescriptorB()].y());
      }
      return FrameworkReturnCode::_SUCCESS;
  }

}
}
}  // end of namespace Solar
