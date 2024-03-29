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
#include "core/Log.h"

namespace xpcf = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::TOOLS::SolARKeypointsReIndexer);

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace TOOLS {

    SolARKeypointsReIndexer::SolARKeypointsReIndexer():ComponentBase(xpcf::toUUID<SolARKeypointsReIndexer>())
    {
        declareInterface<SolAR::api::features::IKeypointsReIndexer>(this);
		LOG_DEBUG("SolARKeypointsReIndexer constructor");
    }

  FrameworkReturnCode SolARKeypointsReIndexer::reindex(const std::vector<Keypoint> & refKeypoints,
                                                       const std::vector<Keypoint> & imgKeypoints,
                                                       const std::vector<DescriptorMatch> & matches,
                                                       std::vector<Point2Df>& matchedRefKeypoints,
                                                       std::vector<Point2Df>& matchedImgKeypoints)
    {
        matchedRefKeypoints.clear();
        matchedImgKeypoints.clear();

       for( int i = 0; i < matches.size(); i++ )
       {
            matchedRefKeypoints.push_back(Point2Df(refKeypoints[ matches[i].getIndexInDescriptorA()].getX(),refKeypoints[ matches[i].getIndexInDescriptorA()].getY()));
            matchedImgKeypoints.push_back(Point2Df(imgKeypoints[ matches[i].getIndexInDescriptorB()].getX(),imgKeypoints[ matches[i].getIndexInDescriptorB()].getY()));
       }
       return FrameworkReturnCode::_SUCCESS;
    }

}
}
}  // end of namespace Solar
