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

#include "SolARMapFilter.h"
#include "datastructure/Keyframe.h"

namespace xpcf  = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::TOOLS::SolARMapFilter);

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace TOOLS {

SolARMapFilter::SolARMapFilter():ConfigurableBase(xpcf::toUUID<SolARMapFilter>())
{
    addInterface<IMapFilter>(this);
    SRef<xpcf::IPropertyMap> params = getPropertyRootNode();
    params->wrapFloat("reprojErrorThreshold", m_reprojErrorThreshold);
    params->wrapInteger("cheiralityCheck", m_cheiralityCheck);
}

void  SolARMapFilter::filter(const Transform3Df pose1, const Transform3Df pose2, const std::vector<SRef<CloudPoint>>& input,  std::vector<SRef<CloudPoint>>& output)
{
    if (input.size() == 0)
    {
        LOG_INFO("mapFilter opencv has an empty vector as input");
    }

       Transform3Df invPose1 = pose1.inverse();
       Transform3Df invPose2 = pose2.inverse();

    output.clear();

    for (int i = 0; i < input.size(); i++)
    {
        // Check for cheirality (if the point is in front of the camera)

        // BUG patch To correct, Vector4f should but is not accepted with windows !
#if (_WIN64) || (_WIN32)
        Vector3f point(input[i]->getX(), input[i]->getY(), input[i]->getZ());
        Vector3f pointInCam1Ref, pointInCam2Ref;
#else
        Vector4f point(input[i]->getX(), input[i]->getY(), input[i]->getZ(), 1);
        Vector4f pointInCam1Ref, pointInCam2Ref;
#endif
        pointInCam1Ref = invPose1 * point ;
        pointInCam2Ref = invPose2 * point ;

        if ((!m_cheiralityCheck) || ((pointInCam1Ref(2) >= 0) && pointInCam2Ref(2) >=0))
        {
            // if the reprojection error is less than the threshold
            if (input[i]->getReprojError() < m_reprojErrorThreshold)
                output.push_back(input[i]);
        }
    }
    LOG_DEBUG("Nb filtered points (){}/{})", output.size(), input.size());

}

}
}
}
