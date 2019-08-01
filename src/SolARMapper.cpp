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

#include "SolARMapper.h"
#include "core/Log.h"

namespace xpcf  = org::bcom::xpcf;


XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::TOOLS::SolARMapper)

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace TOOLS {

    SolARMapper::SolARMapper():ComponentBase(xpcf::toUUID<SolARMapper>())
    {
        declareInterface<IMapper>(this);
        m_map = xpcf::utils::make_shared<Map>();
    }

    FrameworkReturnCode SolARMapper::update (SRef<Map> & map,
                                             SRef<Keyframe> & newKeyframe,
                                             const std::vector<CloudPoint> & newCloud,
                                             const std::vector<DescriptorMatch> & newPointsMatches,
                                             const std::vector<DescriptorMatch> & existingPointsMatches)
    {
        if (m_kframes.empty())
        {
            if (!newCloud.empty() || !newPointsMatches.empty() || !existingPointsMatches.empty())
            {
                LOG_WARNING("For the first update of the Mapper, only the first keyframe is required");
            }
            m_kframes.push_back(newKeyframe);
            map = m_map;
            return FrameworkReturnCode::_SUCCESS;
        }

        // A point cloud exist and we need to check if matches have already a corresponding 3D points
        SRef<Keyframe> referenceKeyframe = newKeyframe->getReferenceKeyframe();
        if (referenceKeyframe == nullptr)
        {
            LOG_ERROR("When updating the map, if the keyframe is not the first one, it should have a reference keyframe");
            map = m_map;
            return FrameworkReturnCode::_ERROR_;
        }

        std::map<unsigned int, CloudPoint> keyframeVisibility, newRefKeyframeVisibility, refKeyframeVisibility = referenceKeyframe->getVisibleMapPoints();
        std::map<unsigned int, unsigned int>::iterator visIt;

        for (const auto & i : newCloud)
        {
            std::map<unsigned int, unsigned int> cloudPointVisibility = i.getVisibility();
            // update the visibility of the current keyframe with the new 3D points
            visIt = cloudPointVisibility.find(newKeyframe->m_idx);
            if (visIt != cloudPointVisibility.end())
            {
                keyframeVisibility[visIt->second] = i;
            }
            // update the visibility of the reference keyframe with the new 3D points
            visIt = cloudPointVisibility.find(referenceKeyframe->m_idx);
            if (visIt != cloudPointVisibility.end())
            {
                newRefKeyframeVisibility[visIt->second] = i;
            }
        }

        if (m_kframes.size() == 1)
        {
            if (!existingPointsMatches.empty())
            {
                LOG_WARNING("For the second update of the Mapper, not need of existing points");
            }
        }
        else
        {
            for (const auto & existingPointsMatche : existingPointsMatches)
            {
                // update the existing 3D points already in the map visible by the current keyframe and reciprocally
                auto refKFVisIt = refKeyframeVisibility.find(existingPointsMatche.getIndexInDescriptorA());
                if ( refKFVisIt != refKeyframeVisibility.end() )
                {
                    keyframeVisibility[existingPointsMatche.getIndexInDescriptorB()] = refKFVisIt->second;
                    refKFVisIt->second.visibilityAddKeypoint(newKeyframe->m_idx, existingPointsMatche.getIndexInDescriptorB());
                }
            }
        }
        m_gmatches[std::make_pair(referenceKeyframe->m_idx, newKeyframe->m_idx)] = newPointsMatches;
        newKeyframe->addVisibleMapPoints(keyframeVisibility);
        referenceKeyframe->addVisibleMapPoints(newRefKeyframeVisibility);
        m_kframes.push_back(newKeyframe);
        // Add the 3D points that have just been triangulated
        m_map->addCloudPoints(newCloud);
        map = m_map;
        return FrameworkReturnCode::_SUCCESS;
    }

    std::vector<SRef<Keyframe>> SolARMapper::getKeyframes(){
        return m_kframes;
    }
    SRef<Map> SolARMapper::getMap(){
        return m_map;
    }

}
}
}
