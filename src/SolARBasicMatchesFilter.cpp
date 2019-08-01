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

#include "SolARBasicMatchesFilter.h"
#include "xpcf/component/ComponentFactory.h"
#include <set>

namespace xpcf = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::TOOLS::SolARBasicMatchesFilter);

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace TOOLS {

SolARBasicMatchesFilter::SolARBasicMatchesFilter():ComponentBase(xpcf::toUUID<SolARBasicMatchesFilter>())
{
    declareInterface<api::features::IMatchesFilter>(this);
}


SolARBasicMatchesFilter::~SolARBasicMatchesFilter()= default;


bool sortMatchByDistance(const std::pair<int,float> &lhs, const std::pair<int,float> &rhs)
{
    return lhs.second < rhs.second;
}

// filter matches : keep the best match in case of multiple matches per keypoint
void SolARBasicMatchesFilter::filter(const std::vector<DescriptorMatch> & inputMatches,
                                     std::vector<DescriptorMatch> & outputMatches,
                                     const std::vector<Keypoint> & inputKeyPointsA,
                                     const std::vector<Keypoint> & inputKeyPointsB)
{
    std::map<int,std::vector<std::pair<int,float>>> matchesMap;

    for(const auto& match : inputMatches){
        matchesMap[match.getIndexInDescriptorA()].push_back(std::make_pair(match.getIndexInDescriptorB(),match.getMatchingScore()));
    }

    std::vector<DescriptorMatch> matches;
    for (auto & kv : matchesMap) {//std::map<int,std::vector<std::pair<int,float>>>::iterator itr=matchesMap.begin();itr!=matchesMap.end();++itr){
        std::vector<std::pair<int,float>> &  vect = kv.second;
        if(vect.size()>1){
            std::sort(vect.begin(),vect.end(),sortMatchByDistance);
        }
        matches.push_back(DescriptorMatch(kv.first, vect.begin()->first,vect.begin()->second));
    }

    matchesMap.clear();
    for(const auto& match : matches){
        matchesMap[match.getIndexInDescriptorB()].push_back(std::make_pair(match.getIndexInDescriptorA(),match.getMatchingScore()));
    }

    matches.clear();
   for (auto & kv : matchesMap) {//for (std::map<int,std::vector<std::pair<int,float>>>::iterator itr=matchesMap.begin();itr!=matchesMap.end();++itr){
        std::vector<std::pair<int,float>> & vect = kv.second;
        if(vect.size()>1){
            std::sort(vect.begin(),vect.end(),sortMatchByDistance);
        }
        matches.push_back(DescriptorMatch(vect.begin()->first,kv.first,vect.begin()->second));
    }

    outputMatches = matches;
 }

}
}
}
