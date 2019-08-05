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

#include "SolARSBPatternReIndexer.h"


#include "xpcf/component/ComponentFactory.h"

namespace xpcf = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::TOOLS::SolARSBPatternReIndexer)

namespace SolAR {
namespace MODULES {
namespace TOOLS {

    SolARSBPatternReIndexer::SolARSBPatternReIndexer():ConfigurableBase(xpcf::toUUID<SolARSBPatternReIndexer>())
    {
        declareInterface<api::features::ISBPatternReIndexer>(this);
        declareProperty("sbPatternSize", m_sbPatternSize);
        m_sbPatternSize = 1;
    }


    FrameworkReturnCode SolARSBPatternReIndexer::reindex(const std::vector<Contour2Df> & candidateContours, const std::vector<DescriptorMatch> & matches, std::vector<Point2Df> & patternPoints, std::vector<Point2Df>& imagePoints)
    {
        patternPoints.clear();
        imagePoints.clear();
        patternPoints.reserve(matches.size() * 4);
        imagePoints.reserve(matches.size() * 4);
        for (auto itr = matches.begin(); itr != matches.end(); ++itr)
        {
            patternPoints.emplace_back(0.0f, 0.0f);
            patternPoints.emplace_back(m_sbPatternSize, 0.0f);
            patternPoints.emplace_back(m_sbPatternSize, m_sbPatternSize);
            patternPoints.emplace_back(0.0f,m_sbPatternSize);

            Contour2Df recognizedContour = candidateContours[itr->getIndexInDescriptorB()];
            imagePoints.emplace_back(recognizedContour[0].x(), recognizedContour[0].y());
            imagePoints.emplace_back(recognizedContour[1].x(), recognizedContour[1].y());
            imagePoints.emplace_back(recognizedContour[2].x(), recognizedContour[2].y());
            imagePoints.emplace_back(recognizedContour[3].x(), recognizedContour[3].y());
        }
        return FrameworkReturnCode::_SUCCESS;
    }

}
}
}  // end of namespace Solar
