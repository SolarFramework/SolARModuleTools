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
        for (auto itr = matches.begin(); itr != matches.end(); ++itr)
        {
            patternPoints.push_back(Point2Df(0.0f, 0.0f));
            patternPoints.push_back(Point2Df(m_sbPatternSize, 0.0f));
            patternPoints.push_back(Point2Df(m_sbPatternSize, m_sbPatternSize));
            patternPoints.push_back(Point2Df(0.0f,m_sbPatternSize));

            Contour2Df recognizedContour = candidateContours[itr->getIndexInDescriptorB()];
            imagePoints.push_back(Point2Df(recognizedContour[0].getX(), recognizedContour[0].getY()));
            imagePoints.push_back(Point2Df(recognizedContour[1].getX(), recognizedContour[1].getY()));
            imagePoints.push_back(Point2Df(recognizedContour[2].getX(), recognizedContour[2].getY()));
            imagePoints.push_back(Point2Df(recognizedContour[3].getX(), recognizedContour[3].getY()));
        }
        return FrameworkReturnCode::_SUCCESS;
    }

}
}
}  // end of namespace Solar
