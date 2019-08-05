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

#include "SolAR2DTransform.h"
#include "xpcf/component/ComponentFactory.h"

namespace xpcf  = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::TOOLS::SolAR2DTransform);

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace TOOLS {

SolAR2DTransform::SolAR2DTransform():ComponentBase(xpcf::toUUID<SolAR2DTransform>())
{
   declareInterface<api::geom::I2DTransform>(this);
}


SolAR2DTransform::~SolAR2DTransform()= default;

FrameworkReturnCode SolAR2DTransform::transform(const std::vector<Point2Df> & inputPoints, const Transform2Df & transformation, std::vector<Point2Df> & outputPoints)
{
    Point2Df outputPoint2D;
    Vector3f outputVector3f;

    outputPoints.reserve(inputPoints.size());
    for (auto inputPoint2D : inputPoints){
        Vector3f inputVector3f(inputPoint2D.x(),inputPoint2D.y(),1);
        outputVector3f=transformation*inputVector3f;
        if (outputVector3f[2]!=0) {
            outputPoint2D = {outputVector3f[0]/outputVector3f[2], outputVector3f[1]/outputVector3f[2]};
        } else {
            outputPoint2D = {0, 0};
        }
        outputPoints.emplace_back(outputPoint2D);
    }
    return FrameworkReturnCode::_SUCCESS;
}

}
}
}
