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

#include "SolAR3DTransform.h"
#include "xpcf/component/ComponentFactory.h"
namespace xpcf  = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::TOOLS::SolAR3DTransform);

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace TOOLS {

SolAR3DTransform::SolAR3DTransform():ComponentBase(xpcf::toUUID<SolAR3DTransform>())
{
    declareInterface<api::geom::I3DTransform>(this);
}


SolAR3DTransform::~SolAR3DTransform()= default;

FrameworkReturnCode SolAR3DTransform::transform(const std::vector<Point3Df> & inputPoints, const Transform3Df & transformation, std::vector<Point3Df> & outputPoints)
{
    Point3Df outputPoint3D;
    Vector4f outputVector4f;

    outputPoints.reserve(inputPoints.size());
    for (const auto& inputPoint3D : inputPoints){
        Vector4f inputVector4f(inputPoint3D.x(),inputPoint3D.y(), inputPoint3D.z(), 1);
        outputVector4f=transformation*inputVector4f;
        if (outputVector4f[3]!=0) {
            outputPoint3D = {outputVector4f[0]/outputVector4f[3], outputVector4f[1]/outputVector4f[3], outputVector4f[2]/outputVector4f[3]};
        } else {
            outputPoint3D = {0, 0, 0};
        }
        outputPoints.emplace_back(outputPoint3D);
    }

    return FrameworkReturnCode::_SUCCESS;
}

}
}
}
