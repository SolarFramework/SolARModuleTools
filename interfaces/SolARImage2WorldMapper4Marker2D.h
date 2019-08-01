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

#ifndef SOLARIMAGE2WORLDMAPPER4MARKER2D_H
#define SOLARIMAGE2WORLDMAPPER4MARKER2D_H

#include "api/geom/IImage2WorldMapper.h"
#include "xpcf/component/ConfigurableBase.h"
#include "SolARToolsAPI.h"

#include <vector>

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace TOOLS {

class SOLAR_TOOLS_EXPORT_API SolARImage2WorldMapper4Marker2D : public org::bcom::xpcf::ConfigurableBase,
        public api::geom::IImage2WorldMapper {
public:

    SolARImage2WorldMapper4Marker2D();
    ~SolARImage2WorldMapper4Marker2D() override;

    FrameworkReturnCode map(const std::vector<Point2Df> & digitalPoints, std::vector<Point3Df> & worldPoints) override;

    void unloadComponent () final;


private:
    /// @brief the width of the 2D marker in pixels for a natural image marker, or in cells for a squared binary marker.
    int m_digitalWidth{};

    /// @brief the height of the 2D marker in pixels for a natural image marker, or in cells for a squared binary marker.
    int m_digitalHeight{};

    /// @brief the width of the marker in a user-defined world coordinate system (meters, cenimeters, etc.)
    float m_worldWidth{};

    /// @brief the height of the marker in a user-defined world coordinate system (meters, cenimeters, etc.)
    float m_worldHeight{};


};

}
}
}

#endif // SOLARIMAGE2WORLDMAPPER4MARKER2D_H
