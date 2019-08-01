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

#include "SolARImage2WorldMapper4Marker2D.h"
#include "xpcf/component/ComponentFactory.h"
#include "core/Log.h"

namespace xpcf  = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::TOOLS::SolARImage2WorldMapper4Marker2D);

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace TOOLS {

SolARImage2WorldMapper4Marker2D::SolARImage2WorldMapper4Marker2D():ConfigurableBase(xpcf::toUUID<SolARImage2WorldMapper4Marker2D>())
{
    declareInterface<api::geom::IImage2WorldMapper>(this);
    declareProperty("digitalWidth",m_digitalWidth);
    declareProperty("digitalHeight",m_digitalHeight);
    declareProperty("worldWidth",m_worldWidth);
    declareProperty("worldHeight",m_worldHeight);
}


SolARImage2WorldMapper4Marker2D::~SolARImage2WorldMapper4Marker2D()= default;

FrameworkReturnCode SolARImage2WorldMapper4Marker2D::map(const std::vector<Point2Df> & digitalPoints, std::vector<Point3Df> & worldPoints)
{
    worldPoints.clear();
    Point3Df point_3D;
    if (m_digitalWidth <= 0 || m_digitalHeight <= 0)
    {
        LOG_ERROR("SolARImage2WorldMapper::map: marker resolution width or height is equal to zero or negative")
        return FrameworkReturnCode::_ERROR_;
    }
    float width_ratio =  m_worldWidth / m_digitalWidth;
    float height_ratio = m_worldHeight / m_digitalHeight;
    float half_img_width = (float) m_digitalWidth / 2;
    float half_img_height = (float) m_digitalHeight / 2;


    for (const auto & point_2D : digitalPoints)
    {
        point_3D.setX((point_2D.getX()-half_img_width)*width_ratio);
        point_3D.setY((point_2D.getY()-half_img_height)*height_ratio);
        point_3D.setZ(0);
        worldPoints.push_back(Point3Df(point_3D));
    }
    return FrameworkReturnCode::_SUCCESS;
}

}
}
}
