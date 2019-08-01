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

#ifndef SOLARHOMOGRAPHYVALIDATION_H
#define SOLARHOMOGRAPHYVALIDATION_H

#include "api/solver/pose/IHomographyValidation.h"
#include "xpcf/component/ConfigurableBase.h"
#include "SolARToolsAPI.h"

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace TOOLS {

/**
 * @class SolARHomographyValidation
 * @brief Check if an homography is valid based on 4 corners of a squared marker and their projection through a given homography
 *
 */

class SOLAR_TOOLS_EXPORT_API SolARHomographyValidation : public org::bcom::xpcf::ConfigurableBase,
        public api::solver::pose::IHomographyValidation {
public:
    SolARHomographyValidation();

    ~SolARHomographyValidation() override = default;

    /// @brief evaluate the homography according to the projection of the 4 corners of a 2D squared Marker
    /// @param[in] Set of 2d_points of the corners in the template view.
    /// @param[in] Set of 2d_points of the corners projected in the current view.
    bool isValid(const std::vector<Point2Df> & ref2DSquaredMarkerCorners, const std::vector<Point2Df> & projected2DSquaredMarkerCorners) override;

    void unloadComponent () final;

private:
    /// @brief minimum length ratio of the opposite reprojected sides of the squared marker
    float m_oppositeSideRatio = 0.5f;

    /// @brief minimum surface ratio between the squared marker and its reprojection through the homography
    float m_surfaceRatio = 0.15f;

    /// @brief the maximum of the dot product of the two opposite and normalized sides of the reprojected squared marker
    /// Here, we check is the opposite reprojected sides of the squared marker are not to perpendicular. If the dot product is close to 1, it means that the opposite side are perpendicular and so that the homography is not good.
    float m_maxOppositeDotProduct = 0.9;
};

}
}
}  // end of namespace Solar

#endif // SOLARHOMOGRAPHYVALIDATION_H
