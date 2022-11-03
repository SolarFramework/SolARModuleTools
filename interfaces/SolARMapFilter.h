#ifndef SOLARMAPFILTER_H
#define SOLARMAPFILTER_H


#include "api/solver/map/IMapFilter.h"
#include "xpcf/component/ConfigurableBase.h"
#include <vector>
#include "SolARToolsAPI.h"

namespace SolAR {
namespace MODULES {
namespace TOOLS {

/**
 * @class SolARMapFilter
 * @brief <B>Filters a cloud of 3D points by removing points with a too important reprojection error or those which are behind the camera.</B>
 * <TT>UUID: 09205b96-7cba-4415-bc61-64744bc26222</TT>
 *
 * The projection error threshold as well as the test of cheirality (removing points behind the camera) can be configured.
 *
 * @SolARComponentPropertiesBegin
 * @SolARComponentProperty{ reprojErrorThreshold,
 *                          maximum reprojection error to keep the triangulated 3D point,
 *                          @SolARComponentPropertyDescNum{ float, [0..MAX FLOAT], 0.5f }}
 * @SolARComponentProperty{ cheiralityCheck,
 *                          if not 0\, the point reconstructed behind the camera are removed,
 *                          @SolARComponentPropertyDescNum{ int, [0..1], 1 }}
 * @SolARComponentProperty{ minTriangulationAngle,
 *                          min angle between rays,
 *                          @SolARComponentPropertyDescNum{ float, [0..MAX FLOAT], 0.03f }}
 * @SolARComponentProperty{ maxTriangulationAngle,
 *                          max angle between rays,
 *                          @SolARComponentPropertyDescNum{ float, [0..MAX FLOAT], 0.2f }}
 * @SolARComponentPropertiesEnd
 *
 */

class SOLAR_TOOLS_EXPORT_API SolARMapFilter : public org::bcom::xpcf::ConfigurableBase,
        public SolAR::api::solver::map::IMapFilter {
public:
    SolARMapFilter();

    ~SolARMapFilter() override = default;

    /// @brief  Filter point cloud reconstructed from 2 viewpoints
    /// @param[in] pose1: the first pose used for building the point cloud.
    /// @param[in] pose2: the second pose used for building the point cloud.
    /// @param[in] input: The set of points to filter
    /// @param[out] output: the filtered point cloud
    void  filter(const SolAR::datastructure::Transform3Df & pose1, const SolAR::datastructure::Transform3Df & pose2, const std::vector<SRef<SolAR::datastructure::CloudPoint>>& input,  std::vector<SRef<SolAR::datastructure::CloudPoint>>& output) override;

	/// @brief  Filter point cloud reconstructed from 2 viewpoints
	/// @param[in] pose1: the first pose used for building the point cloud.
	/// @param[in] pose2: the second pose used for building the point cloud.
	/// @param[in] input: The set of points to filter
	/// @param[out] output: the filtered point cloud
	/// @param[out] index: the index of filtered point cloud
	void  filter(const SolAR::datastructure::Transform3Df & pose1, const SolAR::datastructure::Transform3Df & pose2, const std::vector<SRef<SolAR::datastructure::CloudPoint>>& input, std::vector<SRef<SolAR::datastructure::CloudPoint>>& output, std::vector<int> &index) override;

    void unloadComponent () override final;

protected :

private:
    //@brief maximum reprojection error to keep the triangulated 3D point
    float m_reprojErrorThreshold = 0.5f;

    //@brief if not null, the point reconstructed behind the camera are removed
    int m_cheiralityCheck = 1;

	//@brief min angle between rays
    float m_minTriangulationAngle = 0.03f;

	//@brief max angle between rays
    float m_maxTriangulationAngle = 0.2f;
};
}
}
}

#endif // SOLARMAPFILTER_H
