#ifndef SOLARMAPPER_H
#define SOLARMAPPER_H


#include "api/solver/map/IMapper.h"
#include "xpcf/component/ComponentBase.h"
#include <vector>
#include "SolARToolsAPI.h"

#include <string>

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace TOOLS {
/**
* @class SolARMapper
* @brief Store the 3D Map and keyframes
*/
class SOLAR_TOOLS_EXPORT_API SolARMapper : public org::bcom::xpcf::ComponentBase,
    public api::solver::map::IMapper {
public:
    SolARMapper();

    ~SolARMapper() override = default;

    /// @brief update the current map with the new triangulated map points at the insertion of a new keyframe.
    /// minArg(pts3ds,intrinsics,extrinsics) = MIN_cam_i(MIN_3d_j(pts2d_j - reproje(pt3ds_j,intrinsics_i,extrinsics_i)),
    /// @param[in,out] map current constructed map.
    /// @param[in,out] neyKeyframe current new keyframe to insert.
    /// @param[in] newCloud new triangulated 3D points
    /// @param[in] newPointMatches new detected matches from the reference keyframe and current frame.
    /// @param[in] existingPointMatches new detected matches from the reference keyframe and current frame.
    /// @return FrameworkReturnCode::_SUCCESS if the map updating succeed, else FrameworkReturnCode::_ERROR_
    FrameworkReturnCode update (SRef<Map> & map,
                                        SRef<Keyframe> & newKeyframe,
                                        const std::vector<CloudPoint> & newCloud = {},
                                        const std::vector<DescriptorMatch> & newPointsMatches = {},
                                        const std::vector<DescriptorMatch> & existingPointsMatches = {}) override;

    std::vector<SRef<Keyframe>> getKeyframes() override;
    SRef<Map> getMap() override;
    void unloadComponent () final;

private:
    std::vector<SRef<Keyframe>> m_kframes;
    std::map<std::pair<int, int>, std::vector<DescriptorMatch> > m_gmatches;
    SRef<Map> m_map;

};
}
}
}

#endif // SOLARMAPPER_H
