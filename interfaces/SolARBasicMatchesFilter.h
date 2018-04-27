#ifndef SolARBasicMatchesFilter_H
#define SolARBasicMatchesFilter_H

#include "api/features/IMatchesFilter.h"
#include "ComponentBase.h"
#include "SolARToolsAPI.h"

#include <vector>

namespace SolAR {
    using namespace datastructure;
    namespace MODULES {
        namespace TOOLS {
        /**
         * @class SolARBasicMatchesFilter
         * @brief Filters matches based on basic strategies (redundancy, match score..).
         */
            class SOLAR_TOOLS_EXPORT_API SolARBasicMatchesFilter : public org::bcom::xpcf::ComponentBase,
                    public api::features::IMatchesFilter {
            public:
               ///@brief SolARBasicMatchesFilter constructor.
               SolARBasicMatchesFilter();
               ///@brief SolARBasicMatchesFilter destructor.
               ~SolARBasicMatchesFilter();

               /// @brief filter matches based on redundancy strategy. This filter removes all the mmulitples matches.
               /// @param[in] Original matches found between two descriptors "desc_1" and "desc_2".
               /// @param[out] Filtred matches based on redanduncy or geometric relations such as epipolar constraint.
               /// @param[in] Original keypoints associated to desc_1.
               /// @param[in] Original keypoints associated to desc_2.
               void filter(const std::vector<DescriptorMatch>&inputMatches,
                           std::vector<DescriptorMatch>&outputMatches,
                           const std::vector<SRef<Keypoint>>&inputKeyPointsA,
                           const std::vector<SRef<Keypoint>>&inputKeyPointsB);

                void unloadComponent () override final;

                    XPCF_DECLARE_UUID("cbb620c3-a7fc-42d7-bcbf-f59b475b23b0");

             private:

            };

}
}
}




#endif // SOLARMATCHESFILTEROPENCV_H
