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

#ifndef SOLARWORLDGRAPHLOADER_H
#define SOLARWORLDGRAPHLOADER_H

#include "api/input/files/IWorldGraphLoader.h"
#include "api/image/IImageLoader.h"
#include "xpcf/component/ConfigurableBase.h"
#include "SolARToolsAPI.h"

namespace SolAR {
namespace MODULES {
namespace TOOLS {
/**
 * @class SolARWorldGraphLoader
 * @brief Load a world graph of trackables.
 * <TT>UUID: 8ee6aa50-f6bb-4b01-a1fe-727b54ed0457</TT>
 * @SolARComponentInjectablesBegin
 * @SolARComponentInjectable{SolAR::api::image::IImageLoader}
 * @SolARComponentInjectablesEnd
 */
class SOLAR_TOOLS_EXPORT_API SolARWorldGraphLoader : public org::bcom::xpcf::ConfigurableBase,
        public SolAR::api::input::files::IWorldGraphLoader {
public:

    SolARWorldGraphLoader();
    ~SolARWorldGraphLoader() = default;

    /// @brief Loads a world graph of trackables.
    /// @param [out] trackables the set of trackables.
    /// @return FrameworkReturnCode::_SUCCESS if load succeed, else FrameworkReturnCode::_ERROR_
    FrameworkReturnCode load(std::vector<SRef<SolAR::datastructure::Trackable>>& trackables) override;

    void unloadComponent () override final;

 private:
    std::string m_filePath;
    std::vector<SRef<SolAR::datastructure::Trackable>> m_trackables;
	SRef<api::image::IImageLoader> m_imageLoader;
};

}
}
}

#endif // SOLARWORLDGRAPHLOADER_H
