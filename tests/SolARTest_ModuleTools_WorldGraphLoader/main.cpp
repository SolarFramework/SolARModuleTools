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

#include <boost/log/core.hpp>
#include "xpcf/xpcf.h"
#include <xpcf/api/IComponentManager.h>
#include <api/input/files/IWorldGraphLoader.h>
#include "core/Log.h"

namespace xpcf = org::bcom::xpcf;
using namespace SolAR;
using namespace SolAR::datastructure;
using namespace SolAR::api;

int main(int argc, char* argv[])
{
#if NDEBUG
	boost::log::core::get()->set_logging_enabled(false);
#endif
	LOG_ADD_LOG_TO_CONSOLE();
	SRef<xpcf::IComponentManager> xpcfComponentManager = xpcf::getComponentManagerInstance();

    if (xpcfComponentManager->load("SolARTest_ModuleTools_WorldGraphLoader_conf.xml") != org::bcom::xpcf::_SUCCESS)
	{
        std::cerr << "Failed to load the configuration file SolARTest_ModuleTools_WorldGraphLoader_conf.xml" << std::endl;
		return -1;
	}
    auto worldGraphLoader = xpcfComponentManager->resolve<SolAR::api::input::files::IWorldGraphLoader>();

	// load trackables of the world graph
	std::vector<SRef<Trackable>> trackables;
	if (worldGraphLoader->load(trackables) == FrameworkReturnCode::_ERROR_) {
		LOG_ERROR("Error during load the world graph file");
		return -1;
	}

	// display trackables
	int nbTrackables = trackables.size();
	LOG_INFO("Number of trackables: {}", nbTrackables);
	for (int i = 0; i < nbTrackables; ++i) {
		LOG_INFO("============== Trackable {}", i);
		LOG_INFO("Type: {}", trackables[i]->getType());
		LOG_INFO("URL: {}", trackables[i]->getURL());		
		LOG_INFO("Transform: \n{}", trackables[i]->getTransform3D().matrix());
		auto trackable2D = xpcf::utils::dynamic_pointer_cast<Trackable2D>(trackables[i]);
		LOG_INFO("Size: {} x {}", trackable2D->getWidth(), trackable2D->getHeight());
		std::vector<Point3Df> worldCorners;
		trackable2D->getWorldCorners(worldCorners);
		LOG_INFO("World corners (x, y, z)");
		for (const auto& pt : worldCorners)
			LOG_INFO("{} {} {}", pt[0], pt[1], pt[2]);

		switch (trackables[i]->getType())
		{
		case SolAR::datastructure::FIDUCIAL_MARKER: {
			auto marker = xpcf::utils::dynamic_pointer_cast<FiducialMarker>(trackables[i]);
			LOG_INFO("Pattern: \n{}", marker->getPattern().getPatternMatrix());			
			break;
		}
		case SolAR::datastructure::IMAGE_MARKER: {
			auto marker = xpcf::utils::dynamic_pointer_cast<ImageMarker>(trackables[i]);
			break;
		}
		case SolAR::datastructure::QRCODE_MARKER: {
			auto marker = xpcf::utils::dynamic_pointer_cast<QRCode>(trackables[i]);
			LOG_INFO("Code: {}", marker->getCode());
			break;
		}
		default:
			break;
		}		
	}	
    return 0;
}
