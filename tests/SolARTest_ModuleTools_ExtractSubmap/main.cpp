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

#include "xpcf/xpcf.h"
#include "api/storage/IMapManager.h"
#include "api/display/I3DPointsViewer.h"
#include "core/Log.h"
#include <boost/log/core.hpp>
#include <numeric>

using namespace SolAR;
using namespace SolAR::datastructure;
using namespace SolAR::api;

namespace xpcf  = org::bcom::xpcf;

int main(int argc,char** argv)
{
#if NDEBUG
    boost::log::core::get()->set_logging_enabled(false);
#endif

    LOG_ADD_LOG_TO_CONSOLE();
	/* instantiate component manager*/
	/* this is needed in dynamic mode */
	SRef<xpcf::IComponentManager> xpcfComponentManager = xpcf::getComponentManagerInstance();

    if (xpcfComponentManager->load("SolARTest_ModuleTools_ExtractSubmap_conf.xml") != org::bcom::xpcf::_SUCCESS)
	{
        LOG_ERROR("Failed to load the configuration file SolARTest_ModuleTools_ExtractSubmap_conf.xml")
		return -1;
	}

	// declare and create components
	LOG_INFO("Start creating components");

	// storage components
	auto mapManager = xpcfComponentManager->resolve<storage::IMapManager>();
	auto viewer3DPoints = xpcfComponentManager->resolve<display::I3DPointsViewer>();

	// load map
	mapManager->loadFromFile();
    
	// get map
	SRef<Map> globalMap;
	mapManager->getMap(globalMap);
	const SRef<KeyframeCollection>& globalKeyframeCollection = globalMap->getConstKeyframeCollection();
	const SRef<PointCloud>& globalPointCloud = globalMap->getConstPointCloud();
	LOG_INFO("Number of keyframes {} and cloud points {} of the global map", globalKeyframeCollection->getNbKeyframes(), globalPointCloud->getNbPoints());

	SRef<Map> submap;
	mapManager->getSubmap(10, 200, submap);
	SRef<KeyframeCollection> subKeyframeCollection = submap->getConstKeyframeCollection();
	SRef<PointCloud> subPointCloud = submap->getConstPointCloud();
	LOG_INFO("Number of keyframes {} and cloud points {} of the submap", subKeyframeCollection->getNbKeyframes(), subPointCloud->getNbPoints());
	

	// get global point cloud and keyframes	
	std::vector<SRef<Keyframe>> globalKeyframes, subKeyframes;
	std::vector<Transform3Df> globalKeyframesPoses, subKeyframesPoses;
	std::vector<SRef<CloudPoint>> globalCloudPoints, subCloudPoints;
	globalKeyframeCollection->getAllKeyframes(globalKeyframes);
	globalPointCloud->getAllPoints(globalCloudPoints);
	for (const auto& it : globalKeyframes)
		globalKeyframesPoses.push_back(it->getPose());

	subKeyframeCollection->getAllKeyframes(subKeyframes);
	subPointCloud->getAllPoints(subCloudPoints);
	for (const auto& it : subKeyframes)
		subKeyframesPoses.push_back(it->getPose());

	while (viewer3DPoints->display(globalCloudPoints, Transform3Df::Identity(), subKeyframesPoses, {},
		subCloudPoints, globalKeyframesPoses) == FrameworkReturnCode::_SUCCESS);
    


    return 0;
}
