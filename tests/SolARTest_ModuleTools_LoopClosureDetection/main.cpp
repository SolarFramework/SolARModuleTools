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
#include "api/input/devices/ICamera.h"
#include "api/storage/IMapManager.h"
#include "api/reloc/IKeyframeRetriever.h"
#include "api/storage/ICovisibilityGraphManager.h"
#include "api/storage/IKeyframesManager.h"
#include "api/storage/IPointCloudManager.h"
#include "api/loop/ILoopClosureDetector.h"
#include "api/geom/I3DTransform.h"
#include "api/display/I3DPointsViewer.h"
#include "core/Log.h"
#include <boost/log/core.hpp>

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

    if (xpcfComponentManager->load("SolARTest_ModuleTools_LoopClosureDetection_conf.xml") != org::bcom::xpcf::_SUCCESS)
	{
        LOG_ERROR("Failed to load the configuration file SolARTest_ModuleTools_LoopClosureDetection_conf.xml")
			return -1;
	}

	// declare and create components
	LOG_INFO("Start creating components");

	// storage components
    auto pointCloudManager = xpcfComponentManager->resolve<storage::IPointCloudManager>();
    auto keyframesManager = xpcfComponentManager->resolve<storage::IKeyframesManager>();
    auto covisibilityGraph = xpcfComponentManager->resolve<storage::ICovisibilityGraphManager>();
    auto keyframeRetriever = xpcfComponentManager->resolve<reloc::IKeyframeRetriever>();
	auto mapManager = xpcfComponentManager->resolve<storage::IMapManager>();
	auto loopDetector = xpcfComponentManager->resolve<loop::ILoopClosureDetector>();
	auto camera = xpcfComponentManager->resolve<input::devices::ICamera>();
	auto transform3D = xpcfComponentManager->resolve<geom::I3DTransform>();
	auto viewer3DPoints = xpcfComponentManager->resolve<display::I3DPointsViewer>();

	// Load map from file
	if (mapManager->loadFromFile() != FrameworkReturnCode::_SUCCESS) {
		LOG_INFO("Cannot load map");
		return 0;
	}

	LOG_INFO("Number of keyframes: {}", keyframesManager->getNbKeyframes());
	LOG_INFO("Number of cloud points: {}", pointCloudManager->getNbPoints());

	// get all keyframes and point cloud to display
	std::vector<Transform3Df> keyframePoses;
	std::vector<SRef<Keyframe>> allKeyframes;
	keyframesManager->getAllKeyframes(allKeyframes);	
	for (auto const &it : allKeyframes)
		keyframePoses.push_back(it->getPose());
	std::vector<SRef<CloudPoint>> pointCloud;
	pointCloudManager->getAllPoints(pointCloud);

    // test loop closure with all keyframes
    // SRef<Keyframe> lastKeyframe = allKeyframes[allKeyframes.size() - 1];
    std::vector<SRef<CloudPoint>> localPointCloudTrans;
    bool loopDetected = false;
    SRef<Keyframe> requestedLoopKeyframe, detectedLoopKeyframe;
    for (SRef<Keyframe> currentKeyframe : allKeyframes)
    {
        uint32_t currentKeyframeId = currentKeyframe->getId();
        LOG_DEBUG("Current keyframe id: {}", currentKeyframeId);
        Transform3Df sim3Transform;
        std::vector<std::pair<uint32_t, uint32_t>> duplicatedPointsIndices;
        if (loopDetector->detect(currentKeyframe, detectedLoopKeyframe, sim3Transform, duplicatedPointsIndices) == FrameworkReturnCode::_SUCCESS) {
            // try to transform local point cloud of current keyframe to the detected loop keyframe
            std::vector<SRef<CloudPoint>> localPointCloud;
            std::vector<Point3Df> localPoint3D, localPoint3DTrans;
            mapManager->getLocalPointCloud(currentKeyframe, 10, localPointCloud);
            for (auto it : localPointCloud)
                localPoint3D.push_back(Point3Df(it->getX(), it->getY(), it->getZ()));
            transform3D->transform(localPoint3D, sim3Transform, localPoint3DTrans);
            for (auto it : localPoint3DTrans)
                localPointCloudTrans.push_back(xpcf::utils::make_shared<CloudPoint>(it.getX(), it.getY(), it.getZ()));

            // detected loop keyframe
            LOG_INFO("Requested loop keyframe id: {}", currentKeyframe->getId());
            LOG_INFO("Detected loop keyframe id: {}", detectedLoopKeyframe->getId());
            LOG_INFO("Number of duplicated points: {}", duplicatedPointsIndices.size());
            LOG_INFO("Transform 3D from last keyframe and best detected loop keyframe: \n{}", sim3Transform.matrix());
            // display point cloud
            requestedLoopKeyframe = currentKeyframe;
            loopDetected = true;
        }
        if (loopDetected)
            break;
    }
    if (loopDetected)
        while (viewer3DPoints->display(localPointCloudTrans, requestedLoopKeyframe->getPose(), { detectedLoopKeyframe->getPose() }, {}, pointCloud, keyframePoses) == FrameworkReturnCode::_SUCCESS);
    else
        LOG_INFO("Cannot detect a loop closure");
	

    return 0;
}
