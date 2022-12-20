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

#include "SolARLoopCorrector.h"
#include "core/Log.h"


namespace xpcf  = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::TOOLS::SolARLoopCorrector);


namespace SolAR {
using namespace datastructure;
using namespace api;
namespace MODULES {
namespace TOOLS {


SolARLoopCorrector::SolARLoopCorrector():ConfigurableBase(xpcf::toUUID<SolARLoopCorrector>())
{
    addInterface<SolAR::api::loop::ILoopCorrector>(this);
    declareInjectable<storage::IMapManager>(m_mapManager);
    declareInjectable<storage::ICameraParametersManager>(m_cameraParametersManager);
    declareInjectable<storage::IKeyframesManager>(m_keyframesManager);
    declareInjectable<storage::IPointCloudManager>(m_pointCloudManager);
    declareInjectable<storage::ICovisibilityGraphManager>(m_covisibilityGraphManager);
    declareInjectable<features::IDescriptorMatcherRegion>(m_matcher);
    declareInjectable<geom::I3DTransform>(m_transform3D);
    declareInjectable<geom::IProject>(m_projector);
	LOG_DEBUG("SolARLoopCorrector constructor");
}

void SolARLoopCorrector::getLocalMapPoints(const std::map<uint32_t, SRef<Keyframe> > &connectedKfs, std::vector<SRef<CloudPoint>>& localMapPoints)
{
	// get ids of all cloud point visibilities from keyframes
	std::set<uint32_t> tmpIdxLocalMap;
	for (const auto &kf : connectedKfs) {
		const std::map<uint32_t, uint32_t> &visibility = kf.second->getVisibility();
		for (auto const &v : visibility)
			tmpIdxLocalMap.insert(v.second);
	}
	// get local point cloud
	for (auto const &it : tmpIdxLocalMap) {
		SRef<CloudPoint> point;
		if (m_pointCloudManager->getPoint(it, point) == FrameworkReturnCode::_SUCCESS)
			localMapPoints.push_back(point);
	}
}

FrameworkReturnCode SolARLoopCorrector::correct(const SRef<Keyframe> queryKeyframe, const SRef<Keyframe> detectedLoopKeyframe, const Transform3Df & S_wl_wc, const std::vector<std::pair<uint32_t, uint32_t>> & duplicatedPointsIndices)
{
    // Get current and loop neighbors
    std::vector<uint32_t> kfLoopNeighborsIds;
    std::vector<uint32_t> kfCurrentNeighborsIds;
    m_covisibilityGraphManager->getNeighbors(queryKeyframe->getId(), 20.0, kfCurrentNeighborsIds);
    m_covisibilityGraphManager->getNeighbors(detectedLoopKeyframe->getId(), 20.0, kfLoopNeighborsIds);

    // Compute current keyframe's neighboors similarity poses in loop keyframe and current keyframe worlds c.s.
    std::map<uint32_t, Transform3Df > KfSim_wl_i;
    std::map<uint32_t, Transform3Df > KfSim_i_wc;
    std::map<uint32_t, SRef<Keyframe> > currentConnectedKfs;
	currentConnectedKfs[queryKeyframe->getId()] = queryKeyframe;
	KfSim_i_wc[queryKeyframe->getId()] = queryKeyframe->getPose().inverse();
	KfSim_wl_i[queryKeyframe->getId()] = S_wl_wc * queryKeyframe->getPose();

	for (const auto &it : kfCurrentNeighborsIds) {
		SRef<Keyframe> keyframe;
		if (m_keyframesManager->getKeyframe(it, keyframe) != FrameworkReturnCode::_SUCCESS)
			continue;
		currentConnectedKfs[it] = keyframe;
		KfSim_i_wc[it] = keyframe->getPose().inverse();
		KfSim_wl_i[it] = S_wl_wc * keyframe->getPose();
	}

    // get local map points seen from queryKeyframe and its neighbors	
	std::vector<SRef<CloudPoint>> currentLocalMapPoints;
	getLocalMapPoints(currentConnectedKfs, currentLocalMapPoints);
	
	// correct positions of local map points seen from queryKeyframe and its neighbors
	// convert P_wi to P_wl: P_wl = S_wl_wi * P_wi = S_wl_wc * S_wc_wi * P_wi = S_wl_wc * P_wi
	std::vector<Point3Df> inPosCurrentlocalMapPoints, outPosCurrentlocalMapPoints;
	for (const auto &it : currentLocalMapPoints)
		inPosCurrentlocalMapPoints.push_back(Point3Df(it->getX(), it->getY(), it->getZ()));
	m_transform3D->transform(inPosCurrentlocalMapPoints, S_wl_wc, outPosCurrentlocalMapPoints);
	for (int i = 0; i < currentLocalMapPoints.size(); ++i) {
		currentLocalMapPoints[i]->setX(outPosCurrentlocalMapPoints[i][0]);
		currentLocalMapPoints[i]->setY(outPosCurrentlocalMapPoints[i][1]);
		currentLocalMapPoints[i]->setZ(outPosCurrentlocalMapPoints[i][2]);
	}
	
	// correct pose of keyframes connected to the query keyframe
	// convert T_wi_i to T_wl_i
	// S_wl_i = S_wl_wc * S_wc_i = S_wl_wc * S_wc_wi * S_wi_i = S_wl_wc * S_wi_i
	// Remove scale factor of S_wl_i to obtain T_wl_i
	for (const auto &kf : currentConnectedKfs) {
		SRef<Keyframe> keyframe = kf.second;
		Transform3Df S_wl_i = KfSim_wl_i[kf.first];
		Eigen::Matrix3f scale;
		Eigen::Matrix3f rot;
		S_wl_i.computeScalingRotation(&scale, &rot);
		S_wl_i.linear() = rot;
		keyframe->setPose(S_wl_i);
	}

	// Merges points observed by both loop keyframe and current keyframe neighborhoods
	// update the covisibility graph according when a point merge occurs

	// Get duplicated points between the query keyframe and the loop keyframe
	std::vector < std::pair<SRef<CloudPoint>, SRef<CloudPoint>>> duplicatedCPs;	
	std::set<uint32_t> checkCurrentlocalMapPoints, checkLooplocalMapPoints;
	for (const auto &it : duplicatedPointsIndices) {
		SRef<CloudPoint> cp1, cp2;
		if ((m_pointCloudManager->getPoint(it.first, cp1) == FrameworkReturnCode::_SUCCESS) &&
			(m_pointCloudManager->getPoint(it.second, cp2) == FrameworkReturnCode::_SUCCESS)) {
			duplicatedCPs.push_back(std::make_pair(cp1, cp2));
			checkCurrentlocalMapPoints.insert(it.first);
			checkLooplocalMapPoints.insert(it.second);
		}		
	}

	// Search duplicated points between the local map point of the query keyframe and one of the loop keyframe
	// - project uncheck current local map to each connected loop keyframe
	// - match in region
	// - for each match, find corresponding cloud point, add a pair to the duplicatedCPs
	kfLoopNeighborsIds.push_back(detectedLoopKeyframe->getId());
	for (const auto &it : kfLoopNeighborsIds) {
		// get a connected loop keyframe
		SRef<Keyframe> keyframe;
		if (m_keyframesManager->getKeyframe(it, keyframe) != FrameworkReturnCode::_SUCCESS)
			continue;
		// get visibility's keyframe
		std::map<uint32_t, uint32_t> cpVisibilities = keyframe->getVisibility();
		// get uncheck current local map and their features
		std::vector<SRef<CloudPoint>> uncheckCurrentlocalCPs;
		std::vector<SRef<DescriptorBuffer>> desUncheckCurrentlocalCPs;
		for (const auto &cp : currentLocalMapPoints)
			if (checkCurrentlocalMapPoints.find(cp->getId()) == checkCurrentlocalMapPoints.end()) {
				uncheckCurrentlocalCPs.push_back(cp);
				desUncheckCurrentlocalCPs.push_back(cp->getDescriptor());
			}
		// projection points
		std::vector< Point2Df > projected2DPts;
        SRef<CameraParameters> camParams;
        if (m_cameraParametersManager->getCameraParameters(keyframe->getCameraID(), camParams) != FrameworkReturnCode :: _SUCCESS)
        {
            LOG_WARNING("Camera parameteres with id {} does not exists in the camera parameters manager", keyframe->getCameraID());
            continue;
        }
        m_projector->project(uncheckCurrentlocalCPs, keyframe->getPose(), *camParams, projected2DPts);
		// Matching features
		std::vector<DescriptorMatch> matches;
		m_matcher->match(projected2DPts, desUncheckCurrentlocalCPs, keyframe, matches, 5.f);
		// get duplicated point if finding the corresponding cloud point
		for (const auto &it_match : matches) {
			int idxKeypoint = it_match.getIndexInDescriptorB();
			int idxCPCurrent = it_match.getIndexInDescriptorA();
			std::map< uint32_t, uint32_t>::iterator kpIt = cpVisibilities.find(idxKeypoint);
			if (kpIt != cpVisibilities.end()) {
				uint32_t idCPLoop = kpIt->second;
				SRef<CloudPoint> cpLoop;
				if ((checkLooplocalMapPoints.find(idCPLoop) != checkLooplocalMapPoints.end()) || 
					m_pointCloudManager->getPoint(idCPLoop, cpLoop) != FrameworkReturnCode::_SUCCESS)
					continue;
				duplicatedCPs.push_back(std::make_pair(uncheckCurrentlocalCPs[idxCPCurrent], cpLoop));
				checkCurrentlocalMapPoints.insert(uncheckCurrentlocalCPs[idxCPCurrent]->getId());
				checkLooplocalMapPoints.insert(idCPLoop);
			}
		}
	}

	// Fuse duplicated points, replace the first cloud point by the second one
	// - change keyframe visibility of cp1 to cp2
	// - keyframes see cp1, right now see cp2
	// - update covisibility graph, appear new connections from 2 sets of keyframes seen cp1 and cp2.
	// - supress cp1
	for (const auto &dup : duplicatedCPs) {
		SRef<CloudPoint> cp1 = dup.first;
		SRef<CloudPoint> cp2 = dup.second;
		std::map<uint32_t, uint32_t> visibilities1 = cp1->getVisibility();
		std::map<uint32_t, uint32_t> visibilities2 = cp2->getVisibility();
		for (const auto &vi1 : visibilities1) {
			uint32_t id_kf1 = vi1.first;
			uint32_t id_kp1 = vi1.second;
			SRef<Keyframe> kf1;
			// update visibility of keyframes seen cp1
			if (m_keyframesManager->getKeyframe(id_kf1, kf1) != FrameworkReturnCode::_SUCCESS)
				continue;
			kf1->addVisibility(id_kp1, cp2->getId());
			// move visibility of cp1 to cp2
			cp2->addVisibility(id_kf1, id_kp1);
			// update covisibility graph
			for (const auto &vi2 : visibilities2) {
				uint32_t id_kf2 = vi2.first;
                m_covisibilityGraphManager->increaseEdge(id_kf1, id_kf2, 1.0);
			}			
		}
		// suppress cp1
		m_pointCloudManager->suppressPoint(cp1->getId());
	}

    return FrameworkReturnCode::_SUCCESS;
}


}
}
}
