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

#include "SolARMapManager.h"
#include "core/Log.h"

namespace xpcf  = org::bcom::xpcf;


XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::TOOLS::SolARMapManager)

namespace SolAR {
using namespace datastructure;
using namespace api::storage;
using namespace api::reloc;
namespace MODULES {
namespace TOOLS {

SolARMapManager::SolARMapManager():ConfigurableBase(xpcf::toUUID<SolARMapManager>())
{
    declareInterface<IMapManager>(this);
    declareInjectable<IPointCloudManager>(m_pointCloudManager);
    declareInjectable<IKeyframesManager>(m_keyframesManager);
    declareInjectable<ICovisibilityGraphManager>(m_covisibilityGraphManager);
    declareInjectable<ICameraParametersManager>(m_cameraParametersManager);
    declareInjectable<IKeyframeRetriever>(m_keyframeRetriever);	
	declareProperty("directory", m_directory);
	declareProperty("identificationFileName", m_identificationFileName);
	declareProperty("coordinateFileName", m_coordinateFileName);
	declareProperty("pointCloudManagerFileName", m_pcManagerFileName);
	declareProperty("keyframesManagerFileName", m_kfManagerFileName);
    declareProperty("cameraParametersManagerFileName", m_cpManagerFileName);
	declareProperty("covisibilityGraphFileName", m_covisGraphFileName);
	declareProperty("keyframeRetrieverFileName", m_kfRetrieverFileName);
	declareProperty("reprojErrorThreshold", m_reprojErrorThres);
	declareProperty("thresConfidence", m_thresConfidence);
	declareProperty("ratioRedundantObs", m_ratioRedundantObs);
	declareProperty("boWFeatureFromMatchedDescriptors", m_boWFeatureFromMatchedDescriptors);
	LOG_DEBUG("SolARMapManager constructor");
}

xpcf::XPCFErrorCode SolARMapManager::onConfigured()
{
	LOG_DEBUG(" SolARMapManager onConfigured");
	m_map = xpcf::utils::make_shared<datastructure::Map>();
	m_map->setPointCloud(m_pointCloudManager->getConstPointCloud());
	m_map->setKeyframeCollection(m_keyframesManager->getConstKeyframeCollection());
    m_map->setCameraParametersCollection(m_cameraParametersManager->getConstCameraParametersCollection());
	m_map->setCovisibilityGraph(m_covisibilityGraphManager->getConstCovisibilityGraph());
	m_map->setKeyframeRetrieval(m_keyframeRetriever->getConstKeyframeRetrieval());
	return xpcf::XPCFErrorCode::_SUCCESS;
}

FrameworkReturnCode SolARMapManager::setMap(const SRef<Map> map)
{
	m_map = map;
	m_pointCloudManager->setPointCloud(m_map->getConstPointCloud());
	m_keyframesManager->setKeyframeCollection(m_map->getConstKeyframeCollection());
    m_cameraParametersManager->setCameraParametersCollection(m_map->getConstCameraParametersCollection());
	m_covisibilityGraphManager->setCovisibilityGraph(m_map->getConstCovisibilityGraph());
	m_keyframeRetriever->setKeyframeRetrieval(m_map->getConstKeyframeRetrieval());
	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARMapManager::getMap(SRef<Map>& map)
{
	map = m_map;

	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARMapManager::getSubmap(uint32_t idCenteredKeyframe, uint32_t nbKeyframes, SRef<SolAR::datastructure::Map>& submap)
{
	// create submap
	submap = xpcf::utils::make_shared<Map>();
	const SRef<KeyframeCollection>& subKeyframeCollection = submap->getConstKeyframeCollection();
    const SRef<CameraParametersCollection>& subcameraParametersCollection = submap->getConstCameraParametersCollection();
	const SRef<PointCloud>& subPointCloud = submap->getConstPointCloud();
	const SRef<KeyframeRetrieval>& subKeyframeRetrieval = submap->getConstKeyframeRetrieval();
	const SRef<CovisibilityGraph>& subCovisiGraph = submap->getConstCovisibilityGraph();

	// get keyframes around the centered keyframe
	std::vector<uint32_t> idKfs;
	idKfs.push_back(idCenteredKeyframe);
	int pos(0);
	while ((pos < idKfs.size()) && (idKfs.size() < nbKeyframes)) {
		std::vector<uint32_t> neighbors;
		m_covisibilityGraphManager->getNeighbors(idKfs[pos], 1.f, neighbors);
		for (auto id : neighbors)
			if ((std::find(idKfs.begin(), idKfs.end(), id) == idKfs.end()) &&
				(idKfs.size() < nbKeyframes))
				idKfs.push_back(id);
		pos++;
	}
	std::vector<SRef<Keyframe>> keyframes;
	m_keyframesManager->getKeyframes(idKfs, keyframes);

	// get all points seen by these keyframes	
	std::set<uint32_t> idCPs;
	for (auto kf : keyframes) {
		const std::map<uint32_t, uint32_t>& visibilities = kf->getVisibility();
		for (auto vi : visibilities)
			idCPs.insert(vi.second);
	}
	std::vector<SRef<CloudPoint>> cloudPoints;
	for (const auto& it : idCPs) {
		SRef<CloudPoint> cp;
		if (m_pointCloudManager->getPoint(it, cp) == FrameworkReturnCode::_SUCCESS)
			cloudPoints.push_back(cp);
	}
	
	// add cloud points	to submap
	for (const auto &cp : cloudPoints)
		subPointCloud->addPoint(*cp);

	// add keyframes to submap
	for (const auto &kf : keyframes)
		subKeyframeCollection->addKeyframe(*kf);
	
    // add camera Parameters to submap
    std::vector<SRef<CameraParameters>> allCamParams;
    if (m_cameraParametersManager->getAllCameraParameters(allCamParams) == FrameworkReturnCode::_SUCCESS) {
        for (const auto &it : allCamParams)
            subcameraParametersCollection->addCameraParameters(*it);
    }

	// find corresponding id of cloud point and keyframe from map to submap
	std::vector<SRef<CloudPoint>> subCloudPoints;
	std::vector<SRef<Keyframe>> subKeyframes;
	subPointCloud->getAllPoints(subCloudPoints);
	subKeyframeCollection->getAllKeyframes(subKeyframes);
	std::map<uint32_t, uint32_t> idCP2Sub, idKf2Sub;
	for (int i = 0; i < cloudPoints.size(); ++i)
		idCP2Sub[cloudPoints[i]->getId()] = subCloudPoints[i]->getId();

	for (int i = 0; i < keyframes.size(); ++i)
		idKf2Sub[keyframes[i]->getId()] = subKeyframes[i]->getId();
	
	// update visibilities of keyframes in submap
	for (const auto &kf : subKeyframes) {
		std::map<uint32_t, uint32_t> visibilities = kf->getVisibility();
		for (auto vi : visibilities) {
			uint32_t idKp = vi.first;
			uint32_t idCp = vi.second;
			kf->removeVisibility(idKp, idCp);
			auto it = idCP2Sub.find(idCp);
			if (it != idCP2Sub.end())
				kf->addVisibility(idKp, it->second);
		}
	}
	
	// update visibilities of keyframes in submap
	for (const auto &cp : subCloudPoints) {
		std::map<uint32_t, uint32_t> visibilities = cp->getVisibility();
		for (auto vi : visibilities) {
			uint32_t idKf = vi.first;
			uint32_t idKp = vi.second;
			cp->removeVisibility(idKf);
			auto it = idKf2Sub.find(idKf);
			if (it != idKf2Sub.end())
				cp->addVisibility(it->second, idKp);
		}
	}
	
	// update covisibility graph
	if (idKf2Sub.size() >= 2) {
		for (auto it1 = idKf2Sub.begin(); std::next(it1) != idKf2Sub.end(); ++it1)
			for (auto it2 = std::next(it1); it2 != idKf2Sub.end(); ++it2) {
				float weight;
				if (m_covisibilityGraphManager->getEdge(it1->first, it2->first, weight) == FrameworkReturnCode::_SUCCESS)
					subCovisiGraph->increaseEdge(it1->second, it2->second, weight);
			}
	}

	// add keyframe retriever of local map to global map
	const SRef<KeyframeRetrieval>& keyframeRetrieval = m_keyframeRetriever->getConstKeyframeRetrieval();
	for (const auto &idKf : idKf2Sub) {
		datastructure::BoWFeature bowDesc;
		datastructure::BoWLevelFeature bowLevelDesc;
		keyframeRetrieval->getBoWFeature(idKf.first, bowDesc);
		keyframeRetrieval->getBoWLevelFeature(idKf.first, bowLevelDesc);
		subKeyframeRetrieval->addDescriptor(idKf.second, bowDesc, bowLevelDesc);
	}
	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARMapManager::getLocalPointCloud(const std::vector<SRef<SolAR::datastructure::Keyframe>> &keyframes,
                                                        std::vector<SRef<SolAR::datastructure::CloudPoint>> &localPointCloud) const
{
    // get all cloud point visibilities from keyframes
    std::map<uint32_t, std::map<uint32_t, uint32_t>> idxLocalMap;
    for (auto const & it : keyframes) {
        std::map<uint32_t, uint32_t> visibility = it->getVisibility();
        for (auto const &v : visibility)
            idxLocalMap[v.second][it->getId()] = v.first;
    }
    // get local point cloud
    for (auto const &it : idxLocalMap) {
        SRef<CloudPoint> point;
        if (m_pointCloudManager->getPoint(it.first, point) == FrameworkReturnCode::_SUCCESS)
            localPointCloud.push_back(point);
        else {
            for (auto const &v : it.second) {
                SRef<Keyframe> keyframe;
                if (m_keyframesManager->getKeyframe(v.first, keyframe) != FrameworkReturnCode::_SUCCESS)
                    continue;
                keyframe->removeVisibility(v.second, it.first);
            }
        }
    }
    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARMapManager::getLocalPointCloud(const SRef<Keyframe> keyframe,
                                                        const float minWeightNeighbor,
                                                        std::vector<SRef<CloudPoint>>& localPointCloud) const
{	
	// get neighbor keyframes of the keyframe
	std::vector<uint32_t> neighKeyframesId;
	m_covisibilityGraphManager->getNeighbors(keyframe->getId(), minWeightNeighbor, neighKeyframesId);
    // get keyframes
    std::vector<SRef<Keyframe>> keyframes;
	for (auto const &it : neighKeyframesId) {
		SRef<Keyframe> keyframe;
        if (m_keyframesManager->getKeyframe(it, keyframe) == FrameworkReturnCode::_SUCCESS)
            keyframes.push_back(keyframe);
	}
    keyframes.push_back(keyframe);
    return getLocalPointCloud(keyframes, localPointCloud);
}

FrameworkReturnCode SolARMapManager::addCloudPoint(const SRef<CloudPoint> cloudPoint)
{
	// add point to cloud
	m_pointCloudManager->addPoint(cloudPoint);
	const std::map<uint32_t, uint32_t>& pointVisibility = cloudPoint->getVisibility();
	std::vector<uint32_t> keyframeIds;
	// add visibility to keyframes
	for (auto const &v : pointVisibility) {		
		SRef<Keyframe> keyframe;
		if (m_keyframesManager->getKeyframe(v.first, keyframe) == FrameworkReturnCode::_SUCCESS) {
			keyframeIds.push_back(v.first);
			keyframe->addVisibility(v.second, cloudPoint->getId());
		}
	}
	// update covisibility graph
	for (int i = 0; i < keyframeIds.size() - 1; i++)
		for (int j = i + 1; j < keyframeIds.size(); j++)
			m_covisibilityGraphManager->increaseEdge(keyframeIds[i], keyframeIds[j], 1);

    // update internal map
    m_map->setPointCloud(m_pointCloudManager->getConstPointCloud());
    m_map->setCovisibilityGraph(m_covisibilityGraphManager->getConstCovisibilityGraph());

	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARMapManager::removeCloudPoint(const SRef<CloudPoint> cloudPoint)
{	
	const std::map<uint32_t, uint32_t>& pointVisibility = cloudPoint->getVisibility();
	std::vector<uint32_t> keyframeIds;
	// remove visibility from keyframes
	for (auto const &v : pointVisibility) {
		SRef<Keyframe> keyframe;
		if (m_keyframesManager->getKeyframe(v.first, keyframe) == FrameworkReturnCode::_SUCCESS) {
			keyframeIds.push_back(v.first);
			keyframe->removeVisibility(v.second, cloudPoint->getId());
		}
	}
	// update covisibility graph
    if (keyframeIds.size() >= 2) {
        for (int i = 0; i < keyframeIds.size() - 1; i++)
            for (int j = i + 1; j < keyframeIds.size(); j++)
                m_covisibilityGraphManager->decreaseEdge(keyframeIds[i], keyframeIds[j], 1);
    }

	// remove cloud point
	m_pointCloudManager->suppressPoint(cloudPoint->getId());

    // update internal map
    m_map->setPointCloud(m_pointCloudManager->getConstPointCloud());
    m_map->setCovisibilityGraph(m_covisibilityGraphManager->getConstCovisibilityGraph());

    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARMapManager::addKeyframe(const SRef<datastructure::Keyframe> keyframe)
{
	// add to keyframe manager
	m_keyframesManager->addKeyframe(keyframe);
	// add to keyframe retriever
	m_keyframeRetriever->addKeyframe(keyframe);

    // update internal map
    m_map->setKeyframeCollection(m_keyframesManager->getConstKeyframeCollection());
    m_map->setKeyframeRetrieval(m_keyframeRetriever->getConstKeyframeRetrieval());

    return FrameworkReturnCode();
}

FrameworkReturnCode SolARMapManager::removeKeyframe(const SRef<Keyframe> keyframe)
{
	std::map<uint32_t, uint32_t> keyframeVisibility = keyframe->getVisibility();
	// remove visibility of point cloud
	for (auto const &v : keyframeVisibility) {
		SRef<CloudPoint> point;
		if (m_pointCloudManager->getPoint(v.second, point) == FrameworkReturnCode::_SUCCESS) {
			// remove this cloud point if the number of visibilities is less than 2
			if (point->getVisibility().size() <= 2)
				this->removeCloudPoint(point);
			else
				point->removeVisibility(keyframe->getId());							
		}
	}
	// remove covisibility graph
	m_covisibilityGraphManager->suppressNode(keyframe->getId());
	// remove keyframe retriever
	m_keyframeRetriever->suppressKeyframe(keyframe->getId());
	// remove keyframe
	m_keyframesManager->suppressKeyframe(keyframe->getId());

    // update internal map
    m_map->setKeyframeCollection(m_keyframesManager->getConstKeyframeCollection());
    m_map->setCovisibilityGraph(m_covisibilityGraphManager->getConstCovisibilityGraph());
    m_map->setKeyframeRetrieval(m_keyframeRetriever->getConstKeyframeRetrieval());

    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARMapManager::addCameraParameters(const SRef<datastructure::CameraParameters> cameraParameters)
{
    // add to camera parameters manager
    m_cameraParametersManager->addCameraParameters(cameraParameters);

    // update internal map
    m_map->setCameraParametersCollection(m_cameraParametersManager->getConstCameraParametersCollection());

    return FrameworkReturnCode();
}

FrameworkReturnCode SolARMapManager::removeCameraParameters(const SRef<datastructure::CameraParameters> cameraParameters)
{
    FrameworkReturnCode result = m_cameraParametersManager->suppressCameraParameters(cameraParameters->id);

    // update internal map
    m_map->setCameraParametersCollection(m_cameraParametersManager->getConstCameraParametersCollection());

    return result;
}

FrameworkReturnCode SolARMapManager::getCameraParameters(const uint32_t id, SRef<SolAR::datastructure::CameraParameters>& cameraParameters)
{
    return m_cameraParametersManager->getCameraParameters(id, cameraParameters);
}

FrameworkReturnCode SolARMapManager::getCameraParameters(const uint32_t id, SolAR::datastructure::CameraParameters & cameraParameters)
{
    return m_cameraParametersManager->getCameraParameters(id, cameraParameters);
}


int SolARMapManager::pointCloudPruning(const std::vector<SRef<CloudPoint>> &cloudPoints)
{
	// get cloud points
	std::vector<SRef<CloudPoint>> cloudPointsPruning;
	if (cloudPoints.size() == 0) {
		m_pointCloudManager->getAllPoints(cloudPointsPruning);
	}
	else {
		cloudPointsPruning = cloudPoints;
	}

	// check reprojection error to prune cloud points
	int count(0);
	for (const auto &it : cloudPointsPruning)
		if ((!it->isValid()) || (it->getReprojError() > m_reprojErrorThres) || 
			(it->getConfidence() < m_thresConfidence) || (it->getVisibility().size() < 2)) {
			this->removeCloudPoint(it);
			count++;
		}

	return count;
}

int SolARMapManager::keyframePruning(const std::vector<SRef<Keyframe>>& keyframes)
{
	std::vector<SRef<Keyframe>> keyframesPruning;
	if (keyframes.size() == 0) {
		m_keyframesManager->getAllKeyframes(keyframesPruning);
	}
	else {
		keyframesPruning = keyframes;
	}

	int nbRemovedKfs(0);
	for (const auto &itKf : keyframesPruning) {
		if (itKf->getId() == 0 || itKf->isFixedPose()) {
			continue;
		}
		const std::map<uint32_t, uint32_t>& pcVisibility = itKf->getVisibility();
		int nbRedundantObs(0);
		for (const auto &itPC : pcVisibility) {
			uint32_t idxPC = itPC.second;
			SRef<CloudPoint> cloudPoint;
			if (m_pointCloudManager->getPoint(idxPC, cloudPoint) == FrameworkReturnCode::_SUCCESS) {
				if (cloudPoint->getVisibility().size() >= 4)
					nbRedundantObs++;
			}
			else
				itKf->removeVisibility(itPC.first, idxPC);
		}
		if (nbRedundantObs > m_ratioRedundantObs * pcVisibility.size()) {
			this->removeKeyframe(itKf);
			nbRemovedKfs++;
		}
	}
	return nbRemovedKfs;
}

FrameworkReturnCode SolARMapManager::visibilityPruning()
{
    std::vector<SRef<Keyframe>> keyframes;
    if (m_keyframesManager->getAllKeyframes(keyframes) != FrameworkReturnCode::_SUCCESS) {
        LOG_ERROR("Failed to get keyframes");
        return FrameworkReturnCode::_ERROR_;
    }
    for (auto& kf : keyframes) {
        // get camera intrinsics
        SRef<CameraParameters> camParams;
        if (m_cameraParametersManager->getCameraParameters(kf->getCameraID(), camParams) != FrameworkReturnCode::_SUCCESS) {
            LOG_ERROR("Failed to get camera parameters of keyframe");
            return FrameworkReturnCode::_ERROR_;
        }
        // get visibilities
        std::map<uint32_t, uint32_t> curVis = kf->getVisibility();
        auto pose = kf->getPose().inverse();
        for (const auto& visi : curVis) {
            Keypoint kpt = kf->getUndistortedKeypoint(visi.first);
            SRef<CloudPoint> cloudPoint;
            if (m_pointCloudManager->getPoint(visi.second, cloudPoint) != FrameworkReturnCode::_SUCCESS) {
                LOG_ERROR("Failed to get cloud point");
                return FrameworkReturnCode::_ERROR_;
            }
            Vector3f ptSolar(cloudPoint->getX(), cloudPoint->getY(), cloudPoint->getZ());
            auto ptCamera = pose*ptSolar;
            bool toRemove = false;
            if (ptCamera(2) == 0.f) {
                toRemove = true;
            }
            else {
                float projX = ptCamera(0) / ptCamera(2) * camParams->intrinsic(0, 0) + camParams->intrinsic(0, 2);
                float projY = ptCamera(1) / ptCamera(2) * camParams->intrinsic(1, 1) + camParams->intrinsic(1, 2);
                float reprojErr = std::sqrt( (projX-kpt.getX())*(projX-kpt.getX()) + (projY-kpt.getY())*(projY-kpt.getY()) );
                if (reprojErr > m_reprojErrorThres)
                    toRemove = true;
            }
            if (toRemove) {
                kf->removeVisibility(visi.first, visi.second);
                cloudPoint->removeVisibility(kf->getId());
            }
        }
    }
    return FrameworkReturnCode::_SUCCESS;
} 

FrameworkReturnCode SolARMapManager::saveToFile() const
{
	if (m_pointCloudManager->getNbPoints() == 0)
	{
		LOG_WARNING("Map is empty: nothing to save");
	}
	else
	{
		LOG_INFO("Saving the map to file...");
		boost::filesystem::create_directories(boost::filesystem::path(m_directory.c_str()));
		LOG_DEBUG("Save identification");
		std::ofstream ofs_iden(m_directory + "/" + m_identificationFileName, std::ios::binary);
		OutputArchive oa_iden(ofs_iden);
		SRef<Identification> identification;
		m_map->getIdentification(identification);
		oa_iden << identification;
		ofs_iden.close();
		LOG_DEBUG("Save coordinate system");
		std::ofstream ofs_coor(m_directory + "/" + m_coordinateFileName, std::ios::binary);
		OutputArchive oa_coor(ofs_coor);
		SRef<CoordinateSystem> coordinateSystem;
		m_map->getCoordinateSystem(coordinateSystem);
		oa_coor << coordinateSystem;
		ofs_coor.close();
		LOG_DEBUG("Save point cloud manager");
		if (m_pointCloudManager->saveToFile(m_directory + "/" + m_pcManagerFileName) == FrameworkReturnCode::_ERROR_)
			return FrameworkReturnCode::_ERROR_;
		LOG_DEBUG("Save keyframes manager");
		if (m_keyframesManager->saveToFile(m_directory + "/" + m_kfManagerFileName) == FrameworkReturnCode::_ERROR_)
			return FrameworkReturnCode::_ERROR_;
        LOG_DEBUG("Save cameraParameters manager");
        if (m_cameraParametersManager->saveToFile(m_directory + "/" + m_cpManagerFileName) == FrameworkReturnCode::_ERROR_)
            return FrameworkReturnCode::_ERROR_;
		LOG_DEBUG("Save covisibility graph");
		if (m_covisibilityGraphManager->saveToFile(m_directory + "/" + m_covisGraphFileName) == FrameworkReturnCode::_ERROR_)
			return FrameworkReturnCode::_ERROR_;
		LOG_DEBUG("Save keyframe retriever");
		if (m_keyframeRetriever->saveToFile(m_directory + "/" + m_kfRetrieverFileName) == FrameworkReturnCode::_ERROR_)
			return FrameworkReturnCode::_ERROR_;
		LOG_INFO("Save done!");
	}

	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARMapManager::loadFromFile()
{
	LOG_INFO("Loading the map from file...");
	LOG_DEBUG("Load identification");
	std::ifstream ifs_iden(m_directory + "/" + m_identificationFileName, std::ios::binary);
	if (!ifs_iden.is_open())
    {
        LOG_WARNING("Cannot load map identification file with url: {}", m_directory + "/" + m_identificationFileName);
		return FrameworkReturnCode::_ERROR_;
    }
	InputArchive ia_iden(ifs_iden);
	SRef<Identification> identification;
	ia_iden >> identification;
	m_map->setIdentification(identification);
	ifs_iden.close();
	LOG_DEBUG("Load coordinate system");
	std::ifstream ifs_coor(m_directory + "/" + m_coordinateFileName, std::ios::binary);
	if (!ifs_coor.is_open())
    {
        LOG_WARNING("Cannot load map coordinate file with url: {}", m_directory + "/" + m_coordinateFileName);
		return FrameworkReturnCode::_ERROR_;
    }
    InputArchive ia_coor(ifs_coor);
	SRef<CoordinateSystem> coordinateSystem;
	ia_coor >> coordinateSystem;
	m_map->setCoordinateSystem(coordinateSystem);
	ifs_coor.close();
	LOG_DEBUG("Load point cloud manager");
	if (m_pointCloudManager->loadFromFile(m_directory + "/" + m_pcManagerFileName) == FrameworkReturnCode::_ERROR_)
    {
        LOG_WARNING("Cannot load map point cloud manager file with url: {}", m_directory + "/" + m_pcManagerFileName);
        return FrameworkReturnCode::_ERROR_;
    }
	m_map->setPointCloud(m_pointCloudManager->getConstPointCloud());
	LOG_DEBUG("Load keyframes manager");
	if (m_keyframesManager->loadFromFile(m_directory + "/" + m_kfManagerFileName) == FrameworkReturnCode::_ERROR_)
    {
        LOG_WARNING("Cannot load map keyframe manager file with url: {}", m_directory + "/" + m_kfManagerFileName);
        return FrameworkReturnCode::_ERROR_;
    }
	m_map->setKeyframeCollection(m_keyframesManager->getConstKeyframeCollection());
    LOG_DEBUG("Load camera parameters manager");
    if (m_cameraParametersManager->loadFromFile(m_directory + "/" + m_cpManagerFileName) == FrameworkReturnCode::_ERROR_)
    {
        LOG_WARNING("Cannot load map camera parameters manager file with url: {}", m_directory + "/" + m_cpManagerFileName);
        return FrameworkReturnCode::_ERROR_;
    }
    m_map->setCameraParametersCollection(m_cameraParametersManager->getConstCameraParametersCollection());
	LOG_DEBUG("Load covisibility graph");
	if (m_covisibilityGraphManager->loadFromFile(m_directory + "/" + m_covisGraphFileName) == FrameworkReturnCode::_ERROR_)
    {
        LOG_WARNING("Cannot load map covisibility graph file with url: {}", m_directory + "/" + m_covisGraphFileName);
		return FrameworkReturnCode::_ERROR_;
    }
	m_map->setCovisibilityGraph(m_covisibilityGraphManager->getConstCovisibilityGraph());
	LOG_DEBUG("Load keyframe retriever");
	if (m_keyframeRetriever->loadFromFile(m_directory + "/" + m_kfRetrieverFileName) == FrameworkReturnCode::_ERROR_)
    {
        LOG_WARNING("Cannot load map keyframe retriever file with url: {}", m_directory + "/" + m_kfRetrieverFileName);
        return FrameworkReturnCode::_ERROR_;
    }
	m_map->setKeyframeRetrieval(m_keyframeRetriever->getConstKeyframeRetrieval());
	if (m_pointCloudManager->getNbPoints() == 0)
	{
		LOG_WARNING("Loaded map is empty");
	}
	LOG_INFO("Load done!");
	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARMapManager::deleteFile()
{
    LOG_INFO("Deleting the map files...");
    LOG_DEBUG("Delete identification");
    if (boost::filesystem::remove(m_directory + "/" + m_identificationFileName)) {
        LOG_DEBUG("Identification file deleted");
    }
    else {
        LOG_DEBUG("No identification file to delete");
    }
    LOG_DEBUG("Delete coordinate system");
    if (boost::filesystem::remove(m_directory + "/" + m_coordinateFileName)) {
        LOG_DEBUG("Coordinate system file deleted");
    }
    else {
        LOG_DEBUG("No coordinate system file to delete");
    }
    LOG_DEBUG("Delete point cloud manager");
    if (boost::filesystem::remove(m_directory + "/" + m_pcManagerFileName)) {
        LOG_DEBUG("Point cloud manager file deleted");
    }
    else {
        LOG_DEBUG("No point cloud manager file to delete");
    }
    LOG_DEBUG("Delete keyframes manager");
    if (boost::filesystem::remove(m_directory + "/" + m_kfManagerFileName)) {
        LOG_DEBUG("Keyframes manager file deleted");
    }
    else {
        LOG_DEBUG("No keyframes manager file to delete");
    }
    LOG_DEBUG("Delete camera parameters manager");
    if (boost::filesystem::remove(m_directory + "/" + m_cpManagerFileName)) {
        LOG_DEBUG("camera parameters manager file deleted");
    }
    else {
        LOG_DEBUG("No camera parameters manager file to delete");
    }
    LOG_DEBUG("Delete covisibility graph");
    if (boost::filesystem::remove(m_directory + "/" + m_covisGraphFileName)) {
        LOG_DEBUG("Covisibility graph file deleted");
    }
    else {
        LOG_DEBUG("No covisibility graph file to delete");
    }
    LOG_DEBUG("Delete keyframe retriever");
    if (boost::filesystem::remove(m_directory + "/" + m_kfRetrieverFileName)) {
        LOG_DEBUG("Keyframe retriever file deleted");
    }
    else {
        LOG_DEBUG("No keyframe retriever file to delete");
    }
    LOG_INFO("Deletion done!");

    return FrameworkReturnCode::_SUCCESS;
}

}
}
}
