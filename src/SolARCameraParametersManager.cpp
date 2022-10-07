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

#include "SolARCameraParametersManager.h"
#include "xpcf/component/ComponentFactory.h"
#include "core/Log.h"

namespace xpcf  = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::TOOLS::SolARCameraParametersManager);

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace TOOLS {

SolARCameraParametersManager::SolARCameraParametersManager():ComponentBase(xpcf::toUUID<SolARCameraParametersManager>())
{
    addInterface<api::storage::ICameraParametersManager>(this);
    m_cameraParametersCollection = xpcf::utils::make_shared<CameraParametersCollection>();
    LOG_DEBUG("SolARCameraParametersManager constructor");
}

FrameworkReturnCode SolARCameraParametersManager::addCameraParameters(const SRef<CameraParameters> cameraParameters)
{
    m_cameraParametersCollection->acquireLock();
    const auto [lb, ub] = m_camParamsSet.equal_range(*cameraParameters);
    if (lb != m_camParamsSet.end())
    {
        for (auto it = lb; it != ub; it++)
        {
            if (it->intrinsic == cameraParameters->intrinsic &&
                it->distortion == cameraParameters->distortion &&
                it->type == cameraParameters->type &&
                it->resolution.width == cameraParameters->resolution.width &&
                it->resolution.height == cameraParameters->resolution.height &&
                it->name == cameraParameters->name)
            {
                // A Camera parameters with the same characteristics already exists.
                cameraParameters->id = it->id;
                return FrameworkReturnCode::_SUCCESS;
            }
        }
    }
    m_camParamsSet.insert(*cameraParameters);
    return m_cameraParametersCollection->addCameraParameters(cameraParameters);
}

FrameworkReturnCode SolARCameraParametersManager::addCameraParameters(CameraParameters & cameraParameters)
{
    m_cameraParametersCollection->acquireLock();
    const auto [lb, ub] = m_camParamsSet.equal_range(cameraParameters);
    if (lb != m_camParamsSet.end())
    {
        for (auto it = lb; it != ub; it++)
        {
            if (it->intrinsic == cameraParameters.intrinsic &&
                it->distortion == cameraParameters.distortion &&
                it->type == cameraParameters.type &&
                it->resolution.width == cameraParameters.resolution.width &&
                it->resolution.height == cameraParameters.resolution.height &&
                it->name == cameraParameters.name)
            {
                // A Camera parameters with the same characteristics already exists.
                cameraParameters.id = it->id;
                return FrameworkReturnCode::_SUCCESS;
            }
        }
    }
    m_camParamsSet.insert(cameraParameters);
    return m_cameraParametersCollection->addCameraParameters(cameraParameters);
}

FrameworkReturnCode SolARCameraParametersManager::getCameraParameters(const uint32_t id, SRef<CameraParameters> & cameraParameters) const
{
    m_cameraParametersCollection->acquireLock();
    return m_cameraParametersCollection->getCameraParameters(id, cameraParameters);
}

FrameworkReturnCode SolARCameraParametersManager::getCameraParameters(const uint32_t id, CameraParameters & cameraParameters) const
{
    m_cameraParametersCollection->acquireLock();
    return m_cameraParametersCollection->getCameraParameters(id, cameraParameters);
}

FrameworkReturnCode SolARCameraParametersManager::getCameraParameters(const std::vector<uint32_t>& ids, std::vector<SRef<CameraParameters>>& cameraParameters) const
{
    m_cameraParametersCollection->acquireLock();
    return m_cameraParametersCollection->getCameraParameters(ids, cameraParameters);
}

FrameworkReturnCode SolARCameraParametersManager::getAllCameraParameters(std::vector<SRef<CameraParameters>>& cameraParameters) const
{
    m_cameraParametersCollection->acquireLock();
    return m_cameraParametersCollection->getAllCameraParameters(cameraParameters);
}

FrameworkReturnCode SolARCameraParametersManager::suppressCameraParameters(const uint32_t id)
{
    m_cameraParametersCollection->acquireLock();
    CameraParameters cameraParameters;
    m_cameraParametersCollection->getCameraParameters(id, cameraParameters);
    const auto [lb, ub] = m_camParamsSet.equal_range(cameraParameters);
    if (lb != m_camParamsSet.end())
    {
        for (auto it = lb; it != ub; it++)
        {
            if (it->intrinsic == cameraParameters.intrinsic &&
                it->distortion == cameraParameters.distortion &&
                it->type == cameraParameters.type &&
                it->resolution.width == cameraParameters.resolution.width &&
                it->resolution.height == cameraParameters.resolution.height &&
                it->name == cameraParameters.name)
            {
               m_camParamsSet.erase(it);
               break;
            }
        }
    }
    return m_cameraParametersCollection->suppressCameraParameters(id);
}

bool SolARCameraParametersManager::isExistCameraParameters(const uint32_t id) const
{
    m_cameraParametersCollection->acquireLock();
    return m_cameraParametersCollection->isExistCameraParameters(id);
}

int SolARCameraParametersManager::getNbCameraParameters() const
{
    m_cameraParametersCollection->acquireLock();
    return m_cameraParametersCollection->getNbCameraParameters();
}

FrameworkReturnCode SolARCameraParametersManager::saveToFile(const std::string& file) const
{
	std::ofstream ofs(file, std::ios::binary);
	OutputArchive oa(ofs);
    oa << m_cameraParametersCollection;
	ofs.close();
	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARCameraParametersManager::loadFromFile(const std::string& file)
{
    std::ifstream ifs(file, std::ios::binary);
	if (!ifs.is_open())
		return FrameworkReturnCode::_ERROR_;
    InputArchive ia(ifs);
    ia >> m_cameraParametersCollection;
	ifs.close();
    m_camParamsSet.clear();
    std::vector<SRef<CameraParameters>> cameraParameters;
    getAllCameraParameters(cameraParameters);
    for (auto camParams : cameraParameters)
    {
        m_camParamsSet.insert(*camParams);
    }
	return FrameworkReturnCode::_SUCCESS;
}

const SRef<datastructure::CameraParametersCollection>& SolARCameraParametersManager::getConstCameraParametersCollection() const
{
    return m_cameraParametersCollection;
}

std::unique_lock<std::mutex> SolARCameraParametersManager::getCameraParametersCollection(SRef<datastructure::CameraParametersCollection>& cameraParametersCollection)
{
    cameraParametersCollection = m_cameraParametersCollection;
    return m_cameraParametersCollection->acquireLock();
}

void SolARCameraParametersManager::setCameraParametersCollection(const SRef<datastructure::CameraParametersCollection> cameraParametersCollection)
{
    std::vector<SRef<CameraParameters>> camParams;
    if (cameraParametersCollection->getAllCameraParameters(camParams) != FrameworkReturnCode::_SUCCESS)
    {
        LOG_WARNING("cannot get all camera parameters from the camera parameters collection");
        return;
    }

    // Add camera parameters to the collection
    for (auto camParam : camParams)
    {
        addCameraParameters(camParam);
    }
}


}
}
}
