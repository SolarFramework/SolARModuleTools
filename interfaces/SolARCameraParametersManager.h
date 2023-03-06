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

#ifndef SOLARCAMERAPARAMETERSMANAGER_H
#define SOLARCAMERAPARAMETERSMANAGER_H

#include "api/storage/ICameraParametersManager.h"
#include "xpcf/component/ComponentBase.h"
#include "SolARToolsAPI.h"
#include <core/SerializationDefinitions.h>

namespace SolAR {
namespace MODULES {
namespace TOOLS {
/**
 * @class SolARCameraParametersManager
 * @brief A storage component to store a persistent set of CameraParameters, based on a CameraParametersCollection.
 * <TT>UUID: e046cf87-d0a4-4c6f-af3d-18dc70881a34</TT>
*/
class SOLAR_TOOLS_EXPORT_API SolARCameraParametersManager : public org::bcom::xpcf::ComponentBase,
        public SolAR::api::storage::ICameraParametersManager {
public:

    /// @brief SolARCameraParametersManager default constructor
    SolARCameraParametersManager();

    /// @brief SolARCameraParametersManager default destructor
    ~SolARCameraParametersManager() = default;

    /// @brief This method allow to add camera parameters to the CameraParameters manager component
    /// @param[in] cameraParameters the camera parameters to add to the set of persistent CameraParameters collection.
	/// @return FrameworkReturnCode::_SUCCESS_ if the addition succeed, else FrameworkReturnCode::_ERROR.
    FrameworkReturnCode addCameraParameters(const SRef<SolAR::datastructure::CameraParameters> cameraParameters) override;

    /// @brief This method allow to add camera parameters to the CameraParameters manager component
    /// @param[in] cameraParameters the camera parameters to add to the set of persistent CameraParameters collection.
	/// @return FrameworkReturnCode::_SUCCESS_ if the addition succeed, else FrameworkReturnCode::_ERROR.
    FrameworkReturnCode addCameraParameters(SolAR::datastructure::CameraParameters & cameraParameters) override;

    /// @brief This method allows to get camera parameters by their id
    /// @param[in] id of the camera parameters to get
    /// @param[out] a camera parameters stored in the CameraParameters manager
	/// @return FrameworkReturnCode::_SUCCESS_ if succeed, else FrameworkReturnCode::_ERROR.
    FrameworkReturnCode getCameraParameters(const uint32_t id, SRef<SolAR::datastructure::CameraParameters> & cameraParameters) const override;

    /// @brief This method allows to get cameraParameters by their id
    /// @param[in] id id of the cameraParameters to get
    /// @param[out] cameraParameters cameraParameters stored in the CameraParameters manager
    /// @return FrameworkReturnCode::_SUCCESS_ if succeed, else FrameworkReturnCode::_ERROR.
    FrameworkReturnCode getCameraParameters(const uint32_t id, SolAR::datastructure::CameraParameters & cameraParameters) const override;

    /// @brief This method allows to get a collection of camera parameters by their ids
    /// @param[in] a vector of ids of the camera parameters to get
    /// @param[out] a vector of camera parameters stored in the CameraParameters manager
	/// @return FrameworkReturnCode::_SUCCESS_ if succeed, else FrameworkReturnCode::_ERROR.
    FrameworkReturnCode getCameraParameters(const std::vector<uint32_t> & ids, std::vector<SRef<SolAR::datastructure::CameraParameters>>& cameraParameters) const override;

    /// @brief This method allows to get all camera parameters
    /// @param[out] the set of camera parameters
	/// @return FrameworkReturnCode::_SUCCESS_ if succeed, else FrameworkReturnCode::_ERROR.
    FrameworkReturnCode getAllCameraParameters(std::vector<SRef<SolAR::datastructure::CameraParameters>>& cameraParameters) const override;

    /// @brief This method allow to suppress camera parameters by their id
    /// @param[in] id of the camera parameters to suppress
	/// @return FrameworkReturnCode::_SUCCESS_ if the suppression succeed, else FrameworkReturnCode::_ERROR.
    FrameworkReturnCode suppressCameraParameters(const uint32_t id) override;

    /// @brief This method allows to know if camera parametersis already stored in the component
    /// @param[in] Id of these camera parameters
	/// @return true if exist, else false
    bool isExistCameraParameters(const uint32_t id) const override;

    /// @brief This method allows to get the number of camera parameters stored in the point cloud
    /// @return The number of camera parameters
    int getNbCameraParameters() const override;

    /// @brief This method allows to save the camera parameters to the external file
	/// @param[in] file the file name
	/// @return FrameworkReturnCode::_SUCCESS_ if the suppression succeed, else FrameworkReturnCode::_ERROR.
    FrameworkReturnCode saveToFile(const std::string& file) const override;

    /// @brief This method allows to load camera parameters from the external file
	/// @param[in] the file name
	/// @return FrameworkReturnCode::_SUCCESS_ if the suppression succeed, else FrameworkReturnCode::_ERROR.
	FrameworkReturnCode loadFromFile(const std::string& file) override;

    /// @brief This method returns the camera parameters collection
    /// @return the camera parameters collection
    const SRef<datastructure::CameraParametersCollection> & getConstCameraParametersCollection() const override;

    /// @brief This method returns the camera parameters collection
    /// @param[out] cameraParametersCollection the camera parameters collection of map
    /// @return the camera parameters collection
    std::unique_lock<std::mutex> getCameraParametersCollection(SRef<datastructure::CameraParametersCollection>& cameraParametersCollection) override;

    /// @brief This method is to set the camera parameters collection
    /// @param[in] cameraParametersCollection the camera parameters collection of map
    void setCameraParametersCollection(const SRef<datastructure::CameraParametersCollection> cameraParametersCollection) override;

    void unloadComponent () override final;


 private:
    struct Compare
    {
        bool operator()(const SolAR::datastructure::CameraParameters lcam, const SolAR::datastructure::CameraParameters rcam) const /* noexcept */ { return lcam.intrinsic(0,0) > lcam.intrinsic(0,0); }
    };

    SRef<SolAR::datastructure::CameraParametersCollection> m_cameraParametersCollection;
};

}
}
}

#endif // SOLARCAMERAPARAMETERSMANAGER_H
