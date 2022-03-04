#ifndef SOLARMAPMANAGER_H
#define SOLARMAPMANAGER_H

#include "api/storage/IMapManager.h"
#include "xpcf/component/ComponentBase.h"
#include "xpcf/component/ConfigurableBase.h"
#include "datastructure/Map.h"
#include <vector>
#include <set>
#include "SolARToolsAPI.h"
#include <boost/filesystem.hpp>
#include <string>
#include <fstream>
#include "core/SerializationDefinitions.h"

namespace SolAR {
namespace MODULES {
namespace TOOLS {

/**
 * @class SolARMapManager
 * @brief <B>Allow to manage all components of a map.</B>
 * <TT>UUID: 8e3c926a-0861-46f7-80b2-8abb5576692c</TT>
 *
 * @SolARComponentInjectablesBegin
 * @SolARComponentInjectable{SolAR::api::storage::IPointCloudManager}
 * @SolARComponentInjectable{SolAR::api::storage::IKeyframesManager}
 * @SolARComponentInjectable{SolAR::api::storage::ICovisibilityGraph}
 * @SolARComponentInjectable{SolAR::api::reloc::IKeyframeRetriever}
 * @SolARComponentInjectablesEnd
 *
 * @SolARComponentPropertiesBegin
 * @SolARComponentProperty{ directory,
 *                          ,
 *                          @SolARComponentPropertyDescString{ "" }}
 * @SolARComponentProperty{ identificationFileName,
 *                          ,
 *                          @SolARComponentPropertyDescString{ "" }}
 * @SolARComponentProperty{ coordinateFileName,
 *                          ,
 *                          @SolARComponentPropertyDescString{ "" }}
 * @SolARComponentProperty{ pointCloudManagerFileName,
 *                          ,
 *                          @SolARComponentPropertyDescString{ "" }}
 * @SolARComponentProperty{ keyframesManagerFileName,
 *                          ,
 *                          @SolARComponentPropertyDescString{ "" }}
 * @SolARComponentProperty{ covisibilityGraphFileName,
 *                          ,
 *                          @SolARComponentPropertyDescString{ "" }}
 * @SolARComponentProperty{ keyframeRetrieverFileName,
 *                          ,
 *                          @SolARComponentPropertyDescString{ "" }}
 * @SolARComponentProperty{ reprojErrorThreshold,
 *                          ,
 *                           @SolARComponentPropertyDescNum{ float, [0..MAX FLOAT], 3.f }}
 * @SolARComponentProperty{ thresConfidence,
 *                          ,
 *                           @SolARComponentPropertyDescNum{ float, [0..MAX FLOAT], 3.f }}
 * @SolARComponentPropertiesEnd
 *
 */

/**
* @class SolARMapManager
* @brief Store all components of a map
*/

class SOLAR_TOOLS_EXPORT_API SolARMapManager : public org::bcom::xpcf::ConfigurableBase,
    public api::storage::IMapManager {
public:
	SolARMapManager();

    ~SolARMapManager() override = default;

	/// @brief Set the map
	/// @param[in] map the data of map
	/// @return FrameworkReturnCode::_SUCCESS_ if all data structures successfully setted, else FrameworkReturnCode::_ERROR.
	FrameworkReturnCode setMap(const SRef<SolAR::datastructure::Map> map) override;

	/// @brief Get the map
	/// @param[out] map the data of map
	/// @return FrameworkReturnCode::_SUCCESS_ if successfully, else FrameworkReturnCode::_ERROR.
	FrameworkReturnCode getMap(SRef<SolAR::datastructure::Map> & map) override;

	/// @brief Get the submap around a centered keyframe
	/// @param[in] idCenteredKeyframe the id of the centered keyframe
	/// @param[in] nbKeyframes the maximum number of keyframes of the submap
	/// @param[out] submap the submap
	/// @return FrameworkReturnCode::_SUCCESS_ if successfully, else FrameworkReturnCode::_ERROR.
	FrameworkReturnCode getSubmap(uint32_t idCenteredKeyframe,
								  uint32_t nbKeyframes,
								  SRef<SolAR::datastructure::Map> & submap) override;

	/// @brief Get local point cloud seen from the keyframe and its neighbors
	/// @param[in] keyframe the keyframe to get local point cloud
	/// @param[in] minWeightNeighbor the weight to get keyframe neighbors
	/// @param[out] localPointCloud the local point cloud
	/// @return FrameworkReturnCode::_SUCCESS if succeed, else FrameworkReturnCode::_ERROR_
    FrameworkReturnCode getLocalPointCloud(const SRef<SolAR::datastructure::Keyframe> keyframe, const float minWeightNeighbor, std::vector<SRef<SolAR::datastructure::CloudPoint>> &localPointCloud) const override;

	/// @brief Add a point cloud to map manager and update visibility of keyframes and covisibility graph
	/// @param[in] cloudPoint the cloud point to add to the map manager
	/// @return FrameworkReturnCode::_SUCCESS if succeed, else FrameworkReturnCode::_ERROR_
    FrameworkReturnCode addCloudPoint(const SRef<SolAR::datastructure::CloudPoint> cloudPoint) override;

	/// @brief Remove a point cloud from map manager and update visibility of keyframes and covisibility graph
	/// @param[in] cloudPoint the cloud point to remove to the map manager
	/// @return FrameworkReturnCode::_SUCCESS if succeed, else FrameworkReturnCode::_ERROR_
    FrameworkReturnCode removeCloudPoint(const SRef<SolAR::datastructure::CloudPoint> cloudPoint) override;

	/// @brief Add a keyframe to map manager
	/// @param[in] keyframe the keyframe to add to the map manager
	/// @return FrameworkReturnCode::_SUCCESS if succeed, else FrameworkReturnCode::_ERROR_
	FrameworkReturnCode addKeyframe(const SRef<SolAR::datastructure::Keyframe> keyframe) override;

	/// @brief Remove a keyframe from map manager and update visibility of point cloud and covisibility graph
	/// @param[in] keyframe the keyframe to remove from the map manager
	/// @return FrameworkReturnCode::_SUCCESS if succeed, else FrameworkReturnCode::_ERROR_
    FrameworkReturnCode removeKeyframe(const SRef<SolAR::datastructure::Keyframe> keyframe) override;

	/// @brief Prune cloud points of a map
    /// @param[in] cloudPoints: the cloud points are checked to prune
	int pointCloudPruning(const std::vector<SRef<SolAR::datastructure::CloudPoint>> &cloudPoints = {}) override;

	/// @brief Prune keyframes of a map
	/// @param[in] keyframes: the keyframes are checked to prune
	int keyframePruning(const std::vector<SRef<SolAR::datastructure::Keyframe>> &keyframes = {}) override;

	/// @brief Save the map to the external file
	/// @return FrameworkReturnCode::_SUCCESS_ if the suppression succeed, else FrameworkReturnCode::_ERROR.
    FrameworkReturnCode saveToFile() const override;

	/// @brief Load the map from the external file
	/// @return FrameworkReturnCode::_SUCCESS_ if the suppression succeed, else FrameworkReturnCode::_ERROR.
	FrameworkReturnCode loadFromFile() override;

	org::bcom::xpcf::XPCFErrorCode onConfigured() override final;
    void unloadComponent () override final;	

private:
	SRef<SolAR::datastructure::Map>							m_map;
	SRef<SolAR::api::storage::IPointCloudManager>			m_pointCloudManager;
	SRef<SolAR::api::storage::IKeyframesManager>			m_keyframesManager;
	SRef<SolAR::api::storage::ICovisibilityGraphManager>	m_covisibilityGraphManager;
	SRef<SolAR::api::reloc::IKeyframeRetriever>				m_keyframeRetriever;

	std::string					m_directory;
	std::string					m_identificationFileName;
	std::string					m_coordinateFileName;
	std::string					m_pcManagerFileName;
	std::string					m_kfManagerFileName;
	std::string					m_covisGraphFileName;
	std::string					m_kfRetrieverFileName;

    float						m_reprojErrorThres = 3.0f;
    float						m_thresConfidence = 0.3f;
    float						m_ratioRedundantObs = 0.9f;
};
}
}
}

#endif // SOLARMAPMANAGER_H
