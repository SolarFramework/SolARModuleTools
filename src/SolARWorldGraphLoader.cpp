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

#include "SolARWorldGraphLoader.h"
#include "core/Log.h"


namespace xpcf = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::TOOLS::SolARWorldGraphLoader);

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace TOOLS {

SolARWorldGraphLoader::SolARWorldGraphLoader() : ConfigurableBase(xpcf::toUUID<SolARWorldGraphLoader>())
{
    declareInterface<SolAR::api::input::files::IWorldGraphLoader>(this);
	declareInjectable<SolAR::api::image::IImageLoader>(m_imageLoader);
    declareProperty("filePath", m_filePath);
    LOG_DEBUG("SolARWorldGraphLoader constructor");
}

FrameworkReturnCode SolARWorldGraphLoader::load(std::vector<SRef<SolAR::datastructure::Trackable>>& trackables)
{
	trackables.clear();
	std::ifstream f(m_filePath);
	if (!f.is_open()) {
		LOG_ERROR("Cannot open the world graph file: {}", m_filePath);
		return FrameworkReturnCode::_ERROR_;
	}
	nlohmann::ordered_json j = nlohmann::ordered_json::parse(f, nullptr, false);
	if (j.is_discarded()) {
		LOG_ERROR("Error when parsing file: {}", m_filePath);
		return FrameworkReturnCode::_ERROR_;
	}
	int nbTrackables = j.at("nbTrackables");
	for (int i = 0; i < nbTrackables; ++i) {
		std::string nodeName = "trackable " + std::to_string(i);
		TrackableType type = j[nodeName].at("type");
		switch (type)
		{
		case SolAR::datastructure::FIDUCIAL_MARKER: {
			FiducialMarker marker = j[nodeName].at("data");
			trackables.push_back(xpcf::utils::make_shared<FiducialMarker>(marker));
			break;
		}
		case SolAR::datastructure::IMAGE_MARKER: {
			ImageMarker marker = j[nodeName].at("data");
			m_imageLoader->bindTo<xpcf::IConfigurable>()->getProperty("filePath")->setStringValue(marker.getURL().c_str());
			if (m_imageLoader->reloadImage() != FrameworkReturnCode::_SUCCESS) {
				LOG_ERROR("Cannot load image marker from url: {}", marker.getURL());
				return FrameworkReturnCode::_ERROR_;
			}
			SRef<Image> image;
			m_imageLoader->getImage(image);
			marker.setImage(image);
			trackables.push_back(xpcf::utils::make_shared<ImageMarker>(marker));
			break;
		}
		case SolAR::datastructure::QRCODE_MARKER: {
			QRCode marker = j[nodeName].at("data");
			trackables.push_back(xpcf::utils::make_shared<QRCode>(marker));
			break;
		}
		default:{
			LOG_ERROR("The type trackable {} is not supported", type);
			return FrameworkReturnCode::_ERROR_;
		}			
		}
	}
	f.close();
	return FrameworkReturnCode::_SUCCESS;
}

}
}
}
