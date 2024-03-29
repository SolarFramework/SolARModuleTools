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
#ifndef SOLARSINKPOSETEXTUREBUFFEROPENGL_H
#define SOLARSINKPOSETEXTUREBUFFEROPENGL_H

#include "api/sink/ISinkPoseImage.h"
#include "xpcf/component/ConfigurableBase.h"

#include "datastructure/Image.h"

#include "SolARToolsAPI.h"

#include <mutex>

namespace SolAR {
namespace MODULES {
namespace TOOLS {

/**
 * @class SolARBasicSink
 * @brief <B>A Sink for a synchronized pose and texture buffer based on an image buffer useful for AR video see-through pipelines.</B>.
 * <TT>UUID: 85db2f25-4f1c-4e06-9011-e020284bfc4f</TT>
 *
 * This interface allows to make available a pose to a third party application and to update image buffer with a new image.
 */

class SOLAR_TOOLS_EXPORT_API SolARBasicSink : public org::bcom::xpcf::ConfigurableBase,
    public SolAR::api::sink::ISinkPoseImage
{
public:
    SolARBasicSink();
    ~SolARBasicSink() override = default;

    /// @brief Set a new image and pose coming from the pipeline.
    /// @param[in] pose The new pose to be made available to a third party application.
    /// @param[in] image The new image to update a buffer texture when required.
    void set( const SolAR::datastructure::Transform3Df& pose, const SRef<SolAR::datastructure::Image> image ) override;

    /// @brief Set a new image without pose.
    /// @param[in] image The new image to update a buffer texture when required.
    void set( const SRef<SolAR::datastructure::Image> image ) override;

    /// @brief Set a pointer to the texture buffer to update it with the new image when required.
    /// @param[in] imageBuffer the texture buffer uses to contain the new image
    /// @return FrameworkReturnCode::_SUCCESS_ if the texture buffer pointer is well set.
    FrameworkReturnCode setImageBuffer(unsigned char* imageBufferPointer) override;


    /// @brief Provide an access to the new pose and update the texture buffer with the new image.
    /// The implementation of this interface must be thread safe
    /// @param[in,out] pose the new pose made available by the pipeline.
    /// @return return SinkReturnCode::_SUCCESS if a new pose and image are available, otherwise frameworkReturnCode::_ERROR.
    SolAR::api::sink::SinkReturnCode get(SolAR::datastructure::Transform3Df& pose) override;

    /// @brief Provide an access to the new pose and update the texture buffer with the new image only if the image and the pose have been updated by the pipeline.
    /// The implementation of this interface must be thread safe
    /// @param[in,out] pose the new pose made available by the pipeline.
    /// @return return SinkReturnCode::_SUCCESS if a new pose and image are available, otherwise frameworkReturnCode::_ERROR.
    SolAR::api::sink::SinkReturnCode tryGet(SolAR::datastructure::Transform3Df& pose) override;

    void unloadComponent () override final;

private:
    SRef<SolAR::datastructure::Image> m_image;
    SolAR::datastructure::Transform3Df m_pose;

    bool m_newPose;
    bool m_newImage;

    unsigned char* m_imageBufferPointer;
    mutable std::mutex m_mutex;

};

}
}
}

#endif // SOLARSINKPOSETEXTUREBUFFEROPENGL_H
