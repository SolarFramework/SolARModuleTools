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

#include "SolARBasicSink.h"
#include "core/Log.h"
namespace xpcf = org::bcom::xpcf;


XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::TOOLS::SolARBasicSink)

namespace SolAR {
using namespace datastructure;
using namespace api::sink;
namespace MODULES {
namespace TOOLS {


SolARBasicSink::SolARBasicSink():ConfigurableBase(xpcf::toUUID<ISinkPoseImage>())
{
   declareInterface<api::sink::ISinkPoseImage>(this);
   m_image = nullptr;
   m_pose = Transform3Df::Identity();
   m_newPose = false;
   m_newImage = false;
   m_imageBufferPointer = nullptr;
}

void SolARBasicSink::set( const SRef<Image>& image )
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_image = image->copy();
    m_newImage = true;
    m_newPose = false;
}


FrameworkReturnCode SolARBasicSink::setImageBuffer(unsigned char* imageBufferPointer){
   std::lock_guard<std::mutex> lock(m_mutex);
   m_imageBufferPointer=imageBufferPointer;
   return FrameworkReturnCode::_SUCCESS;
}

void SolARBasicSink::set(const Transform3Df& pose, const SRef<Image>& image )
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_pose = Transform3Df(pose);
    m_image = image->copy();
    m_newPose = true;
    m_newImage = true;
}


SinkReturnCode SolARBasicSink::get( Transform3Df& pose)
{
    SinkReturnCode returnCode = SinkReturnCode::_NOTHING;
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_newPose)
    {
        pose = Transform3Df(m_pose);
        returnCode |= SinkReturnCode::_NEW_POSE;
    }

    if (m_newImage)
    {
        m_newImage = false;

        if(m_imageBufferPointer)
            memcpy(m_imageBufferPointer,(unsigned char*)m_image->data(),m_image->getWidth()* m_image->getHeight()*sizeof(unsigned char)*3);

        returnCode |= SinkReturnCode::_NEW_IMAGE;
    }
    return returnCode;
}


SinkReturnCode SolARBasicSink::tryGet( Transform3Df& pose)
{

    if (m_newPose || m_newImage)
        return get(pose);
    return SinkReturnCode::_NOTHING;
}

}
}
}
