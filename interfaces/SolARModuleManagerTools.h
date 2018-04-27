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

#ifndef SOLARMODULEMANAGERTOOLS_H
#define SOLARMODULEMANAGERTOOLS_H

#include "IComponentManager.h"
#include "SolARToolsAPI.h"

#include "api/features/ISBPatternReIndexer.h"
#include "api/features/IKeypointsReIndexer.h"
#include "api/features/IMatchesFilter.h"

#include "api/geom/I2DTransform.h"
#include "api/geom/I3DTransform.h"
#include "api/geom/IImage2WorldMapper.h"

#include "api/solver/pose/IHomographyValidation.h"

using namespace std;

namespace xpcf  = org::bcom::xpcf;
namespace SolAR {
namespace MODULES {
namespace TOOLS {
namespace UUID {
// declaration of components UUIDs
const string TRANSFORM2D="edcedc0a-9841-4377-aea1-9fa9fdb46fde";
const string TRANSFORM3D="f05dd955-33bd-4d52-8717-93ad298ed3e3";
const string BASIC_MATCHES_FILTER="cbb620c3-a7fc-42d7-bcbf-f59b475b23b0";
const string HOMOGRAPHY_VALIDATION="112f9f03-79c1-4393-b8f3-e02227bebfed";
const string IMAGE2WORLD_MAPPER="6fed0169-4f01-4545-842a-3e2425bee248";
const string KEYPOINTS_REINDEXER="c2836cc0-0344-4956-8959-84936fb4bcf2";
const string SBPATTERN_REINDEXER="a2ef5542-029e-4fce-9974-0aea14b29d6f";
}  // End namespace UUID


// class SolARComponentManagerOpencv declaration
class SOLAR_TOOLS_EXPORT_API SolARModuleManagerTools
{
public:
    SolARModuleManagerTools();
    SolARModuleManagerTools(const char *iniFile);
    ~SolARModuleManagerTools() = default;

    template <class T>
    int createComponent(string uuid, SRef<T> &compRef);

    template <class T>
    SRef<T> createComponent(string uuid);

    inline bool isLoaded() const {return loaded;}

protected:
    SRef<xpcf::IComponentManager> m_xpcfComponentManager;

private:
    bool loaded;
};

template <class T>
SRef<T> SolARModuleManagerTools::createComponent(string uuid)
{
    SRef<T> output;
    if (createComponent(uuid, output))
        return output;
    else
        return nullptr;
}

template <class T>
int SolARModuleManagerTools::createComponent(string uuid, SRef<T> &compRef)
{
    boost::uuids::string_generator gen;
    int res;

    if (uuid == UUID::TRANSFORM2D) // Transform 2D component
    {
        res=m_xpcfComponentManager->createComponent(gen(uuid), gen(api::geom::I2DTransform::UUID), compRef);
        if (res == -1)
             LOG_ERROR("Transform 2D component creation has failed");
        return res;
    }

    else if (uuid == UUID::TRANSFORM3D) // 3D overlay component
    {
        res=m_xpcfComponentManager->createComponent(gen(uuid), gen(api::geom::I3DTransform::UUID), compRef);
        if (res == -1)
             LOG_ERROR("Transform 3D component creation has failed");
        return res;
    }

    else if (uuid == UUID::BASIC_MATCHES_FILTER) // Basic matches filter component
    {
        res=m_xpcfComponentManager->createComponent(gen(uuid), gen(api::features::IMatchesFilter::UUID), compRef);
        if (res == -1)
             LOG_ERROR("Basic Matches Filter component creation has failed");
        return res;
    }

    else if (uuid == UUID::HOMOGRAPHY_VALIDATION) // homography validation
    {
        res=m_xpcfComponentManager->createComponent(gen(uuid), gen(api::solver::pose::IHomographyValidation::UUID), compRef);
        if (res == -1)
             LOG_ERROR("Homography validation component creation has failed");
        return res;
    }

    else if (uuid == UUID::IMAGE2WORLD_MAPPER) // Image to world mapper component
    {
        res=m_xpcfComponentManager->createComponent(gen(uuid), gen(api::geom::IImage2WorldMapper::UUID), compRef);
        if (res == -1)
             LOG_ERROR("Image to world mapper component creation has failed");
        return res;
    }

    else if (uuid == UUID::KEYPOINTS_REINDEXER) // Keypoints reindexer component
    {
        res=m_xpcfComponentManager->createComponent(gen(uuid), gen(api::features::IKeypointsReIndexer::UUID), compRef);
        if (res == -1)
             LOG_ERROR("Keypoints reindexer component creation has failed");
        return res;
    }

    else if (uuid == UUID::SBPATTERN_REINDEXER) // posefinder
    {
        res=m_xpcfComponentManager->createComponent(gen(uuid), gen(api::features::ISBPatternReIndexer::UUID), compRef);
        if (res == -1)
             LOG_ERROR("Squared binary pattern reindexer component creation has failed");
        return res;
    }

    return -1;
}
}   // End namespace TOOLS
}   // End namespace MODULES
}   // End namespace SolAR

#endif // SOLARMODULEMANAGERTOOLS_H
