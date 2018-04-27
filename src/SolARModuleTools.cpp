/**
 * @copyright Copyright (c) 2015 All Right Reserved, B-com http://www.b-com.com/
 *
 * This file is subject to the B<>Com License.
 * All other rights reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 */

#include "ContainerFactory.h"

#include "SolARHomographyValidation.h"
#include "SolARImage2WorldMapper4Marker2D.h"
#include "SolARSBPatternReIndexer.h"
#include "SolARKeypointsReIndexer.h"
#include "SolAR2DTransform.h"
#include "SolAR3DTransform.h"
#include "SolARBasicMatchesFilter.h"

#include <iostream>

namespace xpcf=org::bcom::xpcf;

XPCF_DECLARE_CONTAINER("28b89d39-41bd-451d-b19e-d25a3d7c5797", "SolARModuleTools")

extern "C" XPCF_EXPORT_API void XPCF_getComponent(const boost::uuids::uuid& componentUUID,SRef<xpcf::IComponentIntrospect>& interfaceRef)
{

    boost::uuids::uuid uuidOf_XPCF_CID_SolARHomographyValidation = xpcf::toUUID(SolAR::MODULES::TOOLS::SolARHomographyValidation::UUID );
    boost::uuids::uuid uuidOf_XPCF_CID_SolARBasicMatchesFilter = xpcf::toUUID(SolAR::MODULES::TOOLS::SolARBasicMatchesFilter::UUID );
    boost::uuids::uuid uuidOf_XPCF_CID_SolARImage2WorldMapper4Marker2D = xpcf::toUUID(SolAR::MODULES::TOOLS::SolARImage2WorldMapper4Marker2D::UUID );
    boost::uuids::uuid uuidOf_XPCF_CID_SolARSBPatternReIndexer = xpcf::toUUID(SolAR::MODULES::TOOLS::SolARSBPatternReIndexer::UUID );
    boost::uuids::uuid uuidOf_XPCF_CID_SolARKeypointsReIndexer = xpcf::toUUID(SolAR::MODULES::TOOLS::SolARKeypointsReIndexer::UUID );
    boost::uuids::uuid uuidOf_XPCF_CID_SolAR2DTransform = xpcf::toUUID(SolAR::MODULES::TOOLS::SolAR2DTransform::UUID );
    boost::uuids::uuid uuidOf_XPCF_CID_SolAR3DTransform = xpcf::toUUID(SolAR::MODULES::TOOLS::SolAR3DTransform::UUID );

    if (componentUUID==uuidOf_XPCF_CID_SolARHomographyValidation)
    {
        xpcf::ComponentFactory::createComponent<SolAR::MODULES::TOOLS::SolARHomographyValidation>(interfaceRef);
    }
    else if (componentUUID==uuidOf_XPCF_CID_SolARBasicMatchesFilter)
    {
        xpcf::ComponentFactory::createComponent<SolAR::MODULES::TOOLS::SolARBasicMatchesFilter>(interfaceRef);
    }
    else if (componentUUID==uuidOf_XPCF_CID_SolARImage2WorldMapper4Marker2D)
    {
        xpcf::ComponentFactory::createComponent<SolAR::MODULES::TOOLS::SolARImage2WorldMapper4Marker2D>(interfaceRef);
    }
     else if (componentUUID==uuidOf_XPCF_CID_SolARKeypointsReIndexer)
    {
        xpcf::ComponentFactory::createComponent<SolAR::MODULES::TOOLS::SolARKeypointsReIndexer>(interfaceRef);
    }
    else if (componentUUID==uuidOf_XPCF_CID_SolARSBPatternReIndexer)
    {
        xpcf::ComponentFactory::createComponent<SolAR::MODULES::TOOLS::SolARSBPatternReIndexer>(interfaceRef);
    }
    else if (componentUUID==uuidOf_XPCF_CID_SolAR2DTransform)
    {
        xpcf::ComponentFactory::createComponent<SolAR::MODULES::TOOLS::SolAR2DTransform>(interfaceRef);
    }
    else if (componentUUID==uuidOf_XPCF_CID_SolAR3DTransform)
    {
        xpcf::ComponentFactory::createComponent<SolAR::MODULES::TOOLS::SolAR3DTransform>(interfaceRef);
    }
}

XPCF_BEGIN_COMPONENTS_DECLARATION
XPCF_ADD_COMPONENT(SolAR::MODULES::TOOLS::SolARHomographyValidation::UUID,"Component SolARHomographyValidation")
XPCF_ADD_COMPONENT(SolAR::MODULES::TOOLS::SolARBasicMatchesFilter::UUID,"Component SolARBasicMatchesFilter")
XPCF_ADD_COMPONENT(SolAR::MODULES::TOOLS::SolARImage2WorldMapper4Marker2D::UUID,"Component SolARImage2WorldMapper4Marker2D")
XPCF_ADD_COMPONENT(SolAR::MODULES::TOOLS::SolARKeypointsReIndexer::UUID,"Component SolARKeypointsReIndexer")
XPCF_ADD_COMPONENT(SolAR::MODULES::TOOLS::SolARSBPatternReIndexer::UUID,"Component SolARSBPatternReIndexer")
XPCF_ADD_COMPONENT(SolAR::MODULES::TOOLS::SolAR2DTransform::UUID,"Component SolAR2DTransform")
XPCF_ADD_COMPONENT(SolAR::MODULES::TOOLS::SolAR3DTransform::UUID,"Component SolAR3DTransform")
XPCF_END_COMPONENTS_DECLARATION

