// Copyright (C) 2017-2019 Jonathan Müller <jonathanmueller.dev@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#include <boost/log/core.hpp>
#include "xpcf/xpcf.h"
#include <iostream>
#include <xpcf/api/IComponentManager.h>
#include <xpcf/core/helpers.h>
#include <api/input/devices/IARDevice.h>
#include <api/storage/ICameraParametersManager.h>
#include "core/Log.h"

namespace xpcf = org::bcom::xpcf;
using namespace SolAR;
using namespace SolAR::datastructure;
using namespace SolAR::api;

int main(int argc, char* argv[])
{
#if NDEBUG
	boost::log::core::get()->set_logging_enabled(false);
#endif
	LOG_ADD_LOG_TO_CONSOLE();
    try {
        SRef<xpcf::IComponentManager> xpcfComponentManager = xpcf::getComponentManagerInstance();

        if (xpcfComponentManager->load("SolARTest_ModuleTools_CameraParametersManager_conf.xml") != org::bcom::xpcf::_SUCCESS)
        {
            std::cerr << "Failed to load the configuration file SolARTest_ModuleTools_CameraParametersManager_conf.xml" << std::endl;
            return -1;
        }

        auto cameraParametersManager1 = xpcfComponentManager->resolve<SolAR::api::storage::ICameraParametersManager>();
        auto cameraParametersManager2 = xpcfComponentManager->resolve<SolAR::api::storage::ICameraParametersManager>();
        auto ardevice = xpcfComponentManager->resolve<SolAR::api::input::devices::IARDevice>();

        // file name to save camera Parameters
        std::string fileName = "cameraParameters.bin";
        std::vector<uint32_t> ids;
        SolAR::datastructure::CameraRigParameters camRig = ardevice->getCameraParameters();
        for (auto camParams : camRig.cameraParams)
        {
            cameraParametersManager1->addCameraParameters(camParams.second);
            std::cout << "-------------" << std::endl;
            std::cout << "Try to add cameraParameters: " << camParams.second.name << ", get id " << camParams.second.id << std::endl;
            ids.push_back(camParams.second.id);
            std::cout << "All camera parameters in camera parameters manager 1:" << std::endl;
            std::vector<SRef<CameraParameters>> allCamParams;
            cameraParametersManager1->getAllCameraParameters(allCamParams);
            for (auto cp : allCamParams)
                std::cout << "   - camera parameters with id " << cp->id << ", named " << cp->name << std::endl;
        }

        std::cout << std::endl << std::endl;
        std::cout << "Copy camera parameters from manager 1 to manager 2" << std::endl;
        cameraParametersManager2->setCameraParametersCollection(cameraParametersManager1->getConstCameraParametersCollection());
        std::vector<SRef<SolAR::datastructure::CameraParameters>> camParamsVector;
        cameraParametersManager2->getAllCameraParameters(camParamsVector);
        std::cout << "All camera parameters in Manager 2" << std::endl;
        for (auto camParams : camParamsVector)
        {
            std::cout << "   - camera parameters with id " << camParams->id << ", named " << camParams->name << std::endl;
        }

        cameraParametersManager2->saveToFile(fileName);
        std::cout << std::endl << std::endl;
        std::cout << "All camera parameters from manager 2 saved" << std::endl;
        cameraParametersManager2->loadFromFile(fileName);
        std::cout << "All camera parameters from manager 2 reloaded" << std::endl;
        std::cout << std::endl << std::endl;

        for (auto camParams : camRig.cameraParams)
        {
            cameraParametersManager2->addCameraParameters(camParams.second);
            std::cout << "-------------" << std::endl;
            std::cout << "Try to add cameraParameters: " << camParams.second.name << ", get id " << camParams.second.id << std::endl;
            std::cout << "All camera parameters in camera parameters manager 2:" << std::endl;
            std::vector<SRef<CameraParameters>> allCamParams;
            cameraParametersManager2->getAllCameraParameters(allCamParams);
            for (auto cp : allCamParams)
                std::cout << "   - camera parameters with id " << cp->id << ", named " << cp->name << std::endl;
        }


        for (auto id : ids)
        {
            cameraParametersManager2->suppressCameraParameters(id);
            std::cout << "-------------" << std::endl;
            std::cout << "Try to remove cameraParameters from camera parameters manager 2 with id: " << id << std::endl;
            std::cout << "All camera parameters in camera parameters manager 2:" << std::endl;
            std::vector<SRef<CameraParameters>> allCamParams;
            cameraParametersManager2->getAllCameraParameters(allCamParams);
            for (auto cp : allCamParams)
                std::cout << "   - camera parameters with id " << cp->id << ", named " << cp->name << std::endl;
        }
    }
    catch (xpcf::Exception e)
    {
        LOG_ERROR ("The following exception has been catch : {}", e.what());
        return -1;
    }

    return 0;
}
