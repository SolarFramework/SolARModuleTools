cmake_minimum_required(VERSION 3.7.2)

##################################################
set (VERSION_NUMBER "0.4.0")
project("SolARModuleTools")
set (SOURCES 
		src/SolARImage2WorldMapper4Marker2D.cpp
		src/SolAR2DTransform.cpp
		src/SolAR3DTransform.cpp
		src/SolARHomographyValidation.cpp
		src/SolARSBPatternReIndexer.cpp
		src/SolARKeypointsReIndexer.cpp
		src/SolARBasicMatchesFilter.cpp
		src/SolARModuleTools.cpp
		src/SolARPoseHelper.cpp
		src/SolARModuleManagerTools.cpp
)

set (HEADERS
		interfaces/SolARImage2WorldMapper4Marker2D.h
		interfaces/SolAR2DTransform.h
		interfaces/SolAR3DTransform.h
		interfaces/SolARHomographyValidation.h
		interfaces/SolARSBPatternReIndexer.h
		interfaces/SolARKeypointsReIndexer.h
		interfaces/SolARModuleManagerTools.h
		interfaces/SolARToolsAPI.h
		interfaces/SolARBasicMatchesFilter.h
)
##################################################

# various macros
include("$ENV{BCOMDEVROOT}/bcomBuild/SolARFramework/solarmacros.cmake")
# config setup
setup()
# process packagedependencies.txt
processPackagedependencies()
# define targets (type library)
defineTargets("library" "")


