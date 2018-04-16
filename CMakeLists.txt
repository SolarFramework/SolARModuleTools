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
		src/SolARModuleTools.cpp
		src/SolARPoseHelper.cpp
		src/SolARModuleManagerTools.cpp
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


