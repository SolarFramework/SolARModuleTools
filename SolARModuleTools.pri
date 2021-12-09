HEADERS += interfaces/SolARImage2WorldMapper4Marker2D.h \    
interfaces/SolAR2DTransform.h \
interfaces/SolAR3DTransform.h \
interfaces/SolARHomographyValidation.h \
interfaces/SolARSBPatternReIndexer.h \
interfaces/SolARKeypointsReIndexer.h \
interfaces/SolARMapManager.h \
interfaces/SolARMapFilter.h \
interfaces/SolARToolsAPI.h \
interfaces/SolARModuleTools_traits.h \
interfaces/SolARBasicMatchesFilter.h \
interfaces/SolARKeyframeSelector.h \
interfaces/SolARBasicSink.h \
interfaces/SolARBasicSource.h \
interfaces/SolARPointCloudManager.h \
interfaces/SolARKeyframesManager.h \
interfaces/SolARCovisibilityGraphManager.h \
interfaces/SolARBoostCovisibilityGraph.h \
interfaces/SolARLoopCorrector.h \
interfaces/SolARLoopClosureDetector.h \
interfaces/SolAR3D3DcorrespondencesFinder.h \
interfaces/SolAR3DTransformEstimationSACFrom3D3D.h \
interfaces/SolARFiducialMarkerPoseEstimator.h \
interfaces/SolARSLAMBootstrapper.h \
interfaces/SolARSLAMTracking.h \
interfaces/SolARSLAMMapping.h \
interfaces/SolAROverlapDetector.h \
interfaces/SolARMapUpdate.h \
interfaces/SolARStereoDepthEstimation.h \
interfaces/SolARStereoReprojection.h \
interfaces/SolARStereoFeatureExtractionAndDepthEstimation.h \
interfaces/SolARStereoBootstrapper.h \
interfaces/SolARWorldGraphLoader.h


SOURCES += src/SolARImage2WorldMapper4Marker2D.cpp \
    src/SolAR2DTransform.cpp \
    src/SolAR3DTransform.cpp \
    src/SolARHomographyValidation.cpp \
    src/SolARSBPatternReIndexer.cpp \
    src/SolARKeypointsReIndexer.cpp \
    src/SolARBasicMatchesFilter.cpp \
    src/SolARMapManager.cpp \
    src/SolARMapFilter.cpp \
    src/SolARModuleTools.cpp \
    src/SolARKeyframeSelector.cpp \
    src/SolARBasicSink.cpp \
    src/SolARBasicSource.cpp \
    src/SolARPointCloudManager.cpp \
    src/SolARKeyframesManager.cpp \
    src/SolARCovisibilityGraphManager.cpp \
    src/SolARBoostCovisibilityGraph.cpp \
    src/SolARLoopCorrector.cpp \
    src/SolARLoopClosureDetector.cpp \
    src/SolAR3D3DcorrespondencesFinder.cpp \
    src/SolAR3DTransformEstimationSACFrom3D3D.cpp \
    src/SolARFiducialMarkerPoseEstimator.cpp \
    src/SolARSLAMBootstrapper.cpp \
    src/SolARSLAMTracking.cpp \
    src/SolARSLAMMapping.cpp \
    src/SolAROverlapDetector.cpp \
    src/SolARMapUpdate.cpp \
    src/SolARStereoDepthEstimation.cpp \
    src/SolARStereoReprojection.cpp \
    src/SolARStereoFeatureExtractionAndDepthEstimation.cpp \
    src/SolARStereoBootstrapper.cpp \
    src/SolARWorldGraphLoader.cpp
