## remove Qt dependencies
QT       -= core gui
CONFIG -= qt

## global defintions : target lib name, version
TARGET = SolARModuleTools
INSTALLSUBDIR = bcomBuild
FRAMEWORK = $$TARGET
VERSION=0.4.0

DEFINES += MYVERSION=$${VERSION}
DEFINES += TEMPLATE_LIBRARY
CONFIG += Cpp11
CONFIG += c++11


CONFIG(debug,debug|release) {
    DEFINES += _DEBUG=1
    DEFINES += DEBUG=1
}

CONFIG(release,debug|release) {
    DEFINES += _NDEBUG=1
    DEFINES += NDEBUG=1
}

PROJECTDEPLOYDIR = $$(BCOMDEVROOT)/$${INSTALLSUBDIR}/$${FRAMEWORK}/$${VERSION}
DEPENDENCIESCONFIG = shared

include ($$(BCOMDEVROOT)/builddefs/qmake/templatelibconfig.pri)

## DEFINES FOR MSVC/INTEL C++ compilers
msvc {
DEFINES += "_BCOM_SHARED=__declspec(dllexport)"
}

INCLUDEPATH += interfaces/

HEADERS += interfaces/SolARImage2WorldMapper4Marker2D.h \
interfaces/SolAR2DTransform.h \
interfaces/SolAR3DTransform.h \
interfaces/SolARHomographyValidation.h \
interfaces/SolARSBPatternReIndexer.h \
interfaces/SolARKeypointsReIndexer.h \
interfaces/SolARModuleManagerTools.h \
interfaces/SolARPoseHelper.h \
interfaces/SolARToolsAPI.h \
interfaces/SolARBasicMatchesFilter.h


SOURCES += src/SolARImage2WorldMapper4Marker2D.cpp \
    src/SolAR2DTransform.cpp \
    src/SolAR3DTransform.cpp \
    src/SolARHomographyValidation.cpp \
    src/SolARSBPatternReIndexer.cpp \
    src/SolARKeypointsReIndexer.cpp \
    src/SolARBasicMatchesFilter.cpp \
    src/SolARModuleTools.cpp \
	src/SolARPoseHelper.cpp \
    src/SolARModuleManagerTools.cpp

unix {
}

macx {
    DEFINES += _MACOS_TARGET_
    QMAKE_MAC_SDK= macosx
    QMAKE_CFLAGS += -mmacosx-version-min=10.7 -std=c11 #-x objective-c++
    QMAKE_CXXFLAGS += -mmacosx-version-min=10.7 -std=c11 -std=c++11 -O3 -fPIC#-x objective-c++
    QMAKE_LFLAGS += -mmacosx-version-min=10.7 -v -lstdc++
    LIBS += -lstdc++ -lc -lpthread
}

win32 {

    DEFINES += WIN64 UNICODE _UNICODE
    QMAKE_COMPILER_DEFINES += _WIN64
    QMAKE_CXXFLAGS += -wd4250 -wd4251 -wd4244 -wd4275
}

header_files.path = $${PROJECTDEPLOYDIR}/interfaces
header_files.files = $$files($${PWD}/interfaces/*.h*)

xpcf_xml_files.path = $${PROJECTDEPLOYDIR}
xpcf_xml_files.files=$$files($${PWD}/xpcf*.xml)

INSTALLS += header_files
INSTALLS += xpcf_xml_files
