TEMPLATE = lib

CONFIG += release
DEFINES += NDEBUG

INCLUDEPATH += ./
INCLUDEPATH += ./thirdparty/libigl/include
INCLUDEPATH += ./thirdparty/eigen

SOURCES += simpleuv/parametrize.cpp
HEADERS += simpleuv/parametrize.h

SOURCES += simpleuv/uvunwrapper.cpp
HEADERS += simpleuv/uvunwrapper.h

HEADERS += simpleuv/meshdatatype.h

QMAKE_CXXFLAGS += -std=c++11

target.path = ./
INSTALLS += target
