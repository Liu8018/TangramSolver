TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp \
    TangramSolver.cpp \
    PolygonPattern.cpp

HEADERS += \
    TangramSolver.h \
    PolygonPattern.h

INCLUDEPATH += /usr/local/include

LIBS += /usr/local/lib/libopencv_*.so
