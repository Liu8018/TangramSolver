TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp \
    TangramSolver.cpp \
    PolygonPattern.cpp \
    functions.cpp

HEADERS += \
    TangramSolver.h \
    PolygonPattern.h \
    functions.h

INCLUDEPATH += /usr/local/include

LIBS += /usr/local/lib/libopencv_highgui.so \
        /usr/local/lib/libopencv_imgproc.so \
        /usr/local/lib/libopencv_core.so \
        /usr/local/lib/libopencv_imgcodecs.so
