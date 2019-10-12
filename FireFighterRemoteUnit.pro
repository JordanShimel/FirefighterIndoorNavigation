QT += core gui widgets
CONFIG += c++11

DEFINES += QT_DEPRECATED_WARNINGS

SOURCES += \
    main.cpp \
    mainwindow.cpp \
    viewer.cpp

HEADERS += \
    mainwindow.h \
    viewer.hpp

FORMS += \
    mainwindow.ui

#extra include and library paths
INCLUDEPATH += /usr/lib/x86_64-linux-gnu
DEPENDPATH += /usr/lib/x86_64-linux-gnu
#librealsense2, Intel Camera API
#used to collect and process camera data
unix: LIBS += -L$$PWD/../../../../usr/lib/x86_64-linux-gnu/ -lrealsense2
#lgflw, the GLFW library
#used for camera preview
unix: LIBS += -L$$PWD/../../../../usr/lib/x86_64-linux-gnu/ -lglfw
