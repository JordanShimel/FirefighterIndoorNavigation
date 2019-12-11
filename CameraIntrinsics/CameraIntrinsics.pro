QT -= gui

CONFIG += c++14 console
CONFIG -= app_bundle

DEFINES += QT_DEPRECATED_WARNINGS

SOURCES += \
        main.cpp

#librealsense2, Intel Camera API
#used to collect and process camera data
LIBS += -L/usr/lib/x86_64-linux-gnu/ -lrealsense2
