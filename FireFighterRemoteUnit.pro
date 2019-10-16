QT += core gui widgets
CONFIG += c++11

DEFINES += QT_DEPRECATED_WARNINGS

SOURCES += \
    main.cpp \
    mainWindow.cpp \
    rosNodeWidget.cpp \
    viewerWidget.cpp

HEADERS += \
    mainWindow.hpp \
    rosNodeWidget.hpp \
    viewerWidget.hpp

FORMS += \
    mainWindow.ui

#librealsense2, Intel Camera API
#used to collect and process camera data
LIBS += -L/usr/lib/x86_64-linux-gnu/ -lrealsense2

#lgflw, the GLFW library
#used for camera preview
LIBS += -L/usr/lib/x86_64-linux-gnu/ -lglfw

#ROS libraries
QMAKE_RPATHDIR += /opt/ros/kinetic/lib
INCLUDEPATH += /opt/ros/kinetic/include
DEPENDPATH += /opt/ros/kinetic/include
LIBS += -L/opt/ros/kinetic/lib/ -lroscpp -lroscpp_serialization -lrosconsole -lrostime -lcv_bridge -limage_transport

#ROS OpenCV libraries
INCLUDEPATH += /opt/ros/kinetic/lib/x86_64-linux-gnu
DEPENDPATH += /opt/ros/kinetic/lib/x86_64-linux-gnu
LIBS += -L/opt/ros/kinetic/lib/x86_64-linux-gnu/ -lopencv_core3 -lopencv_imgcodecs3 -lopencv_imgproc3

#OpenCV libraries
INCLUDEPATH += /usr/local/include/opencv4/
DEPENDPATH += /usr/local/include/
LIBS += -L/usr/local/lib -lopencv_core

DISTFILES += \
    firefighterremoteunit.qmodel
