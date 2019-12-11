#indicates main Qt libraries to include
QT += core gui widgets

#indicates Qt warnings for deprecated functions should be shown
DEFINES += QT_DEPRECATED_WARNINGS

#C++14 is required for some of our the base unit features, used here for consistency
CONFIG += c++14

#list of source files
SOURCES += \
        main.cpp \
        mainWindow.cpp \
        rosNodeWidget.cpp \
        viewerWidget.cpp

#list of header files
HEADERS += \
        mainWindow.hpp \
        rosNodeWidget.hpp \
        viewerWidget.hpp

#list of form files
FORMS += \
        mainWindow.ui

#librealsense2, Intel Camera API
#used to collect and process camera data
LIBS += -L/usr/lib/x86_64-linux-gnu/ -lrealsense2

#lgflw, the GLFW library
#used for camera preview
LIBS += -L/usr/lib/x86_64-linux-gnu/ -lglfw

#ROS libraries
#used to transmit the data from remote to base
QMAKE_RPATHDIR += /opt/ros/kinetic/lib
INCLUDEPATH += /opt/ros/kinetic/include
DEPENDPATH += /opt/ros/kinetic/include
LIBS += -L/opt/ros/kinetic/lib/ -lroscpp -lroscpp_serialization -lrosconsole -lrostime -lcv_bridge -limage_transport

#ROS OpenCV libraries
#used to convert data prior to transmission
INCLUDEPATH += /opt/ros/kinetic/lib/x86_64-linux-gnu
DEPENDPATH += /opt/ros/kinetic/lib/x86_64-linux-gnu
INCLUDEPATH += /opt/ros/kinetic/include/opencv-3.3.1-dev
DEPENDPATH += /opt/ros/kinetic/include/opencv-3.3.1-dev
LIBS += -L/opt/ros/kinetic/lib/x86_64-linux-gnu/ -lopencv_core3 -lopencv_imgcodecs3 -lopencv_imgproc3
