#indicates main Qt libraries to include
QT += core gui widgets webenginewidgets

#indicates Qt warnings for deprecated functions should be shown
DEFINES += QT_DEPRECATED_WARNINGS

#C++11 is required for some of our features
CONFIG += c++11

#list of source files
SOURCES += \
        main.cpp \
        mainWindow.cpp \
        pointcloudWidget.cpp \
        rosNodeWidget.cpp \
        videoWidget.cpp

#list of header files
HEADERS += \
        mainWindow.hpp \
        pointcloudWidget.hpp \
        rosNodeWidget.hpp \
        videoWidget.hpp

#list of form files
FORMS += \
        mainWindow.ui

#list of model files
DISTFILES += \
        firefighterbaseunit.qmodel

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
