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
        PC.jpg \
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

unix:!macx: LIBS += -L$$PWD/../../ORB_SLAM2/lib/ -lORB_SLAM2

INCLUDEPATH += $$PWD/../../ORB_SLAM2/include
DEPENDPATH += $$PWD/../../ORB_SLAM2/include
INCLUDEPATH += $$PWD/../../ORB_SLAM2/
DEPENDPATH += $$PWD/../../ORB_SLAM2/
INCLUDEPATH += /opt/ros/kinetic/include/opencv-3.3.1-dev/
DEPENDPATH += /opt/ros/kinetic/include/opencv-3.3.1-dev/

unix:!macx: LIBS += -L$$PWD/../../Pangolin/build/src/ -lpangolin

INCLUDEPATH += $$PWD/../../Pangolin/build/src/
DEPENDPATH += $$PWD/../../Pangolin/build/src/
INCLUDEPATH += /home/jordan/Pangolin/include/
DEPENDPATH += /home/jordan/Pangolin/include/
INCLUDEPATH += /home/jordan/Pangolin/build/src/include/
DEPENDPATH += /home/jordan/Pangolin/build/src/include/
INCLUDEPATH += /home/jordan/eigen-eigen-323c052e1731/
DEPENDPATH += /home/jordan/eigen-eigen-323c052e1731/
LIBS += -L/usr/lib/x86_64-linux-gnu/ -lrealsense2 -lglfw -lGL -lGLU -lGLEW
