#indicates main Qt libraries to include
QT += core gui widgets

#indicates Qt warnings for deprecated functions should be shown
DEFINES += QT_DEPRECATED_WARNINGS

#C++14 is required for some of our features(pcl)
CONFIG += c++14

#list of source files
SOURCES += \
        main.cpp \
        mainWindow.cpp \
        pointCloudWidget.cpp \
        rosNodeWidget.cpp

#list of header files
HEADERS += \
        mainWindow.hpp \
        pointCloudWidget.hpp \
        rosNodeWidget.hpp

#list of form files
FORMS += \
        mainWindow.ui

#list of other files
DISTFILES += \
        $$PWD/ORB_SLAM2_firefighter/Vocabulary/ORBvoc.txt \
        camera.yaml

#these commands cause qmake to copy the camera settings and vocabulary files to the build directory
copyCameraSettings.commands = $(COPY_DIR) $$PWD/camera.yaml $$OUT_PWD
copyVocabulary.commands = $(COPY_DIR) $$PWD/ORB_SLAM2_firefighter/Vocabulary/ORBvoc.txt $$OUT_PWD
first.depends = $(first) copyCameraSettings copyVocabulary
export(first.depends)
QMAKE_EXTRA_TARGETS += first copyCameraSettings copyVocabulary

#Extra includes
INCLUDEPATH += /usr/local/include
DEPENDPATH += /usr/local/include
INCLUDEPATH += /usr/lib/x86_64-linux-gnu
DEPENDPATH += /usr/lib/x86_64-linux-gnu

#Custom ORB_SLAM2
#used to do point cloud construction and manipulation
INCLUDEPATH += $$PWD/ORB_SLAM2_firefighter/include
DEPENDPATH += $$PWD/ORB_SLAM2_firefighter/include
INCLUDEPATH += $$PWD/ORB_SLAM2_firefighter
DEPENDPATH += $$PWD/ORB_SLAM2_firefighter
LIBS += -L$$PWD/ORB_SLAM2_firefighter/lib -lORB_SLAM2

#ROS libraries
#used to receive data sent from remote to base
QMAKE_RPATHDIR += /opt/ros/kinetic/lib
INCLUDEPATH += /opt/ros/kinetic/include
DEPENDPATH += /opt/ros/kinetic/include
LIBS += -L/opt/ros/kinetic/lib/ -lroscpp -limage_transport -lrosconsole -lrostime -lcv_bridge -lroscpp_serialization -lmessage_filters

#ROS OpenCV libraries
#used to convert data before point cloud processing
INCLUDEPATH += /opt/ros/kinetic/lib/x86_64-linux-gnu
DEPENDPATH += /opt/ros/kinetic/lib/x86_64-linux-gnu
INCLUDEPATH += /opt/ros/kinetic/include/opencv-3.3.1-dev
DEPENDPATH += /opt/ros/kinetic/include/opencv-3.3.1-dev
LIBS += -L/opt/ros/kinetic/lib/x86_64-linux-gnu/ -lopencv_core3 -lopencv_highgui3 -lopencv_imgcodecs3 -lopencv_imgproc3 -lopencv_features2d3 -lopencv_calib3d3

#Pangolin libraries
#used by ORB_SLAM2 to show sparse point clouds
PANGOLIN_PATH = /home/jordan/Libraries/Pangolin/build/src
INCLUDEPATH += $${PANGOLIN_PATH}/include
DEPENDPATH += $${PANGOLIN_PATH}/include
INCLUDEPATH += $${PANGOLIN_PATH}/../../include
DEPENDPATH += $${PANGOLIN_PATH}/../../include
LIBS += -L$${PANGOLIN_PATH}/ -lpangolin

#GLEW libraries
#used by ORB_SLAM2 to display current frame
LIBS += -L/usr/lib/x86_64-linux-gnu/ -lGLEW

#Boost libraries
#used by ORB_SLAM2 for threading system
LIBS += -L/usr/lib/x86_64-linux-gnu/ -lboost_system

#LXDO libraries
#used to capture ORB_SLAM2 windows for integration into main ui
INCLUDEPATH += /usr/lib/x86_64-linux-gnu
DEPENDPATH += /usr/lib/x86_64-linux-gnu
LIBS += -L/usr/lib/x86_64-linux-gnu/ -lxdo

#Eigen libraries
#used by ORB_SLAM2 for matrix operations
INCLUDEPATH += /usr/include/eigen3
DEPENDPATH += /usr/include/eigen3

#PCL libraries
#used by ORB_SLAM2 for dense point cloud construction and display
INCLUDEPATH += /usr/local/include/pcl-1.9
DEPENDPATH += /usr/local/include/pcl-1.9
LIBS += -L/usr/lib/ -lpcl_common -lpcl_visualization -lpcl_octree -lpcl_filters
