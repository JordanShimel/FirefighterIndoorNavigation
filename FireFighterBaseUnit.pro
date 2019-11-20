#indicates main Qt libraries to include
#note: webenginewidgets is not available by default on manual Qt installs
#make sure to use a Qt version from the installer and select web engine optional components
QT += core gui widgets webenginewidgets

#indicates Qt warnings for deprecated functions should be shown
DEFINES += QT_DEPRECATED_WARNINGS

#C++14 is required for some of our features(pcl)
CONFIG += c++14

#list of source files
SOURCES += \
        main.cpp \
        mainWindow.cpp \
        pointcloudWidget.cpp \
        rosNodeWidget.cpp

#list of header files
HEADERS += \
        mainWindow.hpp \
        pointcloudWidget.hpp \
        rosNodeWidget.hpp

#list of form files
FORMS += \
        mainWindow.ui

#list of other files
DISTFILES += \
        $$PWD/ORB_SLAM2_firefighter/Vocabulary/ORBvoc.txt \
        camera.yaml \
        firefighterbaseunit.qmodel

#These commands cause qmake to copy the camera settings and vocabulary files to the build directory
copyCameraSettings.commands = $(COPY_DIR) $$PWD/camera.yaml $$OUT_PWD
copyVocabulary.commands = $(COPY_DIR) $$PWD/ORB_SLAM2_firefighter/Vocabulary/ORBvoc.txt $$OUT_PWD
first.depends = $(first) copyCameraSettings copyVocabulary
export(first.depends)
export(copyCameraSettings.commands)
QMAKE_EXTRA_TARGETS += first copyCameraSettings copyVocabulary

#Extra includes
INCLUDEPATH += /usr/local/include
DEPENDPATH += /usr/local/include
INCLUDEPATH += /usr/lib/x86_64-linux-gnu
DEPENDPATH += /usr/lib/x86_64-linux-gnu

#Custom ORB_SLAM2
INCLUDEPATH += $$PWD/ORB_SLAM2_firefighter/include
DEPENDPATH += $$PWD/ORB_SLAM2_firefighter/include
INCLUDEPATH += $$PWD/ORB_SLAM2_firefighter
DEPENDPATH += $$PWD/ORB_SLAM2_firefighter
LIBS += -L$$PWD/ORB_SLAM2_firefighter/lib -lORB_SLAM2

#ROS libraries
QMAKE_RPATHDIR += /opt/ros/kinetic/lib
INCLUDEPATH += /opt/ros/kinetic/include
DEPENDPATH += /opt/ros/kinetic/include
LIBS += -L/opt/ros/kinetic/lib/ -lroscpp -limage_transport -lrosconsole -lrostime -lcv_bridge -lroscpp_serialization -lmessage_filters

#ROS OpenCV libraries
INCLUDEPATH += /opt/ros/kinetic/lib/x86_64-linux-gnu
DEPENDPATH += /opt/ros/kinetic/lib/x86_64-linux-gnu
INCLUDEPATH += /opt/ros/kinetic/include/opencv-3.3.1-dev
DEPENDPATH += /opt/ros/kinetic/include/opencv-3.3.1-dev
LIBS += -L/opt/ros/kinetic/lib/x86_64-linux-gnu/ -lopencv_core3 -lopencv_highgui3 -lopencv_imgcodecs3 -lopencv_imgproc3 -lopencv_features2d3 -lopencv_calib3d3

#Pangolin libraries
PANGOLIN_PATH = /home/jordan/Libraries/Pangolin
INCLUDEPATH += $${PANGOLIN_PATH}/include
DEPENDPATH += $${PANGOLIN_PATH}/include
INCLUDEPATH += $${PANGOLIN_PATH}/build/src/include
DEPENDPATH += $${PANGOLIN_PATH}/build/src/include
LIBS += -L$${PANGOLIN_PATH}/build/src/ -lpangolin

#GLEW libraries
LIBS += -L/usr/lib/x86_64-linux-gnu/ -lGLEW

#Boost libraries
LIBS += -L/usr/lib/x86_64-linux-gnu/ -lboost_system

#LXDO libraries
INCLUDEPATH += /usr/lib/x86_64-linux-gnu
DEPENDPATH += /usr/lib/x86_64-linux-gnu
LIBS += -L/usr/lib/x86_64-linux-gnu/ -lxdo

#Eigen
INCLUDEPATH += /usr/include/eigen3
DEPENDPATH += /usr/include/eigen3

#PCL libraries
INCLUDEPATH += /usr/include/pcl-1.9
DEPENDPATH += /usr/include/pcl-1.9
LIBS += -L/usr/lib/ -lpcl_common -lpcl_visualization -lpcl_octree -lpcl_filters

#VTK libraries
INCLUDEPATH += /usr/include/vtk-6.2
DEPENDPATH += /usr/include/vtk-6.2
