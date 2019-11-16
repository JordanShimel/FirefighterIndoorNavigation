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
        rosNodeWidget.cpp

#list of header files
HEADERS += \
        mainWindow.hpp \
        pointcloudWidget.hpp \
        rosNodeWidget.hpp

#list of form files
FORMS += \
        mainWindow.ui

#list of model files
DISTFILES += \
        firefighterbaseunit.qmodel

#These commands cause qmake to copy the camera settings and vocabulary files to the build directory
copyCameraSettings.commands = $(COPY_DIR) $$PWD/camera.yaml $$OUT_PWD
copyVocabulary.commands = $(COPY_DIR) $$PWD/ORBvoc.txt $$OUT_PWD
first.depends = $(first) copyCameraSettings copyVocabulary
export(first.depends)
export(copyCameraSettings.commands)
QMAKE_EXTRA_TARGETS += first copyCameraSettings copyVocabulary

#Custom library path, the place you installed Eigen, ORB_SLAM2, and Pangolin
#Set this to yours to make qmake function
USER_LIBRARY_PATH = /home/jordan/Libraries

#Extra includes
INCLUDEPATH += $${USER_LIBRARY_PATH}
DEPENDPATH += $${USER_LIBRARY_PATH}
INCLUDEPATH += /usr/lib/x86_64-linux-gnu
DEPENDPATH += /usr/lib/x86_64-linux-gnu

#ROS libraries
QMAKE_RPATHDIR += /opt/ros/kinetic/lib
INCLUDEPATH += /opt/ros/kinetic/include
DEPENDPATH += /opt/ros/kinetic/include
LIBS += -L/opt/ros/kinetic/lib/ -lroscpp -lroscpp_serialization -lrosconsole -lrostime -lcv_bridge -limage_transport -lmessage_filters

#ROS OpenCV libraries
INCLUDEPATH += /opt/ros/kinetic/lib/x86_64-linux-gnu
DEPENDPATH += /opt/ros/kinetic/lib/x86_64-linux-gnu
INCLUDEPATH += /opt/ros/kinetic/include/opencv-3.3.1-dev/
DEPENDPATH += /opt/ros/kinetic/include/opencv-3.3.1-dev/
LIBS += -L/opt/ros/kinetic/lib/x86_64-linux-gnu/ -lopencv_core3 -lopencv_imgcodecs3 -lopencv_imgproc3

#OpenCV libraries
INCLUDEPATH += /usr/local/include/opencv4/
DEPENDPATH += /usr/local/include/
LIBS += -L/usr/local/lib -lopencv_core

#ORB_SLAM2 libraries
INCLUDEPATH += $${USER_LIBRARY_PATH}/ORB_SLAM2/include/
DEPENDPATH += $${USER_LIBRARY_PATH}/ORB_SLAM2/include/
INCLUDEPATH += $${USER_LIBRARY_PATH}/ORB_SLAM2/
DEPENDPATH += $${USER_LIBRARY_PATH}/ORB_SLAM2/
LIBS += -L$${USER_LIBRARY_PATH}/ORB_SLAM2/lib/ -lORB_SLAM2
LIBS += -L$${USER_LIBRARY_PATH}/ORB_SLAM2/Thirdparty/DBoW2/lib/ -lDBoW2

#Pangolin libraries
INCLUDEPATH += $${USER_LIBRARY_PATH}/Pangolin/build/src/
DEPENDPATH += $${USER_LIBRARY_PATH}/Pangolin/build/src/
INCLUDEPATH += $${USER_LIBRARY_PATH}/Pangolin/include/
DEPENDPATH += $${USER_LIBRARY_PATH}/Pangolin/include/
INCLUDEPATH += $${USER_LIBRARY_PATH}/Pangolin/build/src/include/
DEPENDPATH += $${USER_LIBRARY_PATH}/Pangolin/build/src/include/
LIBS += -L$${USER_LIBRARY_PATH}/Pangolin/build/src/ -lpangolin

#GLEW libraries
LIBS += -L/usr/lib/x86_64-linux-gnu/ -lglfw -lGL -lGLU -lGLEW

#Boost libraries
LIBS += -L/usr/lib/x86_64-linux-gnu/ -lboost_system
