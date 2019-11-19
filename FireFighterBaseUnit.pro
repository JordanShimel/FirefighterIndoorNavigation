#indicates main Qt libraries to include
QT += core gui widgets webenginewidgets

#indicates Qt warnings for deprecated functions should be shown
DEFINES += QT_DEPRECATED_WARNINGS

#C++11 is required for some of our features
CONFIG += c++11

#list of source files
SOURCES += \
        ORB_SLAM2_firefighter/Thirdparty/DBoW2/DBoW2/BowVector.cpp \
        ORB_SLAM2_firefighter/Thirdparty/DBoW2/DBoW2/FORB.cpp \
        ORB_SLAM2_firefighter/Thirdparty/DBoW2/DBoW2/FeatureVector.cpp \
        ORB_SLAM2_firefighter/Thirdparty/DBoW2/DBoW2/ScoringObject.cpp \
        ORB_SLAM2_firefighter/Thirdparty/DBoW2/DUtils/Random.cpp \
        ORB_SLAM2_firefighter/Thirdparty/DBoW2/DUtils/Timestamp.cpp \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/core/batch_stats.cpp \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/core/cache.cpp \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/core/estimate_propagator.cpp \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/core/factory.cpp \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/core/hyper_dijkstra.cpp \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/core/hyper_graph.cpp \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/core/hyper_graph_action.cpp \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/core/jacobian_workspace.cpp \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/core/marginal_covariance_cholesky.cpp \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/core/matrix_structure.cpp \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/core/optimizable_graph.cpp \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/core/optimization_algorithm.cpp \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/core/optimization_algorithm_dogleg.cpp \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/core/optimization_algorithm_factory.cpp \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/core/optimization_algorithm_gauss_newton.cpp \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.cpp \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/core/optimization_algorithm_with_hessian.cpp \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/core/parameter.cpp \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/core/parameter_container.cpp \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/core/robust_kernel.cpp \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/core/robust_kernel_factory.cpp \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/core/robust_kernel_impl.cpp \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/core/solver.cpp \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/core/sparse_optimizer.cpp \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/stuff/os_specific.c \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/stuff/property.cpp \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/stuff/string_tools.cpp \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/stuff/timeutil.cpp \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/types/types_sba.cpp \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/types/types_seven_dof_expmap.cpp \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/types/types_six_dof_expmap.cpp \
        ORB_SLAM2_firefighter/src/Converter.cc \
        ORB_SLAM2_firefighter/src/Frame.cc \
        ORB_SLAM2_firefighter/src/FrameDrawer.cc \
        ORB_SLAM2_firefighter/src/Initializer.cc \
        ORB_SLAM2_firefighter/src/KeyFrame.cc \
        ORB_SLAM2_firefighter/src/KeyFrameDatabase.cc \
        ORB_SLAM2_firefighter/src/LocalMapping.cc \
        ORB_SLAM2_firefighter/src/LoopClosing.cc \
        ORB_SLAM2_firefighter/src/Map.cc \
        ORB_SLAM2_firefighter/src/MapDrawer.cc \
        ORB_SLAM2_firefighter/src/MapPoint.cc \
        ORB_SLAM2_firefighter/src/ORBextractor.cc \
        ORB_SLAM2_firefighter/src/ORBmatcher.cc \
        ORB_SLAM2_firefighter/src/Optimizer.cc \
        ORB_SLAM2_firefighter/src/PnPsolver.cc \
        ORB_SLAM2_firefighter/src/pointcloudmapping.cc \
        ORB_SLAM2_firefighter/src/Sim3Solver.cc \
        ORB_SLAM2_firefighter/src/System.cc \
        ORB_SLAM2_firefighter/src/Tracking.cc \
        ORB_SLAM2_firefighter/src/Viewer.cc \
        main.cpp \
        mainWindow.cpp \
        pointcloudWidget.cpp \
        rosNodeWidget.cpp

#list of header files
HEADERS += \
        ORB_SLAM2_firefighter/Thirdparty/DBoW2/DBoW2/BowVector.h \
        ORB_SLAM2_firefighter/Thirdparty/DBoW2/DBoW2/FClass.h \
        ORB_SLAM2_firefighter/Thirdparty/DBoW2/DBoW2/FORB.h \
        ORB_SLAM2_firefighter/Thirdparty/DBoW2/DBoW2/FeatureVector.h \
        ORB_SLAM2_firefighter/Thirdparty/DBoW2/DBoW2/ScoringObject.h \
        ORB_SLAM2_firefighter/Thirdparty/DBoW2/DBoW2/TemplatedVocabulary.h \
        ORB_SLAM2_firefighter/Thirdparty/DBoW2/DUtils/Random.h \
        ORB_SLAM2_firefighter/Thirdparty/DBoW2/DUtils/Timestamp.h \
        ORB_SLAM2_firefighter/Thirdparty/g2o/config.h \
        ORB_SLAM2_firefighter/Thirdparty/g2o/config.h.in \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/core/base_binary_edge.h \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/core/base_binary_edge.hpp \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/core/base_edge.h \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/core/base_multi_edge.h \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/core/base_multi_edge.hpp \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/core/base_unary_edge.h \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/core/base_unary_edge.hpp \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/core/base_vertex.h \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/core/base_vertex.hpp \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/core/batch_stats.h \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/core/block_solver.h \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/core/block_solver.hpp \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/core/cache.h \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/core/creators.h \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/core/eigen_types.h \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/core/estimate_propagator.h \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/core/factory.h \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/core/hyper_dijkstra.h \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/core/hyper_graph.h \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/core/hyper_graph_action.h \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/core/jacobian_workspace.h \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/core/linear_solver.h \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/core/marginal_covariance_cholesky.h \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/core/matrix_operations.h \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/core/matrix_structure.h \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/core/openmp_mutex.h \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/core/optimizable_graph.h \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/core/optimization_algorithm.h \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/core/optimization_algorithm_dogleg.h \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/core/optimization_algorithm_factory.h \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/core/optimization_algorithm_gauss_newton.h \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/core/optimization_algorithm_property.h \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/core/optimization_algorithm_with_hessian.h \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/core/parameter.h \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/core/parameter_container.h \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/core/robust_kernel.h \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/core/robust_kernel_factory.h \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/core/robust_kernel_impl.h \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/core/solver.h \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/core/sparse_block_matrix.h \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/core/sparse_block_matrix.hpp \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/core/sparse_block_matrix_ccs.h \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/core/sparse_block_matrix_diagonal.h \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/core/sparse_optimizer.h \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/solvers/linear_solver_dense.h \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/stuff/color_macros.h \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/stuff/macros.h \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/stuff/misc.h \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/stuff/os_specific.h \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/stuff/property.h \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/stuff/string_tools.h \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/stuff/timeutil.h \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/types/se3_ops.h \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/types/se3_ops.hpp \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/types/se3quat.h \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/types/sim3.h \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/types/types_sba.h \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h \
        ORB_SLAM2_firefighter/Thirdparty/g2o/g2o/types/types_six_dof_expmap.h \
        ORB_SLAM2_firefighter/include/Converter.h \
        ORB_SLAM2_firefighter/include/Frame.h \
        ORB_SLAM2_firefighter/include/FrameDrawer.h \
        ORB_SLAM2_firefighter/include/Initializer.h \
        ORB_SLAM2_firefighter/include/KeyFrame.h \
        ORB_SLAM2_firefighter/include/KeyFrameDatabase.h \
        ORB_SLAM2_firefighter/include/LocalMapping.h \
        ORB_SLAM2_firefighter/include/LoopClosing.h \
        ORB_SLAM2_firefighter/include/Map.h \
        ORB_SLAM2_firefighter/include/MapDrawer.h \
        ORB_SLAM2_firefighter/include/MapPoint.h \
        ORB_SLAM2_firefighter/include/ORBVocabulary.h \
        ORB_SLAM2_firefighter/include/ORBextractor.h \
        ORB_SLAM2_firefighter/include/ORBmatcher.h \
        ORB_SLAM2_firefighter/include/Optimizer.h \
        ORB_SLAM2_firefighter/include/PnPsolver.h \
        ORB_SLAM2_firefighter/include/pointcloudmapping.h \
        ORB_SLAM2_firefighter/include/Sim3Solver.h \
        ORB_SLAM2_firefighter/include/System.h \
        ORB_SLAM2_firefighter/include/Tracking.h \
        ORB_SLAM2_firefighter/include/Viewer.h \
        mainWindow.hpp \
        pointcloudWidget.hpp \
        rosNodeWidget.hpp

#list of form files
FORMS += \
        mainWindow.ui

#list of model files
DISTFILES += \
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
INCLUDEPATH += $$PWD/ORB_SLAM2_firefighter
DEPENDPATH += $$PWD/ORB_SLAM2_firefighter
INCLUDEPATH += $$PWD/ORB_SLAM2_firefighter/include
DEPENDPATH += $$PWD/ORB_SLAM2_firefighter/include
INCLUDEPATH += /usr/local/include
DEPENDPATH += /usr/local/include
INCLUDEPATH += /usr/lib/x86_64-linux-gnu
DEPENDPATH += /usr/lib/x86_64-linux-gnu

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
#Custom library path, the place you installed Pangolin
USER_LIBRARY_PATH = /home/jordan/Libraries
INCLUDEPATH += $${USER_LIBRARY_PATH}/Pangolin/include
DEPENDPATH += $${USER_LIBRARY_PATH}/Pangolin/include
INCLUDEPATH += $${USER_LIBRARY_PATH}/Pangolin/build/src/include
DEPENDPATH += $${USER_LIBRARY_PATH}/Pangolin/build/src/include
LIBS += -L$${USER_LIBRARY_PATH}/Pangolin/build/src/ -lpangolin

#GLEW libraries
LIBS += -L/usr/lib/x86_64-linux-gnu/ -lglfw -lGL -lGLU -lGLEW

#Boost libraries
LIBS += -L/usr/lib/x86_64-linux-gnu/ -lboost_system

#Eigen
INCLUDEPATH += /usr/include/eigen3
DEPENDPATH += /usr/include/eigen3

#PCL libraries
INCLUDEPATH += /usr/local/include/pcl-1.9
DEPENDPATH += /usr/local/include/pcl-1.9
LIBS += -L/usr/local/lib/ -lpcl_common -lpcl_visualization -lpcl_octree -lpcl_filters

#VTK libraries
INCLUDEPATH += /usr/include/vtk-6.2
DEPENDPATH += /usr/include/vtk-6.2
