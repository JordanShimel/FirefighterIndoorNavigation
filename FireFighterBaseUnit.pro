#indicates main Qt libraries to include
QT += core gui widgets

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

#add additional library includes here
