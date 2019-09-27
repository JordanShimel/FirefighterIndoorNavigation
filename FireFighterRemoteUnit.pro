QT += core gui widgets
CONFIG += c++11

# The following define makes your compiler emit warnings if you use
# any Qt feature that has been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    main.cpp \
    mainwindow.cpp

# example and stb_easy_font are temporary and will be replaced with Qt based functionality
HEADERS += \
    example.hpp \
    mainwindow.h \
    stb_easy_font.h

FORMS += \
    mainwindow.ui

#Extra includes and libraries
INCLUDEPATH += /usr/lib/x86_64-linux-gnu
DEPENDPATH += /usr/lib/x86_64-linux-gnu
#librealsense2, Intel Camera API
unix: LIBS += -L$$PWD/../../../../usr/lib/x86_64-linux-gnu/ -lrealsense2
#lglu, used by example.hpp, may be removable later
unix: LIBS += -L$$PWD/../../../../usr/lib/x86_64-linux-gnu/ -lGLU
#lgflw, used by example.hpp, may be removable later
unix: LIBS += -L$$PWD/../../../../usr/lib/x86_64-linux-gnu/ -lglfw
#lgl, used by example.hpp, may be removable later
unix: LIBS += -L$$PWD/../../../../usr/lib/x86_64-linux-gnu/ -lGL

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
