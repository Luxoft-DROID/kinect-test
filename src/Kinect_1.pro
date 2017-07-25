QT += core
QT -= gui

CONFIG += c++11

TARGET = openkinect
CONFIG += console
CONFIG -= app_bundle


INCLUDEPATH += /usr/local/include/libfreenect /usr/include/libusb-1.0
LIBS += -L/usr/local/lib `pkg-config --cflags libfreenect` -lfreenect -lglut -lGL -lGLU -lpthread -lX11 -lXxf86vm -lXrandr -lXinerama -lXcursor -lrt -lm -ldl

TEMPLATE = app

SOURCES += main.cpp \
    kinectdevice.cpp

HEADERS += \
    kinectdevice.h
