TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp \
    busmanager.cpp \
    robot.cpp \
    libmodbus/modbus.c \
    libmodbus/modbus-rtu.c \
    libmodbus/modbus-data.c \
    ottoutils.cpp \
    ottomator.cpp

include(deployment.pri)
qtcAddDeployment()

HEADERS += \
    busmanager.h \
    robot.h \
    libmodbus/modbus.h \
    ottoutils.h \
    ottomator.h

INCLUDEPATH += libmodbus
