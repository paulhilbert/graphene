QMAKE_CXXFLAGS += -std=c++11 -g -Wall -fPIC -O2 -shared
TEMPLATE = lib
TARGET = ../backendQt5
QT += opengl widgets
DEPENDPATH += . \
              GUI \
				  GUI/Property \
				  GUI/Mode \
				  ../../ \
				  $(COMMONS) \
				  /usr/include/eigen3
INCLUDEPATH += . \
              GUI \
				  GUI/Property \
				  GUI/Mode \
				  ../../ \
				  $(COMMONS) \
				  /usr/include/eigen3 \
				  /usr/include/qt
CONFIG += object_parallel_to_source
CONFIG += dll

LIBS += -L../../ -lgraphene -ldl -lboost_filesystem -lboost_program_options -lboost_system -lGL -lGLU -lGLEW

HEADERS += GUI/*.h GUI/Property/*.h GUI/Mode/*.h
SOURCES += backendQt5.cpp \
           GUI/*.cpp \
           GUI/Property/*.cpp \
           GUI/Mode/*.cpp
