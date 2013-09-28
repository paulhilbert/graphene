#!/bin/sh

scons && cd Backends/Qt5 && qmake graphene_qt5.pro && make && cd ../../ || cd $GRAPHENE_ROOT
