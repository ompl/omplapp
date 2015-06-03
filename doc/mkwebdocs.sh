#!/bin/sh
tar cf - -s/html// html | tar xf - -C ${HOME}/src/ompl.github.io
