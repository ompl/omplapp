#!/bin/sh
cd latex && make primer clean && cp OMPL_Primer.pdf ../html
cd ..
tar cf - -s/html// html | tar xf - -C ${HOME}/src/ompl.github.io
