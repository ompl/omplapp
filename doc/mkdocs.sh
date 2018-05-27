#!/bin/sh
rm -rf html
doxygen Doxyfile
cp -r ../ompl/doc/ompl.css ../ompl/doc/images ../ompl/doc/ieee-ram-2012-ompl.pdf images ../install-ompl-ubuntu.sh html
