#!/bin/sh

BRANCH=trunk

# web server
export SERVER=arachne.cs.rice.edu
# directory on web server where different versions of the web site
# and other assets are located.
export ASSETS_ROOT=/mnt/data2/ompl/$BRANCH
export ASSET_DIR=omplapp

echo "Exporting OMPL.app documentation for $BRANCH"

rm -rf ${ASSET_DIR}
# copy all assets to the ASSET_DIR directory
mkdir -p ${ASSET_DIR}
for f in html/*.html; do
    sed 's/="..\//=".\//g' $f > ${ASSET_DIR}/`basename $f`
done
cd latex && make primer clean && cp OMPL_Primer.pdf ../${ASSET_DIR}
cd ..
cp -r css js php images html/*.png html/*.map html/search ${ASSET_DIR}

# add symlink to OMPL
cd $ASSET_DIR && ln -s ../ompl core && cd ..

# copy everything to web server and fix permissions
tar cf - ${ASSET_DIR} | ssh ${SERVER} 'mkdir -p '${ASSETS_ROOT}'; cd '${ASSETS_ROOT}'; tar xf -; ln -s /mnt/data2/ompl/wordpress '${ASSET_DIR}'/blog; chmod -R a+rX '${ASSET_DIR}'; chgrp -R ompl '${ASSET_DIR}'; chmod -R g+w '${ASSET_DIR}

# clean up
rm -rf ${ASSET_DIR}

# beta site should not be indexed
if [ $BRANCH == "trunk" ]
then
    (echo "User-agent: *"; echo "Disallow: /") | ssh ${SERVER} 'cat - > '${ASSETS_ROOT}'/'${ASSET_DIR}'/robots.txt'
fi

echo The web site has been copied to:
echo "      " ${SERVER}:${ASSETS_ROOT}/${ASSET_DIR}.
