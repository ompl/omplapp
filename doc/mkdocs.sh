#!/bin/sh
rm -rf html
doxygen Doxyfile
cp -r ../ompl/doc/css ../ompl/doc/fonts ../ompl/doc/images ../ompl/doc/js ../ompl/doc/ieee-ram-2012-ompl.pdf images html

cd html
for f in md_doc_markdown_*; do mv $f `echo $f | cut -c17-1000`; done
for f in md_ompl_doc_markdown_*; do mv $f `echo $f | cut -c22-1000`; done
rm *8md_source.html

for f in *.html search/*.js; do
    sed -i "" 's/href="md_ompl_doc_markdown_/href="/g;s/href="md_doc_markdown_/href="/g' $f
done
