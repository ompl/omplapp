#!/bin/sh
rm -rf html

# copy assets from ompl subrepo
for dir in css fonts images js; do
    rsync -av ../ompl/doc/${dir} .
done

doxygen Doxyfile
cd html
for f in md_doc_markdown_*; do mv $f `echo $f | cut -c17-1000`; done
for f in md_ompl_doc_markdown_*; do mv $f `echo $f | cut -c22-1000`; done
rm *8md_source.html
