#!/bin/bash
# Generate PDF documentation from Markdown
pandoc README.md -o pipeline_documentation.pdf --pdf-engine=xelatex \
    -V geometry:margin=1in \
    -V linkcolor:blue \
    -V fontsize=12pt \
    --toc
