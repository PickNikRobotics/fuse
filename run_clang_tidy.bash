#!/bin/bash

# -j $(nproc --all) runs with all cores, but the prepended nice runs with a low priority so it won't make your computer unusable while clang tidy is going. The "$@" at the end passes all the filenames from pre-commit so it should only look for clang tidy fixes in the files you directly changed in the commit that is being checked.
nice run-clang-tidy -p ../../build_dbg -j $(nproc --all) -quiet -fix "$@"
