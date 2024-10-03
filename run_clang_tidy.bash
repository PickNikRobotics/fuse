#!/bin/bash

if [ "$PRE_COMMIT_CLANG_TIDY" != "1" ]
then
    if [ "$PRE_COMMIT_CLANG_TIDY" != "0" ]
    then
        >&2 echo "Skipped clang tidy (it is a slow check). To run clang tidy, use 'export PRE_COMMIT_CLANG_TIDY=1' and then run again."
        >&2 echo "To skip clang tidy and make this check pass, 'export PRE_COMMIT_CLANG_TIDY=0' and then run again."
        exit 1
    fi
    # the user intends to skip this check
    exit 0
fi

# -j $(nproc --all) runs with all cores, but the prepended nice runs with a low priority so it won't make your computer unusable while clang tidy is going. The "$@" at the end passes all the filenames from pre-commit so it should only look for clang tidy fixes in the files you directly changed in the commit that is being checked.
nice run-clang-tidy -p ../../build_dbg -j $(nproc --all) -quiet -fix "$@"
