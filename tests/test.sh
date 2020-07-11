#!/bin/sh

RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m' 

if [ $1 = "run" ]; then
    unitTestPath="unit"
    unitTestCasesPath=$unitTestPath"/tests/"
    unitTestCases="$(ls $unitTestCasesPath)"

    echo "--------- calling make ---------"
	for unitTestCase in $unitTestCases
	do
        echo "---Make "$unitTestCase
		(cd $unitTestCasesPath$unitTestCase && make)
	done
    unitExePath=$unitTestPath"/bin/"
    unitExeFiles="$(ls $unitExePath)"
    echo "--------- start test ---------"
    for exe in $unitExeFiles
	do
        echo "---Run ./"$unitExePath$exe
        if eval "./$unitExePath$exe";then
            echo "${GREEN}ok${NC}"
        else
            echo "${RED}error${NC}"
        fi
	done
    
elif [ $1 = "clean" ]; then
    rm -rf ./unit/build
fi
