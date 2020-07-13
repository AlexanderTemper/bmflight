#!/bin/sh

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
NC='\033[0m' 

if [ $1 = "run" ]; then
    unitTestPath="unit"
    unitTestCasesPath=$unitTestPath"/tests/"
    unitTestCases="$(ls $unitTestCasesPath)"

    echo "${YELLOW}--------- calling make ---------${NC}"
	for unitTestCase in $unitTestCases
	do
        echo "---Make "$unitTestCasesPath$unitTestCase
		(cd $unitTestCasesPath$unitTestCase && make)
	done
    unitExePath=$unitTestPath"/bin/"
    unitExeFiles="$(ls $unitExePath)"
    echo "${YELLOW}--------- start test ---------${NC}"
    for exe in $unitExeFiles
	do
        echo "---Run ./"$unitExePath$exe
        if eval "./$unitExePath$exe" > /dev/null 2>&1 ;then
            echo "${GREEN}OK${NC}"
        else
            #eval "./$unitExePath$exe"
            echo "${RED}ERROR${NC}"
        fi
	done
    
elif [ $1 = "clean" ]; then
    rm -rf ./unit/build
fi
