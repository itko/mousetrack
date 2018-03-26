DIR=`pwd`

# run in build directory
echo "Running lib/tests"

./lib/tests

RESULT_LIB="$?"

# echo "Result: ${RESULT_LIB}"

echo "Running app/appTest"

# create environment for tests

cd ../app/matlab_reader.test/
./create_files.sh
cd ${DIR}

./app/appTests

RESULT_APP="$?"

# echo "Result: ${RESULT_APP}"

RESULT_ALL=$((${RESULT_LIB} + ${RESULT_APP}))

exit ${RESULT_ALL};
