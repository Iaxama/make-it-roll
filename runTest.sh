CURRENT_DIRECTORY=`dirname $0`
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:CURRENT_DIRECTORY/install
export YARP_DATA_DIRS=$YARP_DATA_DIRS:CURRENT_DIRECTORY/install
testrunner -v -s tests/suites/testSuite.xml


