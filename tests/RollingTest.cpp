// -*- mode:C++ { } tab-width:4 { } c-basic-offset:4 { } indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2015 iCub Facility
 * Authors: Ali Paikan
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "RollingTest.h"

using namespace RTF;
using namespace yarp::math;
using namespace yarp::os;
using namespace yarp::sig;


PREPARE_PLUGIN(RollingTest)

RollingTest::RollingTest() : TestCase("RollingTest") { }

RollingTest::~RollingTest() { }

bool RollingTest::setup(int argc, char** argv) {
    RTF_TEST_REPORT("Testing network connection");
    RTF_ASSERT_ERROR_IF(yarp.checkNetwork(),"Not connected to network");

    RTF_TEST_REPORT("Testing port connection");
    rpcCommandPort.open("/testingClient/commands");
    RTF_ASSERT_ERROR_IF(yarp.connect("/testingClient/commands","/service"),"RPC port not connected");

    rpcSimulatorPort.open("/testingClient/simulator");
    RTF_ASSERT_ERROR_IF(yarp.connect("/testingClient/simulator","/icubSim/world"),"RPC port not connected");

    return true;
}


void RollingTest::run() {

    Bottle command, reply;

    RTF_TEST_REPORT("Getting initial ball position");
    command.addString("world");
    command.addString("get");
    command.addString("ball");
    rpcSimulatorPort.write(command,reply);
    Vector initialBallPosition(3);
    initialBallPosition [0]= reply.get(0).asDouble();
    initialBallPosition [1]= reply.get(1).asDouble();
    initialBallPosition [2]= reply.get(2).asDouble();

    RTF_TEST_REPORT(Asserter::format("Initial ball position = [%f, %f, %f]", initialBallPosition[0],initialBallPosition[1],initialBallPosition[2]));
//    std::cout << "initial ball position = " << initialBallPosition.toString() << std::endl;

    command.clear();
    reply.clear();
    RTF_TEST_REPORT("Testing look_down method");
    command.addString("look_down");
    rpcCommandPort.write(command,reply);
    RTF_TEST_REPORT(Asserter::format("Reply: %s",reply.get(0).asString().c_str()));

    command.clear();
    reply.clear();
    RTF_TEST_REPORT("Testing make_it_roll method");
    command.addString("make_it_roll");
    rpcCommandPort.write(command,reply);
    RTF_TEST_REPORT(Asserter::format("Reply: %s",reply.get(0).asString().c_str()));

    command.clear();
    reply.clear();
    RTF_TEST_REPORT("Getting new ball position");
    command.addString("world");
    command.addString("get");
    command.addString("ball");
    rpcSimulatorPort.write(command,reply);
    Vector newBallPosition(3);
    newBallPosition [0]= reply.get(0).asDouble();
    newBallPosition [1]= reply.get(1).asDouble();
    newBallPosition [2]= reply.get(2).asDouble();

    RTF_TEST_REPORT(Asserter::format("New ball position = [%f, %f, %f]", newBallPosition[0],newBallPosition[1],newBallPosition[2]));

    RTF_ASSERT_FAIL_IF(!(newBallPosition == initialBallPosition),"The ball was not hit");

}

void RollingTest::tearDown() {

    RTF_TEST_REPORT("Closing ports");
    rpcCommandPort.close();
    rpcSimulatorPort.close();
}