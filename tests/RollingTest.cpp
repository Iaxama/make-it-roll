// -*- mode:C++ { } tab-width:4 { } c-basic-offset:4 { } indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2015 iCub Facility
 * Authors: Ali Paikan
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */


#include <rtf/TestAssert.h>
#include <rtf/dll/Plugin.h>

#include "RollingTest.h"

using namespace RTF;

PREPARE_PLUGIN(RollingTest)

RollingTest::RollingTest() : TestCase("RollingTest") { }

RollingTest::~RollingTest() { }

bool RollingTest::setup(int argc, char** argv) {

    ctrlModule = new CtrlModule;

    RTF_ASSERT_ERROR_IF(ctrlModule != NULL, "ctrlModule not correctly instantiated");
    RTF_TEST_REPORT("running RollingTest::setup...");

    return true;
}


void RollingTest::run() {
    RTF_TEST_REPORT("testing integers");
    RTF_TEST_FAIL_IF(4<3, "is not smaller");
    RTF_TEST_CHECK(2<3,"test 2<3");
    RTF_TEST_CHECK(4<3,"test 4<3");
    int a = 5;
    int b = 3;
    RTF_TEST_FAIL_IF(a<b, Asserter::format("%d is not smaller than %d.", a, b));
}

void RollingTest::tearDown() {

    delete ctrlModule;
    RTF_TEST_REPORT("running RollingTest::teardown...");
    // assert an arbitrary error for example.
    RTF_ASSERT_ERROR("this is just for example!");
}