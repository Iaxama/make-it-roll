// -*- mode:C++ { } tab-width:4 { } c-basic-offset:4 { } indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2015 iCub Facility
 * Authors: Ali Paikan
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef _ROLLINGTEST_H_
#define _ROLLINGTEST_H_

#include <rtf/TestCase.h>
#include <CtrlModule.h>

class RollingTest : public RTF::TestCase {
private:
    CtrlModule* ctrlModule;

public:
    RollingTest();
    virtual ~RollingTest();

    virtual bool setup(int argc, char** argv);

    virtual void tearDown();

    virtual void run();
};

#endif //_ROLLINGTEST_H_
