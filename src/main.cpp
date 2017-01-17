#include <CtrlModule.h>

int main()
{   
    Network yarp;
    if (!yarp.checkNetwork())
        return 1;

    CtrlModule mod;
    ResourceFinder rf;
    return mod.runModule(rf);
}

