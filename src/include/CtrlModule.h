//
// Created by miacono on 17/01/17.
//

#ifndef ASSIGNMENT_MAKE_IT_ROLL_CTRLMODULE_H
#define ASSIGNMENT_MAKE_IT_ROLL_CTRLMODULE_H

#include <string>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;

class CtrlModule: public RFModule {
protected:
    PolyDriver drvArm, drvGaze;
    ICartesianControl *iarm;
    IGazeControl      *igaze;
    int startup_ctxt_arm;
    int startup_ctxt_gaze;

    BufferedPort<ImageOf<PixelRgb> > imgLPortIn,imgRPortIn;
    BufferedPort<ImageOf<PixelRgb> > imgLPortOut,imgRPortOut;
    RpcServer rpcPort;

    Mutex mutex;
    Vector cogL,cogR;
    bool okL,okR;

    bool getCOG(ImageOf<PixelRgb> &img, Vector &cog);

    Vector retrieveTarget3D(const Vector &cogL, const Vector &cogR);

    void fixate(const Vector &x);

    Vector computeHandOrientation();

    void approachTargetWithHand(const Vector &x, const Vector &o);

    void make_it_roll(const Vector &x, const Vector &o);

    void look_down();

    void roll(const Vector &cogL, const Vector &cogR);

    void home();

public:

    bool configure(ResourceFinder &rf);

    bool interruptModule();

    bool close();

    bool respond(const Bottle &command, Bottle &reply);

    double getPeriod();

    bool updateModule();
};


#endif //ASSIGNMENT_MAKE_IT_ROLL_CTRLMODULE_H
