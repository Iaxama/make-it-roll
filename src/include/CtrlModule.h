//
// Created by miacono on 09/01/17.
//

#ifndef MAKE_IT_ROLL_CTRLMODULE_H
#define MAKE_IT_ROLL_CTRLMODULE_H

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
    Port imgLPortOut,imgRPortOut;
    RpcServer rpcPort;

    Mutex mutex;
    Vector cogL,cogR;
    bool okL,okR;


    bool getCOG(ImageOf<PixelRgb> &img, Vector &cog);

    Vector retrieveTarget3D(const Vector &cogL, const Vector &cogR);

    void fixate(const Vector &x);
    
    Vector computeHandOrientation();
    
    void approachTargetWithHand(const Vector &x, const Vector &o);
    
    void makeItRoll(const Vector &x, const Vector &o);
    
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


#endif //MAKE_IT_ROLL_CTRLMODULE_H
