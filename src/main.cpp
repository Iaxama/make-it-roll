#include <string>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

#include <iCub/ctrl/math.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;


/***************************************************/
class CtrlModule: public RFModule
{
protected:
    PolyDriver drvArm, drvGaze;
    ICartesianControl *iarm;
    IGazeControl      *igaze;
    Vector homeHeadOrientation, armHomePosition, armHomeOrientation;

    BufferedPort<ImageOf<PixelRgb> > imgLPortIn,imgRPortIn;
    Port imgLPortOut,imgRPortOut;
    RpcServer rpcPort;

    Mutex mutex;
    Vector cogL,cogR;
    bool okL,okR;

    /***************************************************/
    bool getCOG(ImageOf<PixelRgb> &img, Vector &cog)
    {
        int xMean=0;
        int yMean=0;
        int ct=0;

        for (int x=0; x<img.width(); x++)
        {
            for (int y=0; y<img.height(); y++)
            {
                PixelRgb &pixel=img.pixel(x,y);
                if ((pixel.b>5.0*pixel.r) && (pixel.b>5.0*pixel.g))
                {
                    xMean+=x;
                    yMean+=y;
                    ct++;
                }
            }
        }

        if (ct>0)
        {
            cog.resize(2);
            cog[0]=xMean/ct;
            cog[1]=yMean/ct;
            return true;
        }
        else
            return false;
    }

    /***************************************************/
    Vector retrieveTarget3D(const Vector &cogL, const Vector &cogR)
    {
        Vector ballPosition;
        igaze->triangulate3DPoint(cogL,cogR,ballPosition);
        return ballPosition;
    }

    /***************************************************/
    void fixate(const Vector &x)
    {
        igaze->lookAtFixationPoint(x);
    }

    /***************************************************/
    Vector computeHandOrientation()
    {
        Vector oy(4), oz(4);
        oy[0]=0.0; oy[1]=1.0; oy[2]=0.0; oy[3]=+M_PI/2.0;
        oz[0]=0.0; oz[1]=0.0; oz[2]=1.0; oz[3]=-M_PI/2.0;
        Matrix Ry=yarp::math::axis2dcm(oy);        // from axis/angle to rotation matrix notation
        Matrix Rz=yarp::math::axis2dcm(oz);
        Matrix R=Rz*Ry;                            // compose the two rotations keeping the order
        return yarp::math::dcm2axis(R);          // from rotation matrix back to the axis/angle notation
    }

    /***************************************************/
    void approachTargetWithHand(const Vector &x, const Vector &o)
    {
        Vector approachPosition (3);
        approachPosition[0] = x[0];
        approachPosition[1] = x[1] + 0.2;
        approachPosition[2] = x[2];

        iarm->goToPose(approachPosition,o);
        iarm->waitMotionDone();
    }

    /***************************************************/
    void makeItRoll(const Vector &x, const Vector &o)
    {
        Vector rollingPosition (3);
        rollingPosition[0] = x[0];
        rollingPosition[1] = x[1] -0.2;
        rollingPosition[2] = x[2];

        iarm->goToPose(rollingPosition,o);
        iarm->waitMotionDone();
    }

    /***************************************************/
    void look_down()
    {
        Vector ang(3);
        ang[0] = 0;
        ang[1] = -40;
        ang[2] = 0;

        igaze->lookAtRelAngles(ang);
        igaze->waitMotionDone();
    }

    /***************************************************/
    void roll(const Vector &cogL, const Vector &cogR)
    {
//        yInfo("detected cogs = (%s) (%s)",
//              cogL.toString(0,0).c_str(),cogR.toString(0,0).c_str());
//
//        Vector x=retrieveTarget3D(cogL,cogR);
//        yInfo("retrieved 3D point = (%s)",x.toString(3,3).c_str());
//
//        fixate(x);
//        yInfo("fixating at (%s)",x.toString(3,3).c_str());

        Vector x,o;
        iarm->getPose(x,o);
        o=computeHandOrientation();
        yInfo("computed orientation = (%s)",o.toString(3,3).c_str());

        approachTargetWithHand(x,o);
        yInfo("approached");

//        makeItRoll(x,o);
//        yInfo("roll!");
    }

    /***************************************************/
    void home()
    {
        igaze->lookAtAbsAngles(homeHeadOrientation);
        iarm->goToPose(armHomePosition,armHomeOrientation);
        igaze->waitMotionDone();
        iarm->waitMotionDone();
    }

public:
    /***************************************************/
    bool configure(ResourceFinder &rf)
    {
        // FILL IN THE CODE

        imgLPortIn.open("/imgL:i");
        imgRPortIn.open("/imgR:i");

        imgLPortOut.open("/imgL:o");
        imgRPortOut.open("/imgR:o");

        rpcPort.open("/service");
        attach(rpcPort);

        //Configuring drivers for head
        Property options;
        options.put("device", "gazecontrollerclient");
        options.put("local", "/client/head");
        options.put("remote", "/iKinGazeCtrl");
        drvGaze.open(options);

        if (!drvGaze.isValid()){
            std::cerr << "Device not available! The known devices are" << std::endl;
            std::cerr << Drivers::factory().toString().c_str() << std::endl;
            return false;
        }

        drvGaze.view(igaze);
        if (igaze == 0){
            std::cerr << "Error opening position control interface for gaze" << std::endl;
            return false;
        }

        //Configuring drivers for right arm
        options.put("device", "cartesiancontrollerclient");
        options.put("local", "/client/right_arm");
        options.put("remote", "/icubSim/cartesianController/right_arm");

        drvArm.open(options);
        if (!drvArm.isValid()){
            std::cerr << "Device not available! The known devices are" << std::endl;
            std::cerr << Drivers::factory().toString().c_str() << std::endl;
            return false;
        }

        drvArm.view(iarm);
        if (iarm == 0){
            std::cerr << "Error opening position control interface for right arm" << std::endl;
            return false;
        }


        igaze->getAngles(homeHeadOrientation);
        iarm->getPose(armHomePosition,armHomeOrientation);
        std::cout << "armHomePosition = " << armHomePosition.toString() << std::endl;
        std::cout << "armHomeOrientation = " << armHomeOrientation.toString() << std::endl;
        return true;
    }

    /***************************************************/
    bool interruptModule()
    {
        imgLPortIn.interrupt();
        imgRPortIn.interrupt();
        return true;
    }

    /***************************************************/
    bool close()
    {
        home();
        drvArm.close();
        drvGaze.close();
        imgLPortIn.close();
        imgRPortIn.close();
        imgLPortOut.close();
        imgRPortOut.close();
        rpcPort.close();
        return true;
    }


/***************************************************/
    bool respond(const Bottle &command, Bottle &reply)
    {
        string cmd=command.get(0).asString();
        if (cmd=="help")
        {
            reply.addVocab(Vocab::encode("many"));
            reply.addString("Available commands:");
            reply.addString("- look_down");
            reply.addString("- make_it_roll");
            reply.addString("- home");
            reply.addString("- quit");
        }
        else if (cmd=="look_down")
        {
            look_down();
            reply.addString("Yep! I'm looking down now!");
        }
        else if (cmd=="roll")
        {
            double p[3] = {-0.1,0.35,0.15};
            iarm->goToPosition(Vector(3,p));

            if (okL && okR)
            {
                roll(cogL,cogR);
                reply.addString("Yeah! I've made it roll like a charm!");
            }
            else
                reply.addString("I don't see any object!");
        }
        else if (cmd=="home")
        {
            home();
            reply.addString("I've got the hard work done! Going home.");
        }
        else
            return RFModule::respond(command,reply);

        return true;
    }

    /***************************************************/
    double getPeriod()
    {
        return 0.0;     // sync upon incoming images
    }

    /***************************************************/
    bool updateModule()
    {
        // get fresh images
        ImageOf<PixelRgb> *imgL=imgLPortIn.read();
        ImageOf<PixelRgb> *imgR=imgRPortIn.read();

        // interrupt sequence detected
        if ((imgL==NULL) || (imgR==NULL))
            return true;

        // compute the center-of-mass of pixels of our color
        mutex.lock();
        okL=getCOG(*imgL,cogL);
        okR=getCOG(*imgR,cogR);
        mutex.unlock();

        PixelRgb color;
        color.r=255; color.g=0; color.b=0;

        if (okL)
            draw::addCircle(*imgL,color,(int)cogL[0],(int)cogL[1],5);

        if (okR)
            draw::addCircle(*imgR,color,(int)cogR[0],(int)cogR[1],5);

        imgLPortOut.write(*imgL);
        imgRPortOut.write(*imgR);

        return true;
    }


};


/***************************************************/
int main()
{
    Network yarp;
    if (!yarp.checkNetwork())
        return 1;

    CtrlModule mod;
    ResourceFinder rf;
    mod.runModule(rf);

    return 0;
}

