//
// Created by miacono on 17/01/17.
//

#include "include/CtrlModule.h"

    /***************************************************/
    bool CtrlModule::getCOG(ImageOf<PixelRgb> &img, Vector &cog)
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
    Vector CtrlModule::retrieveTarget3D(const Vector &cogL, const Vector &cogR)
    {
        Vector x;
        igaze->triangulate3DPoint(cogL,cogR,x);
        return x;
    }

    /***************************************************/
    void CtrlModule::fixate(const Vector &x)
    {
        igaze->lookAtFixationPoint(x);
        igaze->waitMotionDone();
        igaze->setTrackingMode(true);
    }

    /***************************************************/
    Vector CtrlModule::computeHandOrientation()
    {
        Matrix R(3,3);
        R(0,0)=-1.0; R(0,1)= 0.0; R(0,2)= 0.0;
        R(1,0)= 0.0; R(1,1)= 0.0; R(1,2)=-1.0;
        R(2,0)= 0.0; R(2,1)=-1.0; R(2,2)= 0.0;

        return dcm2axis(R);
    }

    /***************************************************/
    void CtrlModule::approachTargetWithHand(const Vector &x, const Vector &o)
    {
        // enable torso dofs
        Vector dof(3,1.0);
        iarm->setDOF(dof,dof);

        Vector approach_x=x;
        approach_x[1]+=0.1;
        iarm->goToPoseSync(approach_x,o);
        iarm->waitMotionDone();
    }

    /***************************************************/
    void CtrlModule::roll(const Vector &x, const Vector &o)
    {
        iarm->setTrajTime(0.3);
        iarm->goToPoseSync(x,o);
        iarm->waitMotionDone();
    }

    /***************************************************/
    void CtrlModule::look_down()
    {
        Vector ang(3,0.0);
        ang[1]=-40.0;
        igaze->lookAtAbsAngles(ang);
        igaze->waitMotionDone();
    }

    /***************************************************/
    void CtrlModule::make_it_roll(const Vector &cogL, const Vector &cogR)
    {
        yInfo()<<"detected cogs = ("<<cogL.toString(0,0)<<") ("<<cogR.toString(0,0)<<")";

        Vector x=retrieveTarget3D(cogL,cogR);
        yInfo()<<"retrieved 3D point = ("<<x.toString(3,3)<<")";

        fixate(x);
        yInfo()<<"fixating at ("<<x.toString(3,3)<<")";

        Vector o=computeHandOrientation();
        yInfo()<<"computed orientation = ("<<o.toString(3,3)<<")";

        approachTargetWithHand(x,o);
        yInfo()<<"approached";

        roll(x,o);
        yInfo()<<"roll!";
    }

    /***************************************************/
    void CtrlModule::home()
    {
        Vector ang(3,0.0);
        igaze->setTrackingMode(true);
        igaze->lookAtAbsAngles(ang);

        Vector x(3);
        x[0]=-0.2;
        x[1]=0.35;
        x[2]=0.1;
        iarm->setTrajTime(0.75);
        iarm->goToPosition(x);
    }

    /***************************************************/
    bool CtrlModule::configure(ResourceFinder &rf)
    {
        Property optArm;
        optArm.put("device","cartesiancontrollerclient");
        optArm.put("remote","/icubSim/cartesianController/right_arm");
        optArm.put("local","/cartesian_client/right_arm");

        // let's give the controller some time to warm up
        bool ok=false;
        double t0=Time::now();
        while (Time::now()-t0<10.0)
        {
            // this might fail if controller
            // is not connected to solver yet
            if (drvArm.open(optArm))
            {
                ok=true;
                break;
            }

            Time::delay(1.0);
        }

        if (!ok)
        {
            yError()<<"Unable to open the Cartesian Controller";
            return false;
        }

        Property optGaze("(device gazecontrollerclient)");
        optGaze.put("remote","/iKinGazeCtrl");
        optGaze.put("local","/gaze_client");
        if (!drvGaze.open(optGaze))
        {
            drvArm.close();
            return false;
        }

        drvArm.view(iarm);
        drvGaze.view(igaze);

        // save startup contexts
        iarm->storeContext(&startup_ctxt_arm);
        igaze->storeContext(&startup_ctxt_gaze);

        // make the controllers move faster
        iarm->setTrajTime(0.5);
        igaze->blockNeckRoll(0.0);
        igaze->setNeckTrajTime(0.5);

        imgLPortIn.open("/imgL:i");
        imgRPortIn.open("/imgR:i");

        imgLPortOut.open("/imgL:o");
        imgRPortOut.open("/imgR:o");

        rpcPort.open("/service");

        if (!attach(rpcPort))
            return false;

        return true;
    }

    /***************************************************/
    bool CtrlModule::interruptModule()
    {
        imgLPortIn.interrupt();
        imgRPortIn.interrupt();
        return true;
    }

    /***************************************************/
    bool CtrlModule::close()
    {
        iarm->restoreContext(startup_ctxt_arm);
        igaze->restoreContext(startup_ctxt_gaze);

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
    bool CtrlModule::respond(const Bottle &command, Bottle &reply)
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
            // we assume the robot is not moving now
            reply.addString("ack");
            reply.addString("Yep! I'm looking down now!");
        }
        else if (cmd=="make_it_roll")
        {
            mutex.lock();
            Vector cogL=this->cogL;
            Vector cogR=this->cogR;
            bool go=okL && okR;
            mutex.unlock();

            if (go)
            {
                make_it_roll(cogL,cogR);
                // we assume the robot is not moving now
                reply.addString("ack");
                reply.addString("Yeah! I've made it roll like a charm!");
            }
            else
            {
                reply.addString("nack");
                reply.addString("I don't see any object!");
            }
        }
        else if (cmd=="home")
        {
            home();
            // we assume the robot is not moving now
            reply.addString("ack");
            reply.addString("I've got the hard work done! Going home.");
        }
        else
            // the father class already handles the "quit" command
            return RFModule::respond(command,reply);

        return true;
    }

    /***************************************************/
    double CtrlModule::getPeriod()
    {
        return 0.0;     // sync upon incoming images
    }

    /***************************************************/
    bool CtrlModule::updateModule()
    {
        // get fresh images
        ImageOf<PixelRgb> *imgL=imgLPortIn.read();
        ImageOf<PixelRgb> *imgR=imgRPortIn.read();

        // interrupt sequence detected
        if ((imgL==NULL) || (imgR==NULL))
            return false;

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

        imgLPortOut.prepare()=*imgL;
        imgRPortOut.prepare()=*imgR;

        imgLPortOut.write();
        imgRPortOut.write();

        return true;
    }


