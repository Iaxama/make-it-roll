//
// Created by miacono on 09/01/17.
//

#include "CtrlModule.h"


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
    void CtrlModule::makeItRoll(const Vector &x, const Vector &o)
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
    void CtrlModule::roll(const Vector &cogL, const Vector &cogR)
    {
        yInfo("detected cogs = (%s) (%s)",
              cogL.toString(0,0).c_str(),cogR.toString(0,0).c_str());

        Vector x=retrieveTarget3D(cogL,cogR);
        yInfo("retrieved 3D point = (%s)",x.toString(3,3).c_str());

        fixate(x);
        yInfo("fixating at (%s)",x.toString(3,3).c_str());

        Vector o=computeHandOrientation();
        yInfo("computed orientation = (%s)",o.toString(3,3).c_str());

        approachTargetWithHand(x,o);
        yInfo("approached");

        makeItRoll(x,o);
        yInfo("roll!");
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
        Property optArm("(device cartesiancontrollerclient)");
        optArm.put("remote","/icubSim/cartesianController/right_arm");
        optArm.put("local","/cartesian_client/right_arm");
        if (!drvArm.open(optArm)) {
            drvArm.close();
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

        imgLPortOut.write(*imgL);
        imgRPortOut.write(*imgR);

        return true;
    }

