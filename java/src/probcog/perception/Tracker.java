package probcog.perception;

import java.io.*;
import java.util.*;

import lcm.lcm.*;

import april.config.*;
import april.jmat.*;
import april.jmat.geom.*;
import april.util.*;

import probcog.lcmtypes.*;

public class Tracker
{
    static LCM lcm = LCM.getSingleton();
    private Object soarLock;
    private soar_objects_t soar_lcm;

    private KinectSegment segmenter;

    public Tracker(Config color, Config ir, Config armConfig)
    {
        segmenter = new KinectSegment(color, ir, armConfig);

        soarLock = new Object();
        soar_lcm = null;
        new ListenerThread().start();
    }

    public void compareObjects()
    {
        ArrayList<Obj> visibleObjects = getVisibleObjects();
        ArrayList<Obj> soarObjects = getSoarObjects();

        // XXX - Need some sort of matching code here.
    }


    /** Returns a list of objects that the kinect sees on the table. The objects
     *  are returned as pointClouds from the segmenter, and are passed to the
     *  classifiers. The resulting point clouds, their locations, and the
     *  classifications are returned.
     **/
    private ArrayList<Obj> getVisibleObjects()
    {
        ArrayList<Obj> visibleObjects = new ArrayList<Obj>();
        boolean assignID = false;

        ArrayList<PointCloud> ptClouds = segmenter.getObjectPointClouds();
        for(PointCloud ptCloud : ptClouds) {
            Obj vObj = new Obj(assignID, ptCloud);
            // XXX - should we be classifying objects before the tracking?
            // XXX - hand off to classifiers
            visibleObjects.add(vObj);
        }

        return visibleObjects;
    }

    /** Returns a list of Obj that Soar believes exists in the world based on
     *  their most recent lcm message. Obj have information such as their center
     *  and features they were classified with. They do not have point clouds.
     **/
    private ArrayList<Obj> getSoarObjects()
    {
        ArrayList<Obj> soarObjects = new ArrayList<Obj>();

        synchronized(soarLock) {
            if(soar_lcm != null) {
                for(int i=0; i<soar_lcm.num_objects; i++) {
                    object_data_t odt = soar_lcm.objects[i];
                    Obj sObj = new Obj(odt.id);
                    double[] xyzrpy = odt.pos;
                    sObj.setCentroid(new double[]{xyzrpy[0], xyzrpy[1], xyzrpy[2]});

                    for(int j=0; j<odt.num_cat; j++) {
                        // XXX - Need to do something with this information
                    }
                    soarObjects.add(sObj);
                }
            }
        }
        return soarObjects;
    }

    // public void sendMessage()
    // {
    //     observations_t obs = new observations_t();
    //     obs.utime = TimeUtil.utime();
    //     synchronized(objectManager.objects) {
    //         obs.click_id = simulator.getSelectedId();
    //     }
    //     obs.sensables = sensableManager.getSensableStrings();
    //     obs.nsens = obs.sensables.length;
    //     obs.observations = classifierManager.getObjectData();
    //     obs.nobs = obs.observations.length;

    //     lcm.publish("TRACKED_OBJECTS",obs);
    // }


    /** Class that continually listens for messages from Soar about what objects
     *  it believes exists in the world. The received lcm message is stored so it
     *  can be used upon request.
     **/
    class ListenerThread extends Thread implements LCMSubscriber
    {
        LCM lcm = LCM.getSingleton();

        public ListenerThread()
        {
            lcm.subscribe("SOAR_OBJECTS", this);
        }

        public void run()
        {
            while(true) {
                TimeUtil.sleep(1000/60);    // XXX.
            }
        }

        public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
        {
            try {
                if(channel.equals("SOAR_OBJECTS")) {
                    synchronized (soarLock) {
                        soar_lcm = new soar_objects_t(ins);
                    }
                }
            } catch(IOException ioex) {
                System.err.println("ERR: LCM channel ="+channel);
                ioex.printStackTrace();
            }
        }
    }
}