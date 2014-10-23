package probcog.slam;

import java.awt.*;
import java.awt.image.*;
import java.io.*;
import java.util.*;
import javax.swing.*;

import lcm.lcm.*;
import lcm.logging.*;

import april.graph.*;
import april.jmat.*;
import april.tag.*;
import april.util.*;
import april.vis.*;

// We have LCMTYPES in a lot of places. We should be able to do what we need
// for this iteration with stuff that is just in APRIL. Later, we probably want
// to switch over to MAGIC2 officially, but for now, avoid dependency. (Types
// unchanged for this application)
//import april.lcmtypes.*;    // laser_t, pose_t, tag_detection*
import magic2.lcmtypes.*;   // Must be magic2 for new tag_detections...

/** Take as input a robot log and create a maximum likelihood map. */
public class OfflineSLAM
{
    static final long STEP_TIME_US = 1*1000000;
    static final double ODOM_NOISE = 0.1; // XXX No working directly from ticks?

    // GUI Misc.
    VisWorld vw;
    VisLayer vl;
    VisCanvas vc;
    ParameterGUI pg;

    // Log state
    Log log;
    pose_t lastPose = null;
    ArrayList<Integer> poseIdxs = new ArrayList<Integer>(); // In graph nodes
    ArrayList<pose_t> poses = new ArrayList<pose_t>();
    // XXX SYNCHRONIZATION IS AN ISSUE HERE...must examine LASER publishing code
    ArrayList<laser_t> lasers = new ArrayList<laser_t>();   // History of LASER?
    HashMap<Integer, Integer> tagIdxs = new ArrayList<Integer>(); // In graph nodes
    ArrayList<tag_detection_list_t> tags = new ArrayList<tag_detection_list_t>();

    // Graph state
    Graph g;

    private class ButtonHandler implements ParameterListener
    {
        public void parameterChanged(ParameterGUI pg, String name)
        {
            if (name.equals("step")) {
                // Read up until the next node is added to the graph
                processLog(false);
            } else if (name.equals("run")) {
                // Slurp up all of the log file
                processLog(true);
            } else if (name.equals("opt")) {
                // Optimize the graph
            }
        }
    }

    public OfflineSLAM(GetOpt opts)
    {
        initGUI();
        initGraph();

        ArrayList<String> extraArgs = opts.getExtraArgs();
        try {
            log = new Log(extraArgs.get(0), "r");
        } catch (IOException ex) {
            System.err.println("ERR: Could not open LCM log "+extraArgs.get(0));
            System.exit(-2);
        }
    }

    private void initGUI()
    {
        JFrame jf = new JFrame("OfflineSLAM");
        jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        jf.setLayout(new BorderLayout());
        jf.setSize(1000,800);

        vw = new VisWorld();
        vl = new VisLayer(vw);
        vc = new VisCanvas(vl);
        jf.add(vc, BorderLayout.CENTER);

        // XXX TODO: Hook up these buttons
        pg = new ParameterGUI();
        pg.addButtons("opt", "Optimize",
                      "step", "Step",
                      "run", "Run");
        pg.addListener(new ButtonHandler());
        jf.add(pg, BorderLayout.SOUTH);

        jf.setVisible(true);
    }

    private void initGraph()
    {
        g = new Graph();

        GXYTNode n = new GXYTNode();
        n.state = new double[3];
        n.init = new double[3];
        n.truth = new double[3];    // We define our initial XYT to be 0.
        n.setAttribute("type", "robot");
        g.nodes.add(n);
        poseIdxs.add(g.nodes.size()-1);

        GXYTPosEdge e = new GXYTPosEdge();
        e.z = new double[3];
        e.truth = new double[3];
        e.P = new double[][] {{1e-6, 0, 0},
                              {0, 1e-6, 0},
                              {0, 0, 1e-6}};
        e.nodes = new int[] {0};
        e.setAttribute("type", "odom");
        g.edges.add(e);

        // Solver...probably Cholesky? XXX
    }

    /** Process the log. If all == true, read in the entire log immediately.
     *  Otherwise, just read until it's time to add the next odometry node.
     */
    private void processLog(boolean all)
    {
        try {
            do {
                // Use log to construct the next node for our graph and all
                // associated edges.
                lcm.logging.Log.Event event = null;
                while (true) {
                    boolean done = false;
                    event = log.readNext();
                    if (event.channel.equals("POSE"))
                        done |= handlePose(event);
                    else if (event.channel.equals("TAG_DETECTION_TX"))
                        handleTags(event);
                    else if (event.channel.equals("LASER"))
                        handleLaser(event);

                    if (!done) {
                        continue;
                    } else {
                        redraw();
                        break;
                    }
                }
            } while (all);
        } catch (IOException ex) {
            System.out.println("End of log reached.");
        }
    }

    private boolean handlePose(lcm.logging.Log.Event event) throws IOException
    {
        pose_t pose = new pose_t(event.data);
        poses.add(pose);
        if (lastPose == null)
            lastPose = pose;
        if (pose.utime - lastPose.utime < STEP_TIME_US)
            return false;

        // Calculate RBT between last pose and now
        double[] dq = LinAlg.quatMultiply(pose.orientation,
                                          LinAlg.quatInverse(lastPose.orientation));
        // XYZ in local world coordinates. Need to transform to robot's
        // local coords.
        double[] dxyz = LinAlg.subtract(pose.pos,
                                        lastPose.pos);
        dxyz = LinAlg.quatRotate(LinAlg.quatInverse(lastPose.orientation),
                                 dxyz);
        double[][] T = LinAlg.quatPosToMatrix(dq, dxyz);

        // Create new node and edge.
        GXYTNode prevNode = (GXYTNode) g.nodes.get(g.nodes.size()-1);
        double[][] pT = LinAlg.xytToMatrix(prevNode.state);
        double[][] nT = LinAlg.matrixAB(pT, T);

        GXYTNode n = new GXYTNode();
        n.state = LinAlg.matrixToXYT(nT);
        n.init = LinAlg.matrixToXYT(nT);
        n.truth = null; // We don't know the truth
        n.setAttribute("type", "robot");

        GXYTEdge e = new GXYTEdge();
        e.z = new double[] {dxyz[0],
            dxyz[1],
            LinAlg.quatToRollPitchYaw(dq)[2]};
        e.truth = null; // We don't know the truth
        e.setAttribute("type", "odom");

        // XXX Make this proportional eventually
        double var = ODOM_NOISE*ODOM_NOISE;
        e.P = new double[][] {{var, 0, 0},
            {0, var, 0},
            {0, 0, var}};
        e.nodes = new int[] {poseIdxs.get(poseIdxs.size()-1), g.nodes.size()};
        poseIdxs.add(g.nodes.size());

        g.nodes.add(n);
        g.edges.add(e);
        lastPose = pose;
        return true;
    }

    private void handleTags(lcm.logging.Log.Event event) throws IOException
    {
        tag_detection_list_t tl = new tag_detection_list_t(event.data);
        tags.add(tl);

        for (tag_detection_t td: tl.detections) {
            // Find tag pose relative to the camera, and then the robot.
            double[][] T = homographyToRBT(td);

            // If this is the first time we've seen a particular tag, add a new
            // XY state node for it. Note: We may strand this node w/o edge!
            // We'll come back for edges later.
            if (!tagIdxs.containsKey(td.id)) {
                GXYNode n = new GXYNode();
                n.state = null;
                n.init = null;
                n.truth = null; // We don't know the truth
                n.setAttribute("type", "tag");

                tagIdxs.put(td.id, g.nodes.size());
                g.nodes.add(n);
            }
         }
    }

    private double[][] homographyToRBT(tag_detection_t td)
    {
        // Grab from config in future
        double fx = 478;        // XXX Made up
        double fy = 478;        // XXX Made up
        double cx = 376;        // XXX
        double cy = 240;        // XXX
        double tagSize = 0.15;
        double[][] H = new double[3][3];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                H[i][j] = td.H[i][j];
            }
        }

        double[][] M = CameraUtil.homographyToPose(fx, fy, cx, cy, H);
        M = CameraUtil.scalePose(M, 2.0, tagSize);

        // Now, convert to a RBT that relates robot to tag

        return M;   // XXX Not quite
    }

    private void redraw()
    {
        VisWorld.Buffer vbRobots = vw.getBuffer("poses");
        vbRobots.setDrawOrder(0);
        VisWorld.Buffer vbTraj = vw.getBuffer("trajectory");
        vbTraj.setDrawOrder(-10);

        ArrayList<double[]> lines = new ArrayList<double[]>();

        for (GEdge o: g.edges) {
            if (o instanceof GXYTEdge) {
                GXYTEdge e = (GXYTEdge)o;

                if (e.nodes.length < 2)
                    continue;

                String type = (String)e.getAttribute("type");
                GXYTNode a = (GXYTNode) g.nodes.get(e.nodes[0]);
                GXYTNode b = (GXYTNode) g.nodes.get(e.nodes[1]);

                if (type == null)
                    continue;
                else if (type.equals("odom")) {
                    lines.add(LinAlg.resize(a.state, 2));
                    lines.add(LinAlg.resize(b.state, 2));
                }
            }
        }
        vbTraj.addBack(new VzLines(new VisVertexData(lines),
                                   VzLines.LINES,
                                   new VzLines.Style(Color.blue, 2)));

        for (GNode o: g.nodes) {
            if (o instanceof GXYTNode) {
                GXYTNode n = (GXYTNode)o;
                String type = (String)n.getAttribute("type");
                if (type == null)
                    continue;
                else if (type.equals("robot")) {
                    vbRobots.addBack(new VisChain(LinAlg.xytToMatrix(n.state),
                                                  new VzRobot(Color.green)));
                }
            }
        }

        vbRobots.swap();
        vbTraj.swap();
    }

    static public void main(String[] args)
    {
        GetOpt opts = new GetOpt();
        opts.addBoolean('h', "help", false, "Show this help screen");
        opts.addString('c', "config", null, "Optional configuration file");

        if (!opts.parse(args)) {
            System.err.println("Options error: "+opts.getReason());
            System.exit(-1);
        }

        boolean badUsage = false;
        ArrayList<String> extraArgs = opts.getExtraArgs();
        if (extraArgs.size() < 1)
            badUsage = true;

        if (opts.getBoolean("help") || badUsage) {
            System.out.printf("Usage: java progcog.slam.OfflineSLAM [options] log-file\n");
            opts.doHelp();
            System.exit(1);
        }

        new OfflineSLAM(opts);
    }
}
