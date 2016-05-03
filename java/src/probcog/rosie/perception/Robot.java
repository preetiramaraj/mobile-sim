package probcog.rosie.perception;

import java.util.HashSet;
import java.util.Properties;

import probcog.lcmtypes.robot_info_t;
import probcog.lcmtypes.tag_classification_list_t;
import probcog.lcmtypes.tag_classification_t;
import sml.Identifier;
import edu.umich.rosie.soar.ISoarObject;
import edu.umich.rosie.soar.SVSCommands;
import edu.umich.rosie.soar.SoarUtil;
import edu.umich.rosie.soar.StringWME;
import edu.umich.rosie.soarobjects.Pose;

public class Robot implements ISoarObject {
	private static final double VIEW_DIST = 3.8;
	private static final double VIEW_ANGLE = Math.PI/2 * .8;
	private static final double VIEW_HEIGHT = 2.0;
	
	private static final double[] dims = new double[]{.5, .5, .5};
	
	private double[] pos = new double[]{ 0.0, 0.0, 0.0 };
	private double[] rot = new double[]{ 0.0, 0.0, 0.0 };
	private Pose pose;
	
	private boolean updatePose = true;
	private boolean updateWaypoint = true;
	
	private StringBuilder svsCommands = new StringBuilder();
	
	private Identifier selfId = null;
	
	private StringWME movingState;
	private StringWME heldObject;
	
	private Region curRegion = null;
	private Identifier waypointId = null;
	
	private MapInfo mapInfo;
	
	public Robot(Properties props){
		movingState = new StringWME("moving-state", "stopped");
		heldObject = new StringWME("held-object", "none");
		pose = new Pose();
		mapInfo = new MapInfo(props);
	}
	
	public MapInfo getMapInfo(){
		return mapInfo;
	}
	
	public void setHeldObject(String handle){
		this.heldObject.setValue(handle == null ? "none" : handle);
	}
	
	public void update(robot_info_t info){
		for(int d = 0; d < 3; d++){
			if(Math.abs(pos[d] - info.xyzrpy[d]) > 0.02){
				pos[d] = info.xyzrpy[d];
				updatePose = true;
			}
		}
		for(int d = 0; d < 3; d++){
			if(Math.abs(rot[d] - info.xyzrpy[3+d]) > 0.05){
				rot[d] = info.xyzrpy[3+d];
				updatePose = true;
			}
		}
		if(updatePose){
			pose.updateWithArray(info.xyzrpy);
		}
		
		HashSet<Region> regions = mapInfo.getRegions(pos);
		Region closest = null;
		Double distance = Double.MAX_VALUE;
		for(Region r : regions){
			double d = r.getDistanceSq(pos);
			if(d < distance){
				distance = d;
				closest = r;
			}
		}
		if(curRegion != closest){
			curRegion = closest;
			updateWaypoint = true;
		}
	}
	
	public Region getRegion(){
		return curRegion;
	}
	 
	 public void updateMovingState(String newMovingState){
		 movingState.setValue(newMovingState);
	 }
	
	/* Creates a triangular view region of height VIEW_DIST
	 * and angle VIEW_ANGLE and a height of 2m
	 */
	public static String getViewRegionVertices(){
		StringBuilder verts = new StringBuilder();
		double dx = VIEW_DIST/2;
		double dy = VIEW_DIST * Math.sin(VIEW_ANGLE/2);
		double dz = VIEW_HEIGHT/2;
		// Top triangle
		verts.append(String.format("%f %f %f ", -dx, 0.0, dz));
		verts.append(String.format("%f %f %f ", dx, -dy, dz));
		verts.append(String.format("%f %f %f ", dx, dy, dz));
		// Top triangle
		verts.append(String.format("%f %f %f ", -dx, 0.0, -dz));
		verts.append(String.format("%f %f %f ", dx, -dy, -dz));
		verts.append(String.format("%f %f %f", dx, dy, -dz));
		
		return verts.toString();
	}
	
	public String getSVSCommands(){
		String cmds = svsCommands.toString();
		svsCommands = new StringBuilder();
		return cmds;
	}
	
	private boolean added = false;

	public boolean isAdded() {
		return added;
	}

	@Override
	public void addToWM(Identifier parentId) {
		if(added){
			return;
		}
		selfId = parentId.CreateIdWME("self");
		pose.addToWM(selfId);
		movingState.addToWM(selfId);
		heldObject.addToWM(selfId);
		
		svsCommands.append(String.format("add robot world p %s r %s\n", 
				SVSCommands.posToStr(pos), SVSCommands.rotToStr(rot)));
		svsCommands.append(String.format("add robot_pos robot\n"));
		svsCommands.append(String.format("add robot_body robot v %s p .2 0 0 s %s\n", 
				SVSCommands.bboxVertices(), SVSCommands.scaleToStr(dims)));
		svsCommands.append(String.format("add robot_view robot v %s p %f %f %f\n", 
				getViewRegionVertices(), VIEW_DIST/2 + .5, 0.0, VIEW_HEIGHT/2 - dims[2]/2));

		added = true;
	}

	@Override
	public void updateWM() {
		if(!added){
			return;
		}

		movingState.updateWM();
		heldObject.updateWM();

		if(updatePose){
			pose.updateWM();
			svsCommands.append(SVSCommands.changePos("robot", pos));
			svsCommands.append(SVSCommands.changeRot("robot", rot));
			updatePose = false;
		}
		
		if(updateWaypoint){
			updateWaypointInfo();
			updateWaypoint = false;
		}
	}
	
	private void updateWaypointInfo(){
		if(waypointId != null){
			waypointId.DestroyWME();
			waypointId = null;
   		}
		if(curRegion != null){
			waypointId = selfId.CreateIdWME("current-waypoint");
			waypointId.CreateStringWME("waypoint-handle", curRegion.id);
			waypointId.CreateStringWME("classification", curRegion.label);
		}
	}

	@Override
	public void removeFromWM() {
		if(!added){
			return;
		}
		
		if(waypointId != null){
			waypointId.DestroyWME();
			waypointId = null;
		}
		movingState.removeFromWM();
		heldObject.removeFromWM();
		pose.removeFromWM();

		selfId.DestroyWME();
		selfId = null;
		
		svsCommands.append(String.format("delete robot\n"));
		added = false;
	}

}
