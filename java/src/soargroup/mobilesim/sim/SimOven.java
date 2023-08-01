package soargroup.mobilesim.sim;

import java.awt.Color;
import java.util.List;
import java.util.ArrayList;
import java.io.IOException;

import april.sim.*;
import april.vis.*;
import april.util.*;
import april.jmat.LinAlg;

import soargroup.mobilesim.vis.VzOpenBox;
import soargroup.rosie.RosieConstants;

import soargroup.mobilesim.sim.attributes.*;

public class SimOven extends SimShelves {
	public static final double TEMPERATURE = 350.0;
	public final static double LIGHT_H = 0.1;

	protected Activatable activatable;
	protected Surface surface;

	public SimOven(SimWorld sw){
		super(sw);
	}

	@Override
	public void init(ArrayList<SimObject> worldObjects){
		boolean isOn = properties.get(RosieConstants.ACTIVATION).equals(RosieConstants.ACT_ON);
		activatable = new Activatable(this, isOn);
		addAttribute(activatable);

		super.init(worldObjects);
    }

	@Override
	public VisChain createVisObject(){
		VisChain vc = new VisChain();

		// Create open or closed box based on whether the object is open or closed
		boolean open = (openable == null || openable.isOpen());
		if(open){
			vc = ObjectModels.createShelfModel(scale_xyz, color);
		} else {
			vc = new VisChain(new VzBox(scale_xyz, new VzMesh.Style(color)));
		}

		// Create light indicator to indicate whether oven is on or off
		double light_dx = scale_xyz[0]*0.30;
		double light_dy = scale_xyz[1]*0.40;
		double light_z  = (scale_xyz[2])*0.5;
		double light_rad = Math.min(scale_xyz[0], scale_xyz[1])*0.1;
		VzMesh.Style lightStyle = new VzMesh.Style(activatable.isOn() ? Color.red : Color.black);
		ObjectModels.addCyl(vc,  light_dx,  light_dy, light_z, light_rad, LIGHT_H, lightStyle);

		// Add text on top of oven indicating temperature
		String tf="<<monospaced,red,dropshadow=#FFFFFF>>";
		String text = String.format("%s%s\n", tf,((int)TEMPERATURE));

		VzText vzText = new VzText(text);
		double[] textLoc = new double[]{light_dx-0.60, light_dy-0.60, light_z + 0.15};
		// Only show temperature when the oven is on
		if(activatable.isOn())
		{
			vc.add(new VisChain(LinAlg.rotateZ(Math.PI/2), LinAlg.translate(textLoc), LinAlg.scale(0.017), vzText));
		}

		return vc;
	}

	// Children can override to implement any dynamics, this is called multiple times/second
	@Override
	public void update(double dt, ArrayList<SimObject> worldObjects){
		List<RosieSimObject> objects = receptacle.getHeldObjects();
		for(RosieSimObject obj : objects){
			if(obj.is(HasTemp.class)){
				obj.as(HasTemp.class).changeTemperature(TEMPERATURE);
			}
		}
	}
}