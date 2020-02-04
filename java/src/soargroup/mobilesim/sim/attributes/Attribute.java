package soargroup.mobilesim.sim.attributes;

import april.vis.*;

import soargroup.mobilesim.sim.*;
import soargroup.mobilesim.sim.actions.*;
import soargroup.mobilesim.sim.actions.ActionHandler.*;
import soargroup.mobilesim.util.ResultTypes.*;

public class Attribute {
	protected final RosieSimObject baseObject;
	public Attribute(RosieSimObject baseObject){
		this.baseObject = baseObject;
		setupRules();
	}

	// Use to set up any action handler rules
	protected void setupRules(){ }

	// Called multiple times per second, do any dynamics or updating here
	// dt is time since last update (fraction of a second)
	public void update(double dt){ }

	// Called when the baseObject moves
	public void moveHandler(double[] xyzrpy){ }

	// Called when the baseObject needs to recalculate its VisObject
	//   you can add elements to the VisChain, 
	//   any transforms are relative to the baseObject itself (pose transformations already applied)
	public void render(VisChain vc){ }

}
