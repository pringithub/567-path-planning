
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | update robot state from controls

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.applyControls = function robot_apply_controls() {
    // apply robot controls to robot kinematics transforms and joint angles, then zero controls
    // includes update of camera position based on base movement

    // update robot configuration from controls
    for (x in robot.joints) {
        if (isNaN(robot.joints[x].control))
            console.warn("kineval: control value for " + x +" is a nan"); //+robot.joints[x].control);

// from stencil code
//        // update joint angles
//        robot.joints[x].angle += robot.joints[x].control;

		switch ( robot.joints[x].type ) {
			case "continuous":
				robot.joints[x].angle += robot.joints[x].control;
				break;

		// STENCIL: enforce joint limits for prismatic and revolute joints
			case "revolute":
				robot.joints[x].angle += robot.joints[x].control;
				var cur_angle = robot.joints[x].angle; 
				if      (cur_angle > robot.joints[x].limit.upper) robot.joints[x].angle = robot.joints[x].limit.upper;
				else if (cur_angle < robot.joints[x].limit.lower) robot.joints[x].angle = robot.joints[x].limit.lower;
				break;

			case "prismatic":
				var axis = 2; // z-axis; TODO: change;here for now
				robot.joints[x].origin.xyz[axis] += robot.joints[x].control;
				// limits chagned from urdf for prismatic joints in kineval_robot_init.js
				if (robot.joints[x].origin.xyz[axis] > (robot.joints[x].limit.upper)) 
					robot.joints[x].origin.xyz[axis] = robot.joints[x].limit.upper;
				else if (robot.joints[x].origin.xyz[axis] < (robot.joints[x].limit.lower))
					robot.joints[x].origin.xyz[axis] = robot.joints[x].limit.lower;
				break;

			case "fixed":
				break;

			// treat un-typed joints as continuous (for mr2 / crawler robots)	
			default:
				robot.joints[x].angle += robot.joints[x].control;
				break;

		}

		// clear controls back to zero for next timestep
        robot.joints[x].control = 0;
    }

    // base motion
    robot.origin.xyz[0] += robot.control.xyz[0];
    robot.origin.xyz[1] += robot.control.xyz[1];
    robot.origin.xyz[2] += robot.control.xyz[2];
    robot.origin.rpy[0] += robot.control.rpy[0];
    robot.origin.rpy[1] += robot.control.rpy[1];
    robot.origin.rpy[2] += robot.control.rpy[2];

    // move camera with robot base
    camera_controls.object.position.x += robot.control.xyz[0];
    camera_controls.object.position.y += robot.control.xyz[1];
    camera_controls.object.position.z += robot.control.xyz[2];

    // zero controls now that they have been applied to robot
    robot.control = {xyz: [0,0,0], rpy:[0,0,0]}; 
}

