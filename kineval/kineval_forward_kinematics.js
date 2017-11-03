
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | forward kinematics

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.robotForwardKinematics = function robotForwardKinematics () { 

    if (typeof kineval.buildFKTransforms === 'undefined') {
        textbar.innerHTML = "forward kinematics not implemented";
        return;
    }

    // STENCIL: implement kineval.buildFKTransforms()
	
	// if geometries are imported and using ROS coordinates (e.g., fetch),
	//   coordinate conversion is needed for kineval/threejs coordinates:
	if (robot.links_geom_imported) {
		offset_xform = matrix_multiply(generate_rotation_matrix_Y(-Math.PI/2),generate_rotation_matrix_X(-Math.PI/2));
	} else {
		offset_xform = generate_identity();
	}
	
	mstack = [generate_identity()];
	kineval.buildFKTransforms();
}



kineval.buildFKTransforms = function traverseFKBase() {

 	//console.log(robot.links["base_link"].name);

	rot   = generate_rotation_matrix_ZYX(robot.origin.rpy);
	trans = generate_translation_matrix(robot.origin.xyz);
	xform = matrix_multiply_3(mstack[mstack.length-1],trans,rot);
	geometried_xform = matrix_multiply(xform,offset_xform);
	mstack.push(geometried_xform);
	robot.links[robot.base].xform = geometried_xform;
	//robot.links[robot.base].xform = matrix_multiply(offset_xform, xform);

	// for scene rendering
	//   multiply un-geometried xform (why, who knows - magic!)
	var robot_h = [[0],[0],[1],[1]]; // z-axis
	robot_heading = matrix_multiply(xform, robot_h);
	var robot_l = [[1],[0],[0],[1]]; // x-axis
	robot_lateral = matrix_multiply(xform, robot_l);

	var num_children = robot.links[robot.base].children.length;
	if (num_children) { // go down the tree 
		for (var i=0; i<num_children; i++) {
			traverseFKJoint(robot.links[robot.base].children[i]);
		}
	}
	else { //done with traversal
		mstack.pop(); // mstack should be eye(4) after
	}
}
function traverseFKLink(link) {

 	//console.log("\t"+robot.links[link].name);

	robot.links[link].xform = mstack[mstack.length-1]; //link has same xform as parent joint

	var num_children = robot.links[link].children.length;
	if (num_children) { // go down the tree 
		for (var i=0; i<num_children; i++) {
			traverseFKJoint(robot.links[link].children[i]);
		}
	}
	else { // link is a leaf; go up the tree
	//	console.log("\n");
	}
}
function traverseFKJoint(joint) {

	rot   = generate_rotation_matrix_ZYX(robot.joints[joint].origin.rpy);
	trans = generate_translation_matrix(robot.joints[joint].origin.xyz);
	xform_before_axisrotation = matrix_multiply_3(mstack[mstack.length-1],trans,rot);

	q = quaternion_from_axisangle(robot.joints[joint].angle,robot.joints[joint].axis); // rotate 0 degrees
	rot_axis = quaternion_to_rotation_matrix(q);	
	xform = matrix_multiply(xform_before_axisrotation, rot_axis);

	mstack.push(xform);
	robot.joints[joint].xform = xform;
	
	// all joints have one (?) child - traditionally, at least
	traverseFKLink(robot.joints[joint].child);

	// returned from Link traversal, so pop xform off stack
	mstack.pop();
}

    // STENCIL: reference code alternates recursive traversal over 
    //   links and joints starting from base, using following functions: 
    //     traverseFKBase
    //     traverseFKLink
    //     traverseFKJoint
    //
    // user interface needs the heading (z-axis) and lateral (x-axis) directions
    //   of robot base in world coordinates stored as 4x1 matrices in
    //   global variables "robot_heading" and "robot_lateral"
    //
    // if geometries are imported and using ROS coordinates (e.g., fetch),
    //   coordinate conversion is needed for kineval/threejs coordinates:
    //
    //   if (robot.links_geom_imported) {
    //       var offset_xform = matrix_multiply(generate_rotation_matrix_Y(-Math.PI/2),generate_rotation_matrix_X(-Math.PI/2));

