
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | collision detection

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

// KE: merge collision test into FK
// KE: make FK for a configuration and independent of current robot state

kineval.robotIsCollision = function robot_iscollision() {
    // test whether geometry of current configuration of robot is in collision with planning world 

    // form configuration from base location and joint angles
    var q_robot_config = [
        robot.origin.xyz[0],
        robot.origin.xyz[1],
        robot.origin.xyz[2],
        robot.origin.rpy[0],
        robot.origin.rpy[1],
        robot.origin.rpy[2]
    ];

    q_names = {};  // store mapping between joint names and q DOFs

    for (x in robot.joints) {
        q_names[x] = q_robot_config.length;
        q_robot_config = q_robot_config.concat(robot.joints[x].angle);
    }

    // test for collision and change base color based on the result
    collision_result = kineval.poseIsCollision(q_robot_config);

    robot.collision = collision_result;
}


kineval.poseIsCollision = function robot_collision_test(q) {
    // perform collision test of robot geometry against planning world 

    // test base origin (not extents) against world boundary extents
    if ((q[0]<robot_boundary[0][0])||(q[0]>robot_boundary[1][0])||(q[2]<robot_boundary[0][2])||(q[2]>robot_boundary[1][2]))
        return robot.base;

    // traverse robot kinematics to test each body for collision
    // STENCIL: implement forward kinematics for collision detection
    //return robot_collision_forward_kinematics(q);
	
	result = collisionFK(q); // generate xforms for hypothetical config q
	result = recursive_robot_traversal( robot.links[robot.base], robot.links[robot.base].q_xform );
	return result;
}


function recursive_robot_traversal( link, q_xform ) {

	result = traverse_collision_forward_kinematics_link( link, q_xform ); 
	if ( result != false ) return result; // result is link.name

	child_joints = robot.links[link.name].children;
	for (var i=0; i<child_joints.length; i++) {
		child_joint = robot.joints[ child_joints[i] ];
		link = robot.links[ child_joint.child ];
		return recursive_robot_traversal( link, link.q_xform );
	}

	// gets here if no children
	return false;
}


function traverse_collision_forward_kinematics_link(link,mstack) {

    // test collision by transforming obstacles in world to link space
/*
    mstack_inv = matrix_invert_affine(mstack);
*/
    mstack_inv = numeric.inv(mstack);

    var i;
    var j;

    // test each obstacle against link bbox geometry by transforming obstacle into link frame and testing against axis aligned bounding box
    //for (j=0;j<robot_obstacles.length;j++) { 
    for (j in robot_obstacles) { 

        var obstacle_local = matrix_multiply(mstack_inv,robot_obstacles[j].location);

        // assume link is in collision as default
        var in_collision = true; 

        // if obstacle lies outside the link extents along any dimension, no collision is detected
        if (
            (obstacle_local[0][0]<(link.bbox.min.x-robot_obstacles[j].radius))
            ||
            (obstacle_local[0][0]>(link.bbox.max.x+robot_obstacles[j].radius))
        )
                in_collision = false;
        if (
            (obstacle_local[1][0]<(link.bbox.min.y-robot_obstacles[j].radius))
            ||
            (obstacle_local[1][0]>(link.bbox.max.y+robot_obstacles[j].radius))
        )
                in_collision = false;
        if (
            (obstacle_local[2][0]<(link.bbox.min.z-robot_obstacles[j].radius)) 
            ||
            (obstacle_local[2][0]>(link.bbox.max.z+robot_obstacles[j].radius))
        )
                in_collision = false;

        // if obstacle lies within link extents along all dimensions, a collision is detected and return true
        if (in_collision)
            return link.name;
    }
/*
    // recurse child joints for collisions, returning true if child returns collision
    if (typeof link.children !== 'undefined') { // return if there are no children
        var local_collision;
        //for (i=0;i<link.children.length;i++) {
        for (i in link.children) {
            local_collision = traverse_collision_forward_kinematics_joint(robot.joints[link.children[i]],mstack,q)
            if (local_collision)
                return local_collision;
        }
    }
*/
    // return false, when no collision detected for this link and children 
    return false;
}






// Literally just FK copy and pasted with a few modifications
//


function collisionFK (q) { 
	// if geometries are imported and using ROS coordinates (e.g., fetch),
	//   coordinate conversion is needed for kineval/threejs coordinates:
	if (robot.links_geom_imported) {
		offset_xform = matrix_multiply(generate_rotation_matrix_Y(-Math.PI/2),generate_rotation_matrix_X(-Math.PI/2));
	} else {
		offset_xform = generate_identity();
	}
	
	mstack = [generate_identity()];

	result = collisionFKBase(q);
//	return result;
}
function collisionFKBase (q) {
	rot   = generate_rotation_matrix_ZYX([q[3],q[4],q[5]]);
	trans = generate_translation_matrix([q[0],q[1],q[2]]);
	xform = matrix_multiply_3(mstack[mstack.length-1],trans,rot);
	geometried_xform = matrix_multiply(xform,offset_xform);
	mstack.push(geometried_xform);
	robot.links[robot.base].q_xform = geometried_xform;


//	result = traverse_collision_forward_kinematics_link( robot.links[robot.base], geometried_xform );
//	if (result != false) return result;


	var num_children = robot.links[robot.base].children.length;
	if (num_children) { // go down the tree 
		for (var i=0; i<num_children; i++) {
			result = collisionFKJoint(robot.links[robot.base].children[i], q);
//			if (result != false) return result;
		}
	}
	else { //done with traversal
		mstack.pop(); // mstack should be eye(4) after
//		return false;
	}
}
function collisionFKLink (link, q) {

	robot.links[link].q_xform = mstack[mstack.length-1]; //link has same xform as parent joint

//	result = traverse_collision_forward_kinematics_link( robot.links[link], robot.links[link].q_xform );
//	if (result != false) return result;


	var num_children = robot.links[link].children.length;
	if (num_children) { // go down the tree 
		for (var i=0; i<num_children; i++) {
			collisionFKJoint(robot.links[link].children[i], q);
		}
	}
	else {
//		return false;
	} // link is a leaf; go up the tree

}
function collisionFKJoint (joint, q) {

	rot   = generate_rotation_matrix_ZYX(robot.joints[joint].origin.rpy);
	trans = generate_translation_matrix(robot.joints[joint].origin.xyz);
	xform_before_axisrotation = matrix_multiply_3(mstack[mstack.length-1],trans,rot);

	if (robot.joints[joint].type == 'prismatic') {
		axis = robot.joints[joint].axis;
		scale = q[q_names[joint]];//robot.joints[joint].angle;
		scaled_axis =  [scale*axis[0], scale*axis[1], scale*axis[2]];
		transformation = generate_translation_matrix(scaled_axis);	
	}
	else if (robot.joints[joint].type == 'revolute' ||
			 robot.joints[joint].type == 'continuous' || 
			 typeof robot.joints[joint].type == 'undefined') {
		// rotate 0 degrees initially 
		var quat = quaternion_from_axisangle(q[q_names[joint]]/*robot.joints[joint].angle*/,robot.joints[joint].axis); 
		transformation = quaternion_to_rotation_matrix(quat);	
	}
	else {
		transformation = generate_identity();
	}

	xform = matrix_multiply(xform_before_axisrotation, transformation);

	mstack.push(xform);
	robot.joints[joint].q_xform = xform;
	
	// all joints have one (?) child - traditionally, at least
	result = collisionFKLink(robot.joints[joint].child, q);
//	if (result != false) return result;

	// returned from Link traversal, so pop xform off stack
	mstack.pop();
}



