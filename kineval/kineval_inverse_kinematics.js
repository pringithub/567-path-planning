
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | inverse kinematics

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.robotInverseKinematics = function robot_inverse_kinematics(endeffector_target_world, endeffector_joint, endeffector_position_local) {

    // compute joint angle controls to move location on specified link to Cartesian location
    if ((kineval.params.update_ik)||(kineval.params.persist_ik)) { 
        // if update requested, call ik iterator and show endeffector and target
        kineval.iterateIK(endeffector_target_world, endeffector_joint, endeffector_position_local);
        if (kineval.params.trial_ik_random.execute)
            kineval.randomizeIKtrial();
        else // KE: this use of start time assumes IK is invoked before trial
            kineval.params.trial_ik_random.start = new Date();
    }

    kineval.params.update_ik = false; // clear IK request for next iteration
}


kineval.randomizeIKtrial = function randomIKtrial () {

   // update time from start of trial
    cur_time = new Date();
    kineval.params.trial_ik_random.time = cur_time.getTime()-kineval.params.trial_ik_random.start.getTime();

	if (kineval.params.go_ahead == 10) {
		kineval.params.go_ahead = 0;i
//		var currentTime = new Date().getTime();
//		while (currentTime + 200 >= new Date().getTime()) {
		}
	}
	else {
		kineval.params.go_ahead++;

		console.log(kineval.params.trial_ik_random.time)

	   // get endeffector Cartesian position in the world
		endeffector_world = matrix_multiply(robot.joints[robot.endeffector.frame].xform,robot.endeffector.position);

	   // compute distance of endeffector to target
		kineval.params.trial_ik_random.distance_current = Math.sqrt(
				Math.pow(kineval.params.ik_target.position[0][0]-endeffector_world[0][0],2.0)
				+ Math.pow(kineval.params.ik_target.position[1][0]-endeffector_world[1][0],2.0)
				+ Math.pow(kineval.params.ik_target.position[2][0]-endeffector_world[2][0],2.0) );

	   // if target reached, increment scoring and generate new target location
		// KE 2 : convert hardcoded constants into proper parameters
	//    if (kineval.params.trial_ik_random.distance_current < 0.01) {
			kineval.params.ik_target.position[0][0] = kineval.ik_data[kineval.params.trial_ik_random.targets][0];  //1.2*(Math.random()-0.5);
			kineval.params.ik_target.position[1][0] = kineval.ik_data[kineval.params.trial_ik_random.targets][1];  //1.2*(Math.random()-0.5)+1.5;
			kineval.params.ik_target.position[2][0] = 0.75;//0.7*(Math.random()-0.5)+0.5;
			kineval.params.trial_ik_random.targets += 1;
			textbar.innerHTML = "IK trial Random: target " + kineval.params.trial_ik_random.targets + " reached at time " + kineval.params.trial_ik_random.time;
	//    }
	}

}



/*
kineval.randomizeIKtrial = function randomIKtrial () {

    // update time from start of trial
    cur_time = new Date();
    kineval.params.trial_ik_random.time = cur_time.getTime()-kineval.params.trial_ik_random.start.getTime();

    // STENCIL: see instructor for random time trial code
}
*/

// endeffector_target_world:
// 		an object with two fields with the target endeffector position 
// 		(as a 3D homogeneous vector) and orientation (as Euler angles) in the world frame
// endeffector_joint:
// 	    the name of the joint directly connected to the endeffector
// endeffector_position_local:
// 	    the location of the endeffector in the local joint frame	
kineval.iterateIK = function iterate_inverse_kinematics(endeffector_target_world, endeffector_joint, endeffector_position_local) {

	// build kinematic chain (should only need to do this once)
	chain = build_kinematic_chain(endeffector_joint);


	// compute endpoint error; delta_xn = xd - xn
  	xdes = [];
	if (!kineval.params.ik_orientation_included) {
		for (var i=0; i<3; i++) xdes[i] = endeffector_target_world.position[i];
		for (var i=3; i<6; i++) xdes[i] = [0]; 
	} else {
		for (var i=0; i<3; i++) xdes[i] = endeffector_target_world.position[i];
		for (var i=3; i<6; i++) xdes[i] = [endeffector_target_world.orientation[i-3]];
	}

	T_location2world = robot.joints[endeffector_joint].xform;
	endeffector_position_world = matrix_multiply( T_location2world, endeffector_position_local ); 
	endeffector_orientation_world = getEulerAnglesFromXForm( T_location2world );
	
	xendeff = [];
	if (!kineval.params.ik_orientation_included) {
		for (var i=0; i<3; i++) xendeff[i] = endeffector_position_world[i];
		for (var i=3; i<6; i++) xendeff[i] = [0]; 
	} else {
		for (var i=0; i<3; i++) xendeff[i] = endeffector_position_world[i];
		for (var i=3; i<6; i++) xendeff[i] = [endeffector_orientation_world[i-3]];
	}
	delta_xn = matrix_addsub( xdes,xendeff, '-' );


	J = getJacobian( endeffector_joint, endeffector_position_local );


	// select pinv or transpose
	if (kineval.params.ik_pseudoinverse) inv_J = matrix_pseudoinverse( J );
	else 							     inv_J = matrix_transpose( J );
	delta_qn = matrix_multiply( inv_J, delta_xn ); // nx1

	// perform step direction
	//   qn+1 = qn + gam*delta_qn
	gamma = kineval.params.ik_steplength; 
	step = scalar_multiply_matrix( gamma,delta_qn );

	
	// place at new configuration
	for (var i=0; i<chain.length-1; i++) { 			   // gripper axis doesn't count
		robot.joints[ chain[i] ].control = step[i][0]; // get first element of row
	}

}




function getJacobian(endeffector_joint, endeffector_position_local) { 
	J = [];
	J[0] = [];
	
	cur_joint = endeffector_joint;
	parent_link = robot.joints[cur_joint].parent;
	parent_joint = robot.links[parent_link].parent;
	while (parent_joint != null) {

		var xform  = robot.joints[parent_joint].xform;
		var axis   = robot.joints[parent_joint].axis; axis[3]=1; // homogenous coords
		axis = generate_column_matrix_from_array( axis );
		var origin = [[0],[0],[0],[1]]; 					  // origin wrt itself is 0 
		joint_axis_wrt_world   = matrix_multiply( xform, axis ); 
		joint_origin_wrt_world = matrix_multiply( xform, origin );

   	    // spong notation; z_iminus1 is joint axis translated to the origin
		z_iminus1 = matrix_addsub( joint_axis_wrt_world, joint_origin_wrt_world, '-' );

		joint_type = robot.joints[parent_joint].type;
		if (joint_type == 'revolute' ||
			joint_type == 'continuous' ||
			joint_type == 'fixed' ||             // fixed controls not applied
			typeof joint_type === 'undefined') { // undefined treated as continuous

			// get distance from current joint to endeffector
			var endeffector_xform = robot.joints[endeffector_joint].xform;
			endeffector_point_wrt_world = matrix_multiply( endeffector_xform, 
														   endeffector_position_local ); 
			endeffector_to_cur_distance = matrix_addsub( endeffector_point_wrt_world, 
														 joint_origin_wrt_world, '-' );			

			// conversion bullshit
			z_iminus1.pop(); endeffector_to_cur_distance.pop();

			// get jac'ed
			jac_top = vector_cross( z_iminus1, endeffector_to_cur_distance );
			jac_bottom = z_iminus1;
		}
		else if (joint_type == 'prismatic') {
			jac_top = z_iminus1;
			jac_bottom = [[0],[0],[0]];
		}

		Ji = matrix_jac_layer( jac_top,jac_bottom );

		// more bullshit !!!!
		Ji = generate_column_matrix_from_array( Ji );

		J = matrix_jac_stack( Ji, J );

		// move up the chain
		cur_joint = parent_joint;
		parent_link = robot.joints[cur_joint].parent;
		parent_joint = robot.links[parent_link].parent;

	}

	return J;
}




function build_kinematic_chain(endeffector_joint) {

	chain = [];
	i = 0;

	cur_joint = endeffector_joint;
	parent_link = robot.joints[cur_joint].parent;
	parent_joint = robot.links[parent_link].parent;
	while (parent_joint != null) { 
		parent_link = robot.joints[cur_joint].parent;
		parent_joint = robot.links[parent_link].parent;
		
		chain[i++] = cur_joint;
		cur_joint = parent_joint;
	}

	// reverse chain
	for (var i=0; i<chain.length/2; i++) {
		var tmp = chain[chain.length-i-1]; // end
		chain[chain.length-i-1] = chain[i];
		chain[i] = tmp;
	}

	return chain;
}



function getEulerAnglesFromXForm( xform ) {

//xzy ????
// also zxy???
	
	var mode = 'zxy';
	switch (mode) {
		case 'xyz':
			theta_x = Math.atan2( -xform[1][2], xform[2][2] );
			theta_y = Math.asin( xform[0][2] );
			theta_z = Math.atan2( -xform[0][1], xform[0][0] );
			break;
		case 'xzy':
			theta_x = Math.atan2( xform[2][1], xform[1][1] );
			theta_y = Math.atan2( xform[0][2], xform[0][0] );
			theta_z = Math.asin( -xform[0][1] );
			break;
		case 'yxz':
			theta_x = Math.asin( -xform[1][2] );
			theta_y = Math.atan2( xform[0][2], xform[2][2] );
			theta_z = Math.atan2( xform[1][0], xform[1][1] );
			break;
		case 'yzx':
			theta_x = Math.atan2( -xform[1][2], xform[1][1] );
			theta_y = Math.atan2( -xform[2][0], xform[0][0] );
			theta_z = Math.asin( xform[1][0] );
			break;
		case 'zxy':
			theta_x = Math.asin( xform[2][1] );
			theta_y = Math.atan2( -xform[2][0], xform[2][2] );
			theta_z = Math.atan2( -xform[0][1], xform[1][1] );
			break;
		case 'zyx':
			theta_x = Math.atan2( xform[2][1], xform[2][2] );
			theta_y = Math.asin( -xform[2][0] );
			theta_z = Math.atan2( xform[1][0], xform[0][0] );
			break;
	}


	return [theta_x, theta_y, theta_z];
}





function matrix_jac_layer(top, bottom) {

	mat = [];
	for (var i=0; i<3; i++) mat[i] = top[i]; 
	for (var i=3; i<6; i++) mat[i] = bottom[i-3];

	return mat;
}

function matrix_jac_stack(newJ, J) {

	mat = [];
	for (var i=0; i<newJ.length; i++) {
		mat[i] = [];
		mat[i][0] = newJ[i][0];
		for (var j=0; j<J[0].length; j++) {
			mat[i][j+1] = J[i][j];
		}
	}

	return mat;
}


