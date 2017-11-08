
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

    // STENCIL: see instructor for random time trial code
}


// endeffector_target_world:
// 		an object with two fields with the target endeffector position 
// 		(as a 3D homogeneous vector) and orientation (as Euler angles) in the world frame
// endeffector_joint:
// 	    the name of the joint directly connected to the endeffector
// endeffector_position_local:
// 	    the location of the endeffector in the local joint frame	
kineval.iterateIK = function iterate_inverse_kinematics(endeffector_target_world, endeffector_joint, endeffector_position_local) {

    // STENCIL: implement inverse kinematics iteration

	// compute endpoint error
	//   delta_xn = xd - xn
	xd = endeffector_target_world.position;
	xn = endeffector_position_local;
	delta_xn = matrix_subtract( xd,xn );


	// compute step direction
	//   delta_qn = inv( J(qn) ) * delta_xn
	J = getJacobian( endeffector_joint );

	version = 'inverse';
	if (version == 'inverse') {
		delta_qn = matrix_inverse( J ) * delta_xn;
	}
	else if (version == 'transpose') {
		delta_qn = matrix_transpose( J ) * delta_xn;
	}


	// perform step direction
	//   qn+1 = qn + gam*delta_qn
	gamma = 1.3; // or something ?
	qn = robot.joints[endeffector_joint].origin.xyz; // ?????
	qn_plus1 =  qn + gamma*delta_qn; 

	return qn_plus1; // ???????
}


function getJacobian(endeffector_target_world) { 
	J = [];
	J[0] = [];
	
	cur_joint = endeffector_target_world;
	while (robot.joints[cur_joint].parent != "") {
		z_iminus1 = robot.joints[cur_joint].parent.origin.axis;

		if (robot.joints[cur_joint].type == 'rotational') {
			cur2parent_dist = matrix_subtract( robot.joints[cur_joint].origin.xyz, 
										   robot.joints[cur_joint].parent.origin.xyz );

			jac_top = vector_cross( z_iminus1,cur2parent_dist );
			jac_bottom = z_iminus1;
		}
		else if (robot.joints[cur_joint].type == 'prismatic') {
			jac_top = ziminus1;
			jac_bottom = [[0],[0],[0]];
		}

		Ji = matrix_jac_layer( jac_top,jac_bottom );
		J = matrix_jac_stack( Ji, J );
	}

	return J;
}


function matrix_jac_layer(top, bottom) {

	mat = [];
	for (var i=0; i<3; i++) mat[i] = top[i]; 
	for (var i=3; i<6; i++) mat[i] = bottom[i];

	return mat;
}

function matrix_jac_stack(newJ, J) {

	mat = [];
	mat[0] = [];
	for (var i=0; i<6; i++) {
		mat[i][0] = newJ[i][0];
		for (var j=1; j<1+J[0].length; j++) {
			mat[i][j] = J[i][j];
		}
	}

	return mat;
}


