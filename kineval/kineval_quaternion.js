//////////////////////////////////////////////////
/////     QUATERNION TRANSFORM ROUTINES 
//////////////////////////////////////////////////

    // STENCIL: reference quaternion code has the following functions:
    //   quaternion_from_axisangle
    //   quaternion_normalize
    //   quaternion_to_rotation_matrix
    //   quaternion_multiply

// TODO: check these functions



// q = (cos(theta/2), omega*sin(theta/2))
// https://en.wikipedia.org/wiki/Axis–angle_representation#Unit_quaternions
function quaternion_from_axisangle(theta, axis) {

	t = Math.sin(theta/2);
	return [Math.cos(theta/2), axis[0]*t, axis[1]*t, axis[2]*t]; 

}

function quaternion_normalize(q) {
	return Math.sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
}

function quaternion_to_rotation_matrix(q) {

	d = q[0];
	a = q[1];
	b = q[2];
	c = q[3];

	rot = [ [d*d+a*a-b*b-c*c, 2*a*b-2*c*d, 2*a*c+2*b*d, 0],
		    [2*a*b+2*c*d, d*d-a*a+b*b-c*c, 2*b*c-2*a*d, 0],
		    [2*a*c-2*b*d, 2*b*c+2*a*d, d*d-a*a-b*b+c*c, 0],
		    [0, 0, 0, 1] ];

	return rot;
}

function quaternion_multiply(q,r) {

	t0 = (r[0]*q[0] - r[1]*q[1] - r[2]*q[2] - r[3]*q[3]);
	t1 = (r[0]*q[1] + r[1]*q[0] - r[2]*q[3] + r[3]*q[2]);
	t2 = (r[0]*q[2] + r[1]*q[3] + r[2]*q[0] - r[3]*q[1]);
	t3 = (r[0]*q[3] - r[1]*q[2] + r[2]*q[1] + r[3]*q[0]);

	return [t0, t1, t2, t3];
}





