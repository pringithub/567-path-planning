//////////////////////////////////////////////////
/////     MATRIX ALGEBRA AND GEOMETRIC TRANSFORMS 
//////////////////////////////////////////////////

function matrix_copy(m1) {
    // returns 2D array that is a copy of m1

    var mat = [];
    var i,j;

    for (i=0;i<m1.length;i++) { // for each row of m1
        mat[i] = [];
        for (j=0;j<m1[0].length;j++) { // for each column of m1
            mat[i][j] = m1[i][j];
        }
    }
    return mat;
}


    // STENCIL: reference matrix code has the following functions:
    //   ./ matrix_multiply
    //   ./ matrix_transpose
    //   matrix_pseudoinverse
    //   matrix_invert_affine (need det and trace first :))
    //   ./ vector_normalize
    //   ./ vector_cross
    //   ./ generate_identity 
    //   ./ generate_translation_matrix
    //   ./ generate_rotation_matrix_X
    //   ./ generate_rotation_matrix_Y
    //   ./ generate_rotation_matrix_Z

//TODO: Test invert/det/trace

function create_empty_matrix(rows,cols) {
	mat = [];
	for (var i=0; i<rows; i++) {
		mat[i] = [];
		for (var j=0; j<cols; j++) {
			mat[i][j] = 0;
		}
	}

	return mat;
}


/***********************************************************************
  __  __       _        _        __  __      _   _               _     
 |  \/  | __ _| |_ _ __(_)_  __ |  \/  | ___| |_| |__   ___   __| |___ 
 | |\/| |/ _` | __| '__| \ \/ / | |\/| |/ _ \ __| '_ \ / _ \ / _` / __|
 | |  | | (_| | |_| |  | |>  <  | |  | |  __/ |_| | | | (_) | (_| \__ \
 |_|  |_|\__,_|\__|_|  |_/_/\_\ |_|  |_|\___|\__|_| |_|\___/ \__,_|___/
************************************************************************/

function matrix_multiply(A, B) {
	// make sure arrays are mxn dimensional, n>0
	if (A.length==0 || B.length==0 || A[0].length==0 || B[0].length==0)
		return "matrices not mxn dimensional";

	// make sure multiplication is possible
	if (A[0].length != B.length)
		return "matrices not mxn, nxp";;

	var mat = create_empty_matrix(A.length,B[0].length);
	var val = 0;
	for (var newrow=0; newrow<A.length; newrow++) {
		for (var newcol=0; newcol<B[0].length; newcol++) {
			//multiply row and col
			for (var i=0; i<A.length; i++) 
				val += A[newrow][i]*B[i][newcol];	

			mat[newrow][newcol] = val;
			val=0; //save space :)
		}
	}

	return mat;
}
function matrix_transpose(A) {
	// make sure arrays are mxn dimensional, n>0
	if (A.length==0 || A[0].length==0)
		return "matrices not mxn dimensional";

	var mat = create_empty_matrix(A[0].length,A.length);
	for (var i=0; i<A.length; i++) {
		for (var j=0; j<A[0].length; j++) {
			mat[j][i] = A[i][j];
		}
	}

	return mat;
}


static function matrix_trace(A) {
	// make sure arrays are nxn dimensional, n>0
	if (A.length==0 || A[0].length==0 || A.length!=A[0].length)
		return "matrices not nxn dimensional";

	var val=0;
	for (var i=0; i<A.length(); i++) {
		val += A[i][i];
	}

 	return val;	
}
static function matrix_determinant(A) {
	// make sure arrays are nxn dimensional, n>0
	if (A.length==0 || A[0].length==0 || A.length!=A[0].length)
		return "matrices not nxn dimensional";

	a = A[0][0]; b = A[0][1]; c = A[0][2]; d = A[0][3];
	e = A[1][0]; f = A[1][1]; g = A[1][2]; h = A[1][3];
	i = A[2][0]; j = A[2][1]; k = A[2][2]; l = A[2][3];
	m = A[3][0]; n = A[3][1]; o = A[3][2]; p = A[3][3];

	// could make this elegang and recursive .. oh well
	first_det  = f*(k*p-l*o) - g*(j*p-l*n) + h*(j*o-k*n);
	second_det = e*(k*p-l*o) - g*(i*p-l*m) + h*(i*o-k*m);
	third_det  = e*(j*p-l*n) - f*(i*p-l*m) + h*(i*n-j*m);
	fourth_det = e*(j*o-k*n) - f*(i*o-k*m) + g*(i*n-j*m);
	det = a*first_det - b*second_det + c*third_det - d*fourth_det;

	return det;
}
function matrix_invert_affine(A) {
	// make sure arrays are nxn dimensional, n>0
	if (A.length==0 || A[0].length==0 || A.length!=A[0].length)
		return "matrices not nxn dimensional";

	Asquared = matrix_multiply(A,A);
	Acubed   = matrix_multiply(Asquared,A);

	trace_A        = matrix_trace(A);
	trace_Asquared = matrix_trace(Asquared); 
	trace_Acubed   = matrix_trace(Acubed);

	first_term = (1/6) * (Math.pow(trace_A,3) - 
						  3*trace_A*trace_Asquared + 
						  2*trace_Acubed) * 
						 generate_identity(); 
	second_term = (1/2)*A*(Math.pow(trace_A,2) - trace_Asquared);
	third_term = Asquared*trace_A;
	fourth_term = Acubed;
	inverse = (1/matrix_determinant(A)) *
		(first_term - second_term + third_term - fourth_term);

	return inverse;
}




/***********************************************************************
 __     __        _               __  __      _   _               _     
 \ \   / /__  ___| |_ ___  _ __  |  \/  | ___| |_| |__   ___   __| |___ 
  \ \ / / _ \/ __| __/ _ \| '__| | |\/| |/ _ \ __| '_ \ / _ \ / _` / __|
   \ V /  __/ (__| || (_) | |    | |  | |  __/ |_| | | | (_) | (_| \__ \
    \_/ \___|\___|\__\___/|_|    |_|  |_|\___|\__|_| |_|\___/ \__,_|___/
************************************************************************/

// done assuming 3D homogenous vectors
function vector_normalize(u) {
	var norm = Math.sqrt(u[0]*u[0] + u[1]*u[1] + u[2]*u[2]);
	u[0] /= norm;
	u[1] /= norm;
	u[2] /= norm;

	return u;
}
function vector_cross(u,v) {
	return [ u[1]*v[2]-u[2]*v[1],
			 u[2]*v[0]-u[0]*v[2],
			 u[0]*v[1]-u[1]*v[0] ];
}


/***********************************************************************
  __  __       _        _         ____                           _   _             
 |  \/  | __ _| |_ _ __(_)_  __  / ___| ___ _ __   ___ _ __ __ _| |_(_) ___  _ __  
 | |\/| |/ _` | __| '__| \ \/ / | |  _ / _ \ '_ \ / _ \ '__/ _` | __| |/ _ \| '_ \ 
 | |  | | (_| | |_| |  | |>  <  | |_| |  __/ | | |  __/ | | (_| | |_| | (_) | | | |
 |_|  |_|\__,_|\__|_|  |_/_/\_\  \____|\___|_| |_|\___|_|  \__,_|\__|_|\___/|_| |_|
************************************************************************/

// may only need for 3D - could simplify to make easier
function generate_identity() {
	// only for 3D
	return [
			[1, 0, 0, 0],
			[0, 1, 0, 0],
			[0, 0, 1, 0],
			[0, 0, 0, 1] ];
}
function generate_translation_matrix(tx,ty,tz) {
	// only for 3D
	return [
			[1, 0, 0, tx],
			[0, 1, 0, ty],
			[0, 0, 1, tz],
			[0, 0, 0, 1] ];
}
function generate_rotation_matrix_X(theta) {
	return [
			[1, 0, 0, 0],
			[0, Math.cos(theta), -Math.sin(theta), 0],
			[0, Math.sin(theta), Math.cos(theta), 0],
			[0, 0, 0, 1] ];
}
function generate_rotation_matrix_Y(theta) {
	return [
			[Math.cos(theta), 0, Math.sin(theta), 0],
			[0, 1, 0, 0],
			[-Math.sin(theta), 0, Math.cos(theta), 0],
			[0, 0, 0, 1] ];
}
function generate_rotation_matrix_Z(theta) {
	return [
			[Math.cos(theta), -Math.sin(theta), 0, 0],
			[Math.sin(theta), Math.cos(theta), 0, 0],
			[0, 0, 1, 0],
			[0, 0, 0, 1] ];
}



