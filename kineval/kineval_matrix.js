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
    //   ./ matrix_pseudoinverse
    //   ./ matrix_invert_affine (need det and trace first :))
    //   ./ vector_normalize
    //   ./ vector_cross
    //   ./ generate_identity 
    //   ./ generate_translation_matrix
    //   ./ generate_rotation_matrix_X
    //   ./ generate_rotation_matrix_Y
    //   ./ generate_rotation_matrix_Z


/***********************************************************************
  __  __       _        _        __  __      _   _               _     
 |  \/  | __ _| |_ _ __(_)_  __ |  \/  | ___| |_| |__   ___   __| |___ 
 | |\/| |/ _` | __| '__| \ \/ / | |\/| |/ _ \ __| '_ \ / _ \ / _` / __|
 | |  | | (_| | |_| |  | |>  <  | |  | |  __/ |_| | | | (_) | (_| \__ \
 |_|  |_|\__,_|\__|_|  |_/_/\_\ |_|  |_|\___|\__|_| |_|\___/ \__,_|___/
************************************************************************/

/* NOT TESTED - used for SVD in pseudoinverse */
function generate_diagonal_matrix_from_array(array) {
	mat = [];
	mat[0] = [];
	for (var i=0; i<array.length; i++) {
		mat[i][i] = array[i];	
	}

	return mat;
}



function generate_column_matrix_from_array(array) {
	mat = [];
	for (var i=0; i<array.length; i++) {
		mat[i] = [array[i]];
	}

	return mat;
}
function generate_row_matrix_from_array(array) {
	mat = [];
	mat[0] = [];
	for (var i=0; i<array.length; i++) {
		mat[0][i] = array[i]; 
	}

	return mat;
}
// layers row after row 
function mat_2_array(mat) {
	if (mat.length==0 || mat[0].length==0) {
		throw "arg isn't a matrix, dummy!";
		exit("arg isn't a matrix, dummy!");
	}

	arr = [];
	arrpos = 0;
	for (var i=0; i<mat.length; i++) {
		for (var j=0; j<mat[0].length; j++, arrpos++) {
			arr[arrpos] = mat[i][j];
		}
	}

	return arr;
}

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

function matrix_addsub(A, B, isAdd) {
	// make sure arrays are mxn dimensional, n>0
	if (A.length==0 || B.length==0 || A[0].length==0 || B[0].length==0)
		return "A and/or B not matrices";
	if ( (A.length != B.length) || (A[0].length != B[0].length) )
		return "matrices must be same size";

	mat = [];
	mat[0] = [];

	for (var i=0; i<A.length; i++) {
		mat[i] = [];
		for (var j=0; j<A[0].length; j++) {
			if      (isAdd == '+') mat[i][j] = A[i][j]+B[i][j];
			else if (isAdd == '-') mat[i][j] = A[i][j]-B[i][j]; 
		}
	}

	return mat;	
}


function scalar_multiply_matrix(s, mat) {

	newmat = mat;

	for (var i=0; i<mat.length; i++) {
		for (var j=0; j<mat[0].length; j++) {
			newmat[i][j] = s*mat[i][j];
		}
	}

	return newmat;
}



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
			for (var i=0; i<A[0].length; i++) 
				val += A[newrow][i]*B[i][newcol];	

			mat[newrow][newcol] = val;
			val=0; //save space :)
		}
	}

	return mat;
}
// performs the operation A*B*C for multiplicable matrices A,B,C
function matrix_multiply_3(A,B,C) {
	return matrix_multiply(A, matrix_multiply(B,C));
}
function matrix_transpose(A) {
	// make sure arrays are mxn dimensional, n>0
	if (A.length==0 || A[0].length==0)
		return "A is not a matrix";

	var mat = create_empty_matrix(A[0].length,A.length);
	for (var i=0; i<A.length; i++) {
		for (var j=0; j<A[0].length; j++) {
			mat[j][i] = A[i][j];
		}
	}

	return mat;
}

function matrix_pseudoinverse(A) {
	// matrix A with dims nxm: use
	//   left pseudoinverse for when N>M
	//   right pseudoinverse fo when N<M

	if (A.length==0 || A[0].length==0)
		return "A is not a matrix";

	Atrans = matrix_transpose(A);

	// use left pinv
	if (A.length >= A[0].length) {
		Atrans_A = matrix_multiply( Atrans, A );
		Atrans_A__inv = matrix_invert_affine( Atrans_A );
		Aplus = matrix_multiply( Atrans_A__inv, Atrans ); 
	}
	// use right pinv
	else {
		A_Atrans = matrix_multiply( A, Atrans );
		A_Atrans__inv = matrix_invert_affine( A_Atrans );
		Aplus = matrix_multiply( Atrans, A_Atrans__inv );
	}

	/* NOT TESTED
	// pseudoinverse by SVD, just in case
	out = numeric.svd(A);
	U = out["U"];
	D = out["S"];
	V = out["V"];

	// Dplus is reciprocal of D 
	Dplus = [];
	for (var i=0; i<D.length; i++) { Dplus[i] = 1.0/D[i]; };
	Dplus = generate_diagonal_matrix_from_array(Dplus);

	Aplus = V*Dplus*matrix_transpose(U);
	*/

	return Aplus;
}




function matrix_invert_affine(A) {
	return numeric.inv(A);
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
	return [
			[1, 0, 0, 0],
			[0, 1, 0, 0],
			[0, 0, 1, 0],
			[0, 0, 0, 1] ];

}
function generate_translation_matrix(t) {
	// only for 3D
	return [
			[1, 0, 0, t[0]],
			[0, 1, 0, t[1]],
			[0, 0, 1, t[2]],
			[0, 0, 0, 1] ];
}
function generate_rotation_matrix_ZYX(thetas) { // should be much easier now

	rotZ = generate_rotation_matrix_Z(thetas[2]);
	rotY = generate_rotation_matrix_Y(thetas[1]);
	rotX = generate_rotation_matrix_X(thetas[0]);

	inter = matrix_multiply(rotY,rotX);
	end   = matrix_multiply(rotZ,inter);

	return end;

//	return matrix_multiply(generate_rotation_matrix_Z(thetas[2]),
//			matrix_multiply(generate_rotation_matrix_Y(thetas[1]),
//							generate_rotation_matrix_X(thetas[0])) );
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



