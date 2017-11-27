/*
function iterateRRT() {

    // STENCIL: implement a single iteration of an RRT algorithm.
    //   An asynch timing mechanism is used instead of a for loop to avoid 
    //   blocking and non-responsiveness in the browser.
    //
    //   Return "failed" if the search fails on this iteration.
    //   Return "succeeded" if the search succeeds on this iteration.
    //   Return "extended" otherwise.
    //
    //   Provided support functions:
    //
    //   testCollision - returns whether a given configuration is in collision
    //   tree_init - creates a tree of configurations
    //   insertTreeVertex - adds and displays new configuration vertex for a tree
    //   insertTreeEdge - adds and displays new tree edge between configurations
    //   drawHighlightedPath - renders a highlighted path in a tree

	q_rand = randomConfig();
	result = extendRRT( q_rand );
	
	return result;

}

function iterateRRTConnect() {


    // STENCIL: implement a single iteration of an RRT-Connect algorithm.
    //   An asynch timing mechanism is used instead of a for loop to avoid 
    //   blocking and non-responsiveness in the browser.
    //
    //   Return "failed" if the search fails on this iteration.
    //   Return "succeeded" if the search succeeds on this iteration.
    //   Return "extended" otherwise.
    //
    //   Provided support functions:
    //
    //   testCollision - returns whether a given configuration is in collision
    //   tree_init - creates a tree of configurations
    //   insertTreeVertex - adds and displays new configuration vertex for a tree
    //   insertTreeEdge - adds and displays new tree edge between configurations
    //   drawHighlightedPath - renders a highlighted path in a tree
}

function iterateRRTStar() {

}
*/



//////////////////////////////////////////////////
/////     RRT IMPLEMENTATION FUNCTIONS
//////////////////////////////////////////////////

    // STENCIL: implement RRT-Connect functions here, such as:
    //   extendRRT
    //   connectRRT
    //   randomConfig
    //   newConfig
    //   findNearestNeighbor
    //   dfsPath


// will have visited_queue to make nn easier

// Takes in an a point array
function euclideanDistance(a,b) {
   return Math.sqrt(Math.pow(a[0]-b[0],2)+Math.pow(a[1]-b[1],2));
}


function extendRRT( T, q ) {

	q_near_idx = findNearestNeighborIdxInTree( T, q );
	q_near = T.vertices[q_near_idx].vertex;
	q_new = newConfig( q, q_near );
	if (q_new != 'invalid') {
		insertTreeVertex( T, q_new );
		insertTreeEdge( T, q_near_idx, T.newest );

		if ( Math.abs(q_new[0]-q_goal[0])<(eps) && Math.abs(q_new[1]-q_goal[1])<(eps) ) 
			result = 'reached'; // to reach goal; reaching other tree is different
		else 
			result = 'advanced';
	}
	else 
		result = 'trapped';

	return [ result, q_new ];
}

// why does this exist ???
function extendRRT_4CONNECT( T, q ) {

	q_near_idx = findNearestNeighborIdxInTree( T, q );
	q_near = T.vertices[q_near_idx].vertex;
	q_new = newConfig( q, q_near );
	if (q_new != 'invalid') {
		insertTreeVertex( T, q_new );
		insertTreeEdge( T, q_near_idx, T.newest );

		if ( Math.abs(q_new[0]-q[0])<(eps) && Math.abs(q_new[1]-q[1])<(eps) ) 
			result = 'reached'; // to reach goal; reaching other tree is different
		else 
			result = 'advanced';
	}
	else 
		result = 'trapped';

	return [ result, q_new ];
}

function connectRRT_4CONNECT( T, q_new ) {

	do {
		result = extendRRT( T, q_new );

	} while (result[0] == 'advanced');

	return result;
}


// input args 2=elt arrays of min/max
function randomConfig(minmaxX,minmaxY) {
	rangeX = minmaxX[1]-minmaxX[0];
	rangeY = minmaxY[1]=minmaxY[0];
	return [(Math.random() * rangeX) + minmaxX[0], 
		    (Math.random() * rangeY) + minmaxY[0]];

	//	return [(Math.random() * 6) - 1, (Math.random() * 6) - 1]; // G is from [-2,7]
}

// q must be an array of [ x_loc, y_loc ]
function findNearestNeighborIdxInTree( T_a, q ) {

	minDistanceToRandNode = Infinity;
	for (var i=0; i<T_a.vertices.length; i++) {
		distanceToRandNode = euclideanDistance( T_a.vertices[i].vertex, q );
		if (distanceToRandNode < minDistanceToRandNode) {
			minDistanceToRandNode = distanceToRandNode;
			closest_idx = i;
		}
	}

	return closest_idx; 
}

function newConfig( q_rand, q_near ) {

	// get angle wrt +x axis
	q_inter = [q_rand[0], q_near[1]];
	delta_rise = q_inter[1]-q_rand[1]; // keep cartesian coords even though canvas doesn't respect it
	delta_run = q_inter[0]-q_near[0];
	angle = Math.atan2(delta_rise,delta_run);

	step_length = 0.1;
	e0 = [ step_length, 0 ];
	rotated = rotateVector2D( e0, angle );
	q_new = [ q_near[0]+rotated[0], q_near[1]-rotated[1] ]; 

	near2new = euclideanDistance(q_near, q_new);	
	if (near2new != step_length) {
//		throw "fuck";
	}

	if (testCollision(q_new)) {
		q_new = 'invalid';
	}

	return q_new;
}

function rotateVector2D(v, theta) {
	// [ [cos(theta), -sin(theta)],
	//   [sin(theta), cos(theta) ];
	
	return [ Math.cos(theta)*v[0] - Math.sin(theta)*v[1],
			 Math.sin(theta)*v[0] + Math.cos(theta)*v[1] ];
}


