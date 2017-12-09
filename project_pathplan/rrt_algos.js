
ranges = [ [-2,6.3],
		   [-2,6.3] ];

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

	q_rand = randomConfig(ranges);
	// draw q_rand for testing
    ctx.fillStyle = "#0088FF";
    ctx.fillRect(xformWorldViewX(q_rand[0])-1.5,xformWorldViewY(q_rand[1])-1.5,3,3);

	// result is [ 'word', q_new ]
	result = extendRRT( T_a, q_rand );
	
	if ( result[0] == 'advanced' ) {
		q_new = result[1];
		if ( Math.abs(q_new[0]-q_goal[0])<(eps) && Math.abs(q_new[1]-q_goal[1])<(eps) ) {
			findPath( T_a, q_new );	
			return "succeeded";
		}
    }
    else if (search_iter_count == search_max_iterations)
		return "failed"; // TODO: change???
    else
        return "extended";
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

	q_rand = randomConfig(ranges);
	// draw q_rand for testing
    ctx.fillStyle = "#0088FF";
    ctx.fillRect(xformWorldViewX(q_rand[0])-1.5,xformWorldViewY(q_rand[1])-1.5,3,3);
	
	// result is [ 'word', q_new ]
	extendResult = extendRRT( T_a, q_rand );

	if ( extendResult[0] != 'trapped' ) {
		q_new = extendResult[1];
		connectResult = connectRRT( T_b, q_new ); 

		if ( connectResult[0] == 'advanced' ) {
			q_new = connectResult[1];
		}
		else if ( connectResult[0] == 'reached' ) {
			findPath( T_a, q_new );
			findPath( T_b, q_new );
			return "succeeded";
		}
	}

	// swap trees
	var tmp = T_a;
	T_a = T_b;
	T_b = tmp;


	if (search_iter_count == search_max_iterations)
		return "failed"; // TODO: change???
    else
        return "extended";
}

function iterateRRTStar() {

	z_rand = randomConfig(ranges);
	// draw q_rand for testing
    ctx.fillStyle = "#0088FF";
    ctx.fillRect(xformWorldViewX(q_rand[0])-1.5,xformWorldViewY(q_rand[1])-1.5,3,3);
	
	z_nearest_idx = findNearestNeighborIdxInTree( T_a, z_rand );
	z_new = steer( T_a, z_nearest_idx, z_rand ); 	
	if (z_new != 'invalid') { // 'invalid' set in newConfig
		Z_near = nearestNeighbors( T_a, z_new );
		[z_min_idx,c_min] = chooseParent( T_a, Z_near, z_nearest_idx, z_new );

		insertTreeVertex( T_a, z_new );
		insertTreeEdge( T_a, z_min_idx, T_a.newest ); // T.newest = z_new_idx

		// make best one parent
		T_a.vertices[T_a.newest].parent_idx = z_min_idx;
		// add current cost
		T_a.vertices[T_a.newest].cost = c_min;

		z_new_idx = T_a.newest;
		T_a = rewire( T_a, Z_near, z_min_idx, z_new_idx );
	}


	RRTStarOptimal = true;
	if (RRTStarOptimal) {
		if ( z_new!='invalid' && euclideanDistance( z_new, q_goal ) < 0.2 ) {
			z_final = z_new;
		}
		
	    if (search_iter_count == search_max_iterations) {
			if (typeof z_final == 'undefined') 
				return "failed"; // TODO: change???
			else {
				findPath( T_a, z_final );
				return "succeeded";
			}
		}
				
		return "extended";
	}
	else {
		if ( z_new!='invalid' && euclideanDistance( z_new, q_goal ) < 0.2 ) {
			findPath( T_a, z_new );	
			return "succeeded";
		}
		else if (search_iter_count == search_max_iterations)
			return "failed"; // TODO: change???
		else
			return "extended";
	}
}



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



global_step_length=0.1;
global_neighbor_range=global_step_length*2;

// Takes in an a point array
function euclideanDistance(a,b) {
   return Math.sqrt(Math.pow(a[0]-b[0],2)+Math.pow(a[1]-b[1],2));
}


function extendRRT( T, q ) {

	q_near_idx = findNearestNeighborIdxInTree( T, q );
	q_near = T.vertices[q_near_idx].vertex;
	q_new = newConfig( q_near, q );
	if (q_new != 'invalid') {
		insertTreeVertex( T, q_new );
		insertTreeEdge( T, q_near_idx, T.newest );

		T.vertices[T.newest].parent_idx = q_near_idx;

		result = 'advanced';
	}
	else 
		result = 'trapped';

	return [ result, q_new ];
}

function connectRRT( T, q_target ) {

	do {
		result = extendRRT( T, q_target );

		q_new = result[1];
		if ( Math.abs(q_new[0]-q_target[0])<(eps) && Math.abs(q_new[1]-q_target[1])<(eps) ) 
			result = ['reached']; 

	} while (result[0] == 'advanced');

	return result;
}


function randomConfig( ranges ) {

	// Input: ranges ( nx2 matrix )
	// Output: new nx1 config
	//

	q_rand = [];
	for (var i=0; i<ranges.length; i++) {
		q_rand[i] = (Math.random()*(ranges[i][1]-ranges[i][0])+ranges[i][0]);
	}

	return q_rand;
}


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

function newConfig( q_start, q_target, step_length=global_step_length ) {

	norm = euclideanDistance( q_start, q_target );
	
	q_new = [];
	for (var i=0; i<q_start.length; i++) {
		normalized_elt = (q_target[i]-q_start[i])/norm;
		q_new[i] = step_length * normalized_elt + q_start[i];	
	}

	isCollision = testCollision( q_new );
	if (isCollision != false) q_new = 'invalid';

	return q_new;
}

function findPath( T, q ) {

	for (var i=0; i<T.vertices.length; i++) {
		if (T.vertices[i].vertex == q) {
			node = T.vertices[i];
			break;
		}
	}
	//node = T.vertices[T.newest];
	
	while ( typeof node.parent_idx != 'undefined' ) {
		draw_2D_configuration( node.vertex, "#FFFF00" );
		node = T.vertices[node.parent_idx];
    }

	// costs only implemented for RRT* currently
	if (typeof node.cost != 'undefined') {
		path_length = node.cost;
//		return node.cost; //path length
	}
}





//////////////
//  For RRT* 
//////////////

function nearestNeighbors( T, q_new, neighbor_range=global_neighbor_range ) {
	
	nearestIdxs = [];
	for (var i=0; i<T.vertices.length; i++) {
		if ( euclideanDistance( T.vertices[i].vertex, q_new ) < neighbor_range ) {
			nearestIdxs.push(i);
		}
	}

	return nearestIdxs;
}


// choose the nearest neighbor that minimizes the (neighbor's cost + nbr2new cost) heuristic
function chooseParent( T, Z_near, z_nearest_idx, z_new, x_new ) {
	
	z_min = z_nearest_idx;
	c_min = T.vertices[z_nearest_idx].cost + 
		euclideanDistance( T.vertices[z_nearest_idx].vertex, z_new );

	for (var i=0; i<Z_near.length; i++) {
	
		// TODO: make sure no obstacles yet

		c_prime = T.vertices[Z_near[i]].cost + 
			euclideanDistance( T.vertices[Z_near[i]].vertex, z_new );

		if (c_prime < c_min) {
			z_min = Z_near[i];
			c_min = c_prime;
		}
	}

	return [z_min, c_min];
}

function rewire(T, Z_near, z_min_idx, z_new_idx) {
	
	// see if it is less costly to go from new to near; if so, rewire
	for (var i=0; i<Z_near.length; i++) {
	if (Z_near[i] != z_min_idx) {
		var z_new_cost = T.vertices[z_new_idx].cost;
		var z_new_2_z_near = euclideanDistance( T.vertices[z_new_idx].vertex, 
												T.vertices[Z_near[i]].vertex);
		var z_near_cost = T.vertices[Z_near[i]].cost; // current cost of the near / neighbor
		
		// make sure the path is clear
		var pathCollision=false;
		// below condition will always be 0 or 1; may need to change to be robust
		for (var j=0; j<Math.floor(z_new_2_z_near*(1/global_step_length)); j++) { 			
			z_test = steer( T, z_new_idx, T.vertices[Z_near[i]].vertex );
			if (z_test == 'invalid') pathCollision = true;	
		}
		
		if ( !pathCollision && ((z_new_cost + z_new_2_z_near) < z_near_cost) ) 
			T = reconnect(T,Z_near[i],z_new_idx);
	}
	}

	return T;
}

function reconnect( T, z_near, z_new ) {

	// remove edge between near and near's parent
	removeTreeEdge( T, T.vertices[z_near].parent_idx, z_near );

	// change parent
	T.vertices[z_near].parent_idx = z_new;

	// add new edge
	insertTreeEdge( T, z_new, z_near ); 

	return T;
}

function steer( T, q_near_idx, q_rand ) {

	q_near = T.vertices[q_near_idx].vertex;
	q_new = newConfig( q_near, q_rand );

	return q_new;
}



