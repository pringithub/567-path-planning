
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | RRT motion planning

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

//////////////////////////////////////////////////
/////     RRT MOTION PLANNER
//////////////////////////////////////////////////

// STUDENT: 
// compute motion plan and output into robot_path array 
// elements of robot_path are vertices based on tree structure in tree_init() 
// motion planner assumes collision checking by kineval.poseIsCollision()

/* KE 2 : Notes:
   - Distance computation needs to consider modulo for joint angles
   - robot_path[] should be used as desireds for controls
   - Add visualization of configuration for current sample
*/

/*
STUDENT: reference code has functions for:

*/

kineval.planMotionRRTConnect = function motionPlanningRRTConnect() {

    // exit function if RRT is not implemented
    //   start by uncommenting kineval.robotRRTPlannerInit 
    if (typeof kineval.robotRRTPlannerInit === 'undefined') return;

    if ((kineval.params.update_motion_plan) && (!kineval.params.generating_motion_plan)) {
        kineval.robotRRTPlannerInit();
        kineval.params.generating_motion_plan = true;
        kineval.params.update_motion_plan = false;
        kineval.params.planner_state = "initializing";
    }
    if (kineval.params.generating_motion_plan) {
        rrt_result = robot_rrt_planner_iterate();
        if (rrt_result === "reached") {
            kineval.params.update_motion_plan = false; // KE T needed due to slight timing issue
            kineval.params.generating_motion_plan = false;
            textbar.innerHTML = "planner execution complete";
            kineval.params.planner_state = "complete";
        }
        else kineval.params.planner_state = "searching";
    }
    else if (kineval.params.update_motion_plan_traversal||kineval.params.persist_motion_plan_traversal) {

        if (kineval.params.persist_motion_plan_traversal) {
            kineval.motion_plan_traversal_index = (kineval.motion_plan_traversal_index+1)%kineval.motion_plan.length;
            textbar.innerHTML = "traversing planned motion trajectory";
        }
        else
            kineval.params.update_motion_plan_traversal = false;

        // set robot pose from entry in planned robot path
        robot.origin.xyz = [
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[0],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[1],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[2]
        ];

        robot.origin.rpy = [
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[3],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[4],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[5]
        ];

        // KE 2 : need to move q_names into a global parameter
        for (x in robot.joints) {
            robot.joints[x].angle = kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[q_names[x]];
        }

    }
}


    // STENCIL: uncomment and complete initialization function
kineval.robotRRTPlannerInit = function robot_rrt_planner_init() {

    // form configuration from base location and joint angles
    q_start_config = [
        robot.origin.xyz[0],
        robot.origin.xyz[1],
        robot.origin.xyz[2],
        robot.origin.rpy[0],
        robot.origin.rpy[1],
        robot.origin.rpy[2]
    ];

    q_names = {};  // store mapping between joint names and q DOFs
    q_index = [];  // store mapping between joint names and q DOFs

    for (x in robot.joints) {
        q_names[x] = q_start_config.length;
        q_index[q_start_config.length] = x;
        q_start_config = q_start_config.concat(robot.joints[x].angle);
    }

    // set goal configuration as the zero configuration
    var i; 
    q_goal_config = new Array(q_start_config.length);
    for (i=0;i<q_goal_config.length;i++) q_goal_config[i] = 0;

    // flag to continue rrt iterations
    rrt_iterate = true;
    rrt_iter_count = 0;


	// added by PHIL
    rrt_alg = "RRTConnect";
	T_a = tree_init(q_start_config);
	T_b = tree_init(q_goal_config);
	search_max_iterations = 2000;
	path_lock = 0;;

    // make sure the rrt iterations are not running faster than animation update
    cur_time = Date.now();
}


function robot_rrt_planner_iterate() {

	if (rrt_iterate && (Date.now()-cur_time > 10)) {
        cur_time = Date.now();

    // STENCIL: implement single rrt iteration here. an asynch timing mechanism 
    //   is used instead of a for loop to avoid blocking and non-responsiveness 
    //   in the browser.
    //
    //   once plan is found, highlight vertices of found path by:
    //     tree.vertices[i].vertex[j].geom.material.color = {r:1,g:0,b:0};
    //
    //   provided support functions:
    //
    //   kineval.poseIsCollision - returns if a configuration is in collision
    //   tree_init - creates a tree of configurations
    //   tree_add_vertex - adds and displays new configuration vertex for a tree
    //   tree_add_edge - adds and displays new tree edge between configurations
   

	// xyz/rpy
	ranges = [ [robot_boundary[0][0]-3, robot_boundary[1][0]+3],  
			   [robot_boundary[0][1], robot_boundary[1][1]], // [0,0]
			   [robot_boundary[0][2]-3, robot_boundary[1][2]+3], 
			   [0, 0],
			   [0, 0], 
			   [0, 0] ];
	// robot DOFs
	for (x in robot.joints) {
		if (typeof robot.joints[x].limit != 'undefined') 
			ranges.push( [robot.joints[x].limit.lower, robot.joints[x].limit.upper] );
		else {
			if (robot.joints[x].type != 'fixed')
				ranges.push( [-2*Math.PI, 2*Math.PI] );
			else  // ow it can't move
				ranges.push( [0,0] );
		}
	}
		
//		for (x in robot.joints) ranges.push([0,0]); 
		// TODO: change to joint limits ??
															 // I argue that since we are going to a goal state of 
		// 														zeros, there is no need to flail about wildly -
		// 														 this should be sufficient and I have not heard otherwise
		// 														as well, there is inherent respect to joint limits here,
		// 														given that we start in a valid configuration


	if (rrt_alg == "RRT") {
		q_rand = randomConfig( ranges );

		// result is [ 'word', q_new ]
		result = extendRRT( T_a, q_rand );

		if ( result[0] == 'advanced' ) {
			q_new = result[1];
			if ( euclideanDistance( q_new, q_goal_config ) < 0.5 ) {
				// draw path 
				closest_node = T_a.vertices[T_a.newest];
				findPath( T_a, closest_node, 'RRT' );
				return "reached";
			}
		}
		else if (rrt_iter_count == search_max_iterations)
			return "failed"; // TODO: change???
		else {
			rrt_iter_count++;
			return "extended";
		}
	}
    else if (rrt_alg == "RRTConnect") {
		q_rand = randomConfig( ranges );

		// result is [ 'word', q_new ]
		extendResult = extendRRT( T_a, q_rand );

		if ( extendResult[0] != 'trapped' ) {
			q_new = extendResult[1];
			connectResult = connectRRT( T_b, q_new );

			if ( connectResult[0] == 'advanced' ) {
				q_new = connectResult[1];
			}
			else if ( connectResult[0] == 'reached' ) {
			   findPath( T_a, T_a.vertices[T_a.newest], 'RRTConnect' );
			   findPath( T_b, T_b.vertices[T_b.newest], 'RRTConnect' );
			   path_lock++;	
			   return "reached";
			}
		}

		// swap trees
		var tmp = T_a;
		T_a = T_b;
		T_b = tmp;

	
		if (rrt_iter_count == search_max_iterations)
			return "failed"; // TODO: change???
		else {
			rrt_iter_count++;
			return "extended";
		}
	}

	else if (rrt_alg == "RRT*") {
		z_rand = randomConfig(ranges);
		
		z_nearest_idx = findNearestNeighborIdxInTree( T_a, z_rand );
		z_new = steer( T_a, z_nearest_idx, z_rand ); 	
		if (z_new != 'invalid') { // 'invalid' set in newConfig
			Z_near = nearestNeighbors( T_a, z_new );
			[z_min_idx,c_min] = chooseParent( T_a, Z_near, z_nearest_idx, z_new );

			
			tree_add_vertex( T_a, z_new );
			tree_add_edge( T_a, z_min_idx, T_a.newest );
//			insertTreeVertex( T_a, z_new );
//			insertTreeEdge( T_a, z_min_idx, T_a.newest ); // T.newest = z_new_idx

			// make best one parent
			T_a.vertices[T_a.newest].parent_idx = z_min_idx;
			// add current cost
			T_a.vertices[T_a.newest].cost = c_min;

			z_new_idx = T_a.newest;
			T_a = rewire( T_a, Z_near, z_min_idx, z_new_idx );
		}


		RRTStarOptimal = false;
		if (RRTStarOptimal) {
			if ( z_new!='invalid' && euclideanDistance( z_new, q_goal_config ) < 0.5 ) {
				z_final = z_new;
			}
			
			if (rrt_iter_count == search_max_iterations) {
				if (typeof z_final == 'undefined') 
					return "failed"; // TODO: change???
				else {
					findPath( T_a, T_a.vertices[z_final] );
					return "reached";
				}
			}
					
			return "extended";
		}
		else {
			if ( z_new!='invalid' && euclideanDistance( z_new, q_goal_config ) < 0.5 ) {
				findPath( T_a, T_a.vertices[T_a.newest] ); // T.newest is z_new	
				return "reached";
			}
			else if (rrt_iter_count == search_max_iterations)
				return "failed"; // TODO: change???
			else
				return "extended";
		}

	}



	} // if (rrt_iterate)	
	

}

//////////////////////////////////////////////////
/////     STENCIL SUPPORT FUNCTIONS
//////////////////////////////////////////////////

function tree_init(q) {

    // create tree object
    var tree = {};

    // initialize with vertex for given configuration
    tree.vertices = [];
    tree.vertices[0] = {};
    tree.vertices[0].vertex = q;
    tree.vertices[0].edges = [];

    // create rendering geometry for base location of vertex configuration
    add_config_origin_indicator_geom(tree.vertices[0]);

    // maintain index of newest vertex added to tree
    tree.newest = 0;

    return tree;
}

function tree_add_vertex(tree,q) {


    // create new vertex object for tree with given configuration and no edges
    var new_vertex = {};
    new_vertex.edges = [];
    new_vertex.vertex = q;

    // create rendering geometry for base location of vertex configuration
    add_config_origin_indicator_geom(new_vertex);

    // maintain index of newest vertex added to tree
    tree.vertices.push(new_vertex);
    tree.newest = tree.vertices.length - 1;
}

function add_config_origin_indicator_geom(vertex, _color=0xffff00) {

    // create a threejs rendering geometry for the base location of a configuration
    // assumes base origin location for configuration is first 3 elements 
    // assumes vertex is from tree and includes vertex field with configuration

    temp_geom = new THREE.CubeGeometry(0.1,0.1,0.1);
    temp_material = new THREE.MeshLambertMaterial( { color: _color, transparent: true, opacity: 0.7 } );
    temp_mesh = new THREE.Mesh(temp_geom, temp_material);
    temp_mesh.position.x = vertex.vertex[0];
    temp_mesh.position.y = vertex.vertex[1];
    temp_mesh.position.z = vertex.vertex[2];
    scene.add(temp_mesh);
    vertex.geom = temp_mesh;
}


function tree_add_edge(tree,q1_idx,q2_idx) {

    // add edge to first vertex as pointer to second vertex
    tree.vertices[q1_idx].edges.push(tree.vertices[q2_idx]);

    // add edge to second vertex as pointer to first vertex
    tree.vertices[q2_idx].edges.push(tree.vertices[q1_idx]);

    // can draw edge here, but not doing so to save rendering computation
}
function tree_remove_edge(tree,q1_idx,q2_idx) {

    // remove 1st edge
    for (var i=0; i<tree.vertices[q1_idx].edges.length; i++) {
        if (tree.vertices[q1_idx].edges[i] == q2_idx)
            tree.vertices[q1_idx].edges.splice(i,1);
    }

    // remove 2nd edge
    for (var i=0; i<tree.vertices[q2_idx].edges.length; i++) {
        if (tree.vertices[q2_idx].edges[i] == q1_idx)
            tree.vertices[q2_idx].edges.splice(i,1);
    }

    // can draw edge here, but not doing so to save rendering computation
}



//////////////////////////////////////////////////
/////     RRT IMPLEMENTATION FUNCTIONS
//////////////////////////////////////////////////



    // STENCIL: implement RRT-Connect functions here, such as:
    //   rrt_extend
    //   rrt_connect
    //   random_config
    //   new_config
    //   nearest_neighbor
    //   normalize_joint_state
    //   find_path
    //   path_dfs



global_step_length = 0.25;
global_neighbor_range = 2*global_step_length;

function extendRRT( T, q ) {
	q_near_idx = findNearestNeighborIdxInTree( T, q );
    q_near = T.vertices[q_near_idx].vertex;
    q_new = newConfig( q_near, q );
    if (q_new != 'invalid') {
		tree_add_vertex( T, q_new );
        tree_add_edge( T, q_near_idx, T.newest );

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
        if ( euclideanDistance( q_new, q_target ) < 0.7 )
            result = ['reached'];

    } while (result[0] == 'advanced');

    return result;
}




// now works for n-dimensions!
function euclideanDistance( q1, q2 ) { 
	vec = [];
	sum_squared = 0;
	for (var i=0; i<q1.length; i++) { 
		vec[i] = q1[i]-q2[i]; // subtract
		sum_squared += Math.pow(vec[i],2);
	}
	norm = Math.sqrt(sum_squared); 
	return norm;
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

function newConfig( q_start, q_target, step_length=global_step_length ) {

	norm = euclideanDistance( q_start, q_target );
	
	q_new = [];
	for (var i=0; i<q_start.length; i++) {
		normalized_elt = (q_target[i]-q_start[i])/norm;
		q_new[i] = step_length * normalized_elt + q_start[i];	
	}

	isCollision = kineval.poseIsCollision( q_new );
//	console.log(isCollision);
	if (isCollision != false) q_new = 'invalid';

	return q_new;
}

function findNearestNeighborIdxInTree( T, q ) {

	minDistanceToRandNode = Infinity;
	for (var i=0; i<T.vertices.length; i++) {
		distanceToRandNode = euclideanDistance( T.vertices[i].vertex, q );
		if (distanceToRandNode < minDistanceToRandNode) {
			minDistanceToRandNode = distanceToRandNode;
			closest_idx = i;
		}
	}

	return closest_idx; 
}

function findPath( T, node, planner='RRT' ) {
	if (path_lock != 0) return; // here for dumb browser bullshit

	arr = [];
	do {
		arr.unshift( node ); // prepend
		add_config_origin_indicator_geom(node,0x0000ff);
		node = T.vertices[node.parent_idx];
	} while (typeof node != 'undefined');
	

	if (planner == 'RRT' || planner == 'RRT*') {
		// add in goal
		tree_add_vertex( T, q_goal_config );
		add_config_origin_indicator_geom(T.vertices[T.newest],0x0000ff);
		arr.push( T.vertices[T.newest] );

		kineval.motion_plan = arr;
	}
	else if (planner == 'RRTConnect') { 

		var goalTree=false;
		for (var i=0; i<T.vertices.length; i++) {
			if (T.vertices[i].vertex == q_goal_config) {
				goalTree = true;
				break;
			}
		}

		if (typeof waiter == 'undefined') {
			waiter = arr;
		}
		else {
			if (goalTree) {
				for (var i=0; i<waiter.length; i++) kineval.motion_plan.push( waiter[i] ); // waiter is start tree
				for (var i=0; i<arr.length; i++) kineval.motion_plan.push( arr[arr.length-1-i] );
			}
			else { 
				for (var i=0; i<arr.length; i++) kineval.motion_plan.push( arr[i] );
				for (var i=0; i<waiter.length; i++) kineval.motion_plan.push( waiter[waiter.length-1-i] );
			}	
		}

	} // RRTConnect


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
//	removeTreeEdge( T, T.vertices[z_near].parent_idx, z_near );
	tree_remove_edge( T, T.vertices[z_near].parent_idx, z_near );	

	// change parent
	T.vertices[z_near].parent_idx = z_new;

	// add new edge
//	insertTreeEdge( T, z_new, z_near ); 
	tree_add_edge( T, z_new, z_near );

	return T;
}

function steer( T, q_near_idx, q_rand ) {

	q_near = T.vertices[q_near_idx].vertex;
	q_new = newConfig( q_near, q_rand );

	return q_new;
}



