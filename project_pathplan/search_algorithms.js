
// Takes in an a point array
function euclideanDistance(a,b) {
	return Math.sqrt(Math.pow(a[0]-b[0],2)+Math.pow(a[1]-b[1],2));
}

function getNeighbors(current_node) {

	// get adjacent neighbors
	neighbors = [];
	if (current_node.i >= 0 && current_node.i < G.length) {
		if (current_node.j+1 < G[0].length) neighbors.push(G[current_node.i][current_node.j+1]);
		if (current_node.j-1 >= 0) neighbors.push(G[current_node.i][current_node.j-1]);
	} 
	if (current_node.j >= 0 && current_node.j < G[0].length) {
		if (current_node.i+1 < G.length) neighbors.push(G[current_node.i+1][current_node.j]);
		if (current_node.i-1 >= 0) neighbors.push(G[current_node.i-1][current_node.j]); 
	}	
	
	return neighbors;
}


function bfs() {

	/**************************************************************************
			$$$$$$$\  $$$$$$$$\  $$$$$$\  
			$$  __$$\ $$  _____|$$  __$$\ 
			$$ |  $$ |$$ |      $$ /  \__|
			$$$$$$$\ |$$$$$\    \$$$$$$\  
			$$  __$$\ $$  __|    \____$$\ 
			$$ |  $$ |$$ |      $$\   $$ |
			$$$$$$$  |$$ |      \$$$$$$  |
			\_______/ \__|       \______/ 
    */                  

	current_node = visit_queue.shift()
	G[current_node.i][current_node.j].visited = true;
	draw_2D_configuration([current_node.x,current_node.y]);
	

	neighbors = getNeighbors(current_node);
	for (var i=0; i<neighbors.length; i++) {
		// filter for non-visited and valid nodes
		if (!neighbors[i].visited && !neighbors[i].queued && !testCollision([neighbors[i].x,neighbors[i].y])) {
			distanceCurToNbr = euclideanDistance([current_node.x,current_node.y],[neighbors[i].x,neighbors[i].y]); 
			if (neighbors[i].distance > current_node.distance + distanceCurToNbr) {
				neighbors[i].parent = current_node;
				neighbors[i].distance = distanceCurToNbr; 
			}
			
			visit_queue.push(neighbors[i]);
			neighbors[i].queued = true;
			drawQueuedNode(neighbors[i]);
		}
	}


	if ( Math.abs(current_node.x-q_goal[0])<(eps/1000) && Math.abs(current_node.y-q_goal[1])<(eps/1000) ) {
		drawHighlightedPathGraph(current_node);	
		return "succeeded"; // check on this
	}
	else if (visit_queue.length>0 && current_node!=q_goal)
		return "iterating";
	else (search_iter_count == search_max_iterations)  
		return "failed"; // is this possible?   //   draw_2D_configuration - draws a square at a given location 
}

function dfs() {

	/**************************************************************************
			$$$$$$$\  $$$$$$$$\  $$$$$$\  
			$$  __$$\ $$  _____|$$  __$$\ 
			$$ |  $$ |$$ |      $$ /  \__|
			$$ |  $$ |$$$$$\    \$$$$$$\  
			$$ |  $$ |$$  __|    \____$$\ 
			$$ |  $$ |$$ |      $$\   $$ |
			$$$$$$$  |$$ |      \$$$$$$  |
			\_______/ \__|       \______/ 
	*/										  
 
	current_node = visit_queue.pop()
	G[current_node.i][current_node.j].visited = true;
	draw_2D_configuration([current_node.x,current_node.y]);
	
		
	neighbors = getNeighbors(current_node);
	for (var i=0; i<neighbors.length; i++) {
		// filter for non-visited and valid nodes
		if (!neighbors[i].visited && !neighbors[i].queued && !testCollision([neighbors[i].x,neighbors[i].y])) {
			distanceCurToNbr = euclideanDistance([current_node.x,current_node.y],[neighbors[i].x,neighbors[i].y]); 
			if (neighbors[i].distance > current_node.distance + distanceCurToNbr) {
				neighbors[i].parent = current_node;
				neighbors[i].distance = distanceCurToNbr; 
			}
			
			visit_queue.push(neighbors[i]);
			neighbors[i].queued = true;
			drawQueuedNode(neighbors[i]);
		}
	}


	if ( Math.abs(current_node.x-q_goal[0])<(eps/1000) && Math.abs(current_node.y-q_goal[1])<(eps/1000) ) {
		drawHighlightedPathGraph(current_node);	
		return "succeeded"; // check on this
	}
	else if (visit_queue.length>0 && current_node!=q_goal)
		return "iterating";
	else (search_iter_count == search_max_iterations)  
		return "failed"; // is this possible?   //   draw_2D_configuration - draws a square at a given location 

    
}	

function greedyBestFirst() {

	/**************************************************************************
			 $$$$$$\                                      $$\                 $$$$$$$\                        $$\           $$$$$$$$\ $$\                       $$\     
			$$  __$$\                                     $$ |                $$  __$$\                       $$ |          $$  _____|\__|                      $$ |    
			$$ /  \__| $$$$$$\   $$$$$$\   $$$$$$\   $$$$$$$ |$$\   $$\       $$ |  $$ | $$$$$$\   $$$$$$$\ $$$$$$\         $$ |      $$\  $$$$$$\   $$$$$$$\ $$$$$$\   
			$$ |$$$$\ $$  __$$\ $$  __$$\ $$  __$$\ $$  __$$ |$$ |  $$ |      $$$$$$$\ |$$  __$$\ $$  _____|\_$$  _|        $$$$$\    $$ |$$  __$$\ $$  _____|\_$$  _|  
			$$ |\_$$ |$$ |  \__|$$$$$$$$ |$$$$$$$$ |$$ /  $$ |$$ |  $$ |      $$  __$$\ $$$$$$$$ |\$$$$$$\    $$ |          $$  __|   $$ |$$ |  \__|\$$$$$$\    $$ |    
			$$ |  $$ |$$ |      $$   ____|$$   ____|$$ |  $$ |$$ |  $$ |      $$ |  $$ |$$   ____| \____$$\   $$ |$$\       $$ |      $$ |$$ |       \____$$\   $$ |$$\ 
			\$$$$$$  |$$ |      \$$$$$$$\ \$$$$$$$\ \$$$$$$$ |\$$$$$$$ |      $$$$$$$  |\$$$$$$$\ $$$$$$$  |  \$$$$  |      $$ |      $$ |$$ |      $$$$$$$  |  \$$$$  |
			 \______/ \__|       \_______| \_______| \_______| \____$$ |      \_______/  \_______|\_______/    \____/       \__|      \__|\__|      \_______/    \____/ 
															  $$\   $$ |                                                                                                
															  \$$$$$$  |                                                                                                
															   \______/                                                                                                 
	*/

	// I'm pretty sure GBF is just A* without g_score (g_score = 0)

	current_node = minheap_extract(visit_queue); 
	G[current_node.i][current_node.j].visited = true;
	draw_2D_configuration([current_node.x,current_node.y]);
	
		
	neighbors = getNeighbors(current_node);
	for (var i=0; i<neighbors.length; i++) {
		// filter for non-visited and valid nodes
		if (!neighbors[i].visited && !neighbors[i].queued && !testCollision([neighbors[i].x,neighbors[i].y])) {
			distanceCurToNbr = euclideanDistance([current_node.x,current_node.y],[neighbors[i].x,neighbors[i].y]); 
			if (neighbors[i].distance > current_node.distance + distanceCurToNbr) {
				neighbors[i].parent = current_node;
				neighbors[i].distance = distanceCurToNbr; 
	
				h_score = euclideanDistance([neighbors[i].x,neighbors[i].y],q_goal);
				neighbors[i].priority = h_score; // f_score
			}
			
			minheap_insert(visit_queue, neighbors[i]);
			neighbors[i].queued = true;
			drawQueuedNode(neighbors[i]);
		}
	}


	if ( Math.abs(current_node.x-q_goal[0])<(eps/1000) && Math.abs(current_node.y-q_goal[1])<(eps/1000) ) {
		drawHighlightedPathGraph(current_node);	
		return "succeeded"; // check on this
	}
	else if (visit_queue.length>0 && current_node!=q_goal)
		return "iterating";
	else (search_iter_count == search_max_iterations)  
		return "failed"; // is this possible?   //   draw_2D_configuration - draws a square at a given location 



}






function dijkstras() {

	current_node = minheap_extract(visit_queue); 
	G[current_node.i][current_node.j].visited = true;
	draw_2D_configuration([current_node.x,current_node.y]);
	
		
	neighbors = getNeighbors(current_node);
	for (var i=0; i<neighbors.length; i++) {
		// filter for non-visited and valid nodes
		if (!neighbors[i].visited && !neighbors[i].queued && !testCollision([neighbors[i].x,neighbors[i].y])) {
			distanceCurToNbr = current_node.distance + eps; //incorrect? euclideanDistance([current_node.x,current_node.y],[neighbors[i].x,neighbors[i].y]); 
			if (neighbors[i].distance > current_node.distance + distanceCurToNbr) {
				neighbors[i].parent = current_node;
				neighbors[i].distance = distanceCurToNbr; 
	
				g_score = neighbors[i].distance;
				neighbors[i].priority = g_score; // f_score
			}
			
			minheap_insert(visit_queue, neighbors[i]);
			neighbors[i].queued = true;
			drawQueuedNode(neighbors[i]);
		}
	}


	if ( Math.abs(current_node.x-q_goal[0])<(eps/1000) && Math.abs(current_node.y-q_goal[1])<(eps/1000) ) {
		drawHighlightedPathGraph(current_node);	
		return "succeeded"; // check on this
	}
	else if (visit_queue.length>0 && current_node!=q_goal)
		return "iterating";
	else (search_iter_count == search_max_iterations)  
		return "failed"; // is this possible?   //   draw_2D_configuration - draws a square at a given location 
}









function iterateGraphSearch() {
    // STENCIL: implement a single iteration of a graph search algorithm
    //   for A-star (or DFS, BFS, Greedy Best-First)
    //   An asynch timing mechanism is used instead of a for loop to avoid 
    //   blocking and non-responsiveness in the browser.
    //
    //   Return "failed" if the search fails on this iteration.
    //   Return "succeeded" if the search succeeds on this iteration.
    //   Return "iterating" otherwise.
    //
    //   Provided support functions:
    //
    //   testCollision - returns whether a given configuration is in collision
    //   drawHighlightedPathGraph - draws a path back to the start location
 
	/**************************************************************************
			 $$$$$$\           
			$$  __$$\  $$\$$\  
			$$ /  $$ | \$$$  | 
			$$$$$$$$ |$$$$$$$\ 
			$$  __$$ |\_$$$ __|
			$$ |  $$ | $$ $$\  
			$$ |  $$ | \__\__| 
			\__|  \__|         
    */              

	current_node = minheap_extract(visit_queue); 
	G[current_node.i][current_node.j].visited = true;
	draw_2D_configuration([current_node.x,current_node.y]);

	
	neighbors = getNeighbors(current_node);
	for (var i=0; i<neighbors.length; i++) {
		// filter for non-visited and valid nodes
		if (!neighbors[i].visited && !neighbors[i].queued && !testCollision([neighbors[i].x,neighbors[i].y])) {
			distanceCurToNbr = current_node.distance + eps; //incorrect? euclideanDistance([current_node.x,current_node.y],[neighbors[i].x,neighbors[i].y]); 
			if (neighbors[i].distance > current_node.distance + distanceCurToNbr) {
				neighbors[i].parent = current_node;
				neighbors[i].distance = distanceCurToNbr; 
	
				g_score = neighbors[i].distance;
				h_score = euclideanDistance([neighbors[i].x,neighbors[i].y],q_goal);
				neighbors[i].priority = g_score + h_score; // f_score
			}
			
			minheap_insert(visit_queue, neighbors[i]);
			neighbors[i].queued = true;
			drawQueuedNode(neighbors[i]);
		}
	}


	if ( Math.abs(current_node.x-q_goal[0])<(eps/1000) && Math.abs(current_node.y-q_goal[1])<(eps/1000) ) {
		drawHighlightedPathGraph(current_node);	
		return "succeeded"; // check on this
	}
	else if (visit_queue.length>0 && current_node!=q_goal)
		return "iterating";
	else (search_iter_count == search_max_iterations)  
		return "failed"; // is this possible?   //   draw_2D_configuration - draws a square at a given location 
}


function getBestNeighborInFollowMode(bestNeighbor, current_node) {
	value = 3*(bestNeighbor.i-current_node.i) + (bestNeighbor.j-current_node.j);
	bestNeighbor = current_node;		
	switch (value) {
		case 3:  // coming from left, go down 
			bestNeighbor.i = current_node.i;
			bestNeighbor.j = current_node.j+1;
			break;
		case 1:  // coming from up, go right
			bestNeighbor.i = current_node.i;
			bestNeighbor.j = current_node.j-1;
			break;
		case -1: // coming from down, go left
			bestNeighbor.i = current_node.i+1;
			bestNeighbor.j = current_node.j;
			break;
		case -3: // coming from right, go up
			bestNeighbor.i = current_node.i-1;
			bestNeighbor.j = current_node.j;
			break;
		default:
			return "something's awry";
			break;
	}

	return bestNeighbor;
}

function bug0() {

// TODO: how to start??

	current_node = visit_queue.pop(); 
//	G[current_node.i][current_node.j].visited = true;
	draw_2D_configuration([current_node.x,current_node.y]);

	// find closest neighbor to goal
	minDistanceNbrToEnd = Infinity;			
	neighbors = getNeighbors(current_node);
	for (var i=0; i<neighbors.length; i++) {
		if ((neighbors[i] != current_node.parent) && !testCollision([neighbors[i].x,neighbors[i].y])) {
			distanceNbrToEnd =  euclideanDistance([neighbors[i].x,neighbors[i].y],q_goal);
			if (distanceNbrToEnd < minDistanceNbrToEnd) {
				minDistanceNbrToEnd = distanceNbrToEnd;
				bestNeighbor = neighbors[i];
			}
		}
	}
	
	if (testCollision([bestNeighbor.x,bestNeighbor.y])) {
		// find direction of wall - flank right
		bestNeighbor = getBestNeighborInFollowMode(bestNeighbor, current_node);
		
		// if we're in a corner, flank again 
		if (testCollision([bestNeighbor.x,bestNeighbor.y])) {
			bestNeighbor = getBestNeighborInFollowMode(bestNeighbor, current_node);
		}
	}

	bestNeighbor.parent = current_node;
	visit_queue.push(bestNeighbor);	
	drawQueuedNode(bestNeighbor);


	if ( Math.abs(current_node.x-q_goal[0])<(eps/1000) && Math.abs(current_node.y-q_goal[1])<(eps/1000) ) {
		drawHighlightedPathGraph(current_node);	
		return "succeeded"; // check on this
	}
	else if (visit_queue.length>0 && current_node!=q_goal)
		return "iterating";
	else (search_iter_count == search_max_iterations)  
		return "failed"; // is this possible?   //   draw_2D_configuration - draws a square at a given location 

}


function bug1() {


}


