
function heap_swap(heap, a, b) {
	var tmp = heap[a];
	heap[a] = heap[b];
	heap[b] = tmp;
}


// define insert function for min binary heap
function minheap_insert(heap, new_element) {
    // STENCIL: implement your min binary heap insert operation
	//console.log("insert");

	// add to heap
	heap.push(new_element);

	var new_element_idx = heap.length - 1;
	//console.log("new elt idx: ");console.log(new_element_idx);

	var parent_idx = Math.floor((new_element_idx - 1) / 2);
	//console.log("parent idx: ");console.log(parent_idx);
	//console.log("new elt: ");console.log(new_element);
	//console.log("parent elt: ");console.log(heap[parent_idx]);


	// heapify up
	while (parent_idx!=-1 && new_element.priority < heap[parent_idx].priority) {
		heap_swap(heap, new_element_idx, parent_idx);
		new_element_idx = parent_idx;
		parent_idx = Math.floor((new_element_idx - 1) / 2);
	}
}


// define extract function for min binary heap
function minheap_extract(heap) {
    // STENCIL: implement your min binary heap extract operation
	//console.log("extract");

	var root = heap[0];

	// remove root (swap roop with end, lop off)
	heap_swap(heap, 0, heap.length-1);
	heap.splice(heap.length-1, 1);

	try {
	
	// heapify down
	var left_child = 1;
	var right_child = 2;
	var moved_node = 0;
	while (true) { 

		// done heapifying
		if ( ((left_child < heap.length && heap[moved_node].priority < heap[left_child].priority)) &&   
	       	 ((right_child < heap.length && heap[moved_node].priority < heap[right_child].priority)) ) {
			break;
		}
		if ( ((left_child < heap.length && heap[moved_node].priority < heap[left_child].priority)) && right_child >= heap.length) {
			break;
		}

		// heapify down
		if (heap[left_child].priority > heap[right_child].priority) {
			heap_swap(heap, moved_node, right_child);
			moved_node = right_child;
		} else {
			heap_swap(heap, moved_node, left_child);
			moved_node = left_child;
		}

		var num_leaves = (heap.length + 1) / 2;
		if (moved_node >= (heap.length-num_leaves) && moved_node < heap.length) {
			// if the node is a leaf, stop	
			break;	
		} 
		else {
			left_child = 2*moved_node + 1;
			right_child = 2*moved_node + 2;
		}
	}

	}
	catch(q) {
		console.log("lol");
	}


	return root;

}




function printVisitQueue(q) {
	for (var i=0; i<q.length; i++) {
		console.log(minheap_extract(q).priority);
	}
}


