#include <iostream>
#include <vector>
#include <string>
#include <list>
 
#include <limits> // for numeric_limits
 
#include <set>
#include <utility> // for pair
#include <algorithm>
#include <iterator>
 
#define NV 6 // TODO: Generalize
#define NT 4
 
typedef int vertex_t;
typedef double weight_t;
 
const weight_t max_weight = std::numeric_limits<double>::infinity();
 
struct neighbor {
    vertex_t target;
    weight_t weight;
    neighbor(vertex_t arg_target, weight_t arg_weight)
        : target(arg_target), weight(arg_weight) { }
};
 
typedef std::vector<std::vector<neighbor>> adjacency_list_t;
 
 
void aStarComputePaths(vertex_t source,
                          const adjacency_list_t &adjacency_list,
                          std::vector<weight_t> &min_distance,
                          std::vector<vertex_t> &previous)
{
/* MY CODE START */


	int n = adjacency_list.size();
    min_distance.clear();
    min_distance.resize(n, max_weight);
    min_distance[source] = 0;
    previous.clear();
    previous.resize(n, -1);
	int doneProcessingFlag = 1 << (source / nthreads);
	

	int nthreads = NT;  // TODO: generalize
	int nvertices = NV; // TODO: generalize
	std::vector< std::set< std::pair<weight_t, vertex_t>>> openSets (NT);
	std::vector<int> min_distanceLock (NV);
	std::vector<vertex_t> parentVertex (NV);
	std::vector<int> parentLock (NV);


	# pragma omp parallel private ( first_vertex, last_vertex, tid) \
			 shared( nvertices, nthreads, adjacency_list)
  	{
  		int tid;
		int first_vertex;
		int last_vertex;

  		tid      = omp_get_thread_num();
    	nthreads = omp_get_num_threads(); 
    	first_vertex  =  (  tid       * nvertices) / nthreads;
    	last_vertex  =   (( tid + 1 ) * nvertices) / nthreads - 1;

    	while(doneProcessingFlag > 0) {

    		if (!openSets[tid].empty()){
    			weight_t dist = openSets[tid].begin()->first;
    			vertex_t u = openSets[tid].begin()->second;
    			openSets[tid].erase(openSets[tid].begin());

    			const std::vector<neighbor> &neighbors = adjacency_list[u];
        		for (std::vector<neighbor>::const_iterator neighbor_iter = neighbors.begin();
             		neighbor_iter != neighbors.end();
             		neighbor_iter++)
        		{
					vertex_t v = neighbor_iter->target;
            		weight_t weight = neighbor_iter->weight;
           			weight_t distance_through_u = dist + weight;
	    			if (distance_through_u < min_distance[v]) {
	    				// erase current openSets
	        			openSets[tid].erase(std::make_pair(min_distance[v], v));
	        			
	        			int tid_to_process = v / nthreads;
	        			// TODO: lock parent[v] & openSets[v/nthreads] & min_distance[v]	
	        			doneProcessingFlag |= 1 << thread_to_process;
	        			min_distance[v] = distance_through_u;
	        			previous[v] = u;
	        			openSets[tid_to_process].insert(std::make_pair(min_distance[v], v));
	        			// TODO: unlock parent[v] & openSets[v/nthreads] & min_distance[v]
 
	    			}
    			}

    		} else {
    			doneProcessingFlag &= ~(1 << tid); // not correct unless you have 32 threads
    		}
    	}

  	}
    
}
 
 
std::list<vertex_t> aStarGetShortestPathTo(
    vertex_t vertex, const std::vector<vertex_t> &previous)
{
    std::list<vertex_t> path;
    for ( ; vertex != -1; vertex = previous[vertex])
        path.push_front(vertex);
    return path;
}
 
 
int main()
{
    // remember to insert edges both ways for an undirected graph
    adjacency_list_t adjacency_list(6);
    // 0 = a
    adjacency_list[0].push_back(neighbor(1, 7));
    adjacency_list[0].push_back(neighbor(2, 9));
    adjacency_list[0].push_back(neighbor(5, 14));
    // 1 = b
    adjacency_list[1].push_back(neighbor(0, 7));
    adjacency_list[1].push_back(neighbor(2, 10));
    adjacency_list[1].push_back(neighbor(3, 15));
    // 2 = c
    adjacency_list[2].push_back(neighbor(0, 9));
    adjacency_list[2].push_back(neighbor(1, 10));
    adjacency_list[2].push_back(neighbor(3, 11));
    adjacency_list[2].push_back(neighbor(5, 2));
    // 3 = d
    adjacency_list[3].push_back(neighbor(1, 15));
    adjacency_list[3].push_back(neighbor(2, 11));
    adjacency_list[3].push_back(neighbor(4, 6));
    // 4 = e
    adjacency_list[4].push_back(neighbor(3, 6));
    adjacency_list[4].push_back(neighbor(5, 9));
    // 5 = f
    adjacency_list[5].push_back(neighbor(0, 14));
    adjacency_list[5].push_back(neighbor(2, 2));
    adjacency_list[5].push_back(neighbor(4, 9));
 
    std::vector<weight_t> min_distance;
    std::vector<vertex_t> previous;
    aStarComputePaths(0, adjacency_list, min_distance, previous);
    std::cout << "Distance from 0 to 4: " << min_distance[4] << std::endl;
    std::list<vertex_t> path = aStarGetShortestPathTo(4, previous);
    std::cout << "Path : ";
    std::copy(path.begin(), path.end(), std::ostream_iterator<vertex_t>(std::cout, " "));
    std::cout << std::endl;
 
    return 0;
}