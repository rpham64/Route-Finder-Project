package roadgraph;


import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections of multiple roads
 * Edges are the roads.
 *
 */
public class MapGraph {
	//TODO: Add your member variables here in WEEK 2
	// Maintain both nodes and edges as you will need to
	// be able to look up nodes by lat/lon or by roads
	// that contain those nodes.

	// Adjacency List Map that maps location -> node
	private HashMap<GeographicPoint, MapNode> intersections;
	
	// Adjacency List of edges
	private HashSet<MapEdge> roads;
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		// TODO: Implement in this constructor in WEEK 2
		intersections = new HashMap<GeographicPoint, MapNode>();
		roads = new HashSet<MapEdge>();
		
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//TODO: Implement this method in WEEK 2
		return intersections.values().size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Collection<GeographicPoint> getVertices()
	{
		//TODO: Implement this method in WEEK 2
		return intersections.keySet();
	}
	
	// get a set of neighbor nodes from a mapnode
	private Set<MapNode> getNeighbors(MapNode node) {
		return node.getNeighbors();
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		//TODO: Implement this method in WEEK 2
		return roads.size();
	}

	// For DEBUGGING.  Print the Nodes in the graph
	public void printNodes()
	{
		System.out.println("******PRINTING NODES******");
		System.out.println("There are " + getNumVertices() + " Nodes: \n");
		for (GeographicPoint pt : intersections.keySet())
		{
			MapNode node = intersections.get(pt);
			System.out.println(node);
		}
	}

	// For DEBUGGING.  Print the Edges in the graph
	public void printEdges()
	{
		System.out.println("******PRINTING EDGES******");
		System.out.println("There are " + getNumEdges() + " Edges:\n");
		for (MapEdge road : roads)
		{
			System.out.println(road);
		}

	}
	
	/*Check if this node exists in the graph
	 * 
	 * @param intersection node
	 * @return True if MapGraph contains this node
	 */
	public boolean contains(GeographicPoint intersection) {
		return intersections.containsKey(intersection);
	}
	
	
	/** Add a node corresponding to an intersection
	 *
	 * @param latitude The latitude of the location
	 * @param longitude The longitude of the location
	 * */
	public void addVertex(double latitude, double longitude)
	{
		GeographicPoint location = new GeographicPoint(latitude, longitude);
		this.addVertex(location);
	}
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		// TODO: Implement this method in WEEK 2
		MapNode node = intersections.get(location);
		
		// Check: node is null
		if (node == null) {
			node = new MapNode(location);
			intersections.put(location, node);
		}
		else {
			System.out.println("Fail! Node at location " + location + " already exists!");
			return false;
		}
		
		// Uncomment to see information about added vertex
//		System.out.println("Added vertex: " + node);
		return true;
	}
	
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {

		//TODO: Implement this method in WEEK 2
		MapNode start = intersections.get(from);
		MapNode end = intersections.get(to);
		
		// Check: GeographicPoints "from" and "to" exist in MapGraph
		if (!intersections.containsValue(start)) {
			System.out.println("Error! GeographicPoint at location " + start + " does not exist!");
		}
		if (!intersections.containsValue(end)) {
			System.out.println("Error! GeographicPoint at location " + end + " does not exist!");
		}
		
		// Check: GeographicPoints "from" and "to" are not NULL
		if (start == null) {
			throw new NullPointerException("addEdge: node " + start + " does not exist."); 
		}
		if (end == null) {
			throw new NullPointerException("addEdge: node " + end + " does not exist."); 
		}
		
		// Create new road connecting start and end.
		// Add to HashSet roads. Then add to start's MapNode property using MapNode.addEdge
		MapEdge road = new MapEdge(roadName, roadType, start, end, length);
		roads.add(road);
		start.addEdge(road);
		
		// Uncomment to see information about added edge
//		System.out.println("Added edge: " + road);
	}
	
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 2
		// Check: start and goal exist in MapGraph
		if (start == null || goal == null) {
			throw new NullPointerException("One or both MapNodes have NULL location. Please verify"
					+ " these locations exist or use method addVertex to add them.");
		}
		
		MapNode startNode = intersections.get(start);
		MapNode goalNode = intersections.get(goal);
		
		if (startNode == null) { 
			System.out.println("Starting node: " + startNode + " does not exist.");
			return null;
		}
		if (goalNode == null) { 
			System.out.println("Starting node: " + goalNode + " does not exist.");
			return null;
		}
		
		// Begin Breadth-first Search Algorithm
		Queue<MapNode> queue = new LinkedList<MapNode>();	// Intersections to explore
		HashSet<MapNode> visited = new HashSet<MapNode>();	// Visited intersections
		HashMap<MapNode, MapNode> parentMap = new HashMap<MapNode, MapNode>();		// Parent map

		// Loop through queue and construct the parentMap
		bfsLoop(startNode, goalNode, queue, visited, parentMap, nodeSearched);

		// Construct the shortest path from start to goal using the constructed parentMap
		List<GeographicPoint> path = findPath(startNode, goalNode, parentMap);
		
		// Check: No path if path contains only one element (we didn't move)
		if (path.size() == 1) { return null; }
		
		return path;
	}

	
	/*Helper method for bfs*/
	private void bfsLoop(MapNode startNode, MapNode goalNode, Queue<MapNode> queue, 
			HashSet<MapNode> visited, HashMap<MapNode, MapNode> parentMap, 
			Consumer<GeographicPoint> nodeSearched) {
		
		// Add start to queue and visited
		queue.add(startNode);
		visited.add(startNode);
		
		// Loop through queue until queue is empty or we reach "goal"
		while (!queue.isEmpty()) {
			
			MapNode curr = queue.remove();
			
			// Hook for visualization.  See writeup.
			nodeSearched.accept(curr.getLocation());
			
			// If we found the goalNode, then quit loop
			if (curr.equals(goalNode)) { break; }
			
			Set<MapNode> neighbors = getNeighbors(curr);
			
			// Check: neighbor intersections of curr. See if any are "goal"
			for (MapNode neighbor : neighbors) {
				
				if (!visited.contains(neighbor)) {
					
					queue.add(neighbor);
					visited.add(neighbor);
					parentMap.put(neighbor, curr);
					
				}
				
			}
			
		}
	}
	
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3
		return heuristicSearch(start, goal, nodeSearched, "Dijkstra");
		
	}

	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3
		return heuristicSearch(start, goal, nodeSearched, "AStar");

	}

	/** Helper method to find the shortest path using Dijkstra or A* search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @param searchType Either Dijkstra or AStar
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> heuristicSearch(GeographicPoint start, 
			 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched, String searchType)
	{
		// TODO: Implement this method in WEEK 3
		// Check: start and goal locations exist
		if (start == null || goal == null) {
			throw new NullPointerException("One or both MapNodes have NULL location. Please verify"
				+ " these locations exist or use method addVertex to add them.");
		}
	
		MapNode startNode = intersections.get(start);
		MapNode goalNode = intersections.get(goal);
		
		// Check: startNode and goalNode exist in MapGraph
		if (startNode == null) { 
			System.out.println("Starting node: " + startNode + " does not exist.");
			return null;
		}
		if (goalNode == null) { 
			System.out.println("Starting node: " + goalNode + " does not exist.");
			return null;
		}
		
		PriorityQueue<MapNode> queue = new PriorityQueue<MapNode>();			// Intersections to explore
		HashSet<MapNode> visited = new HashSet<MapNode>();						// Visited intersections
		HashMap<MapNode, MapNode> parentMap = new HashMap<MapNode, MapNode>();	// Parent map
		
		// Set all mapnode distances to infinity (except for startNode)
		for (MapNode node : intersections.values()) {
		node.setDistance(Double.POSITIVE_INFINITY);
		}
		startNode.setDistance(0);
		
		// Initiate search loop and fill parentMap
		heuristicSearchLoop(startNode, goalNode, queue, visited, parentMap, nodeSearched, searchType);
		
		// Construct shortest path from startNode to goalNode
		List<GeographicPoint> path = findPath(startNode, goalNode, parentMap);
		
		return path;
	}
	
	/*Helper method for searching MapGraph and constructing parentMap*/
	private void heuristicSearchLoop(MapNode startNode, MapNode goalNode, PriorityQueue<MapNode> queue, 
			HashSet<MapNode> visited, HashMap<MapNode, MapNode> parentMap, 
			Consumer<GeographicPoint> nodeSearched, String searchType) {
		
		queue.add(startNode);	// Add startNode to queue
		
		while (!queue.isEmpty()) {
			
			// Remove head of priority queue and assign to "curr"
			MapNode curr = queue.remove();
			
			// Hook for visualization.  See writeup.
			nodeSearched.accept(curr.getLocation());
			
			if (!visited.contains(curr)) {
				
				// Add curr to visited
				visited.add(curr);
				
				// If we reached goalNode, then break loop
				if (curr.equals(goalNode)) { break; }
				
				for (MapEdge edge : curr.getEdges()) {
					
					// Get neighbor nodes of curr
					MapNode next = edge.getOtherNode(curr);
					
					/* Calculate current distance = current node's distance + edge length
					 * Sum of distance from start -> current and heuristic cost from current -> goal
					 * If searchType is "AStar", add heuristic cost 
					 */
					Double currentDistance = curr.getDistance() + edge.getLength();
					
					// Check: searchType is AStar
					if (searchType.equals("AStar")) {
						GeographicPoint currentLocation = curr.getLocation();
						GeographicPoint goalLocation = goalNode.getLocation();
						Double currToGoal = currentLocation.distance(goalLocation);
						
						currentDistance += currToGoal;
					}
					
					// Find: neighbor node with the least distance
					if (currentDistance < next.getDistance()) {
						
						next.setDistance(currentDistance);
						parentMap.put(next, curr);	// Child -> Parent
						queue.add(next);
						
					}
					
				}
			}
			
		}
	}
	
	
	/*Returns the shortest path from start to goal
	 * 
	 * @param start (Starting location)
	 * @param goal (Ending location)
	 * @param parentMap (HashMap with child-parent pairs)
	 * @return List of intersections that form the shortest path from
	 * 			start to goal (including both start and goal)
	 * 
	 * */
	private List<GeographicPoint> findPath(MapNode startNode, MapNode goalNode,
			HashMap<MapNode, MapNode> parentMap) {
		
		// Reconstruct Shortest Path
		LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
		MapNode curr = goalNode;
		
		while (curr != null) {
			path.add(curr.getLocation());
			curr = parentMap.get(curr);
		}
		
		// Reverse constructed path
		Collections.reverse(path);
		
		return path;
	}
	
	public static void main(String[] args)
	{
		/*System.out.print("Making a new map...");
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
		System.out.println("DONE.");*/
		
		// You can use this method for testing.  
		
		// Use this code in Week 3 End of Week Quiz
		System.out.print("Making a new map...");
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		
		
	}
	
}
