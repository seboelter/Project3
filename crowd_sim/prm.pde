
//A box obstacle
class Box {
  Vec2 topLeft;
  float w, h;
  
  Box(Vec2 topLeft, float w, float h){
    this.topLeft = topLeft;
    this.w = w;
    this.h = h;
  }
}



class PRM {
  int numNodes;
  ArrayList<Integer>[] neighbors;
  boolean[] visited;
  int[] parent;
  Vec2[] nodePos;
  ArrayList<Integer> path;
  int startNode, goalNode;
  int randomNodeStartID;
  ArrayList<Vec2> pathRandom;

  PRM(int numNodes, Vec2 agent, Vec2 goal) {
    
    this.numNodes = numNodes;
    neighbors = new ArrayList[numNodes];
    visited = new boolean[numNodes];
    parent = new int[numNodes];
    nodePos = new Vec2[numNodes];
    path = new ArrayList();
    
    startNode = 0;
    goalNode = 1;
    randomNodeStartID = 2;
    
    nodePos[startNode] = agent;
    nodePos[goalNode] = goal;
    
    pathRandom = new ArrayList<Vec2>();
  }
  
  void createRandomPath(Vec2 start, Vec2 end, int size) {
    pathRandom.add(start);
    for (int i = 0; i < size; i++){
      Vec2 next = generateNextValidNode();
      pathRandom.add(next);
    }
    pathRandom.add(end);
  }
  
  
  Vec2 generateNextValidNode() {
    // Definition in randomMap
    return getValidPosition();
  }
  
  void generateRandomNodes(Vec2[] circleCenters, float[] circleRadii, int numCircles, Box[] all_boxes, int numBoxes) {
    for (int i = randomNodeStartID; i < numNodes; i++) {
      //Vec2 randPos = new Vec2(random(width), random(40, height));
      //nodePos[i] = generateValidNode(circleCenters, circleRadii, numCircles, all_boxes, numBoxes, randPos);
      nodePos[i] = generateNextValidNode();
    }
  }

  Vec2 generateValidNode(Vec2[] circleCenters, float[] circleRadii, int numCircles, Box[] all_boxes, int numBoxes, Vec2 potentialPos) {
    boolean insideAnyCircle = true;
    boolean insideAnyBox = true;
    while (insideAnyCircle || insideAnyBox) {
      potentialPos = new Vec2(random(width), random(40, height));
      insideAnyCircle = pointInCircleList(circleCenters, circleRadii, numCircles, potentialPos);
      insideAnyBox = pointInBoxList(all_boxes, numBoxes, potentialPos);
    }  
    return potentialPos;
  }
   
  //Set which nodes are connected to which neighbors based on PRM rules
  void connectNeighbors(Vec2[] circleCenters, float[] circleRadii, int numCircles, Box[] all_boxes, int numBoxes){
    for (int i = 0; i < numNodes; i++){
      neighbors[i] = new ArrayList<Integer>();  //Clear neighbors list
      
      for (int j = 0; j < numNodes; j++){
        if (i == j) continue; //don't connect to myself 
        float distBetween = nodePos[i].distanceTo(nodePos[j]);
  
        if (distBetween > 200) continue;
  
        Vec2 dir = nodePos[j].minus(nodePos[i]).normalized();
        
        hitInfo circleListCheck = rayCircleListIntersect(circleCenters, circleRadii, nodePos[i], dir, distBetween, numCircles);
        hitInfo boxListIntersect = rayBoxListIntersect(all_boxes, nodePos[i], dir, distBetween, numBoxes);
        if (!circleListCheck.hit && !boxListIntersect.hit){
          neighbors[i].add(j);
        }
        
      }
    }
  }
  
  void buildPRM(Vec2[] circleCenters, float[] circleRadii, int numCircles, Box[] all_boxes, int numBoxes){
    generateRandomNodes(circleCenters, circleRadii, numCircles, all_boxes, numBoxes);
    connectNeighbors(circleCenters, circleRadii, numCircles, all_boxes, numBoxes);
  }
  
  // If some dynamic obstacle breaks the PRM then fix it
  void fixPRM(Vec2[] circleCenters, float[] circleRadii, int numCircles, Box[] all_boxes, int numBoxes){
    for (int i = randomNodeStartID; i < numNodes; i++) {
      nodePos[i] = generateValidNode(circleCenters, circleRadii, numCircles, all_boxes, numBoxes, nodePos[i]);
    }
    connectNeighbors(circleCenters, circleRadii, numCircles, all_boxes, numBoxes);
  }
  
  
  // Return the closest node the passed in point
  int closestNode(Vec2 point){
    int closest = -1;
    float bestDist = 100000000;
  
    for (int i = 0; i < numNodes; i++) {
      float dist = point.distanceTo(nodePos[i]);
      if (dist < bestDist){
        closest = i;
        bestDist = dist;
      }
    }
    return closest;
  }
  
  //BFS
  void runBFS(Vec2 start, Vec2 goal){
    int startID = closestNode(start);
    int goalID = closestNode(goal);
    
    startNode = startID;
    goalNode = goalID;
    
    ArrayList<Integer> fringe = new ArrayList();  //Make a new, empty fringe
    path = new ArrayList(); //Reset path
    for (int i = 0; i < numNodes; i++) { //Clear visit tags and parent pointers
      visited[i] = false;
      parent[i] = -1; //No parent yet
    }
  
    visited[startID] = true;
    fringe.add(startID);
    
    while (fringe.size() > 0){
      int currentNode = fringe.get(0);
      fringe.remove(0);
      if (currentNode == goalID){
        break;
      }
      for (int i = 0; i < neighbors[currentNode].size(); i++){
        int neighborNode = neighbors[currentNode].get(i);
        if (!visited[neighborNode]){
          visited[neighborNode] = true;
          parent[neighborNode] = currentNode;
          fringe.add(neighborNode);
        }
      } 
    }
    
    for (int i = 0; i < numNodes; i++)
      println(parent[i]);
    
    int prevNode = parent[goalID];
    path.add(0,goalID);
    while (prevNode >= 0){
      path.add(0,prevNode);
      prevNode = parent[prevNode];
    }
    
  }
  
  void draw() {
    //Draw Planned Path
    
    stroke(20,255,40);
    strokeWeight(1);
    for (int i = 0; i < pathRandom.size()-1; i++){
      Vec2 curNode = pathRandom.get(i);
      Vec2 nextNode = pathRandom.get(i+1);
      stroke(250);
      line(curNode.x, curNode.y,  nextNode.x,  nextNode.y);
      stroke(20,30,150,20);
      circle(curNode.x, curNode.y, 2);
    }
  }
}
