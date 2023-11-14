static int maxNumAgents = 30;
int numAgents = 15;
int obstacles = maxNumAgents - numAgents;

int agentCount = 0;
int goalCount = 0;


float k_goal = 30;
float k_avoid = 500;
float agentRad = 10;
float goalSpeed = 100;


float sensingRadius = agentRad * 200;
float radiusScaler = 0.9;

//The agent states
Vec2[] agentPos = new Vec2[maxNumAgents];
Vec2[] agentVel = new Vec2[maxNumAgents];
Vec2[] agentAcc = new Vec2[maxNumAgents];

// Neighbors
ArrayList<ArrayList<Integer>> neighbors = new ArrayList<ArrayList<Integer>>();

//The agent goals
Vec2[] goalPos = new Vec2[maxNumAgents];


void setup(){
  size(850,650,P2D);
  reset();
}


void reset() {
  agentCount = 0;
  goalCount = 0;
  neighbors.clear();
  
  // Set initial agent positions and goals
  for (int i = 0; i < numAgents; i++)
    createRandomAgent(i);
  
  // Obstacles
  for (int i = numAgents; i < maxNumAgents; i++)
    createRandomAgent(i);
 
 
  //Set initial velocities to cary agents towards their goals
  for (int i = 0; i < maxNumAgents; i++){
    agentVel[i] = goalPos[i].minus(agentPos[i]);
    if (agentVel[i].length() > 0)
      agentVel[i].setToLength(goalSpeed);
  }

  // Create 2D arraylist of neighbors, first index is i'th agent's neighbors arraylist
  for (int i = 0; i < numAgents; i++){
    neighbors.add(new ArrayList<Integer>());
  }
}

void updateNeighbors(){
    for (int i = 0; i < numAgents; i++){
        neighbors.get(i).clear();
        for (int j = 0; j < maxNumAgents; j++){
            if (i == j) continue;
            float dist = agentPos[i].minus(agentPos[j]).length();
            if (dist < sensingRadius) { //<>//
                neighbors.get(i).add(j);
            }   
        }
    }
}


boolean paused = true;
boolean resetMap = false;

void draw(){
  background(255,255,255); //White background
  
  if (resetMap) {
    reset();
    resetMap = !resetMap;
  }
   
  fill(0);
  textSize(18);
  text("r: reset         space: resume/pause", width/2, 20);
  stroke(2);
  line(0, 30, width, 30);
  
  //Update agent if not paused
  if (!paused){
    moveAgent(1.0/frameRate);
  }
 
  //Draw orange goal rectangle
  fill(0); text("Goals", 30, 20);
  fill(255,150,50); 
  rect(5, 5, 20, 20);
  for (int i = 0; i < numAgents; i++){
    rect(goalPos[i].x-10, goalPos[i].y-10, 20, 20);
  }
 
  //Draw the green agents
  fill(0); text("Agents", 115, 20);
  fill(20,200,150); 
  circle(100, 15, 20);
  for (int i = 0; i < numAgents; i++){
    // Draw agents and arrows
    strokeWeight(0.1);
    fill(20,200,150);
    circle(agentPos[i].x, agentPos[i].y, agentRad*2*radiusScaler);
    drawArrowHead(agentPos[i], goalPos[i]);
    
    // Draw lines to show agents path to goal
    strokeWeight(2);
    stroke(50, 50, 50, 30);
    line(agentPos[i].x, agentPos[i].y, goalPos[i].x, goalPos[i].y);
  }
  
  //Draw the red agents
  fill(0); text("Obstacles", 10+200, 20);
  fill(200,30,30); 
  circle(195, 15, 20);
  for (int i = numAgents; i < maxNumAgents; i++){
    circle(agentPos[i].x, agentPos[i].y, agentRad*2*radiusScaler);
  }
  
}

//Pause/unpause the simulation
void keyPressed(){
  if (key == ' ') paused = !paused;
  if (key == 'r') resetMap = !resetMap;
}


// To show orientation
void drawArrowHead(Vec2 start, Vec2 end) {
  
  float arrowSize = agentRad*1.1;
  Vec2 dir = end.minus(start);
  
  // Once goal is reached don't draw arrow
  if (dir.length() < 2) return;
  
  float angle = getAngle(dir);
  float halfAngle = PI / 10;
  
  float x1 = start.x - arrowSize * cos(angle - halfAngle);
  float y1 = start.y - arrowSize * sin(angle - halfAngle);

  float x2 = start.x - arrowSize * cos(angle + halfAngle);
  float y2 = start.y - arrowSize * sin(angle + halfAngle);
  
  fill(0);
  triangle(start.x, start.y, x1, y1, x2, y2);
}
