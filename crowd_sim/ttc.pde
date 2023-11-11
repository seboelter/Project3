//CSCI 5611 - Pathing and Crowd Sim
//Anticipatory Collision Avoidance (TTC-Forces) [Exercise]
// Stephen J. Guy <sjguy@umn.edu>

//NOTE: The simulation starts paused! Press the spacebar to run it.

/*
TODO:
  1. Currently, there are no forces acting on the agents. Add a goal force that penalizes
     the difference between an agent's current velocity and it's goal velocity. 
     i.e., goal_force = goal_vel-current_vel; acc += goal_force;
     Make sure that the length of goal velocity is always set to the goal_speed unless 
     the agent is slowing down to avoid jittering around the goal.

  1a. Scale the goal force by the value k_goal. Find a value of k_goal that lets the agents
     smoothly reach their goal without overshooting it first.
     i.e., goal_force = k_goal*(goal_vel-current_vel); acc += goal_force


  2. Finish the function computeTTC() so that it computes the time-to-collision (TTC) between
     two agents given their positions, velocities and radii. Your implementation should likely
     call rayCircleIntersectTime to compute the time a ray intersects a circle as part of
     determining the time to collision. (i.e., at what time would a ray moving at your
     current relative velocity intersect the Minkowski sum of the obstacle and your reflection?)
     If there is no collision, return a TTC of -1.
     ie., TTC(A,B) = Ray-Disk(A's position, relative velocity, B's position, combined radius)

  2a. Test computeTTC() with some scenarios where you know the (approximate) true value.

  3. Compute an avoidance force as follows:
       - Find the ttc between agent "id" and it's neighbor "j" (make sure id != j)
       - If the ttc is negative (or there is no collision) then there is no avoidance force w.r.t agent j
       - Predict where both agent's well be at the moment of collision (time = TTC)
         ie., A_future = A_current + A_vel*ttc; B_future + B_current + B_vel*ttc
       - Find the relative vector between the agents at the moment of collision, normalize it
         i.e, relative_future_direction = (A_future - B_future).normalized()
       - Compute the per-agent avoidance force by scaling this direction by k_avoid and
         dividing by ttc (smaller ttc -> more urgent collisions)
         acc += k_avoid * (1/ttc) * relative_future_direction
      The final force should be the avoidance force for each neighbor plus the overall goal force.
      Try to find a balance between k_avoid and k_goal that gives good behavior. Note: you may need a
      very large value of k_goal to avoid collisions between agents.
      
   4. Make the following simulation improvements:
     - Draw agents with a slightly smaller radius than the one used for collisions
     - When an agent reaches its goal, stop it from reacting to other agents
     
   5. Finally, place 2 more agents in the scene and try to make an interesting scenario where the
      agents interact with each other.

CHALLENGE:
  1. Give agents a maximum force and maximum velocity and don't let them violate these constraints.
  2. Start two agents slightly colliding. Update the approach to better handle collisions.
  3. Add a static obstacle for the agent to avoid (hint: treat it as an agent with 0 velocity).
*/

static int maxNumAgents = 25;

int numAgents = 15;

int obstacles = maxNumAgents - numAgents;

int agentCount = 0;
int goalCount = 0;


float k_goal = 30;  //TODO: Tune this parameter to agent stop naturally on their goals
float k_avoid = 500;
float agentRad = 10;
float goalSpeed = 150;


float sensingRadius = agentRad * 200;
float radiusScaler = 0.9;

//The agent states
Vec2[] agentPos = new Vec2[maxNumAgents];
Vec2[] agentVel = new Vec2[maxNumAgents];
Vec2[] agentAcc = new Vec2[maxNumAgents];

ArrayList<ArrayList<Integer>> neighbors = new ArrayList<ArrayList<Integer>>();

//The agent goals
Vec2[] goalPos = new Vec2[maxNumAgents];





void setup(){
  //size(850,650);
  size(850,650,P3D); //Smoother //<>//
 
  reset(); //<>//
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
  text("r: reset    space: resume/pause", width/2, 20);
  
  //Update agent if not paused
  if (!paused){
    moveAgent(1.0/frameRate);
  }
 
  //Draw orange goal rectangle
  fill(255,150,50);
  text("Goals", 10, 20);
  for (int i = 0; i < numAgents; i++){
    rect(goalPos[i].x-10, goalPos[i].y-10, 20, 20);
  }
 
  //Draw the green agents
  fill(20,200,150);
  text("Agents", 10, 50);
  for (int i = 0; i < numAgents; i++){
    circle(agentPos[i].x, agentPos[i].y, agentRad*2*radiusScaler);
  }
  
  //Draw the red agents
  fill(200,30,30);
  text("Obstacles", 10, 80);
  for (int i = numAgents; i < maxNumAgents; i++){
    circle(agentPos[i].x, agentPos[i].y, agentRad*2*radiusScaler);
  }
  
}

//Pause/unpause the simulation
void keyPressed(){
  if (key == ' ') paused = !paused;
  if (key == 'r') resetMap = !resetMap;
}
