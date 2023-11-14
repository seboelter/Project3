
int agentCount = 0;
int goalCount = 0;


float k_goal = 40;
float k_avoid = 600;
float agentRad = 10;
float goalSpeed = 80;


float sensingRadius = agentRad * 30;
float radiusScaler = 0.9;

//The agent states
Vec2[] agentPos = new Vec2[maxNumAgents];
Vec2[] agentVel = new Vec2[maxNumAgents];
Vec2[] agentAcc = new Vec2[maxNumAgents];

float[] agentRads = new float[maxNumAgents];


// Neighbors
ArrayList<ArrayList<Integer>> neighbors = new ArrayList<ArrayList<Integer>>();

//The agent goals
Vec2[] goalPosInit = new Vec2[maxNumAgents];
Vec2[] goalPos = new Vec2[maxNumAgents];

PRM[] prms = new PRM[numAgents];


void updateNeighbors(){
    for (int i = 0; i < numAgents; i++){
        neighbors.get(i).clear();
        for (int j = 0; j < maxNumAgents; j++){
            if (i == j) continue;
            float dist = agentPos[i].minus(agentPos[j]).length();
            if (dist < sensingRadius) {
                neighbors.get(i).add(j);
            }   
        }
    }
}




//Return at what time agents 1 and 2 collide if they keep their current velocities
// or -1 if there is no collision.
float computeTTC(Vec2 pos1, Vec2 vel1, float radius1, Vec2 pos2, Vec2 vel2, float radius2){
  
  // ie., TTC(A,B) = Ray-Disk(A's position, relative velocity, B's position, combined radius)
  float combined_radius = radius1 + radius2;
  Vec2 w = pos2.minus(pos1);
  if (dot(w, w) < combined_radius * combined_radius)
      return 0;

  Vec2 relative_velocity = vel1.minus(vel2);
  float ttc = rayCircleIntersectTime(pos2, combined_radius, pos1, relative_velocity);
  return ttc;
}


// Compute attractive forces to draw agents to their goals,
// and avoidance forces to anticipatory avoid collisions
Vec2 computeAgentForces(int id){
  //TODO: Make this better
  Vec2 acc = new Vec2(0,0);
  Vec2 goalVel = goalPos[id].minus(agentPos[id]);

  if (goalVel.length() > 1) 
      goalVel.setToLength(goalSpeed);

  Vec2 goalForce = goalVel.minus(agentVel[id]).times(k_goal);
  acc.add(goalForce);

  if (goalVel.length() < 1) return acc;
  
  float ttc;
  
  // Avoidance forces
  for (int j : neighbors.get(id)) {
    
    if (id == j) continue;
    
    ttc = computeTTC(agentPos[id], agentVel[id], agentRads[id], agentPos[j], agentVel[j], agentRads[j]);
  
    // Resolve collision
    if (ttc < 0.01 && ttc >= 0 && j < numAgents && id < numAgents) {
      //continue;
      Vec2 dist = agentPos[id].minus(agentPos[j]);
      float overlap = agentRads[id] + agentRads[j] - dist.length();
      Vec2 dir = dist.normalized();
      agentPos[id] = agentPos[id].minus(dir.times(overlap/1.9));
      agentPos[j] = agentPos[j].plus(dir.times(overlap/1.9));
    }
    // If not collision continue
    if (ttc < 0) continue;

    // Collision Avoidance force
    Vec2 futurePos1 = agentPos[id].plus(agentVel[id].times(ttc)); //<>//
    Vec2 futurePos2 = agentPos[j].plus(agentVel[j].times(ttc));

    // Relative direction vector
    Vec2 relative_future_direction = futurePos1.minus(futurePos2).normalized();

    // Force Magnitude
    acc.add(relative_future_direction.times(k_avoid / ttc));
  }

  return acc;
}


//Update agent positions & velocities based acceleration
void moveAgent(float dt)
{  
  // Precompute all the neighbors
  updateNeighbors();
  
  // Update agents goal based on path calculated using RRT
  for (int i = 0; i < numAgents; i++) {
    if (agentPos[i].distanceTo(goalPos[i]) < 1) {
      
      if (!prms[i].pathRandom.isEmpty())
        prms[i].pathRandom.remove(0);
      if (!prms[i].pathRandom.isEmpty())
        goalPos[i] = prms[i].pathRandom.get(0);
    }
    
  }
  
  //Compute accelerations for every agents
  for (int i = 0; i < numAgents; i++){
    agentAcc[i] = computeAgentForces(i);
  }

  //Update position and velocity using (Eulerian) numerical integration
  for (int i = 0; i < numAgents; i++){
    agentVel[i].add(agentAcc[i].times(dt));
    agentPos[i].add(agentVel[i].times(dt));
  }
}



///////////////////////

float rayCircleIntersectTime(Vec2 center, float r, Vec2 l_start, Vec2 l_dir){
 
  //Compute displacement vector pointing from the start of the line segment to the center of the circle
  Vec2 toCircle = center.minus(l_start);
 
  //Solve quadratic equation for intersection point (in terms of l_dir and toCircle)
  float a = l_dir.length()*l_dir.length();
  float b = -2*dot(l_dir,toCircle); //-2*dot(l_dir,toCircle)
  float c = toCircle.lengthSqr() - (r*r); //different of squared distances
 
  float d = b*b - 4*a*c; //discriminant
 
  if (d >=0 ){
    //If d is positive we know the line is colliding
    float t = (-b - sqrt(d))/(2*a); //Optimization: we typically only need the first collision!
    if (t >= 0) return t;
    return -1;
  }
 
  return -1; //We are not colliding, so there is no good t to return
}
