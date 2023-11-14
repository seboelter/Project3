// Returns a random position
Vec2 getRandomPosition() {
  return new Vec2(random(agentRad*1.5, width-agentRad*1.5), random(agentRad*4, height-agentRad*1.5));
}

int MIN_DISTANCE_SCALER = 4;

// Returns whether a given position is an agent's starting or goal position
boolean unusedPosition(Vec2 pos) 
{
  for (int i = 0; i < agentCount; i++) {
    if (pos.distanceTo(agentPos[i]) < MIN_DISTANCE_SCALER * agentRad)
      return false;
  }
      
  for (int i = 0; i < goalCount; i++) {
    if (pos.distanceTo(goalPos[i]) < MIN_DISTANCE_SCALER * agentRad)
      return false;
  }
  
  return true;
}

// Returns an unused (neither start nor goal) random position 
Vec2 getValidPosition() 
{
  Vec2 position = getRandomPosition();
  while (!unusedPosition(position))
    position = getRandomPosition();
  return position;
}

Vec2 getValidGoalPosition(Vec2 start_position) {
  while (true) {
    Vec2 position = getValidPosition();
    if (position.distanceTo(start_position) > width/2) {
      return position;
    }
  }
}


void createRandomAgent(int id)
{
  agentPos[id] = getValidPosition();
  if (id < numAgents){
    goalPos[id] = getValidGoalPosition(agentPos[id]);
  } else {
    // Otherwise these are obstacles and obstacles' start and goal are the same
    // This ensure that they don't have to move.
    goalPos[id] = agentPos[id];
  }
  agentCount++;
  goalCount++;
}
