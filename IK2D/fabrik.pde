// Returns a point that len distance away from the goal
Vec3 next(Vec3 goal, Vec3 point, float len) {
  // Point to goal vector
  Vec3 p2g = goal.minus(point);
  
  // Getting the direction only
  Vec3 p2g_dir = p2g.normalized();
  
  // Direction vector of appropriate length
  Vec3 arm = p2g_dir.times(len);
  
  // Goal vector minus arm vector gives the start point (vector) of the arm
  return goal.minus(arm);
}


void fabrik(Vec3 goal) {
  endPoint = goal;
  
  for (int i = LINKS-1; i > 0; i--) {
    starts[i] = next(goal, starts[i], lengths[i]);
    goal = starts[i];
  }
  
  for (int i = 1; i < LINKS; i++) {
    starts[i] = next(starts[i-1], starts[i], lengths[i]);
  }
  endPoint = next(starts[LINKS-1], endPoint, lengths[LINKS-1]);
  
}
