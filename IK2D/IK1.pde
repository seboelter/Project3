

void ik(Vec2 goal, int i) 
{
  Vec2 startToGoal = goal.minus(starts[i]);
  Vec2 startToEndEffector = endPoint.minus(starts[i]);
  
  float dotProd = dot(startToGoal.normalized(), startToEndEffector.normalized());
  dotProd = clamp(dotProd, -1, 1);
  

  if (cross(startToGoal, startToEndEffector) < 0){
    angles[i] += acos(dotProd);
  }
  else {
    angles[i] -= acos(dotProd);
  }
  
  // Angle restrictions
  float minAngle = -PI/2;
  float maxAngle = PI/2;
  
  // Shoulder joint (0th element) angle restriction
  if (i == 0) 
    minAngle = 0;
  
  angles[i] = clamp(angles[i], minAngle, maxAngle);
}


void fk(){
  for (int i=0; i < LINKS; i++) {
    // Accumulate the angles in curr_angle
    float curr_angle = 0;
    for (int j=0; j<=i; j++)
      curr_angle += angles[j];
    
    Vec2 direction = new Vec2(cos(curr_angle)*lengths[i], sin(curr_angle)*lengths[i]);
    Vec2 curr_point = direction.plus(starts[i]);
    
    if (i < LINKS-1)
      starts[i+1] = curr_point;
    else
      endPoint = curr_point;
  }
  
}

void solve(){
  Vec2 goal = new Vec2(mouseX, mouseY);
  
  for (int j = LINKS-1; j >= 0; j--){
    ik(goal, j);
    fk(); //Update link positions with fk (e.g. end effector changed)
    //print("\tAngle", j, ":", angles[j]);
  }
  println();
  
  //println("Angle 0:",nf(a0,1,2),"Angle 1:",nf(a1,1,2),"Angle 2:",nf(a2,1,2));
}
