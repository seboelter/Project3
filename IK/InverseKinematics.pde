//Inverse Kinematics
//CSCI 5611 

void setup(){
  size(800,800);
  surface.setTitle("Inverse Kinematics");
}

//Root
Vec2 root = new Vec2(400,400);

//Upper Arm
float l0 = 100; 
float a0 = -1.1; //Shoulder joint

//Lower Arm
float l1 = 100;
float a1 = 0.0; //Elbow joint

//Lower Arm
float l1_left = 100;
float a1_left = 0.3; //Elbow joint

//Hand
float l2 = 100;
float a2 = 0.3; //Wrist joint

//Hand
float l2_left = 100;
float a2_left = 0.3; //Wrist joint

//Finger
float l3 = 100;
float a3 = 0.3; //finger joint

//Finger
float l3_left = 100;
float a3_left = 0.3; //finger joint


Vec2 start_l1, start_l2, start_l2_left,start_l3, start_l3_left,endPoint, endPoint_left;
Vec2 obstacle = new Vec2 (670, 250);
int obstacleR = 50;
float old_a3_left, old_a2_left;
Boolean a3_left_bool = false;
  
void armCircleCollision(Vec2 start,Vec2 end){
  Vec2 v = end.minus(start);
  float v_len = v.length();
  v.normalize(); //Save the distance of the line
  Vec2 toCircle = obstacle.minus(start);
  ////Solve quadratic equation for intersection point (in terms of l_dir and toCircle)
  float a = 1;  //Lenght of l_dir (we noramlized it)
  float b = -2*dot(v,toCircle); //-2*dot(l_dir,toCircle)
  float c = toCircle.lengthSqr() - (10+obstacleR/2)*(10+obstacleR/2); //different of squared distances
  
  float d = b*b - 4*a*c; //discriminant 
  if (d >=0 ){ 
  //    //If d is positive we know the line is colliding, but we need to check if the collision line within the line segment
  //    //  ... this means t will be between 0 and the lenth of the line segment
      float t1 = (-b - sqrt(d))/(2*a); //Optimization: we only take the first collision [is this safe?]
  //    //println(hit.t,t1,t2);
  if (t1>0 && t1 < v_len){
    println("collision");
      a3_left_bool = true;
    } 
  else{
    println("not collision");
     a3_left_bool = false;
    }
    }
   else{
   println("not collision");
   a3_left_bool =false;
   }


}

void solve(){
  old_a3_left = a3_left;
  Vec2 goal = new Vec2(mouseX, mouseY);
  
  Vec2 startToGoal, startToEndEffector;
  float dotProd, angleDiff;
  //Update finger joint
  startToGoal = goal.minus(start_l3_left);
  startToEndEffector = endPoint_left.minus(start_l3_left);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  print("Bool =  ", a3_left_bool);
  armCircleCollision(start_l3_left, endPoint_left);
  if(a3_left_bool == true){//while (armCircleCollision(start_l3_left, endPoint_left))
    if (cross(startToGoal,startToEndEffector) < 0)
      a3_left = old_a3_left;
    else
      a3_left = old_a3_left;
    fk(); //Update link positions with fk (e.g. end effector changed)
    armCircleCollision(start_l3_left, endPoint_left);
    fk();
  }
  else{
  if (cross(startToGoal,startToEndEffector) < 0)
    a3_left += angleDiff;
  else
    a3_left -= angleDiff;  
  fk();
  }
  fk();
  
  //if(a3_left > 1)
  //  a3_left = 1;
  //if(a3_left <-.5)
  //  a3_left = -.5;



  
  //Update finger joint 2
  startToGoal = goal.minus(start_l3);
  startToEndEffector = endPoint.minus(start_l3);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  // do {
  //  if (cross(startToGoal,startToEndEffector) < 0)
  //    a3 += angleDiff;
  //  else
  //    a3 -= angleDiff;
  //  fk(); //Update link positions with fk (e.g. end effector changed)
  //  angleDiff *= -.5; //Shrink angle difference and try again if there is a collision
  //} while (armCircleCollision(start_l3));
  if (cross(startToGoal,startToEndEffector) < 0)
    a3 += angleDiff;
  else
    a3 -= angleDiff;  
  if(a3 > 1)
    a3 = 1;
  if(a3 <0)
    a3 = 0;
  /*TODO: Finger joint limits here*/
  fk(); //Update link positions with fk (e.g. end effector changed)

  
  //Update wrist joint
  old_a2_left = a2_left;
  startToGoal = goal.minus(start_l2_left);
  startToEndEffector = endPoint_left.minus(start_l2_left);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  //armCircleCollision(start_l2_left, start_l3_left);
  armCircleCollision(start_l3_left, endPoint_left);
  if(a3_left_bool ==true){//while (armCircleCollision(start_l3_left, endPoint_left))
    if (cross(startToGoal,startToEndEffector) < 0)
      a2_left = old_a2_left;
    else
      a2_left = old_a2_left;
    fk(); //Update link positions with fk (e.g. end effector changed)
    //armCircleCollision(start_l2_left, start_l3_left);
    armCircleCollision(start_l3_left, endPoint_left);
    fk();
  }
  // if (armCircleCollision(start_l2_left, start_l3_left))
  //{
  //  if (cross(startToGoal,startToEndEffector) < 0)
  //    a2_left -= angleDiff;
  //  else
  //    a2_left+= angleDiff;
  //  fk(); //Update link positions with fk (e.g. end effector changed)
  //} //while (armCircleCollision(start_l3_left, endPoint_left));
  //else{
else{
  if (cross(startToGoal,startToEndEffector) < 0)
    a2_left += angleDiff;
  else
    a2_left -= angleDiff;
  //Joint Limits
  //if(a2_left > 1)
  //  a2_left =1;
  //if(a2_left <-.5)
  //  a2_left = -.5;
 fk();
   }//Update link positions with fk (e.g. end effector changed)
   fk();
//}
  
  //Update wrist joint 2
  startToGoal = goal.minus(start_l2);
  startToEndEffector = endPoint.minus(start_l2);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  if (cross(startToGoal,startToEndEffector) < 0)
    a2 += angleDiff;
  else
    a2 -= angleDiff;
  //Joint Limits
  if(a2 > -0)
    a2 = -0;
  if(a2 <-.5)
      a2 = -.5;
    fk(); //Update link positions with fk (e.g. end effector changed)
  // do {
  //  if (cross(startToGoal,startToEndEffector) < 0)
  //    a2 += angleDiff;
  //  else
  //    a2 -= angleDiff;
  //  fk(); //Update link positions with fk (e.g. end effector changed)
  //  angleDiff *= .5; //Shrink angle difference and try again if there is a collision
  //} while (armCircleCollision(start_l2));

  
  
  
 //Update elbow joint 2
  startToGoal = goal.minus(start_l1);
  startToEndEffector = endPoint.minus(start_l1);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);

  
  //if (armCircleCollision(start_l1, start_l2_left))
  //{
  //  if (cross(startToGoal,startToEndEffector) < 0)
  //    a1 -= angleDiff;
  //  else
  //    a1+= angleDiff;
  //  fk(); //Update link positions with fk (e.g. end effector changed)
  //} //while (armCircleCollision(start_l3_left, endPoint_left));
  //else{

  if (cross(startToGoal,startToEndEffector) < 0)
    a1 += angleDiff;
  else
    a1 -= angleDiff;
   //Joint Limits 
  if(a1 > -.5)
    a1 = -.5;
  if(a1 <-1.2)
    a1 = -1.2;
  fk(); //Update link positions with fk (e.g. end effector changed)
//}
  
  
  
  //Update shoulder joint
  startToGoal = goal.minus(root);
  if (startToGoal.length() < .0001) return;
  startToEndEffector = endPoint.minus(root);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  if (cross(startToGoal,startToEndEffector) < 0)
    a0 += angleDiff;
  else
    a0 -= angleDiff;
  //Joint Limits  
  if(a0 > -.8)
    a0 = -.8;
  if(a0 <-1.5)
    a0 = -1.5;
  fk(); //Update link positions with fk (e.g. end effector changed)
 
  println("Angle 0:",a0,"Angle 1:",a1,"Angle 2:",a2,"Angle 3:", a3);
}

void fk(){
  start_l1 = new Vec2(cos(a0)*l0,sin(a0)*l0).plus(root);
  start_l2 = new Vec2(cos(a0+a1)*l1,sin(a0+a1)*l1).plus(start_l1);
  start_l2_left = new Vec2(cos(a0+a1)*l1,sin(a0+a1)*l1).plus(start_l1);
  start_l3 = new Vec2(cos(a0+a1+a2)*l2,sin(a0+a1+a2)*l2).plus(start_l2);
  start_l3_left = new Vec2(cos(a0+a1+a2_left)*l2,sin(a0+a1+a2_left)*l2).plus(start_l2_left);
  endPoint = new Vec2(cos(a0+a1+a2+a3)*l3,sin(a0+a1+a2+a3)*l3).plus(start_l3);
  endPoint_left = new Vec2(cos(a0+a1+a2_left+a3_left)*l3,sin(a0+a1+a2_left+a3_left)*l3).plus(start_l3_left);
}

float armW = 20;
void draw(){
  fk();
  solve();
  
  background(250,250,250);
  

  fill(50,130,200);
  pushMatrix();
  translate(root.x,root.y);
  rotate(a0);
  rect(0, -armW/2, l0, armW, 100);
  popMatrix();
  
  pushMatrix();
  translate(start_l1.x,start_l1.y);
  rotate(a0+a1);
  rect(0, -armW/2, l1, armW, 100);
  popMatrix();
 
  
  pushMatrix();
  translate(start_l2.x,start_l2.y);
  rotate(a0+a1+a2);
  rect(0, -armW/2, l2, armW, 100);
  popMatrix();
  
  pushMatrix();
  translate(start_l2_left.x,start_l2_left.y);
  rotate(a0+a1+a2_left);
  rect(0, -armW/2, l2_left, armW, 100);
  popMatrix();
  
  pushMatrix();
  translate(start_l3.x,start_l3.y);
  rotate(a0+a1+a2+a3);
  rect(0, -armW/2, l3, armW, 100);
  popMatrix();
  
  pushMatrix();
  translate(start_l3_left.x,start_l3_left.y);
  rotate(a0+a1+a2_left+a3_left);
  rect(0, -armW/2, l3_left, armW, 100);
  popMatrix();
  
  pushMatrix();
  circle(obstacle.x, obstacle.y, obstacleR);
  popMatrix();
  
}
