//Inverse Kinematics
//CSCI 5611 
//Use this tutorial to better understand the translate() function
//http://btk.tillnagel.com/tutorials/rotation-translation-matrix.html
//camera
Camera mainCamera;
void setup(){
  size(800,800,P3D);
  surface.setTitle("Inverse Kinematics");
  mainCamera = new Camera();
  mainCamera.position = new PVector(300, 400, 1000);
  //background(100);  
  lights();
}
//sphere
Vec2 ball = new Vec2(200,200);

//Root
Vec2 root = new Vec2(400,400);

//Upper Arm
float l0 = 100; 
float a0 = 0.3; //Shoulder joint

//Lower Arm
float l1 = 100;
float a1 = 0.3; //Elbow joint

//Hand
float l2 = 100;
float a2 = 0.3; //Wrist joint

//Finger
float l3 = 100;
float a3 = 0.3; //finger joint

Vec2 start_l1,start_l2,start_l3,endPoint;

void solve(){
  Vec2 goal = new Vec2(mouseX, mouseY);
  
  Vec2 startToGoal, startToEndEffector;
  float dotProd, angleDiff;
  
  //Update finger joint
  startToGoal = goal.minus(start_l3);
  startToEndEffector = endPoint.minus(start_l3);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  if (cross(startToGoal,startToEndEffector) < 0)
    a3 += angleDiff;
  else
    a3 -= angleDiff;
  /*TODO: Finger joint limits here*/
  fk(); //Update link positions with fk (e.g. end effector changed)
  
  
  //Update wrist joint
  startToGoal = goal.minus(start_l2);
  startToEndEffector = endPoint.minus(start_l2);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  if (cross(startToGoal,startToEndEffector) < 0)
    a2 += angleDiff;
  else
    a2 -= angleDiff;
  fk(); //Update link positions with fk (e.g. end effector changed)
  
  
  
  //Update elbow joint
  startToGoal = goal.minus(start_l1);
  startToEndEffector = endPoint.minus(start_l1);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  if (cross(startToGoal,startToEndEffector) < 0)
    a1 += angleDiff;
  else
    a1 -= angleDiff;
  fk(); //Update link positions with fk (e.g. end effector changed)
  
  
  //Update shoulder joint
  startToGoal = goal.minus(root);
  if (startToGoal.length() < .0001) return;
  startToEndEffector = endPoint.minus(root);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  //if (cross(startToGoal,startToEndEffector) > 0)
  //  //a0 -= angleDiff;
  //else
  //  a0 += angleDiff;
  /*TODO: Shoulder joint limits here*/
  fk(); //Update link positions with fk (e.g. end effector changed)
 
  println("Angle 0:",a0,"Angle 1:",a1,"Angle 2:",a2,"Angle 3:", a3);
}

void fk(){
  start_l1 = new Vec2(cos(a0)*l0,sin(a0)*l0).plus(root);
  start_l2 = new Vec2(cos(a0+a1)*l1,sin(a0+a1)*l1).plus(start_l1);
  start_l3 = new Vec2(cos(a0+a1+a2)*l2,sin(a0+a1+a2)*l2).plus(start_l2);
  endPoint = new Vec2(cos(a0+a1+a2+a3)*l3,sin(a0+a1+a2+a3)*l3).plus(start_l3);
}

float armW = 20;
void draw(){
  fk();
  solve();
  
  background(250,250,250);
  mainCamera.Update(1.0/frameRate);
  pointLight(255, 255, 255, 300, 400, 800);
  
  noStroke();
  fill(50,130,200);
  pushMatrix();
  translate(root.x-l1/2,root.y-10, 0);
  rotate(a0); 
  translate(l1/2,10, 0);
  //rect(0, -armW/2, l0, armW, 100);
  box(l0, armW, armW);
  popMatrix();
  
  pushMatrix();
  translate(start_l1.x-l1/2,start_l1.y-10, 0);
  rotate(a0+a1);
  translate(l1/2,10, 0);
  box(l1, armW, armW);
  //rect(0, -armW/2, l1, armW, 100);
  popMatrix();
  
  pushMatrix();
  translate(start_l2.x-l2/2,start_l2.y-10, 0);
  rotate(a0+a1+a2);
  translate(l2/2,10, 0);
  box(l2, armW, armW);
  //rect(0, -armW/2, l2, armW, 100);
  popMatrix();
  
  pushMatrix();
  translate(start_l3.x-l3/2,start_l3.y-10, 0);
  rotate(a0+a1+a2+a3);
  translate(l3/2,10, 0);
  box(l3, armW, armW);
  popMatrix();
  
  //fill(10,0,0); //Goal/mouse
  //pushMatrix();
  //translate(ball.x,ball.y,0);
  //sphere(30);
  //popMatrix();
  
}

void keyPressed() {
      //if (key == 'u') {
      //  ball.x = ball.y-10;
      
      //} else if (key == 'h') {
      //  ball.y = ball.x-10;

      //} else if (key == 'k' ) {
      //  ball.y = ball.y+10;

      //} else if (key == 'j') {
      //  ball.x = ball.x+10;

      //}
    mainCamera.HandleKeyPressed();
}

void keyReleased() {
    mainCamera.HandleKeyReleased();
}
