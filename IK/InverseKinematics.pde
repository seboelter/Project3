//Inverse Kinematics
//CSCI 5611 
//Use this tutorial to better understand the translate() function
//http://btk.tillnagel.com/tutorials/rotation-translation-matrix.html
//camera
Camera mainCamera;
void setup(){
  size(1000,700,P3D);
  surface.setTitle("Inverse Kinematics");
  mainCamera = new Camera();
  mainCamera.position = new PVector(300, 400, 1000);
  //background(100);  
  lights();
}
//sphere
Vec3 ball = new Vec3(630,70, 10);

//Root
Vec3 root = new Vec3(300,300, 0);

//Upper Arm
float l0 = 70; 
float a0 = 0.0; //Shoulder joint
float a0z =0.0;

//Lower Arm
float l1 = 70;
float a1 = 0.0; //Elbow joint
float a1z = 0.0;

//Hand
float l2 = 70;
float a2 = 0.0; //Wrist joint
float a2z = 0.0;

//Finger
float l3 = 70;
float a3 = 0.0; //finger joint
float a3z = 0.0;

Vec2 x1, x2, y1,y2;
float dotx, doty;

Vec3 start_l1,start_l2,start_l3,endPoint, start_l1z,start_l2z,start_l3z,endPointz;

void solve(){
  Vec3 goal = new Vec3(ball.x, ball.y,ball.z);
  
  Vec3 startToGoal, startToEndEffector;
  float dotProd, angleDiff, angleDiffZ;
  
  //Update finger joint
  //change this into rotating
  startToGoal = goal.minus(start_l3);
  startToEndEffector = endPoint.minus(start_l3);
  x1 = new Vec2(startToGoal.x,startToGoal.y);
  x2 = new Vec2(startToEndEffector.x,startToEndEffector.y);
  y1 = new Vec2(startToGoal.x,startToGoal.y);
  y2 = new Vec2(startToEndEffector.x,startToEndEffector.y);
  dotx = dot(x1.normalized(), x2.normalized());
  doty = dot(y1.normalized(), y2.normalized());
  dotx = clamp(dotx,-1,1);
  doty = clamp(doty,-1,1);
  angleDiff = acos(dotx);
  angleDiffZ = acos(doty);
  //dotProd = dot3(startToGoal.normalized(),startToEndEffector.normalized());
  //dotProd = clamp(dotProd,-1,1);
  //angleDiff = acos(dotProd);
  //angle restrictions
  //if (cross3(startToGoal,startToEndEffector) < 0 && (a3+angleDiff < 0.1))
  //  a3 += angleDiff;
  //else if (a3 -angleDiff >-1.5)
  //  a3 -= angleDiff;
  //else
  //  a3 = a2;
  if (cross(x1,x2) < 0)
    a3 += angleDiff;
  else
    a3 -= angleDiff;
       
  if (cross(y1,y2)<0)
    a3z += angleDiffZ;
  //else if (a3 -angleDiff >-1.5)
  //  a3z -= angleDiffZ;
  else
    a3z -= angleDiffZ;
  fk(); 
  
  
  //Update wrist joint
  startToGoal = goal.minus(start_l2);
  startToEndEffector = endPoint.minus(start_l2);
  x1 = new Vec2(startToGoal.x,startToGoal.y);
  x2 = new Vec2(startToEndEffector.x,startToEndEffector.y);
  y1 = new Vec2(startToGoal.x,startToGoal.y);
  y2 = new Vec2(startToEndEffector.x,startToEndEffector.y);
  dotx = dot(x1.normalized(), x2.normalized());
  doty = dot(y1.normalized(), y2.normalized());
  dotx = clamp(dotx,-1,1);
  doty = clamp(doty,-1,1);
  angleDiff = acos(dotx);
  angleDiffZ = acos(doty);
  //dotProd = dot3(startToGoal.normalized(),startToEndEffector.normalized());
  //dotProd = clamp(dotProd,-1,1);
  //angleDiff = acos(dotProd);
  //angle restrictions
  //if (cross3(startToGoal,startToEndEffector) < 0 && (a2+angleDiff < 0.1))
  //  a2 += angleDiff;
  //else if (a2 -angleDiff >-1.5)
  //  a2 -= angleDiff;
  //else
  //  a2 = a2;
  //no angle restrictions
  //if (cross3(startToGoal,startToEndEffector) < 0)
    //a2 += angleDiff;
  //else 
   //a2 -= angleDiff;
  if (cross(x1,x2) < 0)
    a2 += angleDiff;
  else
    a2 -= angleDiff;
       
  if (cross(y1,y2)<0)
    a2z += angleDiffZ;
  //else if (a3 -angleDiff >-1.5)
  //  a3z -= angleDiffZ;
  else
    a2z -= angleDiffZ;

  fk(); //Update link positions with fk (e.g. end effector changed)
  
  
  
  //Update elbow joint
  startToGoal = goal.minus(start_l1);
  startToEndEffector = endPoint.minus(start_l1);
  x1 = new Vec2(startToGoal.x,startToGoal.y);
  x2 = new Vec2(startToEndEffector.x,startToEndEffector.y);
  y1 = new Vec2(startToGoal.x,startToGoal.y);
  y2 = new Vec2(startToEndEffector.x,startToEndEffector.y);
  dotx = dot(x1.normalized(), x2.normalized());
  doty = dot(y1.normalized(), y2.normalized());
  dotx = clamp(dotx,-1,1);
  doty = clamp(doty,-1,1);
  angleDiff = acos(dotx);
  angleDiffZ = acos(doty);
  //dotProd = dot3(startToGoal.normalized(),startToEndEffector.normalized());
  //dotProd = clamp(dotProd,-1,1);
  //angleDiff = acos(dotProd);
  //angle restrictions
  //if (cross3(startToGoal,startToEndEffector) < 0 && (a1+angleDiff <0.1))
  //  a1 += angleDiff;
  //else if (a1-angleDiff >-1.5)
  //  a1 -= angleDiff;
  //else
  //  a1 = a1;
  
  if (cross3(startToGoal,startToEndEffector) < 0)
    a1 += angleDiff;
  else 
    a1 -= angleDiff;
  if (cross(y1,y2)<0)
    a1z += angleDiffZ;
  //else if (a3 -angleDiff >-1.5)
  //  a3z -= angleDiffZ;
  else
    a1z -= angleDiffZ;

  fk(); 
  
  
  //Update shoulder joint
  startToGoal = goal.minus(root);
  if (startToGoal.length3() < .0001) return;
  startToEndEffector = endPoint.minus(root);
  dotProd = dot3(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  //if (cross3(startToGoal,startToEndEffector) > 0 && (a0+angleDiff > 0))
  //  a0 = a0;
  //else //if (a0-angleDiff <3)
  //  a0= a0;
  
  /*TODO: Shoulder joint limits here*/
  fk(); //Update link positions with fk (e.g. end effector changed)
 
  println("Angle 0:",a0,"Angle 1:",a1,"Angle 2:",a2,"Angle 3:", a3);
  println("AngleZ:",a3z);
  println("Ball X:",ball.x,"Ball Y:",ball.y,"Ball Z:",ball.z);
}

void fk(){
  start_l1 = new Vec3(cos(a0)*l0,sin(a0)*l0,0).plus(root);
  start_l2 = new Vec3(cos(a0+a1)*l1,sin(a0+a1)*l1,0).plus(start_l1);
  start_l3 = new Vec3(cos(a0+a1+a2)*l2,sin(a0+a1+a2)*l2,0).plus(start_l2);
  endPoint = new Vec3(cos(a0+a1+a2+a3)*l3,sin(a0+a1+a2+a3)*l3,0).plus(start_l3);
  
  start_l1z = new Vec3(0,cos(a0z),sin(a0z));
  start_l2z = new Vec3(0,cos(a0z+a1z),sin(a0z+a1z));
  start_l3z = new Vec3(0,cos(a0z+a1z+a2z),sin(a0z+a1z+a2z));
  endPointz = new Vec3(0,cos(a0z+a1z+a2z+a3z),sin(a0z+a1z+a2z+a3z));
  
  //start_l1 = cross3(start_l1, start_l1z);
  //start_l2 = start_l2z.plus(start_l2);
  //start_l3 = start_l3z.plus(start_l3);
  //endPoint = endPointz.plus(endPoint);
  
  
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
  rotateZ(a0); 
  //rotateX(a0);
  rotateY(a0z);
  translate(l1/2,10, 0);
  //rect(0, -armW/2, l0, armW, 100);
  box(l0, armW, armW);
  popMatrix();
  
  pushMatrix();
  translate(start_l1.x-l1/2,start_l1.y-10,start_l1.z -10);
  rotateZ(a0+a1);
  //rotateX(a0+a1);
  //rotateY(a0z+a1z);
  //translate(l1/2,10, 10);
  //translate(start_l1z.x-l1/2,start_l1z.y-10,start_l1z.z -10);
  rotateY(a0z+a1z);
  translate(l1/2,10, 10);
  box(l1, armW, armW);
  popMatrix();
  
  
  pushMatrix();
  translate(start_l2.x-l2/2,start_l2.y-10,cos(a2)*(l2) -10);
  rotateZ(a0+a1+a2);
  translate(l2/2,10, 10);
  translate(start_l2z.x-l2/2,start_l2z.y-10,start_l2z.z -10);
  rotateY(a0z+a1z+a2z);
  //rotateX(a0+a1+a2);
  translate(l2/2,10, 10);
  box(l2, armW, armW);
  popMatrix();
  
  pushMatrix();
  translate(start_l3.x-l3/2,start_l3.y-10,start_l3.z -10);
  rotateZ(a0+a1+a2+a3);
  //rotateX(a0+a1+a2+a3);
  rotateY(a0z+a1z+a2z+a3z);
  translate(l3/2,10,10);
  box(l3, armW, armW);
  popMatrix();
  

  fill(10,100,0); //Goal/mouse
  pushMatrix();
  translate(ball.x,ball.y,ball.z);
  sphere(30);
  popMatrix();
  
}

void keyPressed() {
      if (key == 'u') {
        ball.x = ball.x-10;
      
      } else if (key == 'h') {
        ball.y = ball.y-10;

      } else if (key == 'k' ) {
        ball.y = ball.y+10;

      } else if (key == 'j') {
        ball.x = ball.x+10;
        
      } else if (key == 'y') {
        ball.z = ball.z+10;
        
      } else if (key == 'i') {
        ball.z = ball.z-10;

      }
    mainCamera.HandleKeyPressed();
}

void keyReleased() {
    mainCamera.HandleKeyReleased();
}
