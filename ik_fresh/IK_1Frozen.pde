//FK & IK 
//CSCI 5611 3-link IK Chain follows mouse [Example]
// Stephen J. Guy <sjguy@umn.edu>

Vec2 root = new Vec2(300,400);
float armW = 20;


class Circle{
  Vec2 origin;
  float radius;

  Circle(Vec2 origin, float radius) {
    this.origin = origin;
    this.radius = radius;
  }

}

class Arm {
  float angle, length, originalAngle;
  Vec2 start, end;

  Arm prev = null;
  ArrayList<Arm> next = new ArrayList<Arm>();

  Arm(Vec2 start, float length, float angle) {
    this.start = start;
    this.length = length;
    this.angle = angle;
    this.originalAngle = angle;
    calculateEnd();
  }

  // Calculate the end based on the start, length, and angle
  void calculateEnd() {
    float dx = cos(angle) * length;
    float dy = sin(angle) * length;
    end = new Vec2(start.x + dx, start.y + dy);

    // Update the next arms
    for (Arm arm : next) {
      arm.start = end;
      arm.calculateEnd();
    }

  }

  // Storing the next arms in a list
  void addNext(Arm next) {
    this.next.add(next);
    next.prev = this;
  }

  void change_angle(float a){
    originalAngle = a;
    forwardKinematics();
  }

  void forwardKinematics(){
    angle = originalAngle;
    if (prev != null) {
      angle += prev.angle;
      // start = prev.end;
    }
    calculateEnd();
  }

  // Forward Kinematics
  void addAngle(float a) {
    angle += a;
    calculateEnd();
    for (Arm arm : next) {
      arm.addAngle(a);
    }
  }


  void inverseKinematics(Vec2 goal) {
    Vec2 start_to_goal = goal.minus(start);
    Vec2 start_to_end = end.minus(start);

    float dotProduct = dot(start_to_goal.normalized(), start_to_end.normalized());
    dotProduct = clamp(dotProduct, -1, 1);

    float tempAngle = angle;
    if (cross(start_to_goal, start_to_end) < 0) {
      tempAngle += acos(dotProduct);
    } else {
      tempAngle -= acos(dotProduct);
    }

    // Angle restrictions
    float minAngle = -PI/2;
    float maxAngle = PI/2;
    
    // Shoulder joint (0th element) angle restriction
    // tempAngle = clamp(tempAngle, minAngle, maxAngle);
    change_angle(tempAngle);

    if (prev != null) {
      prev.inverseKinematics(goal);
    }
  }

  // Draw the arm
  void draw() {

    println(start + " -> " + end);
    pushMatrix();
      translate(start.x, start.y);
      rotate(angle);
      rect(0, -armW/2, length, armW, 100);
    popMatrix();

    for (Arm arm : next) {
      arm.draw();
    }
  }

  // Checks if any side and center-line of the arm is colliding with the circle
  boolean circleCollision(Circle c) {

    // Minkowski sum of the circle and half the arm width
    // Ensures that the circle is at least armW/2 away from the arm
    Circle c2 = new Circle(c.origin, c.radius + armW/2);
    return lineCircleCollision(c2, start, end);
  }

}


boolean lineCircleCollision(Circle circle, Vec2 l_start, Vec2 l_dir){
  
  Vec2 center = circle.origin;
  float r = circle.radius;

  //Compute displacement vector pointing from the start of the line segment to the center of the circle
  Vec2 toCircle = center.minus(l_start);
 
  //Solve quadratic equation for intersection point (in terms of l_dir and toCircle)
  float a = l_dir.length()*l_dir.length();
  float b = -2*dot(l_dir,toCircle); //-2*dot(l_dir,toCircle)
  float c = toCircle.lengthSqr() - (r*r); //different of squared distances
 
  float d = b*b - 4*a*c; //discriminant
 
  if (d >= 0){
    //If d is positive we know the line is colliding
    float t = (-b - sqrt(d))/(2*a); //Optimization: we typically only need the first collision!
    if (t >= 0) return true;
    return false;
  }
 
  return false; //We are not colliding, so there is no good t to return
}




Arm shoulder = new Arm(root, 100, -1.1);
Arm elbow = new Arm(shoulder.end, 100, 0);
Arm wrist1 = new Arm(elbow.end, 100, 0.3);
Arm wrist2 = new Arm(elbow.end, 100, 0.2);
Arm finger1 = new Arm(wrist1.end, 100, 0.1);
Arm finger2 = new Arm(wrist2.end, 100, -0.3);



int armLength = 30;
Arm root1 = new Arm(new Vec2(300,400), armLength, 0);


void setup(){
  size(1000,700);
  surface.setTitle("Inverse Kinematics");


  // Connect the arms
  shoulder.addNext(elbow);
  elbow.addNext(wrist1);
  // elbow.addNext(wrist2);
  wrist1.addNext(finger1);
  // wrist2.addNext(finger2);

  // New arm
  // Arm current = root1;
  // for (int i = 0; i < 10; i++) {
  //   Arm next = new Arm(current.end, armLength, random(-PI/10, PI/10));
  //   current.addNext(next);
  //   current = next;
  // }

}


void draw(){
  //fk();
  solve();
  
  background(255,255,255);

  fill(180,20,40);
  pushMatrix();
    translate(root.x,root.y);
    rect(-20,-20,40,40);
  popMatrix();
  
  
  fill(50,130,200);

  // addAngle does fk by adding propagating the angles to all the next arms
  // shoulder.addAngle(0.01);
  Vec2 goal = new Vec2(mouseX, mouseY);
  finger1.inverseKinematics(goal);
  wrist1.inverseKinematics(goal);
  // finger2.inverseKinematics(goal);
  // finger2.inverseKinematics(goal);
  // wrist1.inverseKinematics(goal);
  // elbow.inverseKinematics(goal);
  // shoulder.inverseKinematics(goal);

  // shoulder.change_angle(0.01);
  // finger1.update(goal);
  // finger2.update(goal);
  // wrist1.update(goal);
  // wrist2.update(goal);
  // elbow.update(goal);
  // shoulder.update(goal);
  // shoulder.calculateEnd();


  // shoulder.fk();

  // shoulder.draw();
  // elbow.draw();
  // wrist1.draw();
  // wrist2.draw();
  // finger1.draw();
  // finger2.draw();

  // root1.draw();
  shoulder.draw();
  
  
  fill(0,0,0); //Goal/mouse
  pushMatrix();
    translate(mouseX,mouseY);
    circle(0,0,20);
  popMatrix();
  
}
