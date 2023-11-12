//FK & IK 
//CSCI 5611 3-link IK Chain follows mouse [Example]
// Stephen J. Guy <sjguy@umn.edu>


//Root
Vec2 root = new Vec2(50,50);
Vec2 endPoint;

// Generalize
static int LINKS = 4;
float[] angles = new float[LINKS];
Vec2[] starts = new Vec2[LINKS];

// Lengths
float[] lengths = new float[LINKS];
float start_arm_length = 200;
float arm_decr = 40;


//class Arm {
//  Vec2 start, end;
//  Arm prev, next;
  
//  Arm(Vec2 start, Vec2 end) {
//    this.start = start;
//    this.end = end;
//  }
//}





// Camera
Camera mainCamera;

void setup(){
  size(1000,700,P3D);
  surface.setTitle("Inverse Kinematics");
  mainCamera = new Camera();
  mainCamera.position = new PVector(300, 400, 1000);
  //background(100);  
  lights();
  
  starts[0] = root;
  
  lengths[0] = start_arm_length;
  for (int i=1; i < LINKS; i++) {
    lengths[i] = lengths[i-1] - arm_decr;
    
    float curr_angle = random(-PI, PI);
    Vec2 direction = new Vec2(cos(curr_angle)*lengths[i], sin(curr_angle)*lengths[i]);
    starts[i] = direction.plus(starts[i-1]);
  }
    
    
    
}

float armW = 20;
void draw(){
  //fk();
  solve();
  
  background(255,255,255);
  mainCamera.Update(1.0/frameRate);
  
  fill(180,20,40); //Root
  pushMatrix();
    translate(root.x,root.y);
    rect(-20,-20,40,40);
  popMatrix();
  
  
  fill(10,150,40); //Green IK Chain
  strokeWeight(5);
  for (int i = 0; i < LINKS; i++) 
  {
    //float curr_angle = 0;
    //for (int j = 0; j <= i; j++) 
    //  curr_angle += angles[j];
    strokeWeight(20-5*i);
    if (i == LINKS - 1)
      line(starts[i].x, starts[i].y, 0, endPoint.x, endPoint.y, 0);
    else
      line(starts[i].x, starts[i].y, 0, starts[i+1].x, starts[i+1].y, 0);
    
    //pushMatrix();
    //  translate(starts[i].x, starts[i].y);      
    //  rotate(curr_angle);
    //  quad(0, -armW/2, lengths[i], -.1*armW, lengths[i], .1*armW, 0, armW/2);
    //popMatrix();
  }
  
  
  fill(0,0,0); //Goal/mouse
  pushMatrix();
    translate(mouseX,mouseY);
    sphere(30);
  popMatrix();
  
}


void keyPressed() {
  mainCamera.HandleKeyPressed();
}

void keyReleased() {
  mainCamera.HandleKeyReleased();
}
