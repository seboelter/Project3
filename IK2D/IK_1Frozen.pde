//FK & IK 
//CSCI 5611 3-link IK Chain follows mouse [Example]
// Stephen J. Guy <sjguy@umn.edu>


//Root
Vec3 root = new Vec3(50,50,50);
Vec3 endPoint;

// Generalize
static int LINKS = 4;
float[] angles = new float[LINKS];
Vec3[] starts = new Vec3[LINKS];

// Lengths
float[] lengths = new float[LINKS];
float start_arm_length = 200;
float arm_decr = 40;


// Camera
Camera mainCamera;

void setup(){
  size(1000,700,P3D);
  surface.setTitle("Inverse Kinematics");
  mainCamera = new Camera();
  mainCamera.position = new PVector(300, 400, 500);
  //background(100);  
  lights();
  
  starts[0] = root;
  
  lengths[0] = start_arm_length;
  for (int i=1; i < LINKS; i++) {
    lengths[i] = lengths[i-1] - arm_decr;
    
    float curr_angle = random(-PI, PI);
    Vec3 direction = new Vec3(cos(curr_angle)*lengths[i], sin(curr_angle)*lengths[i], 0);
    starts[i] = direction.plus(starts[i-1]);
  }
    
    
    
}

float armW = 20;
void draw(){
  //fk();
  solve();
  
  background(255,255,255);
  mainCamera.Update(1.0/frameRate);
  pointLight(255, 255, 255, 300, 400, 800);
  
  
  drawGrid();
  
  fill(180,20,40); //Root
  pushMatrix();
    translate(root.x,root.y,root.z);
    rect(-20,-20,40,40);
  popMatrix();
  
  
  fill(10,150,40); //Green IK Chain
  strokeWeight(5);
  for (int i = 0; i < LINKS; i++) 
  {
    strokeWeight(30-5*i);
    Vec3 toPoint = (i == LINKS-1)? endPoint : starts[i+1];
    line(starts[i].x, starts[i].y, starts[i].z, toPoint.x, toPoint.y, toPoint.z);
  }
  
  
  fill(0,0,0); //Goal/mouse
  pushMatrix();
    translate(mouseX,mouseY,0);
    sphere(30);
  popMatrix();
  
}


void keyPressed() {
  mainCamera.HandleKeyPressed();
}

void keyReleased() {
  mainCamera.HandleKeyReleased();
}










int gridSize = 10;
int gridSpacing = 50;

void drawGrid() {
  stroke(150);
  strokeWeight(1);
  for (int i = -gridSize; i <= gridSize; i++) {
    line(i * gridSpacing, -gridSize * gridSpacing, 0, i * gridSpacing, gridSize * gridSpacing, 0);
    line(-gridSize * gridSpacing, i * gridSpacing, 0, gridSize * gridSpacing, i * gridSpacing, 0);
  }
}
