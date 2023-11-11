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

void setup(){
  size(640,480);
  surface.setTitle("Inverse Kinematics [CSCI 5611 Example]");
  strokeWeight(2);
  
  starts[0] = root;
  
  lengths[0] = start_arm_length;
  for (int i=1; i < LINKS; i++)
    lengths[i] =  lengths[i-1] - arm_decr;
    
}

float armW = 20;
void draw(){
  fk();
  solve();
  
  background(255,255,255);
  
  fill(180,20,40); //Root
  pushMatrix();
    translate(root.x,root.y);
    rect(-20,-20,40,40);
  popMatrix();
  
  
  fill(10,150,40); //Green IK Chain
  for (int i = 0; i < LINKS; i++) 
  {
    float curr_angle = 0;
    for (int j = 0; j <= i; j++) 
      curr_angle += angles[j];
        
    pushMatrix();
      translate(starts[i].x, starts[i].y);      
      rotate(curr_angle);
      quad(0, -armW/2, lengths[i], -.1*armW, lengths[i], .1*armW, 0, armW/2);
    popMatrix();
  }
  
  
  fill(0,0,0); //Goal/mouse
  pushMatrix();
    translate(mouseX,mouseY);
    circle(0,0,20);
  popMatrix();
  
}
