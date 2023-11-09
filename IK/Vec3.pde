//Vector Library
//CSCI 5611 Vector 2 Library 
//Created by Stephen J. Guy <sjguy@umn.edu>

public class Vec3 {
  public float x, y,z;
  
  public Vec3(float x, float y, float z){
    this.x = x;
    this.y = y;
    this.z = z;
  }
  
  //public String toString(){
  //  return "(" + x+ "," + y +")";
  //}
  
  public float length3(){
    return sqrt(x*x+y*y+z*z);
  }
  
  public Vec3 plus(Vec3 rhs){
    return new Vec3(x+rhs.x, y+rhs.y, z+rhs.z);
  }
  
  //public void add(Vec2 rhs){
  //  x += rhs.x;
  //  y += rhs.y;
  //}
  
  public Vec3 minus(Vec3 rhs){
    return new Vec3(x-rhs.x, y-rhs.y, z-rhs.z);
  }
  
  //public void subtract(Vec2 rhs){
  //  x -= rhs.x;
  //  y -= rhs.y;
  //}
  
  //public Vec2 times(float rhs){
  //  return new Vec2(x*rhs, y*rhs);
  //}
  
  //public void mul(float rhs){
  //  x *= rhs;
  //  y *= rhs;
  //}
  
  //public void clampToLength(float maxL){
  //  float magnitude = sqrt(x*x + y*y);
  //  if (magnitude > maxL){
  //    x *= maxL/magnitude;
  //    y *= maxL/magnitude;
  //  }
  //}
  
  //public void setToLength(float newL){
  //  float magnitude = sqrt(x*x + y*y);
  //  x *= newL/magnitude;
  //  y *= newL/magnitude;
  //}
  
  //public void normalize(){
  //  float magnitude = sqrt(x*x + y*y);
  //  x /= magnitude;
  //  y /= magnitude;
  //}
  
  public Vec3 normalized(){
    float magnitude = sqrt(x*x + y*y + z*z);
    return new Vec3(x/magnitude, y/magnitude, z/magnitude);
  }
  
  //public float distanceTo(Vec2 rhs){
  //  float dx = rhs.x - x;
  //  float dy = rhs.y - y;
  //  return sqrt(dx*dx + dy*dy);
  //}
}

//Vec2 interpolate(Vec2 a, Vec2 b, float t){
//  return a.plus((b.minus(a)).times(t));
//}

//float interpolate(float a, float b, float t){
//  return a + ((b-a)*t);
//}

float dot3(Vec3 a, Vec3 b){
  return a.x*b.x + a.y*b.y + a.z*b.z;
}

float cross3(Vec3 a, Vec3 b){
  return (a.y*b.z - a.z*b.x) - (a.x*b.z - a.z*b.x) +(a.x*b.y-a.y*b.x);
}


//Vec2 projAB(Vec2 a, Vec2 b){
//  return b.times(a.x*b.x + a.y*b.y);
//}

//float clamp(float f, float min, float max){
//  if (f < min) return min;
//  if (f > max) return max;
//  return f;
//}
