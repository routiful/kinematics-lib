/**
 * simulation. 
 * 
 * this code receives data from pose_controller.ino
 * to show how to change pose of objects.
*/

PShape link1, link2, link3, link4, link5, gripper, gripper_sub;
float rotX, rotY, transX, transY, scaleFactor;

void setup()
{
  size(600, 600, P3D);
  smooth();
  
  link1 = loadShape("meshes/chain_link1.obj");
  link2 = loadShape("meshes/chain_link2.obj");
  link3 = loadShape("meshes/chain_link3.obj");
  link4 = loadShape("meshes/chain_link4.obj");
  link5 = loadShape("meshes/chain_link5.obj");
  //gripper     = loadShape("meshes/chain_link_grip1.obj");
  //gripper_sub = loadShape("meshes/chain_link_grip2.obj");
  
  initView();
}

void draw()
{
  lights();
  background(32);
  
  translate(width/2 + transX, height/2 + 200 + transY, 0);
  
  rotateX(radians(90) + rotX);
  rotateY(-rotY);
  
  scale(1 + scaleFactor);
  
  //rotateY(radians(frameCount));
  shape(link1);
  
  pushMatrix();
  translate(12, 0, 36);
  //rotateZ(radians(frameCount));
  shape(link2);
  popMatrix();
  
  pushMatrix();
  translate(12, 0, 77);
  //rotateY(radians(frameCount));
  shape(link3);
  popMatrix();
  
  pushMatrix();
  translate(35, 0, 199);
  //rotateY(radians(frameCount));
  shape(link4);
  popMatrix();
  
  pushMatrix();
  translate(0, 0, 0);
  shape(link5);
  popMatrix();
}

void mouseDragged(){
    rotY -= (mouseX - pmouseX) * 0.01;
    rotX -= (mouseY - pmouseY) * 0.01;
}

void keyPressed() {
  if (key == 'a') transX -= 10;
  else if (key == 'd') transX += 30;
  else if (key == 's') transY += 30;
  else if (key == 'w') transY -= 30;
  else if (key == 'r') scaleFactor += 1;
  else if (key == 'f') scaleFactor -= 1;
}

void mouseWheel(MouseEvent event) {
  float e = event.getCount();
  scaleFactor += e * 1.002;
}

void initView()
{
  float cameraY = height/2.0;
  float fov = 200/float(width) * PI/2;
  float cameraZ = cameraY / tan(fov / 2.0);
  float aspect = float(width)/float(height);

  perspective(fov, aspect, cameraZ/10.0, cameraZ*10.0);
  
  // Eye position
  // Scene center
  // Upwards axis
  camera(width/2.0, height/2.0-500, height/2.0 * 4, 
         width/2, height/2, 0, 
         0, 1, 0);
}