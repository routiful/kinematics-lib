/**
 * simulation. 
 * 
 * this code receives data from pose_controller.ino
 * to show how to change pose of objects.
*/

import processing.serial.*;

PShape link1, link2, link3, link4, link5, gripper, gripper_sub;
float model_rot_x, model_rot_y, model_trans_x, model_trans_y, model_scale_factor;

Serial opencr_port;
float[] joint_angle = new float[4];

void setup()
{
  size(600, 600, P3D);
  smooth();
  
  link1 = loadShape("meshes/link1.obj");
  link2 = loadShape("meshes/link2.obj");
  link3 = loadShape("meshes/link3.obj");
  link4 = loadShape("meshes/link4.obj");
  link5 = loadShape("meshes/link5.obj");
  gripper     = loadShape("meshes/link6_l.obj");
  gripper_sub = loadShape("meshes/link6_r.obj");
  
  printArray(Serial.list());
  
  String port_name = Serial.list()[3];
  opencr_port = new Serial(this, port_name, 250000);
  opencr_port.bufferUntil('\n');
  
  initView();
}

void draw()
{
  lights();
  smooth();
  background(32);
  
  translate(width/2-100 + model_trans_x, height/2+200 + model_trans_y, 0);
  
  rotateX(radians(90) + model_rot_x);
  rotateY(-model_rot_y);
  
  drawManipulator();
}

void mouseDragged(){
    model_rot_y -= (mouseX - pmouseX) * 0.01;
    model_rot_x -= (mouseY - pmouseY) * 0.01;
}

void keyPressed() {
  if (key == 'a') model_trans_x -= 50;
  else if (key == 'd') model_trans_x += 50;
  else if (key == 's') model_trans_y += 50;
  else if (key == 'w') model_trans_y -= 50;
  else if (key == 'r') model_scale_factor += 1;
  else if (key == 'f') model_scale_factor -= 1;
}

void mouseWheel(MouseEvent event) {
  float e = event.getCount();
  model_scale_factor += e * 1.002;
}

void initView()
{
  float camera_y = height/2.0;
  float fov = 200/float(width) * PI/2;
  float camera_z = camera_y / tan(fov / 2.0);
  float aspect = float(width)/float(height);

  perspective(fov, aspect, camera_z/10.0, camera_z*10.0);
  
  // Eye position
  // Scene center
  // Upwards axis
  camera(width/2.0, height/2.0-500, height/2.0 * 4, 
         width/2, height/2, 0, 
         0, 1, 0);
}

void serialEvent(Serial opencr_port) 
{
  String opencr_string = opencr_port.readStringUntil('\n');
  opencr_string = trim(opencr_string);

  float[] angles = float(split(opencr_string, ','));
  
  for (int joint_num = 0; joint_num < angles.length; joint_num++)
  {
    joint_angle[joint_num] = angles[joint_num];
    print("joint " + (joint_num+1)  + ": " + angles[joint_num] + "\t");
  }
  
  println();
}

void drawManipulator()
{
  scale(1 + model_scale_factor);
  
  shape(link1);
  
  pushMatrix();
  translate(12, 0, 36);
  rotateZ(radians(joint_angle[0]));
  shape(link2);
  popMatrix();
  
  pushMatrix();
  translate(12, 0, 77);
  rotateY(radians(joint_angle[1]));
  shape(link3);
  popMatrix();
  
  pushMatrix();
  translate(35, 0, 199);
  rotateY(radians(joint_angle[2]));
  shape(link4);
  popMatrix();
  
  pushMatrix();
  translate(193, 0, 199);
  rotateY(radians(joint_angle[3]));
  shape(link5);
  popMatrix();
  
  pushMatrix();
  translate(263, 0, 199);
  translate(0,-30,0);
  shape(gripper);
  popMatrix();
  
  pushMatrix();
  translate(263, 0, 199);
  translate(0,30,0);
  shape(gripper_sub);
  popMatrix();
}