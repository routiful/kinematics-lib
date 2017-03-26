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
float[] gripper_angle = new float[2];

static int tTime;
int update_period = 250;

void setup()
{
  size(600, 600, P3D);
  smooth();
  
  initShape();
  
  connectOpenCR();
  //setJointAngle(0,0,0,0);
  
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
  
  if ((millis()-tTime) >= (1000 / update_period))
  {
    drawManipulator();
    tTime = millis();
  }
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

void initShape()
{
  link1 = loadShape("meshes/link1.obj");
  link2 = loadShape("meshes/link2.obj");
  link3 = loadShape("meshes/link3.obj");
  link4 = loadShape("meshes/link4.obj");
  link5 = loadShape("meshes/link5.obj");
  gripper     = loadShape("meshes/link6_l.obj");
  gripper_sub = loadShape("meshes/link6_r.obj");
  
  setJointAngle(0, 0, 0, 0);
  gripperOff();
}

void connectOpenCR()
{
  printArray(Serial.list());
  
  String port_name = Serial.list()[3];
  opencr_port = new Serial(this, port_name, 250000);
  opencr_port.bufferUntil('\n');
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
 
  pushMatrix();
  shape(link1);
  
  translate(12, 0, 36);
  rotateZ(radians(joint_angle[0]));
  shape(link2);
  
  translate(0, 3, 41);
  rotateY(radians(joint_angle[1]));
  shape(link3);
  
  translate(23, 0 , 122);
  rotateY(radians(joint_angle[2]));
  shape(link4);
  
  translate(159, 0, 0);
  rotateY(radians(joint_angle[3]));
  shape(link5);
  
  translate(70, 0, 0);
  translate(0, gripper_angle[0], 0);
  shape(gripper);
  
  translate(0, 0, 0);
  translate(0, gripper_angle[1], 0);
  shape(gripper_sub);
  popMatrix();
}

void setJointAngle(float angle1, float angle2, float angle3, float angle4)
{
  joint_angle[0] = angle1;
  joint_angle[1] = angle2;
  joint_angle[2] = angle3;
  joint_angle[3] = angle4;
}

void gripperOn()
{
  gripper_angle[0] = -10;
  gripper_angle[1] = -gripper_angle[0] + 10;
}

void gripperOff()
{
  gripper_angle[0] = -33;
  gripper_angle[1] = -gripper_angle[0] + 33;
}

void gripperJointAngle(float angle)
{
  gripper_angle[0] = angle;
  gripper_angle[1] = -gripper_angle[0] - angle;
}