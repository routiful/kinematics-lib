/**
 * simulation.
 *
 * this code receives data from open_manipulator_chain.ino
 * to show how to change pose of objects.
*/

import processing.serial.*;

// Shape variable
PShape link1, link2, link3, link4, link5, gripper, gripper_sub;

// Model pose
float model_rot_x, model_rot_z, model_trans_x, model_trans_y, model_scale_factor;

// Serial variable
Serial opencr_port;

// Angle variable
float[] joint_angle = new float[4];
float[] gripper_pos = new float[2];

// Simulation frequency
static int tTime;
int update_period = 300;

void setup()
{
  size(600, 600, OPENGL);
  smooth();

  initShape();
  initView();

  connectOpenCR(0);
}

void draw()
{
  setWindow();

  drawTitle();
  drawWorldFrame();

  if ((millis()-tTime) >= (1000 / update_period))
  {
    drawManipulator();
    tTime = millis();
  }
}

void mouseDragged()
{
  model_rot_z -= (mouseX - pmouseX) * 0.01;
  model_rot_x -= (mouseY - pmouseY) * 0.01;
}

void keyPressed()
{
  if      (key == 'a') model_trans_x -= 50;
  else if (key == 'd') model_trans_x += 50;
  else if (key == 's') model_trans_y += 50;
  else if (key == 'w') model_trans_y -= 50;
  else if (key == 'q') model_scale_factor += 0.5;
  else if (key == 'e') model_scale_factor -= 0.5;
  else if (key == 'i') model_trans_x = model_trans_y = model_scale_factor = model_rot_z = model_rot_x = 0;
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
         width/2-100, height/2, 0,
         0, 1, 0);
}

void initShape()
{
  link1       = loadShape("meshes/chain/link1.obj");
  link2       = loadShape("meshes/chain/link2.obj");
  link3       = loadShape("meshes/chain/link3.obj");
  link4       = loadShape("meshes/chain/link4.obj");
  link5       = loadShape("meshes/chain/link5.obj");
  gripper     = loadShape("meshes/chain/link6_l.obj");
  gripper_sub = loadShape("meshes/chain/link6_r.obj");

  setJointAngle(0, 0, 0, 0);
  gripperOff();
}

void connectOpenCR(int port_num)
{
  printArray(Serial.list());

  String port_name = Serial.list()[port_num];
  opencr_port = new Serial(this, port_name, 57600);
  opencr_port.bufferUntil('\n');

  opencr_port.write("ready");
}

void serialEvent(Serial opencr_port)
{
  String opencr_string = opencr_port.readStringUntil('\n');
  opencr_string = trim(opencr_string);

  float[] angles = float(split(opencr_string, ','));

  for (int joint_num = 0; joint_num < angles.length; joint_num++)
  {
    if (joint_num == angles.length-1)
    {
      int grip_num = joint_num;
      gripperAngle2Pos(angles[grip_num]);
      print("gripper : " + angles[grip_num] + "\n");
    }
    else
    {
      joint_angle[joint_num] = angles[joint_num];
      print("joint " + (joint_num+1)  + ": " + angles[joint_num] + "\t");
    }
  }
}

void setWindow()
{
  lights();
  smooth();
  background(30);

  translate(width/2, height/2, 0);

  rotateX(radians(90));
  rotateZ(radians(140));
}

void drawTitle()
{
  pushMatrix();
  rotateX(radians(0));
  rotateZ(radians(180));
  textSize(60);
  fill(255,204,102);
  text("OpenManipulator Chain", -450,75,0);
  textSize(40);
  fill(102,255,255);
  text("Press 'A','D','W','S'", -370,150,0);
  text("And   'Q','E'",         -370,225,0);
  popMatrix();
}

void drawManipulator()
{
  scale(1 + model_scale_factor);

  pushMatrix();
  translate(-model_trans_x, -model_trans_y, 0);
  rotateX(model_rot_x);
  rotateZ(model_rot_z);
  shape(link1);
  drawLocalFrame();

  translate(12, 0, 36);
  rotateZ(-joint_angle[0]);
  shape(link2);
  drawLocalFrame();

  translate(0, 2, 40);
  rotateY(-joint_angle[1]);
  shape(link3);
  drawLocalFrame();

  translate(22, 0 , 122);
  rotateY(-joint_angle[2]);
  shape(link4);
  drawLocalFrame();

  translate(124, 0, 0);
  rotateY(-joint_angle[3]);
  shape(link5);
  drawLocalFrame();

  translate(69, 0, 0);
  translate(0, gripper_pos[0], 0);
  shape(gripper);
  drawLocalFrame();

  translate(0, 0, 0);
  translate(0, gripper_pos[1], 0);
  shape(gripper_sub);
  drawLocalFrame();
  popMatrix();
}

void drawWorldFrame()
{
  strokeWeight(10);
  stroke(255, 0, 0, 100);
  line(0, 0 ,0 , 200, 0, 0);

  strokeWeight(10);
  stroke(0, 255, 0, 100);
  line(0, 0, 0, 0, 200, 0);

  stroke(0, 0, 255, 100);
  strokeWeight(10);
  line(0, 0, 0, 0, 0, 200);
}

void drawLocalFrame()
{
  strokeWeight(10);
  stroke(255, 0, 0, 100);
  line(0, 0 ,0 , 100, 0, 0);

  strokeWeight(10);
  stroke(0, 255, 0, 100);
  line(0, 0, 0, 0, 100, 0);

  stroke(0, 0, 255, 100);
  strokeWeight(10);
  line(0, 0, 0, 0, 0, 100);
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
  gripper_pos[0] = -25;
  gripper_pos[1] = -gripper_pos[0] - gripper_pos[0];
}

void gripperOff()
{
  gripper_pos[0] = -45;
  gripper_pos[1] = -gripper_pos[0] - gripper_pos[0];
}

void gripperAngle2Pos(float angle)
{
  float angle2pos = map(angle, 0.0, 1.57, -45.0, 0.0);
  gripper_pos[0] = angle2pos;
  gripper_pos[1] = -gripper_pos[0] - angle2pos;
}
