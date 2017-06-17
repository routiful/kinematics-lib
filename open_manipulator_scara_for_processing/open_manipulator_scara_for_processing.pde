/**
 * simulation.
 *
 * this code receives data from pose_controller.ino
 * to show how to change pose of objects.
*/

import processing.serial.*;

// Shape variable
PShape link1, link2, link3;
// Model pose
float model_rot_x, model_rot_z, model_trans_x, model_trans_y, model_scale_factor;
// Serial variable
Serial opencr_port;
// Angle variable
float[] joint_angle = new float[4];
float[] gripper_angle = new float[2];
// Simulation frequency
static int tTime;
int update_period = 250;

void setup()
{
  size(600, 600, OPENGL);
  smooth();

  initShape();
  initView();
  
  //connectOpenCR(0);
}

void draw()
{
  lights();
  smooth();
  background(30);  
  
  translate(width/2 + model_trans_x, height/2 + model_trans_y, 0);

  rotateX(radians(90) + model_rot_x);
  rotateZ(model_rot_z + radians(140));
  
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
  if (key == 'a') model_trans_x -= 50;
  else if (key == 'd') model_trans_x += 50;
  else if (key == 's') model_trans_y += 50;
  else if (key == 'w') model_trans_y -= 50;
  else if (key == 'r') model_scale_factor += 0.5;
  else if (key == 'f') model_scale_factor -= 0.5;
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
  link1 = loadShape("meshes/scara/link1.obj");
  link2 = loadShape("meshes/scara/link2.obj");
  link3 = loadShape("meshes/scara/link3.obj");

  setJointAngle(0, 0, 0, 0);
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
      gripper_angle[0] = angles[joint_num];
      gripper_angle[1] = -gripper_angle[0] - gripper_angle[0];
      print("gripper : " + angles[joint_num] + "\n");
    }
    else
    {
      joint_angle[joint_num] = angles[joint_num];
      print("joint " + (joint_num+1)  + ": " + angles[joint_num] + "\t");
    }
  }
}

void drawManipulator()
{
  scale(1 + model_scale_factor);

  pushMatrix();
  shape(link1);
  drawLocalFrame();

  translate(0, 0, 94.5);
  rotateZ(-joint_angle[0]);
  shape(link2);
  drawLocalFrame();

  translate(0, 136, 0);
  rotateZ(-joint_angle[1]);
  shape(link3);
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
  gripper_angle[0] = -20;
  gripper_angle[1] = -gripper_angle[0] + 20;
}

void gripperOff()
{
  gripper_angle[0] = -40;
  gripper_angle[1] = -gripper_angle[0] + 44;
}

void gripperJointAngle(float angle)
{
  gripper_angle[0] = angle;
  gripper_angle[1] = -gripper_angle[0] - angle;
}