/**
 * simulation.
 *
 * this code receives data from open_manipulator_chain.ino
 * to show how to change pose of objects.
*/

// Multiple Window
ChildApplet child;

// Control Interface
import controlP5.*;

// Init serial
import processing.serial.*;

// Shape variables
PShape link1, link2, link3, link4, link5, gripper, gripper_sub;

// Model pose
float model_rot_x, model_rot_z, model_trans_x, model_trans_y, model_scale_factor;

// Serial variable
Serial opencr_port;

// Angle variable
float[] joint_angle = new float[4];
float[] gripper_pos = new float[2];

void settings()
{
  size(600, 600, OPENGL);
  smooth();
}

void setup()
{
  surface.setTitle("OpenManipulator Chain");
  child = new ChildApplet();

  initShape();
  initView();

  connectOpenCR(0);
}

void draw()
{
  setWindow();

  drawTitle();
  drawWorldFrame();

  drawManipulator();
}

void connectOpenCR(int port_num)
{
  printArray(Serial.list());

  String port_name = Serial.list()[port_num];
  opencr_port = new Serial(this, port_name, 57600);
  opencr_port.bufferUntil('\n');
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
      print("joint " + (joint_num+1)  + ": " + angles[joint_num] + "  ");
    }
  }
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
  textSize(20);
  fill(102,255,255);
  text("Press 'A','D','W','S'", -450,120,0);
  text("And   'Q','E'",         -450,150,0);
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

  translate(22, 0, 122);
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

class ChildApplet extends PApplet
{
  ControlP5 cp5;

  Textlabel myTextlabelA;

  Knob joint1, joint2, joint3, joint4, gripper;

  float grip_angle;

  public ChildApplet()
  {
    super();
    PApplet.runSketch(new String[]{this.getClass().getName()}, this);
  }

  public void settings()
  {
    size(400, 800);
    smooth();
  }
  public void setup()
  {
    surface.setTitle("Control Interface");

    cp5 = new ControlP5(this);

    cp5.addTab("Application")
       .setColorBackground(color(0, 160, 100))
       .setColorLabel(color(255))
       .setColorActive(color(255,128,0))
       ;

    cp5.getTab("default")
       .activateEvent(true)
       .setLabel("JointSpaceControl")
       .setId(1)
       ;

    cp5.getTab("Application")
       .activateEvent(true)
       .setId(2)
       ;

    myTextlabelA = cp5.addTextlabel("label")
                     .setText("Controller for OpenManipulator Chain")
                     .setPosition(10,20)
                     .setColorValue(0xffffff00)
                     .setFont(createFont("Georgia",20))
                     ;

    cp5.addToggle("Controller_OnOff")
       .setPosition(0,50)
       .setSize(400,50)
       .setMode(Toggle.SWITCH)
       .setFont(createFont("arial",15))
       ;

    joint1 = cp5.addKnob("joint1")
             .setRange(-3.14,3.14)
             .setValue(0)
             .setPosition(50,150)
             .setRadius(60)
             .setDragDirection(Knob.HORIZONTAL)
             .setFont(createFont("arial",10))
             .setColorForeground(color(255))
             .setColorBackground(color(0, 160, 100))
             .setColorActive(color(255,255,0))
             ;

    joint2 = cp5.addKnob("joint2")
             .setRange(-3.14,3.14)
             .setValue(0)
             .setPosition(220,150)
             .setRadius(60)
             .setDragDirection(Knob.HORIZONTAL)
             .setFont(createFont("arial",10))
             .setColorForeground(color(255))
             .setColorBackground(color(0, 160, 100))
             .setColorActive(color(255,255,0))
             ;

    joint3 = cp5.addKnob("joint3")
             .setRange(-3.14,3.14)
             .setValue(0)
             .setPosition(50,300)
             .setRadius(60)
             .setDragDirection(Knob.HORIZONTAL)
             .setFont(createFont("arial",10))
             .setColorForeground(color(255))
             .setColorBackground(color(0, 160, 100))
             .setColorActive(color(255,255,0))
             ;

    joint4 = cp5.addKnob("joint4")
             .setRange(-3.14,3.14)
             .setValue(0)
             .setPosition(220,300)
             .setRadius(60)
             .setDragDirection(Knob.HORIZONTAL)
             .setFont(createFont("arial",10))
             .setColorForeground(color(255))
             .setColorBackground(color(0, 160, 100))
             .setColorActive(color(255,255,0))
             ;

    gripper = cp5.addKnob("gripper")
                .setRange(0.0, 0.8)
                .setValue(0.4)
                .setPosition(135,440)
                .setRadius(60)
                .setDragDirection(Knob.HORIZONTAL)
                .setFont(createFont("arial",10))
                .setColorForeground(color(255))
                .setColorBackground(color(0, 160, 100))
                .setColorActive(color(255,255,0))
                ;

    cp5.addButton("Send_Joint_Angle")
       .setValue(0)
       .setPosition(0,610)
       .setSize(400,40)
       .setFont(createFont("arial",15))
       ;

    cp5.addButton("Set_Gripper")
       .setValue(0)
       .setPosition(0,670)
       .setSize(400,40)
       .setFont(createFont("arial",15))
       ;

    cp5.addToggle("Gripper_OnOff")
       .setPosition(0,730)
       .setSize(400,40)
       .setMode(Toggle.SWITCH)
       .setFont(createFont("arial",15))
       ;

    cp5.getController("label").moveTo("global");
    cp5.getController("Controller_OnOff").moveTo("global");

    // cp5.getController("sliderValue").moveTo("extra");
  }

  public void draw()
  {
    background(0);
  }

  void Controller_OnOff(boolean flag)
  {
    if (flag)
    {
      opencr_port.write("ready" + '\n');
      println("OpenManipulator Chain Ready!!!");
    }
    else
    {
      opencr_port.write("stop" + '\n');
      println("OpenManipulator Chain End...");
    }
  }

  void joint1(float angle)
  {
    joint_angle[0] = angle;
  }

  void joint2(float angle)
  {
    joint_angle[1] = angle;
  }

  void joint3(float angle)
  {
    joint_angle[2] = angle;
  }

  void joint4(float angle)
  {
    joint_angle[3] = angle;
  }

  void gripper(float angle)
  {
    grip_angle = angle;
    gripperAngle2Pos(angle);
  }

  public void Send_Joint_Angle(int theValue)
  {
    opencr_port.write("joint"        + ',' +
                      joint_angle[0] + ',' +
                      joint_angle[1] + ',' +
                      joint_angle[2] + ',' +
                      joint_angle[3] + '\n');
  }

  public void Set_Gripper(int theValue)
  {
    opencr_port.write("gripper"  + ',' +
                      grip_angle + '\n');
  }

  void Gripper_OnOff(boolean flag)
  {
    if (flag)
    {
      opencr_port.write("on" + '\n');
    }
    else
    {
      opencr_port.write("off" + '\n');
    }
  }
}
