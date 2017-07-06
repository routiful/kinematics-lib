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

int ball_num = 0;
float[][] ball_pos = new float[7][4];

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

  drawBall();
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

  String[] cmd = split(opencr_string, ',');

  if (cmd[0].equals("angle"))
  {
    for (int cmd_cnt = 1; cmd_cnt < cmd.length; cmd_cnt++)
    {
      if (cmd_cnt == cmd.length-1)
      {
        gripperAngle2Pos(float(cmd[cmd_cnt]));
        println("gripper : " + cmd[cmd_cnt]);
      }
      else
      {
        joint_angle[cmd_cnt-1] = float(cmd[cmd_cnt]);
        print("joint " + cmd_cnt + ": " + cmd[cmd_cnt] + "  ");
      }
    }
  }
  else if (cmd[0].equals("end"))
  {
    ball_num = -1;
  }
  else
  {
    println("Error");
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

void drawBall()
{
  if (ball_num == -1)
    return;

  for (int i=0; i<ball_num; i++)
  {
    setBall(i);
  }
}

void setBall(int num)
{
  pushMatrix();
  translate(-model_trans_x, -model_trans_y, 0);
  rotateX(model_rot_x);
  rotateZ(model_rot_z);

  translate(12, 0, 36);
  rotateZ(ball_pos[num][0]);

  translate(0, 2, 40);
  rotateY(ball_pos[num][1]);

  translate(22, 0, 122);
  rotateY(ball_pos[num][2]);

  translate(124, 0, 0);
  rotateY(ball_pos[num][3]);

  translate(69, 0, 0);
  drawSphere(30,0,0,
             255,255,255,
             20);
  popMatrix();
}

void drawSphere(int x, int y, int z, int r, int g, int b, int size)
{
  pushMatrix();
  translate(x,y,z);
  stroke(r,g,b);
  sphere(size);
  popMatrix();
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
  drawSphere(30,0,0,255,255,0,20);
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
  float angle2pos = map(angle, 0.0, 3.5, -45.0, 0.0);
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
  if      (key == 'a') model_trans_x      -= 50;
  else if (key == 'd') model_trans_x      += 50;
  else if (key == 's') model_trans_y      += 50;
  else if (key == 'w') model_trans_y      -= 50;
  else if (key == 'q') model_scale_factor += 0.5;
  else if (key == 'e') model_scale_factor -= 0.5;
  else if (key == 'i') model_trans_x = model_trans_y = model_scale_factor = model_rot_z = model_rot_x = 0;
}

/*******************************************************************************
* Controller Window
*******************************************************************************/
class ChildApplet extends PApplet
{
  ControlP5 cp5;

  Textlabel myTextlabelA;

  Knob joint1, joint2, joint3, joint4, gripper;

  float grip_angle;
  boolean onoff_flag = false;
  int motion_num = 0;

  public ChildApplet()
  {
    super();
    PApplet.runSketch(new String[]{this.getClass().getName()}, this);
  }

  public void settings()
  {
    size(400, 600);
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
       .setSize(400,40)
       .setMode(Toggle.SWITCH)
       .setFont(createFont("arial",15))
       ;

    joint1 = cp5.addKnob("joint1")
             .setRange(-3.14,3.14)
             .setValue(0)
             .setPosition(30,140)
             .setRadius(50)
             .setDragDirection(Knob.HORIZONTAL)
             .setFont(createFont("arial",10))
             .setColorForeground(color(255))
             .setColorBackground(color(0, 160, 100))
             .setColorActive(color(255,255,0))
             ;

    joint2 = cp5.addKnob("joint2")
             .setRange(-3.14,3.14)
             .setValue(0)
             .setPosition(150,140)
             .setRadius(50)
             .setDragDirection(Knob.HORIZONTAL)
             .setFont(createFont("arial",10))
             .setColorForeground(color(255))
             .setColorBackground(color(0, 160, 100))
             .setColorActive(color(255,255,0))
             ;

    joint3 = cp5.addKnob("joint3")
             .setRange(-3.14,3.14)
             .setValue(0)
             .setPosition(270,140)
             .setRadius(50)
             .setDragDirection(Knob.HORIZONTAL)
             .setFont(createFont("arial",10))
             .setColorForeground(color(255))
             .setColorBackground(color(0, 160, 100))
             .setColorActive(color(255,255,0))
             ;

    joint4 = cp5.addKnob("joint4")
             .setRange(-3.14,3.14)
             .setValue(0)
             .setPosition(85,260)
             .setRadius(50)
             .setDragDirection(Knob.HORIZONTAL)
             .setFont(createFont("arial",10))
             .setColorForeground(color(255))
             .setColorBackground(color(0, 160, 100))
             .setColorActive(color(255,255,0))
             ;

    gripper = cp5.addKnob("gripper")
                .setRange(0.0, 1.5)
                .setValue(0.0)
                .setPosition(210,260)
                .setRadius(50)
                .setDragDirection(Knob.HORIZONTAL)
                .setFont(createFont("arial",10))
                .setColorForeground(color(255))
                .setColorBackground(color(0, 160, 100))
                .setColorActive(color(255,255,0))
                ;

    cp5.addButton("Origin")
       .setValue(0)
       .setPosition(0,350)
       .setSize(80,40)
       .setFont(createFont("arial",13))
       .setColorForeground(color(150,150,0))
       .setColorBackground(color(100, 160, 0))
       ;

    cp5.addButton("Basic")
       .setValue(0)
       .setPosition(320,350)
       .setSize(80,40)
       .setFont(createFont("arial",13))
       .setColorForeground(color(150,150,0))
       .setColorBackground(color(100, 160, 0))
       ;

    cp5.addButton("Send_Joint_Angle")
       .setValue(0)
       .setPosition(0,400)
       .setSize(400,40)
       .setFont(createFont("arial",15))
       ;

    cp5.addButton("Set_Gripper")
       .setValue(0)
       .setPosition(0,460)
       .setSize(400,40)
       .setFont(createFont("arial",15))
       ;

    cp5.addToggle("Gripper_OnOff")
       .setPosition(0,520)
       .setSize(400,40)
       .setMode(Toggle.SWITCH)
       .setFont(createFont("arial",15))
       ;

    cp5.addButton("Torque_Off")
       .setValue(0)
       .setPosition(0,130)
       .setSize(400,40)
       .setFont(createFont("arial",15))
       ;

    cp5.addButton("Make_Joint_Pose")
       .setValue(0)
       .setPosition(0,190)
       .setSize(400,40)
       .setFont(createFont("arial",15))
       ;

    cp5.addToggle("Make_Gripper_Pose")
       .setPosition(0,250)
       .setSize(400,40)
       .setMode(Toggle.SWITCH)
       .setFont(createFont("arial",15))
       ;

    cp5.addButton("Motion_Start")
       .setValue(0)
       .setPosition(0,330)
       .setSize(400,40)
       .setFont(createFont("arial",15))
       ;

    cp5.addToggle("Motion_Repeat")
       .setPosition(0,390)
       .setSize(400,40)
       .setMode(Toggle.SWITCH)
       .setFont(createFont("arial",15))
       ;

    cp5.getController("label").moveTo("global");
    cp5.getController("Controller_OnOff").moveTo("global");

    cp5.getController("Torque_Off").moveTo("Application");
    cp5.getController("Make_Joint_Pose").moveTo("Application");
    cp5.getController("Make_Gripper_Pose").moveTo("Application");
    cp5.getController("Motion_Start").moveTo("Application");
    cp5.getController("Motion_Repeat").moveTo("Application");
  }

  public void draw()
  {
    background(0);
  }

  void Controller_OnOff(boolean flag)
  {
    onoff_flag = flag;
    if (onoff_flag)
    {
      joint1.setValue(joint_angle[0]);
      joint2.setValue(joint_angle[1]);
      joint3.setValue(joint_angle[2]);
      joint4.setValue(joint_angle[3]);

      opencr_port.write("ready" + '\n');
      println("OpenManipulator Chain Ready!!!");
    }
    else
    {
      opencr_port.write("end" + '\n');
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

  public void Origin(int theValue)
  {
    if (onoff_flag)
    {
      joint_angle[0] = 0.0;
      joint_angle[1] = 0.0;
      joint_angle[2] = 0.0;
      joint_angle[3] = 0.0;

      joint1.setValue(joint_angle[0]);
      joint2.setValue(joint_angle[1]);
      joint3.setValue(joint_angle[2]);
      joint4.setValue(joint_angle[3]);

      opencr_port.write("joint"        + ',' +
                        joint_angle[0] + ',' +
                        joint_angle[1] + ',' +
                        joint_angle[2] + ',' +
                        joint_angle[3] + '\n');
    }
    else
    {
      println("Please, Set On Controller");
    }
  }

  public void Basic(int theValue)
  {
    if (onoff_flag)
    {
      joint_angle[0] = 0.0;
      joint_angle[1] = 60.0  * PI/180.0;
      joint_angle[2] = -20.0 * PI/180.0;
      joint_angle[3] = -40.0 * PI/180.0;

      joint1.setValue(joint_angle[0]);
      joint2.setValue(joint_angle[1]);
      joint3.setValue(joint_angle[2]);
      joint4.setValue(joint_angle[3]);

      opencr_port.write("joint"        + ',' +
                        joint_angle[0] + ',' +
                        joint_angle[1] + ',' +
                        joint_angle[2] + ',' +
                        joint_angle[3] + '\n');
    }
    else
    {
      println("Please, Set On Controller");
    }
  }

  public void Send_Joint_Angle(int theValue)
  {
    if (onoff_flag)
    {
      opencr_port.write("joint"        + ',' +
                        joint_angle[0] + ',' +
                        joint_angle[1] + ',' +
                        joint_angle[2] + ',' +
                        joint_angle[3] + '\n');
    }
    else
    {
      println("Please, Set On Controller");
    }
  }

  public void Set_Gripper(int theValue)
  {
    if (onoff_flag)
    {
      opencr_port.write("gripper"  + ',' +
                        grip_angle + '\n');
    }
    else
    {
      println("Please, Set On Controller");
    }
  }

  void Gripper_OnOff(boolean flag)
  {
    if (onoff_flag)
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
    else
    {
      println("Please, Set On Controller");
    }
  }

  public void Torque_Off(int theValue)
  {
    if (onoff_flag)
    {
      opencr_port.write("loose" + '\n');
      println("Torque Disable of OpenManipulator");
    }
    else
    {
      println("Please, Set On Controller");
    }
  }

  public void Make_Joint_Pose(int theValue)
  {
    if (onoff_flag)
    {
      opencr_port.write("get"      + ',' +
                        motion_num + '\n');

      motion_num++;

      for (int i=0; i<4; i++)
      {
        ball_pos[ball_num][i] = -joint_angle[i];
      }
      ball_num++;
    }
    else
    {
      println("Please, Set On Controller");
    }
  }

  void Make_Gripper_Pose(boolean flag)
  {
    int grip_on  = -1;
    int grip_off = -2;

    if (onoff_flag)
    {
      if (flag)
      {
        opencr_port.write("get"    + ',' +
                          grip_on  + '\n');
      }
      else
      {
        opencr_port.write("get"     + ',' +
                          grip_off  + '\n');
      }
      motion_num++;
    }
    else
    {
      println("Please, Set On Controller");
    }
  }

  public void Motion_Start(int theValue)
  {
    if (onoff_flag)
    {
      opencr_port.write("once" + '\n');
      println("Motion Start!!!");

      motion_num = 0;
    }
    else
    {
      println("Please, Set On Controller");
    }
  }

  void Motion_Repeat(boolean flag)
  {
    if (onoff_flag)
    {
      if (flag)
      {
        opencr_port.write("repeat" + '\n');
      }
      else
      {
        opencr_port.write("stop" + '\n');
      }
    }
    else
    {
      println("Please, Set On Controller");
    }
  }
}
