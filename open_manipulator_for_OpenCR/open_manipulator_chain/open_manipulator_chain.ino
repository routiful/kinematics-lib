#include "link.h"
#include "tf.h"

#define DEBUG
// #define SIMULATION

#define COMMUNICATION_RATE  300
#define GRIPPER_ON          -10.0
#define GRIPPER_OFF         -45.0
#define JOINT_NUM           4

#define BASE   0
#define JOINT1 1
#define JOINT2 2
#define JOINT3 3
#define JOINT4 4
#define END    5

float joint_angle[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float gripper_pos = GRIPPER_OFF;

HardwareTimer serial_timer(TIMER_CH1);

open_manipulator::Link link[JOINT_NUM+2];
open_manipulator::TF tf;

void setup()
{
  Serial.begin(57600);
#ifdef DEBUG
   while(!Serial);
#endif

  initLink();
  timerInit();

  establishContact();

  setJointAngle(joint_angle);
  forwardKinematics(BASE);

#ifdef DEBUG
  for (uint8_t num = BASE; num <= END; num++)
  {
    Serial.print("link : "); Serial.println(link[num].name_);
    Serial.println("p_ : "); print_vt3f(link[num].p_);
    Serial.println("R_ : "); print_mt3f(link[num].R_);
  }
#endif

  open_manipulator::Link goal_pose;
  goal_pose.p_ << 0.158, 0.000, 0.198;
  goal_pose.R_ = tf.calcRotationMatrix("pitch", 0.0);

  inverseKinematics(goal_pose);
}

void loop()
{
  showLedStatus();
}

/*******************************************************************************
* Print link name
*******************************************************************************/
void printLinkName(int8_t me)
{
  if (me != -1)
  {
    Serial.println(link[me].name_);
    printLinkName(link[me].child_);
    printLinkName(link[me].sibling_);
  }
}

/*******************************************************************************
* Get mass of links
*******************************************************************************/
float totalMass(int8_t me)
{
  float mass = 0.0;

  if (me == -1)
  {
    mass = 0.0;
  }
  else
  {
    mass = link[me].mass_ + totalMass(link[me].child_) + totalMass(link[me].sibling_);
  }

  return mass;
}

/*******************************************************************************
* Hardware timer initialization
*******************************************************************************/
void timerInit()
{
  serial_timer.stop();
  serial_timer.setPeriod(COMMUNICATION_RATE);
  serial_timer.attachInterrupt(handler_serial);
  serial_timer.start();
}

/*******************************************************************************
* Forward kinematics
*******************************************************************************/
void forwardKinematics(int8_t me)
{
  int8_t mother = 0;

  if (me == -1)
  {
    return;
  }

  if (me != 0)
  {
    mother = link[me].mother_;
    link[me].p_ = link[mother].R_ * link[me].b_ + link[mother].p_;
    link[me].R_ = link[mother].R_ * tf.calcRodrigues(link[me].a_, link[me].q_);
  }

  forwardKinematics(link[me].sibling_);
  forwardKinematics(link[me].child_);
}

/*******************************************************************************
* Inverse kinematics (Analytical Method)
*******************************************************************************/
void inverseKinematics(open_manipulator::Link end_pose)
{
  Eigen::Vector3f r;
  float A = 0.0, B = 0.0, C = 0.0, alpha = 0.0;
  float result_of_cosine_rule = 0.0;

  r = end_pose.R_.transpose() * (link[JOINT2].p_ - end_pose.p_);
  C = r.norm();
  A = (link[JOINT3].p_ - link[JOINT2].p_).norm();
  B = (link[JOINT4].p_ - link[JOINT3].p_).norm();
  result_of_cosine_rule = (A*A + B*B - C*C) / (2.0 * A * B);

  if (result_of_cosine_rule >= 1.0)
  {
    joint_angle[JOINT3] = 0.0 - M_PI;
  }
  else if (result_of_cosine_rule <= -1.0)
  {
    joint_angle[JOINT3] = M_PI - M_PI;
  }
  else
  {
    joint_angle[JOINT3] = -acos(result_of_cosine_rule) + M_PI - M_PI;
  }

  alpha = asin((B/C) * sin(M_PI-joint_angle[JOINT3]));
  joint_angle[JOINT4] = -atan2(r(0), tf.sign(r(2)) * sqrt(r(1)*r(1) + r(2)*r(2))) - alpha;

  setJointAngle(joint_angle);

#ifdef DEBUG
  print_vt3f(r);
  Serial.println(C);
  Serial.println(A);
  Serial.println(B);
  Serial.println(result_of_cosine_rule);
  Serial.println(joint_angle[JOINT3]*RAD2DEG);
  Serial.println(joint_angle[JOINT4]*RAD2DEG);
#endif

}

/*******************************************************************************
* Manipulator link initialization
*******************************************************************************/
void initLink()
{
  link[0].name_    = "Base";
  link[0].mother_  = -1;
  link[0].sibling_ = -1;
  link[0].child_   = 1;
  link[0].mass_    = 1.0;
  link[0].p_       = Eigen::Vector3f::Zero();
  link[0].R_       = Eigen::Matrix3f::Identity();
  link[0].q_       = 0;
  link[0].dq_      = 0;
  link[0].ddq_     = 0;
  link[0].a_       = Eigen::Vector3f::Zero();
  link[0].b_       = Eigen::Vector3f::Zero();
  link[0].v_       = Eigen::Vector3f::Zero();
  link[0].w_       = Eigen::Vector3f::Zero();

  link[1].name_    = "Joint1";
  link[1].mother_  = 0;
  link[1].sibling_ = -1;
  link[1].child_   = 2;
  link[1].mass_    = 1.0;
  link[1].p_       = Eigen::Vector3f::Zero();
  link[1].R_       = Eigen::Matrix3f::Identity();
  link[1].q_       = 0.0;
  link[1].dq_      = 0.0;
  link[1].ddq_     = 0.0;
  link[1].a_       << 0, 0, 1;
  link[1].b_       << 0.012, 0, 0.036;
  link[1].v_       = Eigen::Vector3f::Zero();
  link[1].w_       = Eigen::Vector3f::Zero();

  link[2].name_    = "Joint2";
  link[2].mother_  = 1;
  link[2].sibling_ = -1;
  link[2].child_   = 3;
  link[2].mass_    = 1.0;
  link[2].p_       = Eigen::Vector3f::Zero();
  link[2].R_       = Eigen::Matrix3f::Identity();
  link[2].q_       = 0.0;
  link[2].dq_      = 0.0;
  link[2].ddq_     = 0.0;
  link[2].a_       << 0, 1, 0;
  link[2].b_       << 0, 0, 0.040;
  link[2].v_       = Eigen::Vector3f::Zero();
  link[2].w_       = Eigen::Vector3f::Zero();

  link[3].name_    = "Joint3";
  link[3].mother_  = 2;
  link[3].sibling_ = -1;
  link[3].child_   = 4;
  link[3].mass_    = 1.0;
  link[3].p_       = Eigen::Vector3f::Zero();
  link[3].R_       = Eigen::Matrix3f::Identity();
  link[3].q_       = 0.0;
  link[3].dq_      = 0.0;
  link[3].ddq_     = 0.0;
  link[3].a_       << 0, 1, 0;
  link[3].b_       << 0.022, 0, 0.122;
  link[3].v_       = Eigen::Vector3f::Zero();
  link[3].w_       = Eigen::Vector3f::Zero();

  link[4].name_    = "Joint4";
  link[4].mother_  = 3;
  link[4].sibling_ = -1;
  link[4].child_   = 5;
  link[4].mass_    = 1.0;
  link[4].p_       = Eigen::Vector3f::Zero();
  link[4].R_       = Eigen::Matrix3f::Identity();
  link[4].q_       = 0.0;
  link[4].dq_      = 0.0;
  link[4].ddq_     = 0.0;
  link[4].a_       << 0, 1, 0;
  link[4].b_       << 0.124, 0, 0;
  link[4].v_       = Eigen::Vector3f::Zero();
  link[4].w_       = Eigen::Vector3f::Zero();

  link[5].name_    = "End";
  link[5].mother_  = 4;
  link[5].sibling_ = -1;
  link[5].child_   = -1;
  link[5].mass_    = 1.0;
  link[5].p_       = Eigen::Vector3f::Zero();
  link[5].R_       = Eigen::Matrix3f::Identity();
  link[5].q_       = 0.0;
  link[5].dq_      = 0.0;
  link[5].ddq_     = 0.0;
  link[5].a_       = Eigen::Vector3f::Zero();
  link[5].b_       << 0.070, 0, 0;
  link[5].v_       = Eigen::Vector3f::Zero();
  link[5].w_       = Eigen::Vector3f::Zero();
}

/*******************************************************************************
* Set joint angle
*******************************************************************************/
void setJointAngle(float joint_angle[6])
{
  link[JOINT1].q_ = joint_angle[JOINT1];
  link[JOINT2].q_ = joint_angle[JOINT2];
  link[JOINT3].q_ = joint_angle[JOINT3];
  link[JOINT4].q_ = joint_angle[JOINT4];
}

/*******************************************************************************
* HardwareTimer function
*******************************************************************************/
void handler_serial()
{
#ifdef SIMULATION
  static String simulator = "";
  static bool communication = false;

  if (Serial.available())
  {
    simulator = Serial.readString();
    if (simulator == "ready")
    {
      communication = true;
    }
    else if (simulator == "stop")
    {
      communication = false;
    }
  }

  if (communication)
  {
    Serial.print(link[JOINT1].q_);
    Serial.print(",");
    Serial.print(link[JOINT2].q_);
    Serial.print(",");
    Serial.print(link[JOINT3].q_);
    Serial.print(",");
    Serial.print(link[JOINT4].q_);
    Serial.print(",");
    Serial.println(gripper_pos);
  }
#endif
}

/*******************************************************************************
* send an initial string
*******************************************************************************/
void establishContact()
{
  if (Serial.available())
  {
    Serial.println("0.0, 0.0, 0.0, 0.0, 0.0");
    delay(300);
  }
}

/*******************************************************************************
* show led status
*******************************************************************************/
void showLedStatus(void)
{
  static uint32_t t_time = millis();

  if ((millis()-t_time) >= 500 )
  {
    t_time = millis();
    digitalWrite(13, !digitalRead(13));
  }

  if (getPowerInVoltage() < 11.1)
  {
    setLedOn(2);
  }
  else
  {
    setLedOff(2);
  }

  if (getUsbConnected() > 0)
  {
    setLedOn(3);
  }
  else
  {
    setLedOff(3);
  }

  updateRxTxLed();
}

void updateRxTxLed(void)
{
  static uint32_t rx_led_update_time;
  static uint32_t tx_led_update_time;
  static uint32_t rx_cnt;
  static uint32_t tx_cnt;


  if ((millis()-tx_led_update_time) > 50)
  {
    tx_led_update_time = millis();

    if (tx_cnt != Serial.getTxCnt())
    {
      setLedToggle(0);
    }
    else
    {
      setLedOff(0);
    }

    tx_cnt = Serial.getTxCnt();
  }

  if( (millis()-rx_led_update_time) > 50 )
  {
    rx_led_update_time = millis();

    if (rx_cnt != Serial.getRxCnt())
    {
      setLedToggle(1);
    }
    else
    {
      setLedOff(1);
    }

    rx_cnt = Serial.getRxCnt();
  }
}

/*******************************************************************************
* print matrix(3x3)
*******************************************************************************/
void print_mt3f(const Eigen::Matrix3f& m)
{
   uint8_t i, j;

   for (i=0; i<3; i++)
   {
       for (j=0; j<3; j++)
       {
           Serial.print(m(i,j), 3);   // print 6 decimal places
           Serial.print(", ");
       }
       Serial.println();
   }
   Serial.println();
}

/*******************************************************************************
* print vector(1x3)
*******************************************************************************/
void print_vt3f(const Eigen::Vector3f& v)
{
   uint8_t i, j;

   for (i=0; i<3; i++)
   {
     Serial.print(v(i), 3);
     Serial.print(", ");
   }
   Serial.println();
}
