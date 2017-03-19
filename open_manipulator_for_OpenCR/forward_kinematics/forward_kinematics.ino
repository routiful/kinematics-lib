#include <Eigen.h>        // Calls main Eigen matrix class library
#include <Eigen/LU>       // Calls inverse, determinant, LU decomp., etc.

#define SERIAL_RATE 1000000

using namespace Eigen;    // Eigen related statement; simplifies syntax for declaration of matrices

float joint_angle[4] = {0.5,20.3,0,0};

HardwareTimer serial_timer(TIMER_CH1);

void setup()
{
  Serial.begin(250000);
  timerInit();

  establishContact();
}

void loop()
{
  showLedStatus();
}

void timerInit()
{
  serial_timer.stop();
  serial_timer.setPeriod(SERIAL_RATE);
  serial_timer.attachInterrupt(handler_serial);
  serial_timer.start();
}

void handler_serial()
{
  Serial.print(joint_angle[0]);
  Serial.print(",");
  Serial.print(joint_angle[1]);
  Serial.print(",");
  Serial.print(joint_angle[2]);
  Serial.print(",");
  Serial.println(joint_angle[3]);
}

void establishContact()
{
  if (Serial.available())
  {
    Serial.println("0,0,0,0");   // send an initial string
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
