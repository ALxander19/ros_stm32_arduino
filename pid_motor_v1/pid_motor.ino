#include "pid_motor.h"

ros::NodeHandle nh;

void setup() {

  nh.initNode();

  nh.subscribe(w_right);
  nh.advertise(vel_motor_right);
  nh.advertise(vel_fil_right);

  pinMode(inaR, OUTPUT);
  pinMode(inbR, OUTPUT);
  
  digitalWrite(inaR, LOW);       // MOTOR RIGHT
  digitalWrite(inbR, LOW);
  
  pinMode(encaR, INPUT);           // MOTOR RIGHT
  digitalWrite(encaR, HIGH);       // turn on pullup resistor
  pinMode(encbR, INPUT);           // MOTOR RIGHT
  digitalWrite(encbR, HIGH);       // turn on pullup resistor

  attachInterrupt(digitalPinToInterrupt(encaR), doEncoderA, CHANGE);  // MOTOR ENCODER
  attachInterrupt(digitalPinToInterrupt(encbR), doEncoderB, CHANGE);  // EXTERNAL ENCODER
}

void loop() {

  uint32_t t = millis();

  if ((t - tTime[0]) >= (1000 / ENCODER_PUBLISH_FREQUENCY))
  {
    newpositionR = encoderRPos;                      // MOTOR RIGHT
    newtimeR = millis();
    wR = (float)(newpositionR - oldpositionR) * 0.7494275 / (newtimeR - oldtimeR);

    oldpositionR = newpositionR;
    oldtimeR = newtimeR;

    wR_round = roundf(wR * 1000);
    vel_right_msg.data = wR;
    vel_motor_right.publish( &vel_right_msg );
    tTime[0] = t;
  }

  if ((t - tTime[1]) >= (1000 / FILTER_PUBLISH_FREQUENCY))
  {
    velfil = wR * 0.0609 + velfil * 0.9391;
    vel_fil_msg.data = velfil;
    vel_fil_right.publish( &vel_fil_msg );
    tTime[1] = t;
  }

  nh.spinOnce();
}


void w_right_cb(const std_msgs::Float32& w_right_msg) {

  new_timeR = millis();
  //nh.advertise(vel_fil);

  error_R = w_right_msg.data - velfil;
  //error_R = w_right_msg.data - wR;

  derivative_R = (error_R - prev_errorR)*1000/(new_timeR - old_timeR);
  integral_R = integral_R + error_R*(new_timeR - old_timeR)/1000;
  
  wR_control = (int)(error_R*kp_R + derivative_R*kd_R + integral_R*ki_R);
  //wR_control = w_right_msg.data;

  prev_errorR = error_R;
  old_timeR = new_timeR;
  
  if (wR_control == 0) {
    digitalWrite(inaR, LOW);
    digitalWrite(inbR, LOW);
  }
  else if (wR_control > 0 and wR_control <= 255) {
    analogWrite(inaR, wR_control);
    digitalWrite(inbR, LOW);
  }
  else if (wR_control > 255) {
    digitalWrite(inaR, HIGH);
    digitalWrite(inbR, LOW);
  }
  else if (wR_control < 0 and wR_control >= -255) {
    digitalWrite(inaR, LOW);
    analogWrite(inbR, wR_control*(-1));
  }
  else if (wR_control < -255) {
    digitalWrite(inaR, LOW);
    digitalWrite(inbR, HIGH);
  } 
}


void doEncoderR() {               // MOTOR RIGHT
  if (digitalRead(encaR) == digitalRead(encbR)) {
    encoderRPos++;
  } else {
    encoderRPos--;
  }
}

void doEncoderA() {
  // look for a low-to-high on channel A
  if (digitalRead(encaR) == HIGH) {

    // check channel B to see which way encoder is turning
    if (digitalRead(encbR) == LOW) {
      encoderRPos = encoderRPos + 1;         // CW
    }
    else {
      encoderRPos = encoderRPos - 1;         // CCW
    }
  }

  else   // must be a high-to-low edge on channel A
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(encbR) == HIGH) {
      encoderRPos = encoderRPos + 1;          // CW
    }
    else {
      encoderRPos = encoderRPos - 1;          // CCW
    }
  }
}

void doEncoderB() {
  // look for a low-to-high on channel B
  if (digitalRead(encbR) == HIGH) {

    // check channel A to see which way encoder is turning
    if (digitalRead(encaR) == HIGH) {
      encoderRPos = encoderRPos + 1;         // CW
    }
    else {
      encoderRPos = encoderRPos - 1;         // CCW
    }
  }

  // Look for a high-to-low on channel B

  else {
    // check channel B to see which way encoder is turning
    if (digitalRead(encaR) == LOW) {
      encoderRPos = encoderRPos + 1;          // CW
    }
    else {
      encoderRPos = encoderRPos - 1;          // CCW
    }
  }
}
