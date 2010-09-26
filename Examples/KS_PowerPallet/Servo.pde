
// Servo
void PulseServo(int servo_pin,double angle) {
  digitalWrite(servo_pin,HIGH);
  delayMicroseconds(1520+angle*6);
  digitalWrite(servo_pin,LOW);
}

// code for driving servos w/ dead band
void DoServos() {
  if (servo_alt == 1) { // pulse every other time through the loop
    if (abs(servo0_pos - servo0_db) > 3) {
      servo0_db = servo0_pos;
    }
    if (abs(servo1_pos - servo1_db) > 3) {
      servo1_db = servo1_pos;
    }
    PulseServo(SERVO0,servo0_db);
    PulseServo(SERVO1,servo1_db);
    servo_alt = 0; 
  }
  servo_alt++;
}
