void DoAlarmUpdate() {
  if (analogRead(54) > 256) {
    // auger on
    auger_on_length++;
    auger_off_length = max(0,auger_off_length*0.8-10);
  } else {
    // auger off
    auger_off_length++;
    auger_on_length = max(0,auger_on_length*.8-10);
  }
}

void DoAlarm() {
  alarm = false;
  if (auger_on_length >= auger_on_alarm_point) {
    Serial.println("# Auger on too long");
    alarm = true;
  }
  if (auger_off_length >= auger_off_alarm_point) {
    Serial.println("# Auger off too long");
    alarm = true;
  }
  if (Temp_Data[LOW_FUEL_TC] > 230) {
    Serial.println("# Reactor fuel may be low");
    alarm = true;
  }
  if (Vrmsave > 50 & (Temp_Data[0]<790 || Temp_Data[1]<790)) {
    Serial.println("# T_tred and/or T_bred below 790�C while engine is running");
    alarm = true;
  }
}
