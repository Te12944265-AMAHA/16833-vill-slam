IntervalTimer cam_exp_on_timer;
IntervalTimer cam_exp_off_timer;
IntervalTimer pps_on_timer;
IntervalTimer pps_off_timer;
IntervalTimer frame_config_timer;

const int cam_pin = 13;
const int laser_pin = 14;
const int led_pin = 15;
const int pps_pin = 16;

int visual_exp = 800;
int profile_exp = 800;

// macros corresponding to serial commands after started
enum serial_cmd {
  NONE,
  STOPPED
};

struct FrameConfig
{
  int cam_exp_width;
  int led_state;
  int laser_state;

  FrameConfig(int _exp, int _led, int _laser)
  : cam_exp_width(_exp)
  , led_state(_led)
  , laser_state(_laser)
  {}
};

FrameConfig frame_configs[2] = {FrameConfig(visual_exp, HIGH, LOW), FrameConfig(profile_exp, LOW, HIGH)};

int frame_type = 0;
int counter = 0;

void wait_for_start(){
  reset_pins();
  bool waiting_flag = false;
  while (true)
  {
    if (Serial.available())
    {
      if (waiting_flag == false) {
        Serial.println("waiting");
        waiting_flag = true;
      }
      if (Serial.readString() == "start"){
        Serial.println("starting");
        break;
      }
      delay(10);
    }
  }
}

void reset_pins() {
  // all lights are off
  digitalWrite(laser_pin, LOW);
  digitalWrite(led_pin, LOW);
}

void setup() {
  pinMode(led_pin, OUTPUT);
  pinMode(laser_pin, OUTPUT);
  pinMode(cam_pin, OUTPUT);
  reset_pins();
  Serial.begin(57600);
}

int process_serial_after_started() {
  if (Serial.available() > 0)
  {
    if (Serial.readString() == "stop"){
      Serial.println("stopping");
      cam_exp_on_timer.end();
      digitalWrite(laser_pin, LOW);
      digitalWrite(led_pin, LOW);
      return STOPPED;
    }
    else
      return NONE;
  }
  return NONE;
}

void cameraTrigger()
{
  if (process_serial_after_started() == STOPPED)
    return;
  // set exposure width
  int exp_width = frame_configs[frame_type].cam_exp_width;

  // set laser and led
  //digitalWrite(led_pin, led_state);
  //digitalWrite(laser_pin, laser_state);

  // send time stamp of this image
  unsigned int microsec = micros();
  unsigned long long stamp_microsec = microsec + exp_width / 2;
  unsigned int stamp_sec = stamp_microsec / 1000000;
  unsigned int stamp_nsec = (stamp_microsec % 1000000) * 1000;
  Serial.print("camera ");
  Serial.print(frame_type);
  Serial.print(" ");
  Serial.print(stamp_sec);
  Serial.print(" ");
  Serial.println(stamp_nsec);

  // trigger
  digitalWrite(cam_pin, HIGH);

  // set timer for turn off exposure
  cam_exp_off_timer.begin(cameraTriggerOff, exp_width);
}

void cameraTriggerOff()
{
  digitalWrite(cam_pin, LOW);
  cam_exp_off_timer.end();
  
  frame_type = (frame_type + 1) % 2;
  
  int led_state = frame_configs[frame_type].led_state;
  int laser_state = frame_configs[frame_type].laser_state;
  digitalWrite(led_pin, led_state);
  digitalWrite(laser_pin, laser_state);
}

// pps todo
void pps() {
  int microsecs = micros();
  int sec = microsecs / int(1e6);
  int nsec = microsecs % int(1e6) * 1000;
  Serial.print("time: ");
  Serial.print(sec);
  Serial.print(", ");
  Serial.println(nsec);
  digitalWrite(pps_pin, HIGH);
  delay(100);
  digitalWrite(pps_pin, LOW);
}


void loop() {
  wait_for_start();
  // Camera is triggered every 12.5 ms, 80 FPS in total
  cam_exp_on_timer.begin(cameraTrigger, 12500);
}
