/**
 * 1) In the first iteration of loop(), block until start cmd is received from computer
 * 2) Turns on the triggering (lights flashing quickly), then loops back 
 * 3) At the second iteration of loop(), block until stop cmd is received
 * 4) Turn off triggering (all lights off)
 * 5) GOTO step 1)
 *
 * @author Tina Tian (yutian)
 * @date   01/26/2022
 */
 
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

bool has_started = false;

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

void reset_pins() {
  // all lights are off
  digitalWrite(laser_pin, LOW);
  digitalWrite(led_pin, LOW);
}


void stop_triggering() {
  cam_exp_on_timer.end();
  reset_pins();
}


int process_serial_cmd() {
  while (true){
    if (Serial.available() > 0)
    {
      String cmd = Serial.readString();
      if (cmd == "start"){
        Serial.println("starting");
        has_started = true;
        break;
      }
      else if (cmd == "stop") {
        Serial.println("stopping");
        stop_triggering();
        has_started = false;
        // remains in the while loop, waiting for the next "start"
      } 
    }
    delay(10);
  }
}

void cameraTrigger()
{
  // set exposure width
  int exp_width = frame_configs[frame_type].cam_exp_width;

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

// pps, not in use
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

void setup() {
  pinMode(led_pin, OUTPUT);
  pinMode(laser_pin, OUTPUT);
  pinMode(cam_pin, OUTPUT);
  reset_pins();
  Serial.begin(57600);
}

void loop() {
  process_serial_cmd();
  // Camera is triggered every 12.5 ms, 80 FPS in total
  cam_exp_on_timer.begin(cameraTrigger, 12500);
}
