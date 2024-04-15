#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_DPS310.h>
#include <queue>


#define LED_PIN 25
#define inner_ring 8
#define outer_ring 12
#define theta_range 30  //in degrees
#define inner_distance_to_pixel 40  // in mm(?) who knows
#define outer_distance_to_pixel 80
#define white_intensity 0.1
#define green_intensity 0.2
#define radius_range 40
#define NUM_LEDS 1+inner_ring+outer_ring
Adafruit_NeoPixel strip(1+inner_ring+outer_ring, LED_PIN, NEO_GRBW + NEO_KHZ800);
#define ambient_pressure 1021.8 //tempoaray for one sensor operations

using namespace std;
#define avg_intensity  60 //out of 255
#define random_range  avg_intensity*0.05
#define LED_enable 32
// Adafruit_LPS22 lps;
Adafruit_DPS310 dps_ambient;
Adafruit_DPS310 dps_flow;

struct fire{ //cylindrical cordinates {theta(in degrees), R(in m*10^-7 one under mm), led intensity(out of 255)}
    int theta;
    int radius;
    int intensity;
};

struct LED_array{
    int address;
    int theta;
    int radius;
};

LED_array LED_location[NUM_LEDS];
fire flame_previous = {0,0,avg_intensity};
fire flame_next = {0,0,avg_intensity};
int pressure_offset;
float wind = 0.0;
bool run = true, swap_state = false;
// create queue
queue<fire> fire_queue;
void setup(){
  Serial.begin(115200);
  delay(100);
  pinMode(LED_enable, OUTPUT);
  digitalWrite(LED_enable, HIGH);
  //create LED location array
  LED_location[0] = {0, 0, 0};  // central LED at center of cordiante array
  for(int i = 1; i <= inner_ring; i++){
    LED_location[i] = {i, i*(360/(inner_ring)), inner_distance_to_pixel}; //address, ignores the middle LED theta location in space, distance to center of the LED module
  } 
  for(int t = 1; t <= outer_ring; t++){
    LED_location[t+inner_ring] = {t+inner_ring, t*(360/(outer_ring)), outer_distance_to_pixel}; //address, ignores the middle LED theta location in space, distance to center of the LED module
  } 

  if (! dps_flow.begin_I2C(0x76)) {             //initialize air speed pressure sensor
    Serial.println("Failed to find DPS_flow");
  }
  Serial.println("DPS_flow OK!");

  if (! dps_ambient.begin_I2C(0x77)) {             //initialize amibient pressure sensor
    Serial.println("Failed to find DPS_ambient");
  }
  Serial.println("DPS_ambient OK!");

  // configure sensors
  dps_flow.configurePressure(DPS310_128HZ, DPS310_16SAMPLES);
  dps_flow.configureTemperature(DPS310_128HZ, DPS310_16SAMPLES);
  dps_ambient.configurePressure(DPS310_128HZ, DPS310_16SAMPLES);
  dps_ambient.configureTemperature(DPS310_128HZ, DPS310_16SAMPLES);
  Serial.println("Init complete!");
  delay(500);
  // calculate offset to zero airspeed of the pressure sensor
  sensors_event_t temp_event_flow, pressure_event_flow;
  sensors_event_t temp_event_ambient, pressure_event_ambient;
  dps_flow.getEvents(&temp_event_flow, &pressure_event_flow);
  dps_ambient.getEvents(&temp_event_ambient, &pressure_event_ambient);
  pressure_offset = pressure_event_flow.pressure-pressure_event_ambient.pressure;
  Serial.print("pressure offset: ");Serial.println(pressure_offset);

  // print out map of led positions
  for(int i = 0; i<NUM_LEDS; i++){
    Serial.print(LED_location[i].address);
    Serial.print(" ");
    Serial.print(LED_location[i].theta);
    Serial.print(" ");
    Serial.println(LED_location[i].radius);
    }
  fire_queue.push(flame_next);
}

void control_led(fire flame_position){
  /**
   * @brief takes in a flame coordiante and turns on the corisponding LED(s)
   * 
   */
  if(flame_position.radius > 20){ // determins if the flame is far enough away from the center to dim the center LED
    int center_intensity = 10*flame_position.intensity/(abs(flame_position.radius));
    strip.setPixelColor(LED_location[0].address,strip.Color(center_intensity,center_intensity*green_intensity,0,center_intensity*white_intensity));
  }
  else{
    strip.setPixelColor(LED_location[0].address,strip.Color(flame_position.intensity,flame_position.intensity*green_intensity,0,flame_position.intensity*white_intensity));
  }
  for(int i = 1; i < NUM_LEDS; i++){  //scan though the LED location array to find LEDs near the flame coordinate
    if(LED_location[i].theta < flame_position.theta+theta_range && LED_location[i].theta > flame_position.theta-theta_range){ // start with theta
      if(LED_location[i].radius < flame_position.radius+radius_range){ // see how far out to light leds
        float diff_theta = abs(LED_location[i].theta-flame_position.theta)/10;
        float diff_radius = abs(LED_location[i].radius-flame_position.radius)/10;
        //to deal with divide by zero errors
        if(diff_theta == 0){
          diff_theta = 1;
        }
        if(diff_radius == 0){
          diff_radius = 1;
        }
        int temp_intensity = flame_position.intensity/(diff_theta*diff_radius);
        strip.setPixelColor(LED_location[i].address,strip.Color(temp_intensity,temp_intensity*green_intensity,0,temp_intensity*white_intensity));
      }
    }
    else if(LED_location[i].theta < flame_position.theta+theta_range-360 || LED_location[i].theta > flame_position.theta-theta_range+360){
      float diff_theta = abs(LED_location[i].theta-flame_position.theta)/10;
      float diff_radius = abs(LED_location[i].radius-flame_position.radius)/10;
      if(diff_theta == 0){
        diff_theta = 1;
      }
      if(diff_radius == 0){
        diff_radius = 1;
      }
      int temp_intensity = flame_position.intensity/(diff_theta*diff_radius);
      strip.setPixelColor(LED_location[i].address,strip.Color(temp_intensity,temp_intensity*green_intensity,0,temp_intensity*white_intensity));
    }
    else{
      strip.setPixelColor(LED_location[i].address,strip.Color(0,0,0,0));

    }
    
  }
  //show the updates on the LEDs
  strip.show();
}

void led_transition(fire next,fire previous, bool verbose = false){
  /***
   * @brief find diffrence between the next and pervious fale variables and interpolates a path between them
   * added the path to the queue
  */
  int diff_intent = next.intensity-previous.intensity;
  int diff_radius = next.radius-previous.radius;
  int diff_theta = next.theta-previous.theta;
  

  int diff_intent_abs = abs(diff_intent);
  int diff_theta_abs = abs(diff_theta);
  int diff_radius_abs = abs(diff_radius);

  if(diff_theta_abs > 180){
    diff_theta_abs = 360 - diff_theta_abs;
    if(diff_theta > 0){
      diff_theta = diff_theta - 360; 
    }
    else{
      diff_theta = 360 + diff_theta;
    }
  }

  int inc = std::max({diff_intent_abs, diff_theta_abs, diff_radius_abs});

  float inc_theta = float(diff_theta)/float(inc);
  float inc_radius = float(diff_radius)/float(inc);
  float inc_intent = float(diff_intent)/float(inc);
  if (inc > 100){
    inc = inc/2;
  }
  fire transition[inc];
  for(int i = 1; i <=inc; i++){
    int theta = int(previous.theta+inc_theta*i);
    if (theta > 360){
      transition[i-1].theta = theta -360;
    }
    else if (theta < 0){
      transition[i-1].theta = 360 + theta;
    }
    else{
      transition[i-1].theta = theta;
    }
    transition[i-1].radius =  int(previous.radius+inc_radius*i);
    transition[i-1].intensity =  int(previous.intensity+inc_intent*i);
  }
  for(int i = 0; i < inc; i++){
    fire_queue.push(transition[i]);
    // control_led(transition[i]);
    // delay(random(10,20)); 
  }
  if(verbose){
    Serial.printf("next: theta:%d radius:%d intensity:%d", next.theta, next.radius, next.intensity);
    Serial.println();
    Serial.printf("previous: theta:%d radius:%d intensity:%d", previous.theta, previous.radius, previous.intensity);
    Serial.println();
    Serial.printf("theta flame: diff: %d abs:%d inc: %f",diff_theta, diff_theta_abs, inc_theta);
    Serial.println();
    Serial.printf("radius flame: diff: %d abs:%d inc: %f",diff_radius, diff_radius_abs, inc_radius);
    Serial.println();
    Serial.printf("intensity flame: diff: %d abs:%d inc: %f",diff_intent, diff_intent_abs, inc_intent);
    Serial.println();
    Serial.printf("increment: %d", inc);
    Serial.println();
    for(int i = 0; i < inc; i++){
      Serial.printf("transition array: theta:%d radius:%d intensity:%d", transition[i].theta, transition[i].radius, transition[i].intensity);
      Serial.println();
    }
  }
}
// fire restore(fire vector){ 

//   if (vector.radius > 20){
//      vector.radius = vector.radius-vector.radius*0.05;
//   }
//   else{
//      vector.radius = vector.radius-10;
//   }

//   if (vector.radius < 0){
//     vector.radius = 0;
//   }

//   if(vector.theta < 0){
//     vector.theta = 360;
//   }
//   else if(vector.theta > 360){
//     vector.theta = 0;
//   }

//   return vector;
// }

fire randomFlame(fire old_vector, float _wind = 0.0){
  int randomchange = random(100);
  fire vector;
  if(randomchange == 20 || _wind > 0.1){
    vector.theta = random(360);
    vector.radius = random(50, outer_distance_to_pixel);
    vector.intensity = random(avg_intensity-random_range,avg_intensity-random_range);
  }
  else{
    vector.radius = old_vector.radius - (0.5*old_vector.radius*(1-((outer_distance_to_pixel-old_vector.radius)/outer_distance_to_pixel)))+random(-10,5);
    if(vector.radius > outer_distance_to_pixel){
      vector.radius = outer_distance_to_pixel;
    }
    else if(vector.radius < 0){
      vector.radius = abs(vector.radius);
      vector.theta = vector.theta + 180;
    }
    vector.theta = old_vector.theta + random(-5,5);
    if(vector.theta > 360){
      vector.theta = vector.theta-360;
    }
    else if(vector.theta < 0){
      vector.theta = vector.theta + 360;
    }
    vector.intensity = old_vector.intensity + random(-random_range,random_range);
    if(vector.intensity > avg_intensity+random_range){
      vector.intensity = avg_intensity+random_range;
    }
    else if (vector.intensity < avg_intensity-random_range){
      vector.intensity = avg_intensity-random_range;
    }
  }
  return vector;
}

void manually_control_LED(){
  /**
   * @brief for controlling the "flame" position with manual serial inputs
   * 
   */
    Serial.println("input theta in XXX");
   while (Serial.available() == 0) {
  }
  int input_theta = Serial.parseInt();
  Serial.println("input radius in XX");
   while (Serial.available() == 0) {
  }
  int input_radius = Serial.parseInt();
  Serial.println("input intensity as XX");
   while (Serial.available() == 0) {
  }
  int input_inten = Serial.parseInt();
  flame_next = {input_theta,input_radius,input_inten};
  control_led(flame_next);
}

float get_airspeed(){
  sensors_event_t temp_event_flow, pressure_event_flow;
  sensors_event_t temp_event_ambient, pressure_event_ambient;
  dps_flow.getEvents(&temp_event_flow, &pressure_event_flow);
  dps_ambient.getEvents(&temp_event_ambient, &pressure_event_ambient);
  float airspeed = (sqrt((2*(pressure_event_flow.pressure-pressure_event_ambient.pressure+pressure_offset))/1.225))-0.6;//TODO remove the 0.6
  // Serial.print("airspeed: ");Serial.print(airspeed);Serial.println(" m/s");
  return airspeed;
}

void loop(){
  // Serial.println("starting...");
  // Serial.print("queue size: ");Serial.println(fire_queue.size());
  wind = get_airspeed();
  Serial.println(wind);
  // Serial.println(run);
  if(run){
    digitalWrite(LED_enable, HIGH);
    if(wind > 0.1){
      fire windy_fire = randomFlame(fire_queue.front(), wind);
      windy_fire.intensity = int(windy_fire.intensity/float(30*wind));
      fire_queue.push(windy_fire);
    }
    else if(fire_queue.size() <= 3){
      flame_next = randomFlame(fire_queue.front());
      led_transition(flame_next,flame_previous);
      flame_previous = fire_queue.back();
    }
    control_led(fire_queue.front());
    fire_queue.pop();
    delay(random(1,5));
  }
  else{
    delay(2);
  }
  if(wind > 0.45){
    if(!swap_state){
      run = !run;
      swap_state = true;
      if(run){
        digitalWrite(LED_enable, HIGH);
        delay(2);
      }
      else{
        digitalWrite(LED_enable, LOW);
      }
    }
    else{
      delay(500);
      swap_state = false;
    }

  }
}