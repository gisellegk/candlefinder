#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

//#define CHATTER
#define LOG_OUT 1 // use the log output function
#define FHT_N 128 // set to 256 point fht

#define LED 10


#include <FHT.h> // include the library

int timeon = 0;

ros::NodeHandle nh;

std_msgs::Bool bool_msg;
ros::Publisher pub("start_bool", &bool_msg);
#ifdef CHATTER
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);
#endif
#ifdef CHATTER
void debugPrint(String str) {
  str_msg.data = str.c_str();
  chatter.publish(&str_msg);
}
#endif

bool published = true;

void setup() {
  //Serial.begin(115200);
  pinMode(8,INPUT);
  digitalWrite(8, HIGH);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  //TIMSK0 = 0; // turn off timer0 for lower jitter
  ADCSRA = 0xe5; // set the adc to free running mode
  ADMUX = 0x40; // use adc0
  DIDR0 = 0x01; // turn off the digital input for adc0

  nh.initNode();
  nh.advertise(pub);
  #ifdef CHATTER
  nh.advertise(chatter);
  #endif
}

void loop() {
  while(1) { // reduces jitter
    //cli();  // UDRE interrupt slows this way down on arduino1.0
    for (uint16_t i = 0 ; i < FHT_N ; i++) { // save 256 samples
    while(!(ADCSRA & 0x10)); // wait for adc to be ready
    ADCSRA = 0xf5; // restart adc
    byte m = ADCL; // fetch adc data
    byte j = ADCH;
    int k = (j << 8) | m; // form into an int
    k -= 0x0200; // form into a signed int
    k <<= 6; // form into a 16b signed int
    fht_input[i] = k; // put real data into bins
    }
    fht_window(); // window the data for better frequency response
    fht_reorder(); // reorder the data before doing the fht
    fht_run(); // process the data in the fht
    fht_mag_log(); // take the output of the fht
    //sei();
    
    //Serial.write(255); // send a start byte
    //Serial.write(fht_log_out, FHT_N/2); // send out the data
    uint16_t totalAvg = 0;
    for(uint16_t i = 2 ; i < FHT_N/2 ; i++) {
      totalAvg += fht_log_out[i];
    }
    totalAvg = totalAvg / (FHT_N/2-2);
    int avg = (fht_log_out[14] + fht_log_out[15] + fht_log_out[16]) / 3; 
    #ifdef CHATTER
    debugPrint(String(avg) + " " + String(totalAvg) + " " + String(timeon));
    #endif
    if(avg > 5 && avg*.9 > totalAvg){
      timeon++;
    } else if (timeon > 0) {
      timeon -= 10;
    }
    if (timeon < 0) {
      timeon = 0;
    }
    if(timeon > 250 || !digitalRead(8)) {
      bool_msg.data = true;
      if(!published) {
        published = true;
        pub.publish( &bool_msg );
      }
      pub.publish( &bool_msg );
    } else {
      bool_msg.data = false;
      if(published) {
        published = false;
        pub.publish( &bool_msg );
      }
    }
    //Serial.println(String(avg) + " " + String(totalAvg) + " " + String(timeon));
    digitalWrite(LED, bool_msg.data);
    nh.spinOnce();
  }
}
