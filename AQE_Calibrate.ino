#include <stdint.h>
#include "Wire.h"
#include "EggBus.h"
#include "SoftReset.h"
#include <avr/wdt.h>
#include <string.h>
#include <errno.h>

EggBus eggBus;
uint8_t num_ids_total = 0;
uint8_t id_to_calibrate = 0;
uint8_t target_sensor_index = 0;

// statistics variables
#define SAMPLE_WINDOW_SIZE 60
uint32_t sample_window[SAMPLE_WINDOW_SIZE] = {0};
uint32_t sample_window_idx = 0;
uint16_t num_samples_collected = 0;

void reportDiscoveredSensors();
uint8_t getIdToCalibrate();
uint8_t validSensorId(char * str);
char *trimwhitespace(char *str);
long parseLong(char * str);
float parseDecimal(char * str);
void promptForIndex();
void promptFor(char * str);
void goToLogicalSensor(uint8_t target_logical_id);
uint32_t sqrtSumSquaredErrorFromMean();
uint32_t minResistance();
uint32_t maxResistance();
uint32_t meanResistance();

// states controlling loop behavior
#define REPORT_STATISTICS     1
#define RECEIVE_CAL_R         2
#define RECEIVE_CAL_R_OVER_R0 3
#define CONFIRM_CAL_PARAMS    4
#define SET_CAL_PARAMS        5
uint8_t state = REPORT_STATISTICS;

float calibration_resistance = 0;
float calibration_resistance_over_r0 = 0.0f;
uint32_t impliedR0 = 0;
uint32_t get_calibration_resistance();
float get_calibration_r_over_r0();
char r_prompt[] = "Calibration Resistance [integer]";    
char r_over_r0_prompt[] = "Target R over R0 [decimal]";  
#define CONFIRM_CALIBRATION_PARAMS_PROMPT "Confirm Calibration Parameters [yes/no]:"
void confirm_calibration_params_prompt(void);
void get_confirmation_string(char * tgt, uint8_t len);

void setup(){
  Serial.begin(9600);
  reportDiscoveredSensors();
  Serial.println(F("===================================================================="));
  promptForIndex();
  id_to_calibrate = getIdToCalibrate(); // blocks until you get a valid input
  
  // re-initialize the eggbus and fast-forward to the sensor we care about
  goToLogicalSensor(id_to_calibrate);  
  Serial.println(F("===================================================================="));
  Serial.println(F("time\tVal\tMin\tMax\tSpan\tMean\tLSE\tppm"));  
  Serial.println(F("--------------------------------------------------------------------"));   
}

void loop(){
  char ch = '\0';
  uint32_t sensor_resistance = 0;    
  char confirm_string[4] = "";
  char yes[] = "yes";
  char no[] = "no";
  switch(state){
  case REPORT_STATISTICS:
    sensor_resistance = (uint32_t) eggBus.getSensorResistance(target_sensor_index);
    sample_window[sample_window_idx++] = sensor_resistance;
    if(sample_window_idx > SAMPLE_WINDOW_SIZE) sample_window_idx = 0;
    if(num_samples_collected < SAMPLE_WINDOW_SIZE) num_samples_collected++;
    Serial.print(millis()/1000);
    Serial.print(F(" "));
    Serial.print(eggBus.getSensorType(target_sensor_index));
    Serial.print(F(": "));
    Serial.print(sensor_resistance);
    Serial.print(F("\t"));
    Serial.print(minResistance());    
    Serial.print(F("\t"));  
    Serial.print(maxResistance());    
    Serial.print(F("\t"));    
    Serial.print(maxResistance()-minResistance());    
    Serial.print(F("\t"));  
    Serial.print(meanResistance());      
    Serial.print(F("\t"));  
    Serial.print(sqrtSumSquaredErrorFromMean());          
    Serial.print(F("\t"));
    Serial.print(eggBus.getSensorValue(target_sensor_index));
    Serial.println();    
    delay(1000);
    
    if(Serial.available() >= 2){
      ch = Serial.read();
      Serial.print(ch);
      if(ch == ' '){
        ch = Serial.read();
        if(ch == '\n' || ch == '\r'){
          Serial.print(ch);    
          state = RECEIVE_CAL_R;    
        }
      } 
    }
    
    break;
  case RECEIVE_CAL_R:
    Serial.println();  
    promptFor(r_prompt);
    calibration_resistance = 1.0f * get_calibration_resistance(); // blocks until valid input
    state = RECEIVE_CAL_R_OVER_R0;
    break;  
  case RECEIVE_CAL_R_OVER_R0:
    Serial.println();  
    promptFor(r_over_r0_prompt);
    calibration_resistance_over_r0 = get_calibration_r_over_r0(); // blocks until valid input
    state = CONFIRM_CAL_PARAMS;
    break;   
  case CONFIRM_CAL_PARAMS:
    Serial.println();
    Serial.print(F("Calibration Resistance: "));
    Serial.println(calibration_resistance, 3);
    Serial.print(F("Calibration R/R0: "));
    Serial.println(calibration_resistance_over_r0);
    Serial.print(F("Implied R0: "));
    impliedR0 = (uint32_t) (calibration_resistance / calibration_resistance_over_r0);
    Serial.println(impliedR0);
    
    confirm_calibration_params_prompt();
    get_confirmation_string(confirm_string, 5);    
    if(strncmp(yes, confirm_string, 3) == 0){
      state = SET_CAL_PARAMS;
    }
    else if(strncmp(no, confirm_string, 2) == 0){
      state = RECEIVE_CAL_R;
    }
    confirm_string[0] = '\0';    
    break;
  case SET_CAL_PARAMS:
    Serial.println();
    Serial.print(F("Applying Calibration Value..."));
    eggBus.setSensorR0(target_sensor_index, impliedR0);
    Serial.print(F("Done"));
    Serial.println();
    soft_restart();
    break;
  default:
    Serial.println(F("IMPOSSIBLE!"));
    break;
  }
}

void printAddress(uint8_t * address){
  for(uint8_t jj = 0; jj < 6; jj++){
    if(address[jj] < 16) Serial.print("0");
    Serial.print(address[jj], HEX);
    if(jj != 5 ) Serial.print(":");
  }
  Serial.println();
}

void reportDiscoveredSensors(){
  uint8_t   egg_bus_address;
  uint8_t   logical_bus_address = 1;
  
  Serial.println(F("Discovered Sensors"));
  Serial.println(F("===================================================================="));
  Serial.println(F("ID\tSensor\tR0"));
  Serial.println(F("--------------------------------------------------------------------"));   
  
  eggBus.init();
  while((egg_bus_address = eggBus.next())){
    uint8_t numSensors = eggBus.getNumSensors();
    for(uint8_t ii = 0; ii < numSensors; ii++){
      uint32_t r0 = eggBus.getSensorR0(ii);
      Serial.print(logical_bus_address);
      Serial.print(F(":\t"));
      Serial.print(eggBus.getSensorType(ii));
      Serial.print(F("\t[ "));
      Serial.print(r0);
      Serial.print(F(" ] "));
      Serial.println();
      logical_bus_address++;      
    }    
  }   
  
  num_ids_total = logical_bus_address - 1;
}

uint8_t getIdToCalibrate(){
  char buf[4] = {0}; // must hold "1\0" up to "255\0"
  char idx = 0;
  char ch = 0;
  for(;;){ // until you get a valid entry
    if(Serial.available()){
      ch = Serial.read();
      Serial.print(ch);
      if(ch != '\n' && ch != '\r') buf[idx++] = ch;      
      if(idx == 4 || ch == '\n' || ch == '\r'){
        if(validSensorId(buf)){
          Serial.print(F("\nCalibrating Sensor ID: "));
          Serial.print(id_to_calibrate);
          Serial.print(F("Hit space then Enter to stop"));
          Serial.println();
          return id_to_calibrate;
        }
        else{
          idx = 0;
          for(uint8_t ii = 0; ii < 4; ii++) buf[ii] = 0;     
           promptForIndex();         
        }
      } 
    }
  }
  
}

uint32_t get_calibration_resistance(){
  char buf[16] = {0};
  char idx = 0;
  char ch = 0;
  for(;;){ // until you get a valid entry
    if(Serial.available()){
      ch = Serial.read();
      Serial.print(ch);
      if(ch != '\n' && ch != '\r') buf[idx++] = ch;      
      if(idx == 16 || ch == '\n' || ch == '\r'){
        calibration_resistance = parseDecimal(buf);
        if(calibration_resistance > 0.0f){
          return calibration_resistance;
        }
        else{
          idx = 0;
          for(uint8_t ii = 0; ii < 16; ii++) buf[ii] = 0;     
          Serial.println();
          promptFor(r_prompt);         
        }
      } 
    }
  }    
}

float get_calibration_r_over_r0(){
  char buf[16] = {0};
  char idx = 0;
  char ch = 0;
  for(;;){ // until you get a valid entry
    if(Serial.available()){
      ch = Serial.read();
      Serial.print(ch);
      if(ch != '\n' && ch != '\r') buf[idx++] = ch;      
      if(idx == 16 || ch == '\n' || ch == '\r'){
        calibration_resistance_over_r0 = parseDecimal(buf);
        if(calibration_resistance_over_r0 > 0.0f){ 
          return calibration_resistance_over_r0;
        }
        else{
          idx = 0;
          for(uint8_t ii = 0; ii < 16; ii++) buf[ii] = 0;     
          Serial.println();
          promptFor(r_over_r0_prompt);         
        }
      } 
    }
  }  
}

void get_confirmation_string(char * tgt, uint8_t len){
  char idx = 0;
  char ch = 0;
  for(;;){ // until you get a valid entry
    if(Serial.available()){
      ch = Serial.read();
      Serial.print(ch);
      if(ch != '\n' && ch != '\r') tgt[idx++] = ch;    
      tgt[idx] = '\0';  
      if(idx == len-1 || ch == '\n' || ch == '\r'){        
        return;
      } 

    }
  }    
}

uint8_t validSensorId(char * str){
  long value = 0;
 
  if(strlen(str) == 0){
    return 0; //quietly return 
  }
  
  value = parseLong(str);
  
  if(value < 0){
    Serial.println(F("\nCould not convert input to a number - Please Try Again"));
    return 0;
  }

  if(value == 0 || value > num_ids_total){
    Serial.println(F("\nEntered value is out of range - Please Try Again"));
    return 0;    
  }
  
  id_to_calibrate = value;
  return 1;
}

long parseLong(char * str)
{
    long _val = 0;
    char * temp;

    _val = strtoul(str, &temp, 0);

    if(*temp != '\0' || errno != 0 ){
      _val = -1;
    }

    return _val;
}

float parseDecimal(char * str){
  // first make sure there are only numbers and decimal points
  uint8_t decimal_point_count = 0;
  uint8_t ii = 0;
  uint8_t flag = 0;  
  uint8_t starts_with_decimal = 0;
  int32_t integer_part = -1, decimal_part = -1;
  uint8_t tokens_processed = 0;
  float decimal_divider = 1.0f;
  
  char * token = trimwhitespace(str); // note this mutilates the string a bit (on purpose)
  float ret_val = -1.0f;
  uint8_t string_length = strlen(str);
  
  if(string_length == 0) return ret_val; 
  
  if(token[0] == '.'){
    starts_with_decimal = 1;
  }
  
  for(ii = 0; ii < string_length; ii++){
    if(str[ii] == '.'){
      decimal_point_count++;
    }
    else if(str[ii] < '0' || str[ii] > '9'){ // it's neither a number nor a decimal point
      flag = 1;
      Serial.println();
      Serial.print(F("Invalid Character ["));      
      Serial.print((byte) str[ii]);
      Serial.print(F("] @ index "));
      Serial.print(ii);
    }      
  }
  
  if(decimal_point_count > 1){
    flag = 1; 
    Serial.println();    
    Serial.print(F("Too Many Decimal Points ["));    
    Serial.print(decimal_point_count);
    Serial.print(F("]"));
    Serial.println();
  }
  
  if(flag){ // return -1.0f since ret_val is unmodified
    Serial.println();  
    return ret_val; 
  }
  
  // it's valid... now convert it
  token = strtok(token, ".");

  while(token != NULL){
    //Serial.println();    
    //Serial.print(F("token: "));
    //Serial.print(token);
    if(tokens_processed == 0){
      if(starts_with_decimal){ // if there's no leading zero
        integer_part = 0;
        decimal_part = parseLong(token);
        ii = 0;
        decimal_divider = 1.0f;
        while(token[ii++] == '0'){
          decimal_divider *= 10.0f;
        }
      }
      else{ // it's a normal case xxx.yyy or xxx
        integer_part = parseLong(token);
      }
      
      if(decimal_point_count == 0){ // if there's no decimal - it's a pure integer i.e. xxx
        decimal_part = 0; 
      }
    }
    else if(tokens_processed == 1 && decimal_part < 0){ // it's not a pure integer , it's xxx.yyy
      decimal_part = parseLong(token);
      ii = 0;
      decimal_divider = 1.0f;
      while(token[ii++] == '0'){
        decimal_divider *= 10.0f;
      }      
    }
    tokens_processed++;
    token = strtok(NULL, ".");
  }
  //Serial.println();    

  if(integer_part < 0 || decimal_part < 0){ //error
    Serial.println();
    Serial.print(F("Invalid Input: Integer Part ["));
    Serial.print(integer_part);
    Serial.print(F("] Decimal Part ["));
    Serial.print(decimal_part);    
    Serial.print(F("]"));
    Serial.println();
    return ret_val; 
  }
  
  //Serial.println();
  //Serial.println(integer_part);
  //Serial.println(decimal_part);
  //Serial.println(decimal_divider);
  ret_val= 0;
  ret_val = 1.0f * decimal_part;
  //Serial.println(ret_val);
  //Serial.println("loop");
  while(ret_val >= 1.0f){
    ret_val /= 10.0f;
    //Serial.println(ret_val);    
  }
  ret_val /= decimal_divider;
  
  ret_val += 1.0f * integer_part;  
  //Serial.println(ret_val); 
  
  return ret_val;
}

void promptForIndex(){
  Serial.print(F("Enter ID Number to Calibrate [1.."));
  Serial.print(num_ids_total);
  Serial.print(F("]: "));        
}

void promptFor(char * str){
  Serial.print(F("Enter current value of "));
  Serial.print(str);
  Serial.print(F(": "));
}

void goToLogicalSensor(uint8_t target_logical_id){
  uint8_t   egg_bus_address;
  uint8_t   logical_bus_address = 1; 
  
  eggBus.init();
  while((egg_bus_address = eggBus.next())){
    uint8_t numSensors = eggBus.getNumSensors();
    for(uint8_t ii = 0; ii < numSensors; ii++){
      target_sensor_index = ii;      
      if(target_logical_id == logical_bus_address) return;
      logical_bus_address++;      
    }    
  }  
}

uint32_t sqrtSumSquaredErrorFromMean(){
  uint32_t mean = meanResistance();
  float r = 0.0f;
  uint8_t total_samples = min(num_samples_collected, SAMPLE_WINDOW_SIZE);
  for(uint8_t ii = 0; ii < total_samples; ii++){
    float diff = 1.0f * ((int32_t) sample_window[ii] - (int32_t) mean);
    r += diff * diff;
  }
  
  return (uint32_t) sqrt(r);
}

uint32_t minResistance(){
  uint32_t r = 99999999;
  uint8_t total_samples = min(num_samples_collected, SAMPLE_WINDOW_SIZE);
  for(uint8_t ii = 0; ii < total_samples; ii++){
    if(sample_window[ii] < r) r = sample_window[ii];
  }
  
  return r;
}

uint32_t maxResistance(){
  uint32_t r = 0;
  uint8_t total_samples = min(num_samples_collected, SAMPLE_WINDOW_SIZE);
  for(uint8_t ii = 0; ii < total_samples; ii++){
    if(sample_window[ii] > r) r = sample_window[ii];
  }
  
  return r;  
}

uint32_t meanResistance(){
  float r = 0.0;
  uint8_t total_samples = min(num_samples_collected, SAMPLE_WINDOW_SIZE);
  for(uint8_t ii = 0; ii < total_samples; ii++){
     r += sample_window[ii];
  }
  
  return (uint32_t) (r / total_samples);
}

char *trimwhitespace(char *str){
  char *pend;

  // Trim leading space
  while(isspace(*str)) str++;

  if(*str == 0)  // All spaces?
    return str;

  // Trim trailing space
  pend = str + strlen(str) - 1;
  while(pend > str && isspace(*pend)) pend--;

  // Write new null terminator
  *(pend+1) = 0;

  return str;
}

void confirm_calibration_params_prompt(void){
  Serial.print(F(CONFIRM_CALIBRATION_PARAMS_PROMPT));
}

