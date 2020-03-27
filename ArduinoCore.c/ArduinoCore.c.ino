

//#include <ESP8266WiFi.h>
#include <WiFi.h>
#include <aREST.h>
#include <Ticker.h>  //Ticker Library
#include <Wire.h>
#include <math.h>
#include <sfm3000wedo.h>


#define VERBOSE_LEVEL 3
#define LISTEN_PORT           80

#define N_PRESSURE_SENSORS 1

#define TIMERCORE_INTERVAL_MS        100 

#define VALVE_CLOSE 0
#define VALVE_OPEN 100

#define VALVE_IN_PIN A0 //15//14  (25)
#define VALVE_OUT_PIN A1 //2 //12   (34)

#define SIMULATE_SENSORS 1


SFM3000wedo measflow(64);


typedef enum {FR_OPEN_INVALVE, FR_WAIT_INHALE_PRESSURE, FR_WAIT_INHALE_PRESSURE_EXTRA, FR_WAIT_INHALE_TIME, FR_OPEN_OUTVALVE, FR_WAIT_EXHALE_PRESSURE, FR_WAIT_EXHALE_PRESSURE_EXTRA, FR_WAIT_EXHALE_TIME} t_core__force_sm;
typedef enum {ALARM_NO_INHALE_PRESSURE_IN_TIME, ALARM_NO_EXHALE_PRESSURE_IN_TIME, PRESSURE_DROP_INHALE, UNABLE_TO_READ_SENSOR_PRESSURE, ALARM_PRESSURE_TO_HIGH} t_ALARM;
typedef enum {VALVE_IN, VALVE_OUT} valves;


// WiFi parameters
const char* ssid = "test";
const char* password =  "alchimista";

// The port to listen for incoming TCP connections


Ticker CoreTask;
// Create aREST instance
aREST rest = aREST();
// Create an instance of the server
WiFiServer server(LISTEN_PORT);

// Variables to be exposed to the API
float temperature;
bool in_pressure_alarm=false;

typedef enum {M_BREATH_FORCED, M_BREATH_ASSISTED} t_assist_mode;
struct
{
  bool run;
  bool constant_rate_mode;
  uint16_t inhale_ms;
  uint16_t exhale_ms;
  uint16_t inhale_ms_extra;
  float pressure_forced_inhale_max;
  float pressure_forced_exhale_min;
  float pressure_drop;

  uint16_t inhale_critical_alarm_ms;
  uint16_t exhale_critical_alarm_ms;
  t_assist_mode BreathMode;

  float pressure_alarm;
  float pressure_alarm_off;

  struct
  {
    float rate_inhale_pressure;
    float rate_exhale_pressure;
  } sim;
} core_config;

typedef struct
{
  float last_pressure;
  long  read_millis;
} t_pressure;
t_pressure pressure[1];

struct 
{
  t_core__force_sm force_sm =FR_OPEN_INVALVE;
  unsigned long timer1;  
  unsigned long timer2;  
} core_sm_context;


typedef struct
{
  int32_t C[6];
} t_5525DSO_calibration_table;


uint8_t pressure_sensor_i2c_address [] = {0x77};
t_5525DSO_calibration_table PRES_SENS_CT[N_PRESSURE_SENSORS];

// Declare functions to be exposed to the API
int API_RUN_Control(String command);
int API_SET_inhale_ms(String command);
int API_SET_exhale_ms(String command);
int API_SET_inhale_critical_alarm_ms(String command);
int API_SET_exhale_critical_alarm_ms(String command);
int API_SET_pressure_forced_inhale_max(String command);
int API_SET_pressure_forced_exhale_min(String command);
int API_SET_pressure_drop(String command);

int read_pressure_sensor(int idx);
int valve_contol(valves valve, int level);
void CoreSM_FORCE_ChangeState(t_core__force_sm *sm, t_core__force_sm NEW_STATE);
void DBG_print(int level, String str);
void TriggerAlarm(t_ALARM Alarm);

void CalibrateDate_5525DSO(t_5525DSO_calibration_table CT, int32_t raw_temp, int32_t raw_pressure, float *T, float *P);
bool Convert_5525DSO(int address, int32_t *temp, int32_t *pressure);
bool Reset_5525DSO(int address);
bool ReadCalibration_5525DSO(int address, t_5525DSO_calibration_table *ct);
bool FirstConversion_5525DSO(int address);

void DBG_print(int level, String str)
{
  if (level<=VERBOSE_LEVEL)
  {
    Serial.println(str);
  }
}

void CoreSM_FORCE_ChangeState(t_core__force_sm *sm, t_core__force_sm NEW_STATE)
{
  *sm = NEW_STATE;
}

void TriggerAlarm(t_ALARM Alarm)
{
  switch(Alarm)
  {
    case ALARM_NO_INHALE_PRESSURE_IN_TIME:
      DBG_print(0,"ALARM @ " + String(millis()) + " ALARM_NO_INHALE_PRESSURE_IN_TIME");
    break;

    case ALARM_NO_EXHALE_PRESSURE_IN_TIME:
      DBG_print(0,"ALARM @ " + String(millis()) + " ALARM_NO_EXHALE_PRESSURE_IN_TIME");
    break;

    case PRESSURE_DROP_INHALE:
      DBG_print(0,"ALARM @ " + String(millis()) + " PRESSURE_DROP_INHALE");
    break;
    
    case UNABLE_TO_READ_SENSOR_PRESSURE:
      DBG_print(0,"ALARM @ " + String(millis()) + " UNABLE_TO_READ_SENSOR_PRESSURE");
    break;

    case ALARM_PRESSURE_TO_HIGH:
      DBG_print(0,"ALARM @ " + String(millis()) + " ALARM_PRESSURE_TO_HIGH");
    break;

    
    
    default:

    break;
  }
}
void  onTimerCoreTask(){
  
  DBG_print(10,"ITimer0: millis() = " + String(millis()));
  


  //This is the core of the ventialtor.
  //This section must work inside a SO timer and must be called every 0.1 seconds
  if (core_config.BreathMode == M_BREATH_FORCED)
  {
    core_sm_context.timer1++;
    core_sm_context.timer2++;
    SimulationFunction();

    if (pressure[0].last_pressure <= core_config.pressure_alarm_off)
    {
       in_pressure_alarm=false;
    }
    if ( (pressure[0].last_pressure >= core_config.pressure_alarm) || (in_pressure_alarm==true))
    {
      TriggerAlarm(ALARM_PRESSURE_TO_HIGH);
      valve_contol(VALVE_IN, VALVE_CLOSE);
      valve_contol(VALVE_OUT, VALVE_OPEN);
      in_pressure_alarm=true;
    }
    else
    {
      switch(core_sm_context.force_sm)
      {
        case FR_OPEN_INVALVE:
          if (core_config.run)
          {
            DBG_print(3,"FR_OPEN_INVALVE");
            valve_contol(VALVE_IN, VALVE_OPEN);
            if (core_config.constant_rate_mode)
              CoreSM_FORCE_ChangeState(&core_sm_context.force_sm, FR_WAIT_INHALE_TIME);
            else
              CoreSM_FORCE_ChangeState(&core_sm_context.force_sm, FR_WAIT_INHALE_PRESSURE);
  
            core_sm_context.timer1 =0;
            DBG_print(3,"FR_WAIT_INHALE_PRESSURE");
          }
          break;
  
        case FR_WAIT_INHALE_PRESSURE:
          if (pressure[0].last_pressure >= core_config.pressure_forced_inhale_max)
          {
            if (core_config.inhale_ms_extra>0)
            {
              core_sm_context.timer2 = 0; 
              CoreSM_FORCE_ChangeState(&core_sm_context.force_sm, FR_WAIT_INHALE_PRESSURE_EXTRA);
              DBG_print(3,"FR_WAIT_INHALE_PRESSURE_EXTRA");             
            }
            else
            {
  
              valve_contol(VALVE_IN, VALVE_CLOSE);
              CoreSM_FORCE_ChangeState(&core_sm_context.force_sm, FR_WAIT_INHALE_TIME);
              DBG_print(3,"FR_WAIT_INHALE_TIME");            
            }
          }
          else
          {
            if (core_sm_context.timer1 >= (core_config.inhale_critical_alarm_ms/TIMERCORE_INTERVAL_MS))
            {
              TriggerAlarm(ALARM_NO_INHALE_PRESSURE_IN_TIME);
              valve_contol(VALVE_IN, VALVE_CLOSE);
              CoreSM_FORCE_ChangeState(&core_sm_context.force_sm, FR_OPEN_OUTVALVE);
              DBG_print(3,"FR_OPEN_OUTVALVE");
            }
          }
          
          break;
  
        case FR_WAIT_INHALE_PRESSURE_EXTRA:
            if ( (core_sm_context.timer2> (core_config.inhale_ms_extra/TIMERCORE_INTERVAL_MS)) ||
            (core_sm_context.timer1> (core_config.inhale_ms/TIMERCORE_INTERVAL_MS)) )
            {
              valve_contol(VALVE_IN, VALVE_CLOSE);
              CoreSM_FORCE_ChangeState(&core_sm_context.force_sm, FR_WAIT_INHALE_TIME);
              DBG_print(3,"FR_OPEN_OUTVALVE");
            }
            else
            {        
              if (pressure[0].last_pressure <= core_config.pressure_drop)
              {
                TriggerAlarm(ALARM_NO_INHALE_PRESSURE_IN_TIME);
              }            
            }
          break;
          
        case FR_WAIT_INHALE_TIME:
            if (core_sm_context.timer1> (core_config.inhale_ms/TIMERCORE_INTERVAL_MS))
            {
              CoreSM_FORCE_ChangeState(&core_sm_context.force_sm, FR_OPEN_OUTVALVE);
              
              if (core_config.constant_rate_mode)
                valve_contol(VALVE_IN, VALVE_CLOSE);
                
              DBG_print(3,"FR_OPEN_OUTVALVE");
            }
            else
            {
              if (pressure[0].last_pressure <= core_config.pressure_drop)
              {
                TriggerAlarm(ALARM_NO_INHALE_PRESSURE_IN_TIME);
              }
            }
          break;
        
        case FR_OPEN_OUTVALVE:
          valve_contol(VALVE_OUT, VALVE_OPEN);
          CoreSM_FORCE_ChangeState(&core_sm_context.force_sm, FR_WAIT_EXHALE_TIME);
          //if (core_config.constant_rate_mode)
          //  CoreSM_FORCE_ChangeState(&core_sm_context.force_sm, FR_WAIT_EXHALE_TIME);
          //else
          //  CoreSM_FORCE_ChangeState(&core_sm_context.force_sm, FR_WAIT_EXHALE_PRESSURE);
            
          core_sm_context.timer1 =0;
          DBG_print(3,"FR_WAIT_EXHALE_TIME");
          
          break;
  
        case FR_WAIT_EXHALE_PRESSURE:
          if (pressure[0].last_pressure <= core_config.pressure_forced_exhale_min)
          {
            valve_contol(VALVE_OUT, VALVE_CLOSE);
            CoreSM_FORCE_ChangeState(&core_sm_context.force_sm, FR_WAIT_EXHALE_TIME);
            DBG_print(3,"FR_WAIT_EXHALE_TIME");
          }
          else
          {
            if (core_sm_context.timer1 >= (core_config.exhale_critical_alarm_ms/TIMERCORE_INTERVAL_MS))
            {
              TriggerAlarm(ALARM_NO_EXHALE_PRESSURE_IN_TIME);
              valve_contol(VALVE_OUT, VALVE_CLOSE);
              CoreSM_FORCE_ChangeState(&core_sm_context.force_sm, FR_OPEN_INVALVE);
              DBG_print(3,"FR_OPEN_INVALVE");
            }
          }
          break;
  
        case FR_WAIT_EXHALE_TIME:
          if (core_sm_context.timer1> (core_config.exhale_ms/TIMERCORE_INTERVAL_MS))
          {
            valve_contol(VALVE_OUT, VALVE_CLOSE);
            CoreSM_FORCE_ChangeState(&core_sm_context.force_sm, FR_OPEN_INVALVE);
            if (core_config.constant_rate_mode)
                valve_contol(VALVE_OUT, VALVE_CLOSE);
                
            DBG_print(3,"FR_OPEN_INVALVE");
          }
          break;
  
        default:
          CoreSM_FORCE_ChangeState(&core_sm_context.force_sm, FR_OPEN_INVALVE);
          break;
      }
    }
  }
  else
  {
    if (core_config.BreathMode == M_BREATH_ASSISTED)  
    {
      
    }
    else
    {
      
    }
  }

      

}



void InitParameters()
{
  core_config.run=true;
  core_config.constant_rate_mode = false;
  core_config.inhale_ms = 2000;
  core_config.inhale_ms_extra = 500;
  core_config.exhale_ms = 1000;
  core_config.pressure_alarm = 50;
  core_config.pressure_alarm_off = 10;
  core_config.pressure_forced_inhale_max = 30;
  core_config.pressure_forced_exhale_min = 5;
  core_config.pressure_drop = 20;
  core_config.inhale_critical_alarm_ms = 3000;
  core_config.exhale_critical_alarm_ms = 2500;
  core_config.BreathMode = M_BREATH_FORCED;
  core_config.sim.rate_inhale_pressure=5;
  core_config.sim.rate_exhale_pressure=10;  
}

void setup(void)
{
  pinMode (VALVE_IN_PIN, OUTPUT);
  pinMode (VALVE_OUT_PIN, OUTPUT);
  // Start Serial
  Serial.begin(115200);
  Wire.begin();
  
  for (int j=0;j<N_PRESSURE_SENSORS;j++)
  {
    Reset_5525DSO((int) pressure_sensor_i2c_address [j]);
  }

  delay(100);
  for (int j=0;j<N_PRESSURE_SENSORS;j++)
  {
    FirstConversion_5525DSO((int) pressure_sensor_i2c_address [j]);
  }

  delay(100);
  
  InitParameters();

  CoreTask.attach(TIMERCORE_INTERVAL_MS/1000.0, onTimerCoreTask);

  
  temperature = 24;

  rest.variable("temperature",&temperature);
  rest.variable("pressure",&pressure[0].last_pressure);

  // Function to be exposed
  rest.function("run",API_RUN_Control);
  rest.function("inhale_ms",API_SET_inhale_ms);
  rest.function("exhale_ms",API_SET_exhale_ms);
  rest.function("inhale_alarm_ms",API_SET_inhale_critical_alarm_ms);
  rest.function("exhale_alarm_ms",API_SET_exhale_critical_alarm_ms);
  rest.function("pressure_max",API_SET_pressure_forced_inhale_max);
  rest.function("pressure_min",API_SET_pressure_forced_exhale_min);
  rest.function("pressure_drop",API_SET_pressure_drop);

  


  // Give name & ID to the device (ID should be 6 characters long)
  rest.set_id("1");
  rest.set_name("MVM");


 
  // Connect to WiFi
 /* WiFi.begin((char*)ssid, (char*)password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
*/

  // Connect to Wi-Fi network with SSID and password
  Serial.print("Setting AP (Access Point)…");
  // Remove the password parameter, if you want the AP (Access Point) to be open
  WiFi.softAP(ssid, password);


  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  // Print ESP8266 Local IP Address
  Serial.println(WiFi.localIP());
  
  // Start the server
  server.begin();
  Serial.println("Server started");

  // Print the IP address
  Serial.println(WiFi.localIP());

  for (int j=0;j<N_PRESSURE_SENSORS;j++)
  {
    ReadCalibration_5525DSO( (int) pressure_sensor_i2c_address [j], &PRES_SENS_CT[j]);
    Serial.print("SENSOR:           ");   Serial.println(j);
    Serial.print("SENS_T1:          ");   Serial.println(PRES_SENS_CT[j].C[0]);
    Serial.print("OFF_T1:           ");   Serial.println(PRES_SENS_CT[j].C[1]);
    Serial.print("TCS:              ");   Serial.println(PRES_SENS_CT[j].C[2]);
    Serial.print("TCO:              ");   Serial.println(PRES_SENS_CT[j].C[3]);
    Serial.print("TREF:             ");   Serial.println(PRES_SENS_CT[j].C[4]);
    Serial.print("TEMPSENS:         ");   Serial.println(PRES_SENS_CT[j].C[5]);
  }
  
}



void loop() {
  static WiFiClient client;
  static uint32_t last_loop_time;
  static uint8_t RestStateMachine =0;
  static int serverhanging = 0; // Used to monitor how long the client is hanging
  static int serverhangingrestartdelay = 500; // delay after which we discard a hanging client
  static uint32_t sensor_read_last_time =0;
  
  if (millis() > sensor_read_last_time + 100)
  {
    if (read_pressure_sensor(0)!= 0)
    {
      TriggerAlarm(UNABLE_TO_READ_SENSOR_PRESSURE);
    }
    sensor_read_last_time= millis();
  }

    //__service_i2c_detect();
    

  switch(RestStateMachine)
  {
    case 0:
      client = server.available();
      serverhanging = 0; // client not hanging, reset timer
      last_loop_time=0;
      if (client) {
        RestStateMachine=1;
      }
    break;

    case 1:
      if(!client.available())
      {
        if (millis() > last_loop_time)
        {
          serverhanging += 1;
          if(serverhanging >= serverhangingrestartdelay){ // if hanging for too long, discard the client
            client.stop();
            client.flush();
            serverhanging = 0;
            RestStateMachine=3;
          }
          last_loop_time=millis();
        }
      }
      else
      {
        RestStateMachine=2;
      }
    break;

    case 2:
    
      rest.handle(client);
      client.stop();
      RestStateMachine=0;
    break;

    case 3:
      if (millis() > last_loop_time + 10)
      {
        RestStateMachine=0;
      }
    break;

  }
   

}

// Custom function accessible by the API
int API_RUN_Control(String command) {

  // Get state from command
  int state = command.toInt();

  if ((core_config.run==false) && (state==1))
  {
    core_config.run=true;
    return 0;
  }
  else
  {
    if ((core_config.run==true) && (state==0))
    {
      core_config.run=false;
      return 0;
    }
    else
    {
      return -1;
    }
  }
    
  
}


int read_pressure_sensor(int idx)
{
  if (idx < N_PRESSURE_SENSORS)
  {

    if (SIMULATE_SENSORS==1)
    {
      
    }
    else
    {
      uint8_t i2c_address = pressure_sensor_i2c_address[idx];
      int32_t raw_temp;
      int32_t raw_pressure;
      float T;
      float P;
      
      if (Convert_5525DSO((int)i2c_address, &raw_temp, &raw_pressure))
      {
        CalibrateDate_5525DSO(PRES_SENS_CT[idx], raw_temp, raw_pressure, &T, &P);
        DBG_print(3,"TEMPERATURE:      " + String(T));
        DBG_print(3,"PRESSURE:      " + String(P));
        pressure[0].last_pressure = P;
        pressure[0].read_millis=millis();  
      }
    }

    return 0;
  }
  else
    return -1;
}


int valve_contol(valves valve, int level)
{
  if (valve == VALVE_IN)
  {
    if (level==VALVE_CLOSE)
    {
      digitalWrite(VALVE_IN_PIN, LOW);
      DBG_print(3,"VALVE IN CLOSED");      
    } 
    else if (level==VALVE_OPEN)
    {
      digitalWrite(VALVE_IN_PIN, HIGH);
      DBG_print(3,"VALVE IN OPENED");
    }
    else
    {
        //PWM MODE TO BE TESTED
         
    }

    //is there a way to have a feedback that the valve is working
    //insert here
    return 0;
  }
  else
  {
    if (valve == VALVE_OUT)
    {
      if (level==VALVE_CLOSE)
      {
        digitalWrite(VALVE_OUT_PIN, LOW);
        DBG_print(3,"VALVE OUT CLOSED");
      } 
      else if (level==VALVE_OPEN)
      {
        digitalWrite(VALVE_OUT_PIN, HIGH);
        DBG_print(3,"VALVE OUT OPENED");
      }
      else
      {
          //PWM MODE TO BE TESTED
           
      }
      
      //is there a way to have a feedback that the valve is working
      //insert here
      return 0;
    }
    else
    {
      return -1;
    }
  }

}



void SimulationFunction()
{
  float r3;
  float LO;
  float HI;
  if (SIMULATE_SENSORS != 1)
  {
    
  }
  else
  {
    switch(core_sm_context.force_sm)
    {
      case FR_OPEN_INVALVE:
        pressure[0].last_pressure = 0;
        break;

      case FR_WAIT_INHALE_PRESSURE:
        HI = core_config.sim.rate_inhale_pressure * 1.5;
        LO = core_config.sim.rate_inhale_pressure - (core_config.sim.rate_inhale_pressure*0.5);
        r3 = LO + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(HI-LO)));
        pressure[0].last_pressure += r3;
        break;

      case FR_WAIT_INHALE_TIME:
        ;
        break;

      case FR_OPEN_OUTVALVE:
        ;
        break;

      case FR_WAIT_EXHALE_PRESSURE:
        HI = core_config.sim.rate_exhale_pressure * 1.5;
        LO = core_config.sim.rate_exhale_pressure - (core_config.sim.rate_exhale_pressure*0.5);
        r3 = LO + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(HI-LO)));
        pressure[0].last_pressure -= r3;
        
        break;

      case FR_WAIT_EXHALE_TIME:
        ;
        break;

      default:
        
        break;
    }
  }
}



// Custom function accessible by the API
int API_SET_inhale_ms(String command) {
  int value = command.toInt();
  if ((value<0) && (value>15000)) return -1;
  core_config.inhale_ms = value;
  return 0;
}

int API_SET_exhale_ms(String command) {
  int value = command.toInt();
  if ((value<0) && (value>15000)) return -1;
  core_config.exhale_ms = value;
  return 0;
}

int API_SET_inhale_critical_alarm_ms(String command) {
  int value = command.toInt();
  if ((value<0) && (value>15000)) return -1;
  core_config.inhale_critical_alarm_ms = value;
  return 0;
}

int API_SET_exhale_critical_alarm_ms(String command) {
  int value = command.toInt();
  if ((value<0) && (value>15000)) return -1;
  core_config.exhale_critical_alarm_ms = value;
  return 0;
}

int API_SET_pressure_forced_inhale_max(String command) {
  float value = command.toFloat();
  if ((value<0) && (value>15000)) return -1;
  core_config.pressure_forced_inhale_max = value;
  return 0;
}

int API_SET_pressure_forced_exhale_min(String command) {
  float value = command.toFloat();
  if ((value<0) && (value>15000)) return -1;
  core_config.pressure_forced_exhale_min = value;
  return 0;
}

int API_SET_pressure_drop(String command) {
  float value = command.toFloat();
  if ((value<0) && (value>15000)) return -1;
  core_config.pressure_drop = value;
  return 0;
}


void __service_i2c_detect()
{
   byte error, address;
  int nDevices;
  Serial.println("Scanning... I2C");
  nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
      nDevices++;
    }
    else if (error==4) {
      Serial.print("Unknow error at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  }
  else {
    Serial.println("done\n");
  }

}

bool ReadCalibration_5525DSO(int address, t_5525DSO_calibration_table *ct)
{
  bool bres=true;
  int error;
  for (int i=0;i<6;i++)
  {
    Wire.beginTransmission(address);
    Wire.write(0xA0 + ((i+1)<<1)); // MSB 
    error = Wire.endTransmission(); 
    Wire.requestFrom(address,2);
    byte MSB = Wire.read();
    byte LSB = Wire.read();
    error = Wire.endTransmission(); 
    ct->C[i] = (MSB<<8)+LSB;
  }

  return bres;
}

bool Reset_5525DSO(int address)
{
  int error;
    Wire.beginTransmission(address);
    Wire.write(0x1E); // MSB 
    error = Wire.endTransmission(); 
  return true;
}

bool FirstConversion_5525DSO(int address)
{
    Wire.beginTransmission(address);
    Wire.write(0x58); // MSB 
    Wire.endTransmission(); 
    return true;
}

bool Convert_5525DSO(int address, int32_t *temp, int32_t *pressure)
{
  static int32_t last_temp;
  static int32_t last_pressure;
  static uint8_t temp_read=1;
  bool bres=true;
  int error;
  byte b1, b2, b3;
  
  Wire.beginTransmission(address);
    Wire.write(0x48); // MSB 
    error = Wire.endTransmission(); 

    delay(15);
    
    Wire.beginTransmission(address);
    Wire.write(0x00); // MSB 
    error = Wire.endTransmission(); 


    delay(1);
    
    error = Wire.requestFrom(address,3, true);
    if (error < 3) return false;
    b1 = Wire.read();
    b2 = Wire.read();
    b3 = Wire.read();

    *pressure = (b1<<16) + (b2<<8) + b3;
     
      
    Wire.beginTransmission(address);
    Wire.write(0x58); // MSB 
    error = Wire.endTransmission(); 

    delay(15);
    
    Wire.beginTransmission(address);
    Wire.write(0x00); // MSB 
    error = Wire.endTransmission(); 

    delay(1);
    
    error = Wire.requestFrom(address,3, true);
    if (error < 3) return false;
    b1 = Wire.read();
    b2 = Wire.read();
    b3 = Wire.read();
    *temp = (b1<<16) + (b2<<8) + b3;
  
   
  return true;
}

void CalibrateDate_5525DSO(t_5525DSO_calibration_table CT, int32_t raw_temp, int32_t raw_pressure, float *T, float *P)
{
  int32_t Q1 = 15;
  int32_t Q2 = 17;
  int32_t Q3 = 7;
  int32_t Q4 = 5;
  int32_t Q5 = 7;
  int32_t Q6 = 21;

  int32_t C1 = CT.C[0];
  int32_t C2 = CT.C[1];
  int32_t C3 = CT.C[2];
  int32_t C4 = CT.C[3];
  int32_t C5 = CT.C[4];
  int32_t C6 = CT.C[5];
    
  int32_t dT;
  int64_t OFF;
  int64_t SENS;

  int32_t Temp;
  int64_t Pres;
  //Serial.println(raw_temp);
  //Serial.println(raw_pressure);
  dT = raw_temp - (C5 * pow(2,Q5));
  Temp = 2000 + ( (dT*C6) /pow(2,Q6));
  OFF = (C2 * pow(2,Q2)) + ((C4 * dT)/(pow(2,Q4)));
  SENS = (C1 * pow(2,Q1)) + ((C3 * dT)/(pow(2,Q3)));
  Pres = (((raw_pressure * SENS)/(pow(2,21))) - OFF)/(pow(2,15));

  *T = ((float)Temp)/100.0;
  *P = ((float)Pres)/10000.0 * 68.9476;
  

}
