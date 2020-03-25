

 
//#include <ESP8266WiFi.h>
#include <WiFi.h>
#include <aREST.h>
#include <Ticker.h>  //Ticker Library

#define VERBOSE_LEVEL 3
#define LISTEN_PORT           80

#define N_PRESSURE_SENSORS 1

#define TIMERCORE_INTERVAL_MS        100 

#define VALVE_CLOSE 0
#define VALVE_OPEN 100

#define VALVE_IN_PIN 15//14
#define VALVE_OUT_PIN 2 //12

#define SIMULATE_SENSORS 1



typedef enum {FR_OPEN_INVALVE, FR_WAIT_INHALE_PRESSURE, FR_WAIT_INHALE_TIME, FR_OPEN_OUTVALVE, FR_WAIT_EXHALE_PRESSURE, FR_WAIT_EXHALE_TIME} t_core__force_sm;
typedef enum {ALARM_NO_INHALE_PRESSURE_IN_TIME, ALARM_NO_EXHALE_PRESSURE_IN_TIME, PRESSURE_DROP_INHALE, UNABLE_TO_READ_SENSOR_PRESSURE} t_ALARM;
typedef enum {VALVE_IN, VALVE_OUT} valves;


// WiFi parameters
const char* ssid = "AbbaAndrea0.2.5";
const char* password =  "alchimista";

// The port to listen for incoming TCP connections


Ticker CoreTask;
// Create aREST instance
aREST rest = aREST();
// Create an instance of the server
WiFiServer server(LISTEN_PORT);

// Variables to be exposed to the API
float temperature;


typedef enum {M_BREATH_FORCED, M_BREATH_ASSISTED} t_assist_mode;
struct
{
  bool run;
  uint16_t inhale_ms;
  uint16_t exhale_ms;
  float pressure_forced_inhale_max;
  float pressure_forced_exhale_min;
  float pressure_drop;

  uint16_t inhale_critical_alarm_ms;
  uint16_t exhale_critical_alarm_ms;
  t_assist_mode BreathMode;


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
} core_sm_context;

uint8_t pressure_sensor_i2c_address [] = {0x56};
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
    
    default:

    break;
  }
}
void  onTimerCoreTask(){
  
  DBG_print(10,"ITimer0: millis() = " + String(millis()));
  DBG_print(3,"Pressure:  " + String(pressure[0].last_pressure));


  //This is the core of the ventialtor.
  //This section must work inside a SO timer and must be called every 0.1 seconds
  if (core_config.BreathMode == M_BREATH_FORCED)
  {
    core_sm_context.timer1++;
    SimulationFunction();
    switch(core_sm_context.force_sm)
    {
      case FR_OPEN_INVALVE:
        if (core_config.run)
        {
          DBG_print(3,"FR_OPEN_INVALVE");
          valve_contol(VALVE_IN, VALVE_OPEN);
          CoreSM_FORCE_ChangeState(&core_sm_context.force_sm, FR_WAIT_INHALE_PRESSURE);
          core_sm_context.timer1 =0;
          DBG_print(3,"FR_WAIT_INHALE_PRESSURE");
        }
        break;

      case FR_WAIT_INHALE_PRESSURE:
        if (pressure[0].last_pressure >= core_config.pressure_forced_inhale_max)
        {
          valve_contol(VALVE_IN, VALVE_CLOSE);
          CoreSM_FORCE_ChangeState(&core_sm_context.force_sm, FR_WAIT_INHALE_TIME);
          DBG_print(3,"FR_WAIT_INHALE_TIME");
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

      case FR_WAIT_INHALE_TIME:
          if (core_sm_context.timer1> (core_config.inhale_ms/TIMERCORE_INTERVAL_MS))
          {
            CoreSM_FORCE_ChangeState(&core_sm_context.force_sm, FR_OPEN_OUTVALVE);
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
        CoreSM_FORCE_ChangeState(&core_sm_context.force_sm, FR_WAIT_EXHALE_PRESSURE);
        core_sm_context.timer1 =0;
        DBG_print(3,"FR_WAIT_EXHALE_PRESSURE");
        
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
          CoreSM_FORCE_ChangeState(&core_sm_context.force_sm, FR_OPEN_INVALVE);
          DBG_print(3,"FR_OPEN_INVALVE");
        }
        break;

      default:
        CoreSM_FORCE_ChangeState(&core_sm_context.force_sm, FR_OPEN_INVALVE);
        break;
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
  core_config.run=false;
  core_config.inhale_ms = 3000;
  core_config.exhale_ms = 1200;
  core_config.pressure_forced_inhale_max = 100;
  core_config.pressure_forced_exhale_min = 15;
  core_config.pressure_drop = 20;
  core_config.inhale_critical_alarm_ms = 6000;
  core_config.exhale_critical_alarm_ms = 4000;
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

  InitParameters();

  CoreTask.attach(TIMERCORE_INTERVAL_MS/1000.0, onTimerCoreTask);
  
  temperature = 24;

  rest.variable("temperature",&temperature);
  rest.variable("pressure",&pressure[0].last_pressure);

  // Function to be exposed
  rest.function("run",API_RUN_Control);
  rest.function("inhale_ms",API_SET_inhale_ms);
  rest.function("exhale_ms",API_SET_exhale_ms);
  rest.function("inhale_critical_alarm_ms",API_SET_inhale_critical_alarm_ms);
  rest.function("exhale_critical_alarm_ms",API_SET_exhale_critical_alarm_ms);
  rest.function("pressure_forced_inhale_max",API_SET_pressure_forced_inhale_max);
  rest.function("pressure_forced_exhale_min",API_SET_pressure_forced_exhale_min);
  rest.function("pressure_drop",API_SET_pressure_drop);

  


  // Give name & ID to the device (ID should be 6 characters long)
  rest.set_id("1");
  rest.set_name("MVM");


 
  // Connect to WiFi
  WiFi.begin((char*)ssid, (char*)password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");


 /* // Connect to Wi-Fi network with SSID and password
  Serial.print("Setting AP (Access Point)…");
  // Remove the password parameter, if you want the AP (Access Point) to be open
  WiFi.softAP(ssid, password);


  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
*/
  // Print ESP8266 Local IP Address
  Serial.println(WiFi.localIP());
  
  // Start the server
  server.begin();
  Serial.println("Server started");

  // Print the IP address
  Serial.println(WiFi.localIP());

}



void loop() {
  static WiFiClient client;
  static uint32_t last_loop_time;
  static uint8_t RestStateMachine =0;
  static int serverhanging = 0; // Used to monitor how long the client is hanging
  static int serverhangingrestartdelay = 100; // delay after which we discard a hanging client

  if (read_pressure_sensor(0)!= 0)
  {
      TriggerAlarm(UNABLE_TO_READ_SENSOR_PRESSURE);
  }

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
      RestStateMachine=0;
    break;

    case 3:
      if (millis() > last_loop_time + 100)
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
      pressure[0].last_pressure = 10;
      pressure[0].read_millis=millis();  
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
