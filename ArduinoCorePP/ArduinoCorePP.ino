

//#include <ESP8266WiFi.h>
#include <WiFi.h>
#include <aREST.h>
#include <Ticker.h>  //Ticker Library
#include <Wire.h>
#include <math.h>



#define VERBOSE_LEVEL 1
#define LISTEN_PORT           80

#define N_PRESSURE_SENSORS 1

#define TIMERCORE_INTERVAL_MS        100 

#define VALVE_CLOSE 0
#define VALVE_OPEN 100

//#define VALVE_IN_PIN  A0 //15//14  (25)

#define DAC1 A0 //25
#define VALVE_OUT_PIN A1 //2 //12   (34)

#define SIMULATE_SENSORS 0


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

  float assist_pressure_delta_trigger;
  uint16_t inhale_critical_alarm_ms;
  uint16_t exhale_critical_alarm_ms;
  t_assist_mode BreathMode;

  float pressure_alarm;
  float pressure_alarm_off;

  float flux_close;
  float target_pressure;
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

typedef struct
{
  float last_flux;
  long  read_millis;
} t_flux;
t_flux gasflux[1];



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


uint8_t pressure_sensor_i2c_address [] = {0x76};
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
int API_SET_control_mode(String command);
int API_SET_assist_pressure_delta_trigger(String command);
int API_SET_inhale_ms_extra(String command);
int API_SET_costant_rate(String command);

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
void SimulationFunction();
bool MeasureFlux(float *Flow);
void MeasureFluxInit();
void PressureControlLoop_PRESSIN();
int valve1_status = 0;
int valve2_status = 0;

float Pset = 0;

float PIDMonitor = 0;

  
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
  return;
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
  static float old_pressure[5];
  DBG_print(10,"ITimer0: millis() = " + String(millis()));
  
  old_pressure[4] = old_pressure[3];
  old_pressure[3] = old_pressure[2];
  old_pressure[2] = old_pressure[1];
  old_pressure[1] = old_pressure[0];
  old_pressure[0] = pressure[0].last_pressure;

  float mean = (old_pressure[0] + old_pressure[1] + old_pressure[2])/3.0;
  //This is the core of the ventialtor.
  //This section must work inside a SO timer and must be called every 0.1 seconds
  //if (core_config.BreathMode == M_BREATH_FORCED)
  //{
    core_sm_context.timer1++;
    core_sm_context.timer2++;
    SimulationFunction();

    if (mean <= core_config.pressure_alarm_off)
    {
       in_pressure_alarm=false;
    }
    if ( (mean >= core_config.pressure_alarm) || (in_pressure_alarm==true))
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
            if (core_config.BreathMode == M_BREATH_FORCED)
            {
              //FORCE BREATHING MODE
              DBG_print(3,"FR_OPEN_INVALVE");
               core_sm_context.timer1 =0;
              valve_contol(VALVE_IN, VALVE_OPEN);
              if (core_config.constant_rate_mode)
                CoreSM_FORCE_ChangeState(&core_sm_context.force_sm, FR_WAIT_INHALE_TIME);
              else
                CoreSM_FORCE_ChangeState(&core_sm_context.force_sm, FR_WAIT_INHALE_PRESSURE);
    
              //core_sm_context.timer1 =0;
              DBG_print(3,"FR_WAIT_INHALE_PRESSURE");
            }
            else
            {
              //ASSISTED BREATHING MODE
             
              
              float delta =   pressure[0].last_pressure;
              valve_contol(VALVE_OUT, VALVE_OPEN);
              if (delta < core_config.assist_pressure_delta_trigger)
              {
                DBG_print(3,"FR_OPEN_INVALVE");
                valve_contol(VALVE_IN, VALVE_OPEN);
                valve_contol(VALVE_OUT, VALVE_CLOSE);
                if (core_config.constant_rate_mode)
                  CoreSM_FORCE_ChangeState(&core_sm_context.force_sm, FR_WAIT_INHALE_TIME);
                else
                  CoreSM_FORCE_ChangeState(&core_sm_context.force_sm, FR_WAIT_INHALE_PRESSURE);
                core_sm_context.timer1 =0;
                DBG_print(3,"FR_WAIT_INHALE_PRESSURE");                
              }
            }
          }
          break;
  
        case FR_WAIT_INHALE_PRESSURE:
          if (pressure[0].last_pressure >= core_config.pressure_forced_inhale_max)
          {
            //if (core_config.inhale_ms_extra>0)
            //{
              core_sm_context.timer2 = 0; 
              core_sm_context.timer1 =0;
              CoreSM_FORCE_ChangeState(&core_sm_context.force_sm, FR_WAIT_INHALE_PRESSURE_EXTRA);
              DBG_print(3,"FR_WAIT_INHALE_PRESSURE_EXTRA");             
            //}
            //else
            //{
  
            //  valve_contol(VALVE_IN, VALVE_CLOSE);
            //  CoreSM_FORCE_ChangeState(&core_sm_context.force_sm, FR_WAIT_INHALE_TIME);
            //  DBG_print(3,"FR_WAIT_INHALE_TIME");            
            //}
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
            //if ( (core_sm_context.timer2> (core_config.inhale_ms_extra/TIMERCORE_INTERVAL_MS)) ||
            //(core_sm_context.timer1> (core_config.inhale_ms/TIMERCORE_INTERVAL_MS)) )
            if ( (gasflux[0].last_flux < core_config.flux_close))// ||
            //(core_sm_context.timer1> (core_config.inhale_ms/TIMERCORE_INTERVAL_MS)) )
            {
              valve_contol(VALVE_IN, VALVE_CLOSE);
              CoreSM_FORCE_ChangeState(&core_sm_context.force_sm, FR_WAIT_INHALE_TIME);
              DBG_print(3,"FR_OPEN_OUTVALVE");
            }
            else
            {        
                       
            }
          break;
          
        case FR_WAIT_INHALE_TIME:
            if (core_sm_context.timer1> (core_config.inhale_ms/TIMERCORE_INTERVAL_MS))
            {
              CoreSM_FORCE_ChangeState(&core_sm_context.force_sm, FR_OPEN_OUTVALVE);
               core_sm_context.timer1 =0;
              if (core_config.constant_rate_mode)
                valve_contol(VALVE_IN, VALVE_CLOSE);
                
              DBG_print(3,"FR_OPEN_OUTVALVE");
            }
            else
            {
              if (pressure[0].last_pressure <= core_config.pressure_drop)
              {
                TriggerAlarm(PRESSURE_DROP_INHALE);
              }
            }
          break;
        
        case FR_OPEN_OUTVALVE:
          valve_contol(VALVE_OUT, VALVE_OPEN);
          //CoreSM_FORCE_ChangeState(&core_sm_context.force_sm, FR_WAIT_EXHALE_TIME);
          if (core_config.constant_rate_mode)
            CoreSM_FORCE_ChangeState(&core_sm_context.force_sm, FR_WAIT_EXHALE_TIME);
          else
            CoreSM_FORCE_ChangeState(&core_sm_context.force_sm, FR_WAIT_EXHALE_PRESSURE);
            
          core_sm_context.timer1 =0;
          DBG_print(3,"FR_WAIT_EXHALE_TIME");
          
          break;
  
        case FR_WAIT_EXHALE_PRESSURE:
          if (pressure[0].last_pressure <= core_config.pressure_forced_exhale_min)
          {
 //           valve_contol(VALVE_OUT, VALVE_CLOSE);
            CoreSM_FORCE_ChangeState(&core_sm_context.force_sm, FR_WAIT_EXHALE_TIME);
            core_sm_context.timer1 =0;
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
            if (core_config.BreathMode == M_BREATH_FORCED)
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
 /* }
  else
  {
    if (core_config.BreathMode == M_BREATH_ASSISTED)  
    {
      
    }
    else
    {
      
    }
  }*/

      

}



void InitParameters()
{
/*  
  core_config.run=true;
  core_config.constant_rate_mode = true;
  core_config.inhale_ms = 750;
  core_config.inhale_ms_extra = 00;
  core_config.exhale_ms = 1250;
  core_config.pressure_alarm = 100;
  core_config.pressure_alarm_off = 10;
  core_config.pressure_forced_inhale_max = 15;
  core_config.pressure_forced_exhale_min = 10;
  core_config.pressure_drop = 8;
  core_config.inhale_critical_alarm_ms = 16000;
  core_config.exhale_critical_alarm_ms = 16000;
  core_config.BreathMode = M_BREATH_FORCED; //M_BREATH_ASSISTED;//M_BREATH_FORCED;
  core_config.sim.rate_inhale_pressure=5;
  core_config.sim.rate_exhale_pressure=10;  
  core_config.flux_close = 5;
  core_config.assist_pressure_delta_trigger=5;
  core_config.target_pressure = 30;
  */

    core_config.run=true;
  core_config.constant_rate_mode = false;
  core_config.inhale_ms = 750;
  core_config.inhale_ms_extra = 00;
  core_config.exhale_ms = 1250;
  core_config.pressure_alarm = 100;
  core_config.pressure_alarm_off = 10;
  core_config.pressure_forced_inhale_max = 15;
  core_config.pressure_forced_exhale_min = 10;
  core_config.pressure_drop = 8;
  core_config.inhale_critical_alarm_ms = 16000;
  core_config.exhale_critical_alarm_ms = 16000;
  core_config.BreathMode = M_BREATH_ASSISTED; //M_BREATH_ASSISTED;//M_BREATH_FORCED;
  core_config.sim.rate_inhale_pressure=5;
  core_config.sim.rate_exhale_pressure=10;  
  core_config.flux_close = 30;
  core_config.assist_pressure_delta_trigger=2;
  core_config.target_pressure = 30;
/*
    core_config.run=true;
  core_config.constant_rate_mode = false;
  core_config.inhale_ms = 6000;//2000;
  core_config.inhale_ms_extra = 500;
  core_config.exhale_ms = 1000;
  core_config.pressure_alarm = 100;
  core_config.pressure_alarm_off = 10;
  core_config.pressure_forced_inhale_max = 10;
  core_config.pressure_forced_exhale_min = 3;
  core_config.pressure_drop = 20;
  core_config.inhale_critical_alarm_ms = 15000;
  core_config.exhale_critical_alarm_ms = 15000;
  core_config.BreathMode = M_BREATH_FORCED; //M_BREATH_ASSISTED
  core_config.sim.rate_inhale_pressure=5;
  core_config.sim.rate_exhale_pressure=10;  
  core_config.flux_close = 25;
  core_config.assist_pressure_delta_trigger=1.5;*/
}

void setup(void)
{
  //pinMode (VALVE_IN_PIN, OUTPUT);
  dacWrite(DAC1, 0);
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


  
  valve_contol(VALVE_IN, VALVE_CLOSE);
  valve_contol(VALVE_OUT, VALVE_OPEN);
  delay(3000);     
  valve_contol(VALVE_IN, VALVE_CLOSE);
  valve_contol(VALVE_OUT, VALVE_CLOSE);
   
  InitParameters();

  CoreTask.attach(TIMERCORE_INTERVAL_MS/1000.0, onTimerCoreTask);

  __service_i2c_detect();
  
  temperature = 24;

  //rest.variable("temperature",&temperature);
  rest.variable("pressure",&pressure[0].last_pressure);
  //rest.variable("flux",&gasflux[0].last_flux);
  // Function to be exposed
  rest.function("run",API_RUN_Control);
  rest.function("inhale_ms",API_SET_inhale_ms);
  rest.function("exhale_ms",API_SET_exhale_ms);
  rest.function("inhale_alarm_ms",API_SET_inhale_critical_alarm_ms);
  rest.function("exhale_alarm_ms",API_SET_exhale_critical_alarm_ms);
  rest.function("pressure_max",API_SET_pressure_forced_inhale_max);
  rest.function("pressure_min",API_SET_pressure_forced_exhale_min);
  rest.function("pressure_drop",API_SET_pressure_drop);
  rest.function("control_mode",API_SET_control_mode);
  rest.function("ap_delta_trigger",API_SET_assist_pressure_delta_trigger);
  rest.function("inhale_ms_extra",API_SET_inhale_ms_extra);
  rest.function("costant_rate",API_SET_costant_rate);

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


  MeasureFluxInit();
  Serial.println(" Measure Flow Sensor initialized!");
}

void PressureControlLoop_PRESSIN()
{

  static float pid_error=0;
  static float pid_integral=0;
  static float pid_prec=0;
  static float pid_out=0;

  float PID_P = 0.8;
  float PID_I = 1;        //0.2 over resistance 50
  float PID_D = 0 ;  
  static float Pset2 = 0;

  float Pmeas = 0;

  Pmeas = pressure[0].last_pressure;

  if (Pset == 0)
  {
    Pset2 =Pmeas ;
  }
  else
  {
    Pset2 = (Pset2*0.7 )+ (0.3 * Pset);
  }
  pid_error = Pset2-Pmeas;
  pid_integral += pid_error ;
  if ((pid_integral*PID_I) > 255 ) pid_integral = (255/PID_I);
  if ((pid_integral*PID_I) <-255 ) pid_integral = -(255/PID_I);
  
  pid_out= PID_P* pid_error + PID_I*pid_integral + PID_D*(pid_error-pid_prec);

  if (pid_out<0) pid_out=0;
    pid_out = pid_out + 50;
  if (pid_out>240) pid_out=240;

  pid_prec = pid_error;

  if (Pset==0)
    dacWrite(DAC1, 0);
  else
    dacWrite(DAC1, pid_out);

  PIDMonitor = pid_out;


}

void loop() {
  static WiFiClient client;
  static uint32_t last_loop_time;
  static uint8_t RestStateMachine =0;
  static int serverhanging = 0; // Used to monitor how long the client is hanging
  static int serverhangingrestartdelay = 500; // delay after which we discard a hanging client
  static uint32_t sensor_read_last_time =0;
  
  if (read_pressure_sensor(0)!= 0)
  {
    TriggerAlarm(UNABLE_TO_READ_SENSOR_PRESSURE);
  }

  PressureControlLoop_PRESSIN();
 
  if (millis() > sensor_read_last_time + 20)
  {


    float flux ;
    if (MeasureFlux(&flux) == false)
    {
     
    }

    gasflux[0].last_flux = flux; //gasflux[0].last_flux * 0.5 + (0.5*flux);

    //DBG_print(1,String(millis()) + "," + String(gasflux[0].last_flux) + "," + String(pressure[0].last_pressure)+ "," + String(valve1_status) + "," + String(valve2_status));
    //DBG_print(1, "PRESSURE:      " + String(pressure[0].last_pressure));
    sensor_read_last_time= millis();
    gasflux[0].read_millis = millis();
  }

    DBG_print(1,String(gasflux[0].last_flux) + "," + String(pressure[0].last_pressure)+ "," + String(PIDMonitor/2) + "," + String(valve2_status));

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
      valve1_status=0;
      //digitalWrite(VALVE_IN_PIN, LOW);
      Pset=0;
      DBG_print(3,"VALVE IN CLOSED");      
    } 
    else if (level==VALVE_OPEN)
    {
      valve1_status=100;
      //digitalWrite(VALVE_IN_PIN, HIGH);
      Pset = core_config.target_pressure;
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
        valve2_status=0;
        digitalWrite(VALVE_OUT_PIN, LOW);
        DBG_print(3,"VALVE OUT CLOSED");
      } 
      else if (level==VALVE_OPEN)
      {
        valve2_status=100;
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

int API_SET_inhale_ms_extra(String command) {
  int32_t value = command.toInt();
  if ((value<0) && (value>15000)) return -1;
  core_config.inhale_ms_extra = value;
  return 0;
}

int API_SET_assist_pressure_delta_trigger(String command) {
  float value = command.toFloat();
  if ((value<15000) && (value>15000)) return -1;
  core_config.assist_pressure_delta_trigger = value;
  return 0;
}


int API_SET_control_mode(String command) {
  int32_t value = command.toInt();
  if (value==0)
  {
    core_config.BreathMode = M_BREATH_FORCED;
  }
  else
  {
    if (value==1)
    {
      core_config.BreathMode = M_BREATH_ASSISTED;
    }
    else
      return -1;
  }
}

int API_SET_costant_rate(String command) {
  int32_t value = command.toInt();
  if (value==0)
  {
    core_config.constant_rate_mode = false;
  }
  else
  {
    core_config.constant_rate_mode = true;
  }
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
  static uint8_t temp_read=0;
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

    if (temp_read==0)
    {
      
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
      last_temp= *temp;
      temp_read=50;
    }
    else
    {
      temp_read--;
      *temp = last_temp;
    }
   
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

void MeasureFluxInit()
{
  Wire.begin();
  Wire.beginTransmission(byte(0x40)); // transmit to device with I2C mI2cAddress
  Wire.beginTransmission(byte(0x40)); // transmit to device with I2C mI2cAddress
  Wire.write(byte(0x10));      //
  Wire.write(byte(0x00));      //
  Wire.endTransmission();

}

uint8_t crc8(const uint8_t data, uint8_t crc)
{
     crc ^= data;
     for ( uint8_t i = 8; i; --i ) {
       crc = ( crc & 0x80 )
       ? (crc << 1) ^ 0x31
       : (crc << 1);
    }
  return crc;
}

float MeasureFluxRaw()
{
  Wire.requestFrom(0x40, 3); // read 3 bytes from device with address 0x40
  uint16_t a = Wire.read(); // first received byte stored here. The variable "uint16_t" can hold 2 bytes, this will be relevant later
  uint8_t b = Wire.read(); // second received byte stored here
  uint8_t crc = Wire.read(); // crc value stored here
  uint8_t mycrc = 0xFF; // initialize crc variable

  a = (a << 8) | b; // combine the two received bytes to a 16bit integer value
  // a >>= 2; // remove the two least significant bits
  //float Flow = (float)a;
  int Flow=a;
  return Flow;
}

bool MeasureFlux(float *Flow)
{
  unsigned int result = MeasureFluxRaw();
  int offset = 32000; // Offset for the sensor
  float scale = 140.0; // Scale factor for Air and N2 is 140.0, O2 is 142.8
  *Flow = ((float)result - offset) / scale;
  return true;
}
