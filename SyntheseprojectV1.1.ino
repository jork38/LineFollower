// Demo Code for SerialCommand Library
// Steven Cogswell
// May 2011

#include <EEPROM.h>
#include "SerialCommand.h"
#include "EEPROMAnything.h"

#define SerialPort Serial
SerialCommand sCmd(SerialPort);   // The demo SerialCommand object

struct param_t
{
  bool start;
  int speed = map(speed, 0, 100, 0, 255);
  unsigned long cycleTime;
  int white[6];
  int black[6];
  double kp, ki, kd;
  double outMin, outMax;
} params;


unsigned long time, prevTime, delta, calculationTime;
int channels [] = {A5, A4, A3, A2, A1, A0};
int channelswhite [6];// = {50, 32, 44, 23, 36, 41};
int channelsblack [6];// = {945, 960, 850, 912, 870, 980};
int karretje [6];// = {65, 40, 800, 880, 40, 43};
int value[6];
float normalised[6];
float interpolated;

double output;
double ITerm, lastInput;

//These are the motor-control variables
int pwmA = 13;
int brakeA = 9;
int dirA = 12;

int pwmB = 11;
int brakeB = 8;
int dirB = 13;

void setup()
  { 
    pinMode(pwmA,OUTPUT);
    pinMode(dirA, OUTPUT);
    pinMode(brakeA, OUTPUT);
    pinMode(pwmB, OUTPUT);
    pinMode(dirB, OUTPUT);
    pinMode(brakeB, OUTPUT);
    
    digitalWrite(brakeA, LOW);
    digitalWrite(brakeB, LOW);
    
    attachInterrupt(0, onInterrupt, RISING);
      
    SerialPort.begin(115200); 
    sCmd.addCommand("start", onStart);
    sCmd.addCommand("stop", onStop);
    sCmd.addCommand("set", onSet);
    sCmd.addCommand("debug", onDebug);
    sCmd.addCommand("reset", onReset);
    sCmd.addCommand("calibrate", onCalibrate);
    sCmd.setDefaultHandler(onUnknownCommand);
  
    EEPROM_readAnything(0, params);
    params.start = false;
    
    time = micros();
    prevTime = time;
    
    SerialPort.println("ready");
  }

void loop()
  {  
    sCmd.readSerial();     // We don't do much, just process serial commands
    
    time = micros();
    
    if (time > prevTime) delta = time - prevTime;
    else delta = 4294967295 - prevTime + time + 1;
      
    if (delta > params.cycleTime)
    {
      prevTime = time;
        
      // do your cyclic stuff here...

      //read sensors

     for (int i = 0; i < 6; i++) value[i] = analogRead(channels[i]);
     //for (int i = 0; i < 6; i++) value[i] = karretje[i];

     //normaliseren sensors
     for (int i = 0; i < 6; i++)
     {
      normalised[i] = value[i] - params.black[i];
      normalised[i] /= params.white[i] - params.black[i];
      normalised[i] *=100;
     }

     //Interpoleren
     int pos = 0;
     float minimum = normalised[0];
     for (int i = 1; i < 6; i++)
     {
      if (normalised[i] < minimum)
      {
        pos = i;
        minimum = normalised[i];
      }
     }

     if ((pos > 0) && pos < 6)
      {
        float ymin1 = normalised[pos-1];
        float yzero = normalised[pos];
        float yplus1 = normalised[pos+1];

        float a = (yplus1 + ymin1) / 2 - yzero;
        float b = (yplus1 - ymin1) / 2;
        float c = yzero;

        float bottom = -b / (2 *a);

        interpolated = bottom * 50;
        interpolated += (-125 + pos *50); 
      }

      //Compute all the working error variables
      double error = - interpolated;
      
      ITerm += (params.ki * error);
      
      if(ITerm > params.outMax) output = params.outMax;
      
      else if (ITerm < params.outMin) output = params.outMin; 
      
      double dInput = (interpolated - lastInput);

      //Compute PID Output
      
      output = params.kp * error * + ITerm - params.kd * dInput;
      
      if(output > params.outMax) output = params.outMax;
      else if (output < params.outMin) output = params.outMin; 
      
      //Remember some variables for next time
      lastInput = interpolated;
      
      if (params.start == true)
      {
        if (output == 0)
        {
          digitalWrite(dirA, true);
          digitalWrite(dirB, false);
          analogWrite(pwmA, params.speed);
          analogWrite(pwmB, params.speed);
        }
        else if (output >= 0)
        {
          output = map(output, 0, params.outMax, 0, params.speed);
        }
        else if (output <= 0)
        {
          output = map(output, 0, params.outMin, 0, params.speed);
        }
      }
      
    }
    
    unsigned long difference = micros() - time;
    if (difference > calculationTime) calculationTime = difference;
  
  }


void onUnknownCommand(char *command)
  {
    SerialPort.print("unknown command: \"");
    SerialPort.print(command);
    SerialPort.println("\"");
  }

void onSet()
  {
    char* parameter = sCmd.next();
    char* value = sCmd.next();
    
    if (strcmp(parameter, "speed") == 0)
    {
      int x = atoi(value);
      if (x > 100) SerialPort.println("Value for speed is to high...");
      else params.speed = x; 
    }
    else if (strcmp(parameter, "kp") == 0) params.kp = atof(value);
    else if (strcmp(parameter, "ki") == 0)
    {
      params.ki = atof(value);
      params.ki *= params.cycleTime;
      params.ki /= 1000000;
    }
    else if (strcmp(parameter, "kd") == 0) 
    {
      params.kd = atof(value);
      params.kd /= params.cycleTime;
      params.kd *= 1000000;
    }
    else if (strcmp(parameter, "cycle") == 0) 
    {
      double ratio = 1 / params.cycleTime;
      params.cycleTime = atol(value);
      ratio *= params.cycleTime;

      params.ki *= ratio;
      params.kd /= ratio;
    }
    else if (strcmp(parameter, "min") == 0)
    {
      params.outMin = atoi(value);
    }
    else if (strcmp(parameter, "max") == 0)
    {
      params.outMax = atoi(value);
    }
    EEPROM_writeAnything(0, params);
  }

void onStart()
  {
    params.start = true;
    ITerm = output;
  }
  
  void onStop()
    {
      params.start = false;
    }

void onDebug()
  {
    //parameters
    SerialPort.println(" ");
    SerialPort.print("speed: ");
    SerialPort.println(params.speed);
    SerialPort.println("kp: ");
    SerialPort.print(params.kp);
    SerialPort.print(" ");
    SerialPort.print("ki: ");
    SerialPort.print(params.ki);
    SerialPort.print(" ");
    SerialPort.print("kd: ");
    SerialPort.print(params.kd);
    SerialPort.print(" ");
    SerialPort.print("Output: ");
    SerialPort.print(output);
    SerialPort.print(" ");
    SerialPort.print("outMin: ");
    SerialPort.print(params.outMin);
    SerialPort.print(" ");
    SerialPort.print("outMax: ");
    SerialPort.print(params.outMax);
    SerialPort.println();
  
    //cycle times
    SerialPort.print("cycle time: ");
    SerialPort.println(params.cycleTime);
    SerialPort.print("calculation time: ");
    SerialPort.println(calculationTime);
    
    //SerialPort.println();
    calculationTime = 0;  //reset calculation time
  
    //running
    SerialPort.print("running: ");
    SerialPort.println(params.start); 

    //Calibrate white
    SerialPort.print("Sensor calibrate white values: ");
    
    for (int j = 0; j <6; j++) 
    {
      SerialPort.print(params.white[j]);
      SerialPort.print(" ");
    }
    SerialPort.println(" ");

     //Calibrate black
    SerialPort.print("Sensor calibrate black values: ");
    
    for (int j = 0; j <6; j++) 
    {
      SerialPort.print(params.black[j]);
      SerialPort.print(" ");
    }
    SerialPort.println(" ");

    //Normalise
    SerialPort.print("Normalised values: ");

   for (int j = 0; j <6; j++) 
    {
      SerialPort.print(normalised[j]);
      SerialPort.print(" ");
    }
    SerialPort.println(" ");

    //Interpolate
    SerialPort.print("Interpolated value: ");
    SerialPort.print(interpolated);
    SerialPort.println(" ");

    //Output
    SerialPort.print("Output: ");
    SerialPort.print(output);
    SerialPort.println(" ");
}

void onReset()

  {
    SerialPort.println("resetting parameters... ");
    EEPROM_resetAnything(0, params);
    EEPROM_readAnything(0, params);
    interpolated = 0;
    SerialPort.println("done");
    
    
  }

void onCalibrate()
{
  char* arg1 = sCmd.next();
 
  if (strcmp(arg1, "white") ==0)
  {
    SerialPort.println("Calibrating sensors white...");
    for (int j = 0; j < 6; j++) params.white[j] =0;

    for (int i=0; i < 100; i++)
    {
      for (int j=0; j < 6; j++)
      {
        int value = analogRead(channels[j]);
        //int value = channelswhite[j];
        if(value > params.white[j]) params.white[j] = value;
        
      }
    }
    SerialPort.println("Done");
    
  }
  
 else if (strcmp(arg1, "black")==0)
 {
  SerialPort.println("Calibrating sensors black...");
  for (int j = 0; j < 6; j++) params.black[j] = 1023;
    
    for (int i=0; i < 100; i++)
    {
      for (int j=0; j < 6; j++)
      {
        int value = analogRead(channels[j]);
        //int value = channelsblack[j];
        if(value < params.black[j]) params.black[j] = value;
      }
    }
    SerialPort.println("Done");
 }
  EEPROM_writeAnything(0, params);  
}


void onInterrupt()
  {
    params.start = not params.start;
  }

