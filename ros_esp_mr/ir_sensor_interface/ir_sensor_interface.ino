#include <ros.h>
#include <std_msgs/Int16.h>

// Sensor IR
const int sensorPin = 35;
float distance;

// Variables para el filtro de media móvil
const int numReadings = 10;
int readings[numReadings];
int readIndex = 0;
int total = 0;
int average = 0;

// Variables para el filtro de Kalman
float kalmanValue = 0;    // Estimación inicial
const float processVar = 1; // Varianza del proceso
const float sensorVar = 2;  // Varianza del sensor
float estimateErr = 1;   // Estimación del error
float kalmanGain;

ros::NodeHandle  nh;

std_msgs::Int16 ir_msg;
ros::Publisher ir_node("ir_distances", &ir_msg);


void setup()
{
  analogReadResolution(12);  
  
  for (int j = 0; j < numReadings; j++) 
  {
    readings[j] = 0;
  }

  nh.initNode();
  nh.advertise(ir_node);
}

void loop()
{
  // Actualizar el promedio móvil
  total = total - readings[readIndex];
  readings[readIndex] = analogRead(sensorPin);
  total = total + readings[readIndex];
  average = total / numReadings;
  readIndex = readIndex + 1;
  if (readIndex >= numReadings) 
  {
    readIndex = 0;
  }

  float voltage = average * (3.3 / 4095.0);

  // Aplicar el filtro de Kalman
  kalmanGain = estimateErr / (estimateErr + sensorVar);
  kalmanValue = kalmanValue + kalmanGain * (voltage - kalmanValue);
  estimateErr = (1 - kalmanGain) * estimateErr + fabs(kalmanValue - voltage) * processVar;

  distance = convertVoltageToDistance(kalmanValue);

  ir_msg.data = (int16_t)distance;
  ir_node.publish(&ir_msg);
  nh.spinOnce();
  delay(100);
}


float convertVoltageToDistance(float voltage) 
{
  return (27.728 * pow(voltage, -1.2045));
}
