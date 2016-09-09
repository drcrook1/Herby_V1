#include <Servo.h>

//Distance Front Right
int FR_Trig = 13;
int FR_Echo = 12;

//Distance Front Middle
int FM_Trig = 8;
int FM_Echo = 7;

//Distance Front Left
int FL_Trig = 4;
int FL_Echo = 2;

float fr_p;
float fm_p;
float fl_p;

Servo right; 
Servo left;

//Variables for Self-Learning
float learn_rate = 0.0001;

float weights[] = 
  { 0.0018, 0.0017, 0.0019, 0.0011, 0.0013, 0.0012, 0.0011 };

float Pred_Reward(float arr_x[]){
  float total = 0.0;
  Serial.println();
  for(int i = 0; i < 7; i++)
  {
    total = total + (arr_x[i] * weights[i]);
    Serial.print("arr: ");
    Serial.print(arr_x[i]);
    Serial.print("------");
    Serial.print("weight: ");
    Serial.print(weights[i]);
    
  }
  Serial.println();
  return total;
}

void Update_Weights(float y_actual, float y_pred, float arr_x[])
{
  for(int i = 0; i < 7; i++){
    float pDelta = (y_pred - y_actual) * arr_x[i];
    weights[i] = weights[i] - (learn_rate * pDelta);
  }
}

float Dist_Read_CM(int trigPin, int EchoPin){
  // The sensor is triggered by a HIGH pulse of 
  //10 or more microseconds.
  // Give a short LOW pulse beforehand to 
  //ensure a clean HIGH pulse:
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(EchoPin, INPUT);
  float duration = pulseIn(EchoPin, HIGH);

  return (duration/2) / 29.1;
}

void fullForward(){
  left.write(180);
  right.write(0);
}

void fullLeft(){
  left.write(0);
  right.write(0);
}

void fullRight(){
  left.write(180);
  right.write(180);
}

void setup()
{
  Serial.begin(9600);
  right.attach(10);
  left.attach(9);
  pinMode(FR_Trig, OUTPUT);
  pinMode(FM_Trig, OUTPUT);
  pinMode(FL_Trig, OUTPUT);
  pinMode(FR_Echo, INPUT);
  pinMode(FM_Echo, INPUT);
  pinMode(FL_Echo, INPUT);
  float fr_p = Dist_Read_CM(FR_Trig, FR_Echo);
  delay(10);
  float fm_p = Dist_Read_CM(FM_Trig, FM_Echo);
  delay(10);
  float fl_p = Dist_Read_CM(FL_Trig, FL_Echo);
  delay(10);
  fullForward();
}

float Receive_Reward(float fr, float fl, float fm, boolean wentForward){
  float reward = 0.0;
  if(fr < 12.0 && fl < 12.0 && fm < 12.0){
    reward = reward - 3.0;
  }else if(fr < 6.0 || fl < 6.0 || fm < 6.0){
    reward = reward - 2.0;
  } 
  if(wentForward){
    reward = reward + 1.0;
  }
  else{
    reward = reward + 0.65;
  }
  return reward;
}

void loop()
{  
 
  //Scale Distances
  //Use previous distances
  float fr_s = fr_p / 10.0;
  float fm_s = fm_p / 10.0;
  float fl_s = fl_p / 10.0;
  
  //Create Current State Representation
  //using prevoius loops distances
  //Notice indices 3,4 and 5 are left, forward, right.
  //final index is the bias unit.
  float arr_x[7] = {fr_s, fm_s, fl_s, 1, 0, 0, 1};
  //what is predicted for left?
  float left_pred = Pred_Reward(arr_x);
  //re-encode state for forward
  arr_x[3] = 0; arr_x[4] = 1;
  //what is forward reward prediction?
  float for_pred = Pred_Reward(arr_x);
  //re-encode state for right
  arr_x[4] = 0; arr_x[5] = 1;
  //what is right prediction?
  float right_pred = Pred_Reward(arr_x);
  
  Serial.println("Predicted Rewards:");
  Serial.print(left_pred);
  Serial.print(":::");
  Serial.print(for_pred);
  Serial.print(":::");
  Serial.print(right_pred);
  Serial.println();
  
  //Made a decision, lets see what the current state is.
  //Read Distances
  float fr_d = Dist_Read_CM(FR_Trig, FR_Echo);
  delay(10);
  float fm_d = Dist_Read_CM(FM_Trig, FM_Echo);
  delay(10);
  float fl_d = Dist_Read_CM(FL_Trig, FL_Echo);
  delay(10);
  
  Serial.println();
  Serial.print("FR Dist: ");
  Serial.print(fr_d);
  Serial.println();
  Serial.print("FM Dist: ");
  Serial.print(fm_d);
  Serial.println();
  Serial.print("FL Dist: ");
  Serial.print(fl_d);
  Serial.println();
  
  //actual reward
  float a_reward;
  //predicted reward for action actually taken.
  float y_reward;
  if(y_reward > 1000 || y_reward < -1000)
  {
    for(int i = 0; i < 7; i++){
      weights[i] = 0.0001 * i;
    }
  }
  if(for_pred > left_pred && for_pred > right_pred)
  {
    Serial.println("Chose Forward");
    fullForward();
    y_reward = for_pred;
    a_reward = Receive_Reward(fr_d, fl_d, fm_d, true);
    arr_x[3] = 0;
    arr_x[4] = 1;
    arr_x[5] = 0;
  }
  else if(left_pred > right_pred)
  {
    Serial.println("Chose Left");
    fullLeft();
    y_reward = left_pred;
    a_reward = Receive_Reward(fr_d, fl_d, fm_d, false);
    arr_x[3] = 1;
    arr_x[4] = 0;
    arr_x[5] = 0;
  }
  else if(for_pred == right_pred && for_pred == left_pred){
    Serial.println("All Predictions Same - Chose Forward");
    fullForward();
    y_reward = for_pred;
    a_reward = Receive_Reward(fr_d, fl_d, fm_d, true);
    arr_x[3] = 0;
    arr_x[4] = 1;
    arr_x[5] = 0;
  }else{
    Serial.println("Chose Right");
    fullRight();
    y_reward = right_pred;
    a_reward = Receive_Reward(fr_d, fl_d, fm_d, false);
    arr_x[3] = 0;
    arr_x[4] = 0;
    arr_x[5] = 1;
  }
  Serial.println(".....");
  Serial.print("Actual Reward: ");
  Serial.println(a_reward);
  Serial.print("Predicted Reward: ");
  Serial.print(y_reward);
  Serial.println(".....");
  
  //update the model based on experience
  Update_Weights(a_reward, y_reward, arr_x);
  
  //set previous distances to current distances.
  fr_p = fr_d;
  fm_p = fm_d;  
  fl_p = fl_d;
  }

