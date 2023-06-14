/*
* @par Copyright (C): 2010-2019, Shenzhen Yahboom Tech
* @file         potentiometer
* @author       Jessica
* @version      V1.0
* @date         2019.8.21
* @brief        Print and read potentiometer values
* @details
* @par History  
*
*/
int potpin = A1;//Define analog interface A1
int val = 0;//Temporary variable value from sensor

/*
* Function       setup
* @author        Jessica
* @date          2019.8.21
* @brief         Initial configuration
* @param[in]     void
* @retval        void
* @par History   no
*/
void setup() 
{
  pinMode(potpin,INPUT);//set analog interface A1 pin to input mode
  Serial.begin(9600);
}

/*
* Function       loop
* @author        Jessica
* @date          2019.8.21
* @brief         main function,Print the acquired potentiometer value to the serial port, and the measured value will jump between 200 and 450
* @param[in]     void
* @retval        void
* @par History   no
*/
void loop() 
{
  val = analogRead(potpin);//Read the analog value of the potentiometer and assign it to val
  Serial.println(val);//Serial port print val variable
  delay(100);//delay 0.01s
}
