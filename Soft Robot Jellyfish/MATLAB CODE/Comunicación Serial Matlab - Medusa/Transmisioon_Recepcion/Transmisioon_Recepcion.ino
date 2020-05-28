//**********************************************************************************
// configuración de bits para recepcion de datos 7 caracteres
//**********************************************************************************

long int p1;          //xxxxxx1

long int p2;          //xxxxx1x

long int p3;          //xxxx1xx

long int p4;          //xxx1xxx

long int central;     //xx1xxxx

long int num_pulsos;  //x1xxxxx

long int pwm;         //1xxxxxx // [1-5]trabajará en 5 rangos para evitar la saturación de envio de datos

long int num=1701010;
// se envia una cadena de 8 numeros con la descripción de la parte superior, 
// para de este modo ejecutar las ordenes de control desde la tarjeta

//**********************************************************************************
// configuración de pines
//**********************************************************************************
int sensorPin = A1;    // select the input pin for the potentiometer
int ledPin = 4;      // select the pin for the LED
float sensorValue=0;
float data_conv1,data_conv2;
int volt=0;
int portNum;
int Pot=0;
int data_rec1=0;
int cont=0;
int pwm_envio;
int i=0;
int bit_parada=0;

void setup() {
  // declare the ledPin as an OUTPUT:
  Serial.begin(9600);         // Iniciar puerto serie
  pinMode(ledPin, OUTPUT);
}

void loop() {
// envio valor del sensor de Ph  
  Pot = analogRead(sensorPin);
//  Serial.println(Pot);

}


void serialEvent() {
  if (Serial.available()> 0)
{
  bit_parada=0;
    num=Serial.parseInt();  //Converts the string received into an integer                                 //it is also converted into a double by implicit conversion
    //data_rec=Serial.read();
    //data_rec1=data_rec.toInt();
    //Voltage value sent from Matlab through the serial port
    //delay(10); 
  pwm=num/1000000;
  
  num_pulsos=(num-pwm*1000000)/100000; 
  
  central=(num-pwm*1000000-num_pulsos*100000)/10000;

  p4=(num-pwm*1000000-num_pulsos*100000-central*10000)/1000;
  
  p3=(num-pwm*1000000-num_pulsos*100000-central*10000-p4*1000)/100;
  
  p2=(num-pwm*1000000-num_pulsos*100000-central*10000-p4*1000-p3*100)/10;
  
  p1=(num-pwm*1000000-num_pulsos*100000-central*10000-p4*1000-p3*100-p2*10)/1;

  if(pwm==1)
  {
    pwm_envio=25;
  }
  else if (pwm==2)
  {
    pwm_envio=75;
  }
  else if (pwm==3)
  {
    pwm_envio=125;
  }
  else if (pwm==4)
  {
    pwm_envio=175;
  }
  else if (pwm==5)
  {
    pwm_envio=240;
  }
  else if (pwm==0)
  {
    pwm_envio=0;
  }
       
for (i=1;i<=num_pulsos;i++)
{
  if (p1==1)
  {
  analogWrite(5,pwm_envio);  //Sets the output port with the desired voltage value  
  delay(200);
  analogWrite(5,0);  //Sets the output port with the desired voltage value
  delay(300);
  }

  if (p2==1)
  {
  analogWrite(6,pwm_envio);  //Sets the output port with the desired voltage value  
  delay(200);
  analogWrite(6,0);  //Sets the output port with the desired voltage value
  delay(300);
  }
  if (p3==1)
  {
  analogWrite(7,pwm_envio);  //Sets the output port with the desired voltage value  
  delay(200);
  analogWrite(7,0);  //Sets the output port with the desired voltage value
  delay(300);
  }

  if (p4==1)
  {
  analogWrite(8,pwm_envio);  //Sets the output port with the desired voltage value  
  delay(200);
  analogWrite(8,0);  //Sets the output port with the desired voltage value
  delay(300);
  }
   if (central==1)
  {
  analogWrite(9,pwm_envio);  //Sets the output port with the desired voltage value  
  delay(200);
  analogWrite(9,0);  //Sets the output port with the desired voltage value
  delay(300);
  }
}
bit_parada=1;
Serial.println(bit_parada);

}


}
