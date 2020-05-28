int pulsador = 7;
int rele = 52;
int val=0;
const int pot=A4;
int pmww=9;
int pmw=0;
int pot1=0;


void setup() {
  
  pinMode(rele, OUTPUT);
  pinMode(pmww, OUTPUT);
  
  pinMode(pulsador, INPUT);
  Serial.begin (9600);
}

void loop() {
  pot1=analogRead(pot);
  pmw=map(pot1,0,1023,0,250);
  val= digitalRead(pulsador);
  if (val == HIGH) {
    //  Serial.println("Encendido");
      digitalWrite(rele,val);
      analogWrite(pmww,0);
  }
  else {
      //Serial.println("Apagado");
      digitalWrite(rele,val);
      analogWrite(pmww,pmw);

//      Serial.println("Voltaje %d",pmww);
      
  }
  Serial.println(pmw);
  //delay(5);
}
