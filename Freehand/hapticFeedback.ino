char incomingByte; // data from serial port
#define pin 12
int flag=0;
void setup() 
{
    Serial.begin (9600);
    pinMode(pin, OUTPUT);
}
void loop() {
    if (Serial.available() > 0) 
    {
        incomingByte = Serial.read();
        Serial.println(incomingByte);
        if(incomingByte=='w' && !flag)
        {
            digitalWrite(pin, HIGH);
            flag = 1;	
        }
        else if(incomingByte=='r' && flag)
        {
            digitalWrite(pin, LOW);
            flag = 0;
        }
    }
}
