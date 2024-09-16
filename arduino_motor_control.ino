char t = 'S';  // Initialize t to the default value

int M1 = 2; // Front Left Motor
int M1R = 3;
int M2 = 4; // Front Right Motor
int M2R = 5;
int M3 = 6; // Back Left Motor
int M3R = 7;
int M4 = 8; // Back Right Motor
int M4R = 9;

void setup() {
  pinMode(M1, OUTPUT);
  pinMode(M1R, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(M2R, OUTPUT);
  pinMode(M3, OUTPUT);
  pinMode(M3R, OUTPUT);
  pinMode(M4, OUTPUT);
  pinMode(M4R, OUTPUT);

  Serial.begin(9600);

}

void loop() {
  if (Serial.available() > 0) {
    t = Serial.read();
    Serial.println(t);
  }

  switch (t) {
    case 'F':  // Move forward
      digitalWrite(M1, HIGH);
      digitalWrite(M1R, LOW);
      digitalWrite(M2, HIGH);
      digitalWrite(M2R, LOW);
      digitalWrite(M3, HIGH);
      digitalWrite(M3R, LOW);
      digitalWrite(M4, HIGH);
      digitalWrite(M4R, LOW);
      break;
    case 'B':  // Move backward
      digitalWrite(M1, LOW);
      digitalWrite(M1R, HIGH);
      digitalWrite(M2, LOW);
      digitalWrite(M2R, HIGH);
      digitalWrite(M3, LOW);
      digitalWrite(M3R, HIGH);
      digitalWrite(M4, LOW);
      digitalWrite(M4R, HIGH);
      break;
    case 'G':  // Strafe left
      digitalWrite(M1, LOW);
      digitalWrite(M1R, HIGH);
      digitalWrite(M2, HIGH);
      digitalWrite(M2R, LOW);
      digitalWrite(M3, HIGH);
      digitalWrite(M3R, LOW);
      digitalWrite(M4, LOW);
      digitalWrite(M4R, HIGH);
      break;
    case 'H':  // Strafe right
      digitalWrite(M1, HIGH);
      digitalWrite(M1R, LOW);
      digitalWrite(M2, LOW);
      digitalWrite(M2R, HIGH);
      digitalWrite(M3, LOW);
      digitalWrite(M3R, HIGH);
      digitalWrite(M4, HIGH);
      digitalWrite(M4R, LOW);
      break;
    case 'S':  // Stop all motors
      digitalWrite(M1, LOW);
      digitalWrite(M1R, LOW);
      digitalWrite(M2, LOW);
      digitalWrite(M2R, LOW);
      digitalWrite(M3, LOW);
      digitalWrite(M3R, LOW);
      digitalWrite(M4, LOW);
      digitalWrite(M4R, LOW);
      break;
    case 'I':  // Diagonal forward-right
  digitalWrite(M2, HIGH);   
  digitalWrite(M2R, LOW);
  digitalWrite(M3, HIGH);   
  digitalWrite(M3R, LOW);
  break;
case 'J':  // Diagonal forward-left
  digitalWrite(M1, HIGH);   
  digitalWrite(M1R, LOW);
  digitalWrite(M4, HIGH);   
  digitalWrite(M4R, LOW);
  break;
case 'K':  // Diagonal backward-right
  digitalWrite(M2, LOW);    
  digitalWrite(M2R, HIGH);
  digitalWrite(M3, LOW);    
  digitalWrite(M3R, HIGH);
  break;
case 'M':  // Diagonal backward-left
  digitalWrite(M1, LOW);    
  digitalWrite(M1R, HIGH);
  digitalWrite(M4, LOW);    
  digitalWrite(M4R, HIGH);
  break;
case 'Z':  // Spin clockwise
  digitalWrite(M1, HIGH);  
  digitalWrite(M1R, LOW);
  digitalWrite(M2, HIGH);   
  digitalWrite(M2R, LOW);
  digitalWrite(M3, LOW);  
  digitalWrite(M3R, HIGH);
  digitalWrite(M4, LOW);   
  digitalWrite(M4R, HIGH);
  break;

case 'X':  // Spin counterclockwise
  digitalWrite(M1, LOW);   
  digitalWrite(M1R, HIGH);
  digitalWrite(M2, LOW);  
  digitalWrite(M2R, HIGH);
  digitalWrite(M3, HIGH);   
  digitalWrite(M3R, LOW);
  digitalWrite(M4, HIGH); 
  digitalWrite(M4R, LOW);
  break;
  }
}

