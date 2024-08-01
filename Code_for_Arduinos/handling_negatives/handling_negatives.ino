const int MAX_ARRAY_SIZE = 100;
int receivedArray[MAX_ARRAY_SIZE];
int modifiedArray[MAX_ARRAY_SIZE];
int arraySize = 0;

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(100);
}

void loop() {
  if (Serial.available()) {
    // Read the incoming byte
    String incomingStr = Serial.readStringUntil('/n'); // Read until newline character
    int incomingInt = incomingStr.toInt(); // Convert the string to an integer
    Serial.print(incomingInt + 1); // Print the modified number
    } 
  }
