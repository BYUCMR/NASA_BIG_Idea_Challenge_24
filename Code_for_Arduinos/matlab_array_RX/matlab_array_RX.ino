
const int MAX_ARRAY_SIZE = 10; // Maximum size of the integer array

int receivedArray[MAX_ARRAY_SIZE]; // Array to store received integers
int arraySize = 0; // Size of the received array

void setup() {
  Serial.begin(9600); // Initialize serial communication
}

void loop() {
  if (Serial.available()) {
    // Read the incoming byte
    int incomingByte = Serial.read();

    // Check if the received byte is the start of the array
    if (incomingByte == '[') {
      arraySize = 0; // Reset the array size
    }
    // Check if the received byte is the end of the array
    else if (incomingByte == ']') {
      // Process the received array
      processArray();
    }
    // Check if the received byte is a digit
    else if (isdigit(incomingByte)) {
      // Convert the received byte to an integer and add it to the array
      receivedArray[arraySize] = incomingByte - '0';
      arraySize++;
    }
  }
}

void processArray() {
  // Print the received array
  Serial.print("Received Array: ");
  for (int i = 0; i < arraySize; i++) {
    Serial.print(receivedArray[i]);
    Serial.print(" ");
  }
  Serial.println();
}
