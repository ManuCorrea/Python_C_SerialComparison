#define MAX_INPUT_ORDERS 20

byte inputSerial = 0;
int inputOrders[MAX_INPUT_ORDERS];

void setup() {
  // https://forum.arduino.cc/index.php?topic=21497.0
  Serial.begin(115200);
}

void loop() {
  if (Serial.available()) {
    inputSerial = Serial.read();
    if (inputSerial == 0x00) {
      // intercambio rapido una sola orden y ack
      // multiple peticiones  
      Serial.write(0xAA);
    } else if (inputSerial == 0x01) {
      // request de datos
      // 2 bytes respuesta
      Serial.write(0xF0); // mockup data
      Serial.write(0xF2);
      Serial.write(0xFF); // end sended data
    } else if (inputSerial == 0x02) {
      // multiple peticiones desde pc
      int idx = 0;
      inputSerial = 0;
      while (inputSerial != 0xFF) {
        if (Serial.available()){
          inputSerial = Serial.read();

          inputOrders[idx++] = inputSerial;
          if (idx == MAX_INPUT_ORDERS - 1) {
            // input orders overflow
            break;
          }
        }
      }
    } else {
      // x peticiones y respuestas
      // serial data number of request wanted
      for (int i = 0; i<inputSerial; i++) {
        Serial.write(0x11);
      }
      Serial.write(0xFF);
    }
  }
}
