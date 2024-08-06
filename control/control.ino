#include <XboxSeriesXControllerESP32_asukiaaa.hpp>

// Required to replace with your xbox address
// XboxSeriesXControllerESP32_asukiaaa::Core
// xboxController("44:16:22:5e:b2:d4");

XboxSeriesXControllerESP32_asukiaaa::Core xboxController;



enum control_signal {
  PITCH,
  YAW,
  ROLL
};

struct DATA_PACKET {
  int time_frame;
  int left_horizon;
  int left_vertical;
  int right_horizon;
  int right_vertical;
};

float control_2_angle(int origin) {
  return (origin / 728.167) - 45;
}



String comdata = ""; 
long previousMillis = millis();     //上一次激活时间
long interval = 100; 

DATA_PACKET *dat = new DATA_PACKET();






void setup() {
  Serial.begin(9600);
  Serial2.begin(9600,SERIAL_8N1,35, 32);
  Serial.println("Starting Firefly Drone Project Controlling Center");
  xboxController.begin();
}

void send_data(const String& message) {
  Serial2.print(message);
}

void Receive_Data() {
  char endChar = '\n';
  String comdata = "";

  while (Serial2.available() > 0) {
    char incomingChar = char(Serial2.read());

    if (incomingChar == endChar) { 
      break;
    }

    comdata += incomingChar;
    delay(2);
  }
  comdata.trim();
  if (comdata.length() > 0) {
    Serial.print("Received comdata: ");
    Serial.println(comdata);
  }

}


void loop() {

  xboxController.onLoop();
  if (xboxController.isConnected()) {
    if (xboxController.isWaitingForFirstNotification()) {
      Serial.println("waiting for first notification");
    } else {
      // Serial.println("Address: " + xboxController.buildDeviceAddressStr());
      // Serial.print(xboxController.xboxNotif.toString());
      unsigned long receivedAt = xboxController.getReceiveNotificationAt();
      uint16_t joystickMax = XboxControllerNotificationParser::maxJoy;
      dat->left_horizon = control_2_angle(xboxController.xboxNotif.joyLHori);
      dat->left_vertical = control_2_angle(xboxController.xboxNotif.joyLVert);
      dat->right_horizon = control_2_angle(xboxController.xboxNotif.joyRHori);
      dat->right_vertical = control_2_angle(xboxController.xboxNotif.joyRVert);
      
      /*
      Serial.print(dat->left_horizon);
      Serial.print(',');
      Serial.print(dat->left_vertical);
      Serial.print(',');
      Serial.print(dat->right_horizon);
      Serial.print(',');
      Serial.println(dat->right_vertical);
      */

      // Serial.println("received at " + String(receivedAt));
    }
  } else {
    Serial.println("not connected");
    if (xboxController.getCountFailedConnection() > 2) {
      ESP.restart();
    }
  }

  
  if (Serial2.available()) {
    Receive_Data();
  }

  if (millis() - previousMillis > interval) {
    String message = "Joystick: ";
    message += String(dat->left_horizon) + ",";
    message += String(dat->left_vertical) + ",";
    message += String(dat->right_horizon) + ",";
    message += String(dat->right_vertical);

    send_data(message);
    previousMillis = millis();
  }

  /*
  // Read from Serial (for testing input)
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    Serial.print("Sending: ");
    Serial.println(input);
    send_data(input);
  }
  */
  
  delay(30);
}