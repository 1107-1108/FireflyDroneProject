#include <XboxSeriesXControllerESP32_asukiaaa.hpp>


// Required to replace with your xbox address
// XboxSeriesXControllerESP32_asukiaaa::Core
// xboxController("44:16:22:5e:b2:d4");

XboxSeriesXControllerESP32_asukiaaa::Core xboxController("68:6c:e6:91:2b:bc");

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
  return origin / 1456.33;
}


DATA_PACKET *dat = new DATA_PACKET();

void setup() {
  Serial.begin(115200);
  Serial.println("Starting Firefly Drone Project Controlling Center");
  xboxController.begin();
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
      
      Serial.print(dat->left_horizon);
      Serial.print(',');
      Serial.print(dat->left_vertical);
      Serial.print(',');
      Serial.print(dat->right_horizon);
      Serial.print(',');
      Serial.println(dat->right_vertical);

      // Serial.println("received at " + String(receivedAt));
    }
  } else {
    Serial.println("not connected");
  }
  delay(100);
}