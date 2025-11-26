#include <WiFi.h>
#include <ESP32_NOW.h>
#include <esp_mac.h>
#include <vector>
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <string.h>


float g_ypr_acc[6] = {0,0,0,0,0,0}; // 0,1,2 for yaw, pitch and roll. 3,4,5 for accel x,y,z.
int g_posx = 0, g_posy = 0; // joystick x and y position


/*===========================================================================================================
                                                     ESP-NOW
===========================================================================================================*/

TaskHandle_t EspnowTaskHandle = NULL;
int WIFI_CHANNEL = 1;
//uint32_t msg_count = 0;

class ESP_NOW_Slave_Peer : public ESP_NOW_Peer {
public:
    ESP_NOW_Slave_Peer(const uint8_t *mac_addr, uint8_t channel, wifi_interface_t iface, const uint8_t *lmk) : ESP_NOW_Peer(mac_addr, channel, iface, lmk) {}
    ~ESP_NOW_Slave_Peer() {}

    bool add_peer() {
        if (!add()) {
            log_e("Failed to register the slave peer");
            return false;
        }
        return true;
    }

    // This onReceive WILL be called for slave messages
    void onReceive(const uint8_t *data_station, size_t len, bool broadcast) {
        //Serial.printf("Received a message from master " MACSTR " (%s)\n", MAC2STR(addr()), broadcast ? "broadcast" : "unicast");
        //Serial.printf("ESP-NOW Message: %s\n", (char *)data_station);
        char *token = strtok((char *)data_station, " ");
        g_posx = strtod(token, NULL) - 1900;
        token = strtok(NULL, " ");
        g_posy = strtod(token, NULL) - 1900;
    }
};


std::vector<ESP_NOW_Slave_Peer> slaves;

class ESP_NOW_Broadcast_Peer : public ESP_NOW_Peer {
public:
    // Constructor of the class using the broadcast address
    ESP_NOW_Broadcast_Peer(uint8_t channel, wifi_interface_t iface, const uint8_t *lmk) : ESP_NOW_Peer(ESP_NOW.BROADCAST_ADDR, channel, iface, lmk) {}

    // Destructor of the class
    ~ESP_NOW_Broadcast_Peer() {
        remove();
    }

    
    bool begin() {
        if (!ESP_NOW.begin() || !add()) { // 'add()' is safely called internally
            log_e("Failed to initialize ESP-NOW or register the broadcast peer");
            return false;
        }
        return true;
    }

    // Function to send a message to all devices within the network
    bool send_message(const uint8_t *data_mobil, size_t len) {
        if (!send(data_mobil, len)) {
            log_e("Failed to broadcast message");
            return false;
        }
        return true;
    }
};

void register_new_slave(const esp_now_recv_info_t *info, const uint8_t *data, int len, void *arg) {
    // Only register broadcast messages (or any message)
    Serial.printf("Unknown peer " MACSTR " sent a message\n", MAC2STR(info->src_addr));
    Serial.println("Registering the peer as a slave");

    ESP_NOW_Slave_Peer new_slave(info->src_addr, WIFI_CHANNEL, WIFI_IF_AP, NULL);

    slaves.push_back(new_slave);
    if (!slaves.back().add_peer()) {
        Serial.println("Failed to register the new slave");
        return;
    }
}


ESP_NOW_Broadcast_Peer *broadcast_peer = NULL;

void EspnowTask(void *param) {
    
    Serial.println("ESP-NOW Task: Broadcasting messages every n miliseconds.");
    for (;;) {
        char data_mobil[100];
        snprintf(data_mobil, sizeof(data_mobil), "%.2f %.2f %.2f %.2f %.2f %.2f", 
        g_ypr_acc[0], g_ypr_acc[1], g_ypr_acc[2], g_ypr_acc[3], g_ypr_acc[4], g_ypr_acc[5]);

        //Serial.printf("Broadcasting message: %s\n", data_mobil);
        // Check if peer is initialized before using
        if (broadcast_peer && !broadcast_peer->send_message((uint8_t *)data_mobil, sizeof(data_mobil))) {
            Serial.println("Failed to broadcast message");
        }
        
        vTaskDelay(25 / portTICK_PERIOD_MS);
    }
}




/*===========================================================================================================
                                                     FTM
===========================================================================================================*/

const char *WIFI_FTM_SSID = "WiFi_FTM_Responder";
const char *WIFI_FTM_PASS = "ftm_responder";




/*===========================================================================================================
                                                     IMU
===========================================================================================================*/

TaskHandle_t SensorTaskHandle = NULL;

int const INTERRUPT_PIN = 7; 
int const SDA_PIN = 5;
int const SCL_PIN = 4;

bool DMPReady = false;  // Set true if DMP init was successful
uint8_t MPUIntStatus;   // Holds actual interrupt status byte from MPU
uint8_t devStatus;      // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // Expected DMP packet size (default is 42 bytes)
uint8_t FIFOBuffer[64];

volatile bool MPUInterrupt = false;     // Indicates whether MPU6050 interrupt pin has gone high

void DMPDataReady() {
MPUInterrupt = true;
}

MPU6050 mpu;
Quaternion q;           // [w, x, y, z]         Quaternion container
VectorInt16 aa;         // [x, y, z]            Accel sensor measurements
VectorInt16 gy;         // [x, y, z]            Gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            Gravity-free accel sensor measurements
VectorFloat gravity;    // [x, y, z]            Gravity vector
float ypr[3];           // [yaw, pitch, roll]   Yaw/Pitch/Roll container and gravity vector

void SensorTask(void *param){
  
    // #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    //    Wire.begin(SDA_PIN, SCL_PIN);
    //    Wire.setClock(400000); // 400kHz I2C clock. Comment on this line if having compilation difficulties
    // #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    //    Fastwire::setup(400, true);
    // #endif
  
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
  
    /*Verify connection*/
    Serial.println(F("Testing MPU6050 connection..."));
    if(mpu.testConnection() == false){
        Serial.println("MPU6050 connection failed");
        vTaskDelete(NULL);
    }
    else {
        Serial.println("MPU6050 connection successful");
    }
  
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
  
    mpu.setXGyroOffset(0);
    mpu.setYGyroOffset(0);
    mpu.setZGyroOffset(0);
    mpu.setXAccelOffset(0);
    mpu.setYAccelOffset(0);
    mpu.setZAccelOffset(0);
  
    /* Making sure it worked (returns 0 if so) */
    if (devStatus == 0) {
        mpu.CalibrateAccel(6);  // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateGyro(6);
        Serial.println("These are the Active offsets: ");
        mpu.PrintActiveOffsets();
        Serial.println(F("Enabling DMP..."));   //Turning ON DMP
        mpu.setDMPEnabled(true);

        /*Enable Arduino interrupt detection*/
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), DMPDataReady, RISING);
        MPUIntStatus = mpu.getIntStatus();

        /* Set the DMP Ready flag so the main loop() function knows it is okay to use it */
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        DMPReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize(); //Get expected DMP packet size for later comparison
    } 
    else {
        Serial.print(F("DMP Initialization failed (code ")); //Print the error code
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    for(;;){
        if (!DMPReady) return; // Stop the program if DMP programming fails.
     
    /* Read a packet from FIFO */
      if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) { // Get the Latest packet

          mpu.dmpGetQuaternion(&q, FIFOBuffer);
          mpu.dmpGetGravity(&gravity, &q);
          mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
          g_ypr_acc[0] = ypr[0];
          g_ypr_acc[1] = ypr[1];
          g_ypr_acc[2] = ypr[2];
          // Serial.print("ypr\t");
          // Serial.print(ypr[0] * 180/M_PI);
          // Serial.print("\t");
          // Serial.print(ypr[1] * 180/M_PI);
          // Serial.print("\t");
          // Serial.println(ypr[2] * 180/M_PI);

          /* Display real acceleration, adjusted to remove gravity */
          mpu.dmpGetQuaternion(&q, FIFOBuffer);
          mpu.dmpGetAccel(&aa, FIFOBuffer);
          mpu.dmpGetGravity(&gravity, &q);
          mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
          g_ypr_acc[3] = aaReal.x;
          g_ypr_acc[4] = aaReal.y;
          g_ypr_acc[5] = aaReal.z;
          // Serial.print("areal\t");
          // Serial.print(aaReal.x);
          // Serial.print("\t");
          // Serial.print(aaReal.y);
          // Serial.print("\t");
          // Serial.println(aaReal.z);

        vTaskDelay(25 / portTICK_PERIOD_MS);
        }
    }
}  



/*===========================================================================================================
                                                       MOVEMENT
===========================================================================================================*/

TaskHandle_t MovementTaskHandle = NULL;
 
int m1b1 = 10;
int m1b2 = 11;
int m2b1 = 12;
int m2b2 = 13;
int enA = 8;
int enB = 9;
int drift_tresh = 100;
int posx, posy;

void MovementTask(void *param){
    pinMode(m1b1, OUTPUT);
    pinMode(m1b2, OUTPUT);
    pinMode(m2b1, OUTPUT);
    pinMode(m2b2, OUTPUT);
    pinMode(enA, OUTPUT);
    pinMode(enB, OUTPUT);


    for(;;){
        posx = g_posx;
        posy = g_posy;

        if(posx <= drift_tresh && posx >= -drift_tresh) posx = 0;
        if(posy <= drift_tresh && posy >= -drift_tresh) posy = 0;
        posx /= 8;
        posy /= 8;
        int motor_left = posy + posx;
        int motor_right = posy - posx;

        int max_pwm = 255; 
        motor_left = constrain(motor_left, -max_pwm, max_pwm);
        motor_right = constrain(motor_right, -max_pwm, max_pwm);

        analogWrite(enA, abs(motor_left));
        analogWrite(enB, abs(motor_right));
        Serial.printf("POSX: %d, POSY: %d, L: %d, R: %d\n", posx, posy, motor_left, motor_right);

        if (motor_left > 0) {
            Serial.println("LEFT FORWARD");
            digitalWrite(m1b1, HIGH);
            digitalWrite(m1b2, LOW);
        } else if (motor_left < 0) {
            Serial.println("LEFT BACKWARD");
            digitalWrite(m1b1, LOW);
            digitalWrite(m1b2, HIGH);
        } else {
            Serial.println("LEFT STOP");
            digitalWrite(m1b1, LOW);
            digitalWrite(m1b2, LOW);
        }

        if (motor_right > 0) {
            Serial.println("RIGHT FORWARD");
            digitalWrite(m2b1, HIGH);
            digitalWrite(m2b2, LOW);
        } else if (motor_right < 0) {
            Serial.println("RIGHT BACKWARD");
            digitalWrite(m2b1, LOW);
            digitalWrite(m2b2, HIGH);
        } else {
            Serial.println("RIGHT STOP");
            digitalWrite(m2b1, LOW);
            digitalWrite(m2b2, LOW);
        }
        vTaskDelay(25 / portTICK_PERIOD_MS);
    }
}

/*===========================================================================================================
                                                    MAIN
===========================================================================================================*/

void setup() {
    Serial.begin(115200);
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);

    Serial.println("Starting SoftAP with FTM Responder support");
    WiFi.softAP(WIFI_FTM_SSID, WIFI_FTM_PASS, WIFI_CHANNEL, 0, 4, true);
    
    Serial.println("Wi-Fi parameters:");
    Serial.println("  Mode: AP");
    Serial.println("  MAC Address: " + WiFi.softAPmacAddress());
    Serial.printf("  Channel: %d\n", WiFi.channel());


    broadcast_peer = new ESP_NOW_Broadcast_Peer(WIFI_CHANNEL, WIFI_IF_AP, NULL); 
    

     if (!broadcast_peer->begin()) {
         Serial.println("Failed to initialize ESP-NOW or register broadcast peer");
         delay(5000);
    //     ESP.restart();
      }

    xTaskCreatePinnedToCore(
        SensorTask,
        "Sensor Task",
        8192, // memory
        NULL,
        1, // priority
        &SensorTaskHandle,
        1); // core
    
    xTaskCreatePinnedToCore(
        EspnowTask,
        "Esp-Now Task",
        4096,
        NULL,
        2,
        &EspnowTaskHandle,
        0);

    xTaskCreatePinnedToCore(
        MovementTask,
        "Movement Task",
        4096,
        NULL,
        2,
        &MovementTaskHandle,
        1);

    ESP_NOW.onNewPeer(register_new_slave, NULL);
    Serial.println("Setup complete.");
}




/*===========================================================================================================
                                                       LOOP
===========================================================================================================*/

void loop() {
}

