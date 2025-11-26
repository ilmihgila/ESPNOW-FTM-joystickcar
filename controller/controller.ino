#include <WiFi.h>
#include <ESP32_NOW.h>
#include <esp_mac.h>
#include <vector>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <string.h>
#include <Wire.h>


float g_ypr_acc[6] = {0,0,0,0,0,0}; // 0,1,2 for yaw, pitch and roll. 3,4,5 for accel x,y,z.
int g_posx = 0, g_posy = 0; // Joystick x and y position
float g_distance; // Distance between joystick and car in cm


/*===========================================================================================================
                                                     ESP-NOW
===========================================================================================================*/
/*
  Currently the joystick task and esp now task were combined, there are no further plan
  to seperate these task bcz i don't want to deal with these async timing things, there were no 
  problem with this anyway
*/


TaskHandle_t EspnowTaskHandle = NULL;

int VRx = 9;
int VRy = 8;

int WIFI_CHANNEL = 1; 

class ESP_NOW_Peer_Class : public ESP_NOW_Peer {
public:
    ESP_NOW_Peer_Class(const uint8_t *mac_addr, uint8_t channel, wifi_interface_t iface, const uint8_t *lmk) : ESP_NOW_Peer(mac_addr, channel, iface, lmk) {}
    ~ESP_NOW_Peer_Class() {
        remove();
    }

    bool add_peer() {
        if (!add()) {
            log_e("Failed to register the broadcast peer");
            return false;
        }
        return true;
    }


    bool send_message(const uint8_t *data_station, size_t len) {
        if (!send(data_station, len)) {
            log_e("Failed to broadcast message");
            return false;
        }
        return true;
    }

    
    //CAN ONLY SEND UINT8_T DATA. ALWAYS CONVERT BEFORE SEND AND REVERT AFTER RECEIVE
    void onReceive(const uint8_t *data_mobil, size_t len, bool broadcast) {
        //Serial.printf("Received a message from master " MACSTR " (%s)\n", MAC2STR(addr()), broadcast ? "broadcast" : "unicast");
        Serial.printf("ESP-NOW Message: %s\n", (char *)data_mobil);
        char *token = strtok((char *)data_mobil, " "); // split each message to token into an array
        for(int i = 0; i < 6; i++){
          g_ypr_acc[i] = strtof(token, NULL);
          token = strtok(NULL, " ");
        }
    }
};

std::vector<ESP_NOW_Peer_Class> masters;

void register_new_master(const esp_now_recv_info_t *info, const uint8_t *data, int len, void *arg) {
    if (memcmp(info->des_addr, ESP_NOW.BROADCAST_ADDR, 6) == 0) {
        Serial.printf("Unknown peer " MACSTR " sent a broadcast message\n", MAC2STR(info->src_addr));
        Serial.println("Registering the peer as a master");

        ESP_NOW_Peer_Class new_master(info->src_addr, WIFI_CHANNEL, WIFI_IF_STA, NULL);

        masters.push_back(new_master);
        if (!masters.back().add_peer()) {
            Serial.println("Failed to register the new master");
        }
    } else {
        log_v("Received a unicast message from " MACSTR, MAC2STR(info->src_addr));
    }
}

void EspnowTask(void *param){
  /*
    WARN! these thread should only run after wifi setup and if not it would be boot loop'd, perhaps 
    im planning to use semaphore or any dynamic timing such that wifi setup could run first. these one 
    were prioritized bcz it could run into a big problem
  */
  vTaskDelay(1000 / portTICK_PERIOD_MS); 
  if (!ESP_NOW.begin()) {
        Serial.println("Failed to initialize ESP-NOW");
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        //ESP.restart();
    }

    ESP_NOW.onNewPeer(register_new_master, NULL);
    Serial.println("Setup complete. ESP-NOW and FTM are running.");
    Serial.println("Waiting for master broadcasts...");


    for(;;) {
        g_posx = analogRead(VRx);
        g_posy = analogRead(VRy);

        char data_station[100];
        snprintf(data_station, sizeof(data_station), "%d %d ", 
        g_posx, g_posy);

        Serial.printf("Broadcasting message: %s\n", data_station);
        if (masters.size() > 0) {
            if (!masters[0].send_message((uint8_t *)data_station, strlen(data_station))) {
                Serial.println("Failed to send joystick message");
            }
        } else {
            Serial.println("Waiting for a master to register to send joystick data...");
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
        vTaskDelay(25 / portTICK_PERIOD_MS);
    }

}




/*===========================================================================================================
                                                     FTM
===========================================================================================================*/

TaskHandle_t FtmNowTaskHandle = NULL; 
const char *WIFI_FTM_SSID = "WiFi_FTM_Responder";
const char *WIFI_FTM_PASS = "ftm_responder";
const uint8_t FTM_FRAME_COUNT = 16;
const uint16_t FTM_BURST_PERIOD = 2;
SemaphoreHandle_t ftmSemaphore;
bool ftmSuccess = true;

void onFtmReport(arduino_event_t *event) {
    const char *status_str[5] = {"SUCCESS", "UNSUPPORTED", "CONF_REJECTED", "NO_RESPONSE", "FAIL"};
    wifi_event_ftm_report_t *report = &event->event_info.wifi_ftm_report;
    ftmSuccess = report->status == FTM_STATUS_SUCCESS;
    if (ftmSuccess) {
        Serial.printf("FTM Estimate: Distance: %.2f m, Return Time: %lu n*s\n", (float)report->dist_est / 100.0, report->rtt_est);
        g_distance = (float)report->dist_est / 100.0;
        free(report->ftm_report_data);
    } else {
        Serial.print("FTM Error: ");
        Serial.println(status_str[report->status]);
    }
    xSemaphoreGive(ftmSemaphore);
}

bool getFtmReport() {
    if (!WiFi.initiateFTM(FTM_FRAME_COUNT, FTM_BURST_PERIOD)) {
        Serial.println("FTM Error: Initiate Session Failed");
        return false;
    }
    return xSemaphoreTake(ftmSemaphore, portMAX_DELAY) == pdPASS && ftmSuccess;
}

void FtmNowTask(void *param) {
    
    WiFi.mode(WIFI_STA);
    WiFi.onEvent(onFtmReport, ARDUINO_EVENT_WIFI_FTM_REPORT);

    Serial.println("Connecting to FTM Responder...");
    WiFi.begin(WIFI_FTM_SSID, WIFI_FTM_PASS);
    
    while (WiFi.status() != WL_CONNECTED) {
        vTaskDelay(500 / portTICK_PERIOD_MS);
        Serial.print(".");
    }
    Serial.println("\nWiFi Connected");

    WIFI_CHANNEL = WiFi.channel();
    Serial.println("Wi-Fi parameters:");
    Serial.println("  Mode: STA");
    Serial.println("  MAC Address: " + WiFi.macAddress());
    Serial.printf("  Channel: %d\n", WIFI_CHANNEL);

    for (;;) {
        if (!getFtmReport()) {
            Serial.println("FTM Fail.");
        }

     vTaskDelay(25 / portTICK_PERIOD_MS);
    }
}




/*==========================================================================================================
                                                     SCREEN
===========================================================================================================*/
TaskHandle_t ScreenTaskHandle = NULL;

#define SCREEN_WIDTH 128 // Display width
#define SCREEN_HEIGHT 64 // Display height
#define SDA_PIN 5
#define SCL_PIN 4
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);



void ScreenTask(void *param) {
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Change Adress to 0x3D if not work
        Serial.println(F("SSD1306 allocation failed"));
        vTaskDelete(NULL);
    }
  
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    display.println("INI PUNYA ILMIH! :)");
    display.display();
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    for(;;){
        display.clearDisplay();
        display.setCursor(0,0);
        display.println("Rotation:");
        display.print("Y: ");
        display.print(g_ypr_acc[0]);
        display.print(", P: ");
        display.print(g_ypr_acc[1]);
        display.print(", R: ");
        display.print(g_ypr_acc[2]);
        display.println(" rad/s");
    
        display.println("Accel:");
        display.print("X: ");
        display.print(g_ypr_acc[3]);
        display.print(", Y: ");
        display.print(g_ypr_acc[4]);
        display.print(", Z: ");
        display.print(g_ypr_acc[5]);
        display.println(" m/s^2");

        display.print("Pos: ");
        display.print(g_distance);
        display.print(" cm");
    
        display.display();
    }
}





/*===========================================================================================================
                                                     MAIN
===========================================================================================================*/

void setup() {
    Serial.begin(115200);
    Wire.begin(SDA_PIN, SCL_PIN);

    ftmSemaphore = xSemaphoreCreateBinary();

    xTaskCreatePinnedToCore(
        ScreenTask,
        "Screen Task",
        4096,
        NULL,
        1,
        &ScreenTaskHandle,
        1);

    xTaskCreatePinnedToCore(
        FtmNowTask,
        "FTM & ESP-NOW Task",
        4096,
        NULL,
        2,
        &FtmNowTaskHandle,
        0);

    xTaskCreatePinnedToCore(
        EspnowTask,
        "ESP-NOW Task",
        4096,
        NULL,
        2,
        &EspnowTaskHandle,
        0);



    Serial.println("Setup complete, starting FtmNowTask...");
}




/*===========================================================================================================
                                                     LOOP
===========================================================================================================*/

void loop() {
}