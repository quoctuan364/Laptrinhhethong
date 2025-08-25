#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <DHT.h>

/* ===== WiFi & HiveMQ Cloud (TLS) ===== */
const char* WIFI_SSID  = "PMinh Viet";
const char* WIFI_PASS  = "Thuan@312";

const char* MQTT_HOST  = "72a1c30f840949618031af7b25ba5037.s1.eu.hivemq.cloud";
const int   MQTT_PORT  = 8883;                         // TLS
const char* MQTT_USER  = "Thuan";           // Access Management
const char* MQTT_PASSWD= "Thuan@312";

const char* DEVICE_ID  = "boardinghouse01";            // đặt tên thiết bị
char TOPIC_TELE[64], TOPIC_STATUS[64], TOPIC_PUMP_STATE[64], TOPIC_CMD_FORCE[64], TOPIC_LOG[64];

/* ====== Cảm biến DHT11 ====== */
#define DHTPIN   27
#define DHTTYPE  DHT11
DHT dht(DHTPIN, DHTTYPE);

/* ====== Cảm biến analog ====== */
#define RAIN_SENSOR_PIN   32   // Mưa (ADC1)
#define WATER_SENSOR_PIN  33   // Ngập nước (ADC1)
#define SOIL_PIN          34   // Độ ẩm nền (ADC1 - input only)

/* ====== Chấp hành ====== */
#define RELAY_PIN   25         // Relay bơm (đa số Active-LOW)
#define BUZZER_PIN  23         // Buzzer 5V

// LED RGB 4 chân (mỗi kênh R/G/B qua điện trở 220–330Ω)
#define LED_R 14
#define LED_G 26
#define LED_B 18
const bool LED_COMMON_ANODE = false;   // false: chân chung GND (common cathode)

/* ====== Ngưỡng ====== */
const int RAIN_T0 = 500,  RAIN_T1 = 1500, RAIN_T2 = 3000;   // mưa
const int WATER_T0        = 500;    // <500  => nền khô
const int WATER_HIGH_ON   = 1500;   // >=1500 => NGẬP CAO (bật bơm)
const int WATER_HIGH_OFF  = 1300;   // <=1300 => Hết ngập cao (tắt bơm)

/* ====== Calib Soil ====== */
int SOIL_DRY = 3200;   // RAW khi khô (điền theo thực tế)
int SOIL_WET = 1300;   // RAW khi ướt

/* ====== HÀM PHỤ & MÀU LED ====== */
#define LED_OFF   0
#define LED_RED   1
#define LED_BLUE  2
#define LED_GREEN 3

inline int avgAnalog(int pin, int n=5){
  long s=0; for(int i=0;i<n;i++){ s+=analogRead(pin); delay(3);} return s/n;
}
void setRGB(bool r,bool g,bool b){
  auto drive=[&](int pin,bool on){
    if(LED_COMMON_ANODE) digitalWrite(pin, on?LOW:HIGH);
    else                 digitalWrite(pin, on?HIGH:LOW);
  };
  drive(LED_R,r); drive(LED_G,g); drive(LED_B,b);
}
void setColor(uint8_t c){
  switch(c){
    case LED_RED:   setRGB(true,  false, false); break; // Đỏ
    case LED_BLUE:  setRGB(false, false, true ); break; // Xanh dương (xanh nước biển)
    case LED_GREEN: setRGB(false, true,  false); break; // Xanh lá
    default:        setRGB(false,false,false);  break;  // Tắt
  }
}

/* ====== Trạng thái & MQTT ====== */
WiFiClientSecure net;
PubSubClient mqtt(net);

static bool floodHigh = false;          // ngập cao?
static uint8_t curColor = LED_OFF;
static bool forcePumpOn = false;        // lệnh từ cloud
unsigned long lastTick = 0;

/* ============ MQTT helper ============ */
void mqttCallback(char* topic, byte* payload, unsigned int len){
  String msg; for(unsigned int i=0;i<len;i++) msg += (char)payload[i];
  if (String(topic) == String(TOPIC_CMD_FORCE)) {
    if (msg == "ON")  forcePumpOn = true;
    if (msg == "OFF" || msg == "AUTO") forcePumpOn = false;
  }
}

void connectWiFi(){
  if (WiFi.status()==WL_CONNECTED) return;
  Serial.print("WiFi: ");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  int tries=0; while (WiFi.status()!=WL_CONNECTED && tries<40){ delay(250); Serial.print("."); tries++; }
  Serial.println(WiFi.status()==WL_CONNECTED ? "OK" : "FAIL");
}

void ensureMqtt(){
  if (mqtt.connected()) return;
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(mqttCallback);

  // TLS demo nhanh: bỏ kiểm chứng CA. Làm đồ án có thể giải thích & bổ sung CA bằng net.setCACert(...)
  net.setInsecure();

  char clientId[64]; snprintf(clientId, sizeof(clientId), "esp32-%s-%lu", DEVICE_ID, millis());
  while (!mqtt.connected()){
    if (mqtt.connect(clientId, MQTT_USER, MQTT_PASSWD, TOPIC_STATUS, 0, true, "offline")) {
      mqtt.publish(TOPIC_STATUS, "online", true);
      mqtt.subscribe(TOPIC_CMD_FORCE);
    } else {
      delay(1000);
    }
  }
}

/* ==== PUBLISH: JSON + Log text ==== */
void publishTelemetry(float t, float h, int rainRaw, int waterRaw, int soilRaw, float soilPct, int floodState, const char* ledName, int pumpOn){
  // JSON ngắn gọn
  char payload[220];
  snprintf(payload, sizeof(payload),
    "{\"temp\":%.1f,\"humidity\":%.1f,\"rain_raw\":%d,\"water_raw\":%d,"
    "\"soil_raw\":%d,\"soil_pct\":%.1f,\"flood_state\":%d,\"led\":\"%s\",\"pump\":%d}",
    t, h, rainRaw, waterRaw, soilRaw, soilPct, floodState, ledName, pumpOn
  );
  mqtt.publish(TOPIC_TELE, payload, false);

  // Log chữ đẹp (giống Serial)
  String logMsg;
  logMsg.reserve(200);
  logMsg  = "🌡 Nhiet do: " + String(t,1) + "°C\n";
  logMsg += "💧 Do am: "    + String(h,1) + "%\n";
  logMsg += "🌧 Mua RAW: "  + String(rainRaw)  + "\n";
  logMsg += "💦 Nuoc RAW: " + String(waterRaw) + "\n";
  logMsg += "🌱 Dat RAW: "  + String(soilRaw)  + " | %: " + String(soilPct,1) + "\n";
  logMsg += "🚩 Trang thai: " + String(floodState==2?"Ngap cao":(floodState==1?"Ngap thap":"Kho rao")) + "\n";
  logMsg += "💡 LED: " + String(ledName) + "\n";
  logMsg += "🔌 Bom: " + String(pumpOn?"BAT":"TAT");
  mqtt.publish(TOPIC_LOG, logMsg.c_str(), false);

  // Trạng thái bơm (retained)
  mqtt.publish(TOPIC_PUMP_STATE, pumpOn ? "ON" : "OFF", true);
}
/* ===================================== */

void setup() {
  Serial.begin(115200);
  delay(200);

  // Chuẩn bị topic
  snprintf(TOPIC_TELE,       sizeof(TOPIC_TELE),       "flood/%s/telemetry",      DEVICE_ID);
  snprintf(TOPIC_STATUS,     sizeof(TOPIC_STATUS),     "flood/%s/status",         DEVICE_ID);
  snprintf(TOPIC_PUMP_STATE, sizeof(TOPIC_PUMP_STATE), "flood/%s/state/pump",     DEVICE_ID);
  snprintf(TOPIC_CMD_FORCE,  sizeof(TOPIC_CMD_FORCE),  "flood/%s/cmd/force_pump", DEVICE_ID);
  snprintf(TOPIC_LOG,        sizeof(TOPIC_LOG),        "flood/%s/log",            DEVICE_ID);

  // ADC ESP32
  analogSetWidth(12);               // 0..4095
  analogSetAttenuation(ADC_11db);   // ~0..3.3V

  dht.begin();

  pinMode(RAIN_SENSOR_PIN,  INPUT);
  pinMode(WATER_SENSOR_PIN, INPUT);
  pinMode(SOIL_PIN,         INPUT);

  pinMode(RELAY_PIN,  OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_R, OUTPUT); pinMode(LED_G, OUTPUT); pinMode(LED_B, OUTPUT);

  digitalWrite(RELAY_PIN, HIGH);   // OFF (Active-LOW)
  digitalWrite(BUZZER_PIN, LOW);
  setColor(LED_OFF);

  connectWiFi();
  ensureMqtt();
}

void loop() {
  connectWiFi();
  ensureMqtt();
  mqtt.loop();

  // chạy mỗi 2 giây
  if (millis() - lastTick < 2000) return;
  lastTick = millis();

  /* ==== Đọc DHT11 ==== */
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();
  if (!isnan(humidity) && !isnan(temperature)) {
    Serial.print("🌡 Nhiệt độ: "); Serial.print(temperature);
    Serial.print("°C | 💧 Độ ẩm: "); Serial.print(humidity); Serial.println("%");
  } else {
    Serial.println("❌ Lỗi đọc DHT11");
  }

  /* ==== Đọc cảm biến mưa, nước, soil ==== */
  int rainValue  = avgAnalog(RAIN_SENSOR_PIN);
  int waterValue = avgAnalog(WATER_SENSOR_PIN);
  int soilRaw    = avgAnalog(SOIL_PIN);
  float soilPct  = 100.0f * (SOIL_DRY - soilRaw) / (float)(SOIL_DRY - SOIL_WET);
  soilPct = constrain(soilPct, 0.0f, 100.0f);

  Serial.print("🌧 Mưa (RAW): ");   Serial.print(rainValue);
  Serial.print(" | 💦 Nước (RAW): "); Serial.print(waterValue);
  Serial.print(" | 🌱 Độ ẩm nền (RAW): "); Serial.print(soilRaw);
  Serial.print(" | %: "); Serial.println(soilPct, 1);

  if (rainValue < RAIN_T0)       Serial.println("☀️ Trời khô ráo");
  else if (rainValue < RAIN_T1)  Serial.println("💧 Ẩm nhẹ");
  else if (rainValue < RAIN_T2)  Serial.println("☁️ Mưa nhỏ");
  else                           Serial.println("⚠️ Mưa lớn");

  // Hysteresis
  if (!floodHigh && waterValue >= WATER_HIGH_ON)  floodHigh = true;
  if (floodHigh  && waterValue <= WATER_HIGH_OFF) floodHigh = false;

  int floodState = 0; // 0=khô,1=thấp,2=cao
  const char* ledName = "OFF";
  int pumpOn = 0;

  if (forcePumpOn) {
    Serial.println("📲 ÉP BƠM ON từ MQTT");
    digitalWrite(RELAY_PIN, LOW);     // ON
    digitalWrite(BUZZER_PIN, HIGH);
    setColor(LED_RED); Serial.println("🔴 Đèn: ĐỎ");
    floodState = 2; ledName = "RED"; pumpOn = 1;
  } else if (floodHigh) {
    Serial.println("🚨 Ngập cao - BẬT BƠM");
    digitalWrite(RELAY_PIN, LOW);     // ON
    digitalWrite(BUZZER_PIN, HIGH);
    setColor(LED_RED); Serial.println("🔴 Đèn: ĐỎ");
    floodState = 2; ledName = "RED"; pumpOn = 1;
  } else if (waterValue >= WATER_T0) {
    Serial.println("➡️ Ngập thấp - Chưa bơm");
    digitalWrite(RELAY_PIN, HIGH);    // OFF
    digitalWrite(BUZZER_PIN, HIGH);   // cảnh báo
    setColor(LED_BLUE); Serial.println("🔵 Đèn: XANH NƯỚC BIỂN");
    floodState = 1; ledName = "BLUE"; pumpOn = 0;
  } else {
    Serial.println("➡️ Nền khô - Tắt bơm");
    digitalWrite(RELAY_PIN, HIGH);    // OFF
    digitalWrite(BUZZER_PIN, LOW);
    setColor(LED_GREEN); Serial.println("🟢 Đèn: XANH LÁ");
    floodState = 0; ledName = "GREEN"; pumpOn = 0;
  }

  Serial.println("---------------------------");

  // Gửi lên MQTT: JSON + bản log chữ đẹp
  publishTelemetry(temperature, humidity, rainValue, waterValue, soilRaw, soilPct, floodState, ledName, pumpOn);
}
