#include <DHT.h>

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
const bool LED_COMMON_ANODE = false;   // false: chân chung nối +3.3V ;

/* ====== Ngưỡng ====== */
const int RAIN_T0 = 500,  RAIN_T1 = 1500, RAIN_T2 = 3000;   // mưa

// Nước với hysteresis
const int WATER_T0        = 500;    // <500  => nền khô
const int WATER_HIGH_ON   = 1500;   // >=1500 => NGẬP CAO (bật bơm)
const int WATER_HIGH_OFF  = 1300;   // <=1300 => Hết ngập cao (tắt bơm)

/* ====== Calib Soil (điền theo thực tế ) ====== */
int SOIL_DRY = 3200;   // RAW khi khô
int SOIL_WET = 1300;   // RAW khi ướt

/* ====== HÀM PHỤ & MÀU LED (dùng mã số để tránh lỗi auto-prototype) ====== */
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

void setup() {
  Serial.begin(115200);
  delay(200);

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

  // Tắt ban đầu
  digitalWrite(RELAY_PIN, HIGH);   // Active LOW → HIGH = Tắt bơm
  digitalWrite(BUZZER_PIN, LOW);
  setColor(LED_OFF);

  Serial.println("ESP32 - DHT11, Rain, Water + Soil + Relay (Arduino IDE)");
  Serial.println("Lưu ý: cấp 3.3V cho module analog để AO <= 3.3V; nếu dùng 5V, phải chia áp.");
}

void loop() {
  /* ==== Đọc DHT11 ==== */
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();

  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("❌ Lỗi đọc DHT11");
  } else {
    Serial.print("🌡 Nhiệt độ: "); Serial.print(temperature);
    Serial.print("°C | 💧 Độ ẩm: "); Serial.print(humidity); Serial.println("%");
  }

  /* ==== Đọc cảm biến mưa, nước, soil (0..4095) ==== */
  int rainValue  = avgAnalog(RAIN_SENSOR_PIN);
  int waterValue = avgAnalog(WATER_SENSOR_PIN);
  int soilRaw    = avgAnalog(SOIL_PIN);
  float soilPct  = 100.0f * (SOIL_DRY - soilRaw) / (float)(SOIL_DRY - SOIL_WET);
  soilPct = constrain(soilPct, 0.0f, 100.0f);

  Serial.print("🌧 Mưa (RAW): ");   Serial.print(rainValue);
  Serial.print(" | 💦 Nước (RAW): "); Serial.print(waterValue);
  Serial.print(" | 🌱 Độ ẩm nền (RAW): "); Serial.print(soilRaw);
  Serial.print(" | %: "); Serial.println(soilPct, 1);

  /* ==== Phân loại mưa (giữ đúng câu chữ của anh) ==== */
  if (rainValue < RAIN_T0)       Serial.println("☀️ Trời khô ráo");
  else if (rainValue < RAIN_T1)  Serial.println("💧 Ẩm nhẹ");
  else if (rainValue < RAIN_T2)  Serial.println("☁️ Mưa nhỏ");
  else                           Serial.println("⚠️ Mưa lớn");

  /* ==== Phân loại ngập nước + hysteresis & điều khiển ==== */
  static bool floodHigh = false;           // trạng thái ngập cao
  static uint8_t curColor = LED_OFF;

  // Cập nhật trạng thái theo hysteresis
  if (!floodHigh && waterValue >= WATER_HIGH_ON)  floodHigh = true;
  if (floodHigh  && waterValue <= WATER_HIGH_OFF) floodHigh = false;

  if (floodHigh) {
    Serial.println("🚨 Ngập cao - BẬT BƠM");
    digitalWrite(RELAY_PIN, LOW);     // ON (Active-LOW)
    digitalWrite(BUZZER_PIN, HIGH);
    setColor(LED_RED);
    if (curColor != LED_RED) { Serial.println("🔴 Đèn: ĐỎ"); curColor = LED_RED; }
  } else if (waterValue >= WATER_T0) {
    Serial.println("➡️ Ngập thấp - Chưa bơm");
    digitalWrite(RELAY_PIN, HIGH);    // OFF
    digitalWrite(BUZZER_PIN, HIGH);   // cảnh báo
    setColor(LED_BLUE);               // Xanh nước biển
    if (curColor != LED_BLUE) { Serial.println("🔵 Đèn: XANH NƯỚC BIỂN"); curColor = LED_BLUE; }
  } else {
    Serial.println("➡️ Nền khô - Tắt bơm");
    digitalWrite(RELAY_PIN, HIGH);    // OFF
    digitalWrite(BUZZER_PIN, LOW);
    setColor(LED_GREEN);              // Xanh lá
    if (curColor != LED_GREEN) { Serial.println("🟢 Đèn: XANH LÁ"); curColor = LED_GREEN; }
  }

  Serial.println("---------------------------");
  delay(2000);
}
