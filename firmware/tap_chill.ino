/*
 * 敲敲冰 Tap & Chill —— ESP32 主控固件（补版）
 * ============================================
 * 课程项目：产品结构与制造基础 / 实验基础 · B4 组（袁宁 陈森培 黄若涵 卢筠言 潘梦飞）
 *
 * 功能链路（对应 4.26 展板《实验基础》技术说明）：
 *   场景一：用户回家 → 手机连 WiFi → 小智同步 → 红外+超声波两次靠近确认
 *           → TTS 问"来杯冰吗" → 确认后开始制冰 → 敲木鱼游戏自动启动
 *   场景二：语音"我要出冰块" → 小智云端 MCP 工具调用 start_ice_making()
 *           → command_router() → enter_ice_make_mode() → GPIO/舵机动作 → 出冰
 *
 * 依赖库（Arduino IDE 库管理器安装）：
 *   - ESP32Servo（舵机敲木鱼）
 *   - Adafruit SSD1306 + Adafruit GFX（OLED 显示）
 *
 * 说明：本版为功能补写版（原代码待找回后对照合并），逻辑可直接编译烧录验证。
 */

#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ==================== 引脚定义 ====================
// —— 传感输入 ——
const int PIN_IR_PROXIMITY = 34;   // 红外接近传感器（数字输出，靠近=LOW）
const int PIN_ULTRA_TRIG   = 25;   // 超声波 Trig
const int PIN_ULTRA_ECHO   = 26;   // 超声波 Echo
const int PIN_WATER_LEVEL  = 35;   // 水位开关（缺水=LOW）
const int PIN_ICE_FULL     = 32;   // 冰满拨杆（满=LOW）
const int PIN_NTC_TEMP     = 33;   // 蒸发器 NTC 温度（模拟）
const int PIN_TAP_SENSOR   = 27;   // 敲击/振动传感器（敲木鱼输入）
// —— 执行输出 ——
const int PIN_RELAY_COMPRESSOR = 16; // 压缩机继电器（制冷）
const int PIN_RELAY_PUMP       = 17; // 水泵继电器（上水）
const int PIN_RELAY_HOTVALVE   = 18; // 热旁通阀继电器（脱模）
const int PIN_FAN_CONDENSER    = 19; // 冷凝风扇
const int PIN_UV_LIGHT         = 21; // 水箱 UV 消毒灯
const int PIN_SERVO_TAP        = 13; // 敲木鱼舵机
const int PIN_BUZZER           = 22; // 蜂鸣器（木鱼声/提示音）
// —— OLED（I2C：SDA=21? 注意与UV冲突时改用 SDA=23/SCL=5）——
#define OLED_SDA 23
#define OLED_SCL 5
Adafruit_SSD1306 oled(128, 64, &Wire, -1);

// ==================== 制冰状态机 ====================
enum IceState {
  ICE_IDLE,        // 待机
  ICE_FILLING,     // 上水
  ICE_FREEZING,    // 制冷
  ICE_HARVEST,     // 加热脱模落冰
  ICE_DONE,        // 完成
  ICE_ERROR        // 故障（缺水等）
};
IceState iceState = ICE_IDLE;

// —— 制冰流程时序参数（毫秒，按实际机型调校）——
const unsigned long FILL_DURATION    = 90000UL;   // 上水 90s
const unsigned long FREEZE_TARGET_MS = 900000UL;  // 制冷最长 15min（温度达标提前结束）
const unsigned long HARVEST_DURATION = 120000UL;  // 脱模 120s
const int  FREEZE_TARGET_ADC  = 900;   // NTC 模拟读数阈值（≈ -8°C，需实测标定）
const unsigned long COMPRESSOR_MIN_OFF = 180000UL; // 压缩机停机保护 3min

unsigned long stateEnterMs = 0;
unsigned long compressorLastOffMs = 0;
bool iceRequested = false;

// ==================== 敲木鱼游戏 ====================
Servo tapServo;
int  meritCount = 0;              // 功德计数
bool tapGameActive = false;       // 制冰期间自动开启
unsigned long lastTapMs = 0;
const unsigned long TAP_DEBOUNCE = 180; // 敲击去抖

// ==================== 接近检测（两次确认） ====================
int  approachCount = 0;
unsigned long lastApproachMs = 0;
const unsigned long APPROACH_WINDOW = 8000;   // 两次靠近需在 8s 内
const float APPROACH_DIST_CM = 50.0;          // 超声波判定距离

// ==================== UV 定时消毒 ====================
unsigned long uvLastRunMs = 0;
const unsigned long UV_INTERVAL  = 6UL*3600UL*1000UL; // 每 6 小时
const unsigned long UV_DURATION  = 10UL*60UL*1000UL;  // 每次 10 分钟
bool uvOn = false;
unsigned long uvOnMs = 0;

// ==================== 小智 / MCP 语音指令接口 ====================
// 展板对应函数名：start_ice_making → command_router → enter_ice_make_mode
// 小智云端 MCP 工具回调时调用 start_ice_making()

void start_ice_making();   // 前置声明

// ==================== 工具函数 ====================

float readUltrasonicCm() {
  digitalWrite(PIN_ULTRA_TRIG, LOW);  delayMicroseconds(2);
  digitalWrite(PIN_ULTRA_TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(PIN_ULTRA_TRIG, LOW);
  unsigned long dur = pulseIn(PIN_ULTRA_ECHO, HIGH, 30000UL);
  return dur * 0.0343f / 2.0f;
}

bool isWaterShort() { return digitalRead(PIN_WATER_LEVEL) == LOW; }
bool isIceFull()    { return digitalRead(PIN_ICE_FULL) == LOW; }

void tts_say(const char* text) {
  // 接小智 TTS：实际项目里通过串口/云服务下发播报文本
  Serial.printf("[TTS] %s\n", text);
}

void oledShow(const String& l1, const String& l2 = "") {
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(0, 0);  oled.println(l1);
  oled.setCursor(0, 24); oled.println(l2);
  oled.display();
}

void oledShowMerit() {
  oled.clearDisplay();
  oled.setTextSize(2);
  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(10, 4); oled.println("Tap&Chill");
  oled.setCursor(10, 32);
  oled.printf("Merit +%d", meritCount);  // 功德+N
  oled.display();
}

// ==================== 制冰状态机切换 ====================

void enterState(IceState s) {
  iceState = s;
  stateEnterMs = millis();
  switch (s) {
    case ICE_IDLE:
      digitalWrite(PIN_RELAY_COMPRESSOR, LOW);
      digitalWrite(PIN_RELAY_PUMP, LOW);
      digitalWrite(PIN_RELAY_HOTVALVE, LOW);
      digitalWrite(PIN_FAN_CONDENSER, LOW);
      oledShow("Standby", "Tap or say hi");
      break;
    case ICE_FILLING:
      digitalWrite(PIN_RELAY_PUMP, HIGH);
      oledShow("Filling water...");
      break;
    case ICE_FREEZING: {
      // 压缩机重启保护：距上次停机需满 3 分钟
      unsigned long wait = COMPRESSOR_MIN_OFF - (millis() - compressorLastOffMs);
      if (compressorLastOffMs != 0 && (long)wait > 0) delay(wait);
      digitalWrite(PIN_RELAY_COMPRESSOR, HIGH);
      digitalWrite(PIN_FAN_CONDENSER, HIGH);
      oledShow("Freezing...", "Making ice");
      break;
    }
    case ICE_HARVEST:
      digitalWrite(PIN_RELAY_COMPRESSOR, LOW);   // 停压缩机
      compressorLastOffMs = millis();
      digitalWrite(PIN_RELAY_HOTVALVE, HIGH);    // 热阀脱模
      oledShow("Harvesting...");
      break;
    case ICE_DONE:
      digitalWrite(PIN_RELAY_HOTVALVE, LOW);
      digitalWrite(PIN_FAN_CONDENSER, LOW);
      tts_say("出冰了");
      oledShow("Done!", "Enjoy your ice");
      tapGameActive = false;
      break;
    case ICE_ERROR:
      digitalWrite(PIN_RELAY_COMPRESSOR, LOW);
      digitalWrite(PIN_RELAY_PUMP, LOW);
      digitalWrite(PIN_RELAY_HOTVALVE, LOW);
      digitalWrite(PIN_FAN_CONDENSER, LOW);
      tts_say("水箱缺水，请加水");
      oledShow("ERROR", "No water!");
      break;
  }
}

// ==================== 语音指令入口（小智 MCP 工具实现） ====================

void start_ice_making() {
  if (iceState == ICE_IDLE || iceState == ICE_DONE) {
    if (isWaterShort()) { enterState(ICE_ERROR); return; }
    if (isIceFull())    { tts_say("冰已满"); return; }
    iceRequested = true;
    enterState(ICE_FILLING);
    tapGameActive = true;          // 制冰开始 → 敲木鱼游戏自动启动
    meritCount = 0;
    oledShowMerit();
    tts_say("开始制冰啦，敲敲木鱼等等吧");
  }
}

void command_router(const String& intent) {
  // 小智云端意图路由：展板 command_router()
  if (intent == "make_ice" || intent == "我要出冰块") start_ice_making();
}

// ==================== setup / loop ====================

void setup() {
  Serial.begin(115200);

  pinMode(PIN_IR_PROXIMITY, INPUT);
  pinMode(PIN_ULTRA_TRIG, OUTPUT);
  pinMode(PIN_ULTRA_ECHO, INPUT);
  pinMode(PIN_WATER_LEVEL, INPUT_PULLUP);
  pinMode(PIN_ICE_FULL, INPUT_PULLUP);
  pinMode(PIN_NTC_TEMP, INPUT);
  pinMode(PIN_TAP_SENSOR, INPUT_PULLUP);
  pinMode(PIN_RELAY_COMPRESSOR, OUTPUT);
  pinMode(PIN_RELAY_PUMP, OUTPUT);
  pinMode(PIN_RELAY_HOTVALVE, OUTPUT);
  pinMode(PIN_FAN_CONDENSER, OUTPUT);
  pinMode(PIN_UV_LIGHT, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);

  tapServo.attach(PIN_SERVO_TAP);
  tapServo.write(80);   // 复位：木鱼锤抬起

  Wire.begin(OLED_SDA, OLED_SCL);
  oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  oledShow("Tap & Chill", "Booting...");

  enterState(ICE_IDLE);
  Serial.println("TapChill firmware ready.");
}

void loop() {
  unsigned long now = millis();

  // ---------- 1. 制冰状态机 ----------
  switch (iceState) {
    case ICE_FILLING:
      if (isWaterShort()) { enterState(ICE_ERROR); break; }
      if (now - stateEnterMs >= FILL_DURATION) {
        digitalWrite(PIN_RELAY_PUMP, LOW);
        enterState(ICE_FREEZING);
      }
      break;
    case ICE_FREEZING:
      if (analogRead(PIN_NTC_TEMP) <= FREEZE_TARGET_ADC ||
          now - stateEnterMs >= FREEZE_TARGET_MS) {
        enterState(ICE_HARVEST);
      }
      break;
    case ICE_HARVEST:
      if (now - stateEnterMs >= HARVEST_DURATION) enterState(ICE_DONE);
      break;
    case ICE_DONE:
      // 大量档可改为：若未冰满则自动进入下一轮
      iceRequested = false;
      break;
    default: break;
  }

  // ---------- 2. 敲木鱼（制冰等待期间的互动） ----------
  if (tapGameActive && digitalRead(PIN_TAP_SENSOR) == LOW &&
      now - lastTapMs > TAP_DEBOUNCE) {
    lastTapMs = now;
    meritCount++;
    tapServo.write(30);            // 落锤敲
    tone(PIN_BUZZER, 2000, 60);    // 木鱼声
    oledShowMerit();
    delay(80);
    tapServo.write(80);            // 抬锤复位
  }

  // ---------- 3. 两次靠近确认（场景一：回家检测） ----------
  if (iceState == ICE_IDLE) {
    bool near = (digitalRead(PIN_IR_PROXIMITY) == LOW) ||
                (readUltrasonicCm() < APPROACH_DIST_CM);
    if (near) {
      if (now - lastApproachMs > APPROACH_WINDOW) approachCount = 0; // 超时重新计
      if (lastApproachMs == 0 || now - lastApproachMs > 1500) {      // 去抖：两次靠近间隔 >1.5s
        approachCount++;
        lastApproachMs = now;
        if (approachCount >= 2) {
          approachCount = 0;
          tts_say("欢迎回家，来杯冰吗？");
          oledShow("Welcome home!", "Want some ice?");
          // 等待语音/按键确认；确认后由 command_router("make_ice") 触发
        }
      }
    }
  }

  // ---------- 4. UV 定时消毒 ----------
  if (!uvOn && now - uvLastRunMs >= UV_INTERVAL) {
    uvOn = true; uvOnMs = now;
    digitalWrite(PIN_UV_LIGHT, HIGH);
  } else if (uvOn && now - uvOnMs >= UV_DURATION) {
    uvOn = false; uvLastRunMs = now;
    digitalWrite(PIN_UV_LIGHT, LOW);
  }

  delay(20);
}
