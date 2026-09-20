/*
 * lightmeter_v01.ino  — 摄影测光表 v0.4
 * ==================================================================
 * Hardware : ESP32-C3 SuperMini
 *   OLED   : 1.3" SSD1106/SH1106 128x64, I2C 0x3C  (SDA=4 SCL=5), 400kHz
 *   BH1750 : I2C 0x23 (SDA=4 SCL=5)  —— 自动测光
 *   EC11   : A=GPIO6  B=GPIO7  PUSH=GPIO10
 *
 * 交互:
 *   长按 PUSH (开机前)  -> 开机进入主界面
 *   长按 PUSH (开机后)  -> 进入模式选择页 (P/A/AV/TV/M/OFF)
 *   模式页: 旋转选择, 按压确认; OFF = 关机
 *   主界面短按          -> 切字段 SHUTTER/APERT/ISO/FILTER (档位允许时)
 *   旋转                -> 改当前字段
 *   30 秒无操作         -> 息屏; 旋转/按键唤醒
 *
 * 档位:
 *   P  程序优先: 用户动过的项固定(高亮), 其余由程序用安全值算
 *   A  全自动  : 只能调滤镜(旋转), 短按 = 测光出参数
 *   AV 光圈优先: 只能调光圈, 快门/ISO 程序算
 *   TV 快门优先: 只能调快门, 光圈/ISO 程序算
 *   M  全手动  : 全部手动, 只显示偏差
 *
 * 安全边界 (越界 = 左上角偏差闪烁):
 *   快门 <= 1/250 (不能更快)
 *   ISO  <= 6400
 *   光圈 >= f/8   (不能更大光圈)
 * 滤镜: 所有档位可调, 程序永不自动改滤镜
 * ==================================================================
 */

#include <Wire.h>
#include <U8g2lib.h>
#include <math.h>

// ---------- pins ----------
#define PIN_SDA   4
#define PIN_SCL   5
#define PIN_ENC_A 6
#define PIN_ENC_B 7
#define PIN_SW    10
#define BH1750_ADDR 0x23

// ---------- display ----------
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
static uint8_t oledAddr = 0x3C;

static void cmd(uint8_t c) {
  Wire.beginTransmission(oledAddr);
  Wire.write(0x00);
  Wire.write(c);
  Wire.endTransmission();
}

static void panelInit() {
  cmd(0xAE);
  cmd(0xD5); cmd(0x80);
  cmd(0xA8); cmd(0x3F);
  cmd(0xD3); cmd(0x00);
  cmd(0x40);
  cmd(0x8D); cmd(0x14);
  cmd(0x20); cmd(0x02);
  cmd(0xA1);
  cmd(0xC8);
  cmd(0xDA); cmd(0x12);
  cmd(0x81); cmd(0xCF);
  cmd(0xD9); cmd(0xF1);
  cmd(0xDB); cmd(0x40);
  cmd(0xA4);
  cmd(0xA6);
  cmd(0xAF);
}

static void setContrast(uint8_t v) { cmd(0x81); cmd(v); }

static bool ack(uint8_t a) {
  Wire.beginTransmission(a);
  return Wire.endTransmission() == 0;
}

// ---------- photographic scales ----------
static const float kApertures[] = {
  1.0f, 1.1f, 1.2f, 1.4f, 1.6f, 1.8f, 2.0f, 2.2f, 2.5f, 2.8f,
  3.2f, 3.5f, 4.0f, 4.5f, 5.0f, 5.6f, 6.3f, 7.1f, 8.0f, 9.0f,
  10.0f, 11.0f, 13.0f, 14.0f, 16.0f, 18.0f, 20.0f, 22.0f
};
static const int kNApertures = sizeof(kApertures) / sizeof(kApertures[0]);

static const float kShutters[] = {
  30.0f, 15.0f, 8.0f, 4.0f, 2.0f, 1.0f,
  1.0f/2, 1.0f/4, 1.0f/8, 1.0f/15, 1.0f/30, 1.0f/60,
  1.0f/125, 1.0f/250, 1.0f/500, 1.0f/1000, 1.0f/2000, 1.0f/4000, 1.0f/8000
};
static const int kNShutters = sizeof(kShutters) / sizeof(kShutters[0]);

static const float kIsos[] = {50, 100, 200, 400, 800, 1600, 3200, 6400, 12800};
static const int kNIsos = sizeof(kIsos) / sizeof(kIsos[0]);

struct Filter { const char *name; float ev; };
static const Filter kFilters[] = {
  { "NONE", 0.0f }, { "ND2", -1.0f }, { "ND4", -2.0f }, { "ND8", -3.0f },
  { "ND16", -4.0f }, { "ND32", -5.0f }, { "CPL", -1.5f },
};
static const int kNFilters = sizeof(kFilters) / sizeof(kFilters[0]);

// 安全边界索引
#define SAFE_SHUTTER_IDX 13    // 1/250 (不能更快 = 索引不能超过它)
#define SAFE_ISO_IDX     7     // 6400  (不能更高)
#define SAFE_APERT_IDX   18    // f/8.0 (不能更大光圈 = 索引不能低于它)

// ---------- modes ----------
enum Mode { MODE_P = 0, MODE_A, MODE_AV, MODE_TV, MODE_M, MODE_OFF, MODE_COUNT };
static const char *kModeNames[] = { "P", "A", "AV", "TV", "M", "OFF" };

static Mode mode = MODE_M;
static bool selectingMode = false;    // 是否在模式选择页
static int  selIdx = 0;               // 选择页光标

// ---------- fields ----------
enum Field { F_SHUTTER = 0, F_APERTURE, F_ISO, F_FILTER, F_COUNT };

static bool powerOn = false;
static int  field = F_SHUTTER;
static int  shutterIdx  = 12;    // 1/125
static int  apertureIdx = 15;    // f/5.6
static int  isoIdx      = 1;     // ISO 100
static int  filterIdx   = 0;     // NONE
static float lux        = 100.0f;
static bool  needRedraw = false;

// 用户手动动过的项 (P 档用; 高亮表示)
static bool userSet[F_COUNT] = { false, false, false, false };

// 无解闪烁
static bool  noSolution = false;
static unsigned long blinkMs = 0;
static bool  blinkOn = true;

// 延迟重绘 (避开按键瞬间的电气抖动)
static unsigned long pendingDrawMs = 0;

// ---------- 电池计时 (v0.5) ----------
// 上电/断电重插 = 满电 4 格; 每运行 1h 掉 1 格; 4h 后空图标闪烁
static unsigned long bootMillis = 0;
static int  battLevel = 4;          // 4..0
static bool battBlinkOn = true;
static unsigned long battBlinkMs = 0;

static void updateBattery() {
  // 正式版: 每 1 小时掉 1 格, 4 小时后空电闪烁
  unsigned long usedSec = (millis() - bootMillis) / 1000UL;
  int lv = 4 - (int)(usedSec / 3600UL);
  if (lv < 0) lv = 0;
  if (lv > 4) lv = 4;
  battLevel = lv;
}

// 绘制电池图标: 左上角 x, 顶 y(baseline 附近), 4 格
// 总尺寸 18x10: 外框 15x10 + 正极凸起 3x4
static void drawBattery(int x, int y) {
  const int w = 15, h = 10;
  bool blink = (battLevel == 0);
  bool drawFill = true;
  if (blink) {
    // 绝对时间相位, 不依赖重绘频率
    drawFill = ((millis() / 400UL) & 1UL) == 0;
  }
  // 空电闪烁: 整个图标隐藏/显示
  if (!drawFill) return;
  // 外框
  u8g2.drawFrame(x, y, w, h);
  // 正极凸起
  u8g2.drawBox(x + w, y + 3, 2, 4);
  // 内部填充 (1~4 格)
  for (int i = 0; i < battLevel; i++) {
    int bx = x + 2 + i * 3;
    u8g2.drawBox(bx, y + 2, 3, h - 4);
  }
}

static void requestDraw() { needRedraw = true; pendingDrawMs = millis(); }

// ---------- encoder (中断驱动) ----------
static volatile int encState = 0;
static volatile int encDelta = 0;
static volatile unsigned long encLastIsrMs = 0;

static void IRAM_ATTR encISR() {
  int a = digitalRead(PIN_ENC_A);
  int b = digitalRead(PIN_ENC_B);
  int cur = (a << 1) | b;
  unsigned long now = millis();
  if (now - encLastIsrMs < 4) return;
  encLastIsrMs = now;
  static const int8_t tbl[16] = {
     0,  1, -1,  0,
    -1,  0,  0,  1,
     1,  0,  0, -1,
     0, -1,  1,  0
  };
  int d = tbl[(encState << 2) | cur];
  if (d != 0) encDelta += d;
  encState = cur;
}

static int readEncoderStep() {
  noInterrupts();
  int d = encDelta;
  encDelta = 0;
  interrupts();
  static int acc = 0;
  acc += d;
  int step = 0;
  if (acc >= 2)       { step = 1; acc -= 2; }
  else if (acc <= -2) { step = -1; acc += 2; }
  return step;
}

// ---------- math ----------
static float targetEV() {
  if (lux <= 0) return 0;
  return log2f(lux / 2.5f) + log2f((float)kIsos[isoIdx] / 100.0f)
         + kFilters[filterIdx].ev;
}
static float currentEV() {
  float N = kApertures[apertureIdx];
  float t = kShutters[shutterIdx];
  return log2f((N * N) / t);
}
// + = 过曝, - = 欠曝
static float deltaEV() { return targetEV() - currentEV(); }

static void fmtShutter(char *buf, size_t n, float t) {
  if (t >= 1.0f) snprintf(buf, n, "%.0fs", t);
  else snprintf(buf, n, "1/%d", (int)lroundf(1.0f / t));
}

static int nearestShutter(float t) {
  int best = 0; float bestd = 1e9f;
  for (int i = 0; i < kNShutters; i++) {
    float d = fabsf(log2f(kShutters[i]) - log2f(t));
    if (d < bestd) { bestd = d; best = i; }
  }
  return best;
}
static int nearestAperture(float N) {
  int best = 0; float bestd = 1e9f;
  for (int i = 0; i < kNApertures; i++) {
    float d = fabsf(log2f(kApertures[i]) - log2f(N));
    if (d < bestd) { bestd = d; best = i; }
  }
  return best;
}
static int nearestISO(float iso) {
  int best = 0; float bestd = 1e9f;
  for (int i = 0; i < kNIsos; i++) {
    float d = fabsf(log2f(kIsos[i]) - log2f(iso));
    if (d < bestd) { bestd = d; best = i; }
  }
  return best;
}

// 检查是否越界
static bool outOfRange() {
  if (shutterIdx < 0 || shutterIdx > SAFE_SHUTTER_IDX) return true;  // 比 1/250 更快
  if (isoIdx > SAFE_ISO_IDX) return true;                            // ISO > 6400
  if (apertureIdx < SAFE_APERT_IDX) return true;                     // 光圈 > f/8
  return false;
}

// ---------- 自动求解 ----------
// 目标: N^2/t = 2^EV_target  (EV_target 含 ISO 与滤镜)
// 已知某些项, 求未知项. 安全优先级: 快门 1/250, ISO 6400, 光圈 f/8
//
// 求解策略:
//   AV: 固定光圈 -> 先试 快门=安全(1/250), 求 ISO; 若 ISO 越界则报错
//       (更合理: 固定光圈 + ISO 用当前值, 求快门; 快门越界则调 ISO)
//   TV: 固定快门 -> 先试 ISO=当前值, 求光圈; 光圈越界则调 ISO
//   P : 固定用户动过的项, 其余用安全值求解
static void solveAuto() {
  // EV100 部分 (不含 ISO)
  float ev100 = (lux <= 0) ? 0 : log2f(lux / 2.5f) + kFilters[filterIdx].ev;

  bool fixS = userSet[F_SHUTTER];
  bool fixA = userSet[F_APERTURE];
  bool fixI = userSet[F_ISO];

  // 保证被固定的项不与档位冲突
  if (mode == MODE_AV) { fixA = true; fixS = false; fixI = false; }
  if (mode == MODE_TV) { fixS = true; fixA = false; fixI = false; }
  if (mode == MODE_A)  { fixS = false; fixA = false; fixI = false; }

  // 光圈 (N) 与 快门 (t) 决定 EV; ISO 也参与
  // N^2 / t = 2^(ev100 + log2(ISO/100))
  // => log2(N^2/t) = ev100 + log2(ISO/100)

  if (fixA && fixS) {
    // 光圈+快门固定 -> 求 ISO
    float n2t = (kApertures[apertureIdx] * kApertures[apertureIdx]) / kShutters[shutterIdx];
    float needEv = log2f(n2t);            // 需要的总 EV(含 ISO)
    float iso = 100.0f * powf(2.0f, needEv - ev100);
    isoIdx = nearestISO(iso);
  } else if (fixA && !fixS) {
    // 光圈固定 -> 先按安全快门 1/250 求 ISO; 不行再用当前 ISO 求快门
    float t = kShutters[SAFE_SHUTTER_IDX];
    float n2t = (kApertures[apertureIdx] * kApertures[apertureIdx]) / t;
    float needEv = log2f(n2t);
    float iso = 100.0f * powf(2.0f, needEv - ev100);
    int isoIdxTry = nearestISO(iso);
    if (isoIdxTry <= SAFE_ISO_IDX) {
      shutterIdx = SAFE_SHUTTER_IDX;
      isoIdx = isoIdxTry;
    } else {
      // ISO 越界, 用当前 ISO 求快门
      float ev = ev100 + log2f(kIsos[isoIdx] / 100.0f);
      float t2 = (kApertures[apertureIdx] * kApertures[apertureIdx]) / powf(2.0f, ev);
      shutterIdx = nearestShutter(t2);
    }
  } else if (!fixA && fixS) {
    // 快门固定 -> 先试当前 ISO 求光圈
    float ev = ev100 + log2f(kIsos[isoIdx] / 100.0f);
    float N = sqrtf(kShutters[shutterIdx] * powf(2.0f, ev));
    int apTry = nearestAperture(N);
    if (apTry >= SAFE_APERT_IDX) {
      apertureIdx = apTry;
    } else {
      // 光圈太大(超 f/8), 提高 ISO 补偿
      float Nsafe = kApertures[SAFE_APERT_IDX];
      float needEv = log2f((Nsafe * Nsafe) / kShutters[shutterIdx]);
      float iso = 100.0f * powf(2.0f, needEv - ev100);
      isoIdx = nearestISO(iso);
      apertureIdx = SAFE_APERT_IDX;
    }
  } else {
    // 都不固定: 快门 1/250, ISO 当前, 求光圈
    float ev = ev100 + log2f(kIsos[isoIdx] / 100.0f);
    float N = sqrtf(kShutters[SAFE_SHUTTER_IDX] * powf(2.0f, ev));
    int apTry = nearestAperture(N);
    if (apTry >= SAFE_APERT_IDX) {
      shutterIdx = SAFE_SHUTTER_IDX;
      apertureIdx = apTry;
    } else {
      float Nsafe = kApertures[SAFE_APERT_IDX];
      float needEv = log2f((Nsafe * Nsafe) / kShutters[SAFE_SHUTTER_IDX]);
      float iso = 100.0f * powf(2.0f, needEv - ev100);
      isoIdx = nearestISO(iso);
      shutterIdx = SAFE_SHUTTER_IDX;
      apertureIdx = SAFE_APERT_IDX;
    }
  }

  noSolution = outOfRange();
}

// A 档: 全自动, 一次算出最佳
static void solveAutoFull() {
  float ev100 = (lux <= 0) ? 0 : log2f(lux / 2.5f) + kFilters[filterIdx].ev;
  // 安全快门 1/250 + 优先低 ISO
  for (int i = 0; i < kNIsos; i++) {          // ISO 从低到高
    float ev = ev100 + log2f(kIsos[i] / 100.0f);
    float N = sqrtf(kShutters[SAFE_SHUTTER_IDX] * powf(2.0f, ev));
    int ap = nearestAperture(N);
    if (ap >= SAFE_APERT_IDX && ap < kNApertures) {
      isoIdx = i; apertureIdx = ap; shutterIdx = SAFE_SHUTTER_IDX;
      noSolution = false;
      return;
    }
  }
  noSolution = true;                          // 全试完都不行
}

// ---------- UI helpers ----------
static bool fieldEditable(int f) {
  if (f == F_FILTER) return true;             // 滤镜所有档位可调
  switch (mode) {
    case MODE_M:  return true;
    case MODE_P:  return true;                // P: 任意项可动
    case MODE_AV: return (f == F_APERTURE);
    case MODE_TV: return (f == F_SHUTTER);
    case MODE_A:  return false;               // A: 只能调滤镜
    default:      return false;
  }
}

// 该字段是否高亮 (用户动过 / 当前编辑)
static bool fieldHighlight(int f) {
  if (mode == MODE_P) return userSet[f];
  if (mode == MODE_M) return (f == field);
  if (f == F_FILTER)  return (f == field);    // 滤镜当前编辑时高亮
  return (f == field);
}

// ---------- 渲染 ----------
static void renderContent() {
  char s[16], a[12], i2[12], fil[12], dbuf[16], lb[16];

  // 如果无解, 闪烁 (基于时间, 不依赖重绘频率)
  bool showDelta = true;
  if (noSolution) {
    // 用绝对时间算闪烁相位, 与重绘次数无关
    blinkOn = ((millis() / 400UL) & 1UL) == 0;
    showDelta = blinkOn;
  } else {
    blinkOn = true;
  }

  const char *labels[4] = { "SHUTTER", "APERT", "ISO", "FILTER" };
  char vals[4][12];
  fmtShutter(vals[0], 12, kShutters[shutterIdx]);
  snprintf(vals[1], 12, "f/%.1f", kApertures[apertureIdx]);
  snprintf(vals[2], 12, "%d", (int)kIsos[isoIdx]);
  snprintf(vals[3], 12, "%s", kFilters[filterIdx].name);

  int ys[4] = { 24, 37, 50, 63 };

  u8g2.clearBuffer();

  // 顶栏: 电池 | 偏差 | lux | 模式
  u8g2.setFont(u8g2_font_6x12_tf);
  float d = deltaEV();
  if (fabsf(d) < 0.05f) snprintf(dbuf, sizeof(dbuf), "OK");
  else                  snprintf(dbuf, sizeof(dbuf), "%+.1f", d);

  // 电池图标 (最左, 18x10, 顶部 y=0)
  updateBattery();
  drawBattery(2, 1);

  // 偏差 (电池后, 文字基线 y=10)
  if (showDelta) u8g2.drawStr(26, 10, dbuf);
  snprintf(lb, sizeof(lb), "%.0f lx", lux);
  u8g2.drawStr(58, 10, lb);
  u8g2.drawVLine(56, 0, 11);      // 竖线1: 偏差 | lux
  // 模式字母右对齐到右边缘 (128)
  const char *mn = kModeNames[mode];
  int mlen = 0; while (mn[mlen]) mlen++;
  int mw = mlen * 6;
  int mx = 126 - mw;
  u8g2.drawVLine(mx - 4, 0, 11);  // 竖线2: lux | 模式
  u8g2.drawStr(mx, 10, mn);
  u8g2.drawHLine(2, 12, 126);

  // 四行
  for (int r = 0; r < 4; r++) {
    bool hl = (powerOn && fieldHighlight(r));
    if (hl) u8g2.drawBox(2, ys[r] - 10, 126, 12);
    u8g2.setDrawColor(hl ? 0 : 1);
    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.drawStr(4, ys[r], labels[r]);
    u8g2.drawStr(72, ys[r], vals[r]);
    u8g2.setDrawColor(1);
  }
  u8g2.sendBuffer();
}

// 模式选择页
static void renderModeSelect() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(2, 10, "-- SELECT --");
  u8g2.drawHLine(2, 12, 126);

  for (int i = 0; i < MODE_COUNT; i++) {
    int y = 22 + i * 8;
    bool sel = (i == selIdx);
    if (sel) u8g2.drawBox(2, y - 7, 126, 8);
    u8g2.setDrawColor(sel ? 0 : 1);
    u8g2.drawStr(6, y, kModeNames[i]);
    const char *desc = "";
    switch (i) {
      case MODE_P:  desc = "Program AE"; break;
      case MODE_A:  desc = "Auto"; break;
      case MODE_AV: desc = "Aperture Pri"; break;
      case MODE_TV: desc = "Shutter Pri"; break;
      case MODE_M:  desc = "Manual"; break;
      case MODE_OFF:desc = "Power Off"; break;
    }
    u8g2.drawStr(30, y, desc);
    u8g2.setDrawColor(1);
  }
  u8g2.sendBuffer();
}

static void screenOff() { cmd(0xAE); }
static void screenOn()  { panelInit(); delay(30); panelInit(); }

// ---------- button ----------
static unsigned long swDownAt = 0;
static bool swDown = false;
static bool longFired = false;
static int  swSample = 1;
static int  swRawLast = 1;
static unsigned long swChangeMs = 0;
static unsigned long lastActivity = 0;
static bool screenAsleep = false;

static void enterModeSelect() {
  selectingMode = true;
  selIdx = 0;
  renderModeSelect();
}

static void pollButton() {
  int raw = digitalRead(PIN_SW);
  unsigned long now = millis();

  if (raw != swRawLast) { swRawLast = raw; swChangeMs = now; }

  if (now - swChangeMs >= 20 && raw != swSample) {
    swSample = raw;
    lastActivity = now;

    if (swSample == 0) {                    // 按下
      swDown = true; swDownAt = now; longFired = false;
      if (screenAsleep) { screenAsleep = false; screenOn(); }
    } else {                                // 松开
      if (swDown) {
        swDown = false;
        if (!longFired && !screenAsleep) {
          if (selectingMode) {
            // 模式页: 确认
            if (selIdx == MODE_OFF) {
              powerOn = false; selectingMode = false;
              screenOff();
            } else {
              mode = (Mode)selIdx;
              selectingMode = false;
              for (int i = 0; i < F_COUNT; i++) userSet[i] = false;
              if (mode == MODE_A) solveAutoFull();
              else if (mode == MODE_P || mode == MODE_AV || mode == MODE_TV) solveAuto();
              needRedraw = true;
            }
          } else if (powerOn) {
            if (mode == MODE_A) {           // A 档: 短按 = 测光
              solveAutoFull();
              requestDraw();
            } else {                        // 其他: 切字段
              int guard = 0;
              do {
                field = (field + 1) % F_COUNT;
                guard++;
              } while (!fieldEditable(field) && guard < F_COUNT * 2);
              requestDraw();
            }
          }
        }
      }
    }
  }

  // 长按
  if (swDown && swSample == 0 && !longFired && (now - swDownAt >= 1500)) {
    longFired = true;
    if (!powerOn) {
      powerOn = true; screenAsleep = false;
      selectingMode = false;
      screenOn();
      if (mode == MODE_A) solveAutoFull();
      else if (mode == MODE_P || mode == MODE_AV || mode == MODE_TV) solveAuto();
      needRedraw = true;
    } else {
      enterModeSelect();                    // 开机后长按 -> 模式页
    }
  }
}

// ---------- encoder ----------
static void pollEncoder() {
  if (!powerOn) return;
  int step = readEncoderStep();
  if (step == 0) return;
  lastActivity = millis();
  if (screenAsleep) { screenAsleep = false; screenOn(); }

  if (selectingMode) {
    selIdx = (selIdx + step + MODE_COUNT) % MODE_COUNT;
    renderModeSelect();
    return;
  }

  // 主界面: 改当前字段
  int dir = step;
  if (field == F_SHUTTER)       shutterIdx  = constrain(shutterIdx  + dir, 0, kNShutters - 1);
  else if (field == F_APERTURE) apertureIdx = constrain(apertureIdx + dir, 0, kNApertures - 1);
  else if (field == F_ISO)      isoIdx      = constrain(isoIdx      + dir, 0, kNIsos - 1);
  else                          filterIdx   = constrain(filterIdx   + dir, 0, kNFilters - 1);

  if (mode == MODE_P) {
    if (field != F_FILTER) userSet[field] = true;
    solveAuto();
  } else if (mode == MODE_AV) {
    solveAuto();
  } else if (mode == MODE_TV) {
    solveAuto();
  } else if (mode == MODE_M) {
    noSolution = false;
  }
  needRedraw = true;
}

// ---------- BH1750 (非阻塞) ----------
static unsigned long lastLuxRead = 0;
static bool bhPresent = false;
static bool bhWaiting = false;

static void autoBrightness() {
  uint8_t c;
  if      (lux < 10)   c = 0x2F;
  else if (lux < 100)  c = 0x7F;
  else if (lux < 1000) c = 0xCF;
  else                 c = 0xFF;
  static uint8_t lastC = 0;
  if (c != lastC) { lastC = c; setContrast(c); }
}

static void pollBH1750() {
  unsigned long now = millis();
  if (bhWaiting) {
    if (now - lastLuxRead < 120) return;
    Wire.requestFrom((uint8_t)BH1750_ADDR, (uint8_t)2);
    if (Wire.available() >= 2) {
      uint16_t raw = ((uint16_t)Wire.read() << 8) | Wire.read();
      float newLux = raw / 1.2f;
      if (fabsf(newLux - lux) > (lux * 0.05f + 2.0f)) {
        lux = newLux;
        autoBrightness();
        if (mode == MODE_A) solveAutoFull();
        else if (mode == MODE_P || mode == MODE_AV || mode == MODE_TV) solveAuto();
        needRedraw = true;
      }
    }
    bhWaiting = false;
    lastLuxRead = now;
    return;
  }
  if (now - lastLuxRead < 1000) return;
  Wire.beginTransmission(BH1750_ADDR);
  Wire.write(0x10);
  if (Wire.endTransmission() == 0) { bhPresent = true; bhWaiting = true; }
  else { bhPresent = false; lastLuxRead = now; }
}

// ---------- 分页刷新 ----------
static int  drawPage = 0;
static bool drawing = false;
static bool needModeRender = false;

static void sendPage(int page) {
  uint8_t *buf = u8g2.getBufferPtr();
  if (!buf) return;
  // SH1106 可见列从第 2 列开始 (132 列显存 vs 128 列屏)
  cmd(0xB0 | page);          // page address
  cmd(0x02);                 // low col = 2  (关键!)
  cmd(0x10);                 // high col = 0
  Wire.beginTransmission(oledAddr);
  Wire.write(0x40);
  Wire.write(buf + page * 128, 128);
  Wire.endTransmission();
}

void setup() {
  Serial.begin(115200);
  delay(1200);

  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  pinMode(PIN_SW, INPUT_PULLUP);

  encState = (digitalRead(PIN_ENC_A) << 1) | digitalRead(PIN_ENC_B);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), encISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), encISR, CHANGE);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  delay(100);

  for (int i = 0; i < 50; i++) {
    if (ack(0x3C)) { oledAddr = 0x3C; break; }
    if (ack(0x3D)) { oledAddr = 0x3D; break; }
    delay(100);
  }

  u8g2.setI2CAddress(oledAddr << 1);
  u8g2.begin();
  u8g2.setBusClock(400000);
  u8g2.setContrast(0xCF);

  Wire.beginTransmission(BH1750_ADDR);
  Wire.write(0x01);
  bhPresent = (Wire.endTransmission() == 0);
  if (bhPresent) {
    Wire.beginTransmission(BH1750_ADDR);
    Wire.write(0x10);
    Wire.endTransmission();
  }

  powerOn = false;
  selectingMode = false;
  // 电池计时起点 = 上电时刻 (断电重插即重置满电)
  bootMillis = millis();
  battLevel = 4;
  screenOff();
}

void loop() {
  pollButton();
  pollEncoder();
  if (powerOn && !selectingMode) pollBH1750();

  // 电池图标定时刷新 (正式版: 1s tick)
  static unsigned long lastBattTick = 0;
  if (powerOn && !screenAsleep && !selectingMode && millis() - lastBattTick >= 1000) {
    lastBattTick = millis();
    int oldLv = battLevel;
    updateBattery();
    // 掉格 / 空电闪烁 / 无解闪烁期间 都需要重绘
    if (battLevel != oldLv || battLevel == 0 || noSolution) needRedraw = true;
  }

  // 30 秒息屏
  if (powerOn && !screenAsleep && !selectingMode && millis() - lastActivity > 30000) {
    screenAsleep = true;
    screenOff();
  }

  // 分页刷新 (主界面)
  if (!selectingMode) {
    if (drawing && !screenAsleep && powerOn) {
      // 正在发页: 连续把 8 页发完, 中途不接受新重绘
      sendPage(drawPage);
      drawPage++;
      if (drawPage >= 8) { drawing = false; drawPage = 0; }
    } else if (needRedraw && !screenAsleep && powerOn && !drawing
               && millis() - pendingDrawMs >= 20) {
      needRedraw = false;
      renderContent();
      drawing = true;
      drawPage = 0;
    }
  }
}
