// Xbox ワイヤレスコントローラ (Model 1914, Xbox Series X|S 用) を Bluetooth LE で AtomS3R につなぎ,
// 全てのボタン・スティック・トリガーの状態を USB シリアルに表示する.
//
// 接続と HID レポートの解析は XboxSeriesXControllerESP32_asukiaaa (NimBLE-Arduino を使う) に任せる.
// ESP32-S3 は Bluetooth Classic を持たないので, コントローラは BLE で接続する (Model 1914 は対応).
//
// 使い方: 書き込み後, コントローラのペアリングボタン (上面, USB 端子の近く) を Xbox ボタンが
// 速く点滅するまで長押しする. 見つけた最初の Xbox コントローラにつなぐ.
//
// 表示 (115200 bps): 状態が変わったときに 1 行, 最短 PRINT_INTERVAL_MS 間隔.
//   LX/LY/RX/RY  スティック  0-65535 (中央 32768 付近)
//   LT/RT        トリガー    0-1023
//   A B X Y LB RB LS RS View Menu Xbox Share Up Down Left Right   押下中 1, 離すと 0
//   bat          電池残量 [%]
// 未接続の間は SCAN_REPORT_INTERVAL_MS ごとに "xbox: scanning" を表示する.

#include <Arduino.h>
#include <XboxSeriesXControllerESP32_asukiaaa.hpp>

static const uint32_t PRINT_INTERVAL_MS = 50;
// 未接続の間も止まっていないことが分かるよう, この間隔で scanning を表示する.
// USB シリアルは開く前の出力を捨てるので, 起動直後の表示だけでは見逃すことがある.
static const uint32_t SCAN_REPORT_INTERVAL_MS = 3000;
// 接続に続けて失敗したら再起動して BLE をやり直す (ライブラリの例と同じ扱い).
static const uint8_t MAX_FAILED_CONNECTIONS = 3;

static XboxSeriesXControllerESP32_asukiaaa::Core xbox;  // 引数に MAC アドレスを渡すとその 1 台だけにつなぐ

// 表示する状態. 前回と比べて変化を見るため, 解析結果をここへ写す.
struct State {
  uint16_t lx, ly, rx, ry, lt, rt;
  bool a, b, x, y, lb, rb, ls, rs, view, menu, home, share, up, down, left, right;
  uint8_t battery;
};

static State read_state() {
  // 解析結果は BLE の通知のたびに NimBLE のタスクが書き換える. 写している途中で次の通知が来ると
  // 1 行の中に新旧の値が混ざることがあるが, 次の変化の表示で正しい値に戻るので表示用には許容する.
  const XboxControllerNotificationParser &n = xbox.xboxNotif;
  State s;
  s.lx = n.joyLHori;
  s.ly = n.joyLVert;
  s.rx = n.joyRHori;
  s.ry = n.joyRVert;
  s.lt = n.trigLT;
  s.rt = n.trigRT;
  s.a = n.btnA;
  s.b = n.btnB;
  s.x = n.btnX;
  s.y = n.btnY;
  s.lb = n.btnLB;
  s.rb = n.btnRB;
  s.ls = n.btnLS;
  s.rs = n.btnRS;
  s.view = n.btnSelect;  // ライブラリでは Select (左の 2 つの四角のボタン)
  s.menu = n.btnStart;   // ライブラリでは Start (右の 3 本線のボタン)
  s.home = n.btnXbox;
  s.share = n.btnShare;
  s.up = n.btnDirUp;
  s.down = n.btnDirDown;
  s.left = n.btnDirLeft;
  s.right = n.btnDirRight;
  s.battery = xbox.battery;
  return s;
}

static bool same(const State &p, const State &q) {
  return p.lx == q.lx && p.ly == q.ly && p.rx == q.rx && p.ry == q.ry && p.lt == q.lt && p.rt == q.rt &&
         p.a == q.a && p.b == q.b && p.x == q.x && p.y == q.y && p.lb == q.lb && p.rb == q.rb &&
         p.ls == q.ls && p.rs == q.rs && p.view == q.view && p.menu == q.menu && p.home == q.home &&
         p.share == q.share && p.up == q.up && p.down == q.down && p.left == q.left &&
         p.right == q.right && p.battery == q.battery;
}

static void print_state(const State &s) {
  Serial.printf(
      "LX=%5u LY=%5u RX=%5u RY=%5u LT=%4u RT=%4u | A=%d B=%d X=%d Y=%d LB=%d RB=%d LS=%d RS=%d "
      "View=%d Menu=%d Xbox=%d Share=%d | Up=%d Down=%d Left=%d Right=%d | bat=%u%%\n",
      s.lx, s.ly, s.rx, s.ry, s.lt, s.rt, s.a, s.b, s.x, s.y, s.lb, s.rb, s.ls, s.rs, s.view, s.menu,
      s.home, s.share, s.up, s.down, s.left, s.right, s.battery);
}

void setup() {
  Serial.begin(115200);
  xbox.begin();
  Serial.println("xbox: scanning (hold the pair button until the Xbox button blinks fast)");
}

void loop() {
  enum Link { SCANNING, CONNECTED, RECEIVING };
  static Link last_link = SCANNING;
  static State last_state;
  static bool printed = false;
  static uint32_t printed_at = 0;
  static uint32_t scan_reported_at = 0;

  xbox.onLoop();  // 未接続ならスキャンと接続を進める (接続中は数秒止まることがある)

  Link link = !xbox.isConnected()                  ? SCANNING
              : xbox.isWaitingForFirstNotification() ? CONNECTED
                                                     : RECEIVING;
  if (link != last_link) {
    if (link == SCANNING) {
      Serial.println("xbox: disconnected, scanning");
    } else if (link == CONNECTED) {
      Serial.printf("xbox: connected to %s, waiting for the first report\n",
                    xbox.buildDeviceAddressStr().c_str());
    } else {
      Serial.println("xbox: receiving");
    }
    last_link = link;
    printed = false;  // つながり直したら, 変化がなくても 1 度表示する
  }

  if (link == SCANNING && millis() - scan_reported_at >= SCAN_REPORT_INTERVAL_MS) {
    Serial.printf("xbox: scanning (failed connections: %u)\n", xbox.getCountFailedConnection());
    scan_reported_at = millis();
  }

  if (link == SCANNING && xbox.getCountFailedConnection() >= MAX_FAILED_CONNECTIONS) {
    Serial.println("xbox: connection failed repeatedly, restarting");
    Serial.flush();
    ESP.restart();
  }

  if (link == RECEIVING && millis() - printed_at >= PRINT_INTERVAL_MS) {
    State s = read_state();
    if (!printed || !same(s, last_state)) {
      print_state(s);
      last_state = s;
      printed = true;
      printed_at = millis();
    }
  }
  delay(1);
}
