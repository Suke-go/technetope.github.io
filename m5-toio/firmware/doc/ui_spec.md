# UI レイヤ仕様書

## 1. 目的
UI レイヤは M5StickC 上で toio の状態を可視化する最小限の責務のみを担い、スキャンや接続、制御ロジックは `ToioController` 側に委譲する。本仕様は UI が提供すべき API・データ構造・実装方針を定義し、main.cpp の表示処理を切り離す指針を示す。

## 2. 責務と非責務
- **表示とログ出力**: ディスプレイおよび `M5.Log` を通じて姿勢、バッテリー、LED、モータ指令、システム状態を描画する。
- **UI 状態のローカル管理**: 表示の更新タイミング（最終描画時刻、Dirty フラグ処理）を UI 側で保持し、main からは値を渡すだけにする。
- **データ取得の禁止**: UI は `ToioController` のインスタンスや BLE API に直接アクセスしない。すべての入力は main が取得した値を引数で受け取る。

## 3. ファイル構成
| ファイル | 役割 |
| --- | --- |
| `src/main.cpp` | M5 初期化、ToioController の接続フロー、UI へのデータ引き渡し。 |
| `src/ui/ui_helpers.h` | UI API の宣言。`UiStatus` 構造体と `UiHelpers` クラスを定義する。 |
| `src/ui/ui_helpers.cpp` | 描画ロジックとログ出力の実装。`M5.Display` の扱いはここに集約する。 |

## 4. データモデル
UI で扱う値は `UiHelpers` 内部に保持する `UiStatus` 構造体へ集約する。main 側は個別値を引数で渡すだけでよく、構造体生成や状態保持は不要とする。

```cpp
struct UiStatus {
  CubePose pose;
  bool has_pose = false;
  uint8_t battery_level = 0;
  bool has_battery = false;
  ToioLedColor led;
  ToioMotorState motor;
};
```

> 備考: `UiStatus` を公開 API として返さないことで、UI 側の表示フォーマットや補助状態（例: 直前の表示内容）を自由に差し替えられる。

## 5. 公開 API
`UiHelpers` クラスは以下のメソッドを提供する。メソッドは状態を受け取り、内部的に `M5.Display` と `M5.Log` へ出力する。

| メソッド | 説明 |
| --- | --- |
| `void Begin();` | ディスプレイ初期化とフォント設定、`last_display_ms_` 初期化を行う。`setup()` の M5 初期化直後に呼び出す。 |
| `void DrawHeader(const char* message);` | 共通ヘッダ（タイトル + 任意メッセージ）を描画する。接続状態や進捗表示に使用。 |
| `void ShowInitResult(ToioController::InitStatus status);` | `InitStatus` に応じた文言を決定し `DrawHeader` + ログ出力を行う。 |
| `void UpdateStatus(const CubePose& pose, bool has_pose, uint8_t battery_level, bool has_battery, const ToioLedColor& led, const ToioMotorState& motor, bool pose_dirty, bool battery_dirty, uint32_t refresh_interval_ms);` | main から取得した最新値を内部 `UiStatus` へ保存し、Dirty フラグまたは経過時間を条件に描画を更新する。 |

### 5.1 UpdateStatus の詳細
1. 渡された値を `UiStatus` にコピーし、`pose_dirty || battery_dirty || millis() - last_display_ms_ >= refresh_interval_ms` を満たした場合のみ描画する。
2. 描画内容:
   - 現在時刻（`millis()`）の表示とログ出力。
   - `has_pose` に応じて位置または「No position」を表示。
   - `has_battery` が true のときのみバッテリー残量を表示。
   - LED RGB 値と左右モータ指令（符号付き速度値）を表示。
3. 描画後に `last_display_ms_` を更新する。

Dirty フラグのクリア自体は main 側が責任を持って行う。`UpdateStatus` 呼び出し後に `clearPoseDirty()` / `clearBatteryDirty()` を実施する。

## 6. main.cpp からの利用手順
1. `setup()`  
   - `M5.begin()` の後で `UiHelpers ui; ui.Begin();` を呼び出す。  
   - スキャン／接続処理の結果を `ShowInitResult()` へ渡し、結果のみを UI に通知する。  
2. `loop()`  
   - `M5.update()` → `g_toio.loop()` の順に実行。  
   - `UiHelpers::UpdateStatus(g_toio.pose(), g_toio.hasPose(), g_toio.batteryLevel(), g_toio.hasBatteryLevel(), g_toio.ledColor(), g_toio.motorState(), pose_dirty, battery_dirty, kRefreshIntervalMs);` のように各値を直接渡す。  
   - `UpdateStatus()` の呼び出し後に `clearPoseDirty()` / `clearBatteryDirty()` を実行する。  
   - UI から制御コマンドを発行しないことを保証する。

これにより main.cpp はフロー制御とデータ収集に集中し、表示コードは `ui_helpers` へ隔離される。

## 7. 今後の拡張
- **イベント表示の追加**: 走行モードやゴール達成などのステータスメッセージを `DrawHeader` とは別に表示するサブ領域を導入する。
- **設定 UI**: ボタン入力を監視してターゲット名やゴール座標を切り替えられる簡易メニューを追加する場合も、実際の値反映は main → controller で行い、UI 層は表示のみ担当する。
- **テスト容易性**: `UiHelpers` を M5 依存コードと抽象化したインタフェースで分離し、デスクトップビルドでも描画ロジックをテストできるようにする。
