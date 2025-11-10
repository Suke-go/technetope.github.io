# ToioController 仕様書

## 1. 目的
- toio の BLE 制御を UI 層（M5 デバイス依存コード）から切り離し、責務を明確にする。
- 制御モジュールはログ・描画を行わず、取得した状態を外部へ提供するだけに限定する。
- UI 層は制御モジュールの状態を参照して画面・ログ・ボタン処理を行う。

## 2. コンポーネント構成

### 2.1 ToioController
| 役割 | BLE スキャン / 接続 / 通知処理 / 状態保持 |
| --- | --- |
| 入力 | `begin()` 呼び出し時のターゲット名フラグメント、スキャン時間 |
| 出力 | 位置情報 (`CubePose`)、バッテリー残量、各種フラグ |
| 制約 | M5 API やログ関数は呼ばない。純粋にデータ管理のみを行う。 |

#### 公開インタフェース（案）
- `InitStatus scanTargets(const std::string& target_fragment, uint32_t scan_duration_sec, ToioCore** out_target);`
  - 与えられた名前フラグメントとスキャン時間で実行し、ターゲット候補を返す。戻り値で「見つかった／見つからない／一致なし」を判定する。空文字列を渡すと最初の Core が選択される。
- `InitStatus connectAndConfigure(ToioCore* target_core);`
  - 指定された core に接続し、通知登録／初期データ取得を行う。戻り値で接続成否を判定する。成功時は `ActiveCore` が設定される。
- `void loop();`
  - `Toio` ライブラリのイベント処理を回す。
- 状態アクセサ（UI 層が必要な情報を取得するための API）
  - `bool hasActiveCore() const;`
    - 接続済みの core があるかを返す。
  - `bool hasPose() const;` / `const CubePose& pose() const;`
    - 位置情報が取得済みかどうかと、最新 `CubePose` を返す。
  - `bool poseDirty() const;` / `void clearPoseDirty();`
    - 位置情報に未処理の更新があるかを返し、処理後にクリアできる。
  - `bool hasBatteryLevel() const;` / `uint8_t batteryLevel() const;`
    - バッテリー残量が取得済みかどうかと、その値を返す。
  - `bool batteryDirty() const;` / `void clearBatteryDirty();`
    - バッテリー情報の更新有無を返し、処理後にクリアできる。
  - `RGBColor ledColor() const;`
     - 内部で記録している直近の LED 色を返す（`setLedColor()` 成功時に更新）。
- アクションAPI
  - `bool setLedColor(uint8_t r, uint8_t g, uint8_t b);`
  - `bool driveMotor(bool ldir, uint8_t lspeed, bool rdir, uint8_t rspeed);`

#### 内部で保持する主な状態
- `Toio toio_` : Toio ライブラリのインスタンス。
- `ToioCore* active_core_` : 現在接続中のコア。接続に成功したときのみ設定。
- `std::vector<ToioCore*> last_scan_results_` : 直近のスキャン結果を保存（必要に応じて破棄）。
- `CubePose pose_` / `bool has_pose_` / `bool pose_dirty_` / `uint32_t pose_updated_ms_`
- `uint8_t battery_level_` / `bool has_battery_` / `bool battery_dirty_` / `uint32_t battery_updated_ms_`
- `uint32_t scan_duration_sec_` : 直近のスキャン秒数を保持。

#### 内部ヘルパー関数（private）
- `std::vector<ToioCore*> scan(uint32_t duration_sec);`
- `ToioCore* pickTarget(const std::vector<ToioCore*>& cores, const char* fragment);`
- `InitStatus connectCore(ToioCore* core);`
- `void configureCore(ToioCore* core);`
- `void handleIdData(const ToioCoreIDData& data);`
- `void handleBatteryLevel(uint8_t level);`

#### 内部ロジック（公開 API との対応）
- `scanTargets()` 実装内:
  1. **scan()**: `Toio::scan()` を実行し、検出した `ToioCore*` の配列と件数を保持する。
  2. **pickTarget()**: 引数で渡された名前フラグメントで一致する core を選び、見つかった場合は `out_target` へ格納。見つからなければ `InitStatus::kTargetNotFound` を戻り値として返す。フラグメントが空のときは先頭を選ぶ。
- `connectAndConfigure()` 実装内:
  1. **connectCore()**: 指定 core と BLE 接続を行い、成功すれば次のステップへ、失敗すれば `InitStatus::kConnectionFailed` を返す。
  2. **configureCore()**: 接続済み core に対して ID/Battery 通知コールバックを登録し、初期データを読み込む。成功時は `InitStatus::kReady` を返す。
  3. **handleIdData() / handleBatteryLevel()**: 通知を受けるたびに `CubePose` やバッテリー値を更新し、Dirty フラグと最終更新時刻を設定する（ログ出力は行わない）。

### 2.2 UI / M5 層
- **主な責務**: M5 デバイス初期化、画面描画、ログ出力、ボタン入力などユーザーインタラクション処理。
- **処理の流れ**
  1. `setup()`  
     - M5 を初期化しヘッダを描画する。  
     - `scanTargets()` と `connectAndConfigure()` を順番に呼び、戻り値 (`InitStatus`) を見て成功可否を判定する。
  2. 初期化結果に応じて画面メッセージを表示（例: 「未検出」「接続失敗」「接続完了」など）。
  3. `loop()`  
     - `M5.update()` と `g_toio.loop()` を実行し、入力と BLE イベントを処理する。  
     - 一定間隔または Dirty フラグを見て `ShowPositionData()` を呼び、最新状態を描画する。
  4. LED 点灯やモータ操作などハード制御が必要な場合は、ToioController のアクション API（`setLedColor()` / `driveMotor()`）を呼び出す。

## 3. データフロー
1. `setup()` → M5 初期化 (`InitializeM5Hardware`)。
2. `scanTargets()` → スキャン → ターゲット選択（戻り値で結果を取得）。
3. `connectAndConfigure()` → 接続 → 通知登録（戻り値で結果を取得）。
3. 通知受信で `pose` / `batteryLevel` を更新し、Dirty フラグと最終更新時間を記録。
4. UI 層は `poseDirty()` / `batteryDirty()` を確認しつつ表示・ログを更新し、使用後に `clearXXXDirty()` でリセットする。

## 4. ログ・描画方針
- ToioController はログ出力禁止。外部へ公開しているデータのみを更新。
- UI 層が表示／ログの責任を持ち、必要なら `std::function` 等で通知先を追加できるようにする。

## 5. 今後の拡張
- 追加センサ（ボタン、モーション、ID ミス通知など）も ToioController 内でハンドリングし、必要なアクセサを追加する。
- 将来的に複数コアを扱う場合は、ToioController を拡張するか、複数インスタンスを保持するマネージャを追加して対応する。

---
上記要件を満たす形で ToioController を実装し、UI 層からは状態取得とコマンド発行のみ行う構成とする。***
