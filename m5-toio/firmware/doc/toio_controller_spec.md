# ToioController 仕様書

## 1. 概要
ToioController は M5StickC (UI/表示層) と Toio Core Cube の BLE 制御ロジックを分離するためのコントローラです。M5 側では `scanTargets → connectAndConfigure → loop` のシンプルな呼び出しを行い、位置/バッテリー情報、ゴール追従ロジック、LED/モーター制御などは ToioController が内部で処理します。

## 2. 依存関係
- **Toio ライブラリ** (`Toio.h`)：スキャン、接続、通知購読の基盤。
- **CubePose** (`goal_tracker/cube_pose.h`)：キューブ位置・角度を保持する構造体。
- **GoalTracker** (`goal_tracker/goal_tracker.{h,cpp}`)：目標地点追従アルゴリズム。
- **M5Unified**：UI 層のみで使用。ToioController から直接参照しない。

## 3. 公開 API

| メソッド | 説明 |
| --- | --- |
| `InitStatus scanTargets(const std::string& fragment, uint32_t durationSec, ToioCore** out_target)` | 指定フラグメントで BLE スキャン。結果に応じ `kReady` / `kNoCubeFound` / `kTargetNotFound` を返す。`out_target` が nullptr の場合は結果だけ返す。 |
| `InitStatus connectAndConfigure(ToioCore* target)` | `scanTargets` で得た Core に接続して通知登録を行う。接続失敗で `kConnectionFailed`。 |
| `void loop()` | `Toio::loop()` と内部ハンドラ処理 (`updateGoalTracking`) を回す。UI 層は `loop()` 内で `M5.update()` の後に呼ぶ。 |
| `bool hasActiveCore() const` | 接続済みか判定。未接続時は UI 層でリトライ処理などに使用する。 |
| `bool hasPose() const / const CubePose& pose() const` | 位置情報の有無と最新値。 |
| `bool poseDirty() const / void clearPoseDirty()` | 新しいポーズが届いたかを判定し、UI 更新後にクリア。 |
| `bool hasBatteryLevel() const / uint8_t batteryLevel() const` | バッテリーの有無と電圧値。 |
| `bool batteryDirty() const / void clearBatteryDirty()` | バッテリー更新の有無。 |
| `ToioLedColor ledColor() const` | 直近に設定された LED カラー。 |
| `bool setLedColor(uint8_t r, uint8_t g, uint8_t b)` | BLE 経由で LED を設定。成功時に内部状態も更新。 |
| `bool driveMotor(bool ldir, uint8_t lspeed, bool rdir, uint8_t rspeed)` | Toio の左右モーターを直接制御。GoalTracker からも内部的に利用。 |
| `void setGoal(float x, float y, float stop_distance = 20.0f)` | 目標地点を登録し、GoalTracker が追従を開始する。 |
| `void clearGoal()` | 目標追従を停止してモーターを停止する。 |
| `void setGoalTuning(float vmax, float wmax, float k_r, float k_a)` | GoalTracker のチューニングパラメータを変更する。 |

### GoalTracker パラメータ
- `vmax`：直進速度の上限。初期値 70。
- `wmax`：旋回速度の上限。初期値 60。
- `k_r`：距離ゲイン。初期値 0.5。
- `k_a`：角度ゲイン。初期値 1.2。
推奨調整手順：まず `vmax/wmax` でスピード感を合わせ、次に `k_r/k_a` で追従レスポンスを微調整する。

## 4. 内部構造
```
ToioController
 ├─ Toio toio_
 ├─ ToioCore* active_core_
 ├─ CubePose pose_
 ├─ GoalTracker goal_tracker_
 ├─ バッテリー状態/Dirtyフラグ
 ├─ LED状態
 └─ スキャン条件(直近秒数)
```

### 主なプライベート関数
- `scan(durationSec)`: `Toio::scan` 呼び出し結果を保存。
- `pickTarget(cores, fragment)`: 名前一致判定。
- `connectCore(core)`: BLE 接続。失敗時は `kConnectionFailed`。
- `configureCore(core)`: ID/Battery 通知登録と初期値取得。
- `handleIdData(data) / handleBatteryLevel(level)`: 状態更新＋Dirtyフラグ設定。
- `updateGoalTracking()`: `goal_tracker_.computeCommand()` を呼び、指令があれば `driveMotor()` を実行。ゴール到達時は `clearGoal()` 相当の処理を行う。

## 5. データフロー
1. UI 層が `scanTargets(fragment, duration, &core)` → `connectAndConfigure(core)` を呼ぶ。
2. 接続完了後、`loop()` で `toio.loop()` と `updateGoalTracking()` を継続的に実行。
3. 通知を受けるたびに `handleIdData`/`handleBatteryLevel` が `pose`/`battery` を更新し、Dirty フラグをセット。
4. UI 層は `poseDirty`/`batteryDirty` を確認して `ShowPositionData` やログ更新を行い、使用後に `clearXXXDirty()` を呼ぶ。
5. GoalTracker が有効な場合、`loop()` 毎に現在地とゴールを比較し、`driveMotor` で自律移動を実施。停止距離内に入ると `clearGoal()` が呼ばれる。

## 6. UI 層との連携
```
setup():
  InitializeM5Hardware()
  scanTargets(fragment, duration)
  connectAndConfigure(target)
  setGoal(...) / setGoalTuning(...)
loop():
  M5.update()
  g_toio.loop()
  if poseDirty → 表示更新
  if batteryDirty → 表示更新
```
- エラー発生時 (`InitStatus != kReady`) はメッセージを出してリトライ/中断を判断する。
- 目標変更や通信更新が来た場合は、`clearGoal()` → `setGoal()` を順に呼び、次回 `loop()` から新しいゴールに追従。

## 7. エラーハンドリング / 制約
- 同時に扱える Toio Core は1台のみ。
- `scanTargets` と `connectAndConfigure` は順番に呼ぶ必要がある。
- `driveMotor` は 0〜100 の範囲を前提。範囲外は内部でクリップされる。
- GoalTracker は `pose.on_mat == true` のときのみ動作。マット外になると停止したまま保持される。

## 8. 今後の拡張ポイント
- ボタン/モーション/ID missed など追加通知を扱う場合は、状態構造体とアクセサを追加して同様に Dirty 管理を行う。
- 複数ゴールや経路追従を実装する場合は `GoalTracker` を差し替え/派生クラスで拡張可能。
- BLE 再接続やフェイルオーバーが必要な場合、`scanTargets` → `connectAndConfigure` をループ内で再実行する仕組みを UI 層に追加する。
