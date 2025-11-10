# GoalTracker 仕様書

## 1. 概要
GoalTracker は Toio キューブの現在姿勢 (`CubePose`) と設定されたゴール座標から、左右モーターへの速度指令を生成する比例制御モジュールです。`ToioController` が保持し、`loop()` 内で呼び出すことで自律的なゴール追従を実現します。

## 2. 依存関係
- `goal_tracker/cube_pose.h` に定義された `CubePose` 構造体  
  - `x`, `y` (mm単位)、`angle` (0..359度)、`on_mat` (マット上かどうか)
- 標準 `<cmath>` / `<algorithm>`
- ToioController の `driveMotor()`（GoalTracker 自体は BLE や M5 を直接扱わない）

## 3. 公開 API

| メソッド | 説明 |
| --- | --- |
| `void setGoal(float x, float y, float stop_distance = 20.0f)` | 目標座標と停止距離を登録し追従を開始する。単位は mm。 |
| `void clearGoal()` | ゴールを解除する（`computeCommand()` は以降 false を返す）。 |
| `bool hasGoal() const` | ゴール設定の有無を返す。 |
| `void setTuning(float vmax, float wmax, float k_r, float k_a)` | 追従のチューニングパラメータをまとめて設定する。 |
| `bool computeCommand(const CubePose& pose, bool* leftDir, uint8_t* leftSpeed, bool* rightDir, uint8_t* rightSpeed)` | 現在姿勢から左右モータ指令を計算。指令を出した場合 true、未設定や条件未満で false。 |

### パラメータの意味と初期値
- `vmax` (初期 70)：直進速度の最大値 (0..100)。大きいほど速く進む。
- `wmax` (初期 60)：旋回速度の最大値 (0..100)。大きいほど素早く回頭。
- `k_r` (初期 0.5)：距離誤差→直進速度の比例ゲイン。
- `k_a` (初期 1.2)：角度誤差→旋回速度の比例ゲイン。
- `stop_distance` (初期 20mm)：ゴールとみなす距離閾値。

推奨調整: まず `vmax/wmax` で全体スピード感を合わせ、次に `k_r/k_a` を微調整して振動を抑える。`stop_distance` は 10〜30mm を目安に環境に合わせて変更する。

## 4. 制御アルゴリズム
次のような比例制御によって左右モーターの速度を決定する:

1. `dist = hypot(goal.x - pose.x, goal.y - pose.y)`
2. `target_heading = atan2(dy, dx)` → 度数法へ変換
3. `heading_error = wrap_deg180(target_heading - pose.angle)`  
   - `wrap_deg180` は [-180, 180] 範囲に丸める
4. `v = clamp(k_r * dist, -vmax, vmax)`  
   `w = clamp(k_a * heading_error, -wmax, wmax)`
5. 差動合成: `left = clamp(v - 0.5 * w, -100, 100)`  
   `right = clamp(v + 0.5 * w, -100, 100)`
6. 符号で方向を決め、絶対値をスピード値 (0..100) に変換
7. `dist < stop_distance` の場合は左右とも 0 を返し `clearGoal()` 相当の処理を行う

`CubePose.on_mat == false` やゴール未設定の場合は指令を生成せず `false` を返す。

## 5. 状態管理
```cpp
struct GoalState {
  bool active;
  float x;
  float y;
  float stop_distance;
};
```
- `setGoal` で `active=true`、`clearGoal` またはゴール到達で `active=false`
- 複数ゴールは未対応（上書きで最新のみ保持）
- 直線単位は Toio マット基準 mm、角度は度 (0..359)

## 6. ToioController からの利用順序
1. ToioController 接続完了後に `setGoal()` (+必要なら `setTuning()`) を呼ぶ。
2. `loop()` 内で `goal_tracker.computeCommand(pose, ...)` → `driveMotor()` を呼ぶ。
3. ゴール再設定時は `clearGoal()` → `setGoal()`。
4. 外部からゴール終了を検出する場合は `hasGoal()` を監視し false になったら UI へ通知。

## 7. 制限事項
- `CubePose` が未取得 (`hasPose=false` / `on_mat=false`) の間は追従しない。
- 速度指令は -100..100 の範囲に自動クリップされる。`driveMotor` も同範囲を前提。
- ToioController とは 1:1 の利用を想定。複数キューブへの同時追従は未対応。

## 8. 今後の拡張
- 経路追従や複数ゴールを扱う派生クラスの追加。
- 外部からゲインセットを動的制御するためのイベント/コマンドインタフェース。
- 角度ゲインの PID 化、加減速制御など高度なモーション制御への拡張。
