# これからの実装計画

## CLI 構成リファクタリング

1. **config_loader モジュールの新設**  
   - `include/toio/cli/config_loader.hpp` と `src/cli/config_loader.cpp` を追加。  
   - ここに `Options`, `FleetPlan`, `parse_options`, `build_fleet_plan`, `scalar_or_throw`, `random_led_color` など CLI 初期化に関するヘルパーを集約する。  
   - `CMakeLists.txt` の `toio_cli` ターゲットへ新ソースを登録。

2. **main.cpp の責務削減**  
   - 新ヘッダーを `#include` し、`main()` は「オプション取得 → FleetPlan 構築 → FleetManager 起動 → REPL」の流れだけを担う。  
   - 旧ヘルパー（`FleetGuard`, `print_usage`, `tokenize`, `to_int`, `print_status`, `print_received`, `ActiveCube` 等）のうち CLI 全体で共有したいものを新モジュールへ移動、または別ファイルへ分割して可読性を上げる。  
   - 不要になった `#include` やユーティリティ定義を削除し、`main.cpp` の行数を大幅に減らす。

3. **GoalController の導入**  
   - ゴール追従ロジックを `include/toio/control/goal_controller.hpp` / `src/control/goal_controller.cpp` に集約し、CLI 以外からも再利用できる形にする。  
   - `GoalController` は `start_goal`, `stop_goal`, `stop_all`, `has_goal` といった API を公開し、内部で非同期タスクの生成・停止やログ出力、RAII によるクリーンアップを担う。  
   - `main.cpp` の REPL コマンドはこのコントローラを呼び出すだけにし、バックグラウンドスレッドや状態の詳細を意識しなくてよいようにする。

4. **ドキュメントと動作確認**  
   - 新しいファイル構成を `docs/coding_guidelines.md` または `New_develop.md` で共有し、CLI 層の責務が明確になるよう説明を追加。  
   - `cmake --build build` → `./build/toio_cli --fleet-config configs/minimal.yaml` で動作確認し、引数処理や YAML 読み込みが変わらず機能することを保証する。
