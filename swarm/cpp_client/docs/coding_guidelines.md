# cpp_client Coding Guidelines

`cpp_client` は CLI / Middleware / Transport の 3 層で構成される C++20 プロジェクトです。ここでは既存コードの設計意図とスタイルを明文化し、今後の実装でぶれないようガイドラインをまとめます。

## 1. 基本方針
- CLI から Transport までを厳密に分離し、`docs/transport.md` / `docs/middleware.md` / `docs/cli.md` で説明されている責務を崩さない。
- 共有ライブラリ (`toio_lib`) の API と振る舞いは `include/toio/**` に定義し、CLI や将来の UI から再利用できるように保つ。
- C++ らしい RAII と例外伝播を基本とし、状態クリーンアップはデストラクタまたは専用ガード (`FleetGuard`) で行う。

## 2. 言語・ビルド設定
- 標準は C++20 (`CMakeLists.txt` で `CMAKE_CXX_STANDARD 20` / `CMAKE_CXX_EXTENSIONS OFF`)。GCC/Clang 双方で通る記述を意識する。
- 依存は Boost.Asio/Beast（`system`/`thread` ヘッダオンリー指定）、`nlohmann_json`、`yaml-cpp`。他ライブラリを追加する際は CMake での取得方法とライセンスを明示する。
- 識別子や API が外部に露出する場合はヘッダー最小化を意識し、実装ファイル側でのみ重い依存を `#include` する。

## 3. ディレクトリと層の責務
- `include/toio/transport/`：WebSocket 接続や JSON 送受信など最下層の I/O を担当 (`ToioClient`)。
- `include/toio/middleware/`：`ServerSession` / `FleetManager` といったオーケストレーション層。状態 (`CubeState`) と API をここで定義。
- `src/transport/` と `src/middleware/` はそれぞれ上記ヘッダーの実装。CLI 固有のロジックは `src/main.cpp` もしくは将来的に `src/cli/**` へ分離する。
- `docs/` のアーキテクチャ解説を最新状態に保ち、構造を変える際は必ず文書を更新する。

## 4. ヘッダー／実装スタイル
- すべてのヘッダーは `#pragma once` を使用。標準ヘッダー → サードパーティ → プロジェクトヘッダーの順で並べる。
- 名前空間は `namespace toio::middleware { ... }` のように入れ子構文で宣言し、末尾で閉じる。
- ヘッダーでは所有権や責務が明確になる最小限のメンバーだけ公開し、細かい補助関数は `namespace {}` で実装ファイル側に閉じ込める。
- 実装ファイルでは `std::` などの using-directive は避け、`namespace {}` 内で限定的に `namespace beast = boost::beast;` のようなエイリアスを定義する。

## 5. 命名・フォーマット
- 型（class/struct/enum）は `FleetManager`, `CubeState` のように PascalCase。メンバー関数と free 関数は `send_move`, `load_fleet_config` など snake_case。
- メンバー変数は末尾アンダースコア (`host_`, `sessions_`) を付け、`std::atomic`/`std::mutex` のような同期プリミティブも同様に扱う。
- 可視域が限定される型エイリアスは `using Json = nlohmann::json;` 形式で局所化。`typedef` は使用しない。
- ブロックインデントは 2 スペース。長い引数リストは 1 引数ずつ縦に揃え、ラムダ引数の整列も同じスタイルに従う。
- `auto` は推論が明確な場合のみ使用し、戻り値型がインターフェースの一部になる関数は明示的な型を記述する。

## 6. エラーハンドリングとロギング
- 入力検証は早期に行い、失敗時は `std::runtime_error` / `std::out_of_range` など具体的な例外を投げる（例: `scalar_or_throw`、`get_state`）。
- `try/catch (...)` はデストラクタやガードのみに限定し、通常フローでは例外を握り潰さない。
- `ToioClient::log` のように `LogHandler` コールバックを経由してログを出力する。CLI 固有の UI 書式を Transport 層へ持ち込まない。

## 7. 状態管理とスレッドセーフティ
- Transport 層は `std::atomic<bool>` で接続状態を追跡し、送信は `std::mutex` で直列化する。
- Middleware 層は `std::shared_mutex` により読み取り多数・書き込み少数の `CubeState` アクセスを保護。`std::shared_lock` / `std::unique_lock` を明示的に使い分ける。
- 状態更新時は `update_state` のような 1 箇所の関数に集約し、コールバック通知と `last_update` タイムスタンプを忘れずに設定する。
- 非同期処理は Boost.Asio のスレッド（`reader_thread_`）に閉じ込め、外部 API は同期メソッドとして提供する。

## 8. シリアライゼーション／設定読み込み
- JSON: `nlohmann::json` を使い、`read_int_field` / `parse_position` のようなユーティリティで型安全に値を取得。数値フィールドは int/double/string いずれでも読めるよう防御的に扱う。
- YAML: `YAML::Node` から値を取り出す際は `scalar_or_throw` で必須フィールドの存在と型を検証し、`initial_led` など配列フィールドの長さもチェックする。
- `std::optional` で `require_result`, `notify`, `initial_led` などの省略可フィールドを表現し、デフォルト値は構造体側に記載しておく。

## 9. CLI / Middleware インタラクション
- CLI 層は `FleetManager` の public API 以外に直接触れない。`moveall`, `subscribeall` などは *_all ヘルパーをそのまま呼び出す。
- ユーザー入力は `tokenize` → コマンドディスパッチ → `FleetManager` 呼び出しまでを直列で処理し、副作用のあるグローバル状態は `FleetManager` 内で完結させる。
- `FleetGuard` のような RAII オブジェクトで `FleetManager::stop()` を保証し、CLI 例外発生時でもセッションを安全に終了させる。
- `docs/New_develop.md` の方針通り、Controller/Command/REPL の分離を進める際はここに追記し、CLI 以外から `FleetManager` を利用するケースを想定した API 整備を行う。

## 10. 変更フロー
- 新規モジュールを追加する際は
  1. `include/` に公開 API を定義
  2. `src/` に実装
  3. `docs/`（本ファイルを含む）を更新
  4. `CMakeLists.txt` にターゲットを登録
  の順で作業する。
- 重要なロジックを追加したら `docs/cli.md` / `docs/middleware.md` / `docs/transport.md` も更新し、CLI ユーザーの想定操作が変わる場合は README のコマンド一覧を必ず同期させる。

本ガイドラインは Living Document として扱い、アーキテクチャやスタイルに変更が入った際は Pull Request で更新してください。
