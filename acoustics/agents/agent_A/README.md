# Agent A 作業指示書 (OSC/PCシーケンサ担当)

## 役割概要
- PC側で稼働するNTPサーバー・シーケンサ・モニタリングツールを構築し、OSCバンドルを生成してM5Stick群へ送出する。
- 詳細仕様は `acoustics/docs/masterdocs.md` の「サブシステム別 要件定義 > PC側」と `acoustics/docs/osc_contract.md` を参照。

## 作業ステップ
1. **リファレンス読解**
   - `acoustics/docs/masterdocs.md` 該当章、`acoustics/process/coding_rules.md`、`acoustics/docs/pc_workflow.md` を確認。
2. **環境構築 (Week 1)**
   - ローカルNTPサーバーをセットアップ (`chrony` 推奨、代替で `ntpd`) し、Stick群と同一セグメントで時刻配布できることを確認。
   - C++17/CMakeベースのビルド環境を整備し、以下ライブラリを取得・ビルド（Conan/vcpkg もしくはサブモジュール管理）:
     - OSC I/O: `liblo`（LGPL）または `oscpack`（非SPDXライセンス、利用時は法務確認）
     - JSONシリアライザ: `nlohmann/json`（MIT、single header）
     - CLI・ロギング: `CLI11`（BSD-3）、`spdlog`（MIT）／`fmt`（MIT）
     - 暗号化: `OpenSSL`（Apache-2.0）または `Mbed TLS`（Apache-2.0）/`libsodium`（ISC）でAES-CTR/GCMを提供
     - テスト: `Catch2`（Boost Software License 1.0）
   - 依存関係の取得手順とバージョンを `acoustics/docs/pc_workflow.md` に追記。
   - ネットワーク疎通テスト: ブロードキャスト/マルチキャストでパケットが届くかを確認し、結果を `acoustics/tests/README.md` に記録。
3. **シーケンサ実装 (Week 2-3)**
   - `acoustics/pc_tools/scheduler/` に C++ 実装のタイムライン管理ツールを構築。`nlohmann/json` でタイムラインをロードし、`SoundTimeline` 相当のデータモデルを整備。
   - OSCバンドル生成は `liblo`/`oscpack` のAPIを用いて `Texec = now + lead_time` のTimetagを埋め込む。AES層は `OpenSSL EVP` もしくは `Mbed TLS` のラッパーとして抽象化する。
   - コマンドラインは `CLI11` で `preview` / `send` を実装し、`spdlog` + `{fmt}` で実行ログを整形。ユニットテストは `Catch2` で `timeline → bundle`、暗号モジュール、設定パーサをカバー。
   - テスト手順と結果を `acoustics/tests/osc_sync_results.md` に反映。
4. **送出＆モニタリングモジュール (Week 3-4)**
   - `acoustics/pc_tools/monitor/` に C++ 実装の遅延計測・心拍受信・ログ集約ツールを作成。UDP受信は `Boost.Asio`（独立利用）または `liblo` のサーバAPIを活用。
   - CSV出力は `csv-parser`（MIT）等で自動記録し、`spdlog` でリアルタイムモニタを提供。`libsodium`/`OpenSSL` で暗号復号を統合し、設定手順を README に記載。
   - 再送ポリシーと暗号化層（AES-CTR/GCM）の構成ファイル（例: `config/pc_tools.yaml`）雛形を用意。
   - **デバイスID管理とターゲティング**
      - ブロードキャストで受信したStickから固有ID（MAC + 任意ニックネーム）と状態を収集し、`devices.json` にキャッシュ。
      - `SoundTimeline` の各イベントで「対象デバイス集合」を指定できるようデータモデルを拡張。フォールバックとしてグループID/タグをサポート。
      - CLIに `--target device_id1,device_id2` オプションを追加し、単一/複数端末に任意サンプルを送信する試験コマンドを実装。
      - 個別送信時の遅延・成功可否を `acoustics/tests/osc_sync_results.md` へ記録し、失敗時は再送ルールをチューニング。
5. **リアルタイムGUI実装 (Week 4-5)**
   - `acoustics/pc_tools/gui/` にクロスプラットフォームGUIを実装。推奨: Qt 6 / QML (C++) または PySide6/Electron + TypeScript 等、チームのスキルセットに合わせて選定。
   - 必須機能:
       - 起動時にデバイスID一覧を自動取得し、接続状態・心拍遅延を可視化するダッシュボード。
       - タイムラインのプレビュー／編集、即時反映（設定保存→schedulerと共有 or WebSocket/IPC連携）。
       - 任意デバイス／グループを選択し、サンプル再生・音量/パン/ピッチなどのパラメータをリアルタイムに送信するライブコントロール。
       - 操作履歴・送信結果・失敗理由をログパネルおよびCSVエクスポートで確認可能にする。
   - GUIとバックエンド（scheduler/monitor）の通信層（IPC, REST, WebSocket 等）を設計し、低遅延とエラーハンドリングを検証。
5. **テスト計画**
   - 単体テスト: バンドル生成関数・暗号化/復号ユーティリティ。
   - モック結合テスト: 擬似Stick（受信スクリプト）を用意してリハーサル。
   - 実機テスト: 1台→3台→5台の順で遅延データを取得し、`acoustics/tests/osc_sync_results.md` に数値を記す。
   - GUIテスト: ID選択・リアルタイム送信・エラーメッセージ表示を手動/自動で検証し、テストレポートを`tests/`配下に保存。

## C++ツールチェーン構成ガイド
- **ビルド管理**: CMake + Conan/vcpkg を前提に `acoustics/pc_tools` 直下へ `cmake/` 設定を配置。`toolchain.cmake` で共通依存を定義し、CI用に`cmake-presets.json`を整備。
- **OSCレイヤ**: 
  - `liblo`: Timetag対応が成熟、非同期サーバ用のスレッド化APIがある。LGPLのため静的リンク時は注意。
  - `oscpack`: 例外をほぼ使わず軽量だが、BSD互換ライセンスではないため配布形態チェックが必要。
  - 選定後は `pc_tools/libs/osc/` にラッパーを実装し、送信・受信の抽象インターフェース（`OscTransport`）を定義する。
- **暗号化**:
  - 高機能: `OpenSSL EVP` で AES-CTR/GCM + HMAC を実装。キー管理は Config ファイル＋環境変数で読み込み。
  - 軽量: `Mbed TLS` または `libsodium` で同等機能を実装し、`Encryptor` インターフェースで差し替え可能にする。
- **データフォーマット**:
  - `nlohmann/json` でタイムライン・設定ファイルを扱う。`SoundTimeline` は JSON Schema を別途 `docs/` に保存。
  - CSV/ログは `csv-parser` か `std::ofstream` + `{fmt}` で排出。グラフ化用に Python スクリプトと連携。
- **CLI・ログ出力**:
  - `CLI11` で `agent-a-scheduler`, `agent-a-monitor` のサブコマンドを統一。
  - `spdlog` + `{fmt}` で構造化ログ (`json`,`csv`,`text`) を提供。`/var/log/biotope/` など出力先は `.env` で切替。
- **テスト**:
  - `Catch2` で単体テスト、`ctest` で統合。OSC入出力は UDP ループバックモック、暗号化はキー長違反／改ざん検知テストを追加。
  - ベンチマークは Catch2 のマイクロベンチ機能で遅延測定モックを検証、結果は `tests/osc_sync_results.md` に反映。
- **CI/品質管理**:
  - GitHub Actions / GitLab CI 用の `cmake --build --target test` ジョブを追加。
  - `clang-tidy`, `cppcheck`, `valgrind` の実行パスを `docs/pc_workflow.md` に記載。

## 成果物・提出先
- ソースコード: `acoustics/pc_tools/` 配下（`scheduler/`, `monitor/`, `libs/` 構成）。
- 設定/手順書: `acoustics/docs/pc_workflow.md` にビルド手順、依存ライブラリ一覧、鍵管理フローを追記。
- テストログ: `acoustics/tests/` 配下にCSV/グラフを追加し、`osc_sync_results.md` に遅延数値とテスト条件を明記。
- 追加資料: `docs/masterdocs.md` のPC側章へライブラリ選定理由をリンクし、Issue起票時にバージョン情報を添付。

## レポート方法
- 各マイルストーン完了時、Issueを `acoustics/process/issue_template.md` に沿って起票。
- 進捗とブロッカーは週次スタンドアップで共有。参考ログへのリンクを明記。
