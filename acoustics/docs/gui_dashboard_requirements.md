# Acoustic GUI Dashboard Requirements

M5 音響システムを安全に遠隔運用するための Web GUI ダッシュボード要件を定義する。GUI はモニタ／スケジューラの補助ツールではなく、「デバイス状態の参照」「単発発火」「タイムライン配信」「診断」の単一ハブとして機能する。仕様変更時は必ず `acoustics/docs/masterdocs.md`・`acoustics/docs/gui_spec.md`・本書を同時に更新する。

---

## 0. 参照ドキュメント
- `acoustics/docs/masterdocs.md`（全体アーキテクチャ）
- `acoustics/docs/gui_spec.md`（UI/ImGui 実装詳細）
- `acoustics/docs/m5_scheduler_alignment_plan.md`（NTP 揃えと送信リードタイム）
- `acoustics/process/coding_rules.md`（運用ルールのクイックリファレンス）
- `acoustics/pc_tools/gui/` 配下コード（C++/ImGui 実装）

---

## 1. 目的と適用する Coding Rules
1. **データ可視化の中心はデバイス状態**  
   - オンライン台数、各 `device_id`、レイテンシ、時刻同期を 1 画面で把握できること。
2. **送受信のトレーサビリティ確保**  
   - いつ・どのプリセットを誰に送ったか、成功／失敗・失敗理由まで 10 秒以内に遡れる。
3. **即時操作の安全性**  
   - GUI から単発再生／タイムライン送信を確実に行いつつ、誤送信や暗号化ミスを防ぐ。
4. **NTP 絶対時刻基準**  
   - 全表示を NTP サーバー時刻に紐づけ、「何時に何が鳴るか」を UTC/JST 両方で提示。

### Coding Rules へのマッピング
- **責務の分離**: GUI を 4 層（データ取得、状態ストア、UI レンダリング、コマンド実行）に分割し、通信/描画/操作を混在させない。  
- **危険なキャスト禁止**: OSC ペイロードや JSON パースでは型安全なラッパー（`nlohmann::json` or `std::span<const uint8_t>`）のみ使用する。  
- **RAII によるリソース管理**: WebSocket/REST クライアント、ログファイル、NTP セッションは `std::unique_ptr` とスコープ付きクリーンアップで保持し、GUI 終了時に自動解放する。  
- **最適化前の計測**: タイムライン送信遅延、WebSocket 再接続時間、フレーム時間を `std::chrono` or `ofGetElapsedTimeMicros()` で計測・ログ化してからチューニングする。  
- **API 参照の徹底**: Monitor/Scheduler/API のヘッダー (`OscTransport.h` など) を読んでレスポンススキーマを確認し、本書に引用を残す。  
- **シャットダウンパスの担保**: GUI クローズ時に Wi-Fi/OSC ソケット、ログファイル、タイムライン送信ジョブを必ず停止する。未送信ジョブはキャンセル／再送キューに退避する。

---

## 2. システムコンテキストと責務
| レイヤー | 主責務 | 実装候補 | 備考 |
| --- | --- | --- | --- |
| Data Intake | `state/devices.json`・Heartbeat・Diagnostics を WebSocket/REST で取得 | `OscTransport`, `MonitorBridge` | 500 ms ポーリング + イベント駆動。接続喪失時は指数バックオフ。 |
| State Store | 最新スナップショットと履歴 5 分分を保持 | `DeviceStore`, `TimelineStore` | UI スレッドとの同期に lock-free SPSC queue を使用。 |
| UI Rendering | ImGui Docking＋各パネル | `GuiDashboard`, `Panels::*` | 60 FPS を維持、遅延はステータスバーで可視化。 |
| Command Dispatch | Timeline/単発発火/設定変更 | `CommandBus`, `OscSender` | 送信結果をログストリームへ publish。 |

---

## 3. UI モジュール要件
1. **トップバー（固定ヘッダー）**  
   - NTP 現在時刻を UTC/JST で並記。  
   - Scheduler・Monitor・Secrets ロード状態（OK/NG）。  
   - 過去 60 分の送信成功/失敗/拒否数をミニグラフ表示。

2. **デバイスダッシュボード**  
   - 情報源: `state/devices.json` + Heartbeat ストリーム。  
   - カード項目: `device_id`、Alias、IP:Port、最終 heartbeat（ISO8601）、RSSI、遅延平均/最大、NTP 偏差、タグ。  
   - 遅延に応じた色分け（緑 <50 ms / 黄 50–150 ms / 赤 >150 ms）。5 秒更新が無い場合は “offline”。  
   - カード右上にテスト信号ボタン（短いプリセット送信）。結果をカード下部に記録 (`Last test hh:mm:ss`）。

3. **タイムライン & NTP 軸ビュー**  
   - 予定イベントをテーブル＋ガントで表示（列: 予定時刻 NTP, 残り時間, Target, Preset, リードタイム, 状態）。  
   - JSON アップロード → プレビュー → `Send Timeline` ボタン。`--dry-run` 相当のタイムラインシミュレーションを提供。  
   - ガント上でドラッグしてリードタイムを調整すると REST 送信時に自動補正。

4. **送受信ログパネル**  
   - WebSocket (`/ws/events`, `/ws/sendlog`) からリアルタイム表示。  
   - フィルタ（device_id/preset/severity/期間）と一時停止、`logs/gui_dashboard_sendlog.csv` へのエクスポート。  
   - Fatal イベントはデスクトップ通知とバッジ表示。

5. **単発発火コンソール**  
   - フォーム: Target（デバイス or グループ）、Preset、Lead 秒、ゲイン、再生時間上限。  
   - 「Arm」チェックボックス + 「Fire」ボタンの二段階。  
   - 暗号鍵は `Secrets` から自動選択、鍵不一致時は即エラー。  
   - 送信完了までリードタイム残りをカウントダウン表示。

6. **診断センター**  
   - `/diagnostics/reject` リスト（時刻、device_id、理由、関連イベント ID、推奨アクション）。  
   - 行クリックでデバイスカードへジャンプ。  
   - 備考欄でオペレーターがメモを残せる（`state/diagnostics_notes.json` に保存）。  
   - 重大度 High が一定件数を超えるとトップバーに警告バッジ。

---

## 4. データ / API 要件
| データ種別 | 提供元 | 取得方法 | 更新頻度 | 利用先 |
| --- | --- | --- | --- | --- |
| `state/devices.json` | Monitor | HTTPS GET + WebSocket 増分 | 500 ms + イベント | デバイスカード、Target 選択 |
| Heartbeat | Monitor | `/ws/events` (type=`heartbeat`) | イベント駆動 | 状態バッジ、遅延色分け |
| Diagnostics/Reject | Monitor | `/ws/events` (type=`diagnostics`) | イベント駆動 | 診断センター、通知 |
| Scheduler 送信ログ | Scheduler | `/ws/sendlog` | イベント駆動 | タイムライン、ログ |
| Timeline 送信 | GUI → Scheduler | `POST /api/timeline` (body=JSON) | 手動 | ガント更新、ログ |
| 単発発火 | GUI → Scheduler | `POST /api/play` | 手動 | 発火コンソール |
| NTP 情報 | Scheduler or Monitor | `GET /api/time` | 1 s | トップバー、残り時間算出 |

送信・受信いずれも TLS を前提とし、API 失敗時は指数バックオフ＋ UI トースト表示。REST 応答には `request_id` を含め、ログと突合できるようにする。

### API スキーマ参照とフィールド検証
- Monitor 由来データは `acoustics/pc_tools/libs/include/acoustics/osc/OscTransport.h` と `acoustics/state/devices.json` を正とし、GUI 側で取得値を `device_id`・`ntp_offset_ms`・`latency_mean_ms` など型安全にマッピングする。  
- Scheduler 由来の `POST /api/timeline` と `/api/play` は `acoustics/pc_tools/scheduler/include/acoustics/scheduler/SchedulerController.h`・`cli.py` の実装をリファレンスとし、フィールドを増やす際は本書に差分を必ず記す。  
- WebSocket (`/ws/events`, `/ws/sendlog`) では `enum class EventType` に揃えた `type` フィールドと `request_id` を必須化し、GUI は未知フィールドを無視して forward compatibility を得る。

`state/devices.json` の 1 エントリ例:

```json
{
  "device_id": "m5-001",
  "alias": "north-west",
  "ip": "192.168.1.41",
  "port": 7000,
  "last_heartbeat": "2025-01-18T13:22:44.110Z",
  "ntp_offset_ms": -3.4,
  "latency_mean_ms": 42,
  "latency_p99_ms": 77,
  "tags": ["mother_mf", "stageA"]
}
```

タイムライン送信 JSON 例 (`acoustics/pc_tools/scheduler/examples/mother_mf_a_section.json` を縮約):

```json
{
  "timeline_id": "mother-mf-a",
  "events": [
    {
      "time_utc": "2025-01-18T14:00:05.000Z",
      "targets": ["m5-001", "m5-014"],
      "preset": "A4_mf",
      "lead_time_ms": 4000,
      "gain": -3.0
    }
  ]
}
```

GUI が `/ws/sendlog` で受け取るイベント例:

```json
{
  "request_id": "req-9d31",
  "type": "sendlog",
  "status": "succeeded",
  "device_id": "m5-001",
  "preset": "A4_mf",
  "scheduled_time_utc": "2025-01-18T14:00:05.000Z",
  "delivered_at_utc": "2025-01-18T13:59:57.100Z",
  "latency_ms": 12
}
```

上記スキーマを `nlohmann::json` の構造体バインディング（`from_json`/`to_json`）で表現し、危険なキャストを排除する。

---

## 5. ユーザーフロー
1. **起動**: GUI は設定ファイル/Secrets を読み込み、`state/devices.json` が無い場合は空のテンプレを生成。NTP 同期が取れない場合はブロッキングダイアログ。  
2. **監視**: デバイスカードで状態を把握。遅延異常があればカード背景が警告色になり、ログへ自動展開。  
3. **タイムライン配信**: JSON をアップロード → プレビューで NTP 時刻と整合性を確認 → Arm → Send。送信後は `/ws/sendlog` で結果を反映。  
4. **単発発火**: 2段階操作で Target/Preset を指定し、完了後ログパネルに結果（成功/失敗理由）を記録。  
5. **診断対応**: `/diagnostics/reject` → デバイスカード → 再送 or メモ記録 → 状態改善を確認。  
6. **終了**: GUI 終了時に未送信ジョブをキャンセルし、ログファイルと WebSocket を確実にクローズ。

---

## 6. 非機能要件
- **性能と計測（Coding Rule #4）**: フレーム時間、WebSocket 再接続、`POST /api/timeline` の往復時間を `std::chrono::steady_clock` でサンプリングし、`logs/gui_dashboard_metrics.jsonl` に 1 秒間隔で書き出す。フレーム時間は p95 < 16 ms、再接続は 5 s 未満、送信リードタイムは 10 ms 単位で可視化する。  
- **リソース管理（Coding Rule #3）**: WebSocket/REST クライアント、NTP セッション、ログファイルは `std::unique_ptr`＋専用デリータで所有し、`GuiDashboard::Shutdown()` 内で順序付きに解放。`scope_exit` でハートビート購読の解除漏れを防ぐ。  
- **責務分離とスレッド安全（Coding Rule #1）**: ネットワーク I/O はバックグラウンドスレッドで `DataIntakeLoop` が処理し、UI スレッドとは lock-free SPSC queue (`moodycamel::ReaderWriterQueue` 想定) で連携する。UI はステートのスナップショットのみを描画し、副作用は `CommandBus` に委譲する。  
- **トレーサビリティと観測性（Coding Rule #2/#5）**: すべての送受信イベントに `request_id`・`operator`・`crypto_revision` を付与し、`logs/gui_audit.jsonl` と `/ws/sendlog` の内容が 1:1 で照合できるようにする。未知フィールドや暗号鍵 mismatch は WARN として通知し、監査用 CSV にも記録。

### Coding Rules トレーサビリティチェック
| Coding Rule | 本要件での担保 | 検証方法 |
| --- | --- | --- |
| 1. 責務の分離 | セクション 2 の 4 層構造＋セクション 6 の SPSC queue 要件で通信・状態・UI・コマンドを分離。 | `GuiDashboard` 設計レビューで各層が独立クラスになっていることを確認。 |
| 2. 危険なキャスト禁止 | セクション 4 の JSON スキーマと `nlohmann::json` バインディングで型安全に変換。 | `-Wold-style-cast` を CI で有効にし、静的解析で未使用キャストを検出。 |
| 3. RAII によるリソース管理 | セクション 6 の `GuiDashboard::Shutdown()` 手順と `scope_exit` の使用。 | `valgrind` + 強制終了テストでハンドルリークが無いことを確認。 |
| 4. 最適化前の計測 | セクション 6 のメトリクス出力と閾値。 | `logs/gui_dashboard_metrics.jsonl` が生成され、閾値を CI で検証。 |
| 5. API 参照の徹底 | セクション 4 のヘッダーファイル参照とスキーマ例。 | API 変更時に本書と `gui_spec.md` が同時更新されているかコードレビューで確認。 |
| 6. シャットダウンパスの担保 | セクション 5 (終了手順) とセクション 7 (安全なシャットダウン) の要件。 | 終了シナリオの統合テストで未送信ジョブが再送キューに移ることを確認。 |

---

## 7. 安全性・障害対応
- **誤操作防止**: 発火/タイムライン送信は Arm → Fire の 2 ステップ + 最終確認モーダル。  
- **暗号鍵ミス防止**: `Secrets` を UI 起動時に検証し、鍵ハッシュをステータスバーへ表示。  
- **レートリミット**: 単発発火はデバイスごとに 5 秒間隔、タイムライン送信は 30 秒間隔でキュー管理。  
- **監査ログ**: すべての送受信操作に `request_id`, `operator`, `timestamp` を付与し `logs/gui_audit.jsonl` に保存。  
- **フェイルオーバー**: WebSocket 切断時は画面上に「Stale」バッジを出し、再接続試行を可視化。  
- **安全なシャットダウン**: RAII で管理する通信リソースを GUI 破棄時にクローズし、未処理イベントはディスクへ退避。

---

## 8. 実装ロードマップ
1. **通信層整備**  
   - `acoustics/pc_tools/gui/src/net/` に WebSocket/REST クライアントを分離。  
   - Monitor/Scheduler からのイベントを `CommandBus` に publish。  
   - API 仕様を `gui_spec.md` に即時反映。
2. **状態ストア実装**  
   - `DeviceStore` と `TimelineStore` を C++ クラスとして実装し、SPSC queue で UI スレッドへ受け渡し。  
   - 記録保持は 5 分、ダウンサンプリングは 2 s 間隔。  
3. **UI パネル構築**  
   - トップバー／デバイスカード／タイムライン／ログ／診断／発火フォームを個別クラスに分離。  
   - 遅延色分け、通知バッジ、ImGui Docking を組み込む。  
4. **操作系統合**  
   - Timeline/Play API 呼び出しと Arm/Fire ガードを実装。  
   - 結果をログパネル + 監査ログへ流し込む。  
5. **最適化とハードニング**  
   - フレーム時間 >16 ms が 5% を超えないようプロファイル。  
   - 再接続・シャットダウンパスをテストし、CI に GUI テスト（`acoustics/tests/gui_end_to_end.md`）を追加。

---

## 9. テスト & 検証
- **ユニットテスト**: WebSocket クライアント、状態ストア、Arm/Fire ガード、NTP 差分計算。  
- **統合テスト**: Monitor/Scheduler/GUI を同一ネットワークで接続し、デバイス追加 → タイムライン送信 → 診断 → 単発発火の一連シナリオを `acoustics/tests/gui_end_to_end.md` に記述。  
- **UX 検証**: 操作ログ + オペレーターインタビューを実施し、フィードバックを `gui_spec.md` にフィード。  
- **パフォーマンス計測**: 100 台相当のモック heartbeat を流し、CPU <40%、メモリ <500 MB を目標にする。  
- **リソースリーク検査**: GUI 終了時にソケット/ファイルが閉じられているか `valgrind` でチェック。

---

## 10. 今後の拡張余地
- 自動リードタイム補正（混雑状況に応じた推奨値を表示）。  
- セッション記録とリプレイ（過去イベントを NTP 基準で再生）。  
- ロール／権限管理（表示のみ・送信可・管理者など）。  
- `/sync/ping`・`/sync/adjust` ハンドシェイク連携で NTP 精度を GUI から補正。  
- ミニレイテンシグラフ（Sparkline）や地図ビュー等の高度可視化。

本要件を満たすことで、GUI は Monitor/Scheduler/現場オペレーターの信頼できる運用基盤となる。仕様を変更する場合は、本書と `gui_spec.md` に差分理由を記録し、README や masterdocs とも整合させること。
