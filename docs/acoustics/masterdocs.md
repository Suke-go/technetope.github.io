# MASTER DOCS FOR MAKING ACOUSTICS SYSTEM

## Goal
本システムのゴールはM5StickCPlus2とHAT（これらを音響モジュールと名付ける）を並列で処理をして、ある空間に独立的に音を響かせることを目的としている。音響モジュールはtoioの上に載せられ、toioのロコモーションに合わせてそれぞれ音色を変えることを想定している（なおこのディレクトリではtoioに関する制御は扱わない）。また、音響モジュールだけではなく、それらの体験をした支えするウーファーや6chスピーカー、パラメトリック・スピーカー（パラメトリック・スピーカーのみ余裕があった場合）の運用を考えている。これらはmacによって駆動したいと考えている。これら全体を音響システムと総称する。最終的には群ロボットの上に乗った音響モジュール（約30-50台）の豊かな音環境を下にした空間的なオーディオ制御を目指したい。

## Requirements
- M5Stick C Plus2
    - Speaker2 Hat(MAX98357) M5StickC Plus
        - Arduino IDEで開発する際はM5Stackボードマネージャで`M5StickCPlus2`ボードを選択し、ライブラリマネージャから`M5StickCPlus2`ドライバと依存ライブラリを導入する。公式リポジトリではこのドライバが非推奨となり、基盤機能は`M5Unified`と`M5GFX`に移行するよう案内されているため、これらを前提ライブラリとして組み込むことが必須。[StickC-Plus2 Arduino Quick Start](https://docs.m5stack.com/en/arduino/m5stickc_plus2/program) / [M5StickCPlus2 Library README](https://github.com/m5stack/M5StickCPlus2)
        - PlatformIOを使う場合は公式ドキュメントに従い、`platform = espressif32@6.7.0`、`board = m5stick-c`の指定に加えてPSRAM対応フラグを有効化し、`M5Unified`を依存ライブラリとして明示する必要がある。特に`-DBOARD_HAS_PSRAM`と`-mfix-esp32-psram-cache-issue`を付与しないとPSRAM初期化に失敗し音声バッファが確保できない点に留意する。[StickC-Plus2 Softwares](https://docs.m5stack.com/en/core/M5StickC%20PLUS2)
        ```ini
        [env:m5stack-stickc-plus2]
        platform = espressif32@6.7.0
        board = m5stick-c
        framework = arduino
        upload_speed = 1500000
        monitor_speed = 115200
        build_flags =
            -DBOARD_HAS_PSRAM
            -mfix-esp32-psram-cache-issue
            -DCORE_DEBUG_LEVEL=5
        lib_deps =
            M5Unified=https://github.com/m5stack/M5Unified
        ``` 
        - USB経由での書き込みにはCH9102用VCPドライバが前提となる。`Failed to write to target RAM`等の転送エラーが出る場合はWindows/MacいずれもCH9102ドライバを再インストールするのが公式推奨。macOSでは`wchmodem`ポートを選択する必要がある。[StickC-Plus2 Arduino Quick Start](https://docs.m5stack.com/en/arduino/m5stickc_plus2/program)
        - StickC-Plus2では電源管理IC(AXP192)が省かれているため、起動後に`HOLD`(GPIO4)を`HIGH`に保持しないと電源が落ちる。RTCウェイクアップ後も同様にHOLD=1を再設定するのがソフト側要件。スリープ／シャットダウンは`BUTTON C`長押しとGPIO4制御で行う。[StickC-Plus2 Power Differences](https://docs.m5stack.com/en/core/M5StickC%20PLUS2)
        - Speaker2 HatはMAX98357ベースのI2Sアンプであり、`BCLK=GPIO26`、`LRCLK=GPIO0`、`DIN=GPIO25(ジャンパでGPIO36に切替可能)`を使う。StickC-Plus向けサンプルのピン定義をPlus2に合わせて修正しないと発音しないので、`audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);`の各値を必ずStickC-Plus2の実ピンに合わせる。[Hat SPK2 Doc](https://docs.m5stack.com/en/hat/Hat-SPK2) / [SPEAKER2 Arduino Example](https://github.com/m5stack/M5StickC-Plus/blob/master/examples/Hat/SPEAKER2/SPEAKER2.ino)
        - サンプルではWi-Fiストリーミング再生のために`ESP32-audioI2S`ライブラリが必須で、`audio.setPinout()`後に`audio.connecttohost()`等でI2Sバッファを駆動する構成になっている。音声フォーマットは16bit／32〜192kHzに対応しており、高サンプリングで扱う場合もI2S設定値を合わせる必要がある。[SPEAKER2 Arduino Example](https://github.com/m5stack/M5StickC-Plus/blob/master/examples/Hat/SPEAKER2/SPEAKER2.ino) / [Botland Product Page](https://botland.store/stick-series/22870-m5stickc-plus-speaker-2-hat-module-with-speaker-max98357-m5stack-u055-b-6972934174123.html)
        - MAX98357アンプ基板は短絡／過熱保護を備えるが、HatとStickC-Plus2の組み合わせではバス配線が固定されているため、他デバイスとのピン競合がある場合はソフト側でピンモードを適切に初期化し、未使用ピンをINに戻すことが推奨されている。[Hat SPK2 Doc](https://docs.m5stack.com/en/hat/Hat-SPK2)
- Open Sound Control
    - OSCアドレス設計と型シグネチャは最初に決め、sender/receiver双方で同じテーブルを共有する。後から型を変えたくなった場合は`std::variant`等でバージョン互換レイヤを作り、`const_cast`やCスタイルキャストで無理に読み替えない。
    - 受信処理は専用スレッドやタスクに隔離し、所有権を持つバッファを`std::unique_ptr`やリングバッファで管理する。メインスレッドには`const&`で読み取り用データを渡し、「とりあえず`mutable`」やグローバル変数化は避ける。
    - UDP遅延・順序入れ替わりに備えてタイムタグを常に検査し、想定閾値を超えた場合はログとドロップ処理を行う。挙動が不明瞭なまま推測で最適化せず、`ofGetElapsedTimeMicros()`等で処理時間を測定しボトルネックの有無を判断する。
    - パースに失敗したパケットはログ化して破棄し、`reinterpret_cast`で黙って読み進めない。解析用のデバッグログを残し、再送要求やフォールバック経路の有無を検討する。
    - 密集したOSCイベントに対してCPUで逐次波形生成をせず、M5Stick側のI2S/DMAや外部DSPに委譲できないか常に検討する。逆に複雑な状態遷移はCPU側で処理し、GPU/シェーダで無理をしない。

## OSC Timetag Synchrony Concept
- **目的**: 30台規模のM5StickC Plus2がWi-Fi経由でも指定時刻に同期して音を鳴らすことを保証する。
- **時刻同期**:
    - PCを局所NTPサーバーとして動作させ、全M5Stickが起動時および定期的に時刻同期を行う。必要なら心拍メッセージでドリフトを推定し、Stick側で補正。
- **通信方式**:
    - OSC over UDPを採用し、ブロードキャスト/マルチキャストで一斉送信。メッセージはOSCバンドルにまとめ、Timetagに未来の実行時刻`Texec`を設定する。
    - ペイロードは固定長バイナリ（`sample_id`, `gain`, `pan`, `flags`など）とし、ESP32のAES-CTR/GC Mで暗号化・認証。軽量かつパース容易にする。
- **PC側ワークフロー**:
    1. NTPサーバー起動、Stick群の同期状態を監視。
    2. シーケンサ（SoundTimeline）で「どのデバイスが、いつ、どの音を、どのパラメータで鳴らすか」を管理。
    3. 各イベントに対して`Texec = now + lead_time`(例: 100ms)を計算し、OSCバンドル生成→ブロードキャスト送信。
    4. 送信ログ・失敗検知・遅延統計を記録し、閾値超過時はアラートと再送方針を検討。
- **M5Stick側ワークフロー**:
    1. 起動→NTP同期→プリセット音の`SampleRegistry`構築→OSCリスナー開始。
    2. 受信したバンドルのTimetagとローカル時刻を比較し、`Texec > now`なら`PlaybackQueue`に積む。
    3. FreeRTOSタスクで`PlaybackQueue`を監視し、`Texec`到達時にSPIFFS内のサンプルをI2S出力。割り込み優先度を調整し、Wi-Fiタスクとの競合を回避。
    4. 再生完了後は状態をログ化し、必要に応じてPCへ状態通知（遅延/エラー）を返す。
- **音素材管理**:
    - 短尺WAV/PCMをStick内のSPIFFS/LittleFSに格納し、IDで参照。容量超過や更新が必要な場合は起動時DLまたは外付けmicroSDを検討。
    - 素材バージョン表と更新手順（OTA/USB）を用意し、差し替え時の検証フローを定義。
- **信頼性向上策**:
    - コマンドは冪等に設計し、送信を2度行う/ACKチャネルを別途用意するなどで欠損対策。
    - ジッタ許容値（例: ±5ms）を定義し、測定結果が閾値を超える場合の対応（AP設定変更、lead_time調整）を手順化。
    - 専用Wi-Fi APを使用し、チャンネル固定・他トラフィック排除で衝突を抑制。必要に応じてネットワーク冗長構成を検討。
- **評価・PoC**:
    - 3〜5台でPoCを実施し、遅延・ズレ・再生成功率を数値化。`docs/test/osc_sync_results.md`に結果を残す。
    - 30台スケールテスト前に、Stick再起動時の再参加手順や時計ずれ検知→補正プロセスをリハーサルする。

## Coding Rules
- **データ所有と責務分離**
    - `ofApp.h`に変数を安易に追加しない。OSC受信、音響レンダリング、UI、ログなどの責務ごとに専用クラスを定義し、データ所有者を明確にする。
    - 共有が必要な情報は`const&`で読み取り専用として公開し、書き込みが必要な場合のみ`&`を渡す。`mutable`の乱用は禁止。
- **キャスト禁止事項**
    - `const_cast`、Cスタイルキャスト`((Type*)p)`、`reinterpret_cast`を使った“とりあえず動かす”対応を禁止する。型不一致が発生したら設計（インターフェースやポリモーフィズム、`std::variant`）を見直す。
    - APIの`const`制約を外したい場合は、なぜ`const`なのかを検証し、設計変更を優先する。
- **メモリ管理とRAII**
    - 所有権を伴う生ポインタを禁止。`std::unique_ptr`で単一所有、共有が必要な場合は`std::shared_ptr`を使用し、RAIIでリソース寿命を管理する。
    - 外部リソース（ファイル、ソケット、バッファ、シェーダ、FBOなど）は作成と同じスコープで確実に解放されるようラッパークラス化する。
- **CPU/GPU・周辺機能の役割分担**
    - ピクセル処理や重いDSP処理をCPUのforループで済ませない。GPUまたはESP32のI2S/DMA、周辺機能へオフロードできるか常に検討する。
    - 逆に、複雑な状態管理や条件分岐をシェーダに押し込めない。CPU側で制御しやすい設計を優先する。
- **パフォーマンス計測**
    - 「たぶん速い／遅い」で判断しない。`ofGetElapsedTimeMicros()`等で処理時間を計測し、ボトルネック特定後に最適化する。
    - OSCやI2S処理のスループットが足りない場合は、ログとメトリクスを元にボトルネックの所在（ネットワーク・メインループ・オーディオタスク）を明確にする。
- **ドキュメント・リファレンス重視**
    - 仕様が不明なAPIは必ず公式ドキュメントかヘッダを確認してから利用する。推測ベースの呼び出しや副作用無視は禁止。
    - 参照した資料（データシート、GitHubサンプル、ブログ等）は課題チケットやドキュメントにURLで明記し、後追い可能にする。
- **リソースの“死”まで見届ける**
    - `setup()`で生成したオブジェクトは`exit()`やデストラクタで必ず破棄する。`new`したら`delete`、`malloc`したら`free`ではなく、ライフサイクルをクラスで管理する（RAII徹底）。
    - Wi-Fiやオーディオストリームの停止処理（`audio.stopSong()`、`WiFi.disconnect(true)`など）を忘れず、長期運用でリソースリークが起きないようにする。

## Task List
1. OSCインターフェース設計を完了する
    - 目標:sender/receiver双方が共有するOSCアドレス・型シグネチャ表とタイムタグ仕様を確定し、ドキュメント化する。
    - 必須要件:型互換レイヤ(`std::variant`等)の方針、パケット破損時の破棄ルール、遅延計測方法を明文化し、測定用ログ出力を設計に含める。
    - 詳細ToDo:
        1. 既存資料・参考実装を読み、必要なパラメータ（音量、パン、再生IDなど）を洗い出して一覧化する。
        2. 各パラメータごとにOSCアドレス、型、許容値範囲、既定値、更新頻度を定義し、Markdownにまとめレビュー依頼を出す。
        3. 遅延測定・補正指針（時刻ソース、閾値、補正アルゴリズム）を設計図に書き込み、チーム合意を取る。
        4. 不正パケット処理フロー（破棄条件・ログ形式・再送方針）をフローチャート化し、Notion/Repoの設計ページに反映する。
        5. 確定した仕様を共有リポジトリの`docs/spec/osc_contract.md`に反映し、アルバイト含む全員への周知ミーティングで説明する。
2. PC側OSCディスパッチャを実装・検証する
    - 目標:専用スレッドでOSC受信し、所有権管理されたリングバッファを介してメインループに`const&`でデータを渡す構造を構築する。
    - 必須要件:危険キャスト禁止(Cスタイル/`const_cast`/`reinterpret_cast`)、処理時間の計測とログ出力、CPU/GPU役割分担の検証(重い処理はGPU/別タスクへ委譲可否を見直す)。
    - 詳細ToDo:
        1. 受信モジュールのクラス図を描き、責務分離ポリシー（受信→解析→配布）を設計レビューで確定する。
        2. Boost.Asio等のOSCライブラリ選定理由を比較表にまとめ、採用決定後にセットアップ手順をREADMEに追記する。
        3. リングバッファ実装時に`std::unique_ptr`で所有権を管理し、push/pop時のロック方針（mutex or lock-free）を決め性能評価する。
        4. 処理時間計測のために`ofGetElapsedTimeMicros()`でラップした計測コードを追加し、メトリクスをCSVで出力する仕組みを作る。
        5. 危険キャスト禁止の静的解析ルール（clang-tidyなど）をCIに組み込み、簡単なサンプルでCIが通ることを確認する。
        6. ユニットテストで異常系（破損パケット・遅延超過）を再現し、期待通りドロップ・ログ出力されるか検証する。
3. M5StickC Plus2ファームウェアを実装する
    - 目標:`M5Unified`/`M5GFX`ベースでI2S出力とOSC受信を統合し、HOLD(GPIO4)制御と`audio.setPinout()`のピン定義修正を行う。
    - 必須要件:PSRAMフラグ設定済みビルド環境(PlatformIO/Arduino)を整備し、CH9102ドライバ確認、`ESP32-audioI2S`導入、Wi-Fi接続の確立と異常時リソース解放。
    - 詳細ToDo:
        1. PlatformIO環境構築手順をスクリーンショット付きでドキュメント化し、新人用オンボードタスクとしてチケット化する。
        2. `platformio.ini`にPSRAM設定、`lib_deps`を追加し、サンプルビルドでPSRAMが有効になっているかシリアルログで確認する。
        3. ファームコードでHOLD(GPIO4)の初期化・スリープ復帰処理を実装し、動作確認（電源落ちないこと、再起動シナリオ）を動画で残す。
        4. `audio.setPinout()`をPlus2のピン配置に修正し、Speaker2 Hatから音が出ることを確認する運用手順（音声ファイル、期待音）を記録する。
        5. Wi-Fi接続・切断シナリオをテストし、タイムアウト発生時に`audio.stopSong()`や`WiFi.disconnect(true)`が呼ばれることをログで検証する。
        6. ファームの例外/エラーハンドリング方針をREADMEに追記し、アルバイト向けトラブルシュート手順をまとめる。
4. 統合テストと性能検証を実施する
    - 目標:PC↔M5Stick間でOSC連携した状態で音声再生を行い、遅延・ドロップ率を計測して許容閾値と比較する。
    - 必須要件:計測データをログ化し、タイムタグ補正/再送方針を評価。CPU負荷・I2Sバッファ枯渇・メモリ使用量を監視し、設計改善事項を洗い出す。
    - 詳細ToDo:
        1. テストケース表を作成し、シナリオ（通常遅延、遅延大、パケット欠損、複数台同時）ごとの合格基準を明記する。
        2. 計測用スクリプトを準備し、遅延・ドロップ率・CPU使用率を自動採取してCSV/グラフ生成まで行う簡易ダッシュボードを作る。
        3. 測定結果に基づき、閾値を超えた項目について原因調査コメントと改善策（例:バッファサイズ変更、Wi-Fiチャンネル変更）を記載する。
        4. テスト環境セットアップ手順（配線図、IP設定、電源管理）を写真付きでWikiにまとめ、だれでも再現できるようにする。
        5. テスト後はログ・計測データを共有ストレージに保管し、レポートをPull Requestに添付してレビュー依頼する。
5. 運用準備とドキュメント整備を行う
    - 目標:複数台(toio搭載想定)展開時の手順書、OSCアドレス表、ビルド環境手順、リソース掃除手順(停止・再起動)をまとめる。
    - 必須要件:参照資料URLを付記し、チーム全体で遵守するコーディングルールとチェックリストを明示。動作確認済みファーム/PCソフトのバージョン管理を整備する。
    - 詳細ToDo:
        1. セットアップから運用、停止、トラブルシュートまでの手順書をステップごとに分解し、アルバイトがチェックボックスで進められるようテンプレート化する。
        2. コーディングルール・レビュー項目・IssueテンプレートをGitリポジトリ内`docs/process/`に配置し、Onboardingセッションで説明する。
        3. ファームウェア、PCソフト、依存ライブラリ、ドライバのバージョン表を作成し、更新時の手順（検証→展開→周知）を定めてSlack等で告知する。
        4. ハードウェア在庫、シリアル番号、使用履歴をスプレッドシートで管理し、貸し出し/返却手順を決めて運用する。
        5. 追加開発やバグフィックスが発生した際のエスカレーションルート（誰に報告・どのIssueテンプレートを使うか）を明文化する。

### Step/Deliverable Details
- **Step 1 Deliverables**
    - 完成したOSCアドレス/型シグネチャ表（Markdownまたはスプレッドシート）
    - パケット破損・遅延時の処理フローチャート
    - シミュレータやテストスクリプトで送受信を検証するためのモックコード
- **Step 2 Deliverables**
    - PC側受信モジュールのクラス設計図と実装(`Receiver`, `Dispatcher`, バッファ管理クラス)
    - 処理時間・遅延を記録するメトリクスログサンプル
    - 危険キャスト使用がないことを示すコードレビュー記録
- **Step 3 Deliverables**
    - PlatformIO/Arduino設定ファイル（`platformio.ini`や`.ino`）と依存ライブラリ一覧
    - HOLD制御・I2Sピン設定を行うファームウェアコードとユニットテスト
    - Wi-Fi接続・異常時リカバリを検証したログ、リソース解放手順書
- **Step 4 Deliverables**
    - 遅延・ドロップ率・CPU/I2Sバッファ使用率を示す測定レポート
    - 許容閾値を満たさない場合の改善策リストと対応優先度
    - テスト環境構成図（ネットワーク・電源・使用デバイス）
- **Step 5 Deliverables**
    - 展開マニュアル(初期セットアップ→運用→停止手順)
    - コーディングルールチェックリスト、レビュー手順、Issueテンプレート
    - バージョン表（ファームウェア・PCアプリ・依存ライブラリ・ドライバ）


## アイディア
- 時刻動機の方法としてNTPとかRTPを用いてRTCのズレがないことを祈りながら指示をおくるなど？
- 
