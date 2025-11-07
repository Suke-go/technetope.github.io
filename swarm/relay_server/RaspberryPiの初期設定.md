
# Raspberry Pi の初期設定
### ネットワーク
```bash
sudo nmcli device wifi connect "SSID名" password "パスワード"
sudo nmcli device wifi connect "JCOM_OIHI" password "530741765746"
```

### 基本更新
```bash
sudo apt update && sudo apt full-upgrade -y
```

### BLE 関連とビルドに必要な最低限
```bash
sudo apt install -y bluetooth bluez bluez-tools \
  libbluetooth-dev libglib2.0-dev \
  python3-full python3-venv python3-pip \
  libdbus-1-dev dbus git
```

### Bluetooth デーモン起動
```bash
sudo systemctl enable --now bluetooth
```

### 確認
```bash
bluetoothctl show
```

# uv
### uv インストール（公式ワンライナー）
```bash
curl -LsSf https://astral.sh/uv/install.sh | bash
```
### 今のシェルの確認
```bash
echo $SHELL
```

### パス通す（~/.bashrc or ~/.zshrc に追記）
```bash
echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc  # bash の場合
echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.zshrc  # zsh の場合
```

### 反映
```bash
source ~/.bashrc  # bash の場合
source ~/.zshrc   # zsh の場合
```

# コード
### technetope リポジトリ クローンと toio ブランチへ切り替え
```bash
git clone https://github.com/Suke-go/technetope.git
cd technetope
git switch -c toio origin/toio 
```

### uv 環境のセットアップ
```bash
uv python install 3.12
uv lock --upgrade --refresh
uv sync
```

### 起動してみる
```bash
uv run uvicorn relay_server.server:app --host 0.0.0.0 --port 8765 --reload
```


# 自動起動
### systemd サービスファイル作成
```bash
sudo tee /etc/systemd/system/swarm-relay.service >/dev/null <<'UNIT'
[Unit]
Description=Swarm Relay (FastAPI + BLE)
After=network-online.target bluetooth.service
Wants=network-online.target bluetooth.service

[Service]
# 実行ユーザー
User=technetope
# 作業ディレクトリ（プロジェクトのルート）
WorkingDirectory=/home/technetope/technetope/swarm

# 起動コマンド
ExecStart=/home/technetope/.local/bin/uv run uvicorn relay_server.server:app --host 0.0.0.0 --port 8765

# 異常終了時は再起動
Restart=on-failure
# ログをすぐ流す
Environment=PYTHONUNBUFFERED=1

[Install]
WantedBy=multi-user.target
UNIT
```

### 反映して起動
```bash
sudo systemctl daemon-reload
sudo systemctl enable --now swarm-relay
```

### ステータス確認
```bash
sudo systemctl status swarm-relay
```

### bluetoothを再起動(toioを切断できる)
```bash
sudo systemctl restart bluetooth
```