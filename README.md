# NHK2024_R2_Raspi

## 色々処理を追加するとき
基本的には`main`関数の`while True ... `の中身を書き換えてもらえると.

## 実行方法
### 初回
リポジトリのクローン
```
git clone git@github.com:T-semi-Tohoku-Uni/NHK2024_R2_Raspi.git
cd NHK2024_R2_Raspi
```
`pyenv`で環境の初期化をする
```
python -m venv env
. ./env/bin/activate
```
ライブラリのインストール
```
pip install -r requirements.txt
```
プログラムの実行
```
cd src
python main.py
```

仮想環境から抜ける時は次のコマンドを実行
```
deactivate
```

### 2回目以降
初回時に作った`env`を起動させる. (ディレクトリは`NHK2024_R2_Raspi`で実行する)
```
. ./env/bin/activate
```
プログラムを実行する
```
cd src
python main.py
```

### UDPのポートが空いてないぞ！と怒られる場合
多分、プログラムは停止せずにエラーログだけ出力されるはずだけど、一応.
以下のコマンドを実行して、バックグラウンドで動いてるプログラムを止める
```
sudo systemctl stop nhk2024.service
```

### 新しくライブラリを追加した時
`requirements.txt`を更新する.
githubを更新するのを忘れないようにね.
```
pip freeze > requirements.txt
```

## ログの確認
標準出力する内容は, 同時にファイルにも保存している.

`src/logs/YYYMMMDDD_HHTT`ディレクトリ(`YYY...`はその時の時刻)が作成され, その中で
- `can/` : canのIDごとにログを保存
- `udp/` : クライアントのIPアドレスごとにログを保存
- `error.log` : エラーをハンドリングしたものを保存（なるべくプログラムを止めないように, エラーをハンドリングしている）
- `log_main.log` : 上記全てのログを時系列順に記録

リアルタイムでログを確認するには, 次のコマンドを実行
```
tail -f log_main.log
```

## プログラムの自動起動
ラズパイの電源が入った時に, 自動でプログラムを起動する. 
まずは`/etc/systemd/system`で`sudo vim nhk2024.service`を実行して, ファイルを作成し, 以下の内容を記述する
```
[Unit]
Description=NHK2024 UDP Server
After=network.target

[Service]
ExecStart=/bin/bash -c 'cd /home/pi/NHK2024/NHK2024_R2_Raspi && . ./env/bin/activate && cd src && rm -rf logs && python main.py'
WorkingDirectory=/home/pi/NHK2024/NHK2024_R2_Raspi
StandardOutput=inherit
StandardError=inherit
Restart=always
User=root

[Install]
WantedBy=multi-user.target
```
以前のログファイルの削除と, 仮想環境の起動, `main.py`の起動を行う

~~常時起動を有効する~~ (CANの設定の関係でこれでは動かなかった)
```
sudo systemctl daemon-reload
sudo systemctl enable nhk2024.service
sudo systemctl start nhk2024.service
```

`can_init.sh`にcanの初期設定とプログラムの起動を行うコマンドを記載した.
これを, `/usr/local/bin/`へ移動する
```
sudo cp can_init.sh /usr/local/bin/
```
実行権限を変更
```
sudo chmod 700 /usr/local/bin/can_init.sh
```
`/etc/rc.local`を開く
```
sudo vim /etc/rc.local
```
最後の行で, `can_init.sh`を実行する
```
_IP=$(hostname -I) || true
if [ "$_IP" ]; then
  printf "My IP address is %s\n" "$_IP"
fi
/usr/local/bin/can_init.sh & // <- これを追加
exit 0
```

再起動する
```
sudo reboot
```

状態を確認
```
sudo systemctl status nhk2024.service
```
`active`になってればOK
```
nhk2024.service - NHK2024 UDP Server
     Loaded: loaded (/etc/systemd/system/nhk2024.service; enabled; preset: enabled)
     Active: active (running) since Wed 2024-02-28 16:42:00 JST; 27min ago
   Main PID: 6867 (python)
      Tasks: 1 (limit: 8734)
        CPU: 706ms
     CGroup: /system.slice/nhk2024.service
             └─6867 python main.py
```

## ラズパイでRealsenseを使えるようにしたかった
- まず、以下を参考にやってみた\
https://github.com/IntelRealSense/librealsense/blob/development/doc/installation.md#prerequisites
https://raspida.com/rpi-buster-error

```
sudo apt-get update --allow-releaseinfo-change
sudo apt full-upgrade
```

- 前準備
```
sudo apt-get install libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev
```
```
sudo apt-get install git wget cmake build-essential
```
```
sudo apt-get install libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev at
```
- librealsenceインストール
```
git clone https://github.com/IntelRealSense/librealsense.git
```
```
./scripts/setup_udev_rules.sh
```
ここでPermission deniedがたくさん出てエラー

- 次に以下を参考にやってみた\
https://github.com/IntelRealSense/librealsense/blob/master/doc/installation_raspbian.md
https://openvino.jp/intel-realsense-camera-d435i-2/

swap追加
```
sudo nano /etc/dphys-swapfile
```
CONF_SWAPSIZE=2048に変更

```
sudo /etc/init.d/dphys-swapfile restart swapon -s
```

E: Unable to locate packageが出るものは飛ばした結果、以下をインストール
```
sudo apt-get install -y libdrm-amdgpu1 libdrm-dev libdrm-exynos1 libdrm-freedreno1 libdrm-nouveau2 libdrm-omap1 libdrm-radeon1 libdrm-tegra0 libdrm2

sudo apt-get install libglu1-mesa libglu1-mesa-dev glusterfs-common libglu1-mesa libglu1-mesa-dev

sudo apt-get install libglu1-mesa libglu1-mesa-dev mesa-utils mesa-utils-extra xorg-dev libgtk-3-dev libusb-1.0-0-dev
```

librealsenseのクローン
```
git clone https://github.com/IntelRealSense/librealsense.git

cd librealsense

sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/ 

sudo udevadm control --reload-rules && udevadm trigger 
```

ここでPermission deniedがたくさん出てエラー

- 以下を参考にやってみた\
https://github.com/datasith/Ai_Demos_RPi/wiki/Raspberry-Pi-4-and-Intel-RealSense-D435

```
sudo su
udevadm control --reload-rules && udevadm trigger
exit
```

pathの追加
~/.bashrcにexport LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATHを追加
```
source ~/.bashrc 
```

- まだインストールしてなかったパッケージのインストール
```
sudo apt-get install automake libtool
```

システム領域の拡張
```
sudo raspi-config
```
Advanced Options -> Expand Filesystemsを選択し、再起動

~~protobufのインストール~~

~~`cd ~`~~
~~git clone --depth=1 -b v3.10.0 https://github.com/google/protobuf.git~~
~~cd protobuf~~
~~./autogen.sh~~
~~./configure~~
~~make -j1~~
~~sudo make install~~
~~cd python~~
~~export LD_LIBRARY_PATH=../src/.libs~~
~~python3 setup.py build --cpp_implementation~~

~~error: invalid use of incomplete type ‘PyFrameObject’ {aka ‘struct _frame’}がでる。~~
~~->python 3.11以降'PyFrameObject'が使えないことによるエラーらしい~~

~~sudo ldconfig をして sudo make uninstall をして cd .. && rm -rf protobuf/~~\
protobuf,libtbb-devはインストールされていたので飛ばす

librealsenseのmake
```
cd ~/librealsense
mkdir  build  && cd build
cmake .. -DBUILD_EXAMPLES=true -DCMAKE_BUILD_TYPE=Release -DFORCE_LIBUVC=true
make -j1
sudo make install
```
realsense-viewerが起動できる

~~pyrealsenseのmake
(このときenvをactivateにするとwhich python3がenvの方を指してくれる)~~\

~~`cd ~/librealsense/build`
cmake .. -DBUILD_PYTHON_BINDINGS=bool:true -DPYTHON_EXECUTABLE=$(which python3)
make -j1
sudo make install~~\
~~`~/.bashrcにexport PYTHONPATH=$PYTHONPATH:/home/pi/NHK2024/NHK2024_R2_Raspi/env/lib/を追加`~~
~~`source ~/.bashrc`~~

~~openglのインストール(envをactivateにすること)~~
~~pip install pyopengl
pip install pyopengl_accelerate~~
raspi-configでのGL Driverの設定は無かったので飛ばした

~~NHK2024_Camera_Libraryのrs_sample.pyを実行すると
no module named pyrealsense2のエラーが出る~~

librealsenseのみmakeした状態で
~/.bashrcにexport PYTHONPATH=$PYTHONPATH:/usr/local/OFFを追加して
source ~/.bashrcを実行するとpyrealsense2が使えるようになった
