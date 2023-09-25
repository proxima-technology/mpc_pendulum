# mpc_pendulum

モデル予測制御を用いて倒立振子の制御を行うデモのプログラムです。

## 設定
ハードウェア
* [XM430-W350](https://e-shop.robotis.co.jp/product.php?id=44)
* [USBシリアル変換インターフェース U2D2](https://e-shop.robotis.co.jp/product.php?id=190)と[U2D2 PHB Set](https://e-shop.robotis.co.jp/product.php?id=325)
* 振子（木の板：130[g]、8[cm]×60[cm]）
* note PC
ソフトウェア
* Ubuntu 22.04
* ROS2 humble
* [DynamixelSDK](https://github.com/ROBOTIS-GIT/DynamixelSDK/tree/humble-devel)
* [CasADi](https://github.com/casadi/casadi)

## 環境構築
* UbuntuとROS2 humbleをインストールします。

* python環境構築します（MPCロジック開発用）。
```
sudo apt install python3-pip
pip3 install casadi matplotlib notebook
```

* CasADiをビルドします（MPC組み込み用）。
```
cd ~
git clone https://github.com/casadi/casadi
cd casadi/
mkdi build
mkdir build
cd build/
cmake .. -DWITH_IPOPT=ON
cmake .. -DWITH_IPOPT=ON -DWITH_BUILD_REQUIRED=ON
make -j8
sudo make install
echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/casadi/build/lib" >> ~/.bashrc
```

* dynamixel SDKとmpc_pendulum（本プログラム）をビルドします。
```
cd ~
mkdir ros2_ws
cd ros2_ws
mkdir src
cd src
git clone -b humble-devel https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone https://github.com/proxima-technology/mpc_pendulum.git
cd ..
colcon build 
```

## 使用方法
1. MPCロジック開発をします。
```
cd ~/ros2_ws/src/mpc_pendulum/lib
jupyter notebook
```
上からセルを実行していくと、`~/ros2_ws/src/mpc_pendulum/lib/nlpsol_swingup.c`がコード生成されます。
ホライゾンやコストなどを変更してみでください。

2. 生成されたCコードをコンパイルします。
```
cd ~/ros2_ws/src/mpc_pendulum/lib
bash compile.bash
```

3. （ソルバーオプション・ホライズン・制約条件を変更する場合には必要）MPC組み込み用コードを編集し、再ビルドします。
```
cd ~/ros2_ws/src/mpc_pendulum/src
gedit do_experiment_node.cpp
cd ~/ros2_ws
colcon build
```

4. 実行します（装置が動きます）。
```
source ~/ros2_ws/install/setup.bash
ros2 run mpc_pendulum do_experiment_node
```
