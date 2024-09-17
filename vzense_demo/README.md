# Vzense カメラキャリブレーションデモ

## 前準備

1. `MC-Calib` リポジトリをクローン：
   ```bash
   git clone https://github.com/rameau-fr/MC-Calib ~/MC-Calib
   ```

2. キャリブレーション用のDockerイメージを取得する：

```bash
docker pull bailool/mc-calib-prod
```

キャリブレーション環境をビルドする：

```bash
docker run -ti --rm \
    --volume="$HOME/MC-Calib:/home/MC-Calib" \
    --volume="$HOME/MC-Calib/data:/home/MC-Calib/data" \
    bailool/mc-calib-prod bash -c "cd /home/MC-Calib && mkdir build && cd build && cmake -DCMAKE_BUILD_TYPE=Release .. && make -j10"
```

```
mkdir -p ~/ros/vzense/src/
cd ~/ros/vzense/src/
git clone https://github.com/Vzense/NebulaSDK
git clone --single-branch https://github.com/iory/jsk_demos -b vzense
cd ~/ros/vzense/src/NebulaSDK/ROS2
touch CATKIN_IGNORE
cd ~/ros/vzense/src/NebulaSDK/ROS/src/VzenseROS
python install.py Ubuntu18.04
cd ~/ros/vzense/src/NebulaSDK/ROS/src/VzenseROS_MultiCameras
python install.py Ubuntu18.04
cd ~/ros/vzense
rosdep update
rosdep install --from-paths -i -y -r .
catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCATKIN_WHITELIST_PACKAGES="VzenseROS" -DCATKIN_WHITELIST_PACKAGES="VzenseROS_MultiCameras"
```

## キャリブレーションボードの準備

キャリブレーションボードがない場合、以下の画像を印刷し、金属などの曲がりにくい面に貼り付ける：

![Calibration Board](./calib_board/charuco_board_000.bmp)


## 2つのカメラでチェッカーボードの画像を集める

1. 2台のVzenseカメラを接続し、IPの設定を行う。

2. 以下のコマンドを実行して、カメラインターフェースを立ち上げる：

```bash
roslaunch vzense_demo calib_vzense.launch
```

3. RVizを使用し、両方のカメラにキャリブレーションボードが映るように調整して、複数の画像を撮影する。

## キャリブレーションの実行

1. 撮影した画像をキャリブレーション用のフォーマットに変換：

```
rosrun vzense_demo aggregate_calib_images.py
```

2. キャリブレーションを実行：

```
docker run -ti --rm \
    --volume="$HOME/MC-Calib:/home/MC-Calib" \
    --volume="$HOME/MC-Calib/data:/home/MC-Calib/data" \
    --volume="$(rospack find vzense_demo)/config/calib_param_two_vzense.yaml:/home/MC-Calib/configs/calib_param_two_vzense.yaml" \
    --volume="$(rospack find vzense_demo)/calib_results:/home/MC-Calib/data/vzense_data" \
    bailool/mc-calib-prod bash -c "cd /home/MC-Calib/build && ./apps/calibrate/calibrate ../configs/calib_param_two_vzense.yaml"
```

## キャリブレーション結果を反映

```
rosrun vzense_demo set_calib_tf.py
```



















