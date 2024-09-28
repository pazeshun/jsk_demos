# Camera Sync

### Prerequirements

Install udev to give permission to the device.

```
curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules | sudo tee /etc/udev/rules.d/99-platformio-udev.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Ubuntu/Debian users may need to add own “username” to the “dialout” group if they are not “root”, doing this issuing

```
sudo usermod -a -G dialout $USER
sudo usermod -a -G plugdev $USER
```

```
pip3 install platformio -U
```

## 書き込み

カメラを同期するには以下。
このとき他のUSBがついた機器はすべて外してarduino nano everyだけと接続する。

```
pio run -t upload -e camera_sync
```

カメラタイミングをずらす場合には以下。

```
pio run -t upload -e camera_async
```

Asyncの場合にはarduino nano everyのUSBポートの左横のオレンジのLEDが点灯しない。

