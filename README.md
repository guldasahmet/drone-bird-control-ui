# Drone Bird Control UI

Raspberry Pi 5 ve tek Hailo-8 üzerinde iki CSI kamerayı aynı süreçte çalıştıran GTK arayüzü.
Güncel arayüz telefonu (`cell phone`) izler. IMX477 HQ kamera başlangıçta kontrolü alır;
IMX296 Global Shutter aynı telefonu iki ardışık doğrulanmış karede gördüğünde kontrol
ona geçer. Global Shutter hedefi beş kare kaybederse kontrol HQ'ya döner. İki kamera
görüntüsü de bu sırada açık kalır.

## Güncel çalışma yolu

```text
IMX296 + IMX477 → Picamera2 → appsrc → hailoroundrobin → hailonet
  → hailofilter → hailostreamrouter → iki görüntü paneli
```

Telefon profili `/usr/share/hailo-models/yolov8s_h8.hef` modelini ve
`config/phone_labels.json` hedef listesini kullanır. Her kamera için ayrı ByteTrack
durumu tutulur. Varsayılan görüntü çıkışı TigerVNC ile uyumlu `appsink → GTK`
yoludur. Kamera 0 IMX296, kamera 1 IMX477 olmalıdır; uygulama bu eşleşmeyi açılışta
kontrol eder. Çözünürlük 640×640, hedef kare hızı kamera başına 30 FPS'tir.

```bash
cd /home/spikeedge/Desktop/hailo-workspace/drone-bird-control-ui
./run.sh
```

`run.sh`, yanındaki `hailo-apps/setup_env.sh` ortamını yükler ve hedef çizgisi için
`native/` eklentisini gerektiğinde derler. UART varsayılan olarak kapalıdır;
`./run.sh --uart` ile açılır. Telefon profilinde görüntü merkezine göre hesaplanan
X ve Y hatalarının işaretleri STM'ye gönderilirken ters çevrilir. Paket 5 bayttır:
`<Bhh`; `0xFF` takip, `0xFE` kilit anlamına gelir. Hedef yoksa `0xFF, 0, 0`
gönderilir.

Arayüz F11 ile tam ekran olur. Görüntü sorunu tanısı için
`./run.sh --display-backend wayland` eski görüntü çıkışını seçer. Terminalde kamera,
Hailo sonrası hat ve ekrana çizilen kare hızları yazılır.

## Dosyalar

```text
config/phone_labels.json     Telefon hedef filtresi
native/                      Çift kamera hedef çizgisi eklentisi
src/dual_app.py              GTK ekranı
src/dual_camera_pipeline.py  Çift kamera ve Hailo akışı
src/dual_runtime.py          Telefon takibi, kamera devri, UART
src/tracking.py              İki kameranın kullandığı ByteTrack kodu
src/uart.py                  STM paket kodu
tests/                       Çift kamera birim ve donanım testleri
run.sh                       Çalıştırma betiği
```

## Test

```bash
cd /home/spikeedge/Desktop/hailo-workspace/hailo-apps
source setup_env.sh
cd ../drone-bird-control-ui
PYTHONPATH="$PWD/src:$PYTHONPATH" python -m unittest discover -s tests -p 'test_*.py'
```

UART kapalı kısa donanım testi:

```bash
./native/build.sh
GST_PLUGIN_PATH="$PWD/native/build:${GST_PLUGIN_PATH:-}" \
SMOKE_SECONDS=8 PYTHONPATH="$PWD/src:$PYTHONPATH" \
python tests/dual_hardware_smoke.py
```

`videos/`, loglar ve derleme çıktıları Git'e eklenmez.
