# Drone–Bird Control UI

Raspberry Pi 5, Hailo-8 ve Raspberry Pi Global Shutter Camera için iki
sınıflı `DRONE`/`BIRD` tespit, takip ve pan/tilt kontrol arayüzüdür.

## Özellikler

- Yerel `yolo11s.hef` modeliyle Hailo-8 inference
- `DRONE` ve `BIRD` için bağımsız ByteTrack durumları
- DRONE öncelikli aktif hedef seçimi
- Bütün doğrulanmış hedeflerde kırmızı kutu
- Yalnız aktif hedef için kırmızı merkez çizgisi ve telemetri
- `640×480 @ 40 FPS` kamera hattı
- Piksel ve normalize hedef hatası
- Config ile ayarlanabilen merkez kilit toleransı
- STM32 için signed `int16` UART hedef paketi
- Canlı kamera ve video dosyası desteği
- Düşük gecikmeli, sınırlı ve leaky GStreamer kuyrukları

## Donanım

- Raspberry Pi 5 8 GB
- Hailo-8 26 TOPS
- Raspberry Pi Global Shutter Camera (IMX296)
- 16 mm lens
- İsteğe bağlı STM32 pan/tilt denetleyicisi

## Model

Proje modeli:

```text
models/yolo11s.hef
```

Model Hailo-8 için derlenmiştir; `640×640×3` giriş ve iki sınıflı Hailo NMS
çıkışı üretir. Sınıf sırası `config/labels.json` içinde `DRONE`, `BIRD`
olarak tanımlıdır.

## Ayarlar

Bütün çalışma ayarları `config/app.toml` dosyasındadır:

- Model ve labels yolları
- Kamera çözünürlüğü, FPS ve aynalama
- Confidence ve ByteTrack eşikleri
- Sınıf önceliği
- Kilit toleransı
- UART portu, baudrate ve X yönü

## Aktif hedef politikası

1. Kullanıcının elle seçtiği görünür hedef korunur.
2. Görünür aktif DRONE ID'si korunur.
3. DRONE varsa merkeze en yakın DRONE seçilir.
4. DRONE yoksa merkeze en yakın BIRD seçilir.
5. Çizgi, hata ve UART yalnız aktif hedef için üretilir.

## UART protokolü

Paket biçimi little-endian 5 bayttır:

```text
<Bhh
```

- `0xFF`: hedef takip ediliyor
- `0xFE`: hedef kilit toleransı içinde
- `hata_x`: signed int16
- `hata_y`: signed int16

Hedef bulunmadığında `0xFF, 0, 0` gönderilir. UART kullanmadan güvenli test
için uygulama `DRONE_BIRD_DISABLE_UART=1` ortam değişkeniyle başlatılabilir.

## Çalıştırma

```bash
cd /home/raspberrypi/Desktop/hailo-workspace/drone-bird-control-ui
./run.sh
```

`run.sh`, Hailo Apps ortamını yükler ve native overlay eklentisini gerektiğinde
otomatik olarak derler.

## Test

```bash
cd /home/raspberrypi/Desktop/hailo-workspace/hailo-apps
source setup_env.sh
cd ../drone-bird-control-ui
PYTHONPATH="$PWD/src:$PYTHONPATH" python -m unittest discover -s tests -v
```

UART kapalı kamera smoke testi:

```bash
DRONE_BIRD_DISABLE_UART=1 SMOKE_SECONDS=8 \
PYTHONPATH="$PWD/src:$PYTHONPATH" python tests/hardware_smoke.py
```
