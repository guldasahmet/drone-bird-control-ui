# Drone–Bird Control UI

## Güncel çift kamera arayüzü

`./run.sh`, IMX296 Global Shutter ve IMX477 HQ kameralarını aynı süreçte,
tek Hailo-8 üzerinde çalıştırır. Başlangıç kontrolü HQ'dadır; Global Shutter
telefonu gördüğünde kontrol ona geçer ve beş kayıp kareden sonra HQ'ya döner.
İki kamera bu sırada da sürekli görüntü üretir. GTK ekranı iki canlı görüntüyü,
hedefleri, kamera devrini ve kamera/Hailo FPS değerlerini gösterir.

İlk çalışma profili `/usr/share/hailo-models/yolov8s_h8.hef` ile yalnızca
`cell phone` hedefidir. Hedef etiketi `config/phone_labels.json` içindedir.
Depodaki iki sınıflı `models/yolo11s.hef` DRONE/BIRD modeli korunur; bu profil
çift kamera arayüzüne henüz bağlanmadı.

```bash
cd /home/spikeedge/Desktop/hailo-workspace/drone-bird-control-ui
./run.sh
```

`./run.sh` UART'ı açmaz. STM ile bilinçli deneme için `./run.sh --uart`
kullanılır. STM hız yanıtı vermesine rağmen motor hareketi henüz
doğrulanmadığından motor çalışması arayüz testiyle kanıtlanmış sayılmaz.
İki kamera da 640×640 @30 FPS hedefiyle çalışır; gerçek FPS ekranda ölçülür.
F11 tam ekranı açıp kapatır.

Akış: `Picamera2 ×2 → appsrc (otomatik timestamp) → hailoroundrobin →
hailonet → hailofilter → hailostreamrouter → iki GTK görüntüsü`. Kontrol
kamerası değişirken görüntü kolları durdurulmaz. Kamera 0 için IMX296,
kamera 1 için IMX477 doğrulanır; sıra değişmişse uygulama açık hata verir.

`src/dual_app.py` güncel ekran, `src/dual_runtime.py` hedef ve devir mantığı,
`src/dual_camera_pipeline.py` görüntü hattıdır. Eski tek kamera ekranı
`src/app.py`, DRONE/BIRD ayarları `config/app.toml` ve modeli yerinde durur.

## Eski tek kamera uygulamasının kayıtları

Aşağıdaki bilgiler önceki DRONE/BIRD tek kamera uygulamasını anlatır;
`./run.sh` artık yukarıdaki çift kamera ekranını açar.

## Donanım

- Raspberry Pi 5 8 GB
- Hailo-8 26 TOPS
- Raspberry Pi Global Shutter Camera (IMX296)
- 16 mm lens
- İsteğe bağlı STM32 pan/tilt denetleyicisi

## Temel özellikler

- Projeye dahil iki sınıflı `yolo11s.hef` modeli
- DRONE ve BIRD için ayrı ByteTrack durum makineleri
- DRONE öncelikli aktif hedef seçimi
- Görünür aktif DRONE ID'sini koruma
- Tüm doğrulanmış hedeflerde kırmızı kutu
- Yalnız aktif hedefte kırmızı merkez çizgisi
- Piksel ve normalize X/Y hedef hatası
- Config ile ayarlanan merkez kilit toleransı
- Confidence değerini arayüzden değiştirme
- `640×480 @ 40 FPS` canlı kamera hattı
- Canlı kamera ve video dosyası desteği
- STM32 için 5 baytlık signed hedef paketi
- Düşük gecikmeli GStreamer kuyrukları ve native Wayland görüntüleme

## Sistem akışı

```text
IMX296 / video dosyası
        │
        ▼
GStreamer RGB pipeline
        │
        ▼
Hailo-8 YOLO11s inference
        │
        ▼
DRONE / BIRD sınıf filtresi
        │
        ├── DRONE ByteTrack
        └── BIRD ByteTrack
                 │
                 ▼
Aktif hedef politikası → overlay / arayüz / UART
```

Arayüz thread'i video karesi taşımaz. Kamera, inference, metadata takibi,
native overlay ve Wayland sunumu ayrı GStreamer aşamalarında çalışır. Sınırlı
ve leaky kuyruklar eski karelerin birikerek pan/tilt gecikmesi oluşturmasını
önler.

## Model

Model proje içinde bulunur:

```text
models/yolo11s.hef
```

Doğrulanan model özellikleri:

- Hailo-8 mimarisi
- `640×640×3` giriş
- İki sınıflı Hailo NMS çıkışı
- Sınıf sırası: `DRONE`, `BIRD`

Sınıf isimleri `config/labels.json` içinde tanımlıdır. Uygulama başlangıçta
HEF sınıf sayısı ile labels sırasının uyumunu doğrular.

## Aktif hedef politikası

Hedef seçimi aşağıdaki sırayla yapılır:

1. Kullanıcının tablodan elle seçtiği görünür hedef
2. Görünür durumdaki mevcut aktif DRONE ID'si
3. Merkeze en yakın DRONE
4. DRONE yoksa merkeze en yakın BIRD

Bütün doğrulanmış hedefler görüntülenir; kırmızı çizgi, hata telemetrisi,
kilit durumu ve UART yalnız aktif hedef için üretilir. Güncel karede görülmeyen
nesneye tahmini kutu çizilmez.

## Hata hesabı

`640×480` görüntünün merkezi `(320, 240)` pikseldir:

```text
hata_x = hedef_merkez_x - 320
hata_y = hedef_merkez_y - 240

hata_x_norm = hata_x / 320
hata_y_norm = hata_y / 240
```

Görüntü koordinatında sağ ve aşağı pozitiftir. STM32'ye gönderilen X işareti
`config/app.toml` içindeki `invert_x` ayarıyla pan mekanizmasına uyarlanır.

## UART protokolü

Paket little-endian 5 bayttır:

```text
<Bhh
```

| Alan | Tür | Açıklama |
|---|---|---|
| Header | `uint8` | `0xFF`: takip, `0xFE`: kilit |
| hata_x | `int16` | Pan ekseni hatası |
| hata_y | `int16` | Tilt ekseni hatası |

Hedef bulunmadığında `0xFF, 0, 0` gönderilir. UART kullanmadan güvenli test
için `DRONE_BIRD_DISABLE_UART=1` kullanılabilir.

## Yapılandırma

Bütün çalışma ayarları `config/app.toml` dosyasındadır:

```toml
[model]
path = "models/yolo11s.hef"
labels = "config/labels.json"
expected_classes = 2

[video]
width = 640
height = 480
camera_fps = 40
mirror_x_axis = true

[tracking]
classes = ["DRONE", "BIRD"]
priority = ["DRONE", "BIRD"]
confidence = 0.30
lock_tolerance_px = 25

[uart]
enabled = true
port = "/dev/ttyACM0"
baudrate = 115200
invert_x = true
```

## Proje yapısı

```text
drone-bird-control-ui/
├── models/       # İki sınıflı HEF modeli
├── config/       # TOML ayarları ve labels
├── src/          # GTK, runtime, tracking, UART ve ayar kodu
├── native/       # Native GStreamer overlay
├── tests/        # Core ve donanım smoke testleri
├── run.sh
└── README.md
```

## Çalıştırma

Hailo Apps çalışma alanı projenin yanında bulunmalıdır:

```text
hailo-workspace/
├── hailo-apps/
└── drone-bird-control-ui/
```

Uygulamayı başlatmak için:

```bash
cd /home/raspberrypi/Desktop/hailo-workspace/drone-bird-control-ui
./run.sh
```

`run.sh`, Hailo ortamını yükler, Python yolunu ayarlar ve native overlay'i
gerektiğinde otomatik olarak derler.

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

Native overlay'i elle derlemek için:

```bash
./native/build.sh
```
