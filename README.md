# Drone Bird Control UI

Raspberry Pi 5 ve tek Hailo-8 üzerinde iki CSI kamerayı aynı süreçte çalıştıran GTK arayüzü.
Varsayılan `phone` profili telefonu (`cell phone`) izler. IMX477 HQ kamera başlangıçta kontrolü alır;
IMX296 Global Shutter hedefi iki ardışık doğrulanmış karede gördüğünde kontrol
ona geçer. Global Shutter hedefi beş kare kaybederse kontrol HQ'ya döner. İki kamera
görüntüsü de bu sırada açık kalır.

## Güncel çalışma yolu

```text
IMX296 + IMX477 → Picamera2 → appsrc → hailoroundrobin → hailonet
  → hailofilter → hailostreamrouter → iki görüntü paneli
```

Telefon profili sistemdeki `/usr/share/hailo-models/yolov8s_h8.hef` ile aynı içeriğe
sahip `models/phone.hef` dosyasını ve
`config.json` hedef listesini kullanır. DRONE/BIRD profili eski çalışan arayüzdeki
`models/yolo11s.hef` modelini ve `config.json` içindeki `postprocess` etiketlerini kullanır.
Her kamera için ayrı ByteTrack
durumu tutulur. Varsayılan görüntü çıkışı TigerVNC ile uyumlu `appsink → GTK`
yoludur. Kamera 0 IMX296, kamera 1 IMX477 olmalıdır; uygulama bu eşleşmeyi açılışta
kontrol eder. Çözünürlük 640×640, hedef kare hızı kamera başına 30 FPS'tir.

```bash
cd /home/spikeedge/Desktop/hailo-workspace/drone-bird-control-ui
./run.sh
```

`run.sh`, yanındaki `hailo-apps/setup_env.sh` ortamını yükler ve hedef çizgisi için
`native/` eklentisini gerektiğinde derler. UART varsayılan olarak kapalıdır;
`./run.sh --uart` ile açılır. `camera.flip_vertical=true` görüntüyü Hailo'dan önce
üst-alt çevirir. Görsel Y yönü değiştiği için UART tarafında bu değişim telafi edilir;
pan-tilt'e giden fiziksel yön eski çift kamera sürümüyle aynı kalır. Paket 5 bayttır:
`<Bhh`; `0xFF` takip, `0xFE` kilit anlamına gelir. Hedef yoksa `0xFF, 0, 0`
gönderilir.

Arayüz F11 ile tam ekran olur. Görüntü sorunu tanısı için
`./run.sh --display-backend wayland` eski görüntü çıkışını seçer. Terminalde kamera,
Hailo sonrası hat ve ekrana çizilen kare hızları yazılır.

## Dataset kaydı

İki kameranın görüntüsü açıldıktan sonra **DATASET KAYDINI BAŞLAT** düğmesi,
her kameradan saniyede iki kutusuz 640×640 JPEG kaydeder (kalite 92).
Dosyalar `dataset/<UTC oturum zamanı>/cam0_global_shutter/` ve
`cam1_hq/` klasörlerine yazılır; `session.json` kayıt ayarlarını tutar.
Bu kareler kameradan alınan, üst-alt çevirme ve Hailo çiziminden **önceki**
görüntüdür; sensörün Bayer RAW verisi değildir.

JPEG yazıcısı ayrı iş parçacığında ve en fazla sekiz karelik kuyrukta çalışır.
Yazıcı yetişmezse yalnızca kayıt karesi düşer; düşen ve kaydedilen kare sayıları
arayüzde görünür. Kayıt sırasında boş alan 1 GB altına inerse kayıt durur;
kamera ve Hailo akışı çalışmaya devam eder. `session.json`, aktif model profilini de
kaydeder. Uygulama durdurulduğunda kayıt da
kapatılır. `dataset/` Git tarafından yok sayılır.

## Model profili

Proje kökündeki `config.json` içinde `active_profile` adını `phone` veya
`drone_bird` yaparak model seçilir. Normal açılışta `phone` aktiftir.
Arayüzde profil adı, HEF dosyası, hedef başlığı, hedef sınıfları ve başlangıç güven eşiği
aktif profilden okunur. Hedef listesindeki `id`, HEF çıkışındaki sıfırdan başlayan sınıf
numarasıdır; `label` arayüzde görünür. `match_by=label`, çalışan telefon sürümündeki
gibi postprocess etiketini kullanır; `match_by=id` sınıf numarasını kullanır.
`priority` hedef önceliğidir; `sticky`, görünür kaldıkça aynı hedefte kalacak sınıfları
belirtir. Kamera yönü bütün profiller için ortak `camera.flip_vertical` ayarıdır.

Yeni bir algılama modeli için `profiles` altına yeni profil ekleyin ve HEF, uyumlu YOLO
postprocess `.so`, hedef sınıf numaraları, eşikler ve gerekirse `postprocess`
etiketlerini belirtin. Hailo eklentisi ayrı dosya beklediği için bu etiketler çalışma
anında geçici JSON dosyasına yazılır ve akış kapanınca silinir. Kullanılacak HEF ve `.so`
dosyaları Pi üzerinde bulunmalıdır. Başlangıçta
HEF'in tek 640×640×3 girişi ve `[sınıf, 5, kutu]` biçimli tek Hailo NMS çıkışı
doğrulanır; başka çıkış düzenleri bu akışta desteklenmez. Profil değişikliğinden sonra
uygulamayı yeniden başlatın. İsterseniz başka bir dosya `./run.sh --config /yol/config.json`
ile seçilebilir. DRONE/BIRD HEF'i açılıp iki kamera ile denendi; bu modelde toplam
Hailo hızı yaklaşık 44–45 FPS görüldü, dolayısıyla kamera pencereleri her zaman
30+30 FPS göstermeyebilir. Telefon modeliyle toplam yaklaşık 60 FPS görüldü.

## Dosyalar

```text
config.json                  Aktif model, hedef sınıfları ve eşikler
models/yolo11s.hef           İki sınıflı DRONE/BIRD Hailo modeli
models/phone.hef             COCO-80 telefon tespitinde kullanılan Hailo modeli
native/                      Çift kamera hedef çizgisi eklentisi
src/dual_app.py              GTK ekranı
src/dual_camera_pipeline.py  Çift kamera ve Hailo akışı
src/dual_runtime.py          Hedef takibi, kamera devri, UART
src/model_profile.py         Model profili okuyucu ve HEF doğrulama
src/dataset_recorder.py       Arka planda JPEG dataset yazıcısı
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

Dataset kaydını da denemek için komuta `SMOKE_RECORD=1` ve `SMOKE_SECONDS=10`
eklenebilir; bu testin UART'ı kapalıdır.

`dataset/`, loglar ve derleme çıktıları Git'e eklenmez.
