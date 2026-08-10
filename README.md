# Cell Phone Tracking Control UI

Raspberry Pi 5, Hailo-8 ve Raspberry Pi Global Shutter (IMX296) kamera için
telefon algılama, ByteTrack takibi ve pan/tilt hata telemetrisi arayüzüdür.
Klasörün eski adı korunmuştur; çalışma profili yalnız `CELL PHONE` hedefidir.
Savaşan İHA projesinden tamamen ayrıdır.

## Çalıştırma

```bash
cd /home/raspberrypi/Desktop/hailo-workspace/bird-drone-control-ui
./run.sh
```

Varsayılan resmi Hailo modeli:

```text
/usr/share/hailo-models/yolov8s_h8.hef
```

Model `640×640` girişli ve 80 COCO sınıflıdır. Pipeline bütün sınıfları çözer,
fakat diğer 79 sınıf tracking ve overlay aşamasından önce atılır. Yalnız COCO
`cell phone` sınıfı doğrulanır, kutulanır ve aktif hedef olabilir.

Hailo post-process sınıf `0` değerini `unlabeled` için ayırdığı için
`config/coco_labels.json` 81 isim içerir: `unlabeled` ve ardından 80 COCO
sınıfı. Telefon etiketi bu eşlemede Hailo sınıf ID `68` değeridir.

## Davranış

- Tüm doğrulanmış telefonlar kırmızı kutuyla gösterilir.
- Aktif telefonun ID'si görünür olduğu sürece korunur.
- Aktif hedef kaybolursa merkeze en yakın doğrulanmış telefon seçilir.
- Yalnız aktif telefona ince kırmızı merkez çizgisi çizilir.
- Canlı görüntünün merkezinde 5×5 piksel beyaz `×` bulunur.
- Sınıf, confidence, ID ve hata değerleri sağ panel ve hedef tablosundadır.
- Confidence eşiği çalışma sırasında `0.25–0.90` aralığında ayarlanabilir.
- ByteTrack düşük güvenli eşleştirme eşiği bağımsız olarak `0.10` kalır.
- Aynı telefona ait yüksek IoU'lu tekrar kutular güven değerine göre
  tekilleştirilir.
- Kaybolan telefona tahmini veya yalancı kutu çizilmez.

## Hata telemetrisi

```text
dx_norm = dx_px / (görüntü_genişliği / 2)
dy_norm = dy_px / (görüntü_yüksekliği / 2)
```

`dx_norm` sağa, `dy_norm` aşağı doğru pozitiftir. Her eksen yaklaşık
`[-1, +1]` aralığındadır. STM pan/tilt bağlantısı eklendiğinde bu iki normalize
eksen kullanılabilir; seri haberleşme bu sürümde etkin değildir.

## Pipeline mimarisi

- GTK ana thread'i yalnız olayları işler; hedef paneli 10 Hz, sistem
  telemetrisi 1 Hz güncellenir.
- Picamera2 capture thread'i IMX296'dan `BGR888` ister. Pi kamera katmanındaki
  RGB bellek düzeni doğrudan `appsrc`'a gönderilir; Python `cvtColor` yapmaz.
- GStreamer capture, ölçekleme, inference, overlay ve Wayland sunumu için ayrı
  native worker thread'leri kullanır.
- Hailo-8, YOLOv8s inference'ını batch 1 olarak çalıştırır.
- Metadata filtresi yalnız `CELL PHONE` etiketini ByteTrack'e geçirir.
- Aynı sınıf kutuları IoU ile tekilleştirilir ve iki karede doğrulanır.
- Arayüze video karesi değil yalnız küçük, değişmez telemetri snapshot'ları
  aktarılır.
- Kutular Hailo'nun native RGB overlay'iyle çizilir. Merkez ve aktif hedef
  çizgisi `bdtargetoverlay` C++ filtresinde kare kopyalanmadan çizilir.
- `gtkwaylandsink` native Wayland widget'ına render eder; XWayland/GL köprüsü ve
  ayrı Cairo repaint döngüsü yoktur.
- Kamera kuyrukları en fazla bir güncel kare tutar ve geciken eski kareyi atar.
- Video dosyaları inference'dan önce kendi PTS saatine bağlanır.

Gerçek IMX296 + Hailo-8 testinde resmi YOLOv8s modeli `1280×720 @ 40 FPS`
akışta `40.00 FPS`, `%60.7` toplam uygulama CPU'su ve `37.71 ms` ölçülen
pipeline gecikmesi verdi. COCO telefon görüntüsü testinde `CELL PHONE ID=1`,
kırmızı kutu, aktif çizgi ve pixel/normalize telemetri `29.99 FPS` ile
doğrulandı.

## Native overlay

`./run.sh`, native nişangâh filtresini gerektiğinde otomatik derler ve
GStreamer plugin yoluna ekler. Elle derlemek için:

```bash
./native/build.sh
```

Bir subprocess'e 1280×720 RGB kareleri 40 FPS taşımak yaklaşık `110 MB/s` ham
IPC trafiği ve ek kopyalama oluşturur. GStreamer ağır aşamaları zaten native
thread'lere dağıttığı için ayrı görüntü process'i kullanılmaz.

## Kontrol testleri

```bash
cd /home/raspberrypi/Desktop/hailo-workspace/hailo-apps
source setup_env.sh
PYTHONPATH=../bird-drone-control-ui/src:$PYTHONPATH \
  python -m unittest discover -s ../bird-drone-control-ui/tests -v
```
