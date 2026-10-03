# ESP32-CAM Kurulum ve İlk Deneme Rehberi

Bu belge, projenin ESP32-CAM tarafının gerçek bir kart üzerinde nasıl kurulacağını ve nasıl deneneceğini adım adım anlatır.

> **Bu sürümün kapsamı (v2.2, parçalı tarama):** Drone 25–30 m'den arama yapacak. Bu irtifada bir insan 96×96'lık görüntüde yaklaşık 1,5 piksel kaplar. Bu yüzden kamera artık **320×240** veriyor ve model her karenin **birbirine binen on iki 96×96 parçasına** sırayla bakıyor (hepsinin üzerinden bir geçiş = bir "tarama"). Aynı firmware hem **AI-Thinker ESP32-CAM** (geliştirme, yavaş tarama) hem de **ESP32-S3 kamera kartı** (uçuş hedefi) için derlenir. `model_data.c` içindeki model hâlâ **sentetik** veriyle eğitildi: zincirin çalıştığını gösterir ve kartta gerçek süre ölçümü verir, ama uçuşa uygun değildir. Kart **SHADOW** modunda açılır: `$EVT`/`$ACT` göndermez, yalnızca `$SHD` gönderir.

Bu denemede şunlar doğrulanacak:

- Kodun derlenmesi ve karta yüklenmesi
- Kameranın 320×240 gri kare vermesi
- Modelin kartta çalışması (kare başına süre, bellek)
- Uçuş kontrolcüsüyle UART bağlantısının iki yönde çalışması

---

## 1. Gerekenler

### Donanım

| Parça | Açıklama |
|---|---|
| ESP32-CAM (AI-Thinker) **ya da** ESP32-S3 kamera kartı | ESP32-CAM: OV2640, 4 MB PSRAM. ESP32-S3: kamera bağlantılı, 8 MB PSRAM'li kart (`main/board.h` içindeki pinler Freenove ESP32-S3-WROOM CAM / ESP32-S3-EYE dizilimine göre; başka kartta yalnızca o blok değişir) |
| ESP32-CAM-MB programlama kartı | ESP32-CAM'in altına takılan USB kartı. Yoksa bir USB-TTL (FTDI) dönüştürücü de olur |
| USB kablosu | Veri aktarabilen, kısa ve kaliteli bir kablo |
| İkinci bir USB-TTL dönüştürücü | Masada uçuş kontrolcüsü rolünü oynamak için (`training/fake_fc.py`) |

### Yazılım

- ESP-IDF v6.0.x ve VS Code ESP-IDF eklentisi
- Masa denemesi için `pyserial` kurulu Python 3

---

## 2. Proje Yapısı

```
esp32_camera_scripts/
├── CMakeLists.txt          Proje tanımı
├── sdkconfig.defaults      Ortak: 240 MHz işlemci, PSRAM, watchdog, optimizasyon
├── sdkconfig.defaults.esp32     ESP32-CAM: 4 MB flash
├── sdkconfig.defaults.esp32s3   ESP32-S3: 8 MB flash, octal PSRAM
├── KURULUM.md              Bu belge
├── SETUP.md                Bu belgenin İngilizcesi
├── test/test_host.c        proto/tracker/policy/scan birim testleri (bilgisayarda gcc ile)
└── main/
    ├── CMakeLists.txt      Kaynak listesi
    ├── idf_component.yml   Dış kütüphaneler (kamera, TFLite Micro + ESP-NN)
    ├── config.h            Bütün ayarlar (saf C, ESP-IDF include'u yok)
    ├── main.c              app_main + görüş görevi (kamera -> model -> takip -> politika)
    ├── board.h             Karta göre kamera ve UART pinleri (derleme hedefi seçer)
    ├── camera.c / .h       320x240 gri kamera, kurtarma
    ├── thermal.c / .h      Termal kamera: üçüncü görüntü kanalının karesi (sensör sürücüsü henüz yok)
    ├── scan.c / .h         Parça geometrisi ve tarama sonucu (durum başına en iyi parça)
    ├── fc_uart.c / .h      UART: $HB gönderme, $TEL/$ACK/$CMD alma
    ├── proto.c / .h        Satır biçimi, sağlama toplamı, ayrıştırma, telemetri özellikleri
    ├── infer.cpp / .h      TFLite Micro sarmalayıcı: önceki kare, h hafızası
    ├── tracker.c / .h      Durum başına NORMAL / ADAY / AKTİF / SOĞUMA
    ├── policy.c / .h       Tepkiler: $ACK gelene kadar $EVT, sonra $ACT / $TGT, sonra done
    ├── situations.c / .h   Durum tablosu: her sınıf için eşikler ve tepki
    ├── model_data.c / .h   int8 model (training/export_c.py üretir)
    ├── model_meta.h        Sınıf adları, sıcaklık, durum boyutu (üretilir)
    ├── collect.c / .h      Veri toplama sürümü: kare + telemetri Wi-Fi UDP ile
    ├── wifi.c / .h         Wi-Fi (yalnızca veri toplama sürümü)
    └── util.h              now_ms()
```

---

## 3. ESP32 Ayarları

Bütün ayarlar `main/config.h` içinde. İlk deneme için bir şey değiştirmen gerekmiyor. İleride değiştireceklerin:

| Ayar | Anlamı |
|---|---|
| `CAM_MOUNT` | `CAM_MOUNT_DOWN` (kamera aşağı bakar, görüntünün üstü = burun) ya da `CAM_MOUNT_FORWARD` |
| `VISION_MODE_DEFAULT` | İlk uçuşlarda `VISION_MODE_SHADOW`; FC `$CMD,MODE,n` ile değiştirebilir |
| `COLLECT_MODE` | `1` = veri toplama sürümü (Wi-Fi yayını), `0` = uçuş sürümü |
| `WIFI_SSID`, `WIFI_PASS`, `COLLECT_HOST_IP` | Yalnızca veri toplama sürümü için. Gerçek değerleri `config.h`'ye yazma (aşağıya bak) |
| `LAND_FINAL_ALT_M` | İniş işareti: drone bu irtifaya kadar konumunu düzelterek alçalır, sonra iner |
| `CAM_GROUND_PER_ALT` | Uçuş yönünde görülen zemin uzunluğu ÷ irtifa. Kendi lensin için ölç (8. bölüm) |

> **Wi-Fi şifresi:** `main/wifi_secret.h` adında bir dosya aç ve içine kendi üç satırını yaz (`#define WIFI_SSID "..."`, `#define WIFI_PASS "..."`, `#define COLLECT_HOST_IP "..."`). Bu dosya varsa `config.h` onu kullanır; `.gitignore` dosyayı git'in dışında tutar, yani şifre GitHub'a gitmez.

---

## 4. Derleme

1. VS Code'da **File → Open Folder** ile `esp32_camera_scripts` klasörünü aç.
2. Ctrl+Shift+P → **ESP-IDF: Select Current ESP-IDF Version** → kurulu sürümü seç.
3. Ctrl+Shift+P → **ESP-IDF: Set Espressif device target** → ESP32-CAM için **esp32**, S3 kartı için **esp32s3**. Hedef, pin dizilimini ve kart ayarlarını seçer.
4. **Eski `sdkconfig` dosyasını sil** (`sdkconfig.defaults*` dosyalarındaki ayarlar yalnızca yeni bir `sdkconfig` dosyasına uygulanır). Hedefi her değiştirdiğinde bunu tekrarla.
5. Ctrl+Shift+P → **ESP-IDF: Build your project**.

İlk derleme birkaç dakika sürer: kamera kütüphanesi, TensorFlow Lite Micro ve ESP-NN `managed_components/` klasörüne iner.

---

## 5. Karta Yükleme

### 5.1. Bağlantı

**ESP32-CAM-MB ile:** ESP32-CAM'i MB kartına tak (kamera ve MB kartındaki USB girişi aynı yöne bakmalı), USB kablosunu bilgisayara bağla. Çoğu MB kartı yükleme moduna kendiliğinden geçer.

**USB-TTL (FTDI) ile:**

| USB-TTL | ESP32-CAM |
|---|---|
| 5V | 5V |
| GND | GND |
| TX | U0R (GPIO3) |
| RX | U0T (GPIO1) |

Yüklemeden önce **IO0 pinini GND'ye bağla** ve RST düğmesine bas. Yükleme bitince IO0–GND bağlantısını çıkar ve tekrar RST'ye bas.

### 5.2. Portu seç ve yükle

1. Aygıt Yöneticisi'nde **Bağlantı noktaları (COM ve LPT)** altında kartın hangi COM portunu aldığına bak. Görünmüyorsa CH340 (ya da CP2102) sürücüsünü kur.
2. Ctrl+Shift+P → **ESP-IDF: Select Port to Use** → bu portu seç.
3. Ctrl+Shift+P → **ESP-IDF: Build, Flash and Monitor**.

Monitörden çıkmak için **Ctrl+]**. Yükleme "Failed to connect to ESP32" hatasıyla durursa: **IO0** düğmesini basılı tut, **RST**'ye bir kez bas, IO0'ı bırak, yüklemeyi tekrar başlat.

---

## 6. Deneme Adımları

### Deneme 1: Açılış, kamera, model

Seri monitörde şunları görmelisin:

```
I (...) main: board: AI-Thinker ESP32-CAM, model: dronelaya-v21-synthetic-...
I (...) main: Camera OK: 320x240, 76800 bytes
I (...) infer: ready: arena ..... / 81920 B, 2 image channels, img fast path 1, h in ... out ...
I (...) main: sweep 1200 ms (0.8/s), invoke 98000 us avg, best=marker p=0.04 tile 7, tel age 4294967295 ms
I (...) main: speed 2.0 m/s, limit 2.1 m/s at 15 m          (yalnızca telemetri gelince, Deneme 3)
```

Bu satırlardan üç sayıyı not et; bunlar modelin kart üzerindeki ilk gerçek ölçümleridir:

- **arena used** (bayt): 81920'ın çok altındaysa `infer.cpp` içindeki `INFER_ARENA_BYTES` değerini kullanılan + 8 KB civarına indir.
- **invoke** (parça başına µs): ESP32-CAM'de yaklaşık 100 ms, ESP32-S3'te çok daha az beklenir (henüz ölçülmedi). Çok yavaşsa işlemcinin 240 MHz'de çalıştığını kontrol et (`sdkconfig`'i sil, yeniden derle).
- **sweep** (12 parçanın tamamı için ms): drone'un ne kadar hızlı uçabileceğini bu belirler. İki tarama arasında zemin en çok yarım parça (48 piksel) kayabilir; böylece bir hedef kareden geçerken yaklaşık beş kez görülür. 28 m'de bu, tarama başına 4,8 m eder: 0,2 s'lik taramada 24 m/s, 1,2 s'lik taramada 4 m/s. Kart bu sınırı o anki hızın yanında yazar ve drone sınırın üstündeyse `TOO FAST` uyarısı verir. Eğitim verisini hazırlarken `--gap-ms` için ölçtüğün tarama süresini kullan.

`tel age 4294967295` henüz telemetri gelmediği anlamına gelir; Deneme 3'e kadar beklenen budur.

### Deneme 2: Uçuş kontrolcüsü olmadan kalp atışı

İkinci dönüştürücünün **RX**'ini **GPIO14**'e, **GND**'sini GND'ye bağla, COM portunu 115200 baud ile aç. Saniyede 5 satır görmelisin:

```
$HB,120,NOTEL,89*..
```

`NOTEL` = model çalışıyor ama telemetri yok, bu yüzden karar verilmiyor. `LOST` model çalışmıyor demektir. Son sayı saniyedeki tarama sayısı × 10'dur.

**Uçuş kontrolcüsü için kural:** `LOST` ya da `NOTEL` görevi hiçbir zaman durdurmaz. FC yalnızca görüşü yok sayar. (Bu rehberin eski sürümü FC'nin `LOST`'ta beklemeye geçmesini söylüyordu; bu yanlıştı: ESP32'nin her yeniden başlaması bütün görevi dondururdu.)

### Deneme 3: Sahte uçuş kontrolcüsüyle tam bağlantı

Dönüştürücünün **TX**'ini de **GPIO15**'e bağla. Sonra:

```
pip install pyserial
python training/fake_fc.py COM5 --mode 1
```

Betik 10 Hz'de `$TEL` ve `$CMD,MODE,1` gönderir. Bir saniye içinde kalp atışı `$HB,...,OK,...` olmalı ve monitördeki `tel age` 500 ms'nin altına inmeli. Kamerayı farklı şeylere çevir: SHADOW modunda olacak her olay `$SHD,<durum>,<güven>` olarak görünür. `--mode 2 --ack` ile betik gerçek FC betiği gibi davranır: önce `$EVT`, sonra `$ACT`/`$TGT` satırları, sonra `$EVT,<n>,done` görürsün.

### Bilgisayarda birim testleri (kart gerekmez)

```
cd test
gcc -std=c11 -Wall -Wextra -I../main test_host.c ../main/proto.c ../main/tracker.c ../main/policy.c ../main/scan.c -lm -o test_host
./test_host
```

Beklenen: `127 checks, 0 failed`.

---

## 7. Sık Karşılaşılan Sorunlar

| Belirti | Olası neden | Çözüm |
|---|---|---|
| `Brownout detector was triggered` ve sürekli yeniden başlama | Besleme yetersiz | Daha kısa, kaliteli USB kablosu, başka USB girişi. Drone'da ayrı 5V BEC ve 470–1000 µF kondansatör (`warning.txt`) |
| `Camera probe failed` / `camera init failed` | Kamera kablosu oturmamış ya da PSRAM kapalı | Şerit kabloyu kilidiyle düzgünce tak. `CONFIG_SPIRAM=y` olduğunu kontrol et |
| `AllocateTensors failed` | Arena küçük ya da resolver'da eksik op | `INFER_ARENA_BYTES` değerini artır; `model_meta.h` içindeki op listesini `infer_init()` ile karşılaştır |
| `model tensors do not match model_meta.h` | `model_data.c` ve `model_meta.h` farklı dışa aktarmalardan | `export_c.py`'yi yeniden çalıştır; ikisini birlikte yazar |
| `situations.c row N ... != model class ...` | Durum tablosu modelin sınıflarıyla uyuşmuyor | `situations.c` içindeki adları `MODEL_CLASS_NAMES` ile aynı sıraya koy |
| invoke beklenenden çok yavaş | 160 MHz işlemci (eski `sdkconfig`) | `sdkconfig`'i sil, yeniden derle |
| `no PSRAM for the previous frame` | PSRAM algılanmadı (S3 modülü için yanlış PSRAM modu) | `sdkconfig.defaults.esp32s3` içinde `CONFIG_SPIRAM_MODE_OCT` yerine `CONFIG_SPIRAM_MODE_QUAD` yaz, `sdkconfig`'i sil, yeniden derle |
| S3: `Camera probe failed` | Pin dizilimi karta uymuyor | `main/board.h` içindeki ESP32-S3 bloğunu düzenle |
| Yüklemede `Failed to connect to ESP32` | Kart yükleme modunda değil | 5.2'nin sonundaki IO0/RST adımları |

---

## 8. Sonraki Adımlar

1. **Kartta ölç** (Deneme 1) ve sayıları mimari belgesine yaz.
2. **Piksel ölçeğini ölç:** kamerayı bilinen bir yükseklikten dümdüz aşağı çevir, yere boyu bilinen bir cisim koy, kaydet (`COLLECT_MODE 1`, `training/recorder.py`) ve cismin kaç piksel tuttuğunu say. Piksel başına zemin = boy ÷ piksel; `CAM_GROUND_PER_ALT` = piksel başına zemin × 240 ÷ yükseklik. Değeri `config.h`'ye ve `training/tiles.py`'ye (`GROUND_PER_ALT`) yaz.
3. **İlk modeli indirdiğin verisetiyle eğit:** `training/README.md` (prepare_data → train → evaluate → export_c). `export_c.py`; `model_data.c`, `model_meta.h` ve `situations.c` dosyalarını yazar.
4. **Gerçek veri topla:** `COLLECT_MODE 1` yap, `training/recorder.py`'yi çalıştır, uç. Olayları etiketle, uçuşları eğitim verisine ekle.
5. **Termal kamera:** `v22` olarak eğitilen modelin termal kare için üçüncü bir görüntü kanalı var. Firmware tarafı hazır (`thermal.h` kare biçimini anlatır, `scan_thermal_tile()` kareyi parçalara böler) ama `thermal.c` içinde henüz sensör sürücüsü yok: o zamana kadar kayıtta `thermal: model wants it, no sensor` yazar ve model yalnızca kamerayla çalışır. Sürücü ESP32-S3 kartı ister (ESP32-CAM'de boş pin yok).
6. **Uçuş kontrolcüsü:** drone INAV kullanacak. `fc/dronelaya.lua`, FC tarafının ArduPilot için yazılmış örneğidir (INAV'da çalışmaz); INAV bağdaştırıcısı henüz yazılmadı.
7. **Shadow uçuşları:** her `$SHD`'yi incele, yanlış alarmları "normal" olarak eğitime ekle, yeniden eğit. Shadow uçuşları saatte ≤ 1 yanlış olay gösterince ACTIVE moda geç.
