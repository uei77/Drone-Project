# ESP32-CAM Kurulum ve İlk Deneme Rehberi

Bu belge, projenin ESP32-CAM tarafının gerçek bir kart üzerinde nasıl kurulacağını ve nasıl deneneceğini adım adım anlatır.

> **Bu sürümün kapsamı:** Sunucu kaldırıldı. Karar modeli ileride doğrudan ESP32'nin içinde çalışacak; bu sürümde henüz model yok. ESP32 açılışta kamerayı başlatır, bir kare alarak kameranın çalıştığını doğrular ve uçuş kontrolcüsüne kalp atışı gönderir. Karar üreten bir model olmadığı için kalp atışı sürekli `LOST` der; bu beklenen ve güvenli davranıştır.

Bu denemede şunların çalıştığı doğrulanacak:

- Kodun derlenmesi ve karta yüklenmesi
- Kameranın açılması ve görüntü alınması
- Uçuş kontrolcüsüne giden UART çıktısı (kalp atışı)

---

## 1. Gerekenler

### Donanım

| Parça | Açıklama |
|---|---|
| ESP32-CAM (AI-Thinker) | OV2640 kameralı model |
| ESP32-CAM-MB programlama kartı | ESP32-CAM'in altına takılan USB kartı. Yoksa bir USB-TTL (FTDI) dönüştürücü de olur |
| USB kablosu | Veri aktarabilen, kısa ve kaliteli bir kablo |
| İkinci bir USB-TTL dönüştürücü *(isteğe bağlı)* | Uçuş kontrolcüsüne giden UART çıktısını görmek için |

### Yazılım

- ESP-IDF ve VS Code ESP-IDF eklentisi (kurulu)

---

## 2. Proje Yapısı

```
esp32_camera_scripts/
├── CMakeLists.txt          Proje tanımı
├── sdkconfig.defaults      Donanım ve sistem ayarları (PSRAM, watchdog vb.)
├── KURULUM.md              Bu belge
├── SETUP.md                Bu belgenin İngilizcesi
└── main/
    ├── CMakeLists.txt      Derlenecek .c dosyalarının listesi
    ├── idf_component.yml   Dışarıdan inen kütüphaneler (kamera)
    ├── config.h            Bütün ayarlar
    ├── main.c              Başlangıç noktası (app_main)
    ├── wifi.c / .h         Wi-Fi bağlantısı (şu an çağrılmıyor; veri toplama için ayrıldı)
    ├── camera.c / .h       Kamera başlatma ve kurtarma
    └── fc_uart.c / .h      Uçuş kontrolcüsüne UART çıktısı ve kalp atışı
```

---

## 3. ESP32 Ayarları

Bu sürümde doldurman gereken bir ayar yok. `main/config.h` içindeki `WIFI_SSID` ve `WIFI_PASS` değerleri ileride eğitim verisini bilgisayara aktarmak için ayrıldı; şu an kullanılmıyor.

> **Uyarı:** `config.h` dosyasına gerçek Wi-Fi şifreni yazmadan önce, şifrenin git'e gitmemesi için şu komutu çalıştır: `git update-index --skip-worktree scripts/camera_scripts/esp32_camera_scripts/main/config.h`

---

## 4. Derleme

1. VS Code'da **File → Open Folder** ile `esp32_camera_scripts` klasörünü aç.
2. Ctrl+Shift+P → **ESP-IDF: Select Current ESP-IDF Version** → kurulu sürümü seç.
3. Ctrl+Shift+P → **ESP-IDF: Set Espressif device target** → **esp32**.
4. Ctrl+Shift+P → **ESP-IDF: Build your project**.

İlk derleme birkaç dakika sürer. Kamera kütüphanesi bu sırada `managed_components/` klasörüne iner. Derleme başarılı olursa terminalde programın ne kadar yer kapladığını gösteren bir özet çıkar.

> `sdkconfig.defaults` dosyasını değiştirirsen, değişikliğin geçerli olması için oluşan `sdkconfig` dosyasını silip yeniden derlemen gerekir.

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

Yükleme bitince seri monitör kendiliğinden açılır. Monitörden çıkmak için **Ctrl+]** tuşlarını kullan.

Yükleme "Failed to connect to ESP32" hatasıyla durursa: MB kartındaki **IO0** düğmesini basılı tut, **RST**'ye bir kez bas, IO0'ı bırak ve yüklemeyi tekrar başlat.

---

## 6. Deneme Adımları

Her adımda neyi göreceğin aşağıda yazıyor. Bir adım beklendiği gibi çalışmazsa sonrakine geçmeden 7. bölüme bak.

### Deneme 1: Açılış ve kamera

Seri monitörde şuna benzer bir satır görmelisin (çözünürlük ve bayt sayısı sende farklı olabilir):

```
I (...) main: Camera OK: 640x480, 18234 bytes
```

Bu satır, kameranın açıldığını ve gerçekten bir kare aldığını gösterir. Bunun yerine `Camera could not capture a frame` görürsen ESP32 kamerayı yeniden başlatmayı dener; düzelmezse 7. bölüme bak.

### Deneme 2: Kalp atışı

UART çıktısında (bkz. 6.3) saniyede 5 kez `$HB,<sayaç>,LOST*<toplam>` satırı akmalı. Karar modeli henüz olmadığı için durum her zaman `LOST`'tur; uçuş kontrolcüsü bu durumda güvenli modda (hold) kalmalıdır. Model eklendiğinde, model son 2 saniye içinde çalıştıysa durum `OK` olacak.

### 6.3. UART çıktısını izleme

Uçuş kontrolcüsüne giden satırlar **GPIO14** pininden çıkar; seri monitörde görünmezler. Görmenin iki yolu var:

**İkinci bir USB-TTL dönüştürücü ile (önerilen):**

| USB-TTL | ESP32-CAM |
|---|---|
| RX | GPIO14 |
| GND | GND |

Dönüştürücünün COM portunu 115200 baud ile bir seri terminalde aç (PuTTY ya da VS Code'un "Serial Monitor" eklentisi). Şuna benzer satırlar görmelisin:

```
$HB,0,LOST*3E
$HB,1,LOST*3F
$HB,2,LOST*3C
```

Sondaki `*` işaretinden sonra gelen iki karakter, satırın sağlama toplamıdır (XOR). Sayaç sende farklı olacağı için toplamlar da farklı çıkabilir.

**İkinci dönüştürücü yoksa (geçici çözüm):** `main/fc_uart.c` dosyasında en üste `#include "esp_log.h"` ekle ve `fc_send` fonksiyonunda `uart_write_bytes` satırının hemen altına şunu yaz:

```c
ESP_LOGI("fc", "%s", line);
```

Böylece aynı satırlar seri monitörde de görünür. Kalp atışı saniyede 5 kez yazdığı için ekran hızlı akar. Denemeden sonra bu satırı sil.

---

## 7. Sık Karşılaşılan Sorunlar

| Belirti | Olası neden | Çözüm |
|---|---|---|
| `Brownout detector was triggered` ve sürekli yeniden başlama | Besleme yetersiz | Daha kısa ve kaliteli bir USB kablosu kullan, başka bir USB girişi dene. Drone'da ayrı bir 5V BEC ve 470–1000 µF kondansatör kullan (`warning.txt`) |
| `Camera probe failed`, `camera init failed` ya da `Camera could not capture a frame` | Kamera kablosu tam oturmamış ya da PSRAM kapalı | Kameranın şerit kablosunu çıkarıp kilidiyle birlikte düzgünce tak. `sdkconfig.defaults` içinde `CONFIG_SPIRAM=y` olduğunu kontrol et |
| Yüklemede `Failed to connect to ESP32` | Kart yükleme modunda değil | 5.2'nin sonundaki IO0/RST adımlarını uygula |
| COM portu görünmüyor | USB sürücüsü eksik ya da kablo sadece şarj kablosu | CH340/CP2102 sürücüsünü kur, veri kablosu kullan |

---

## 8. Sonraki Adımlar

Bu denemeler başarılı olduktan sonraki aşamalar:

1. **Karar modelini ESP32'ye eklemek:** Model, TensorFlow Lite Micro ile doğrudan ESP32'nin içinde çalışacak. Kamera bu aşamada 96×96 gri kareye geçecek. İlk olarak model rastgele ağırlıklarla eklenip kart üzerinde süresi ve bellek kullanımı ölçülecek.
2. **Eğitim verisi toplama:** Drone'un kendi kamerasından alınan kareler bilgisayara aktarılacak (Wi-Fi ile küçük bir kayıt betiği ya da SD kart). `wifi.c` bunun için projede duruyor.
3. **Modeli bilgisayarda eğitmek:** Model bilgisayarda eğitilip int8'e çevrilecek ve firmware'e gömülerek USB ile karta yüklenecek.
4. **Uçuş kontrolcüsü tarafı:** Uçuş kontrolcüsünde UART satırlarını okuyan, sağlama toplamını doğrulayan ve `LOST` durumunda güvenli moda geçen kod yazılacak.
