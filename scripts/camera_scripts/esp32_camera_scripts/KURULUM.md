# ESP32-CAM Kurulum ve İlk Deneme Rehberi

Bu belge, projenin ESP32-CAM tarafının gerçek bir kart üzerinde nasıl kurulacağını ve nasıl deneneceğini adım adım anlatır.

> **Bu denemenin kapsamı:** Yalnızca ESP32-CAM tarafı denenecektir. Sunucuya henüz bir model bağlı değildir; sunucu her kareye sabit bir cevap döndürür (`"hold"`, güven `0.0`). Modelli deneme sonraki aşamanın konusudur.

Bu denemede şunların çalıştığı doğrulanacak:

- Kodun derlenmesi ve karta yüklenmesi
- Kameranın açılması ve görüntü alınması
- Wi-Fi bağlantısı ve kopunca yeniden bağlanma
- Görüntünün JSON olarak sunucuya gönderilmesi ve cevabın alınması
- Uçuş kontrolcüsüne giden UART çıktısı (kalp atışı ve aksiyon satırları)
- Sunucu kapandığında güvenli duruma (failsafe) geçilmesi

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
- Python (kurulu; test sunucusu için)
- Telefonun kişisel erişim noktası (hotspot)

> **Önemli:** ESP32 yalnızca **2.4 GHz** Wi-Fi'ye bağlanabilir. Telefonun hotspot ayarlarında bant 2.4 GHz olmalıdır. iPhone'da bunun için **"Uyumluluğu En Üst Düzeye Çıkar"** seçeneğini aç.

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
    ├── idf_component.yml   Dışarıdan inen kütüphaneler (kamera, cJSON)
    ├── config.h            Bütün ayarlar
    ├── main.c              Başlangıç noktası (app_main)
    ├── wifi.c / .h         Wi-Fi bağlantısı
    ├── camera.c / .h       Kamera başlatma ve kurtarma
    ├── fc_uart.c / .h      Uçuş kontrolcüsüne UART çıktısı ve kalp atışı
    └── net.c / .h          Görüntü gönderme ve cevap işleme
```

Sunucu dosyaları bir üst klasördeki `server/` klasöründedir.

---

## 3. Test Sunucusunu Bilgisayarda Çalıştırma

Deneme için sunucu Hugging Face'e yüklenmeden, doğrudan bilgisayarda çalıştırılacak.

### 3.1. Paketleri kur

PowerShell'de `server` klasörüne geç ve paketleri kur:

```powershell
cd "C:\Users\Utku Ege\Documents\drone_project\scripts\camera_scripts\server"
python -m pip install -r requirements.txt
```

### 3.2. Sunucuyu başlat

```powershell
$env:API_TOKEN = "deneme-anahtari"
$env:SAVE_FRAMES = "1"
python -m uvicorn app:app --host 0.0.0.0 --port 8000
```

- `API_TOKEN` değeri, ESP32 tarafındaki `config.h` içinde yazan değerle **birebir aynı** olmalıdır.
- `SAVE_FRAMES = "1"` olduğu için gelen her kare `server/frames/` klasörüne JPEG olarak kaydedilir. Kameranın gerçekten görüntü aldığını buradan kontrol edeceğiz.

Sunucunun çalıştığını görmek için tarayıcıda `http://localhost:8000/docs` adresini aç. FastAPI'nin belge sayfası açılıyorsa sunucu hazırdır.

İlk çalıştırmada Windows Güvenlik Duvarı, Python için izin isteyebilir. **Özel** ve **Ortak** ağların ikisine de izin ver; telefon hotspot'u genellikle "ortak ağ" olarak görünür.

### 3.3. ESP32'nin sunucuya ulaşma yolunu seç

İki seçenek var. Önce A'yı dene; olmazsa B'ye geç.

**Seçenek A: Aynı hotspot üzerinden (en basit)**

1. Bilgisayarı da ESP32 ile aynı telefon hotspot'una bağla.
2. PowerShell'de `ipconfig` yaz ve Wi-Fi bağdaştırıcısının **IPv4 adresini** not al (örneğin `192.168.43.25`).
3. Sunucu adresi: `http://192.168.43.25:8000/frame` (kendi IP adresinle).

Bu yolda bağlantı HTTPS değil HTTP'dir. Deneme için sorun değildir, kod iki türü de destekler.

**Seçenek B: Cloudflare Tunnel üzerinden (gerçek kullanıma daha yakın)**

Hotspot, cihazların birbirini görmesine izin vermiyorsa ya da gerçek HTTPS yolunu denemek istiyorsan bu yolu kullan. Hesap açmak gerekmez.

```powershell
winget install --id Cloudflare.cloudflared
cloudflared tunnel --url http://localhost:8000
```

Komut, `https://....trycloudflare.com` biçiminde bir adres verir. Sunucu adresi bu adresin sonuna `/frame` eklenmiş halidir. Bu adres, komut her çalıştırıldığında değişir.

---

## 4. ESP32 Ayarlarını Yapma

`main/config.h` dosyasının başındaki değerleri doldur:

```c
#define WIFI_SSID   "hotspot-adi"
#define WIFI_PASS   "hotspot-sifresi"
#define SERVER_URL  "http://192.168.43.25:8000/frame"   // 3.3'te belirlediğin adres
#define API_TOKEN   "deneme-anahtari"                   // 3.2'deki ile aynı
#define DEVICE_ID   "drone1"
```

Diğer değerlere bu denemede dokunmana gerek yok.

---

## 5. Derleme

1. VS Code'da **File → Open Folder** ile `esp32_camera_scripts` klasörünü aç.
2. Ctrl+Shift+P → **ESP-IDF: Select Current ESP-IDF Version** → kurulu sürümü seç.
3. Ctrl+Shift+P → **ESP-IDF: Set Espressif device target** → **esp32**.
4. Ctrl+Shift+P → **ESP-IDF: Build your project**.

İlk derleme birkaç dakika sürer. Kamera ve cJSON kütüphaneleri bu sırada `managed_components/` klasörüne iner. Derleme başarılı olursa terminalde programın ne kadar yer kapladığını gösteren bir özet çıkar.

> `sdkconfig.defaults` dosyasını değiştirirsen, değişikliğin geçerli olması için oluşan `sdkconfig` dosyasını silip yeniden derlemen gerekir.

---

## 6. Karta Yükleme

### 6.1. Bağlantı

**ESP32-CAM-MB ile:** ESP32-CAM'i MB kartına tak (kamera ve MB kartındaki USB girişi aynı yöne bakmalı), USB kablosunu bilgisayara bağla. Çoğu MB kartı yükleme moduna kendiliğinden geçer.

**USB-TTL (FTDI) ile:**

| USB-TTL | ESP32-CAM |
|---|---|
| 5V | 5V |
| GND | GND |
| TX | U0R (GPIO3) |
| RX | U0T (GPIO1) |

Yüklemeden önce **IO0 pinini GND'ye bağla** ve RST düğmesine bas. Yükleme bitince IO0–GND bağlantısını çıkar ve tekrar RST'ye bas.

### 6.2. Portu seç ve yükle

1. Aygıt Yöneticisi'nde **Bağlantı noktaları (COM ve LPT)** altında kartın hangi COM portunu aldığına bak. Görünmüyorsa CH340 (ya da CP2102) sürücüsünü kur.
2. Ctrl+Shift+P → **ESP-IDF: Select Port to Use** → bu portu seç.
3. Ctrl+Shift+P → **ESP-IDF: Build, Flash and Monitor**.

Yükleme bitince seri monitör kendiliğinden açılır. Monitörden çıkmak için **Ctrl+]** tuşlarını kullan.

Yükleme "Failed to connect to ESP32" hatasıyla durursa: MB kartındaki **IO0** düğmesini basılı tut, **RST**'ye bir kez bas, IO0'ı bırak ve yüklemeyi tekrar başlat.

---

## 7. Deneme Adımları

Her adımda neyi göreceğin aşağıda yazıyor. Bir adım beklendiği gibi çalışmazsa sonrakine geçmeden 8. bölüme bak.

### Deneme 1: Açılış ve Wi-Fi

Seri monitörde sırayla şunları görmelisin:

```
I (...) wifi: Waiting for connection
I (...) wifi: Connected
```

Kamera sorunsuz açıldıysa kamera ile ilgili hata satırı çıkmaz.

### Deneme 2: Sunucuya görüntü gönderme

- Sunucunun çalıştığı PowerShell penceresinde saniyede birkaç kez `POST /frame HTTP/1.1" 200 OK` satırı akmalı.
- `server/frames/` klasöründe JPEG dosyaları birikmeli. Birkaçını açıp görüntünün düzgün olduğunu kontrol et. Görüntü baş aşağıysa ya da aynalıysa bu daha sonra kamera ayarlarından düzeltilebilir; deneme için sorun değildir.
- Her şey yolundayken ESP32'nin seri monitörü **sessiz kalır**. Ağ modülü yalnızca bir sorun olduğunda uyarı yazar.

### Deneme 3: Aksiyon yolunu sahte bir cevapla deneme

Model olmadığı için sunucu şu an hep `"hold"` ve güven `0.0` döndürüyor. Güven eşiğin (%60) altında olduğu için ESP32 bu cevabı uçuş kontrolcüsüne **iletmez**, ama bağlantıyı sağlıklı sayar.

Aksiyonun uçuş kontrolcüsüne kadar gittiğini görmek için `server/app.py` içindeki `run_model` fonksiyonunu geçici olarak şöyle değiştir:

```python
def run_model(img: Image.Image):
    return "left", 0.9
```

Sunucuyu durdurup (Ctrl+C) yeniden başlat. Artık her karede UART çıktısına bir `ACT` satırı gitmeli (bkz. 7.6). Denemeden sonra bu değişikliği geri al.

### Deneme 4: Sunucu kapanınca failsafe

1. Sunucuyu Ctrl+C ile durdur.
2. ESP32'nin seri monitöründe `Request failed: ...` uyarıları görünmeli. İsteklerin arası giderek açılır (en fazla 1 saniye).
3. Yaklaşık 2 saniye içinde kalp atışı satırları `OK` yerine `LOST` demeye başlamalı.
4. Sunucuyu yeniden başlat. ESP32 kendiliğinden toparlanmalı; kalp atışı tekrar `OK` olmalı.

### Deneme 5: Wi-Fi kopması

1. Telefonun hotspot'unu kapat. Kalp atışı `LOST` olmalı, program çökmemeli.
2. Hotspot'u tekrar aç. ESP32 kendiliğinden yeniden bağlanmalı ve gönderime devam etmeli.

### 7.6. UART çıktısını izleme

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
$HB,57,OK*0C
$ACT,12,left,90*6B
```

Sondaki `*` işaretinden sonra gelen iki karakter, satırın sağlama toplamıdır (XOR). Sayaç ve sıra numaraları sende farklı olacağı için toplamlar da farklı çıkacaktır.

**İkinci dönüştürücü yoksa (geçici çözüm):** `main/fc_uart.c` dosyasında en üste `#include "esp_log.h"` ekle ve `fc_send` fonksiyonunda `uart_write_bytes` satırının hemen altına şunu yaz:

```c
ESP_LOGI("fc", "%s", line);
```

Böylece aynı satırlar seri monitörde de görünür. Kalp atışı saniyede 5 kez yazdığı için ekran hızlı akar. Denemeden sonra bu satırı sil.

---

## 8. Sık Karşılaşılan Sorunlar

| Belirti | Olası neden | Çözüm |
|---|---|---|
| `Brownout detector was triggered` ve sürekli yeniden başlama | Besleme yetersiz | Daha kısa ve kaliteli bir USB kablosu kullan, başka bir USB girişi dene. Drone'da ayrı bir 5V BEC ve 470–1000 µF kondansatör kullan (`warning.txt`) |
| `Camera probe failed` ya da `camera init failed` | Kamera kablosu tam oturmamış ya da PSRAM kapalı | Kameranın şerit kablosunu çıkarıp kilidiyle birlikte düzgünce tak. `sdkconfig.defaults` içinde `CONFIG_SPIRAM=y` olduğunu kontrol et |
| `Waiting for connection` satırında takılı kalma | SSID/şifre yanlış ya da hotspot 5 GHz | `config.h` değerlerini kontrol et, hotspot'u 2.4 GHz yap |
| `Request failed: ESP_ERR_HTTP_CONNECT` | Sunucuya ulaşılamıyor | IP adresini, portu (8000) ve Güvenlik Duvarı iznini kontrol et. Olmazsa Seçenek B'ye (Cloudflare Tunnel) geç |
| `Status 401` | Token uyuşmuyor | `config.h` içindeki `API_TOKEN` ile sunucudaki `$env:API_TOKEN` aynı olmalı |
| `Status 422` | JSON biçimi sunucunun beklediğiyle uyuşmuyor | ESP32 ile sunucu kodunun güncel sürümde olduğunu kontrol et |
| `Bad response: ... ms` | Cevap çok geç geliyor | Bağlantı yavaş. `config.h` içinde `CAM_FRAME_SIZE` değerini `FRAMESIZE_QVGA` yap |
| `seq does not fit` | Eski bir isteğe ait cevap geldi | Arada bir görünmesi normaldir; sürekli görünüyorsa sunucunun `seq` değerini geri döndürdüğünü kontrol et |
| Derlemede `cJSON.h` bulunamadı | ESP-IDF v6 kullanılıyor ama cJSON eklenmemiş | `main/idf_component.yml` içinde `espressif/cjson: "*"` satırının açık olduğunu kontrol et |
| Derlemede cJSON ile ilgili "birden fazla tanım" hatası | ESP-IDF v5 kullanılıyor ve cJSON iki kez eklenmiş | v5'te cJSON zaten hazır gelir; `idf_component.yml` içindeki `espressif/cjson` satırının başına `#` koy |
| Yüklemede `Failed to connect to ESP32` | Kart yükleme modunda değil | 6.2'nin sonundaki IO0/RST adımlarını uygula |
| COM portu görünmüyor | USB sürücüsü eksik ya da kablo sadece şarj kablosu | CH340/CP2102 sürücüsünü kur, veri kablosu kullan |

---

## 9. Sonraki Adımlar

Bu denemeler başarılı olduktan sonraki aşamalar:

1. **Eğitim verisi toplama:** `SAVE_FRAMES=1` ile yapılan denemelerde biriken kareler, modelin eğitimi için ilk veri setidir. Modeli ESP32-CAM'in kendi görüntüleriyle eğitmek sahada daha iyi sonuç verir.
2. **Model entegrasyonu:** Eğitilen model, `server/app.py` içindeki `run_model` fonksiyonuna bağlanacak.
3. **Sunucuyu internete taşıma:** Sunucu Hugging Face Spaces'e yüklenecek ve `SERVER_URL` kalıcı HTTPS adresiyle değiştirilecek.
4. **Uçuş kontrolcüsü tarafı:** Uçuş kontrolcüsünde UART satırlarını okuyan, sağlama toplamını doğrulayan ve `LOST` durumunda güvenli moda geçen kod yazılacak.
