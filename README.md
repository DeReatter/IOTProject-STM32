# IOTProject-STM32

Firmware mikrokontroler STM32 untuk sistem IoT Fire System. Repo ini menangani pembacaan sensor lapangan, logika kontrol lokal, dan pertukaran data dengan modul ESP32.

## Ekosistem Project

Sistem ini terbagi ke dalam tiga repository terpisah:
* **Firmware STM32 (Repo ini):** Akuisisi data sensor dan kontrol aktuator utama.
* **Firmware ESP32:** Konektivitas jaringan dan pengiriman data ke server/cloud (`[Tambahkan link repo ESP32 di sini]`).
* **Aplikasi Android:** Antarmuka monitoring dan kontrol pengguna di [Firesystem-app](https://github.com/DeReatter/Firesystem-app).

## Fitur Utama

* Pembacaan sensor suhu, asap, atau api secara berkala (`[sesuaikan dengan tipe sensor yang dipakai]`).
* Aktivasi alarm dan aktuator proteksi otomatis saat nilai sensor melewati ambang batas.
* Komunikasi serial (`[UART / I2C / SPI]`) untuk mengirim status sistem ke mikrokontroler ESP32.

## Struktur Direktori

* `Core/Inc/` dan `Core/Src/` - Kode utama aplikasi dan konfigurasi peripheral.
* `Drivers/` - Library STM32 HAL dan CMSIS.
* `*.ioc` - File konfigurasi pinout dan clock untuk STM32CubeMX / STM32CubeIDE.

## Kebutuhan Hardware dan Software

* Board mikrokontroler `[seri STM32, misal STM32F103C8T6 / STM32F401]`.
* ST-Link V2 untuk flashing dan debugging.
* STM32CubeIDE.

## Cara Build dan Flash

1. Clone repository ini:
   ```bash
   git clone [https://github.com/DeReatter/IOTProject-STM32.git](https://github.com/DeReatter/IOTProject-STM32.git)
2. Buka STM32CubeIDE, pilih File - Open Projects from File System..., lalu arahkan ke folder repo ini.
3. Klik ikon Build untuk mengompilasi project.
4. Hubungkan ST-Link ke board STM32, lalu klik Run untuk mengunggah firmware ke mikrokontroler.
