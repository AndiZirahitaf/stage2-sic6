import machine
import time
import math
from machine import Pin, I2C, PWM
import network
import urequests

# Konfigurasi pin
BUZZER_PIN = 4
BUTTON_PIN = 15
LED_PIN_RED = 26
LED_PIN_GREEN = 27
MPU_INT_PIN = 18

# Konfigurasi MPU6050
MPU_ADDR = 0x68  # Alamat I2C default MPU6050
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43

# Konfigurasi threshold deteksi jatuh
ACCEL_THRESHOLD = 2.5  # dalam satuan g
GYRO_THRESHOLD = 300   # dalam derajat/detik

# Variabel status
fall_detected = False
system_active = True

# Inisialisasi komponen
i2c = I2C(0, scl=Pin(22), sda=Pin(21), freq=400000)
button = Pin(BUTTON_PIN, Pin.IN, Pin.PULL_UP)
red_led = Pin(LED_PIN_RED, Pin.OUT)
green_led = Pin(LED_PIN_GREEN, Pin.OUT)
buzzer = PWM(Pin(BUZZER_PIN))
buzzer.duty(0)  # Matikan buzzer

# Setup WiFi (opsional untuk notifikasi)
WIFI_SSID = ""
WIFI_PASSWORD = ""

# URL untuk notifikasi (misalnya dengan Telegram Bot atau layanan lain)
NOTIFICATION_URL = "https://api.telegram.org/botYOUR_TOKEN/sendMessage"
CHAT_ID = "YOUR_CHAT_ID"

# Inisialisasi MPU6050
def mpu_init():
    # Bangunkan MPU6050 (keluar dari sleep mode)
    i2c.writeto_mem(MPU_ADDR, PWR_MGMT_1, b'\x00')
    time.sleep(0.1)
    
    # Verifikasi komunikasi dengan MPU6050
    if MPU_ADDR not in i2c.scan():
        print("MPU6050 tidak terdeteksi!")
        return False
    return True

# Baca data akselerometer dari MPU6050
def read_accel():
    data = i2c.readfrom_mem(MPU_ADDR, ACCEL_XOUT_H, 6)
    ax = (data[0] << 8 | data[1]) / 16384.0
    ay = (data[2] << 8 | data[3]) / 16384.0
    az = (data[4] << 8 | data[5]) / 16384.0
    return (ax, ay, az)

# Baca data gyroscope dari MPU6050
def read_gyro():
    data = i2c.readfrom_mem(MPU_ADDR, GYRO_XOUT_H, 6)
    gx = (data[0] << 8 | data[1]) / 131.0
    gy = (data[2] << 8 | data[3]) / 131.0
    gz = (data[4] << 8 | data[5]) / 131.0
    return (gx, gy, gz)

# Hitung vektor magnitude dari akselerometer
def calculate_accel_magnitude(ax, ay, az):
    return math.sqrt(ax*ax + ay*ay + az*az)

# Hitung vektor magnitude dari gyroscope
def calculate_gyro_magnitude(gx, gy, gz):
    return math.sqrt(gx*gx + gy*gy + gz*gz)

# Hubungkan ke WiFi
def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        print('Menghubungkan ke WiFi...')
        wlan.connect(WIFI_SSID, WIFI_PASSWORD)
        for _ in range(10):  # Tunggu max 10 detik
            if wlan.isconnected():
                break
            time.sleep(1)
    if wlan.isconnected():
        print('Terhubung ke WiFi')
        print('IP:', wlan.ifconfig()[0])
        return True
    else:
        print('Gagal terhubung ke WiFi')
        return False

# Kirim notifikasi
def send_notification(message):
    try:
        data = {
            'chat_id': CHAT_ID,
            'text': message
        }
        response = urequests.post(NOTIFICATION_URL, json=data)
        response.close()
        return True
    except:
        return False

# Aktifkan alarm
def trigger_alarm():
    global fall_detected
    fall_detected = True
    red_led.value(1)
    green_led.value(0)
    
    # Bunyi buzzer dengan nada tertentu
    buzzer.freq(1000)  # 1kHz
    buzzer.duty(512)   # 50% duty cycle
    
    # Kirim notifikasi jika terhubung ke WiFi
    if connect_wifi():
        send_notification("PERHATIAN! Deteksi jatuh terdeteksi!")

# Reset alarm
def reset_alarm():
    global fall_detected
    fall_detected = False
    red_led.value(0)
    green_led.value(1)
    buzzer.duty(0)  # Matikan buzzer

# Interrupt handler untuk tombol darurat
def button_handler(pin):
    global fall_detected, system_active
    time.sleep(0.1)  # Debounce
    if not button.value():  # Tombol ditekan
        if fall_detected:
            reset_alarm()
        else:
            # Toggle sistem aktif/tidak aktif
            system_active = not system_active
            if system_active:
                green_led.value(1)
            else:
                green_led.value(0)

# Setup interrupt untuk tombol
button.irq(trigger=Pin.IRQ_FALLING, handler=button_handler)

# Inisialisasi sistem
def init_system():
    if not mpu_init():
        for _ in range(5):  # Kedipkan LED merah 5 kali jika MPU6050 tidak terdeteksi
            red_led.value(1)
            time.sleep(0.2)
            red_led.value(0)
            time.sleep(0.2)
        return False
    
    # Sistem siap
    green_led.value(1)
    buzzer.freq(2000)
    buzzer.duty(512)
    time.sleep(0.1)
    buzzer.duty(0)
    return True

# Loop utama
def main_loop():
    prev_accel_magnitude = 1.0  # Nilai gravitasi normal
    time_fall_detected = 0
    
    while True:
        if not system_active:
            time.sleep(0.5)
            continue
            
        if fall_detected:
            # Cek apakah tombol reset ditekan
            time.sleep(0.5)
            continue
            
        # Baca data sensor
        try:
            ax, ay, az = read_accel()
            gx, gy, gz = read_gyro()
            
            accel_magnitude = calculate_accel_magnitude(ax, ay, az)
            gyro_magnitude = calculate_gyro_magnitude(gx, gy, gz)
            
            # Deteksi jatuh dengan algoritma sederhana
            accel_change = abs(accel_magnitude - prev_accel_magnitude)
            
            if accel_change > ACCEL_THRESHOLD and gyro_magnitude > GYRO_THRESHOLD:
                # Fase 1: Deteksi jatuh - perubahan akselerasi yang signifikan
                print("Perubahan akselerasi terdeteksi:", accel_change)
                print("Gyro magnitude:", gyro_magnitude)
                
                # Tunggu sebentar untuk melihat apakah ada fase "impact" dan kemudian "rest"
                time.sleep(0.5)
                
                # Baca ulang untuk mengonfirmasi fase "rest"
                ax2, ay2, az2 = read_accel()
                accel_magnitude2 = calculate_accel_magnitude(ax2, ay2, az2)
                
                # Jika setelah jatuh tubuh berada dalam posisi diam (mendekati 1g)
                if abs(accel_magnitude2 - 1.0) < 0.3:
                    print("Deteksi jatuh dikonfirmasi!")
                    trigger_alarm()
            
            # Simpan nilai akselerasi sebelumnya
            prev_accel_magnitude = accel_magnitude
            
        except Exception as e:
            print("Error:", e)
        
        time.sleep(0.1)

# Program utama
if _name_ == "_main_":
    if init_system():
        try:
            main_loop()
        except KeyboardInterrupt:
            pass
        finally:
            buzzer.deinit()
