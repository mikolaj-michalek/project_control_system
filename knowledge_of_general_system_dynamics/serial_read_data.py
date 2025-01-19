import serial
import time

# Konfiguracja portu szeregowego
serial_port = "COM6"  # Zmień na odpowiedni port (np. COM3 dla Windows, /dev/ttyUSB0 dla Linux)
baud_rate = 115200  # Prędkość transmisji (musi być taka sama jak w STM32)
file_name = "output_final.txt"

try:
    # Otwórz port szeregowy
    ser = serial.Serial(serial_port, baud_rate, timeout=1)
    print(f"Połączono z {serial_port} na prędkości {baud_rate}")

    # Otwórz plik do zapisu
    with open(file_name, "a") as file:
        print(f"Zapisuję dane do pliku {file_name}...")

        while True:
            # Odczytaj linię danych z portu szeregowego
            data = ser.readline().decode("utf-8").strip()  # Strip usuwa białe znaki i nowe linie
            if data:
                print(f"Odebrano: {data}")
                file.write(data + "")  # Zapisz do pliku z nową linią
                file.flush()  # Wymuszenie zapisu do pliku na bieżąco

except serial.SerialException as e:
    print(f"Błąd portu szeregowego: {e}")
except KeyboardInterrupt:
    print("Zatrzymano przez użytkownika.")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()
        print("Port szeregowy zamknięty.")
