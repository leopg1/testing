# Instrucțiuni de instalare și utilizare pentru robotul de navigare în labirint

Acest document conține instrucțiunile complete pentru instalarea, configurarea și utilizarea robotului de navigare în labirint pe Raspberry Pi.

## 1. Cerințe hardware

- Raspberry Pi 5 cu RPi OS (64-bit) instalat
- LIDAR STL 19P
- Senzor Hall KY-024
- Senzor vibrații SW-420
- Senzor alcool MQ-3
- LED-uri RGB și buzzer pentru semnalizare

## 2. Instalare ROS2 Humble pe Raspberry Pi OS

```bash
# Actualizare sistem
sudo apt update && sudo apt upgrade -y

# Configurare locale
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Adăugare repository-uri ROS2
sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Instalare ROS2 Humble
sudo apt update
sudo apt install -y ros-humble-ros-base

# Instalare dependențe necesare
sudo apt install -y python3-pip python3-colcon-common-extensions
sudo apt install -y ros-humble-slam-toolbox
sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install -y python3-rpi.gpio python3-numpy python3-opencv

# Adăugare inițializare ROS2 în .bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 3. Copiere și construire proiect

### 3.1 Copierea proiectului pe Raspberry Pi

Opțiunea 1: Folosind USB:
- Copiați directorul `HS` pe un stick USB
- Conectați stick-ul la Raspberry Pi
- Copiați directorul în directorul principal: `cp -r /media/pi/USB/HS ~/`

Opțiunea 2: Folosind SCP (din calculatorul vostru):
```bash
scp -r HS/ pi@<IP_RASPBERRY>:~/
```

### 3.2 Construirea proiectului

```bash
cd ~/HS
chmod +x start_maze_robot.sh
colcon build
source install/setup.bash

# Adăugare setup în .bashrc pentru a nu fi nevoie să-l rulăm de fiecare dată
echo "source ~/HS/install/setup.bash" >> ~/.bashrc
```

## 4. Conectarea senzorilor

### 4.1 Senzor Hall (KY-024)
- VCC → 3.3V (Pin 1)
- GND → GND (Pin 6)
- OUT → GPIO17 (Pin 11)

### 4.2 Senzor vibrații (SW-420)
- VCC → 3.3V (Pin 1)
- GND → GND (Pin 9)
- DO → GPIO27 (Pin 13)

### 4.3 Senzor alcool (MQ-3)
- VCC → 5V (Pin 2)
- GND → GND (Pin 14)
- AOUT → Conectat la ADC (ADS1115) canal A0
- ADC I2C → GPIO2 (SDA, Pin 3) și GPIO3 (SCL, Pin 5)

### 4.4 LED-uri RGB
- LED R → GPIO18 (Pin 12)
- LED G → GPIO23 (Pin 16)
- LED B → GPIO24 (Pin 18)

### 4.5 Buzzer
- Buzzer → GPIO25 (Pin 22)

### 4.6 LIDAR STL 19P
- Conectați LIDAR-ul la un port USB
- Verificați dispozitivul: `ls /dev/ttyUSB*`
- Modificați permisiunile: `sudo chmod 666 /dev/ttyUSB0` (sau numărul corespunzător)

## 5. Rularea proiectului

### 5.1 Folosind scriptul de pornire

```bash
cd ~/HS
./start_maze_robot.sh
```

Acest script vă va prezenta un meniu cu opțiuni:
1. Testare senzori - verifică funcționarea senzorilor și LED-urilor
2. Navigare completă - pornește explorarea completă a labirintului
3. Testare LIDAR - testează doar funcționalitatea LIDAR

### 5.2 Rularea manuală a componentelor

#### Testare senzori
```bash
ros2 run maze_robot sensor_test
```

#### Navigare completă
```bash
ros2 launch maze_robot launch_maze_navigation.py
```

#### Vizualizare jurnale de evenimente
```bash
cd ~/HS/logs
cat events_*.json
```

## 6. Depanare probleme

### 6.1 Problema: Nu pot accesa GPIO

```bash
sudo usermod -a -G gpio $USER
sudo chmod a+rw /dev/gpiomem
# Reconectați-vă sau reporniți
```

### 6.2 Problema: LIDAR nu este detectat

```bash
ls -l /dev/ttyUSB*
sudo chmod a+rw /dev/ttyUSB0  # Sau numărul corespunzător
```

### 6.3 Problema: Erori de compilare

```bash
# Curățați build-ul anterior
rm -rf build/ install/ log/
# Reconstruiți
colcon build
```

### 6.4 Verificare topic-uri

```bash
# Listează toate topic-urile active
ros2 topic list

# Monitorizează un topic specific
ros2 topic echo /sensors/status
```

## 7. Note și sfaturi

- Asigurați-vă că bateria robotului este încărcată complet înainte de a începe explorarea
- Poziționați robotul la intrarea în labirint (0,0) înainte de a porni algoritmul de explorare
- LIDAR-ul ar trebui să fie montat la înălțimea optimă pentru a detecta pereții labirintului
- Pentru a opri executarea oricărui nod ROS2, folosiți Ctrl+C în terminalul în care rulează
