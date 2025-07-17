## **ບົດລາຍງານ**
**ວິຊາ: ໄມໂຄຣໂປຣເຊັດເຊີ - ໄມໂຄຣຄອນພິວເຕີ້**
**ສອນໂດຍ: ອຈ. ປຕ. ລັດທິດາ ຄົມສອນລະສິນ**

---
### **ສະມາຊິກໃນກຸ່ມ**
1. ທ້າວ ວິລະພົນ ຄຳວົງທອງ					(ເລກທີ 8)
2. ທ້າວ ອົງກອນ ຂຸນພິທັກ					(ເລກທີ 10)
3. ທ້າວ ພຸດທະສອນ ຈັນທະວົງສາ				(ເລກທີ 25)
4. ທ້າວ ອານຸພັດ ຈິນດາຮັກ					(ເລກທີ 27)
5. ທ້າວ ສາຍຟ້າ ສຸຂະຫວັດ					(ເລກທີ 35)
6. ທ້າວ ສຸກພະໄຊ ທຽງທຳ 					(ເລກທີ 45)

---

### **ສາລະບານ**
* [ບົດນຳ](#ບົດນຳ)
* [ບົດທີ 1: LED 8x8 DOT Matrix Display](#ບົດທີ-1-LED-8x8-DOT-Matrix-Display)
* [ບົດທີ 2: Photoresistor (Light Sensor)](#ບົດທີ-2-PHOTORESISTOR-Light-sensor)
* [ບົດທີ 3: PIR Sensor](#ບົດທີ-3-PIR-sensor)
* [ບົດທີ 4: Ultrasonic Sensor](#ບົດທີ-4-Ultrasonic-sensor)
* [ບົດທີ 5: LCD](#ບົດທີ-5-LCD)
* [ບົດທີ 6: Joystick](#ບົດທີ-6-Joystick)
* [ບົດທີ 7: Bluetooth](#ບົດທີ-7-Bluetooth)
* [Midterm](#Midterm)
    * [Lock System](#Midterm-Lock-System)
    * [Sensor Motion](#Midterm-Sensor-Motion)
    * [Automatic Elevator Door](#Midterm-Automatic-Elevator-Door)
    * [Ultrasonic Radar Display](#Midterm-Ultrasonic-Radar-Display)
    * [Flood Alert](#Midterm-Flood-Alert)

---

### **ບົດນຳ**

ໄມໂຄຣໂປຣເຊສເຊີ ແລະ ໄມໂຄຄອມພີວເຕີເປັນອົງປະກອບ ທີ່ສຳຄັນໃນຄອມພິວເຕີສະ ໄໝໃໝ່, ລະບົບອັດຕະໂນມັດ ແລະລະບົບຝັງຕົວ. ໄມໂຄຣໂປຣເຊສເຊີ (microprocessor) ເຮັດຫນ້າທີ່ເປັນຫນ່ວຍປະມວນຜົນກາງ (CPU) ທີ່ປະຕິບັດຄໍາສັ່ງ ແລະ ຄວບຄຸມການເຮັດວຽກຕ່າງໆ. ໄມໂຄຄອມພີວເຕີ (microcomputer) ແມ່ນການປະສົມປະສານຂອງ microprocessor ກັບຫນ່ວຍຄວາມຈໍາ ແລະ ອຸປະກອນເສີມເພື່ອເຮັດໃຫ້ມັນປະຕິບັດຫນ້າວຽກສະເພາະໃດຫນຶ່ງປະສິດທິພາບ.

ບົດ​ລາຍ​ງານ​ນີ້​ໄດ້ສຳຫຼວດການ​ທົດ​ລອງ​ທີ່​ສໍາ​ຄັນ​ທີ່​ສະ​ແດງ​ໃຫ້​ເຫັນ​ເຖິງການ​ເຮັດວຽກ​ພື້ນ​ຖານ​ຂອງ microcontroller ໄດ້, ລວມມີການຄວບຄຸມການປ້ອນຂໍ້ມູນ/ການສົ່ງອອກແບບດິຈິຕອນ, ການເຊື່ອມຕໍ່ເຊັນເຊີ และ ການສື່ສານກັບອຸປະກອນຕ່າງໆ. ການທົດລອງເຫຼົ່ານີ້ນໍາສະເໜີການໃຊ້ງານຕົວຈິງທີ່ຊ່ວຍໃຫ້ເຂົ້າໃຈການເຮັດວຽກແລະການປະຍຸກໃຊ້ຂອງmicrocontrollersໃນສະຖານະການຈິງ. ຫຼັງຈາກຜ່ານການທົດສອບເຫຼົ່ານີ້, ນັກສຶກສາຈະ ໄດ້ຮັບປະສົບການທາງດ້ານປະຕິບັດຕົວຈິງໃນການເຊື່ອມຕໍ່ໄມໂຄຣຄອນໂທຣອນເລີກັບອຸປະກອນຕ່າງໆ, ການນຳໃຊ້ເຊັນເຊີແລະການພັດທະນາໂປຣແກຣມ, ເຊິ່ງເປັນພື້ນຖານສຳຄັນໃນການອອກ ແບບ ແລະ ພັດທະນາລະບົບທີ່ຊັບຊ້ອນຂຶ້ນໃນອະນາຄົດ.

---

### **ບົດທີ 1: LED 8x8 DOT Matrix Display**

**ບົດນຳ**
ການທົດລອງນີ້ສະແດງໃຫ້ເຫັນວິທີການພື້ນຖານທີ່ສຸດໃນການຄວບຄຸມ Output ດ້ວຍ Arduino, ເຊິ່ງແມ່ນການເຮັດໃຫ້ໄຟ LED ກະພິບ.

**ຈຸດປະສົງ**
- ຮຽນຮູ້ທີ່ຈະຄວບຄຸມ LED 8x8 ເປີດແລະປິດໂດຍໃຊ້ Arduino.
- ເຂົ້າໃຈຫຼັກການການເຮັດວຽກຂອງຄໍາສັ່ງ `digitalWrite()` ແລະ `delay()`.

**ອຸປະກອນ**
- Microcontroller (Arduino)
- 8x8 DOT Martix Display 1 ຕົວ
- ສາຍເຊື່ອມ (Jumper wires)

**ການເຊື່ອມຕໍ່ວົງຈອນ**
- Dot Matrix: ໃຊ້ ໃນຕົວ GND Arduino.
- ການສະຫນອງພະລັງງານ: Arduino ສະຫນອງພະລັງງານໃຫ້ກັບວົງຈອນທັງ ຫມົດ.

<img width="452" height="203" alt="image" src="https://github.com/user-attachments/assets/564d7bfd-3f7e-45f5-a23d-c04e82456679"/>
<img width="366" height="202" alt="image" src="https://github.com/user-attachments/assets/36059be7-449a-4707-aade-9d5cb5a3d9a4" />

**Code**
```cpp
// C++ code
// ໝາຍເຫດ: ໂຄດນີ້ແມ່ນສຳລັບການກະພິບ LED_BUILTIN (Pin 13), 
// ບໍ່ແມ່ນສຳລັບການຄວບຄຸມ 8x8 Dot Matrix ໂດຍກົງ.
// ການຄວບຄຸມ Dot Matrix ຕ້ອງການ Library ແລະ ໂຄດທີ່ຊັບຊ້ອນກວ່ານີ້.

void setup() {
  // ກໍານົດ Pin 13 ເປັນ Output
  pinMode(LED_BUILTIN, OUTPUT); 
}

void loop() {
  // ເປີດໄຟ LED
  digitalWrite(LED_BUILTIN, HIGH);
  // ລໍຖ້າ 1 ວິນາທີ
  delay(1000); 
  // ປິດໄຟ LED
  digitalWrite(LED_BUILTIN, LOW);
  // ລໍຖ້າ 1 ວິນາທີ
  delay(1000);
}
```

**ການເຮັດວຽກຂອງວົງຈອນ**
- **ກຳນົດ PIN:** Pin 13 ຖືກຕັ້ງເປັນ OUTPUT ເພື່ອຄວບຄຸມການເປີດ ແລະປິດ LED.
- **ເປີດ/ປິດ LED:**
  - ໃຊ້ຄໍາສັ່ງ `digitalWrite(13, HIGH)` ເພື່ອເປີດໄຟ LED.
  - ໃຊ້ຄໍາສັ່ງ `digitalWrite(13, LOW)` ເພື່ອປິດໄຟ LED.
- **ໜ່ວງເວລາ:** ໃຊ້ `delay(1000)` ເພື່ອລໍຖ້າ 1 ວິນາທີກ່ອນທີ່ຈະປ່ຽນສະຖານະຂອງ LED.

---

### **ບົດທີ 2: PHOTORESISTOR (Light sensor)**
**ບົດນຳ**
ການທົດລອງ Photoresistor ຫຼື LDR (Light Dependent Resistor) ແມ່ນໂຕກວດຈັບແສງ ໂດຍຄ່າຄວາມຕ້ານທານຂອງມັນຈະປ່ຽນແປງໄປຕາມຄວາມແຈ້ງຂອງແສງທີ່ຕົກກະທົບ.

**ອຸປະກອນ**
- Microcontroller (Arduino)
- Photoresistor (LDR)
- Resistor (10kΩ)
- Jumper wires
- Breadboard

**ການເຊື່ອມຕໍ່ວົງຈອນ**
- LDR ເຊື່ອມຕໍ່ກັບ Analog Pin (A0) ເພື່ອອ່ານຄ່າຄວາມສະຫວ່າງ.
- ຂາໜຶ່ງຂອງ LDR ຕໍ່ກັບ 5V.
- ອີກຂາໜຶ່ງຂອງ LDR ຕໍ່ກັບ A0 ແລະ ຕໍ່ກັບ Resistor 10kΩ ທີ່ຕໍ່ລົງ GND.

<img width="172" height="157" alt="image" src="https://github.com/user-attachments/assets/7370814a-1468-488f-9430-c0de13e47f80" />
<img width="162" height="156" alt="image" src="https://github.com/user-attachments/assets/f4ae0200-6bde-4d72-afeb-33eede2af704" />


**Code**
```cpp
// C++ code
const int LDR_PIN = A0; // ກໍານົດ LDR ຢູ່ທີ່ Analog Pin 0

void setup() {
  Serial.begin(9600); // ເລີ່ມການສື່ສານຜ່ານ Serial Port
}

void loop() {
  int ldrValue = analogRead(LDR_PIN); // ອ່ານຄ່າຈາກ LDR (0-1023)
  Serial.println(ldrValue); // ສະແດງຄ່າອອກທາງ Serial Monitor
  delay(500); // ລໍຖ້າ 0.5 ວິນາທີ
}
```

**ການເຮັດວຽກຂອງວົງຈອນ**
- Arduino ອ່ານຄ່າແຮງດັນຈາກຂາ A0, ເຊິ່ງປ່ຽນແປງໄປຕາມຄວາມຕ້ານທານຂອງ LDR.
- ຄ່າທີ່ອ່ານໄດ້ (0-1023) ຈະຖືກສະແດງຜົນຜ່ານ Serial Monitor ທຸກໆ 0.5 ວິນາທີ.
- ຄ່າຈະສູງເມື່ອມີແສງໜ້ອຍ ແລະ ຄ່າຈະຕໍ່າເມື່ອມີແສງຫຼາຍ.

---

### **ບົດທີ 3: PIR sensor**
**ບົດນຳ**
ການທົດລອງນີ້ສະແດງໃຫ້ເຫັນການໃຊ້ PIR (Passive Infrared) Sensor ເພື່ອກວດຈັບການເຄື່ອນໄຫວ. ເມື່ອມີການເຄື່ອນໄຫວ, ເຊັນເຊີຈະສົ່ງສັນຍານ HIGH ອອກມາ, ເຊິ່ງສາມາດໃຊ້ຄວບຄຸມອຸປະກອນອື່ນ ເຊັ່ນ LED.

**ອຸປະກອນ**
- Microcontroller (Arduino)
- PIR Sensor
- LED
- Resistor (220Ω)
- Jumper wires
- Breadboard

**ການເຊື່ອມຕໍ່ວົງຈອນ**
- **PIR Sensor:**
    - VCC ຕໍ່ກັບ 5V.
    - GND ຕໍ່ກັບ GND.
    - OUT ຕໍ່ກັບ Digital Pin 12.
- **LED:**
    - ຂາບວກ (Anode) ຕໍ່ກັບ Digital Pin 13.
    - ຂາລົບ (Cathode) ຕໍ່ກັບ Resistor 220Ω ແລ້ວຕໍ່ລົງ GND.
 
  <img width="259" height="194" alt="image" src="https://github.com/user-attachments/assets/b857bcf9-048d-4a4b-9d53-9125af8f897f" />


**Code**
```cpp
// C++ code
void setup() {
  Serial.begin(115200);
  pinMode(12, INPUT);  // Pin ຂອງ PIR ເປັນ Input
  pinMode(13, OUTPUT); // Pin ຂອງ LED ເປັນ Output
}

void loop() {
  int value = digitalRead(12); // ອ່ານຄ່າຈາກ PIR Sensor
  Serial.println(value);
  if (value == 1) { // ຖ້າກວດພົບການເຄື່ອນໄຫວ
    digitalWrite(13, HIGH); // ເປີດໄຟ LED
  } else {
    digitalWrite(13, LOW);  // ປິດໄຟ LED
  }
}
```
**ການເຮັດວຽກຂອງວົງຈອນ**
- Arduino ອ່ານຄ່າສະຖານະຈາກ PIR Sensor ທີ່ຕໍ່ກັບຂາທີ 12.
- ຖ້າເຊັນເຊີກວດພົບການເຄື່ອນໄຫວ (ສົ່ງຄ່າ 1), Arduino ຈະສັ່ງໃຫ້ໄຟ LED ທີ່ຂາ 13 ສະຫວ່າງຂຶ້ນ.
- ຖ້າບໍ່ມີການເຄື່ອນໄຫວ (ส่งค่า 0), ໄຟ LED ຈະດັບ.
- ຄ່າທີ່ອ່ານໄດ້ຈະຖືກສະແດງຜົນຜ່ານ Serial Monitor.

---

### **ບົດທີ 4: Ultrasonic sensor**
**ບົດນຳ**
ການທົດລອງ Ultrasonic sensor ສະແດງໃຫ້ເຫັນເຖິງການໃຊ້ເຊັນເຊີ້ເພື່ອວັດແທກໄລຍະຫ່າງ ແລະ ຄວບຄຸມ Output ເຊັ່ນ Servo Motor. ເຊັນເຊີຈະສົ່ງຄື້ນສຽງອອກໄປ ແລະ ຄຳນວນໄລຍະຫ່າງຈາກເວລາທີ່ຄື້ນສຽງສະທ້ອນກັບມາ.

**ອຸປະກອນ**
- Arduino
- Ultrasonic sensor (HC-SR04)
- Servo motor
- Jumper wires
- Breadboard

**ການເຊື່ອມຕໍ່ວົງຈອນ**
- **Ultrasonic Sensor:**
    - VCC → 5V
    - GND → GND
    - Trig → Pin 13
    - Echo → Pin 12
- **Servo Motor:**
    - ສາຍສັນຍານ (ສີສົ້ມ) → Pin 7
    - VCC (ສີແດງ) → 5V
    - GND (ສີດຳ) → GND
 
<img width="295" height="223" alt="image" src="https://github.com/user-attachments/assets/f0a0d101-74af-471d-bdd6-7d2e485e7a4a" />


**Code**
```cpp
// C++ code
#include <Servo.h>

Servo srv;
#define maxdistance 100

void setup() {
  Serial.begin(9600);
  pinMode(13, OUTPUT); // Trig Pin
  pinMode(12, INPUT);  // Echo Pin
  srv.attach(7);
}

void loop() {
  // ສົ່ງ Pulse ອອກໄປ
  digitalWrite(13, LOW);
  delayMicroseconds(5);
  digitalWrite(13, HIGH);
  delayMicroseconds(10);
  digitalWrite(13, LOW);
  
  // ວັດແທກເວລາທີ່ສັນຍານກັບມາ
  int d = pulseIn(12, HIGH);
  d = d / 29 / 2; // ແປງເວລາເປັນໄລຍະຫ່າງ (cm)
  
  Serial.println(d);
  
  if (d <= 20) { // ຖ້າໄລຍະຫ່າງນ້ອຍກວ່າ ຫຼື ເທົ່າກັບ 20 cm
    srv.write(90); // ໝຸນ Servo ໄປ 90 ອົງສາ
  } else {
    srv.write(0);  // ໝຸນ Servo ກັບຄືນ
  }
  delay(500);
}
```
**ການເຮັດວຽກຂອງວົງຈອນ**
- Arduino ສົ່ງສັນຍານ ultrasonic ອອກຈາກ Trig pin.
- ຫຼັງຈາກນັ້ນ, ມັນຈະລໍຖ້າຮັບສັນຍານທີ່ສະທ້ອນກັບມາທີ່ Echo pin.
- ໄລຍະເວລາທີ່ໃຊ້ຈະຖືກນຳມາຄຳນວນເປັນໄລຍະຫ່າງ.
- ຖ້າຫາກມີວັດຖຸເຂົ້າມາໃກ້ໃນໄລຍະທີ່ກຳນົດ (20 cm), Arduino ຈະສົ່ງ Output ໄປຫາ Servo ເພື່ອໃຫ້ມັນໝຸນໄປ 90 ອົງສາ. ຖ້າບໍ່ດັ່ງນັ້ນ, Servo ຈະກັບຄືນສູ່ຕຳແໜ່ງເດີມ.

---

### **ບົດທີ 5: LCD**
**ບົດນຳ**
ຈໍ LCD (Liquid Crystal Display) ເປັນອຸປະກອນສະແດງຜົນທີ່ນິຍົມໃຊ້ໃນໂຄງງານເອເລັກໂຕຣນິກ ເພື່ອສະແດງຂໍ້ມູນ ຫຼື ຂໍ້ຄວາມຕ່າງໆ. ໃນການທົດລອງນີ້, ເຮົາຈະໃຊ້ LCD ທີ່ມີໂມດູນ I2C ເພື່ອໃຫ້ການເຊື່ອມຕໍ່ງ່າຍຂຶ້ນ.

**ອຸປະກອນ**
- Microcontroller (Arduino)
- LCD 16x2 with I2C module
- Jumper wires

**ການເຊື່ອມຕໍ່ວົງຈອນ**
- **LCD I2C Module:**
    - GND → GND
    - VCC → 5V
    - SDA → A4 (ຫຼື SDA pin)
    - SCL → A5 (ຫຼື SCL pin)
 
<img width="452" height="602" alt="image" src="https://github.com/user-attachments/assets/6ed7e5b3-88b9-45b2-b0ab-21561d205ba9" />


**Code**
```cpp
// C++ code
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ກໍານົດ Address ຂອງ I2C ແລະ ຂະໜາດຂອງຈໍ (16 ຖັນ 2 ແຖວ)
LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup() {
  lcd.begin(); // ເລີ່ມຕົ້ນການເຮັດວຽກຂອງ LCD
  lcd.backlight(); // ເປີດໄຟພື້ນຫຼັງ
  lcd.setCursor(0, 0); // ຕັ້ງ Cursor ໄປທີ່ຖັນ 0, ແຖວ 0
  lcd.print("Hello, Anouphat!");
  lcd.setCursor(0, 1); // ຕັ້ງ Cursor ໄປທີ່ຖັນ 0, ແຖວ 1
  lcd.print("3 COM 2");
}

void loop() {
  // ບໍ່ມີໂຄດໃນ loop ເພາະສະແດງຂໍ້ຄວາມຄັ້ງດຽວ
}
```
**ການເຮັດວຽກຂອງວົງຈອນ**
- Arduino ສົ່ງຄຳສັ່ງ ແລະ ຂໍ້ຄວາມຜ່ານໂປຣໂຕຄໍ I2C ໄປຫາຈໍ LCD.
- ຈໍ LCD ຈະຮັບຂໍ້ມູນ ແລະ ສະແດງຂໍ້ຄວາມ "Hello, Anouphat!" ໃນແຖວທຳອິດ ແລະ "3 COM 2" ໃນແຖວທີສອງ.

---

### **ບົດທີ 6: Joystick**
**ບົດນຳ**
Joystick ເປັນອຸປະກອນປ້ອນຂໍ້ມູນທີ່ສາມາດກວດຈັບການເຄື່ອນໄຫວໃນ 2 ແກນ (X ແລະ Y) ແລະ ມີປຸ່ມກົດ. ເຮົາສາມາດໃຊ້ມັນເພື່ອຄວບຄຸມເກມ, ຫຸ່ນຍົນ ຫຼື ອຸປະກອນອື່ນໆ.

**ອຸປະກອນ**
- Arduino UNO
- Joystick Module
- 4 LEDs
- 4 Resistors (220Ω)
- Breadboard ແລະ ສາຍ jumper

**ການເຊື່ອມຕໍ່ວົງຈອນ**
- **Joystick:**
    - VCC → 5V
    - GND → GND
    - VRx (ແກນ X) → A0
    - VRy (ແກນ Y) → A1
    - SW (ປຸ່ມກົດ) → D2
- **LEDs:**
    - Pin 8 → LED Up
    - Pin 9 → LED Right
    - Pin 10 → LED Left
    - Pin 11 → LED Down
    - ຂາລົບຂອງ LED ທຸກຕົວຕໍ່ຜ່ານ Resistor ແລ້ວລົງ GND.
 
<img width="279" height="203" alt="image" src="https://github.com/user-attachments/assets/49f6f286-1729-4a41-8d01-2c5230c6fd26" />


**Code**
```cpp
// C++ code
const int SW_pin = 2; 
const int X_pin = A0; 
const int Y_pin = A1; 

#define UP_LED 8
#define RIGHT_LED 9
#define LEFT_LED 10
#define DOWN_LED 11

void setup() {
  pinMode(SW_pin, INPUT_PULLUP);
  Serial.begin(115200);

  pinMode(UP_LED, OUTPUT);
  pinMode(RIGHT_LED, OUTPUT);
  pinMode(LEFT_LED, OUTPUT);
  pinMode(DOWN_LED, OUTPUT);
}

void loop() {
  int xValue = analogRead(X_pin);
  int yValue = analogRead(Y_pin);
  int swValue = digitalRead(SW_pin);

  // Reset all LEDs
  digitalWrite(UP_LED, LOW);
  digitalWrite(RIGHT_LED, LOW);
  digitalWrite(LEFT_LED, LOW);
  digitalWrite(DOWN_LED, LOW);

  // Check Y-axis for Up/Down
  if (yValue < 100) {
    digitalWrite(UP_LED, HIGH);
  } else if (yValue > 900) {
    digitalWrite(DOWN_LED, HIGH);
  }

  // Check X-axis for Left/Right
  if (xValue < 100) {
    digitalWrite(LEFT_LED, HIGH);
  } else if (xValue > 900) {
    digitalWrite(RIGHT_LED, HIGH);
  }
  
  // Check switch press
  if (swValue == 0) {
    digitalWrite(UP_LED, HIGH);
    digitalWrite(LEFT_LED, HIGH);
    digitalWrite(RIGHT_LED, HIGH);
    digitalWrite(DOWN_LED, HIGH);
  }

  delay(100);
}
```
**ການເຮັດວຽກຂອງວົງຈອນ**
- Arduino ອ່ານຄ່າ Analog ຈາກແກນ X ແລະ Y, ແລະຄ່າ Digital ຈາກປຸ່ມກົດ.
- ເມື່ອ Joystick ຖືກຍູ້ໄປທິດທາງໃດໜຶ່ງ, ຄ່າ Analog ຈະປ່ຽນແປງ.
- ໂປຣແກຣມຈະກວດສອບຄ່າເຫຼົ່ານີ້ ແລະ ເປີດໄຟ LED ທີ່ສອດຄ້ອງກັບທິດທາງນັ້ນໆ (ຂຶ້ນ, ລົງ, ຊ້າຍ, ຂວາ).
- ເມື່ອກົດປຸ່ມ, ໄຟ LED ທຸກດວງຈະສະຫວ່າງຂຶ້ນ.

---

### **ບົດທີ 7: Bluetooth**
**ບົດນຳ**
ໂມດູນ Bluetooth (ເຊັ່ນ HC-05 ຫຼື HC-06) ຊ່ວຍໃຫ້ Arduino ສາມາດສື່ສານແບບໄຮ້ສາຍກັບອຸປະກອນອື່ນ ເຊັ່ນ: ໂທລະສັບສະມາດໂຟນ ຫຼື ຄອມພິວເຕີ, ເພື່ອຮັບຄຳສັ່ງ ຫຼື ສົ່ງຂໍ້ມູນ.

**ອຸປະກອນ**
- Microcontroller (Arduino)
- Bluetooth Module (HC-05/HC-06)
- LED
- Resistor (220Ω)
- Jumper wires
- Breadboard

**ການເຊື່ອມຕໍ່ວົງຈອນ**
- **Bluetooth Module:**
    - VCC → 5V
    - GND → GND
    - TXD → Arduino RX (Pin 0)
    - RXD → Arduino TX (Pin 1)
- **LED:**
    - ຂາບວກຕໍ່ກັບ Pin 13
    - ຂາລົບຕໍ່ຜ່ານ Resistor ລົງ GND
 
<img width="263" height="231" alt="image" src="https://github.com/user-attachments/assets/38a92ebc-ca14-43e4-abde-29415835c038" />


**Code**
```cpp
// C++ code
char data; // Variable to store received data

void setup() {
  Serial.begin(9600); // Start serial communication
  pinMode(13, OUTPUT); // Set LED pin as output
}

void loop() {
  if (Serial.available() > 0) { // Check if data is received
    data = Serial.read(); // Read the data
    
    if (data == '1') {
      digitalWrite(13, HIGH); // Turn LED on if '1' is received
      Serial.println("LED ON");
    } else if (data == '0') {
      digitalWrite(13, LOW); // Turn LED off if '0' is received
      Serial.println("LED OFF");
    }
  }
}
```
**ການເຮັດວຽກຂອງວົງຈອນ**
- Arduino ລໍຖ້າຮັບຂໍ້ມູນຜ່ານ Bluetooth.
- ເມື່ອມີຂໍ້ມູນເຂົ້າມາ (ເຊັ່ນ, ຈາກແອັບໃນໂທລະສັບ), Arduino ຈະອ່ານຂໍ້ມູນນັ້ນ.
- ຖ້າຂໍ້ມູນທີ່ໄດ້ຮັບແມ່ນ '1', ໄຟ LED ຈະເປີດ.
- ຖ້າຂໍ້ມູນທີ່ໄດ້ຮັບແມ່ນ '0', ໄຟ LED ຈະປິດ.
- Arduino ຍັງສົ່ງຂໍ້ຄວາມກັບຄືນໄປຫາອຸປະກອນທີ່ເຊື່ອມຕໍ່ເພື່ອຢືນຢັນສະຖານະ.

---

### **Midterm**

#### **Midterm: Lock System**
**ບົດນຳ**
ລະບົບນີ້ແມ່ນລະບົບລັອກອີເລັກໂທຣນິກທີ່ຕ້ອງໃສ່ລະຫັດຜ່ານ 3 ຕົວເລກໃຫ້ຖືກຕ້ອງເພື່ອປົດລັອກ. ຜູ້ໃຊ້ຈະເລືອກຕົວເລກໂດຍການໝຸນ Potentiometer ແລະ ຢືນຢັນດ້ວຍປຸ່ມກົດ.

**ອຸປະກອນ**
- Arduino
- Breadboard
- Switch button
- Potentiometer
- 7-segment display
- Servo motor
- 2 LEDs (ສີແດງ, ສີຟ້າ)
- Buzzer
- Resistors

**ວິທີການເຊື່ອມຕໍ່**
1.  **7-segment display:** ເຊື່ອມຕໍ່ pins a-g ກັບ pins 3-9 ຂອງ Arduino.
2.  **LEDs:**
    - LED ສີຟ້າ (ຖືກຕ້ອງ) → Pin 10.
    - LED ສີແດງ (ຜິດ) → Pin 11.
3.  **Buzzer:** → Pin 12.
4.  **Button:** → Pin 2.
5.  **Potentiometer:**
    - VCC → 5V, GND → GND.
    - Wiper (ຂາກາງ) → A0.
6.  **Servo Motor:**
    - ສາຍສັນຍານ → Pin ທີ່ຮອງຮັບ PWM (ຕົວຢ່າງ Pin 9).
    - VCC → 5V, GND → GND.

<img width="256" height="167" alt="image" src="https://github.com/user-attachments/assets/86015f39-cdc0-4eda-a545-869b49e8dc77" />
<img width="326" height="245" alt="image" src="https://github.com/user-attachments/assets/f4259cf0-d0f0-4c7d-be4d-e5aff52d646f" />



**Code**
```cpp
// C++ code
#include <Servo.h>

// Pin definitions
const int blueLEDPin = 10;
const int redLEDPin = 11;
const int buzzerPin = 12;
const int buttonPin = 2;
const int potentiometerPin = A0;
const int servoPin = 9; // Servo pin

// 7-segment display pins
const int segmentPins[7] = {3, 4, 5, 6, 7, 8, 9};
const byte digitPatterns[10][7] = {
  {1, 1, 1, 1, 1, 1, 0}, // 0
  {0, 1, 1, 0, 0, 0, 0}, // 1
  {1, 1, 0, 1, 1, 0, 1}, // 2
  {1, 1, 1, 1, 0, 0, 1}, // 3
  {0, 1, 1, 0, 0, 1, 1}, // 4
  {1, 0, 1, 1, 0, 1, 1}, // 5
  {1, 0, 1, 1, 1, 1, 1}, // 6
  {1, 1, 1, 0, 0, 0, 0}, // 7
  {1, 1, 1, 1, 1, 1, 1}, // 8
  {1, 1, 1, 1, 0, 1, 1}  // 9
};

Servo lockServo;
const int password[3] = {1, 3, 7};
int enteredPassword[3] = {0, 0, 0};
int currentIndex = 0;

void setup() {
  pinMode(blueLEDPin, OUTPUT);
  pinMode(redLEDPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  lockServo.attach(servoPin);
  lockServo.write(0); // Lock initially

  for (int i = 0; i < 7; i++) {
    pinMode(segmentPins[i], OUTPUT);
  }
  Serial.begin(9600);
}

void loop() {
  int potValue = analogRead(potentiometerPin);
  int currentDigit = map(potValue, 0, 1023, 0, 9);
  displayDigit(currentDigit);

  if (digitalRead(buttonPin) == LOW) {
    delay(200);
    enteredPassword[currentIndex] = currentDigit;
    currentIndex++;

    if (currentIndex == 3) {
      if (checkPassword()) {
        correctPassword();
      } else {
        wrongPassword();
      }
      resetPassword();
    }
  }
}

void displayDigit(int digit) {
  for (int i = 0; i < 7; i++) {
    digitalWrite(segmentPins[i], digitPatterns[digit][i]);
  }
}

bool checkPassword() {
  for (int i = 0; i < 3; i++) {
    if (enteredPassword[i] != password[i]) return false;
  }
  return true;
}

void correctPassword() {
  digitalWrite(blueLEDPin, HIGH);
  lockServo.write(90); // Unlock
  delay(2000);
  digitalWrite(blueLEDPin, LOW);
  lockServo.write(0); // Re-lock
}

void wrongPassword() {
  digitalWrite(redLEDPin, HIGH);
  tone(buzzerPin, 1000);
  delay(2000);
  noTone(buzzerPin);
  digitalWrite(redLEDPin, LOW);
}

void resetPassword() {
  currentIndex = 0;
}
```
**ການເຮັດວຽກຂອງວົງຈອນ**
- ໝຸນ potentiometer ເພື່ອເລືອກຕົວເລກ (0-9) ເຊິ່ງຈະສະແດງເທິງ 7-segment display.
- ກົດປຸ່ມເພື່ອຢືນຢັນຕົວເລກທີ່ເລືອກ.
- ເຮັດຊ້ຳຈົນຄົບ 3 ຕົວ.
- ຖ້າລະຫັດຖືກຕ້ອງ (1-3-7), LED ສີຟ້າຈະສະຫວ່າງ ແລະ Servo motor ຈະໝຸນເພື່ອ "ປົດລັອກ".
- ຖ້າລະຫັດຜິດ, LED ສີແດງຈະສະຫວ່າງ ແລະ Buzzer ຈະດັງ.
- ຫຼັງຈາກນັ້ນ, ລະບົບຈະຣີເຊັດເພື່ອໃຫ້ປ້ອນລະຫັດໃໝ່.

---

#### **Midterm: Sensor Motion**
**ບົດນຳ**
ລະບົບນີ້ແມ່ນລະບົບກວດຈັບການເຄື່ອນໄຫວແບບປະສົມປະສານ, ໂດຍໃຊ້ PIR sensor ເພື່ອກວດຈັບການມີຢູ່ຂອງຄົນ ແລະ ໃຊ້ Ultrasonic sensor ເພື່ອວັດແທກໄລຍະຫ່າງຂອງວັດຖຸທີ່ກວດພົບ.

**ອຸປະກອນ**
- Arduino Uno
- PIR Motion Sensor
- Ultrasonic Sensor (HC-SR04)
- LED (ສີແດງ)
- Resistor (220Ω)
- Breadboard
- Jumper wires

**ການເຊື່ອມຕໍ່ສາຍໄຟ**
- **PIR Sensor:**
    - VCC → 5V, GND → GND
    - OUT → Digital Pin 2
- **Ultrasonic Sensor:**
    - VCC → 5V, GND → GND
    - Trig → Digital Pin 9
    - Echo → Digital Pin 10
- **LED:**
    - ຂາບວກ → Digital Pin 13
    - ຂາລົບ → Resistor → GND

**Code**
```cpp
// C++ code
const int pirPin = 2;
const int trigPin = 9;
const int echoPin = 10;
const int ledPin = 13;

void setup() {
  Serial.begin(9600);
  pinMode(pirPin, INPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(ledPin, OUTPUT);
}

void loop() {
  int motionState = digitalRead(pirPin);

  if (motionState == HIGH) {
    Serial.println("Motion detected!");
    
    // Measure distance
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    
    long duration = pulseIn(echoPin, HIGH);
    int distance = duration * 0.034 / 2;
    
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");

    // Control LED based on distance
    if (distance < 100) { // If object is closer than 100cm
      digitalWrite(ledPin, HIGH);
    } else {
      digitalWrite(ledPin, LOW);
    }
    
  } else {
    Serial.println("No motion");
    digitalWrite(ledPin, LOW);
  }
  
  delay(500);
}
```

**ມັນເຮັດວຽກແນວໃດ**
1.  **ກວດຈັບການເຄື່ອນໄຫວ:** Arduino ກວດສອບສະຖານະຂອງ PIR sensor.
2.  **ຖ້າກວດພົບການເຄື່ອນໄຫວ:**
    - ມັນຈະພິມ "Motion detected!" ອອກທາງ Serial Monitor.
    - ຫຼັງຈາກນັ້ນ, ມັນຈະໃຊ້ Ultrasonic sensor ເພື່ອວັດແທກໄລຍະຫ່າງ.
    - ຖ້າໄລຍະຫ່າງຂອງວັດຖຸທີ່ກວດພົບນ້ອຍກວ່າ 100 ຊັງຕີແມັດ, ໄຟ LED ຈະສະຫວ່າງ. ຖ້າໄກກວ່ານັ້ນ, LED ຈະດັບ.
3.  **ຖ້າບໍ່ມີການເຄື່ອນໄຫວ:**
    - ມັນຈະພິມ "No motion" ແລະ ໄຟ LED ຈະດັບ.
4.  ລະບົບຈະລໍຖ້າ 0.5 ວິນາທີກ່ອນທີ່ຈະກວດສອບອີກຄັ້ງ.

---

#### **Midterm: Automatic Elevator Door**
**ໂດຍ: ທ. ອົງກອນ ຂຸນພິທັກ**

**ບົດນຳ**
ເປັນການທົດລອງຈຳລອງການເຮັດວຽກຂອງປະຕູລິບອັດຕະໂນມັດ. ລະບົບຈະໃຊ້ PIR sensor ເພື່ອກວດຈັບຄົນທີ່ລໍຖ້າຢູ່ໜ້າລິບ, ແລະ ໃຊ້ Ultrasonic sensor ເພື່ອກວດຈັບວ່າຄົນໄດ້ເຂົ້າໄປໃນລິບແລ້ວຫຼືຍັງ ເພື່ອຄວບຄຸມການເປີດ-ປິດປະຕູດ້ວຍ Servo motor.

**ອຸປະກອນ**
- Arduino
- Breadboard
- Ultrasonic sensor
- PIR sensor
- Servo motor
- 2 LEDs (ສຳລັບບອກຊັ້ນ)
- Resistors
- Jumper wires

**ການເຊື່ອມຕໍ່ວົງຈອນ**
- **PIR Sensor:** OUT → Pin 2
- **Ultrasonic Sensor:** Trig → Pin 3, Echo → Pin 4
- **Servo Motor:** Signal → Pin 9
- **LEDs:** LED ຊັ້ນ 1 → Pin 5, LED ຊັ້ນ 2 → Pin 6

<img width="348" height="258" alt="image" src="https://github.com/user-attachments/assets/5903f920-8fc1-47e4-9fa9-39b9cbdef487" />


**Code**
```cpp
// C++ code
#include <Servo.h>

const int pirPin = 2;
const int trigPin = 3;
const int echoPin = 4;
const int servoPin = 9;

const int led1Pin = 5; // Floor 1 LED
const int led2Pin = 6; // Floor 2 LED

Servo doorServo;

bool doorOpen = false;
bool personInside = false;
int currentFloor = 1; // Start at floor 1

void setup() {
  Serial.begin(9600);
  pinMode(pirPin, INPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(led1Pin, OUTPUT);
  pinMode(led2Pin, OUTPUT);

  doorServo.attach(servoPin);
  doorServo.write(0); // Door closed at start
  updateFloorLEDs();
}

void loop() {
  int motionDetected = digitalRead(pirPin);

  // If motion is detected outside, door is closed, and elevator is at floor 1
  if (motionDetected && !doorOpen && !personInside && currentFloor == 1) {
    Serial.println("Motion Detected at Floor 1 - Opening Door");
    doorServo.write(90); // Open door
    doorOpen = true;
    delay(3000); // Wait for person to enter
  }

  // If door is open, check if someone has entered
  if (doorOpen) {
    long duration, distance;
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH);
    distance = duration * 0.034 / 2;

    Serial.print("Distance: ");
    Serial.println(distance);

    // If person is detected inside (close range)
    if (distance < 20 && distance > 0) {
      Serial.println("Person Detected Inside - Closing Door");
      doorServo.write(0); // Close door
      doorOpen = false;
      personInside = true;
      delay(1000); // Wait before moving
      simulateElevatorMovement();
    }
  }
}

void simulateElevatorMovement() {
  Serial.println("Moving to Floor 2...");
  currentFloor = 2;
  updateFloorLEDs();
  delay(3000); // Simulate travel time
  
  Serial.println("Arrived at Floor 2 - Opening door");
  doorServo.write(90); // Open door at Floor 2
  delay(3000); // Wait for person to exit
  
  Serial.println("Closing door at Floor 2");
  doorServo.write(0); // Close door
  personInside = false;
  
  // Simulate returning to floor 1
  Serial.println("Returning to Floor 1...");
  currentFloor = 1;
  updateFloorLEDs();
  delay(3000);
  Serial.println("Arrived at Floor 1");
}

void updateFloorLEDs() {
  digitalWrite(led1Pin, (currentFloor == 1) ? HIGH : LOW);
  digitalWrite(led2Pin, (currentFloor == 2) ? HIGH : LOW);
}
```
**ການເຮັດວຽກຂອງວົງຈອນ**
1.  **ລໍຖ້າຄົນ:** ລະບົບເລີ່ມຕົ້ນທີ່ຊັ້ນ 1 (ໄຟ LED ຊັ້ນ 1 ຕິດ) ແລະ ປະຕູປິດ. ມັນຈະລໍຖ້າ PIR sensor ກວດຈັບການເຄື່ອນໄຫວ.
2.  **ເປີດປະຕູ:** ເມື່ອມີຄົນມາຢູ່ໜ້າລິບ (PIR ກວດພົບ), Servo ຈະໝຸນເພື່ອເປີດປະຕູ.
3.  **ກວດຈັບຄົນເຂົ້າລິບ:** ຫຼັງຈາກປະຕູເປີດ, Ultrasonic sensor ຈະເລີ່ມວັດແທກໄລຍະຫ່າງ. ຖ້າກວດພົບວັດຖຸໃນໄລຍະໃກ້ (ຄົນເຂົ້າໄປຂ້າງໃນ), Servo ຈະໝຸນເພື່ອປິດປະຕູ.
4.  **ເຄື່ອນທີ່:** ຫຼັງຈາກປະຕູປິດ, ລະບົບຈະຈຳລອງການເຄື່ອນທີ່ໄປຊັ້ນ 2 (ໄຟ LED ຊັ້ນ 2 ຕິດ).
5.  **ຮອດຈຸດໝາຍ:** ເມື່ອຮອດຊັ້ນ 2, ປະຕູຈະເປີດເພື່ອໃຫ້ຄົນອອກ, ລໍຖ້າໄລຍະໜຶ່ງ, ແລ້ວປິດປະຕູ.
6.  **ກັບຄືນ:** ຫຼັງຈາກນັ້ນ, ລິບຈະຈຳລອງການກັບຄືນມາທີ່ຊັ້ນ 1 ເພື່ອລໍຖ້າຮັບໃຊ້ຄົນຕໍ່ໄປ.

---

#### **Midterm: Ultrasonic Radar Display (URD)**
**ໂດຍ: ທ. ອານຸພັດ ຈິນດາຮັກ**

**ບົດນຳ**
ວົງຈອນນີ້ແມ່ນການທົດລອງກວດຈັບວັດຖຸທີ່ເຂົ້າມາໃກ້ໂດຍໃຊ້ເຊັນເຊີ Ultrasonic, ເມື່ອກວດຈັບວັດຖຸໄດ້ແລ້ວຈະໃຫ້ LED 8x8 ສະແດງສັນຍາລັກຕາມໄລຍະຫ່າງ.

**ອຸປະກອນ**
- Microcontroller (Arduino)
- Bread board
- Ultrasonic sensor (HC-SR04)
- 8x8 DOT Matrix Display
- LED
- Resistor
- Jumper

**ການເຊື່ອມຕໍ່ວົງຈອນ**
- **Ultrasonic Sensor (HC-SR04):**
    - VCC → 5V
    - GND → GND
    - Trig → Pin 7
    - Echo → Pin 6
- **8x8 Dot Matrix:**
    - VCC → 5V
    - GND → GND
    - DIN → Pin 11
    - CS → Pin 10
    - CLK → Pin 13
- **LED:**
    - ຂາບວກ (Anode) → Pin 5
    - ຂາລົບ (Cathode) → Resistor → GND

![ພາບການເຊື່ອມຕໍ່ຕົວຈິງ](https://i.imgur.com/kGZfV1g.jpeg)

**Code**
```cpp
#include <LedControl.h>

// Pin Mapping
LedControl lc = LedControl(11, 13, 10, 1); // DIN, CLK, CS, Number of devices

#define TRIG_PIN 7
#define ECHO_PIN 6
#define LED_PIN 5

void setup() {
  lc.shutdown(0, false);
  lc.setIntensity(0, 8);
  lc.clearDisplay(0);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(115200);
}

void loop() {
  long duration;
  int distance;

  // Trigger ultrasonic pulse
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Read echo pulse
  duration = pulseIn(ECHO_PIN, HIGH);
  distance = duration * 0.034 / 2;

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  // แสดงผลตามระยะทาง
  if (distance > 0 && distance <= 10) { // ระยะใกล้มาก
    showWarning();
    digitalWrite(LED_PIN, HIGH);
  } else if (distance > 10 && distance <= 20) { // ระยะกลาง
    showDot();
    digitalWrite(LED_PIN, LOW);
  } else if (distance > 20 && distance <= 30) { // ระยะปานกลางค่อนไปไกล
    showLine();
    digitalWrite(LED_PIN, LOW);
  } else { // ระยะไกลมาก
    showFar();
    digitalWrite(LED_PIN, LOW);
  }

  delay(300);
}

// เตือน: ระยะ <= 10 cm
void showWarning() {
  byte warning[8] = {
    0x00, 0x00, 0x00, 0x18, 0x18, 0x00, 0x00, 0x00
  };
  for (int i = 0; i < 8; i++) {
    lc.setRow(0, i, warning[i]);
  }
}

// จุดกลาง: ระยะ 11-20 cm
void showDot() {
  byte dot[8] = {
    0x00, 0x00, 0x00, 0x3c, 0x3c, 0x24, 0x3c, 0x00
  };
  for (int i = 0; i < 8; i++) {
    lc.setRow(0, i, dot[i]);
  }
}

// เส้นแนวนอน: ระยะ 21-30 cm
void showLine() {
  byte line[8] = {
    0x00, 0x00, 0x7e, 0x42, 0x42, 0x42, 0x7e, 0x00
  };
  for (int i = 0; i < 8; i++) {
    lc.setRow(0, i, line[i]);
  }
}

// วงกลมรอบนอก: ระยะ >30 cm
void showFar() {
  byte farSymbol[8] = {
    0xff, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0xff
  };
  for (int i = 0; i < 8; i++) {
    lc.setRow(0, i, farSymbol[i]);
  }
}
```

**ການເຮັດວຽກຂອງວົງຈອນ**
1.  **ວັດແທກໄລຍະ:** ວົງຈອນໃຊ້ເຊັນເຊີ Ultrasonic ເພື່ອວັດແທກໄລຍະຫ່າງຈາກເຊັນເຊີໄປຫາວັດຖຸທີ່ຢູ່ດ້ານໜ້າ.
2.  **ສະແດງຜົນ:**
    * **ໄລຍະ 1-10 cm:** ສະແດງສັນຍາລັກ "ເຕືອນ" (!) ເທິງຈໍ Dot Matrix ແລະ ເປີດໄຟ LED.
    * **ໄລຍະ 11-20 cm:** ສະແດງສັນຍາລັກ "ຈຸດ" ເທິງຈໍ Dot Matrix.
    * **ໄລຍະ 21-30 cm:** ສະແດງສັນຍາລັກ "ເສັ້ນ" ເທິງຈໍ Dot Matrix.
    * **ໄລຍະ > 30 cm:** ສະແດງສັນຍາລັກ "ວົງກົມ" ເທິງຈໍ Dot Matrix.
3.  **ອັບເດດ:** ລະບົບຈະທຳການວັດແທກ ແລະ ອັບເດດການສະແດງຜົນໃໝ່ທຸກໆ 300 ມິນລິວິນາທີ.

---

#### **Midterm: Flood Alert**
**ໂດຍ: ທ. ວິລະພົນ ຄຳວົງທອງ**

**ບົດນຳ**
ແມ່ນວົງຈອນກວດການລະດັບນ້ຳໂດຍມີການແຈ້ງເຕືອນໄພແລ້ວນຳໄປປະຍຸກໃຊ້ໃນສະຖານະການຈິງເພື່ອຈະປ້ອງກັນການສູນເສຍ ແລະ ຕະກຽມຈາກໄພນ້ຳຖ້ວມ.

**ອຸປະກອນ**
- Breadboard
- Arduino
- Ultrasonic sensor
- Buzzer
- Resistor
- LED
- Jumper wire

**ການເຊື່ອມຕໍ່ວົງຈອນ**
- **Ultrasonic Sensor (HC-SR04):**
    - VCC → 5V
    - GND → GND
    - Trig → Pin 9
    - Echo → Pin 10
- **LEDs:**
    - LED ສີຂຽວ (ປອດໄພ) → Pin 2
    - LED ສີເຫຼືອງ (ເຕືອນ) → Pin 3
    - LED ສີແດງ (ອັນຕະລາຍ) → Pin 4
    - ຂາລົບຂອງ LED ທຸກດອກຕໍ່ຜ່ານ Resistor ແລ້ວລົງ GND.
- **Buzzer:**
    - ຂາບວກ (+) → Pin 8
    - ຂາລົບ (-) → GND

![ພາບການເຊື່ອມຕໍ່ຕົວຈິງ](https://i.imgur.com/A6bJj9J.jpeg)

**Code**
```cpp
#define TRIG_PIN 9
#define ECHO_PIN 10
#define GREEN_LED 2
#define YELLOW_LED 3
#define RED_LED 4
#define BUZZER 8

void setup() {
  Serial.begin(9600);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(BUZZER, OUTPUT);
}

void loop() {
  long duration;
  float distance;

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  duration = pulseIn(ECHO_PIN, HIGH);
  distance = duration * 0.034 / 2;

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  if (distance > 80) {
    digitalWrite(GREEN_LED, HIGH);
    digitalWrite(YELLOW_LED, LOW);
    digitalWrite(RED_LED, LOW);
    digitalWrite(BUZZER, LOW);
  } else if (distance > 50 && distance <= 80) {
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(YELLOW_LED, HIGH);
    digitalWrite(RED_LED, LOW);
    digitalWrite(BUZZER, LOW);
  } else {
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(YELLOW_LED, LOW);
    digitalWrite(RED_LED, HIGH);
    digitalWrite(BUZZER, HIGH);
  }
  delay(1000);
}
```

**ການເຮັດວຽກຂອງວົງຈອນ**
1.  **ເລີ່ມເຮັດວຽກ:** ວົງຈອນເລີ່ມເຮັດວຽກທັນທີເມື່ອອັບໂຫຼດ code ລົງ arduino.
2.  **ວັດແທກລະດັບນ້ຳ:** Ultrasonic ສົ່ງຄື້ນສຽງອອກເພື່ອວັດແທກໄລຍະຫ່າງຈາກເຊັນເຊີຫາໜ້ານ້ຳ.
3.  **ແຈ້ງເຕືອນ:** ວົງຈອນຈະສົ່ງສຽງ ແລະ ສະແດງໄຟສີຕ່າງໆຕາມລະດັບຄວາມເລິກຂອງນ້ຳທີ່ວັດໄດ້:
    * **ໄລຍະຫ່າງ > 80 cm (ນ້ຳໜ້ອຍ):** ໄຟສີຂຽວຕິດ.
    * **ໄລຍະຫ່າງ 51-80 cm (ນ້ຳປານກາງ):** ໄຟສີເຫຼືອງຕິດ.
    * **ໄລຍະຫ່າງ <= 50 cm (ນ້ຳຫຼາຍ/ອັນຕະລາຍ):** ໄຟສີແດງຕິດ ແລະ Buzzer ດັງ.

