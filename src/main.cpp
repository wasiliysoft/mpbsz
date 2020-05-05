#include <Arduino.h>
#include <EEPROM.h>
#define VERSION "\nMPBSZ IZH YUPITER 5 BY WASILIYSOFT v0.5.1 03.05.2020\n"
/*
  Зажигание для мотоциклов ИЖ с оптическим датчиком

  Шторку устанавливать на выход из датчика примерно за 0,5-1 мм до ВМТ
  в дальнейшем УОЗ можно будет уменьшить программно с помощью кнопок

  Таблица расчета УОЗ
  https://docs.google.com/spreadsheets/d/1s24BqFf9aOlpx6sPj3IqwU2ER_uwyi742p3u0c8BsjQ/edit?usp=sharing

  При первом запуске после прошивки необходимо удерживать
  кнопку увеличение УОЗ и с помощью терминала убедиться
  что корректировка УОЗ установлена на 0 так как
  поумолчанию в энергонезависимой памяти новой arduino
  записано значение 255

  PIN_BTN_UOZ_UP
  - отвечает за вход в режим корректировки УОЗ
  - в режиме корректировки УОЗ увеличивает УОЗ

  PIN_BTN_UOZ_DOWN
  - отвечает за вход в режим установки начального УОЗ (настройка шторки)
  - в режиме настройки кторки переключает режим оповещения
  - в режиме корректировки УОЗ уменьшает УОЗ

  Двойной сигнал при включении - вход в режим корректировки УОЗ,

  Четверной сигнал при включении - вход режим установки УОЗ (настройка шторки)
  в этом режиме при сработке датчика будет пищать динамик и загораться
  диод состояния, кнопка PIN_BTN_UOZ_DOWN переключает режим оповещения

  "Большое спасибо" можно отправить на Яндекс деньги
  https://money.yandex.ru/to/41001180308919
*/

// Ограничитель оборотов, работает только если не равен 0
#define MAX_RPM 5000

// Через сколько оборотов выключить режим вспышки в ВМТ без опережения
#define VMT_MODE_OFF_IMPULSE 100

// Количество лепестков модулятора
#define PETALS 2

// Номер версии схемы сборки
#define REVISION 3

//---- КОНСТАНТЫ ПИНОВ
#if REVISION == 2
#define PIN_BTN_UOZ_DOWN 5 // пин кнопки уменьшения угла опережения
#define PIN_BTN_UOZ_UP 6 // пин кнопки увеличения угла опережения
#define PIN_BUZZER A5 // пин динамика
#elif REVISION == 3
#define PIN_BTN_UOZ_DOWN 4 // пин кнопки уменьшения угла опережения
#define PIN_BTN_UOZ_UP 7 // пин кнопки увеличения угла опережения
#define PIN_BUZZER 5 // пин динамика
#endif

#define PIN_LED_MODE 12 // пин светодиода индикации режима работы
#define PIN_BOBBIN 8 // пин катушки

//---- КОНСТАНТЫ ПЕРЕКЛЮЧЕНИЯ ПОРТОВ
// байт включения катушки (большой диод горит)
#define bobbinOnVMT PORTB = B00010000
// байт вЫключения катушки (большой диод горит)
#define bobbinOffVMT PORTB = B00110001

// байт включения катушки (большой диод НЕ горит)
#define bobbinOn PORTB = B00000000
// байт вЫключения катушки (большой диод НЕ горит)
#define bobbinOff PORTB = B00100001
//--------------------------------------
#if MAX_RPM != 0
#define MIN_ROTATION_TIME 60000000 / MAX_RPM
#define RPM_IF if (g_rotation_time > MIN_ROTATION_TIME) {
#define RPM_FI                                                                 \
  }                                                                            \
  else {                                                                       \
    PORTB = B00010000;                                                         \
  }
#else
#define RPM_IF
#define RPM_FI
#endif
//--------------------------------------

//---- ПЕРЕМЕННЫЕ
// положение шторки
volatile bool g_state = 0;

// используется для определения полного оборота
// когда будет равно количеству PETALS значит полный оборот
unsigned int in_count = 0;
unsigned int p = 0;

unsigned long cur_time = 0;
unsigned long last_time = 0;
// время полного оборота КВ
unsigned long g_rotation_time = 0;

// режим искры в ВМТ без задержек, по границе ВЫХОДА шторки
bool g_vmt_mode = true;

// флаг настройки шторки, будет пищать когда шторка в модуляторе
bool g_installation_mode = false;

// флаг доступности кнопок корректировки УОЗ
bool g_uoz_setting_mode = false;

// режим формирования УОЗ (-1|0|1)
int uoz_mode = 0;

// расчетное время задержки относительно ВХОДА шторки,
// оно же определяет УОЗ
unsigned long g_delay_time = 0;

// расчетное время простоя катушки
unsigned long g_bobbin_off_time = 0;

void blink();
void oneBeep();
void doubleBeep();
void vmtMode(bool newState);
void installation_mode();
void btnTick();

void setup() {
  Serial.begin(115200);
  Serial.println(VERSION);

  pinMode(PIN_BOBBIN, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(PIN_LED_MODE, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(3, INPUT_PULLUP);

  pinMode(PIN_BTN_UOZ_UP, INPUT_PULLUP);
  pinMode(PIN_BTN_UOZ_DOWN, INPUT_PULLUP);

  // проверка пина включения режима настройки формирования УОЗ
  if (digitalRead(PIN_BTN_UOZ_UP) == LOW) {
    g_uoz_setting_mode = true;
    Serial.println("\nUOZ SETTING MODE\n");
    doubleBeep();
    delay(2000);
  }

  // проверка пина включения режима настройки шторки
  if (digitalRead(PIN_BTN_UOZ_DOWN) == LOW) {
    g_installation_mode = true;
    Serial.println("\nINSTALATION MODE\n");
    doubleBeep();
    delay(200);
    doubleBeep();
    delay(2000);
  }

  uoz_mode = EEPROM.read(1);
  uoz_mode = constrain(uoz_mode, 0, 2);
  Serial.print("UOZ mode : ");
  Serial.println(uoz_mode, DEC);

  Serial.print("RPM LIMIT: ");
  Serial.println(MAX_RPM, DEC);
#ifdef MIN_ROTATION_TIME
  Serial.print("MIN ROTATION TIME: ");
  Serial.println(MIN_ROTATION_TIME, DEC);
#endif
  vmtMode(true);
  Serial.println("Ready!\n");
  oneBeep();
}

void loop() {

  if (g_installation_mode == true) { // режим настройки шторки
    for (;;) {
      installation_mode();
    }
  }

  if (g_uoz_setting_mode == true) { // режим настройки формирования УОЗ
    for (;;) {
      btnTick();
    }
  }
  for (;;) { // нормальный режим работы с УОЗ
    if (g_state == 1) {
      cur_time = micros();
      if (g_vmt_mode) { // режим искры в ВМТ по границе ВЫХОДА шторки
        bobbinOffVMT; // вспышка
        delay(4); // продолжительность простоя катушки
        bobbinOnVMT; // заряд
        // проверка, не пора ли вЫключить режим ВМТ
        if (++in_count > VMT_MODE_OFF_IMPULSE) {
          vmtMode(false);
          in_count = 0;
        }
      } else { // нормальный режим работы с УОЗ
        // Serial.println((unsigned long)(g_rotation_time), DEC);
        RPM_IF
        if (g_delay_time > 16383) {
          delay((unsigned long)g_delay_time / 1000);
        } else {
          delayMicroseconds(g_delay_time);
        }
        bobbinOff; // вспышка

        if (g_bobbin_off_time > 16383) {
          delay((unsigned long)g_bobbin_off_time / 1000);
        } else {
          delayMicroseconds(g_bobbin_off_time);
        }
        bobbinOn; // заряд
        RPM_FI
      }
      if (++p == PETALS) {
        p = 0;
        g_rotation_time = (unsigned long)(cur_time - last_time);
        if (g_rotation_time > 100000)
          vmtMode(true); // переводим в режим ВМТ
        last_time = cur_time;

        // расчет времени простоя катушки
        // делаем сдвиг на 3, получаем время 360/8 = 45 градусов
        g_bobbin_off_time = g_rotation_time >> 3;

        // выбор времени задержки вспышки относительно
        // ВХОДА шторки модулятора, чем меньше задержка тем больше УОЗ
        // ######################################################
        // СЮДА ВСТАВЛЯЕТСЯ КОД ИЗ ТАБЛИЦЫ РАСЧЕТА УОЗ
        // График САРУМАН обычный
        if (g_rotation_time < 12000) {
          g_delay_time = 1393; // RPM 5000 UOZ +18,2
        } else if (g_rotation_time < 12500) {
          g_delay_time = 1456; // RPM 4800 UOZ +18,08
        } else if (g_rotation_time < 13043) {
          g_delay_time = 1523; // RPM 4600 UOZ +17,96
        } else if (g_rotation_time < 13636) {
          g_delay_time = 1597; // RPM 4400 UOZ +17,84
        } else if (g_rotation_time < 14286) {
          g_delay_time = 1678; // RPM 4200 UOZ +17,72
        } else if (g_rotation_time < 15000) {
          g_delay_time = 1767; // RPM 4000 UOZ +17,6
        } else if (g_rotation_time < 15789) {
          g_delay_time = 1865; // RPM 3800 UOZ +17,48
        } else if (g_rotation_time < 16667) {
          g_delay_time = 1974; // RPM 3600 UOZ +17,36
        } else if (g_rotation_time < 17647) {
          g_delay_time = 2096; // RPM 3400 UOZ +17,24
        } else if (g_rotation_time < 18750) {
          g_delay_time = 2233; // RPM 3200 UOZ +17,12
        } else if (g_rotation_time < 20000) {
          g_delay_time = 2483; // RPM 3000 UOZ +15,3
        } else if (g_rotation_time < 21429) {
          g_delay_time = 2668; // RPM 2800 UOZ +15,18
        } else if (g_rotation_time < 23077) {
          g_delay_time = 2881; // RPM 2600 UOZ +15,06
        } else if (g_rotation_time < 25000) {
          g_delay_time = 3129; // RPM 2400 UOZ +14,94
        } else if (g_rotation_time < 27273) {
          g_delay_time = 3423; // RPM 2200 UOZ +14,82
        } else if (g_rotation_time < 30000) {
          g_delay_time = 3975; // RPM 2000 UOZ +12,3
        } else if (g_rotation_time < 33333) {
          g_delay_time = 4504; // RPM 1800 UOZ +11,36
        } else if (g_rotation_time < 37500) {
          g_delay_time = 5165; // RPM 1600 UOZ +10,42
        } else if (g_rotation_time < 42857) {
          g_delay_time = 6014; // RPM 1400 UOZ +9,48
        } else if (g_rotation_time < 50000) {
          g_delay_time = 7147; // RPM 1200 UOZ +8,54
        } else if (g_rotation_time < 60000) {
          g_delay_time = 8733; // RPM 1000 UOZ +7,6
        } else if (g_rotation_time < 75000) {
          g_delay_time = 11233; // RPM 800 UOZ +6,08
        } else if (g_rotation_time < 100000) {
          g_delay_time = 15400; // RPM 600 UOZ +4,56
        } else if (g_rotation_time < 150000) {
          g_delay_time = 23733; // RPM 400 UOZ +3,04
        } else if (g_rotation_time < 300000) {
          g_delay_time = 48733; // RPM 200 UOZ +1,52
        }
        // КОНЕЦ БЛОКА КОДА ИЗ ТАБЛИЦЫ РАСЧЕТА УОЗ
        // ######################################################
      }
      g_state = 0;
    }
  }
}

void installation_mode() {
  static bool beepMode = true;
  if (digitalRead(PIN_BTN_UOZ_DOWN) == LOW) {
    // инверсия режима сигнализации шторки
    beepMode = !beepMode;
    oneBeep();
    delay(500);
  }
  if (digitalRead(3) == beepMode) {
#if REVISION == 2
    digitalWrite(PIN_BUZZER, HIGH);
#elif REVISION == 3
    analogWrite(PIN_BUZZER, 1);
#endif
    digitalWrite(PIN_LED_MODE, HIGH);
  } else {
    digitalWrite(PIN_BUZZER, LOW);
    digitalWrite(PIN_LED_MODE, LOW);
  }
}

void btnTick() {
  static unsigned long last_pressed = 0;
  // Уменьшение опережения, увеличение задержки
  if (~(PIND >> PIN_BTN_UOZ_DOWN) & B00000001) { // LOW
    if ((millis() - last_pressed) > 1000) {
      last_pressed = millis();
      if (uoz_mode > 0) {
        uoz_mode--;
        EEPROM.write(1, uoz_mode);
      }
      // TODO сделать сигнал
      Serial.println(uoz_mode);
    }
  }

  // Увеличение опережения, сокразение задержки
  if (~(PIND >> PIN_BTN_UOZ_UP) & B00000001) { // LOW
    if ((millis() - last_pressed) > 1000) {
      last_pressed = millis();
      if (uoz_mode < 2) {
        uoz_mode++;
        EEPROM.write(1, uoz_mode);
      }
      // TODO сделать сигнал
      Serial.println(uoz_mode);
    }
  }
}
/**
 * Функция прерывания по приходу шторки
 */

void blink() { g_state = 1; }

/**
   Блок вспомогательных функций
*/
void oneBeep() {
  digitalWrite(PIN_LED_MODE, HIGH);
  digitalWrite(PIN_BUZZER, HIGH);
  delay(40);
  digitalWrite(PIN_LED_MODE, LOW);
  digitalWrite(PIN_BUZZER, LOW);
}

void doubleBeep() {
  oneBeep();
  delay(200);
  oneBeep();
}

/**
   Управлением режимом формирования УОЗ
*/
void vmtMode(bool newState) {
  g_vmt_mode = newState;
  // FALLING - выход шторки
  // RISING - вход шторки
  if (newState) {
    digitalWrite(PIN_LED_MODE, HIGH);
    attachInterrupt(1, blink, FALLING);
  } else {
    attachInterrupt(1, blink, RISING);
  }
}
