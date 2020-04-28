#include <Arduino.h>
#include <EEPROM.h>
#define VERSION "\n\nMPBSZ IZH YUPITER 5 BY WASILIYSOFT v0.4.1 28.04.2020 \n\n"
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
  - в режиме корректировки УОЗ увеличивает смещение УОЗ

  PIN_BTN_UOZ_DOWN
  - отвечает за вход в режим установки начального УОЗ (настройка шторки)
  - в режиме настройки кторки переключает режим оповещения
  - в режиме корректировки УОЗ уменьшает смещение УОЗ

  Двойной сигнал при включении - режим корректировки УОЗ,
  в этом режиме работают кнопки корректировки УОЗ можно
  корректировать на заведенном двигателе, после установки
  перезапустить arduino для снижения задержек

  Четверной сигнал при включении - режим установки УОЗ (настройка шторки)
  в этом режиме при сработке датчика будет пищать динамик и загораться
  диод состояния, кнопка PIN_BTN_UOZ_DOWN переключает режим оповещения

  "Большое спасибо" можно отправить на Яндекс деньги
  https://money.yandex.ru/to/41001180308919
*/

//---- ОБЩИЕ НАСТРОЙКИ ----//
/*
  Ограничитель оборотов, работает только если MIN_ROTATION_TIME больше 0
  для ограничения обортов необходимо ввести, минимальное ВРЕМЯ оборота в
  микросекундах.

  10000 мкс  = 6000 об/мин
  12000 мкс  = 5000 об/мин
  15000 мкс  = 4000 об/мин
  20000 мкс  = 3000 об/мин
  30000 мкс  = 2000 об/мин
*/
#define MIN_ROTATION_TIME 12000

// Через сколько оборотов выключить режим вспышки в ВМТ без опережения
#define VMT_MODE_OFF_IMPULSE 100

// Размер лепестка 30 либо 60 градусов
#define MODULATOR 30
// Количество лепестков модулятора
#define PETALS 2

// Максимальное уменьшение УОЗ при корректировке с помощью кнопок.
// Не рекомендуются значения более 30 градусов.
// Шаг коррекции = 5 градусов
#define MAX_UOZ_OFFSET 20

//---- КОНСТАНТЫ ПИНОВ
#define PIN_BTN_UOZ_UP 5 // пин кнопки увеличения угла опережения
#define PIN_BTN_UOZ_DOWN 6 // пин кнопки уменьшения угла опережения

#define PIN_LED_MODE 12 // пин светодиода индикации режима работы
#define PIN_BUZZER A5 // пин динамика
#define PIN_BOBBIN 8  // пин катушки

//---- КОНСТАНТЫ ПЕРЕКЛЮЧЕНИЯ ПОРТОВ
// байт включения катушки (большой диод горит)
#define bobbinOnVMT PORTB = B00010000;
// байт вЫключения катушки (большой диод горит)
#define bobbinOffVMT PORTB = B00110001;

// байт включения катушки (большой диод НЕ горит)
#define bobbinOn PORTB = B00000000;
// байт вЫключения катушки (большой диод НЕ горит)
#define bobbinOff PORTB = B00100001;

//---- ПЕРЕМЕННЫЕ
// положение шторки
volatile bool g_state = 0;

// режим искры в ВМТ без задержек, по границе ВЫХОДА шторки
bool g_vmt_mode = true;

// флаг доступности кнопок корректировки УОЗ
bool g_btn_uoz_enabled = false;

// флаг настройки шторки, будет пищать когда шторка в модуляторе
bool g_installation_mode = false;

// флаг состояния кнопки
bool g_bPressed = false;

// ячейка памяти хрянящая uoz_offset
#define uoz_offset_address 1

// смещение УОЗ , чем значение больше тем меньше УОЗ
int g_uoz_offset = 0;

// момент прихода последней шторки
volatile unsigned long g_cur_time = 0;

// момент последнего полного оборота
unsigned long g_last_time = 0;

// время полного оборота КВ
unsigned long g_rotation_time = 0;

// расчетное время задержки относительно ВХОДА шторки,
// оно же определяет УОЗ
unsigned long g_delay_time = 0;

// расчетное время простоя катушки
unsigned long g_bobbin_off_time = 0;

// используется для определения полного оборота
// когда будет равно количеству PETALS значит полный оборот
unsigned int g_rotation_counter = 0;

void blink();
void oneBeep();
void doubleBeep();
void vmtMode(bool newState);

void setup() {
  Serial.begin(9600);
  Serial.println(VERSION);

  pinMode(PIN_BOBBIN, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(PIN_LED_MODE, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);

  pinMode(PIN_BTN_UOZ_UP, INPUT_PULLUP);
  pinMode(PIN_BTN_UOZ_DOWN, INPUT_PULLUP);
  delay(100);

  // проверка пина включения кнопок корректировки УОЗ
  if (digitalRead(PIN_BTN_UOZ_UP) == LOW) {
    g_btn_uoz_enabled = true;
    Serial.println("\nUOZ CORRECTION MODE\n");
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

  vmtMode(true);

  int uoz_offset = EEPROM.read(uoz_offset_address);
  g_uoz_offset = constrain(uoz_offset, 0, MAX_UOZ_OFFSET);

  Serial.print("UOZ OFFSET: -");
  Serial.println(g_uoz_offset);

  Serial.print("MIN_ROTATION_TIME: ");
  Serial.print(MIN_ROTATION_TIME);
  Serial.println(" microseconds");

#if MIN_ROTATION_TIME != 0
  Serial.print("RPM LIMIT: ");
  Serial.println((unsigned long)60000000 / MIN_ROTATION_TIME);
#else
  Serial.println("RPM UNLIMIT !!!");
#endif

  Serial.println("Ready");
  oneBeep();
}

void loop() {
  if (g_state == 1) {

    if (g_vmt_mode) { // режим искры в ВМТ по границе ВЫХОДА шторки
      if (g_installation_mode == true) { // режим настройки шторки
        static bool beepMode = false;
        if (digitalRead(PIN_BTN_UOZ_DOWN) == LOW) {
          // инверсия режима сигнализации шторки
          beepMode = !beepMode;
          oneBeep();
          delay(500);
        }
        if (digitalRead(3) == beepMode) {
          digitalWrite(PIN_LED_MODE, HIGH);
          digitalWrite(PIN_BUZZER, HIGH);
        } else {
          digitalWrite(PIN_BUZZER, LOW);
          digitalWrite(PIN_LED_MODE, LOW);
        }
        return;
      } else {
        bobbinOffVMT; // вспышка
        delay(4); // продолжительность простоя катушки
        bobbinOnVMT; // заряд
        // проверка, не пора ли вЫключить режим ВМТ
        static unsigned int in_count = 0;
        if (in_count > VMT_MODE_OFF_IMPULSE) {
          vmtMode(false);
        }
        in_count++;
      }

    } else { // нормальный режим работы с УОЗ

#if MIN_ROTATION_TIME != 0
      if (g_rotation_time > MIN_ROTATION_TIME) {
#endif
        // запаздывание искры
        if (g_delay_time > 16382) {
          delay((unsigned long)(g_delay_time / 1000));
        } else {
          delayMicroseconds(g_delay_time);
        }
        bobbinOff; // вспышка

        // продолжительность отключения катушки
        if (g_bobbin_off_time > 16382) {
          delay((unsigned long)(g_bobbin_off_time / 1000));
        } else {
          delayMicroseconds(g_bobbin_off_time);
        }
        bobbinOn; // заряд

#if MIN_ROTATION_TIME != 0
      }
#endif
    }
    // здесь можно выполнить некоторые действия
    // пока еще не пришла следующая шторка

    g_rotation_counter++;
    if (g_rotation_counter == PETALS) { // полный оборот
      g_rotation_counter = 0;
      // расчет времени оборота
      g_rotation_time = (unsigned long)(g_cur_time - g_last_time);

      // выбор времени задержки вспышки относительно
      // ВХОДА шторки модулятора, чем меньше задержка тем больше УОЗ

      // ######################################################
      // ######################################################
      // СЮДА ВСТАВЛЯЕТСЯ КОД ИЗ ТАБЛИЦЫ РАСЧЕТА УОЗ
#if MODULATOR == 30
      if (g_rotation_time < 10000) {
        g_delay_time = 123; // RPM 6000 UOZ +25,58
      } else if (g_rotation_time < 10909) {
        g_delay_time = 155; // RPM 5500 UOZ +24,88
      } else if (g_rotation_time < 12000) {
        g_delay_time = 195; // RPM 5000 UOZ +24,15
      } else if (g_rotation_time < 13333) {
        g_delay_time = 245; // RPM 4500 UOZ +23,39
      } else if (g_rotation_time < 15000) {
        g_delay_time = 308; // RPM 4000 UOZ +22,6
      } else if (g_rotation_time < 17143) {
        g_delay_time = 392; // RPM 3500 UOZ +21,76
      } else if (g_rotation_time < 18462) {
        g_delay_time = 458; // RPM 3250 UOZ +21,07
      } else if (g_rotation_time < 20000) {
        g_delay_time = 535; // RPM 3000 UOZ +20,37
      } else if (g_rotation_time < 21818) {
        g_delay_time = 627; // RPM 2750 UOZ +19,65
      } else if (g_rotation_time < 24000) {
        g_delay_time = 740; // RPM 2500 UOZ +18,9
      } else if (g_rotation_time < 26667) {
        g_delay_time = 879; // RPM 2250 UOZ +18,13
      } else if (g_rotation_time < 30000) {
        g_delay_time = 1057; // RPM 2000 UOZ +17,32
      } else if (g_rotation_time < 32432) {
        g_delay_time = 1207; // RPM 1850 UOZ +16,6
      } else if (g_rotation_time < 35294) {
        g_delay_time = 1385; // RPM 1700 UOZ +15,87
      } else if (g_rotation_time < 38710) {
        g_delay_time = 1601; // RPM 1550 UOZ +15,11
      } else if (g_rotation_time < 42857) {
        g_delay_time = 1865; // RPM 1400 UOZ +14,33
      } else if (g_rotation_time < 48000) {
        g_delay_time = 2197; // RPM 1250 UOZ +13,52
      } else if (g_rotation_time < 54545) {
        g_delay_time = 2626; // RPM 1100 UOZ +12,67
      } else if (g_rotation_time < 63158) {
        g_delay_time = 3198; // RPM 950 UOZ +11,77
      } else if (g_rotation_time < 75000) {
        g_delay_time = 4000; // RPM 800 UOZ +10,8
      } else if (g_rotation_time < 92308) {
        g_delay_time = 5197; // RPM 650 UOZ +9,73
      } else if (g_rotation_time < 120000) {
        g_delay_time = 7170; // RPM 500 UOZ +8,49
      } else if (g_rotation_time < 171429) {
        g_delay_time = 10986; // RPM 350 UOZ +6,93
      } else if (g_rotation_time < 300000) {
        g_delay_time = 21158; // RPM 200 UOZ +4,61
      } else if (g_rotation_time < 1200000) {
        g_delay_time = 95000; // RPM 50 UOZ +1,5
      }
#else
      if (g_rotation_time < 10000) {
        g_delay_time = 956; // RPM 6000 UOZ +25,58
      } else if (g_rotation_time < 10909) {
        g_delay_time = 1064; // RPM 5500 UOZ +24,88
      } else if (g_rotation_time < 12000) {
        g_delay_time = 1195; // RPM 5000 UOZ +24,15
      } else if (g_rotation_time < 13333) {
        g_delay_time = 1356; // RPM 4500 UOZ +23,39
      } else if (g_rotation_time < 15000) {
        g_delay_time = 1558; // RPM 4000 UOZ +22,6
      } else if (g_rotation_time < 17143) {
        g_delay_time = 1821; // RPM 3500 UOZ +21,76
      } else if (g_rotation_time < 18462) {
        g_delay_time = 1996; // RPM 3250 UOZ +21,07
      } else if (g_rotation_time < 20000) {
        g_delay_time = 2202; // RPM 3000 UOZ +20,37
      } else if (g_rotation_time < 21818) {
        g_delay_time = 2445; // RPM 2750 UOZ +19,65
      } else if (g_rotation_time < 24000) {
        g_delay_time = 2740; // RPM 2500 UOZ +18,9
      } else if (g_rotation_time < 26667) {
        g_delay_time = 3102; // RPM 2250 UOZ +18,13
      } else if (g_rotation_time < 30000) {
        g_delay_time = 3557; // RPM 2000 UOZ +17,32
      } else if (g_rotation_time < 32432) {
        g_delay_time = 3910; // RPM 1850 UOZ +16,6
      } else if (g_rotation_time < 35294) {
        g_delay_time = 4326; // RPM 1700 UOZ +15,87
      } else if (g_rotation_time < 38710) {
        g_delay_time = 4827; // RPM 1550 UOZ +15,11
      } else if (g_rotation_time < 42857) {
        g_delay_time = 5437; // RPM 1400 UOZ +14,33
      } else if (g_rotation_time < 48000) {
        g_delay_time = 6197; // RPM 1250 UOZ +13,52
      } else if (g_rotation_time < 54545) {
        g_delay_time = 7171; // RPM 1100 UOZ +12,67
      } else if (g_rotation_time < 63158) {
        g_delay_time = 8461; // RPM 950 UOZ +11,77
      } else if (g_rotation_time < 75000) {
        g_delay_time = 10250; // RPM 800 UOZ +10,8
      } else if (g_rotation_time < 92308) {
        g_delay_time = 12890; // RPM 650 UOZ +9,73
      } else if (g_rotation_time < 120000) {
        g_delay_time = 17170; // RPM 500 UOZ +8,49
      } else if (g_rotation_time < 171429) {
        g_delay_time = 25271; // RPM 350 UOZ +6,93
      } else if (g_rotation_time < 300000) {
        g_delay_time = 46158; // RPM 200 UOZ +4,61
      } else if (g_rotation_time < 1200000) {
        g_delay_time = 195000; // RPM 50 UOZ +1,5
      }
#endif
      // КОНЕЦ БЛОКА КОДА ИЗ ТАБЛИЦЫ РАСЧЕТА УОЗ
      // ######################################################
      // ######################################################

      // если ручная корректировка УОЗ не равна 0
      // то добавляем ко времени задержки смещение УОЗ
      // таким образом уменьшая УОЗ
      if (g_uoz_offset != 0) {
        g_delay_time +=
            (unsigned long)(((unsigned long)g_rotation_time * g_uoz_offset) /
                            360);
      }

      // расчет времени простоя катушки
      // время оборота КВ (360 градусов) делим на 6
      // получаем время простоя соответствующее 60 градусам
      g_bobbin_off_time = (unsigned long)(g_rotation_time / 6);
      g_last_time = g_cur_time;
    }
    g_state = 0;
  }

  //  Блок работы с кнопками корректировки УОЗ
  if (g_btn_uoz_enabled) {
    static unsigned long last_pressed = 0;
    if (~(PIND >> PIN_BTN_UOZ_DOWN) & B00000001) { // LOW
      if ((millis() - last_pressed) > 1000) {
        last_pressed = millis();
        g_uoz_offset = g_uoz_offset - 5;
        if (g_uoz_offset < 0) {
          g_uoz_offset = 0;
        }
        EEPROM.write(uoz_offset_address, g_uoz_offset);
        Serial.print("UOZ -");
        Serial.println(g_uoz_offset);
      }
    }

    if (~(PIND >> PIN_BTN_UOZ_UP) & B00000001) { // LOW
      if ((millis() - last_pressed) > 1000) {
        last_pressed = millis();
        g_uoz_offset = g_uoz_offset + 5;
        if (g_uoz_offset > MAX_UOZ_OFFSET) {
          g_uoz_offset = MAX_UOZ_OFFSET;
        }
        EEPROM.write(uoz_offset_address, g_uoz_offset);
        Serial.print("UOZ -");
        Serial.println(g_uoz_offset);
      }
    }
  }
}

/**
 * Функция прерывания по приходу шторки
 */
void blink() {
  g_cur_time = micros();
  g_state = 1;
}

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
