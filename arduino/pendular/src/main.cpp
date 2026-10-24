#include <Arduino.h>

// --- ПЕРШИЙ ЕНКОДЕР (5V) ---
const int enc1_pinA = 2; // Зелений дріт (переривання)
const int enc1_pinB = 3; // Білий дріт
volatile long encoder1Pos = 0;

// --- КРОКОВИЙ ДВИГУН ---
const int dirPin = 8;     // Напрямок (CW+)
const int stepPin = 9;    // Крок (CLK+)
const int STEPS_PER_REV = 1600;

// --- КІНЦЕВИК (Синій дріт у пін 20) ---
const int ENDSTOP_PIN = 20;
volatile bool endstopTriggered = false;
volatile int endstopState = HIGH;

// Прототипи функцій
void updateEncoder1();
void endstopISR();
void printStatus();
void stepMotor(int steps, uint16_t pulseDelayUs);

void setup() {
  Serial.begin(9600);

  // 1. Піни енкодера
  pinMode(enc1_pinA, INPUT_PULLUP);
  pinMode(enc1_pinB, INPUT_PULLUP);

  // 2. Піни драйвера двигуна
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);

  // 3. Кінцевик (вмикаємо внутрішню підтяжку до 5V)
  pinMode(ENDSTOP_PIN, INPUT_PULLUP);
  endstopState = digitalRead(ENDSTOP_PIN);

  // Апаратні зовнішні переривання
  // Переривання на пін 2 для енкодера
  attachInterrupt(digitalPinToInterrupt(enc1_pinA), updateEncoder1, RISING);

  // Апаратне переривання на пін 20 для кінцевика (реагує на будь-яку зміну: натиснуто/відпущено)
  attachInterrupt(digitalPinToInterrupt(ENDSTOP_PIN), endstopISR, CHANGE);

  Serial.println("Система готова: Енкодер (пін 2), Кінцевик (пін 20), Двигун (піни 8, 9).");
}

void loop() {
  // 1. Обертання за годинниковою стрілкою
  digitalWrite(dirPin, HIGH);
  stepMotor(STEPS_PER_REV, 500);
  printStatus();

  delay(1000);
  printStatus();

  // 2. Обертання проти годинникової стрілки
  digitalWrite(dirPin, LOW);
  stepMotor(STEPS_PER_REV, 500);
  printStatus();

  delay(1000);
  printStatus();
}

// Генерація імпульсів крокового двигуна
void stepMotor(int steps, uint16_t pulseDelayUs) {
  for (int i = 0; i < steps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(pulseDelayUs);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(pulseDelayUs);
  }
}

// Вивід стану енкодера та кінцевика в Serial Monitor
void printStatus() {
  static long lastPos1 = 0;

  noInterrupts();
  long current1 = encoder1Pos;
  bool triggered = endstopTriggered;
  int currentEndstop = endstopState;
  endstopTriggered = false; // Скидаємо прапорець
  interrupts();

  if (triggered) {
    if (currentEndstop == LOW) {
      Serial.println(">>> [КІНЦЕВИК 20]: НАТИСНУТО! (LOW / 0)");
    } else {
      Serial.println(">>> [КІНЦЕВИК 20]: Відпущено (HIGH / 1)");
    }
  }

  if (lastPos1 != current1) {
    Serial.print("Позиція Енкодера 1: ");
    Serial.println(current1);
    lastPos1 = current1;
  }
}

// --- ОБРОБНИКИ ПЕРЕРИВАНЬ ---

// Обробник переривання енкодера
void updateEncoder1() {
  if (digitalRead(enc1_pinB) == HIGH) {
    encoder1Pos++;
  } else {
    encoder1Pos--;
  }
}

// Апаратне переривання для кінцевика на піні 20
void endstopISR() {
  static unsigned long lastInterruptTime = 0;
  unsigned long currentTime = millis();

  // Програмний антибрязкіт: ігноруємо сигнали частіше ніж раз на 50 мс
  if (currentTime - lastInterruptTime > 50) {
    endstopState = digitalRead(ENDSTOP_PIN);
    endstopTriggered = true;
    lastInterruptTime = currentTime;
  }
}
