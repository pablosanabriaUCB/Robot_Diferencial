// ==========================================
// CALIBRACIÓN DE ENCODERS
// ==========================================
// Instrucciones:
// 1. Subir este sketch al ESP32
// 2. Abrir Monitor Serial (115200 baud)
// 3. Marcar una línea en la rueda
// 4. Girar la rueda exactamente 1 vuelta completa a mano (lento)
// 5. Anotar el número de ticks
// 6. Repetir para la otra rueda
// 7. Ese número es tu TICKS_PER_REV real
// ==========================================

const int ENC_L_A = 34;
const int ENC_L_B = 35;

const int ENC_R_A = 23;
const int ENC_R_B = 19;

volatile long ticks_L = 0;
volatile long ticks_R = 0;

void IRAM_ATTR countLeft()
{
  if (digitalRead(ENC_L_B))
    ticks_L++;
  else
    ticks_L--;
}

void IRAM_ATTR countRight()
{
  if (digitalRead(ENC_R_B))
    ticks_R--;
  else
    ticks_R++;
}

void setup()
{
  Serial.begin(115200);
  delay(1000);

  pinMode(ENC_L_A, INPUT);
  pinMode(ENC_L_B, INPUT);
  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_L_A), countLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), countRight, RISING);

  Serial.println("===========================");
  Serial.println("CALIBRACION DE ENCODERS");
  Serial.println("===========================");
  Serial.println("Gira cada rueda 1 vuelta completa");
  Serial.println("y anota los ticks.");
  Serial.println("Envia 'r' para resetear contadores.");
  Serial.println("===========================");
}

void loop()
{
  // Resetear con 'r'
  if (Serial.available())
  {
    char c = Serial.read();
    if (c == 'r' || c == 'R')
    {
      ticks_L = 0;
      ticks_R = 0;
      Serial.println(">> Contadores reseteados a 0");
    }
  }

  Serial.print("Ticks_L= ");
  Serial.print(ticks_L);
  Serial.print("    Ticks_R= ");
  Serial.println(ticks_R);

  delay(200);
}
