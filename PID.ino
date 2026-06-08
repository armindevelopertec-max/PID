#include <WiFi.h>
#include <WebSocketsServer.h>
#include <math.h>

const char* ssid = "TecRoot";
const char* password = "@dmin2147";

WebSocketsServer webSocket = WebSocketsServer(81);

#define IN1 22
#define IN2 19
#define IN3 18
#define IN4 21
#define ENA 25
#define ENB 26

const int C1L = 35;
const int C2L = 34;
const int C1R = 32;
const int C2R = 33;

const int PWM_FREQ = 5000;
const int PWM_RESOLUTION = 8;
const int PWM_DEADBAND = 235;

const int PULSES_PER_REV = 5600;
const float MAX_RPM = 180.0;
const float WHEEL_BASE = 0.135;
const float WHEEL_DIAMETER = 0.045;

// Constantes y variables de control (Movidas de nuevo aquí para visibilidad global)
const float Kp = 3.8, Ki = 2.5, Kd = 0.15;
const float ALPHA_RPM = 0.5;   
const float ALPHA_DERIV = 0.8; 
const float MAX_ACCEL = 200.0; 
const float K_SYNC = 0.05; 

double targetRPML = 0, targetRPMR = 0;
double rampL = 0, rampR = 0;
float integralL = 0, integralR = 0;
float derivL = 0, derivR = 0;
float derivHistoryL[3] = {0,0,0}, derivHistoryR[3] = {0,0,0};
int pwmL = 0, pwmR = 0;
int dirL = 0, dirR = 0;

volatile long nL = 0, nR = 0;
volatile int actL = 0, actR = 0;
volatile unsigned long lastIntL = 0, lastIntR = 0;

bool moviendoObjetivo = false;
double distObjetivo = 0.0;
double angObjetivo = 0.0;
double xInicial = 0.0, yInicial = 0.0, thetaInicial = 0.0;
int modoMovimiento = 0;
double velCruceroLineal = 0.0;
double velCruceroAngular = 0.0;

double velL = 0, velR = 0;
double filteredVelL = 0, filteredVelR = 0;
double prevRawVelL = 0, prevRawVelR = 0;

double robotX = 0, robotY = 0, robotTheta = 0;

unsigned long lastLoopMicros = 0;
unsigned long lastCmdTime = 0;
unsigned long lastTxTime = 0;

const unsigned long LOOP_PERIOD_US = 20000; 
const unsigned long WATCH_IDLE_MS = 2000;
const unsigned long TELEMETRY_MS = 150;

void IRAM_ATTR encoderL() {
  unsigned long now = micros();
  if (now - lastIntL < 100) return;
  int newL = (digitalRead(C1L) << 1) | digitalRead(C2L);
  int diff = (newL - actL) & 3;
  if (diff == 1) nL++; else if (diff == 3) nL--;
  actL = newL; lastIntL = now;
}

void IRAM_ATTR encoderR() {
  unsigned long now = micros();
  if (now - lastIntR < 100) return;
  int newR = (digitalRead(C1R) << 1) | digitalRead(C2R);
  int diff = (newR - actR) & 3;
  if (diff == 1) nR++; else if (diff == 3) nR--;
  actR = newR; lastIntR = now;
}


void updateSensors(float dt) {
  static long lastPL = 0, lastPR = 0;
  long pL, pR;

  noInterrupts();
  pL = nL; pR = nR;
  interrupts();

  prevRawVelL = velL;
  prevRawVelR = velR;

  if (dt > 0) {
    velL = ((pL - lastPL) * 60.0) / (dt * PULSES_PER_REV);
    velR = ((pR - lastPR) * 60.0) / (dt * PULSES_PER_REV);
  }

  filteredVelL = ALPHA_RPM * velL + (1.0f - ALPHA_RPM) * filteredVelL;
  filteredVelR = ALPHA_RPM * velR + (1.0f - ALPHA_RPM) * filteredVelR;

  // CÁLCULO DE ODOMETRÍA DIRECTO POR PULSOS (CORREGIDO)
  double dPulsosL = pL - lastPL;
  double dPulsosR = pR - lastPR;

  // Convertir delta de pulsos a metros recorridos por cada rueda
  double dsL = (dPulsosL / (double)PULSES_PER_REV) * M_PI * WHEEL_DIAMETER;
  double dsR = (dPulsosR / (double)PULSES_PER_REV) * M_PI * WHEEL_DIAMETER;

  double dc = (dsL + dsR) / 2.0;
  double dtheta = (dsR - dsL) / WHEEL_BASE;

  robotTheta += dtheta;
  robotTheta = atan2(sin(robotTheta), cos(robotTheta));

  robotX += dc * cos(robotTheta);
  robotY += dc * sin(robotTheta);

  lastPL = pL; lastPR = pR;
}

void processCommand(const char *cmd) {
  lastCmdTime = millis();
  if (cmd[0] == 'V') {
    sscanf(cmd + 2, "%lf,%lf", &targetRPML, &targetRPMR);
  } else if (cmd[0] == 'K') {
    double lin, ang;
    if (sscanf(cmd + 2, "%lf,%lf", &lin, &ang) == 2) {
      setVelocity(lin, ang);
    }
  } else if (cmd[0] == 'D') {
    double d; if (sscanf(cmd + 2, "%lf", &d) == 1) moverCentimetros(d, 0.2);
  } else if (cmd[0] == 'A') {
    double a; if (sscanf(cmd + 2, "%lf", &a) == 1) girarGrados(a, 1.5);
  } else if (cmd[0] == 'S') {
    targetRPML = targetRPMR = 0;
    integralL = integralR = 0;
    rampL = rampR = 0;
  } else if (cmd[0] == 'R') {
    noInterrupts(); nL = nR = 0; interrupts();
    robotX = robotY = robotTheta = 0;
  }
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Disconnected!\n", num);
      break;
    case WStype_CONNECTED:
      {
        IPAddress ip = webSocket.remoteIP(num);
        Serial.printf("[%u] Connected from %d.%d.%d.%d\n", num, ip[0], ip[1], ip[2], ip[3]);
      }
      break;
    case WStype_TEXT:
      processCommand((const char*)payload);
      break;
  }
}

void setup() {
  Serial.begin(115200);
  
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  pinMode(C1L, INPUT_PULLUP);
  pinMode(C2L, INPUT_PULLUP);
  pinMode(C1R, INPUT_PULLUP);
  pinMode(C2R, INPUT_PULLUP);

  actL = (digitalRead(C1L) << 1) | digitalRead(C2L);
  actR = (digitalRead(C1R) << 1) | digitalRead(C2R);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(C1L), encoderL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(C1R), encoderR, CHANGE);

  // Nueva API LEDC para ESP32 Core 3.x
  ledcAttach(ENA, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(ENB, PWM_FREQ, PWM_RESOLUTION);

  lastLoopMicros = micros();
  lastCmdTime = millis();
}

void loop() {
  unsigned long nowMicros = micros();
  unsigned long nowMillis = millis();

  webSocket.loop();

  if (nowMicros - lastLoopMicros >= LOOP_PERIOD_US) {
    float dt = (nowMicros - lastLoopMicros) * 1e-6f;
    lastLoopMicros = nowMicros;

    updateSensors(dt);

    if (moviendoObjetivo) {
      if (modoMovimiento == 1) {
        double distRecorrida = sqrt(pow(robotX - xInicial, 2) + pow(robotY - yInicial, 2));
        if (distRecorrida >= distObjetivo) {
          moviendoObjetivo = false; modoMovimiento = 0; setVelocity(0, 0); lastCmdTime = millis();
        } else {
          // Algoritmo de sincronización activa
          long pL, pR;
          noInterrupts();
          pL = nL; pR = nR;
          interrupts();
          long difPulsos = pL - pR;
          double correccion = difPulsos * K_SYNC;
          
          // Aplica la corrección opuesta a cada rueda para mantener el rumbo
          targetRPML = velCruceroLineal * 60.0 / (M_PI * WHEEL_DIAMETER) - correccion;
          targetRPMR = velCruceroLineal * 60.0 / (M_PI * WHEEL_DIAMETER) + correccion;
          
          // Restringe a los límites máximos permitidos
          targetRPML = constrain(targetRPML, -MAX_RPM, MAX_RPM);
          targetRPMR = constrain(targetRPMR, -MAX_RPM, MAX_RPM);
          
          lastCmdTime = millis();
        }
      } else if (modoMovimiento == 2) {
        double deltaTheta = robotTheta - thetaInicial;
        deltaTheta = atan2(sin(deltaTheta), cos(deltaTheta));
        if (abs(deltaTheta) >= angObjetivo) {
          moviendoObjetivo = false; modoMovimiento = 0; setVelocity(0, 0); lastCmdTime = millis();
        } else {
          setVelocity(0.0, velCruceroAngular); lastCmdTime = millis();
        }
      }
    }

    updateControl(dt);
    applyMotors();
  }

  if (nowMillis - lastCmdTime > WATCH_IDLE_MS) {
    targetRPML = 0; targetRPMR = 0;
  }

  if (nowMillis - lastTxTime > TELEMETRY_MS) {
    lastTxTime = nowMillis;
    long pL, pR;
    noInterrupts();
    pL = nL; pR = nR;
    interrupts();
    
    char telemetryBuf[128];
    snprintf(telemetryBuf, sizeof(telemetryBuf), "Posicion_X:%.2f Posicion_Y:%.2f Orientacion_Theta:%.2f RPM_Izq:%.1f RPM_Der:%.1f Pulsos_Izq:%ld Pulsos_Der:%ld",
                    robotX, robotY, robotTheta,
                    filteredVelL, filteredVelR, pL, pR);
    webSocket.broadcastTXT(telemetryBuf);
  }
}
