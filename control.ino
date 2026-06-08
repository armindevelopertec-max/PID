void updateControl(float dt) {
  if (dt <= 0) return;

  float step = MAX_ACCEL * dt;
  rampL += constrain(targetRPML - rampL, -step, step);
  rampR += constrain(targetRPMR - rampR, -step, step);

  if (abs(targetRPML) < 0.1 && abs(targetRPMR) < 0.1 && abs(filteredVelL) < 0.5 && abs(filteredVelR) < 0.5) {
    pwmL = pwmR = 0;
    integralL *= 0.7; integralR *= 0.7;
    rampL = rampR = 0;
    return;
  }

  float errorL = rampL - filteredVelL;
  float rawDL = -(velL - prevRawVelL) / dt;
  derivHistoryL[2] = derivHistoryL[1]; derivHistoryL[1] = derivHistoryL[0];
  derivHistoryL[0] = rawDL;
  float smoothDL = (derivHistoryL[0] + derivHistoryL[1] + derivHistoryL[2]) / 3.0;
  
  derivL = ALPHA_DERIV * smoothDL + (1.0f - ALPHA_DERIV) * derivL;
  float outL = Kp * errorL + Ki * integralL + Kd * derivL;
  outL = constrain(outL, -255, 255);

  if (abs(outL) < 255 || (outL * errorL < 0)) {
    integralL += errorL * dt;
    integralL = constrain(integralL, -60, 60);
  }

  float errorR = rampR - filteredVelR;
  float rawDR = -(velR - prevRawVelR) / dt;
  derivHistoryR[2] = derivHistoryR[1]; derivHistoryR[1] = derivHistoryR[0];
  derivHistoryR[0] = rawDR;
  float smoothDR = (derivHistoryR[0] + derivHistoryR[1] + derivHistoryR[2]) / 3.0;

  derivR = ALPHA_DERIV * smoothDR + (1.0f - ALPHA_DERIV) * derivR;
  float outR = Kp * errorR + Ki * integralR + Kd * derivR;
  outR = constrain(outR, -255, 255);

  if (abs(outR) < 255 || (outR * errorR < 0)) {
    integralR += errorR * dt;
    integralR = constrain(integralR, -60, 60);
  }

  dirL = (outL >= 0) ? 1 : -1;
  dirR = (outR >= 0) ? 1 : -1;
  pwmL = abs(outL);
  pwmR = abs(outR);

  if (pwmL > 0 && pwmL < PWM_DEADBAND) pwmL = PWM_DEADBAND;
  if (pwmR > 0 && pwmR < PWM_DEADBAND) pwmR = PWM_DEADBAND;
}

void applyMotors() {
  digitalWrite(IN1, (dirL == 1));
  digitalWrite(IN2, (dirL == -1));
  digitalWrite(IN3, (dirR == 1));
  digitalWrite(IN4, (dirR == -1));

  // En ESP32 Core 3.x ledcWrite usa el PIN, no el canal
  ledcWrite(ENA, pwmL);
  ledcWrite(ENB, pwmR);
}

void setVelocity(double linear, double angular) {
  double vL = linear - (angular * WHEEL_BASE / 2.0);
  double vR = linear + (angular * WHEEL_BASE / 2.0);
  targetRPML = constrain((vL * 60.0) / (M_PI * WHEEL_DIAMETER), -MAX_RPM, MAX_RPM);
  targetRPMR = constrain((vR * 60.0) / (M_PI * WHEEL_DIAMETER), -MAX_RPM, MAX_RPM);
}

void moverCentimetros(double cm, double velLineal) {
  xInicial = robotX; yInicial = robotY;
  distObjetivo = abs(cm) / 100.0;
  velCruceroLineal = (cm >= 0) ? velLineal : -velLineal;
  modoMovimiento = 1;
  moviendoObjetivo = true;
}

void girarGrados(double grados, double velAngular) {
  thetaInicial = robotTheta;
  angObjetivo = abs(grados * M_PI / 180.0);
  velCruceroAngular = (grados >= 0) ? velAngular : -velAngular;
  modoMovimiento = 2;
  moviendoObjetivo = true;
}
