#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>

// WiFi 
const char* ssid     = "DRAWBOT";
const char* password = "drawbot123";

// Serveur web 
WebServer        server(80);
WebSocketsServer ws(81);

//  Pins 
#define LEDU1      25
#define LEDU2      26
#define EN_D       23
#define EN_G       4
#define IN_1_D     19
#define IN_2_D     18
#define IN_1_G     17
#define IN_2_G     16
#define ENC_G_CH_A 32
#define ENC_G_CH_B 33
#define ENC_D_CH_A 27
#define ENC_D_CH_B 14
#define SDA_PIN    21
#define SCL_PIN    22
#define ADDR_IMU   0x6B
#define ADDR_MAG   0x1E

//  PWM 
#define PWM_FREQ 1000
#define PWM_RES  8
#define CH_IN1_D 0
#define CH_IN2_D 1
#define CH_IN1_G 2
#define CH_IN2_G 3

//  Encodeurs 
volatile long enc_G = 0;
volatile long enc_D = 0;
void IRAM_ATTR ISR_G() { enc_G += (digitalRead(ENC_G_CH_B) == HIGH) ? 1 : -1; }
void IRAM_ATTR ISR_D() { enc_D += (digitalRead(ENC_D_CH_B) == HIGH) ? 1 : -1; }

//  Variables capteurs 
float ax=0, ay=0, az=0;
float gx=0, gy=0, gz=0;
float mx=0, my=0;
float heading = 0;
float rpm_G=0, rpm_D=0;
float pos_x=0, pos_y=0;
float orientation=0;

//  Vitesse moteurs 
int speedLeft  = 0;
int speedRight = 0;

//  Ticks/tour et diamètre roue 
#define TICKS_PAR_TOUR 1052    
#define DIAM_ROUE_CM   9.0
#define ENTRAXE_CM     8.0
#define ROTATION_CALIB 0.94f
#define OFFSET_STYLO_CM 15.0f
#define RAYON_CERCLE_MIN_CM 18.0f
#define CERCLE_CALIB 1.4f

//  Timers 
unsigned long lastSensorRead = 0;
unsigned long lastEncoderCalc = 0;
long prev_enc_G = 0;
long prev_enc_D = 0;


//  PAGE HTML 
const char HTML_PAGE[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html lang="fr">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no">
<title>DRAWBOT</title>
<style>
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body {
    background: #0f0f0f;
    color: #fff;
    font-family: -apple-system, sans-serif;
    padding: 12px;
    user-select: none;
  }
  h1 {
    text-align: center;
    color: #00c8a0;
    font-size: 22px;
    margin-bottom: 12px;
    letter-spacing: 3px;
  }
  .status {
    text-align: center;
    font-size: 12px;
    margin-bottom: 12px;
    padding: 6px;
    border-radius: 8px;
    background: #1a1a1a;
  }
  .status.connected { color: #00c8a0; }
  .status.disconnected { color: #ff4444; }

  /*  Grille données  */
  .grid {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 8px;
    margin-bottom: 12px;
  }
  .card {
    background: #1a1a1a;
    border-radius: 12px;
    padding: 10px;
    border: 1px solid #2a2a2a;
  }
  .card.full { grid-column: span 2; }
  .card-title {
    font-size: 10px;
    color: #888;
    text-transform: uppercase;
    letter-spacing: 1px;
    margin-bottom: 6px;
  }
  .card-value {
    font-size: 18px;
    font-weight: bold;
    color: #00c8a0;
  }
  .card-value.warn { color: #ffaa00; }
  .card-sub {
    font-size: 11px;
    color: #555;
    margin-top: 2px;
  }

  /*  Joystick zone  */
  .joystick-section {
    margin-bottom: 12px;
  }
  .joystick-title {
    text-align: center;
    font-size: 11px;
    color: #888;
    margin-bottom: 8px;
    text-transform: uppercase;
    letter-spacing: 1px;
  }
  .dpad {
    display: grid;
    grid-template-columns: 70px 70px 70px;
    grid-template-rows: 70px 70px 70px;
    gap: 6px;
    justify-content: center;
  }
  .btn {
    background: #1a1a1a;
    border: 1px solid #333;
    border-radius: 12px;
    font-size: 24px;
    display: flex;
    align-items: center;
    justify-content: center;
    cursor: pointer;
    transition: background 0.1s;
    -webkit-tap-highlight-color: transparent;
  }
  .btn:active, .btn.pressed {
    background: #00c8a0;
    border-color: #00c8a0;
  }
  .btn.empty { visibility: hidden; }
  .btn.stop {
    background: #2a0000;
    border-color: #ff4444;
    font-size: 18px;
    color: #ff4444;
  }
  .btn.stop:active { background: #ff4444; color: #fff; }

  /*  vitesse  */
  .speed-section {
    background: #1a1a1a;
    border-radius: 12px;
    padding: 12px;
    margin-bottom: 12px;
    border: 1px solid #2a2a2a;
  }
  .speed-section label {
    font-size: 11px;
    color: #888;
    text-transform: uppercase;
    letter-spacing: 1px;
    display: block;
    margin-bottom: 8px;
  }
  input[type=range] {
    width: 100%;
    accent-color: #00c8a0;
  }
  .speed-val {
    text-align: center;
    color: #00c8a0;
    font-size: 20px;
    font-weight: bold;
    margin-top: 4px;
  }

  /*  Séquences  */
  .seq-section {
    margin-bottom: 12px;
  }
  .seq-title {
    font-size: 11px;
    color: #888;
    text-transform: uppercase;
    letter-spacing: 1px;
    margin-bottom: 8px;
  }
  .seq-grid {
    display: grid;
    grid-template-columns: 1fr 1fr 1fr;
    gap: 8px;
  }
  .seq-btn {
    background: #1a2a25;
    border: 1px solid #00c8a0;
    border-radius: 12px;
    padding: 12px 6px;
    color: #00c8a0;
    font-size: 12px;
    font-weight: bold;
    text-align: center;
    cursor: pointer;
  }
  .seq-btn:active { background: #00c8a0; color: #000; }

  /*  position  */
  canvas {
    display: block;
    margin: 0 auto;
    border-radius: 8px;
    background: #111;
    border: 1px solid #2a2a2a;
  }
</style>
</head>
<body>

<h1>⚡ DRAWBOT</h1>

<div class="status disconnected" id="status">● Déconnecté</div>

<!-- Données capteurs -->
<div class="grid">
  <div class="card">
    <div class="card-title">Cap (Nord)</div>
    <div class="card-value" id="heading">--°</div>
    <div class="card-sub">Magnétomètre</div>
  </div>
  <div class="card">
    <div class="card-title">Orientation</div>
    <div class="card-value" id="orientation">--°</div>
    <div class="card-sub">Gyroscope intégré</div>
  </div>
  <div class="card">
    <div class="card-title">Vitesse G</div>
    <div class="card-value" id="rpm_g">-- RPM</div>
    <div class="card-sub">Encodeur gauche</div>
  </div>
  <div class="card">
    <div class="card-title">Vitesse D</div>
    <div class="card-value" id="rpm_d">-- RPM</div>
    <div class="card-sub">Encodeur droit</div>
  </div>
  <div class="card">
    <div class="card-title">Accéléro</div>
    <div class="card-value warn" id="accel" style="font-size:13px">--</div>
    <div class="card-sub">X / Y / Z (g)</div>
  </div>
  <div class="card">
    <div class="card-title">Position</div>
    <div class="card-value" id="position" style="font-size:13px">--</div>
    <div class="card-sub">X / Y (cm)</div>
  </div>
</div>

<!-- Canvas tracé position -->
<div class="card full" style="margin-bottom:12px; padding:10px;">
  <div class="card-title" style="margin-bottom:8px">Tracé du stylo (vue du dessus)</div>
  <canvas id="traceCanvas" width="280" height="180"></canvas>
  <button onclick="clearTrace()" style="margin-top:8px; width:100%; background:#1a1a1a; border:1px solid #333; color:#888; border-radius:8px; padding:6px; font-size:12px;">Effacer le tracé</button>
</div>

<!-- Slider vitesse -->
<div class="speed-section">
  <label>Vitesse de déplacement</label>
  <input type="range" id="speedSlider" min="50" max="220" value="120"
    oninput="updateSpeed(this.value)">
  <div class="speed-val" id="speedVal">PWM : 120</div>
</div>

<!-- D-Pad -->
<div class="joystick-section">
  <div class="joystick-title">Pilotage manuel</div>
  <div class="dpad">
    <div class="btn empty"></div>
    <div class="btn" id="btn_up"
      ontouchstart="sendCmd('F');this.classList.add('pressed')"
      ontouchend="sendCmd('S');this.classList.remove('pressed')">▲</div>
    <div class="btn empty"></div>

    <div class="btn" id="btn_left"
      ontouchstart="sendCmd('L');this.classList.add('pressed')"
      ontouchend="sendCmd('S');this.classList.remove('pressed')">◀</div>
    <div class="btn stop" id="btn_stop"
      ontouchstart="sendCmd('S')">STOP</div>
    <div class="btn" id="btn_right"
      ontouchstart="sendCmd('R');this.classList.add('pressed')"
      ontouchend="sendCmd('S');this.classList.remove('pressed')">▶</div>

    <div class="btn empty"></div>
    <div class="btn" id="btn_down"
      ontouchstart="sendCmd('B');this.classList.add('pressed')"
      ontouchend="sendCmd('S');this.classList.remove('pressed')">▼</div>
    <div class="btn empty"></div>
  </div>
</div>

<!-- Séquences automatiques -->
<div class="seq-section">
  <div class="seq-title">Séquences automatiques</div>
  <div class="seq-grid">
    <div class="seq-btn" onclick="sendCmd('SEQ1')">SEQ 1<br><small>Escalier</small></div>
    <div class="seq-btn" onclick="sendSeq2()">SEQ 2<br><small>Cercle</small></div>
    <div class="seq-btn" onclick="sendCmd('SEQ3')">SEQ 3<br><small>la rose des vents</small></div>
  </div>
</div>

<script>
let ws;
let speedPWM = 120;
let tracePoints = [];
let scale = 5; // pixels par cm

function connect() {
  ws = new WebSocket('ws://' + location.hostname + ':81/');
  ws.onopen = () => {
    document.getElementById('status').textContent = '● Connecté';
    document.getElementById('status').className = 'status connected';
  };
  ws.onclose = () => {
    document.getElementById('status').textContent = '● Déconnecté — reconnexion...';
    document.getElementById('status').className = 'status disconnected';
    setTimeout(connect, 2000);
  };
  ws.onmessage = (evt) => {
    try {
      const d = JSON.parse(evt.data);
      document.getElementById('heading').textContent     = d.heading.toFixed(1) + '°';
      document.getElementById('orientation').textContent = d.orientation.toFixed(1) + '°';
      document.getElementById('rpm_g').textContent       = d.rpm_g.toFixed(0) + ' RPM';
      document.getElementById('rpm_d').textContent       = d.rpm_d.toFixed(0) + ' RPM';
      document.getElementById('accel').textContent       = d.ax.toFixed(2) + ' / ' + d.ay.toFixed(2) + ' / ' + d.az.toFixed(2);
      document.getElementById('position').textContent    = d.pos_x.toFixed(1) + ' / ' + d.pos_y.toFixed(1) + ' cm';
      updateTrace(d.pos_x, d.pos_y);
    } catch(e) {}
  };
}

function sendCmd(cmd) {
  if (ws && ws.readyState === 1)
    ws.send(JSON.stringify({cmd: cmd, speed: speedPWM}));
}

function sendSeq2() {
  const r = prompt("Rayon du cercle en cm (18 à 40) :", "20");
  if (r !== null) {
    const rayon = parseFloat(r);
    if (rayon >= 18 && rayon <= 40) {
      if (ws && ws.readyState === 1)
        ws.send(JSON.stringify({cmd: "SEQ2", speed: speedPWM, rayon: rayon}));
    } else {
      alert("Rayon invalide ! Entre 18 et 40 cm.");
    }
  }
}

function updateSpeed(v) {
  speedPWM = parseInt(v);
  document.getElementById('speedVal').textContent = 'PWM : ' + speedPWM;
}

// --- Tracé canvas ---
const canvas = document.getElementById('traceCanvas');
const ctx = canvas.getContext('2d');
let traceInitialized = false;

function updateTrace(x, y) {
  const cx = canvas.width  / 2;
  const cy = canvas.height / 2;
  const px = cx + x * scale;
  const py = cy - y * scale;

  if (!traceInitialized) {
    ctx.strokeStyle = '#00c8a0';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(px, py);
    traceInitialized = true;
  } else {
    ctx.lineTo(px, py);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(px, py);
  }

  // Point robot (triangle orienté)
  ctx.fillStyle = '#fff';
  ctx.beginPath();
  ctx.arc(px, py, 3, 0, 2 * Math.PI);
  ctx.fill();
}

function clearTrace() {
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  // Redessine la grille
  ctx.strokeStyle = '#222';
  ctx.lineWidth = 0.5;
  for (let x = 0; x < canvas.width; x += 20) {
    ctx.beginPath(); ctx.moveTo(x,0); ctx.lineTo(x, canvas.height); ctx.stroke();
  }
  for (let y = 0; y < canvas.height; y += 20) {
    ctx.beginPath(); ctx.moveTo(0,y); ctx.lineTo(canvas.width, y); ctx.stroke();
  }
  // Croix centrale
  ctx.strokeStyle = '#333';
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(canvas.width/2, 0);
  ctx.lineTo(canvas.width/2, canvas.height);
  ctx.stroke();
  ctx.beginPath();
  ctx.moveTo(0, canvas.height/2);
  ctx.lineTo(canvas.width, canvas.height/2);
  ctx.stroke();
  traceInitialized = false;
}

clearTrace(); // Dessine la grille au démarrage
connect();
</script>
</body>
</html>
)rawhtml";

//   MOTEURS
void motorDroit(int s) {
  if (s>0)      { ledcWrite(CH_IN1_D,s);  ledcWrite(CH_IN2_D,0); }
  else if (s<0) { ledcWrite(CH_IN1_D,0);  ledcWrite(CH_IN2_D,-s); }
  else          { ledcWrite(CH_IN1_D,0);  ledcWrite(CH_IN2_D,0); }
}
void motorGauche(int s) {
  if (s>0)      { ledcWrite(CH_IN1_G,s);  ledcWrite(CH_IN2_G,0); }
  else if (s<0) { ledcWrite(CH_IN1_G,0);  ledcWrite(CH_IN2_G,-s); }
  else          { ledcWrite(CH_IN1_G,0);  ledcWrite(CH_IN2_G,0); }
}
void stopMoteurs() { motorDroit(0); motorGauche(0); }


//  LECTURE IMU
void readIMU() {
  // Accel
  Wire.beginTransmission(ADDR_IMU);
  Wire.write(0x28); Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)ADDR_IMU,(uint8_t)6);
  if (Wire.available()>=6) {
    int16_t raw_ax = Wire.read()|(Wire.read()<<8);
    int16_t raw_ay = Wire.read()|(Wire.read()<<8);
    int16_t raw_az = Wire.read()|(Wire.read()<<8);
    ax = raw_ax * 0.000061f;
    ay = raw_ay * 0.000061f;
    az = raw_az * 0.000061f;
  }
  // Gyro Z
  Wire.beginTransmission(ADDR_IMU);
  Wire.write(0x24); Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)ADDR_IMU,(uint8_t)2);
  if (Wire.available()>=2) {
    int16_t raw_gz = Wire.read()|(Wire.read()<<8);
    gz = raw_gz * 0.00875f; // dps
  }
}

//  LECTURE MAG
void readMAG() {
  // Active le MAG 
  Wire.beginTransmission(ADDR_MAG);
  Wire.write(0x20); Wire.write(0x70); Wire.endTransmission();

  Wire.beginTransmission(ADDR_MAG);
  Wire.write(0x22); Wire.write(0x00); Wire.endTransmission();

  Wire.beginTransmission(ADDR_MAG);
  Wire.write(0x28); Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)ADDR_MAG,(uint8_t)4);
  if (Wire.available()>=4) {
    int16_t raw_mx = Wire.read()|(Wire.read()<<8);
    int16_t raw_my = Wire.read()|(Wire.read()<<8);
    
    // Calibration offsets
    mx = raw_mx - (-4176);
    my = raw_my - (2522);

    // Calcul angle brut
    heading = atan2(my, mx) * 180.0f / PI;

    if (heading < 0)
      heading += 360.0f;

    // Correction Est/Ouest
    heading = 360.0f - heading;

    // Recalage Nord réel
    heading = heading - 90.0f + 12.0f;

    if (heading < 0)
      heading += 360.0f;

    if (heading >= 360)
      heading -= 360.0f;
      }
}


//  CALCUL ODOMÉTRIE
void updateOdometry(float dt) {
  long d_G = enc_G - prev_enc_G;
  long d_D = enc_D - prev_enc_D;
  prev_enc_G = enc_G;
  prev_enc_D = enc_D;

  float circ   = PI * DIAM_ROUE_CM;
  float dist_G = (float)d_G / TICKS_PAR_TOUR * circ;
  float dist_D = (float)d_D / TICKS_PAR_TOUR * circ;
  float dist   = (dist_G + dist_D) / 2.0f;

  // RPM
  if (dt > 0) {
    rpm_G = ((float)d_G / TICKS_PAR_TOUR) / dt * 60.0f;
    rpm_D = ((float)d_D / TICKS_PAR_TOUR) / dt * 60.0f;
  }

  // Intégration gyro pour orientation
  orientation += gz * dt;
  if (orientation >  180) orientation -= 360;
  if (orientation < -180) orientation += 360;

  float angle_rad = orientation * PI / 180.0f;
  pos_x += dist * cos(angle_rad);
  pos_y += dist * sin(angle_rad);
}

long cmToTicks(float cm) {
  return (long)((cm / (PI * DIAM_ROUE_CM)) * TICKS_PAR_TOUR);
}

long degToTicks(float degres) {
  return (long)(((fabs(degres) / 360.0f) * PI * ENTRAXE_CM / (PI * DIAM_ROUE_CM)) * TICKS_PAR_TOUR * ROTATION_CALIB);
}

void avancerCm(float cm, int spd) {
  const float depassementCm = 3.0f;
  float distanceCible = fabs(cm);
  if (distanceCible > depassementCm) {
    distanceCible -= depassementCm;
  }
  if (fabs(fabs(cm) - 20.0f) < 0.5f) {
    distanceCible -= 1.0f;
  }

  long cible = cmToTicks(distanceCible);
  int sens = (cm >= 0) ? 1 : -1;
  const float kp = 0.30f;
  const int minPwm = 45;
  const int trimGauche = 0;
  const int trimDroit = 0;
  const int maxCorrection = 10;

  enc_G = 0;
  enc_D = 0;

  while (((abs(enc_G) + abs(enc_D)) / 2) < cible) {
    long ticksG = abs(enc_G);
    long ticksD = abs(enc_D);
    int correction = constrain((int)((ticksG - ticksD) * kp), -maxCorrection, maxCorrection);

    int pwmG = constrain(spd + trimGauche - correction, minPwm, spd + 35);
    int pwmD = constrain(spd + trimDroit + correction, minPwm, spd + 35);

    motorGauche(sens * pwmG);
    motorDroit(-sens * pwmD);

    ws.loop();
    delay(5);
  }

  stopMoteurs();
  delay(300);
}

void tournerDegres(float degres, int spd) {
  long cible = degToTicks(degres);
  int sens = (degres >= 0) ? 1 : -1;
  const float kp = 0.25f;
  const int minPwm = 60;
  unsigned long debutRotation = millis();
  long dernierTotal = 0;
  unsigned long dernierMouvement = millis();

  enc_G = 0;
  enc_D = 0;

  while (((abs(enc_G) + abs(enc_D)) / 2) < cible) {
    long ticksG = abs(enc_G);
    long ticksD = abs(enc_D);
    long totalTicks = (ticksG + ticksD) / 2;
    int correction = (int)((ticksG - ticksD) * kp);

    int pwmG = constrain(spd - correction, minPwm, spd + 35);
    int pwmD = constrain(spd + correction, minPwm, spd + 35);

    if (sens > 0) {
      motorGauche(-pwmG);
      motorDroit(-pwmD);
    } else {
      motorGauche(pwmG);
      motorDroit(pwmD);
    }

    ws.loop();
    delay(5);

    if (totalTicks > dernierTotal + 2) {
      dernierTotal = totalTicks;
      dernierMouvement = millis();
    }

    if (millis() - dernierMouvement > 700 || millis() - debutRotation > 5000) {
      break;
    }
  }

  stopMoteurs();
  delay(300);
}

// ================================================
//  SÉQUENCE 1 — L'escalier
//  Avance 20cm, tourne 90° gauche, avance 10cm,
//  tourne 90° droite, avance 40cm
// ================================================
void sequence1() {
  int spdAvance = 65;
  int spdTourne = 75;

  avancerCm(20.0, spdAvance);
  tournerDegres(90.0, spdTourne);
  avancerCm(10.0, spdAvance);
  tournerDegres(-90.0, spdTourne);
  avancerCm(40.0, spdAvance);

  Serial.println("SEQ1 terminee");
}



// ================================================
//  SÉQUENCE 2 — Le cercle
//  Rayon paramétrable envoyé depuis l'iPhone
// ================================================
void sequence2(float rayon_cm) {
  int   vBase   = 90;

  if (rayon_cm < RAYON_CERCLE_MIN_CM) {
    Serial.println("Rayon impossible avec le stylo actuel");
    return;
  }

  float rayonCentre = sqrt((rayon_cm * rayon_cm) - (OFFSET_STYLO_CM * OFFSET_STYLO_CM));

  // Roue droite = extérieure (plus grande trajectoire)
  // Roue gauche = intérieure (plus petite trajectoire)
  float rExt = rayonCentre + ENTRAXE_CM / 2.0;
  float rInt = rayonCentre - ENTRAXE_CM / 2.0;

  // Les vitesses sont proportionnelles aux rayons
  float vExt = vBase;
  float vInt = vBase * (rInt / rExt) * 0.55f;

  // Limites
  if (vExt > 220) vExt = 220;
  if (vInt < 35)  vInt = 35;

  // Distance parcourue par la roue extérieure = circonférence ext
  float distExt    = 2.0 * PI * rExt;
  float ticksTotal = (distExt / (PI * DIAM_ROUE_CM)) * TICKS_PAR_TOUR * CERCLE_CALIB;

  Serial.print("rayonStylo="); Serial.print(rayon_cm);
  Serial.print(" rayonCentre="); Serial.print(rayonCentre);
  Serial.print(" vExt="); Serial.print(vExt);
  Serial.print(" vInt="); Serial.print(vInt);
  Serial.print(" ticksTotal="); Serial.println(ticksTotal);

  enc_G = 0; enc_D = 0;
  motorDroit(-(int)vExt);
  motorGauche((int)vInt);

  while (abs(enc_D) < (long)ticksTotal) {
    ws.loop();
    delay(5);
  }
  stopMoteurs();
}
// ================================================
//  SÉQUENCE 3 — La rose des vents (flèche vers le Nord)
// ================================================

void sequence3() {
  int spd = 120;
  float entraxe = 8.0;

  // Ticks par degré de rotation sur place
  // 1 tour complet (360°) = PI * entraxe parcouru par chaque roue
  float ticksParDegre = (PI * entraxe / (PI * DIAM_ROUE_CM)) * TICKS_PAR_TOUR / 360.0;

  float ticks6cm = (6.0 / (PI * DIAM_ROUE_CM)) * TICKS_PAR_TOUR;
  float ticks2cm = (2.0 / (PI * DIAM_ROUE_CM)) * TICKS_PAR_TOUR;

  // --- 1. Lire le cap actuel ---
  // Plusieurs lectures pour stabiliser
  for (int i = 0; i < 10; i++) { readMAG(); delay(20); }
  float capActuel = heading;  // 0-360°, 0 = Nord

  // --- 2. Calculer l'angle à corriger ---
  // On veut pointer vers le Nord (heading = 0)
  // angleACorrige = combien tourner (+ = gauche, - = droite)
  float angleACorrige = capActuel;
  if (angleACorrige > 180.0) angleACorrige -= 360.0; // ramène entre -180 et +180

  long ticksCible = (long)(abs(angleACorrige) * ticksParDegre);

  Serial.print("Cap actuel : ");   Serial.println(capActuel);
  Serial.print("Angle a corriger: "); Serial.println(angleACorrige);
  Serial.print("Ticks cible : ");  Serial.println(ticksCible);
  Serial.print("TicksParDegre : "); Serial.println(ticksParDegre);

  // --- 3. Rotation pour pointer vers le Nord ---
  enc_G = 0; enc_D = 0;
  if (angleACorrige > 0) {
    // On est à droite du Nord → tourne à gauche
    motorDroit(spd); motorGauche(-spd);
    while (abs(enc_D) < ticksCible) { ws.loop(); delay(5); }
  } else if (angleACorrige < 0) {
    // On est à gauche du Nord → tourne à droite
    motorDroit(-spd); motorGauche(spd);
    while (abs(enc_G) < ticksCible) { ws.loop(); delay(5); }
  }
  stopMoteurs(); delay(500);

  // --- 4. Vérification après rotation ---
  for (int i = 0; i < 5; i++) { readMAG(); delay(20); }
  Serial.print("Cap apres rotation : "); Serial.println(heading);

  // --- 5. Dessiner la tige de la flèche (6 cm vers le Nord) ---
  enc_G = 0; enc_D = 0;
  motorDroit(spd); motorGauche(spd);
  while (abs(enc_G) < (long)ticks6cm && abs(enc_D) < (long)ticks6cm) {
    ws.loop(); delay(5);
  }
  stopMoteurs(); delay(300);

  // --- 6. Pointe GAUCHE du triangle (tourne 150° à droite) ---
  float ticks150 = ticksParDegre * 150.0;
  enc_G = 0; enc_D = 0;
  motorDroit(-spd); motorGauche(spd);
  while (abs(enc_G) < (long)ticks150) { ws.loop(); delay(5); }
  stopMoteurs(); delay(200);

  // Avance 2 cm (côté gauche de la pointe)
  enc_G = 0; enc_D = 0;
  motorDroit(spd); motorGauche(spd);
  while (abs(enc_G) < (long)ticks2cm && abs(enc_D) < (long)ticks2cm) {
    ws.loop(); delay(5);
  }
  stopMoteurs(); delay(200);

  // --- 7. Retour au sommet (tourne 120° à gauche) ---
  float ticks120 = ticksParDegre * 120.0;
  enc_G = 0; enc_D = 0;
  motorDroit(spd); motorGauche(-spd);
  while (abs(enc_D) < (long)ticks120) { ws.loop(); delay(5); }
  stopMoteurs(); delay(200);

  // Avance 2 cm (côté droit de la pointe)
  enc_G = 0; enc_D = 0;
  motorDroit(spd); motorGauche(spd);
  while (abs(enc_G) < (long)ticks2cm && abs(enc_D) < (long)ticks2cm) {
    ws.loop(); delay(5);
  }
  stopMoteurs(); delay(200);

  // --- 8. Fermer le triangle (tourne 150° à gauche) ---
  enc_G = 0; enc_D = 0;
  motorDroit(spd); motorGauche(-spd);
  while (abs(enc_D) < (long)ticks150) { ws.loop(); delay(5); }
  stopMoteurs(); delay(200);

  // Revenir au centre (recule 2 cm)
  enc_G = 0; enc_D = 0;
  motorDroit(-spd); motorGauche(-spd);
  while (abs(enc_G) < (long)ticks2cm && abs(enc_D) < (long)ticks2cm) {
    ws.loop(); delay(5);
  }
  stopMoteurs();

  Serial.println("SEQ3 terminee");
}

// réception commandes
void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  if (type == WStype_TEXT) {
    DynamicJsonDocument doc(128);
    deserializeJson(doc, payload, length);
    String cmd   = doc["cmd"].as<String>();
    int    speed = doc["speed"] | 120;

    if      (cmd == "F")  { motorDroit(-speed);  motorGauche(speed);  }
    else if (cmd == "B")  { motorDroit(speed);   motorGauche(-speed); }
    else if (cmd == "L")  { motorDroit(-speed);  motorGauche(-speed); }
    else if (cmd == "R")  { motorDroit(speed);   motorGauche(speed);  }
    else if (cmd == "S")  { stopMoteurs(); }

    else if (cmd == "SEQ1") { sequence1(); }
    else if (cmd == "SEQ2") {
      float rayon = doc["rayon"] | 10.0;  // rayon par défaut 10 cm
      sequence2(rayon);
    }
    else if (cmd == "SEQ3") { sequence3(); }
  }
}

void setup() {
  Serial.begin(115200);

  // LEDs
  pinMode(LEDU1, OUTPUT);
  pinMode(LEDU2, OUTPUT);

  // Moteurs
  pinMode(EN_D, OUTPUT); digitalWrite(EN_D, HIGH);
  pinMode(EN_G, OUTPUT); digitalWrite(EN_G, HIGH);
  ledcSetup(CH_IN1_D, PWM_FREQ, PWM_RES); ledcAttachPin(IN_1_D, CH_IN1_D);
  ledcSetup(CH_IN2_D, PWM_FREQ, PWM_RES); ledcAttachPin(IN_2_D, CH_IN2_D);
  ledcSetup(CH_IN1_G, PWM_FREQ, PWM_RES); ledcAttachPin(IN_1_G, CH_IN1_G);
  ledcSetup(CH_IN2_G, PWM_FREQ, PWM_RES); ledcAttachPin(IN_2_G, CH_IN2_G);

  // Encodeurs
  pinMode(ENC_G_CH_A, INPUT_PULLUP); pinMode(ENC_G_CH_B, INPUT_PULLUP);
  pinMode(ENC_D_CH_A, INPUT_PULLUP); pinMode(ENC_D_CH_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_G_CH_A), ISR_G, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_D_CH_A), ISR_D, RISING);

  // I2C
  Wire.begin(SDA_PIN, SCL_PIN);

  // Active IMU
  Wire.beginTransmission(ADDR_IMU);
  Wire.write(0x10); Wire.write(0x40); Wire.endTransmission(); // Accel 104Hz
  Wire.beginTransmission(ADDR_IMU);
  Wire.write(0x11); Wire.write(0x40); Wire.endTransmission(); // Gyro 104Hz

  // WiFi 
  WiFi.softAP(ssid, password);
  Serial.print("IP : "); Serial.println(WiFi.softAPIP());

  // Serveur web
  server.on("/", []() {
    server.send_P(200, "text/html", HTML_PAGE);
  });
  server.begin();

  // WebSocket
  ws.begin();
  ws.onEvent(webSocketEvent);

  Serial.println("DRAWBOT prêt !");
  digitalWrite(LEDU1, HIGH);
}


void loop() {
  server.handleClient();
  ws.loop();

  unsigned long now = millis();

  // Capteurs toutes les 50ms
  if (now - lastSensorRead > 50) {
    float dt = (now - lastSensorRead) / 1000.0f;
    lastSensorRead = now;
    readIMU();
    readMAG();
    updateOdometry(dt);

    // Envoie les données à l'iPhone
    DynamicJsonDocument doc(256);
    doc["heading"]     = heading;
    doc["orientation"] = orientation;
    doc["rpm_g"]       = rpm_G;
    doc["rpm_d"]       = rpm_D;
    doc["ax"]          = ax;
    doc["ay"]          = ay;
    doc["az"]          = az;
    doc["pos_x"]       = pos_x;
    doc["pos_y"]       = pos_y;
    String json;
    serializeJson(doc, json);
    ws.broadcastTXT(json);
  }
}  
