#include <AFMotor.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <GY521.h>
#include <NewPing.h>
#include <Servo.h>
#include <ArduinoLog.h>
#include <QTRSensors.h>
#include <math.h>
#include <EEPROM.h>

QTRSensors qtr;

#define LEDA 41
#define LEDB 42
#define TRIGGER_PIN  40
#define ECHO_PIN     39
#define CALIB_FLAG  34 //conecte no GND para calibrar os sensores QTR
#define LED1 38 
#define LED2 37
#define LED3 35
#define LED4 36
#define MAX_DISTANCE 50
#define TCAADDR 0x70
#define DIREITA 0
#define ESQUERDA 1

#define INTERVALO_LEITURA 5
#define TEMPO_PRE90 1000
#define TEMPO_ORBITA 750
#define VEL_NORMAL 105
#define VEL_RESISTENCIA 120
#define VEL_CURVA 130
#define VEL_CURVA_EXTREMA 220
#define DISTANCIA_OBSTACULO 15
#define DISTANCIA_PARADA 15
#define DISTANCIA_MINIMA_VIRADA 10
#define LIMIAR_LINHA 995

#define MAX_SATURACAO_SEGUIDA 50     // Número de leituras saturadas seguidas para considerar defeituoso
#define MAX_NORMAL_SEGUIDO 50        // Número de leituras normais seguidas para reviver

#define SERVO_PIN 10
#define ANGULO_FRENTE 90
#define ANGULO_ESQUERDA 180
#define ANGULO_DIREITA 0
#define MAX_FALHAS_CONTINUAS 10
#define MAX_BUFFER_CORES 10

#define KP 90.0f
#define KI 0.0f
#define KD 6.0f

// Enumerações com descrições detalhadas
enum class Estado {
  SEGUINDO_LINHA,       // Estado normal de seguimento de linha
  RESOLVENDO_BIFURCACAO, // Encontrou uma bifurcação na pista
  DESVIANDO_OBSTACULO,   // Detectou obstáculo e está desviando
  PARADO,                // Robô parado (emergência ou final de percurso)
  INICIALIZANDO          // Estado inicial durante boot
};

enum Codigos {
  OP_SEGUINDO_LINHA = 1,      // 0001 - Operando normalmente
  OP_RESOLVENDO_BIFURCACAO = 2,// 0010 - Resolvendo bifurcação
  OP_DESVIANDO_OBSTACULO = 3,  // 0011 - Desviando de obstáculo
  OP_PARADO = 4,               // 0100 - Parado
  ERRO_MPU = 5,                // 0101 - Erro no sensor MPU6050
  ERRO_TCS_ESQUERDA = 6,       // 0110 - Erro no sensor de cor esquerdo
  ERRO_TCS_DIREITA = 7,        // 0111 - Erro no sensor de cor direito
  ERRO_QTR_SATURADO = 8,       // 1000 - Erro nos sensores de linha QTR
  ERRO_I2C = 9,                // 1001 - Erro de comunicação I2C
  CALIBRANDO = 10,             // 1010 - Calibrando
  OP_INICIALIZANDO = 15     // 1111 - Inicializando
};

enum Cores {
  PRETO,
  BRANCO,
  VERMELHO,
  VERDE,
  AZUL,
  CINZA,
  ERRO
};

struct CorSensor {
  Adafruit_TCS34725 tcs;       // Objeto do sensor de cor
  bool inicializado = false;   // Flag de inicialização
};

// Instanciação de objetos com logs detalhados
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
GY521 mpu(0x68);
bool mpuFuncionando = true;
const int MAX_TENTATIVAS_MPU = 3;
bool tcsFuncionando[2] = {false, false};
volatile Estado estadoAtual = Estado::INICIALIZANDO;
unsigned long ultimaAtualizacaoLED = 0;
const unsigned long INTERVALO_LED_MS = 200;
int ultimoCodigoMostrado = -1;

Servo servoUltrassonico;
AF_DCMotor motorFrenteEsquerdo(4);
AF_DCMotor motorFrenteDireito(1);
AF_DCMotor motorTrasEsquerdo(3);
AF_DCMotor motorTrasDireito(2);

CorSensor corSensores[2];
Cores bufferCores[MAX_BUFFER_CORES];
int indiceBuffer = 0;

unsigned int sensorValues[8];
int sensoresValidos = 8;  // ou inicialize conforme necessário
int validosDireita = 4;
int validosEsquerda = 4;

float media = 0.0;
float ultimaMedia = 0.0;

#define QTR_EEPROM_ADDR 0
#define QTR_EEPROM_MAGIC 0xA5A5
#define QTR_EEPROM_SIZE (2 + 2 + 8*2*2 + 2) // magic + sensorCount + min/max for 8 sensors (on/off) + CRC

// CRC-16-CCITT
uint16_t crc16(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
      else crc <<= 1;
    }
  }
  return crc;
}

void saveQTRCalibrationToEEPROM() {
  int addr = QTR_EEPROM_ADDR;
  EEPROM.put(addr, QTR_EEPROM_MAGIC); addr += 2;
  EEPROM.put(addr, qtr.getType() == QTRType::RC ? 8 : 0); addr += 2;
  for (int i = 0; i < 8; i++) {
    EEPROM.put(addr, qtr.calibrationOn.minimum[i]); addr += 2;
    EEPROM.put(addr, qtr.calibrationOn.maximum[i]); addr += 2;
  }
  for (int i = 0; i < 8; i++) {
    EEPROM.put(addr, qtr.calibrationOff.minimum ? qtr.calibrationOff.minimum[i] : 0); addr += 2;
    EEPROM.put(addr, qtr.calibrationOff.maximum ? qtr.calibrationOff.maximum[i] : 0); addr += 2;
  }
  // CRC
  uint8_t buf[QTR_EEPROM_SIZE-2];
  for (int i = 0; i < QTR_EEPROM_SIZE-2; i++) buf[i] = EEPROM.read(QTR_EEPROM_ADDR + i);
  uint16_t crc = crc16(buf, QTR_EEPROM_SIZE-2);
  EEPROM.put(addr, crc);
}

bool loadQTRCalibrationFromEEPROM() {
  qtr.calibrate(); // este é uma calibração temporária para garantir que os arrays estejam alocados, ou seja, só cria os arrays
  int addr = QTR_EEPROM_ADDR;
  uint16_t magic; EEPROM.get(addr, magic); addr += 2;
  if (magic != QTR_EEPROM_MAGIC) return false;
  uint16_t sensorCount; EEPROM.get(addr, sensorCount); addr += 2;
  if (sensorCount != 8) return false;
  uint16_t minOn[8], maxOn[8], minOff[8], maxOff[8];
  for (int i = 0; i < 8; i++) { EEPROM.get(addr, minOn[i]); addr += 2; EEPROM.get(addr, maxOn[i]); addr += 2; }
  for (int i = 0; i < 8; i++) { EEPROM.get(addr, minOff[i]); addr += 2; EEPROM.get(addr, maxOff[i]); addr += 2; }
  uint16_t crcStored; EEPROM.get(addr, crcStored);
  uint8_t buf[QTR_EEPROM_SIZE-2];
  for (int i = 0; i < QTR_EEPROM_SIZE-2; i++) buf[i] = EEPROM.read(QTR_EEPROM_ADDR + i);
  uint16_t crcCalc = crc16(buf, QTR_EEPROM_SIZE-2);
  if (crcStored != crcCalc) return false;
  // Copy to QTR
  for (int i = 0; i < 8; i++) {
    qtr.calibrationOn.minimum[i] = minOn[i];
    qtr.calibrationOn.maximum[i] = maxOn[i];
    if (qtr.calibrationOff.minimum) qtr.calibrationOff.minimum[i] = minOff[i];
    if (qtr.calibrationOff.maximum) qtr.calibrationOff.maximum[i] = maxOff[i];
  }
  qtr.calibrationOn.initialized = true;
  qtr.calibrationOff.initialized = true;
  return true;
}

void setup() {
  Serial.begin(9600);

  pinMode(CALIB_FLAG, INPUT_PULLUP); // Use pull-up so default is HIGH

  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);
  ledCountTest();

  if (Serial) {
    Log.begin(LOG_LEVEL_VERBOSE, &Serial);
  }

  Log.noticeln(F("============================================="));
  Log.noticeln(F("| Iniciando sistema do seguidor de linha    |"));
  Log.noticeln(F("| Feito em 07/08/25                         |"));
  Log.noticeln(F("| Log level: VERBOSE                        |"));
  Log.noticeln(F("============================================="));

  Log.verboseln(F("Inicializando servo..."));
  servoUltrassonico.attach(SERVO_PIN);
  servoUltrassonico.write(90);

  pinMode(LEDA, OUTPUT);
  pinMode(LEDB, OUTPUT);
  digitalWrite(LEDA, LOW);
  digitalWrite(LEDB, LOW);

  // Inicialização I2C e MPU6050
  Log.verboseln(F("Iniciando comunicação I2C..."));
  Wire.begin();

  Log.verboseln(F("Iniciando MPU..."));
  mpu.begin();
  for (int i = 0; i < MAX_TENTATIVAS_MPU; i++) {
    Log.verboseln("Tentativa %d de inicializar MPU...", i+1);
    if (verificarMPU()){
      break;
    }
    delay(100);
  }

  if (!mpuFuncionando) {
    Log.error("Falha ao inicializar MPU6050 após %d tentativas. Verifique as conexões e o sensor.", MAX_TENTATIVAS_MPU);
    ledBinOutput(ERRO_MPU);
  } else {
    Log.noticeln(F("MPU6050 inicializado com sucesso"));
  }
  
  Log.verboseln(F("Inicializando MPU6050..."));
  mpu.setAccelSensitivity(0);
  mpu.setGyroSensitivity(0);
  mpu.setThrottle(false);
  mpu.setNormalize(false);  // desativa normalização automática

  if (mpuFuncionando) {
    Log.verboseln(F("Calibrando MPU..."));
    mpu.calibrate(500);
    Log.noticeln(F("MPU6050 inicializado e calibrado"));
  } else {
    Log.error(F("Falha na inicialização do MPU6050"));
  }

  // Configuração de pinos com logs
  Log.verboseln(F("Configurando pinos de LED..."));

  ledBinOutput(CALIBRANDO);

  // Calibração dos sensores QTR
  Log.verboseln(F("Iniciando calibração dos sensores QTR..."));
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){43, 44, 45, 46, 47, 48, 49, 50}, 8);
  qtr.setEmitterPin(51);

  bool doCalib = digitalRead(CALIB_FLAG) == LOW; // Calibra se o pino estiver LOW
  Log.verboseln("Calibração QTR: %s", doCalib);
  bool loaded = false;
  if (!doCalib) {
    loaded = loadQTRCalibrationFromEEPROM();
    if (loaded) Log.noticeln(F("QTR calibração carregada da EEPROM."));
    else Log.warningln(F("EEPROM inválida, calibrando QTR."));
  }
  if (doCalib || !loaded) {
    for (int i = 0; i < 1000; i++) {
      qtr.calibrate();
      if (i % 50 == 0) Log.verboseln("Calibração QTR: %d/1000", i);
      delay(20);
    }
    pararMotores();
    Log.noticeln(F("Calibração QTR concluída"));
    saveQTRCalibrationToEEPROM();
    Log.noticeln(F("Calibração QTR salva na EEPROM."));
    while (true) {
      ledBinOutput(OP_PARADO);
    }
  }

  ledBinOutput(OP_INICIALIZANDO);
  
  delay(500);
  
  // Configuração dos LEDs dos sensores de cor
  Log.verboseln(F("Configurando LEDs dos sensores de cor..."));
  pinMode(LEDA, OUTPUT);
  pinMode(LEDB, OUTPUT);
  desligarLEDs();

  // Inicialização dos sensores de cor
  Log.verboseln(F("Inicializando sensores de cor..."));
  corSensores[0].tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_60X);
  corSensores[1].tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_60X);
  
  tcaSelect(0); 
  corSensores[0].inicializado = corSensores[0].tcs.begin();
  Log.verboseln("Sensor de cor 0 (esquerda) - %s", corSensores[0].inicializado ? "OK" : "FALHA");
  
  tcaSelect(1); 
  corSensores[1].inicializado = corSensores[1].tcs.begin();
  Log.verboseln("Sensor de cor 1 (direita) - %s", corSensores[1].inicializado ? "OK" : "FALHA");

  // Rotina inicial
  estadoAtual = Estado::INICIALIZANDO;
  Log.noticeln(F("Sistema inicializado com sucesso"));
  Log.noticeln(F("Estado inicial: SEGUINDO_LINHA"));
}

void loop() {
  Log.verboseln("Estado atual: %d", static_cast<int>(estadoAtual));
  
  switch(estadoAtual) {
    case Estado::SEGUINDO_LINHA: {
      ledBinOutput(OP_SEGUINDO_LINHA);

      int distancia = sonar.ping_cm();
      Log.verboseln("Distância medida: %d cm", distancia);
      if (distancia < DISTANCIA_OBSTACULO && distancia != 0) {
        estadoAtual = Estado::DESVIANDO_OBSTACULO;
        return;
      }

      lerSensores();
      media = calcularPosicaoLinha();

      if (media == 69) {
        estadoAtual = Estado::RESOLVENDO_BIFURCACAO;
        return;
      }

      // PID variables
      static float integral = 0.0;
      static float lastError = 0.0;
      float setpoint = 0.0;
      float error = setpoint - media;
      integral += error;
      float derivative = error - lastError;
      lastError = error;

      int correcaoPID = KP * error + KI * integral + KD * derivative;
      ultimaMedia = media;
      
      int velocidade1 = VEL_NORMAL - correcaoPID;
      int velocidade2 = VEL_NORMAL + correcaoPID;

      controleFino(velocidade1, velocidade2);
      // Remove delay para máxima velocidade
      break;
    }
    case Estado::INICIALIZANDO:{
      Log.warning(F("Estado INICIALIZANDO não deveria ser alcançado no loop"));
      estadoAtual = Estado::SEGUINDO_LINHA;
      return;
    }
    case Estado::RESOLVENDO_BIFURCACAO: {
      pararMotores();
      Log.noticeln(F("Iniciando RESOLVENDO_BIFURCACAO..."));
      ledBinOutput(OP_RESOLVENDO_BIFURCACAO);
    
      // Executa a resolução apenas uma vez
      static bool bifurcacaoResolvida = false;
      if (!bifurcacaoResolvida) {
        resolverBifurcacao();
        bifurcacaoResolvida = true;
        Log.noticeln(F("Bifurcação resolvida. Pronto para voltar a seguir linha"));
      }
      
      // Volta para SEGUINDO_LINHA apenas quando confirmado
      lerSensores();

      media = calcularPosicaoLinha();

      if (media != 69) { // Só volta se não estiver mais na bifurcação
        estadoAtual = Estado::SEGUINDO_LINHA;
        bifurcacaoResolvida = false;
        Log.noticeln(F("Voltando para SEGUINDO_LINHA"));
      }
      return;
    }
    case Estado::DESVIANDO_OBSTACULO: {
      ledBinOutput(OP_DESVIANDO_OBSTACULO);
      Log.noticeln(F("Iniciando desvio de obstáculo..."));
      desviarObstaculo();
      Log.noticeln(F("Obstáculo desviado. Voltando para SEGUINDO_LINHA"));
      estadoAtual = Estado::SEGUINDO_LINHA;
      break;
    }
    case Estado::PARADO:{
      Log.warning(F("ROBÔ PARADO!"));
      pararMotores();
      break;
    }
    default:{
      Log.warningln("PORQUE RAIOS ESTÁ NO DEFAULT? ISSO NÃO É POSSIVEL! O ESTADO ATUAL É: %d", estadoAtual);
    }
  }
  delay(INTERVALO_LEITURA); 
}

void controlarMotores(int esq, int dir, int vel = VEL_NORMAL) {
  motorFrenteEsquerdo.setSpeed(vel);
  motorFrenteDireito.setSpeed(vel);
  motorTrasEsquerdo.setSpeed(vel);
  motorTrasDireito.setSpeed(vel);
  motorFrenteEsquerdo.run(esq ? FORWARD : BACKWARD);
  motorFrenteDireito.run(dir ? FORWARD : BACKWARD);
  motorTrasEsquerdo.run(esq ? FORWARD : BACKWARD);
  motorTrasDireito.run(dir ? FORWARD : BACKWARD);
}

void controleFino(int esq, int dir) {
  int absEsq = constrain(abs(esq), 0, 255);
  int absDir = constrain(abs(dir), 0, 255);
  motorFrenteEsquerdo.setSpeed(absEsq);
  motorFrenteDireito.setSpeed(absDir);
  motorTrasEsquerdo.setSpeed(absEsq);
  motorTrasDireito.setSpeed(absDir);
  
  motorFrenteEsquerdo.run((esq > 0) ? FORWARD : BACKWARD);
  motorFrenteDireito.run((dir > 0) ? FORWARD : BACKWARD);
  motorTrasEsquerdo.run((esq > 0) ? FORWARD : BACKWARD);
  motorTrasDireito.run((dir > 0) ? FORWARD : BACKWARD);
}

void lerSensores() {
  qtr.readCalibrated(sensorValues); // Lê todos os 8 sensores do QTR-8RC
}

float calcularPosicaoLinha() {
  int somaPonderada = 0;
  int total = 0;
  const int posicoes[8] = {-12000, -6000, -3000, -1000, 1000, 3000, 6000, 12000};  
  for(int i = 0; i < 8; i++) {
    if (sensorValues[i] > LIMIAR_LINHA) {
      somaPonderada += posicoes[i];
      total++;
    }
  }
  if (total == 0) return 0;
  if (total == 8) return 69; // Indica bifurcação ou linha completa
  return somaPonderada / total / 1000;
}

void resolverBifurcacao() {
  Log.verboseln("Resolverbifurcacao()");
  ligarLEDs();
  Cores corA = detectarCor(0);
  Cores corB = detectarCor(1);
  desligarLEDs();

  Log.verboseln("corA: %d | corB: %d", corA, corB);

  // Checa erro antes de seguir
  if (corA == ERRO || corB == ERRO) {
    Log.errorln(F("Falha na detecção de cor! corA: %d | corB: %d"), corA, corB);
    pararMotores();
    delay(500);  // Ou lidar de forma mais inteligente
    return;
  }

  if (corA == VERDE && corB != VERDE) {
    virar90(ESQUERDA);
  } else if (corB == VERDE && corA != VERDE) {
    virar90(DIREITA);
  } else if (corA == VERDE && corB == VERDE) {
    virar90(DIREITA);
    virar90(DIREITA);
  } else {
    Log.warningln(F("Condição de bifurcação inesperada. Executando andarReto."));
    andarReto();
    delay(1350);
  }
}

void desviarObstaculo() {
  pararMotores(); delay(100);
  virar90(DIREITA); delay(100);
  andarReto(); delay(TEMPO_ORBITA / 2);
  unsigned long ultimaVirada = millis();

  while (true) {
    lerSensores();
    float HaLinha = calcularPosicaoLinha();
    if (HaLinha != 0) {
      pararMotores();
      virar90(DIREITA);
      return;
    }

    andarReto();
    if (millis() - ultimaVirada >= TEMPO_ORBITA) {
      pararMotores(); delay(100);
      virar90(ESQUERDA); delay(100);
      ultimaVirada = millis();
    }
    delay(10);
  }
}

void atualizarBufferCor(Cores novaCor) {
  bufferCores[indiceBuffer] = novaCor;
  indiceBuffer = (indiceBuffer + 1) % MAX_BUFFER_CORES;
}

Cores detectarCor(uint8_t canal) {
  if (canal > 1 || !corSensores[canal].inicializado) {
    // Fallback usando QTR caso o sensor de cor não esteja disponível
    unsigned int sensorValues[8];
    qtr.readCalibrated(sensorValues);
    int soma = 0;
    for (int i = 0; i < 8; i++) soma += sensorValues[i];
    int media = soma / 8;
    if (media > 400 && media < 900) return VERDE; // fallback supõe marca verde
    return PRETO;
  }

  tcaSelect(canal);
  uint16_t r, g, b, c;
  corSensores[canal].tcs.getRawData(&r, &g, &b, &c);

  Log.verboseln("R:%d G:%d B:%d", r, g, b);

  // Brancos e pretos
  if (r > 4000 && g > 4000 && b > 4000) return BRANCO;
  if (r < 1000 && g < 1000 && b < 1000) return PRETO;

  // Cinza — valores intermediários e parecidos
  if (abs((int)r - (int)g) < 500 &&
      abs((int)r - (int)b) < 500 &&
      abs((int)g - (int)b) < 500) {
    return CINZA;
  }

  // Determina cor predominante
  if (g > r && g > b) return VERDE;
  if (r > g && r > b) return VERMELHO;
  if (b > r && b > g) return AZUL;

  // Se empatar ou não for detectável
  return ERRO;
}

bool verificarMPU() {
  static int tentativasFalhas = 0;
  if (mpu.begin()) {
    mpu.readGyro();
    if (mpu.getGyroX() == 0 && mpu.getGyroY() == 0 && mpu.getGyroZ() == 0) tentativasFalhas++;
    else {
      tentativasFalhas = 0;
      mpuFuncionando = true;
      return true;
    }
  } else tentativasFalhas++;

  if (tentativasFalhas >= MAX_TENTATIVAS_MPU) {
    mpuFuncionando = false;
    ledBinOutput(ERRO_MPU);
  }
  return false;
}

void virar90(int direcao) {
  if (mpuFuncionando) virarComGiro(90, direcao);
  else {
    virarForte(direcao);
    delay(700);
    pararMotores();
  }
}

void virarComGiro(float angulo, int direcao) {
  pararMotores();

  if (!verificarMPU()) {
    Serial.println("MPU não detectada! Girando sem correção...");
    virarForte(direcao);
    delay(angulo * 10);
    pararMotores();
    return;
  }

  mpu.read();
  float yawInicial = mpu.getYaw();
  float yawAtual = yawInicial;
  float giroAcumulado = 0;

  const float margem = 2.0; // tolerância final

  while (fabs(giroAcumulado) < angulo - margem) {
    mpu.read();
    float novoYaw = mpu.getYaw();

    // Diferença entre leituras sucessivas (-180~180)
    float delta = novoYaw - yawAtual;
    if (delta > 180) delta -= 360;
    if (delta < -180) delta += 360;

    giroAcumulado += delta;
    yawAtual = novoYaw;

    float erro = angulo - fabs(giroAcumulado);
    int vel = map(erro, 0, angulo, VEL_CURVA, VEL_CURVA_EXTREMA);
    vel = constrain(vel, 50, VEL_CURVA);

    if (direcao == DIREITA)
      controlarMotores(ESQUERDA, DIREITA, vel);
    else
      controlarMotores(DIREITA, ESQUERDA, vel);


    delay(10);
  }

  pararMotores();
  delay(80);
}



void pararMotores() { controlarMotores(1, 1, 0); }
void andarReto()     { controlarMotores(1, 1, VEL_NORMAL); }
void andarRapido()   { controlarMotores(1, 1, 200); }
void andarTras()     { controlarMotores(0, 0, VEL_NORMAL + 10); }
void virar(int direcao)    { controlarMotores(direcao, !direcao, VEL_CURVA); }
void virarForte(int direcao){ controlarMotores(direcao, !direcao, VEL_CURVA_EXTREMA); }

void vencerResistenciaInicial() {
  controlarMotores(1, 1, VEL_RESISTENCIA);
  delay(100);
}

void desligarLEDs() {
  digitalWrite(LEDA, LOW);
  digitalWrite(LEDB, LOW);
  delay(200);
}

void ligarLEDs() {
  digitalWrite(LEDA, HIGH);
  digitalWrite(LEDB, HIGH);
  delay(200);
}

void tcaSelect(uint8_t channel) {
  if(channel > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

void ledCountTest() {
  digitalWrite(LED1, HIGH);
  delay(125);
  digitalWrite(LED2, HIGH);
  delay(125);
  digitalWrite(LED3, HIGH);
  delay(125);
  digitalWrite(LED4, HIGH);
  delay(12);
}

void ledBinOutput(int code) {
  unsigned long agora = millis();
  if ((agora - ultimaAtualizacaoLED < INTERVALO_LED_MS) && (code == ultimoCodigoMostrado)) return;

  digitalWrite(LED1, (code & 0b0001) ? HIGH : LOW);
  digitalWrite(LED2, (code & 0b0010) ? HIGH : LOW);
  digitalWrite(LED3, (code & 0b0100) ? HIGH : LOW);
  digitalWrite(LED4, (code & 0b1000) ? HIGH : LOW);

  ultimaAtualizacaoLED = agora;
  ultimoCodigoMostrado = code;
}
