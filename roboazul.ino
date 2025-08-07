#include <AFMotor.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <GY521.h>
#include <NewPing.h>
#include <Servo.h>
#include <ArduinoLog.h>
#include <QTRSensors.h>

QTRSensors qtr;

#define LEDA 41
#define LEDB 42
#define TRIGGER_PIN  40
#define ECHO_PIN     39
#define CALIB_FLAG  34
#define LED1 38
#define LED2 37
#define LED3 35
#define LED4 36
#define MAX_DISTANCE 50
#define TCAADDR 0x70
#define DIREITA 0
#define ESQUERDA 1

#define TEMPO_PRE90 1000
#define TEMPO_ORBITA 2000
#define VEL_NORMAL 88
#define VEL_RESISTENCIA 120
#define VEL_CURVA 140
#define VEL_CURVA_EXTREMA 220
#define INTERVALO_LEITURA 50
#define DISTANCIA_OBSTACULO 10
#define DISTANCIA_PARADA 15
#define DISTANCIA_MINIMA_VIRADA 10
#define LIMIAR_LINHA 900

#define LIMITE_ZERO 20
#define MAX_CONTAGEM_FALHA 50
#define MAX_SATURACAO_SEGUIDA 30

#define SERVO_PIN 10
#define ANGULO_FRENTE 90
#define ANGULO_ESQUERDA 180
#define ANGULO_DIREITA 0
#define MAX_FALHAS_CONTINUAS 10
#define MAX_BUFFER_CORES 10

// Enumerações com descrições detalhadas
enum Estado {
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
  OP_INICIALIZANDO = 15        // 1111 - Inicializando
};

enum Cores {
  PRETO,     // Cor preta (linha)
  BRANCO,    // Cor branca (fora da linha)
  COLORIDO,  // Cor colorida (marcações especiais)
  ERRO       // Erro na leitura de cor
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
Estado estadoAtual = INICIALIZANDO;
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

bool sensorDigital[8] = {};
int contagemZeroQTR[8] = {0};
int contagemSaturacaoQTR[8] = {0};
bool sensorDefeituoso[8] = {false};

void setup() {
  Serial.begin(9600);
  Log.begin(LOG_LEVEL_VERBOSE, &Serial);
  Log.notice(F("============================================="));
  Log.notice(F("| Iniciando sistema do seguidor de linha    |"));
  Log.notice(F("| Feito em 07/08/25                         |"));
  Log.notice(F("| Log level: VERBOSE                        |"));
  Log.notice(F("============================================="));
  
  Log.verbose(F("Inicializando servo..."));
  servoUltrassonico.attach(SERVO_PIN);
  servoUltrassonico.write(90);

  // Configuração de pinos com logs
  Log.verbose(F("Configurando pinos de LED..."));
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);
  ledBinOutput(OP_INICIALIZANDO);

  // Calibração dos sensores QTR
  Log.verbose(F("Iniciando calibração dos sensores QTR..."));
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){43, 44, 45, 46, 47, 48, 49, 50}, 8);
  for (int i = 0; i < 250; i++) {
    qtr.calibrate();
    if(i % 50 == 0) Log.verbose("Calibração QTR: %d/250", i);
    delay(20);
  }
  Log.notice(F("Calibração QTR concluída"));

  // Inicialização I2C e MPU6050
  Log.verbose(F("Iniciando comunicação I2C..."));
  Wire.begin();
  
  Log.verbose(F("Inicializando MPU6050..."));
  mpu.begin();
  mpu.setAccelSensitivity(0);
  mpu.setGyroSensitivity(0);
  mpu.setThrottle(false);
  
  for (int i = 0; i < MAX_TENTATIVAS_MPU; i++) {
    Log.verbose("Tentativa %d de inicializar MPU...", i+1);
    if (verificarMPU()) break;
    delay(100);
  }
  
  if (mpuFuncionando) {
    Log.verbose(F("Calibrando MPU..."));
    mpu.calibrate(5);
    Log.notice(F("MPU6050 inicializado e calibrado"));
  } else {
    Log.error(F("Falha na inicialização do MPU6050"));
  }

  // Configuração dos LEDs dos sensores de cor
  Log.verbose(F("Configurando LEDs dos sensores de cor..."));
  pinMode(LEDA, OUTPUT);
  pinMode(LEDB, OUTPUT);
  desligarLEDs();

  // Inicialização dos sensores de cor
  Log.verbose(F("Inicializando sensores de cor..."));
  corSensores[0].tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_60X);
  corSensores[1].tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_60X);
  
  tcaSelect(0); 
  corSensores[0].inicializado = corSensores[0].tcs.begin();
  Log.verbose("Sensor de cor 0 (esquerda) - %s", corSensores[0].inicializado ? "OK" : "FALHA");
  
  tcaSelect(1); 
  corSensores[1].inicializado = corSensores[1].tcs.begin();
  Log.verbose("Sensor de cor 1 (direita) - %s", corSensores[1].inicializado ? "OK" : "FALHA");

  // Rotina inicial
  Log.verbose(F("Vencendo resistência inicial dos motores..."));
  vencerResistenciaInicial();
  
  estadoAtual = SEGUINDO_LINHA;
  Log.notice(F("Sistema inicializado com sucesso"));
  Log.notice(F("Estado inicial: SEGUINDO_LINHA"));
}

void loop() {
  Log.verbose(F("--- Início do loop ---"));
  Log.verbose("Estado atual: %d", estadoAtual);
  
  switch(estadoAtual) {
    case INICIALIZANDO:
      Log.warning(F("Estado INICIALIZANDO não deveria ser alcançado no loop"));
      estadoAtual = SEGUINDO_LINHA;
      break;

    case SEGUINDO_LINHA:
      Log.verbose(F("Executando SEGUINDO_LINHA..."));
      ledBinOutput(OP_SEGUINDO_LINHA);
      lerSensores();
      float media = calcularPosicaoLinha();
      Log.verbose("Posição média da linha: %F", media);

      int distancia = sonar.ping_cm();
      Log.verbose("Distância do obstáculo: %d cm", distancia);
      
      if (distancia < DISTANCIA_OBSTACULO && distancia != 0) {
        Log.notice(F("Obstáculo detectado! Mudando para DESVIANDO_OBSTACULO"));
        estadoAtual = DESVIANDO_OBSTACULO;
        break;
      }

      if (media == 69) {
        Log.notice(F("Bifurcação detectada! Mudando para RESOLVENDO_BIFURCACAO"));
        estadoAtual = RESOLVENDO_BIFURCACAO;
        break;
      }

      if (media < -1.5) {
        Log.verbose(F("Virando forte para ESQUERDA"));
        virarForte(ESQUERDA);
      }
      else if (media > 1.5) {
        Log.verbose(F("Virando forte para DIREITA"));
        virarForte(DIREITA);
      }
      else if (media < -0.7) {
        Log.verbose(F("Virando suave para ESQUERDA"));
        virar(ESQUERDA);
      }
      else if (media > 0.7) {
        Log.verbose(F("Virando suave para DIREITA"));
        virar(DIREITA);
      }
      else {
        Log.verbose(F("Andando reto"));
        andarReto();
      }
      break;

    case RESOLVENDO_BIFURCACAO:
        Log.notice(F("Resolvendo bifurcação..."));
      resolverBifurcacao();
      Log.notice(F("Bifurcação resolvida. Voltando para SEGUINDO_LINHA"));
      estadoAtual = SEGUINDO_LINHA;
      break;

    case DESVIANDO_OBSTACULO:
      Log.notice(F("Iniciando desvio de obstáculo..."));
      desviarObstaculo();
      Log.notice(F("Obstáculo desviado. Voltando para SEGUINDO_LINHA"));
      estadoAtual = SEGUINDO_LINHA;
      break;

    case PARADO:
      Log.warning(F("ROBÔ PARADO!"));
      pararMotores();
      break;
  }
  
  Log.verbose(F("--- Fim do loop ---"));
  delay(INTERVALO_LEITURA);
}

void lerSensores() {
  unsigned int sensorValues[8];
  qtr.readCalibrated(sensorValues); // Lê todos os 8 sensores do QTR-8RC
  
  for (int i = 0; i < 8; i++) {
    Log.verbose("Sensor %d: %d", i, sensorValues[i]);

    // Verifica se sensor deu 0
    if (sensorValues[i] < LIMITE_ZERO) {
      contagemZeroQTR[i]++;
    } else {
      contagemZeroQTR[i] = 0;
    }

    // Verifica se deu 1000 (saturado)
    if (sensorValues[i] >= 1000) {
      contagemSaturacaoQTR[i]++;
    } else {
      contagemSaturacaoQTR[i] = 0;
    }

    // Marca sensor como defeituoso se ultrapassar limites
    if (contagemZeroQTR[i] >= MAX_CONTAGEM_FALHA || contagemSaturacaoQTR[i] >= MAX_SATURACAO_SEGUIDA) {
      if (!sensorDefeituoso[i]) {
        sensorDefeituoso[i] = true;
        Log.warning("Sensor QTR %d marcado como defeituoso!", i);
      }
    }

    // Ignora sensores defeituosos
    if (sensorDefeituoso[i]) {
      sensorDigital[i] = 0;
      continue;
    }

    sensorDigital[i] = sensorValues[i] > LIMIAR_LINHA ? 1 : 0;
  }
}

float calcularPosicaoLinha() {
  int total = 0;
  int somaPonderada = 0;
  int sensoresValidos = 0;

  for(int i = 0; i < 8; i++) {
    total += sensorDigital[i];
    somaPonderada += sensorDigital[i] * (i - 3.5);
    sensoresValidos++;
  }

  if(sensoresValidos == 0) {
    ledBinOutput(ERRO_QTR_SATURADO);
    return 0;
  }

  if(total == sensoresValidos) return 69;
  return somaPonderada / (3.5 * total);
}

void resolverBifurcacao() {
  ligarLEDs();
  Cores corA = detectarCor(0);
  Cores corB = detectarCor(1);
  desligarLEDs();

  vencerResistenciaInicial();

  if (corA == PRETO && corB == PRETO) andarTras(), delay(500);
  else if (corA == PRETO) virarForte(DIREITA), delay(50);
  else if (corB == PRETO) virarForte(ESQUERDA), delay(50);
  else if (corA == COLORIDO && corB != COLORIDO) andarReto(), delay(TEMPO_PRE90), virar90(ESQUERDA);
  else if (corB == COLORIDO && corA != COLORIDO) andarReto(), delay(TEMPO_PRE90), virar90(DIREITA);
  else if (corA == COLORIDO && corB == COLORIDO) virar90(DIREITA), virar90(DIREITA);
  else andarReto(), delay(1350);
}

void desviarObstaculo() {
  pararMotores(); delay(100);
  virar90(ESQUERDA); delay(100);
  andarReto(); delay(TEMPO_ORBITA / 2);
  unsigned long ultimaVirada = millis();

  while (true) {
    lerSensores();
    float HaLinha = calcularPosicaoLinha();
    if (HaLinha != 0) {
      pararMotores();
      virar90(ESQUERDA);
      return;
    }

    andarReto();
    if (millis() - ultimaVirada >= 2000) {
      pararMotores(); delay(100);
      virar90(DIREITA); delay(100);
      ultimaVirada = millis();
    }
    delay(10);
  }
}

void atualizarBufferCor(Cores novaCor) {
  bufferCores[indiceBuffer] = novaCor;
  indiceBuffer = (indiceBuffer + 1) % MAX_BUFFER_CORES;
}

bool detectarParadaVermelho() {
  int brancoSeguido = 0;
  bool viuColorido = false;
  for (int i = 0; i < MAX_BUFFER_CORES; i++) {
    int idx = (indiceBuffer + i) % MAX_BUFFER_CORES;
    if (bufferCores[idx] == COLORIDO) viuColorido = true;
    if (viuColorido && bufferCores[idx] == BRANCO) brancoSeguido++;
  }
  return viuColorido && brancoSeguido >= 4;
}

Cores detectarCor(uint8_t canal) {
  if (canal > 1 || !corSensores[canal].inicializado) {
    unsigned int sensorValues[8];
    qtr.readCalibrated(sensorValues);
    int soma = 0;
    for (int i = 0; i < 8; i++) soma += sensorValues[i];
    int media = soma / 8;
    if (media > 400 && media < 900) return COLORIDO;
    return PRETO;
  }

  tcaSelect(canal);
  uint16_t r, g, b, c;
  corSensores[canal].tcs.getRawData(&r, &g, &b, &c);

  if (r < 3000 && g < 4400 && b < 3400) return PRETO;
  if (r > 6000 && g > 7500 && b > 6500) return BRANCO;
  return COLORIDO;
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
    virarForte(direcao); delay(angulo * 10);
    pararMotores();
    return;
  }

  mpu.readGyro();
  float yawInicial = mpu.getYaw();
  float alvoYaw = fmod((yawInicial + (direcao == DIREITA ? angulo : -angulo) + 360), 360);

  while (true) {
    mpu.readGyro();
    float yawAtual = fmod(mpu.getYaw(), 360);
    float delta = fmod((yawAtual - alvoYaw + 360), 360);
    if (delta < 5 || delta > 355) break;
    virar(direcao);
    delay(10);
  }
  pararMotores();
}

void controlarMotores(int esqFrente, int dirFrente, int vel = VEL_NORMAL) {
  motorFrenteEsquerdo.setSpeed(vel);
  motorFrenteDireito.setSpeed(vel);
  motorTrasEsquerdo.setSpeed(vel);
  motorTrasDireito.setSpeed(vel);
  motorFrenteEsquerdo.run(esqFrente ? FORWARD : BACKWARD);
  motorTrasEsquerdo.run(esqFrente ? FORWARD : BACKWARD);
  motorFrenteDireito.run(dirFrente ? FORWARD : BACKWARD);
  motorTrasDireito.run(dirFrente ? FORWARD : BACKWARD);
}

void pararMotores() { controlarMotores(1, 1, 0); }
void andarReto()     { controlarMotores(1, 1, VEL_NORMAL); }
void andarRapido()   { controlarMotores(1, 1, 200); }
void andarTras()     { controlarMotores(0, 0, VEL_NORMAL + 10); }
void virar(int d)    { controlarMotores(d, !d, VEL_CURVA); }
void virarForte(int d){ controlarMotores(d, !d, VEL_CURVA_EXTREMA); }

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
