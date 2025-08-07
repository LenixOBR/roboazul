#include <AFMotor.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <GY521.h>
#include <NewPing.h>
#include <Servo.h>
#include <ArduinoLog.h>
#include <QTRSensors.h>
#include <math.h>

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
#define VEL_NORMAL 65
#define VEL_RESISTENCIA 120
#define VEL_CURVA 140
#define VEL_CURVA_EXTREMA 220
#define INTERVALO_LEITURA 50
#define DISTANCIA_OBSTACULO 10
#define DISTANCIA_PARADA 15
#define DISTANCIA_MINIMA_VIRADA 10
#define LIMIAR_LINHA 900

#define MAX_SATURACAO_SEGUIDA 50     // Número de leituras saturadas seguidas para considerar defeituoso
#define MAX_NORMAL_SEGUIDO 50        // Número de leituras normais seguidas para reviver
#define LIMIAR_LINHA 500             // Ajuste conforme seu sensor

#define SERVO_PIN 10
#define ANGULO_FRENTE 90
#define ANGULO_ESQUERDA 180
#define ANGULO_DIREITA 0
#define MAX_FALHAS_CONTINUAS 10
#define MAX_BUFFER_CORES 10

#define KP 100.0f
#define KI 0.0f
#define KD 10.0f

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
  CALIBRANDO = 10,             // 1010 - Calibrando
  OP_INICIALIZANDO = 15     // 1111 - Inicializando
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

bool sensorDigital[8] = {false};
int contagemSaturacaoQTR[8] = {0};
int contagemNormalQTR[8] = {0};
bool sensorDefeituoso[8] = {false};
int sensoresValidos = 8;  // ou inicialize conforme necessário

float media = 0.0;
float ultimaMedia = 0.0;

void setup() {
  Serial.begin(9600);

  if (Serial) {
    Log.begin(LOG_LEVEL_VERBOSE, &Serial);
  }

  Log.noticeln(F("============================================="));
  Log.noticeln(F("| Iniciando sistema do seguidor de linha    |"));
  Log.noticeln(F("| Feito em 07/08/25                         |"));
  Log.noticeln(F("| Log level: VERBOSE                        |"));
  Log.noticeln(F("============================================="));
  
  pinMode(LEDA, OUTPUT);
  pinMode(LEDB, OUTPUT);
  digitalWrite(LEDA, LOW);
  digitalWrite(LEDB, LOW);

  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);

  // Inicialização I2C e MPU6050
  Log.verboseln(F("Iniciando comunicação I2C..."));
  Wire.begin();

  Log.verboseln(F("Iniciando MPU..."));
  mpu.begin();
  for (int i = 0; i < MAX_TENTATIVAS_MPU; i++) {
    Log.verboseln("Tentativa %d de inicializar MPU...", i+1);
    if (verificarMPU()) break;
    delay(100);
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

  Log.verboseln(F("Inicializando servo..."));
  servoUltrassonico.attach(SERVO_PIN);
  servoUltrassonico.write(90);

  // Configuração de pinos com logs
  Log.verboseln(F("Configurando pinos de LED..."));

  ledBinOutput(CALIBRANDO);

  // Calibração dos sensores QTR
  Log.verboseln(F("Iniciando calibração dos sensores QTR..."));
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){43, 44, 45, 46, 47, 48, 49, 50, 51}, 8);
  for (int i = 0; i < 250; i++) {
    qtr.calibrate();
    if(i % 50 == 0) Log.verboseln("Calibração QTR: %d/250", i);
    delay(20);
  }
  Log.noticeln(F("Calibração QTR concluída"));

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
  Log.verboseln(F("Vencendo resistência inicial dos motores..."));
  vencerResistenciaInicial();
  
  estadoAtual = SEGUINDO_LINHA;
  Log.noticeln(F("Sistema inicializado com sucesso"));
  Log.noticeln(F("Estado inicial: SEGUINDO_LINHA"));
}

void loop() {
  Log.verboseln(F("--- Início do loop ---"));
  Log.verboseln("Estado atual: %d", estadoAtual);
  
  switch(estadoAtual) {
    case INICIALIZANDO:
      Log.warning(F("Estado INICIALIZANDO não deveria ser alcançado no loop"));
      estadoAtual = SEGUINDO_LINHA;
      break;

    case SEGUINDO_LINHA:
      Log.verboseln(F("Executando SEGUINDO_LINHA..."));
      ledBinOutput(OP_SEGUINDO_LINHA);
      lerSensores();
      media = calcularPosicaoLinha();
      Log.verboseln("Posição média da linha: %F", media);

      int distancia = sonar.ping_cm();
      Log.verboseln("Distância do obstáculo: %d cm", distancia);
      
      if (distancia < DISTANCIA_OBSTACULO && distancia != 0) {
        Log.noticeln(F("Obstáculo detectado! Mudando para DESVIANDO_OBSTACULO"));
        estadoAtual = DESVIANDO_OBSTACULO;
        break;
      }

      if (media == 69) {
        Log.noticeln(F("Bifurcação detectada! Mudando para RESOLVENDO_BIFURCACAO"));
        estadoAtual = RESOLVENDO_BIFURCACAO;
        break;
      }

      float correcaoPID = KP * media + KD * (ultimaMedia - media); 
      ultimaMedia = media;

      Log.verbose(" Correção PID: %c", correcaoPID);

      int velocidade1 = VEL_NORMAL - correcaoPID;
      int velocidade2 = VEL_NORMAL + correcaoPID;

      controleFino(velocidade1, velocidade2);
      
      break;

    case RESOLVENDO_BIFURCACAO:
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
        estadoAtual = SEGUINDO_LINHA;
        bifurcacaoResolvida = false;
        Log.noticeln(F("Voltando para SEGUINDO_LINHA"));
      }
      break;

    case DESVIANDO_OBSTACULO:
      Log.noticeln(F("Iniciando desvio de obstáculo..."));
      desviarObstaculo();
      Log.noticeln(F("Obstáculo desviado. Voltando para SEGUINDO_LINHA"));
      estadoAtual = SEGUINDO_LINHA;
      break;
    case PARADO:
      Log.warning(F("ROBÔ PARADO!"));
      pararMotores();
      break;
    default:
      Log.warningln("PORQUE RAIOS ESTÁ NO DEFAULT? ISSO NÃO É POSSIVEL! O ESTADO ATUAL É: %d", estadoAtual);
  }
  
  Log.verboseln(F("--- Fim do loop ---"));
  delay(INTERVALO_LEITURA);
}

void lerSensores() {
  unsigned int sensorValues[8];
  qtr.readCalibrated(sensorValues); // Lê todos os 8 sensores do QTR-8RC
  
  for (int i = 0; i < 8; i++) {
    Log.verboseln("Sensor %d: %d", i, sensorValues[i]);

    // Verifica se está saturado
    if (sensorValues[i] >= 1000) {
      contagemSaturacaoQTR[i]++;
      contagemNormalQTR[i] = 0;  // Zera contador de leituras normais
    } else {
      contagemSaturacaoQTR[i] = 0;
      contagemNormalQTR[i]++;    // Conta quantas vezes o sensor está normal
    }

    // Marca sensor como defeituoso se estiver saturando por muito tempo
    if (contagemSaturacaoQTR[i] >= MAX_SATURACAO_SEGUIDA && !sensorDefeituoso[i]) {
      sensorDefeituoso[i] = true;
      sensoresValidos--;
      Log.warning("Sensor QTR %d marcado como defeituoso!", i);
    }

    // Revive sensor se voltou ao normal por tempo suficiente
    if (sensorDefeituoso[i] && contagemNormalQTR[i] >= MAX_NORMAL_SEGUIDO) {
      sensorDefeituoso[i] = false;
      sensoresValidos++;
      Log.notice("Sensor QTR %d voltou ao funcionamento normal.", i);
    }

    // Ignora sensores defeituosos na leitura digital
    if (sensorDefeituoso[i]) {
      sensorDigital[i] = 0;
      continue;
    }

    sensorDigital[i] = sensorValues[i] > LIMIAR_LINHA ? 1 : 0;
  }
}

float calcularPosicaoLinha() {
  int total = 0;
  float somaPonderada = 0;

  for(int i = 0; i < 8; i++) {
    int valor = sensorDigital[i];
    total += valor;

    // Peso mais forte: exponencial centrada em 3.5
    float peso = pow((i - 3.5), 3);  // Cubo enfatiza sensores das extremidades
    somaPonderada += valor * peso;
  }

  if(sensoresValidos == 0) {
    ledBinOutput(ERRO_QTR_SATURADO);
    return 0;
  }

  // Linha larga ou cruzamento
  if(total >= sensoresValidos - 1) return 69;

  // Fator de amplificação baseado na quantidade de sensores ativados
  float fatorMultiplicador = 1 + (sensoresValidos - 1) * 0.2;

  // Normaliza e aplica fator
  return (somaPonderada / (pow(3.5, 3) * total)) * fatorMultiplicador;
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

  vencerResistenciaInicial();

  if (corA == PRETO && corB == PRETO) {
    andarTras();
    delay(500);
  } else if (corA == PRETO) {
    virarForte(DIREITA);
    delay(50);
  } else if (corB == PRETO) {
    virarForte(ESQUERDA);
    delay(50);
  } else if (corA == COLORIDO && corB != COLORIDO) {
    virar90(ESQUERDA);
  } else if (corB == COLORIDO && corA != COLORIDO) {
    virar90(DIREITA);
  } else if (corA == COLORIDO && corB == COLORIDO) {
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
    Serial.println("MPU não detectada! Girando sem correção...");
    virarForte(direcao);
    delay(angulo * 10);  // valor empírico
    pararMotores();
    return;
  }

  mpu.read();
  float yawInicial = mpu.getYaw();

  // Define o alvo com adição ou subtração simples
  float alvoYaw = yawInicial + (direcao == DIREITA ? angulo : -angulo);

  Serial.print("Yaw inicial: ");
  Serial.println(yawInicial);
  Serial.print("Alvo Yaw: ");
  Serial.println(alvoYaw);

  while (true) {
    mpu.read();
    float yawAtual = mpu.getYaw();

    float restante = (direcao == DIREITA) ? (yawAtual - alvoYaw) : (alvoYaw - yawAtual);

    Serial.print("Yaw atual: ");
    Serial.println(yawAtual);
    Serial.print("Restante: ");
    Serial.println(restante);

    // Verifica se já passou ou chegou no alvo
    if (restante < 10) break;

    virar(direcao);
    delay(10);  // controle simples de loop
  }

  pararMotores();
  Serial.println("Giro finalizado.");
}

void controlarMotores(int esq, int dir, int vel = VEL_NORMAL) {
  motorFrenteEsquerdo.setSpeed(vel);
  motorFrenteDireito.setSpeed(vel);
  motorTrasEsquerdo.setSpeed(vel);
  motorTrasDireito.setSpeed(vel);
  motorFrenteEsquerdo.run(esq ? FORWARD : BACKWARD);
  motorTrasEsquerdo.run(esq ? FORWARD : BACKWARD);
  motorFrenteDireito.run(dir ? FORWARD : BACKWARD);
  motorTrasDireito.run(dir ? FORWARD : BACKWARD);
}

void controleFino(int esq, int dir) {
  int absEsq = abs(esq);
  int absDir = abs(dir);
  motorFrenteEsquerdo.setSpeed(absEsq);
  motorFrenteDireito.setSpeed(absEsq);
  motorTrasEsquerdo.setSpeed(absDir);
  motorTrasDireito.setSpeed(absDir);
  
  motorFrenteEsquerdo.run((esq > 0) ? FORWARD : BACKWARD);
  motorTrasEsquerdo.run((esq > 0) ? FORWARD : BACKWARD);
  motorFrenteDireito.run((dir > 0) ? FORWARD : BACKWARD);
  motorTrasDireito.run((dir > 0) ? FORWARD : BACKWARD);
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
