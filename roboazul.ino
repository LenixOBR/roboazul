#include <AFMotor.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <GY521.h>
#include "SerialDebug.h"
#include <NewPing.h>

// Configurações de hardware
#define TRIGGER_PIN  48
#define ECHO_PIN     49
#define MAX_DISTANCE 50
#define TCAADDR 0x70
#define DIREITA 0
#define ESQUERDA 1
#define LEDA 46
#define LEDB 47

// Constantes de configuração
#define TEMPO_PRE90 1000
#define TEMPO_ORBITA 2000
#define VEL_NORMAL 88
#define VEL_RESISTENCIA 120
#define VEL_CURVA 140
#define VEL_CURVA_EXTREMA 220
#define INTERVALO_LEITURA 50
#define DISTANCIA_OBSTACULO 10

// Limiares dos sensores
#define LIMIAR_EXT_ESQ 240
#define LIMIAR_CENT_ESQ 110
#define LIMIAR_CENT_DIR 110
#define LIMIAR_EXT_DIR 140

// Estados do robô
enum Estado {
  SEGUINDO_LINHA,
  RESOLVENDO_BIFURCACAO,
  DESVIANDO_OBSTACULO,
  INICIALIZANDO
};

// Estruturas de dados
struct Sensor {
  uint8_t pin;
  uint16_t limiar;
  uint8_t valor;
};

struct CorSensor {
  Adafruit_TCS34725 tcs;
  bool inicializado = false;
};

// Variáveis globais
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
GY521 mpu(0x68);
Estado estadoAtual = INICIALIZANDO;
unsigned long ultimoTempoLeitura = 0;

AF_DCMotor motorFrenteEsquerdo(4);
AF_DCMotor motorFrenteDireito(1);
AF_DCMotor motorTrasEsquerdo(3);
AF_DCMotor motorTrasDireito(2);

Sensor sensores[4] = {
  {A0, LIMIAR_EXT_ESQ, 0},  // extEsq
  {A1, LIMIAR_CENT_ESQ, 0}, // centEsq
  {A2, LIMIAR_CENT_DIR, 0}, // centDir
  {A3, LIMIAR_EXT_DIR, 0}   // extDir
};

CorSensor corSensores[2];
Adafruit_TCS34725 tcs0(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_60X);
Adafruit_TCS34725 tcs1(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_60X);

// Protótipos de funções
void setup();
void loop();
void lerSensores();
float calcularPosicaoLinha();
const char* detectarCor(uint8_t canal);
void controlarMotores(int esqFrente, int dirFrente, int velocidade = VEL_NORMAL);
void pararMotores();
void andarReto();
void andarRapido();
void andarTras();
void virar(int direcao);
void virarForte(int direcao);
void virarComGiro(float anguloAlvo, int direcao);
void vencerResistenciaInicial();
void desligarLEDs();
void ligarLEDs();
void tcaSelect(uint8_t channel);
void resolverBifurcacao();
void desviarObstaculo();

void setup() {
  Serial.begin(9600);
  printlnA("Iniciando seguidor de linha...");
  Wire.begin();

  // Inicialização MPU6050
  mpu.begin();
  mpu.setAccelSensitivity(0);
  mpu.setGyroSensitivity(0);
  mpu.setThrottle(false);
  mpu.calibrate(1500);

  // Configuração LEDs
  pinMode(LEDA, OUTPUT);
  pinMode(LEDB, OUTPUT);
  desligarLEDs();

  // Inicialização motores
  pararMotores();

  // Inicialização sensores de cor
  tcaSelect(0);
  corSensores[0].inicializado = tcs0.begin();
  tcaSelect(1);
  corSensores[1].inicializado = tcs1.begin();

  vencerResistenciaInicial();
  estadoAtual = SEGUINDO_LINHA;
/*
  // Configuração SerialDebug
  #ifndef DEBUG_DISABLE_DEBUGGER
  debugAddGlobalInt(F("TEMPO_PRE90"), &TEMPO_PRE90);
  debugAddGlobalInt(F("TEMPO_ORBITA"), &TEMPO_ORBITA);
  for(int i = 0; i < 4; i++) {
    debugAddGlobalInt((String("sensor") + i).c_str(), &sensores[i].valor);
  }
  #endif
*/
}

void loop() {
  debugHandle();

  // Máquina de estados principal
  switch(estadoAtual) {
    case INICIALIZANDO:
      estadoAtual = SEGUINDO_LINHA;
      break;
      
    case SEGUINDO_LINHA:
      if(millis() - ultimoTempoLeitura >= INTERVALO_LEITURA) {
        lerSensores();
        ultimoTempoLeitura = millis();
        
        float media = calcularPosicaoLinha();
        
        // Verificação de obstáculo
        int distancia = sonar.ping_cm();
        if(distancia < DISTANCIA_OBSTACULO && distancia != 0) {
          estadoAtual = DESVIANDO_OBSTACULO;
          break;
        }
        
        // Verificação de bifurcação
        if(media == 69) {
          estadoAtual = RESOLVENDO_BIFURCACAO;
          break;
        }
        
        // Controle normal de seguimento
        if(media < -1.5) virarForte(ESQUERDA);
        else if(media > 1.5) virarForte(DIREITA);
        else if(media < -0.7) virar(ESQUERDA);
        else if(media > 0.7) virar(DIREITA);
        else andarReto();
      }
      break;
      
    case RESOLVENDO_BIFURCACAO:
      resolverBifurcacao();
      estadoAtual = SEGUINDO_LINHA;
      break;
      
    case DESVIANDO_OBSTACULO:
      desviarObstaculo();
      estadoAtual = SEGUINDO_LINHA;
      break;
  }
}

void lerSensores() {
  for(int i = 0; i < 4; i++) {
    sensores[i].valor = analogRead(sensores[i].pin) > sensores[i].limiar ? 1 : 0;
    printV("Sensor"); printV(i); printV(":"); printlnV(analogRead(sensores[i].pin));
  }
}

float calcularPosicaoLinha() {
  int total = sensores[0].valor + sensores[1].valor + sensores[2].valor + sensores[3].valor;
  
  if(total == 4 || (sensores[0].valor && sensores[3].valor)) {
    pararMotores();
    return 69; // Código especial para bifurcação
  }
  
  if(total == 0) return 0;
  
  return (sensores[0].valor * -2 + sensores[1].valor * -1 + 
          sensores[2].valor * 1 + sensores[3].valor * 2) / (float)total;
}

const char* detectarCor(uint8_t canal) {
  if(canal > 1 || !corSensores[canal].inicializado) return "erro";
  
  tcaSelect(canal);
  uint16_t r, g, b, c;
  corSensores[canal].tcs.getRawData(&r, &g, &b, &c);
  
  if(r < 3000 && g < 4400 && b < 3400) return "preto";
  if(r > 6000 && g > 7500 && b > 6500) return "branco";
  return "colorido";
}

void resolverBifurcacao() {
  ligarLEDs();
  const char* corA = detectarCor(0);
  const char* corB = detectarCor(1);
  desligarLEDs();
  
  vencerResistenciaInicial();
  
  if(strcmp(corA, "preto") == 0 && strcmp(corB, "preto") == 0) {
    andarTras();
    delay(500);
  }
  else if(strcmp(corA, "preto") == 0) {
    virarForte(DIREITA);
    delay(50);
  }
  else if(strcmp(corB, "preto") == 0) {
    virarForte(ESQUERDA);
    delay(50);
  }
  else if(strcmp(corA, "colorido") == 0 && strcmp(corB, "colorido") != 0) {
    andarReto();
    delay(TEMPO_PRE90);
    virarComGiro(90, ESQUERDA);
  }
  else if(strcmp(corA, "colorido") != 0 && strcmp(corB, "colorido") == 0) {
    andarReto();
    delay(TEMPO_PRE90);
    virarComGiro(90, DIREITA);
  }
  else if(strcmp(corA, "colorido") == 0 && strcmp(corB, "colorido") == 0) {
    virarComGiro(90, DIREITA);
    virarComGiro(90, DIREITA);
  }
  else {
    andarReto();
    delay(1350);
  }
}

void desviarObstaculo() {
  pararMotores();
  delay(100);
  virarComGiro(90, ESQUERDA);
  pararMotores();
  delay(100);
  
  andarReto();
  delay(TEMPO_ORBITA / 2);
  
  unsigned long ultimaVirada = millis();
  
  while(true) {
    lerSensores();
    if(sensores[0].valor || sensores[1].valor || sensores[2].valor || sensores[3].valor) {
      pararMotores();
      virarComGiro(90, ESQUERDA);
      return;
    }
    
    andarReto();
    
    if(millis() - ultimaVirada >= 2000) {
      pararMotores();
      delay(100);
      virarComGiro(90, DIREITA);
      pararMotores();
      delay(100);
      ultimaVirada = millis();
    }
    
    delay(10);
  }
}

void controlarMotores(int esqFrente, int dirFrente, int velocidade) {
  motorFrenteEsquerdo.setSpeed(velocidade);
  motorFrenteDireito.setSpeed(velocidade);
  motorTrasEsquerdo.setSpeed(velocidade);
  motorTrasDireito.setSpeed(velocidade);
  
  motorFrenteEsquerdo.run(esqFrente ? FORWARD : BACKWARD);
  motorTrasEsquerdo.run(esqFrente ? FORWARD : BACKWARD);
  motorFrenteDireito.run(dirFrente ? FORWARD : BACKWARD);
  motorTrasDireito.run(dirFrente ? FORWARD : BACKWARD);
}

void pararMotores() {
  motorFrenteEsquerdo.run(RELEASE);
  motorFrenteDireito.run(RELEASE);
  motorTrasEsquerdo.run(RELEASE);
  motorTrasDireito.run(RELEASE);
}

void andarReto() {
  controlarMotores(1, 1, VEL_NORMAL);
}

void andarRapido() {
  controlarMotores(1, 1, 200);
}

void andarTras() {
  controlarMotores(0, 0, VEL_NORMAL + 10);
}

void virar(int direcao) {
  controlarMotores(direcao, !direcao, VEL_CURVA);
}

void virarForte(int direcao) {
  controlarMotores(direcao, !direcao, VEL_CURVA_EXTREMA);
}

void virarComGiro(float anguloAlvo, int direcao) {
  float yawInicial = mpu.getYaw();
  float alvoYaw = fmod((yawInicial + (direcao == DIREITA ? anguloAlvo : -anguloAlvo) + 360), 360);
  
  while(true) {
    mpu.readGyro();
    float yawAtual = fmod(mpu.getYaw(), 360);
    float delta = fmod((yawAtual - alvoYaw + 360), 360);
    
    if(delta < 5 || delta > 355) break;
    
    virar(direcao);
    delay(10);
  }
  pararMotores();
}

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