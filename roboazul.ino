#include <AFMotor.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <GY521.h>


#include <NewPing.h>

#define TRIGGER_PIN  48  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     49  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 50 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.



int tempopre90 = 1000;
int tempoOrbita = 2000;


#define TCAADDR 0x70         // Endereço do TCA9548A
GY521 mpu(0x68);             // Endereço padrão do MPU6050

#define DIREITA 0
#define ESQUERDA 1

// Definição dos motores
AF_DCMotor motorFrenteEsquerdo(4);
AF_DCMotor motorFrenteDireito(1);
AF_DCMotor motorTrasEsquerdo(3);
AF_DCMotor motorTrasDireito(2);

Adafruit_TCS34725* tcs0 = nullptr;
Adafruit_TCS34725* tcs1 = nullptr;

bool VerdeA;
bool VerdeB;


const int LEDA = 46;
const int LEDB = 47;

// Pinos dos sensores
const int sensorExtEsquerdo = A0;
const int sensorCentEsquerdo = A1;
const int sensorCentDireito = A2;
const int sensorExtDireito = A3;
// Variáveis globais dos sensores (valores binários: 0 ou 1)
int extEsq = 0;
int centEsq = 0;
int centDir = 0;
int extDir = 0;


// Limiar para detecção de linha
// Limiar individual para cada sensor
const int limiarExtEsq  = 240;
const int limiarCentEsq = 110;
const int limiarCentDir = 110;
const int limiarExtDir  = 140;

const int velocidadeNormal = 88;
const int velocidadeResistencia = 120;
const int velocidadeCurva = 140;
const int velocidadeCurvaExtrema = 220;

void desligarLEDs(){
  digitalWrite(LEDA, LOW);
  digitalWrite(LEDB, LOW);
  delay(200);
}

void ligarLEDs(){
  digitalWrite(LEDA, HIGH);
  digitalWrite(LEDB, HIGH);
  delay(200);
}

// Seleciona o canal do TCA9548A
void tcaSelect(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

// Função que retorna true se a cor detectada for verde
String detectarCor(uint8_t canal) {
  tcaSelect(canal);
  delay(10); // espera o sensor estar pronto

  Adafruit_TCS34725* tcs;

  if (canal == 0) {
    if (tcs0 == nullptr)
      tcs0 = new Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_60X);
    tcs = tcs0;
  } else if (canal == 1) {
    if (tcs1 == nullptr)
      tcs1 = new Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_60X);
    tcs = tcs1;
  } else {
    return "erro";
  }

  if (!tcs->begin()) {
    Serial.print("Sensor no canal ");
    Serial.print(canal);
    Serial.println(" não encontrado.");
    return "erro";
  }

  uint16_t r, g, b, c;
  tcs->getRawData(&r, &g, &b, &c);

  Serial.print("Canal ");
  Serial.print(canal);
  Serial.print(" - R: "); Serial.print(r);
  Serial.print(" G: "); Serial.print(g);
  Serial.print(" B: "); Serial.print(b);
  Serial.print(" C: "); Serial.println(c);

  // PRETO: todos os canais baixos
  if (r < 3000 && g < 4400 && b < 3400) {
    return "preto";
  }

  // VERDE: verde é significativamente maior que vermelho e azul
  if (r > 6000 && g > 7500 && b > 6500) {
    return "branco";
  }

  // Caso contrário
  return "colorido";
}


// Função para calcular a posição da linha
float calcularPosicaoLinha() {
  // Não há nenhuma maneira de centesq e centdir juntos não ser bifurcação, e acho que é isso que tá causando ele n parar lá, além disso, colocar ele antes de tudo economiza tempo
  // 
  int totalAtivos = extEsq + centEsq + centDir + extDir;

  if (totalAtivos == 4 || extEsq == 1 && extDir == 1) {
    pararMotores();
    return 69;
  }

  if (totalAtivos == 0) return 0; // Nenhum sensor ativ  

  // Atribui pesos aos sensores (esquerda: negativo, direita: positivo)
  float somaIndices = (extEsq * -2) + (centEsq * -1) + (centDir * 1) + (extDir * 2);

  return somaIndices / float(totalAtivos);
}

void vencerResistenciaInicial() {
  // Impulso inicial com velocidade máxima
  motorFrenteEsquerdo.setSpeed(velocidadeResistencia);
  motorFrenteDireito.setSpeed(velocidadeResistencia);
  motorTrasEsquerdo.setSpeed(velocidadeResistencia);
  motorTrasDireito.setSpeed(velocidadeResistencia);
  
  motorFrenteEsquerdo.run(FORWARD);
  motorFrenteDireito.run(FORWARD);
  motorTrasEsquerdo.run(FORWARD);
  motorTrasDireito.run(FORWARD);
  
  delay(100);  // Duração do impulso
}

void lerSensores() {
  int valorExtEsq = analogRead(sensorExtEsquerdo);
  int valorCentEsq = analogRead(sensorCentEsquerdo);
  int valorCentDir = analogRead(sensorCentDireito);
  int valorExtDir = analogRead(sensorExtDireito);

  extEsq  = valorExtEsq  > limiarExtEsq  ? 1 : 0;
  centEsq = valorCentEsq > limiarCentEsq ? 1 : 0;
  centDir = valorCentDir > limiarCentDir ? 1 : 0;
  extDir  = valorExtDir  > limiarExtDir  ? 1 : 0;

  Serial.print("extEsq:");    Serial.print(valorExtEsq);
  Serial.print(" centEsq:");  Serial.print(valorCentEsq); 
  Serial.print(" centDir:");  Serial.print(valorCentDir); 
  Serial.print(" extDir:");   Serial.println(valorExtDir);
}



void setup() {
  Serial.begin(9600);
  Serial.println("Iniciando seguidor de linha...");
  Wire.begin();

  mpu.begin();
  mpu.setAccelSensitivity(0);  //  2g
  mpu.setGyroSensitivity(0);   //  250 degrees/s
  mpu.setThrottle(false);
  mpu.calibrate(1500); // Calibra giroscópio parado

  pinMode(LEDA, OUTPUT);
  pinMode(LEDB, OUTPUT);
  desligarLEDs();


  // Inicializa os motores
  motorFrenteEsquerdo.setSpeed(velocidadeNormal);
  motorFrenteDireito.setSpeed(velocidadeNormal);
  motorTrasEsquerdo.setSpeed(velocidadeNormal);
  motorTrasDireito.setSpeed(velocidadeNormal);
  
  // Para todos os motores inicialmente
  pararMotores();
  
  // Configura os pinos dos sensores como entrada
  pinMode(sensorExtEsquerdo, INPUT);
  pinMode(sensorCentEsquerdo, INPUT);
  pinMode(sensorCentDireito, INPUT);
  pinMode(sensorExtDireito, INPUT);

  
  String corA = detectarCor(0);
  String corB = detectarCor(1);
  
  // Pequeno delay para estabilização   
  delay(1000);

  vencerResistenciaInicial();
}

void loop() {
  lerSensores();

  float media = calcularPosicaoLinha();
  
  Serial.print("Posição média: ");
  Serial.println(media);

  reverificar:


  if (media == 69) {
/*    pararMotores();
    delay(50);
    andarTras();
    delay(750);


    pararMotores();
    delay(50);
    */
    ligarLEDs();

    String corA = detectarCor(0);
    String corB = detectarCor(1);

    desligarLEDs();

    vencerResistenciaInicial();

    
    if (corA == "preto" && corB == "preto") {
      Serial.println("preto A && preto B");
      andarTras();  // ou qualquer ação especial
      delay(500);
    }
    else if (corA == "preto" && corB != "preto") {
      virarForte(DIREITA);
      delay(50);
    
      goto reverificar;
    }
    else if (corA != "preto" && corB == "preto") {
      virarForte(ESQUERDA);
      delay(50);
      goto reverificar;
    }
    else if (corA == "colorido" && corB != "colorido") {
      andarReto();
      delay(tempopre90);
      Serial.println("colorido A && não colorido B");
      virarComGiro(90, ESQUERDA);
    }
    else if (corA != "colorido" && corB == "colorido") {
      andarReto();
      delay(tempopre90);
      Serial.println("não colorido A && colorido B");
      virarComGiro(90, DIREITA);
    }
    else if (corA == "colorido" && corB == "colorido") {
      Serial.println("colorido A && colorido B");
      virarComGiro(90, DIREITA);
      virarComGiro(90, DIREITA);
    }
    else {
      Serial.println("nenhum colorido e nenhum preto ao mesmo tempo");
      andarReto();
      delay(1350);
    }

    return;
  }

  int distance = sonar.ping_cm();
  Serial.print("Distância: ");
  Serial.println(distance);

  if (distance < 10 && distance != 0 ) {
    Serial.println("Distância < 25: iniciando manobra de busca por linha.");

    pararMotores();
    delay(100);
    virarComGiro(90, ESQUERDA);
    pararMotores();
    delay(100);


    andarReto();
    delay(tempoOrbita / 2);

    // Vira 90 graus à esquerda
    unsigned long ultimaVirada = millis();

    while (true) {
      lerSensores();

      if (extEsq || centEsq || centDir || extDir) {
        Serial.println("Linha encontrada!");
        pararMotores();
        virarComGiro(90, ESQUERDA);
        return;
      }

      andarReto();

      // Verifica se já se passaram 2000 ms desde a última virada
      if (millis() - ultimaVirada >= 2000) {
        pararMotores();
        delay(100);
        Serial.println("Virando 90 graus para buscar linha...");
        virarComGiro(90, DIREITA);
        pararMotores();
        delay(100);
        ultimaVirada = millis();
      }

      delay(10);  // Pequeno delay para não travar o loop
    }
  } 

  
  if (extEsq && centEsq) {
    andarReto();
    delay(tempopre90);
    Serial.println("Dois sensores da ESQUERDA ativados: giro 90° esquerda");
    virarComGiro(90, ESQUERDA);
    return;
  }
  else if (extDir && centDir) {
    andarReto();
    delay(tempopre90);
    Serial.println("Dois sensores da DIREITA ativados: giro 90° direita");
    virarComGiro(90, DIREITA);
    return;
  }
  if (media < -1.5) {
    virarForte(ESQUERDA);
    Serial.println("Virar forte para esquerda");
  } 
  else if (media > 1.5) {
    virarForte(DIREITA);
    Serial.println("Virar forte para direita");
  } 
  else if (media < -0.7) {
    virar(ESQUERDA);
    Serial.println("Virar para esquerda");
  } 
  else if (media > 0.7) {
    virar(DIREITA);
    Serial.println("Virar para direita");
  } 
  else {
    andarReto();
    Serial.println("Seguir reto");
  }
  delay(50);


}


void andarReto() {
  motorFrenteEsquerdo.setSpeed(velocidadeNormal);
  motorFrenteDireito.setSpeed(velocidadeNormal);
  motorTrasEsquerdo.setSpeed(velocidadeNormal);
  motorTrasDireito.setSpeed(velocidadeNormal);
  
  // Usa todos os motores para andar reto
  motorTrasEsquerdo.run(FORWARD);
  motorTrasDireito.run(FORWARD);
  motorFrenteEsquerdo.run(FORWARD);
  motorFrenteDireito.run(FORWARD);
}

void andarTras(){
  motorFrenteEsquerdo.setSpeed(velocidadeNormal + 10);
  motorFrenteDireito.setSpeed(velocidadeNormal + 10);
  motorTrasEsquerdo.setSpeed(velocidadeNormal + 10);
  motorTrasDireito.setSpeed(velocidadeNormal + 10);
  
  // Usa todos os motores para andar reto
  motorTrasEsquerdo.run(BACKWARD);
  motorTrasDireito.run(BACKWARD);
  motorFrenteEsquerdo.run(BACKWARD);
  motorFrenteDireito.run(BACKWARD);
}


void virar(int direcao) {
  if (direcao == DIREITA) {
    // Virar para direita (motores esquerdos para frente, direitos para trás)
    motorTrasEsquerdo.run(FORWARD);
    motorFrenteEsquerdo.run(FORWARD);
    motorTrasDireito.run(BACKWARD);
    motorFrenteDireito.run(BACKWARD);
    
    // Ajusta velocidades
    motorTrasEsquerdo.setSpeed(velocidadeCurva);
    motorFrenteEsquerdo.setSpeed(velocidadeCurva);
    motorTrasDireito.setSpeed(velocidadeCurva);
    motorFrenteDireito.setSpeed(velocidadeCurva);
  } 
  else {
    // Virar para esquerda (motores direitos para frente, esquerdos para trás)
    motorTrasDireito.run(FORWARD);
    motorFrenteDireito.run(FORWARD);
    motorTrasEsquerdo.run(BACKWARD);
    motorFrenteEsquerdo.run(BACKWARD);
    
    // Ajusta velocidades
    motorTrasDireito.setSpeed(velocidadeCurva);
    motorFrenteDireito.setSpeed(velocidadeCurva);
    motorTrasEsquerdo.setSpeed(velocidadeCurva);
    motorFrenteEsquerdo.setSpeed(velocidadeCurva);
  }
}

void virarForte(int direcao) {
  if (direcao == DIREITA) {
    // Virar fortemente para direita
    motorTrasEsquerdo.setSpeed(velocidadeCurvaExtrema);
    motorFrenteEsquerdo.setSpeed(velocidadeCurvaExtrema);
    motorTrasDireito.setSpeed(velocidadeCurvaExtrema);
    motorFrenteDireito.setSpeed(velocidadeCurvaExtrema);

    motorTrasEsquerdo.run(FORWARD);
    motorFrenteEsquerdo.run(FORWARD);
    motorTrasDireito.run(BACKWARD);
    motorFrenteDireito.run(BACKWARD);
  } 
  else {
    // Virar fortemente para esquerda
    motorTrasDireito.setSpeed(velocidadeCurvaExtrema);
    motorFrenteDireito.setSpeed(velocidadeCurvaExtrema);
    motorTrasEsquerdo.setSpeed(velocidadeCurvaExtrema);
    motorFrenteEsquerdo.setSpeed(velocidadeCurvaExtrema);

    motorTrasDireito.run(FORWARD);
    motorFrenteDireito.run(FORWARD);
    motorTrasEsquerdo.run(BACKWARD);
    motorFrenteEsquerdo.run(BACKWARD);
  }
}

void virarComGiro(float anguloAlvo, int direcao) {
  pararMotores();
  mpu.read();
  int anguloAlvoRobo = mpu.getYaw() + anguloAlvo;

  while (abs(mpu.getYaw()) < abs(anguloAlvoRobo)) {
    mpu.read();
    if (direcao == DIREITA) {
      // Girar para a direita
      virar(DIREITA);
    } else {
      virar(ESQUERDA);
    }
    // println para pular linha após o último dado


    Serial.print("Yaw: ");
    Serial.print(mpu.getYaw());
    Serial.println(" °");
    delay(10);  // Pequeno atraso para estabilidade
  }
  pararMotores();

}



void pararMotores() {
  motorTrasEsquerdo.run(RELEASE);
  motorFrenteEsquerdo.run(RELEASE);
  motorTrasDireito.run(RELEASE);
  motorFrenteDireito.run(RELEASE);
}