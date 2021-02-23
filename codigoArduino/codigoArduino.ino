#include <SoftwareSerial.h>
// Autores: 
//      - Sofía Florencia Cibello  - Legajo: 79906
//      - Lara Estefanía Parrucci - Legajo: 77749
// Curso: 3K1
// Proyecto Arduino -> Materia ‘Comunicaciones’


SoftwareSerial moduloWifi(2, 4); // RX | TX
const int EchoPin = 10;
const int TriggerPin = 11;
int movimientoPin = 7;
int ledPin = 13;
int motorDelanteroPin = 12;
int motorTraseroPin = 5;

int segundosRecorrido = 15;         // Cuantos segundos se va a estar efectuando el recorrido
int segundosMonitoreo = 15;         // Cuantos segundos se va a estar efectuando el monitoreo

int contadorSegundos = 0;
int estadoMotor = 0;                // 0 = apagado ; 1 = encendido
int estadoMotorDelantero = 0;       // 0 = apagado ; 1 = encendido
int estadoRobot;                    // 0 = recorriendo ; 1 = monitorizando
int prendido = 0;                   // 0 = apagado ; 1 = prendido
int estadoLed = 0;

void setup() {
  Serial.begin(9600); // Velocidad estándar del puerto serial del Arduino, expresada en Baudios
  moduloWifi.begin(115200); // Velocidad estándar del puerto serial del módulo de wifi, expresada en Baudios
  moduloWifi.setTimeout(10000);

  // Reiniciamos, nos conectamos al WiFi y al Servidor
  resetear();
  delay(3000);
  conectarWifi();
  delay(3000);
  conectarServidor();

  Serial.println(">>> Las conexiones se efectuaron con éxito");

  pinMode (movimientoPin, INPUT);
  pinMode(TriggerPin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(motorDelanteroPin, OUTPUT);
  pinMode(motorTraseroPin, OUTPUT);
  pinMode(EchoPin, INPUT);

  // Establecemos el timeout en 1000 para que la espera por nuevos mensajes sea de 1 segundos, utilizando esta misma espera como delay
  moduloWifi.setTimeout(1000);
}


void loop() {
  if (prendido == 1)
  {
    if (contadorSegundos < segundosRecorrido)
    {
      if (estadoRobot != 0)
      {
        estadoRobot = 0;
        Serial.println("Iniciando recorrido");
      }

      if (estadoMotor == 0)
      {
        estadoMotor = 1;
        digitalWrite(motorTraseroPin, HIGH);
        Serial.println("Motor encendido");
      }
      if (medirDistancia(TriggerPin, EchoPin) < 20)
      {
        if (estadoMotorDelantero == 0)
        {
          estadoMotorDelantero = 1;
          digitalWrite(motorDelanteroPin, HIGH);
          Serial.println("Girando");
        }
      }
      else
      {
        if (estadoMotorDelantero == 1)
        {
          estadoMotorDelantero = 0;
          digitalWrite(motorDelanteroPin, LOW);
          Serial.println("Se dejo de girar");
        }
      }
    }
    else
    {
      // Si se cumple la siguiente condición, entonces hay que monitorear
      if (contadorSegundos < (segundosRecorrido + segundosMonitoreo))
      {
        if (estadoMotor == 1)
        {
          estadoMotor = 0;
          estadoMotorDelantero = 0;
          digitalWrite(motorTraseroPin, LOW);
          digitalWrite(motorDelanteroPin, LOW);
          Serial.println("Motores apagados");
        }
        if (estadoRobot != 1)
        {
          estadoRobot = 1;
          Serial.println("Recorrido detenido, monitorizando");
        }

        // Checkeamos el estado del sensor detector de humo
        int lecturaHumo = analogRead(A0); //Leemos la salida analógica del MQ
        Serial.print("Lectura humo: ");
        Serial.println(lecturaHumo);
        // valores superiores a 100 indican presencia de humo o gas en el sensor
        if (lecturaHumo > 100)
        {
          // Si se detecta humo, enviamos un alerta al servidor
          moduloWifi.println("AT+CIPSEND=4");
          delay(500);
          moduloWifi.println("humo");
          titilar();
        }

        // Checkeamos el estado del sensor de movimiento
        if (digitalRead(movimientoPin) == HIGH)
        {
          Serial.println("Detectado movimiento");
          moduloWifi.println("AT+CIPSEND=10");
          delay(500);
          moduloWifi.println("movimiento");
          titilar();
        }
      }
      else
      {
        contadorSegundos = 0;
      }
    }

    if (moduloWifi.find("detener"))
    {
      prendido = 0;
      estadoMotor = 0;
      estadoMotorDelantero = 0;
      digitalWrite(motorTraseroPin, LOW);
      digitalWrite(motorDelanteroPin, LOW);
      Serial.println("Motores apagados");
    }
    if (estadoLed == 0)
    {
      digitalWrite(ledPin, HIGH);
      estadoLed = 1;
    }
    else
    {
      digitalWrite(ledPin, LOW);
      estadoLed = 0;
    }
    contadorSegundos += 1;
  }
  else
  {
    digitalWrite(ledPin, HIGH);
    Serial.println("Dispositivo apagado, esperando instrucciones");
    while (!moduloWifi.find("iniciar"))
    {
    }
    contadorSegundos = 0;
    estadoMotor = 0;
    estadoMotorDelantero = 0;
    estadoRobot = -1;
    prendido = 1;
    Serial.println("Dispositivo iniciado nuevamente, iniciando recorrido");
  }
}


// Esta función se encarga de resetear el módulo de wifi, enviandole la instrucción 'AT+RST'.
// Cuando se reciba un 'OK' de parte del módulo, es porque se ha reseteado correctamente el mismo,
// caso contrario se sigue intentando hasta lograrlo.
void resetear() {
  moduloWifi.println("AT+RST"); // Reinicia el módulo
  while (!moduloWifi.find("OK"))
  {
    Serial.println(">>> Error al resetear el módulo. Intentando nuevamente..");
    moduloWifi.println("AT+RST");
  }
  Serial.println(">>> Reseteado");

}


// Esta función se encarga de establecer la conexión entre el módulo wifi y la red WiFi, mediante el comando 'AT+CWJAP'
// Si el módulo retorna un 'OK', es porque la conexión entre ambos ha sido establecida,
// caso contrario se sigue intentando hasta lograr dicha conexión.
void conectarWifi() {
  // Comando --> AT+CWJAP=”SSID”,”contraseña del WiFi”
  moduloWifi.println("AT+CWJAP=\"Fibertel WiFi997 2.4GHz\",\"panqueques\""); // El módulo se conecta al la red con el nombre SSID indicado y la contraseña suministrada
  while (!moduloWifi.find("OK"))
  {
    Serial.println(">>> Error al conectarse al WiFi. Intentando nuevamente..");
    moduloWifi.println("AT+CWJAP=\"Fibertel WiFi997 2.4GHz\",\"panqueques\"");
  }
  Serial.println(">>> Conexiòn establecida con el WiFi");
}


// Esta función se encarga de establecer la conexión entre el módulo wifi y el servidor, mediante el comando 'AT+CIPSTART'
// Si el módulo retorna un 'OK', es porque la conexión entre ambos ha sido establecida,
// caso contrario se sigue intentando hasta lograr dicha conexión.
void conectarServidor() {
  // Comando --> AT+CIPSTART=\"tipo de protoclo\",\"dirección IP\",número de puerto
  // En dicho comando se especifica el tipo de protocolo (TCP / UDP),
  // la dirección IP (o el dominio si tiene acceso a DNS)
  // y el número de puerto
  moduloWifi.println("AT+CIPSTART=\"TCP\",\"192.168.0.58\",11000");
  while (!moduloWifi.find("OK"))
  {
    Serial.println(">>> Error al conectarse al servidor. Intentando nuevamente..");
    moduloWifi.println("AT+CIPSTART=\"TCP\",\"192.168.0.58\",11000");
  }
  moduloWifi.println("AT+CIPSEND=9");
  delay(1000);
  moduloWifi.println("conectado");
  delay(1000);
  Serial.println(">>> Conexiòn establecida con el servidor");
}


// Función para medir la distancia leída por el sensor de proximidad
int medirDistancia(int TriggerPin, int EchoPin) {
  long duracion, distancia;      // La distancia esta en cm

  digitalWrite(TriggerPin, LOW);  // Lo ponemos en LOW para generar un pulso limpio
  delayMicroseconds(4);
  digitalWrite(TriggerPin, HIGH);  // Se genera el disparo de 10us
  delayMicroseconds(10);
  digitalWrite(TriggerPin, LOW);
  duracion = pulseIn(EchoPin, HIGH);  // Medimos el tiempo entre pulsos (microsegundos)

  distancia = duracion * 10 / 292 / 2;  // Convertimos a distancia a cm
  return distancia;
}

void titilar()
{
  digitalWrite(ledPin, HIGH);
  delay(100);
  digitalWrite(ledPin, LOW);
  delay(50);
  digitalWrite(ledPin, HIGH);
  delay(100);
  digitalWrite(ledPin, LOW);
  delay(50);
  digitalWrite(ledPin, HIGH);
  delay(100);
  digitalWrite(ledPin, LOW);
  delay(50);
  digitalWrite(ledPin, HIGH);
  delay(100);
  digitalWrite(ledPin, LOW);
  delay(50);
  digitalWrite(ledPin, HIGH);
  delay(100);
  digitalWrite(ledPin, LOW);
}
