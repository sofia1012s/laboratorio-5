//*****************************************************************************
// Universidad del Valle de Guatemala
// BE3015: Electrónica Digital 2
//*****************************************************************************

//*****************************************************************************
//Librerias
//*****************************************************************************
#include <Arduino.h>
#include <LiquidCrystal.h> //libreria para LCD

//*****************************************************************************
//Definicion etiquetas
//*****************************************************************************
//Botones de contador
#define Boton1 5
#define Boton2 21

//Potenciometros
#define Potenciometro1 15
#define Potenciometro2 4

//LCD
#define d4 14
#define d5 27
#define d6 26
#define d7 25
#define en 12 //Sincronización
#define rs 13 //Selector

//*****************************************************************************
//Prototipos de funcion
//*****************************************************************************
void configurarBoton1(void);
void configurarBoton2(void);
void emaADC(void);
void valores(void);

//*****************************************************************************
//Varibles globales
//*****************************************************************************
int contadorBoton = 0; //contador de botón

//Variables para filtro Medio Móvil Exponencial
float adcRaw1 = 0.0;     //Valor Crudo potenciómetro 2
float adcRaw2 = 0.0;     //Valor Crudo potenciómetro 2
double potFiltrado1 = 0; //S(0) = Y(0)
double potFiltrado2 = 0; // S(0) = Y(0)
double alpha = 0.09;     // Factor de suavizado
float voltage = 0.0;     //Valor de voltaje filtrado

//LCD
LiquidCrystal LCD(rs, en, d4, d5, d6, d7);
uint8_t decenas1, unidades1, decimal1; //Primer potenciometro
uint8_t decenas2, unidades2, decimal2; //Segundo potenciometro
uint8_t centena, decena, unidad;       //Cotnador

//*****************************************************************************
//ISR: interrupciones
//*****************************************************************************
void IRAM_ATTR ISRBoton1() //interrupción para botón 1 (Aumento)
{
  static unsigned long ultimo_tiempo_interrupcion1 = 0; //último tiempo de la interrupción
  unsigned long tiempo_interrupcion1 = millis();        //tiempo actual de la interrupción

  //Si la interrupcion dura menos de 200ms, asumir que es un rebote e ignorar
  if (tiempo_interrupcion1 - ultimo_tiempo_interrupcion1 > 200)
  {
    contadorBoton++; //aumenta 1 al contador de botón

    if (contadorBoton > 255) //si es mayor a 15 regresa el valor a cero
    {
      contadorBoton = 0;
    }
  }
  ultimo_tiempo_interrupcion1 = tiempo_interrupcion1; //actualiza el valor del tiempo de la interrupción
}

void IRAM_ATTR ISRBoton2() //interrupción para botón 2 (Disminución)
{
  static unsigned long ultimo_tiempo_interrupcion2 = 0; //último tiempo de la interrupción
  unsigned long tiempo_interrupcion2 = millis();        //tiempo actual de la interrupción

  //si la interrupcion dura menos de 200ms, asumir que es un rebote e ignorar
  if (tiempo_interrupcion2 - ultimo_tiempo_interrupcion2 > 200)
  {
    contadorBoton--; //disminuye el contador en 1

    if (contadorBoton < 0) //si es menor a cero, regresa el valor a 4 bits
    {
      contadorBoton = 255;
    }
  }
  ultimo_tiempo_interrupcion2 = tiempo_interrupcion2; //actualiza el valor del tiempo de la interrupción
}
//*****************************************************************************
//configuracion
//*****************************************************************************
void setup()
{
  Serial.begin(115200);

  //Botones
  pinMode(Boton1, INPUT_PULLUP);
  pinMode(Boton2, INPUT_PULLUP);

  //Funciones para configurar los botones
  configurarBoton1();
  configurarBoton2();

  LCD.begin(16, 2);
}

//*****************************************************************************
//loop principal
//*****************************************************************************
void loop()
{
  emaADC();
  valores();
  Serial.print(potFiltrado1);
  Serial.println(potFiltrado2);
  LCD.setCursor(0, 0);
  LCD.print("Pot1: ");
  LCD.setCursor(0, 1);
  LCD.print(decenas1);
  LCD.print('.');
  LCD.print(unidades1);
  LCD.print(decimal1);
  LCD.print("V");

  LCD.setCursor(6, 0);
  LCD.print("Pot2: ");
  LCD.setCursor(6, 1);
  LCD.print(decenas2);
  LCD.print('.');
  LCD.print(unidades2);
  LCD.print(decimal2);
  LCD.print("V");

  LCD.setCursor(12, 0);
  LCD.print("CPU: ");
  LCD.setCursor(12, 1);
  LCD.print(centena);
  LCD.print(decena);
  LCD.print(unidad);
}

//***************************************************
//Función para configurar Botón 1
//***************************************************
void configurarBoton1(void)
{
  //coloca una interrupción en el botón 1 (durante el cambio de alto a bajo)
  attachInterrupt(digitalPinToInterrupt(Boton1), ISRBoton1, RISING);
}

//***************************************************
//Función para configurar Botón 2
//***************************************************
void configurarBoton2(void)
{
  //coloca una interrupción en el botón 2 (durante el cambio de alto a bajo)
  attachInterrupt(digitalPinToInterrupt(Boton2), ISRBoton2, RISING);
}

//****************************************************************
// Filtro media Móvil exponencial EMA
//****************************************************************
void emaADC(void)
{
  adcRaw1 = analogReadMilliVolts(Potenciometro1) / 10; //toma valor que está midiendo el potenciometro
  adcRaw2 = analogReadMilliVolts(Potenciometro2) / 10; //toma valor que está midiendo el potenciometro

  potFiltrado1 = (alpha * adcRaw1) + ((1.0 - alpha) * potFiltrado1); //filtra ese valor

  potFiltrado2 = (alpha * adcRaw2) + ((1.0 - alpha) * potFiltrado2); //filtra ese valor
}

//****************************************************************
// Dividir valor tomado
//****************************************************************
void valores(void)
{
  //Primer potenciómetro
  int temp1 = potFiltrado1;
  decenas1 = temp1 / 100.0;
  temp1 = temp1 - decenas1 * 100.0;
  unidades1 = temp1 / 10.0;
  temp1 = temp1 - unidades1 * 10.0;
  decimal1 = temp1;

  //Segundo potenciómetro
  int temp2 = potFiltrado2;
  decenas2 = temp2 / 100.0;
  temp2 = temp2 - decenas2 * 100.0;
  unidades2 = temp2 / 10.0;
  temp2 = temp2 - unidades2 * 10.0;
  decimal2 = temp2;

  //Contador
  int temp3 = contadorBoton;
  centena = temp3 / 100.0;
  temp3 = temp3 - centena * 100.0;
  decena = temp3 / 10.0;
  temp3 = temp3 - decena * 10.0;
  unidad = temp3;
}