#include <Arduino.h>
#include <stdio.h>
#include <stdlib.h>
#include <LiquidCrystal.h>

#define BIT0 0b00000001
#define BIT1 0b00000010
#define BIT2 0b00000100
#define BIT3 0b00001000
#define BIT4 0b00010000
#define BIT5 0b00100000
#define BIT6 0b01000000
#define BIT7 0b10000000

// CONFIGURA DISPLAY
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

// ===================================== FUNÇÃO PARA MOSTRAR TEMPERATURA E ===================================

void display_data(int temp, int umidade)
{

  if (temp < 20)
  {
    lcd.setCursor(1, 0);
    lcd.print("TEMP. BAIXA     ");
  }
  else if (temp <= 30)
  {
    lcd.setCursor(1, 0);
    lcd.print("TEMP. NORMAL     ");
  }
  else if (temp <= 40)
  {
    lcd.setCursor(1, 0);
    lcd.print("TEMP. ALTA     ");
  }
  else
  {
    lcd.setCursor(1, 0);
    lcd.print("TEMP. EXTREMA     ");
  }
  lcd.setCursor(1, 1);
  if(temp<10)
    lcd.print(0);
  lcd.print(temp);
  lcd.print("     ");
  lcd.setCursor(4, 1);
  lcd.print(".C");

  lcd.setCursor(10, 1);
  lcd.print("* ");
  if(umidade<10)
    lcd.print(0);
  lcd.print(umidade);
  lcd.print("%  ");
  _delay_ms(200);
}

// Configuracoes de Serial
#define FOSC 16000000U
#define BAUD 9600
#define MYUBRR FOSC / 16 / (BAUD - 1)

// configuracoes do transmissor
char msg_tx[50];
int pos_msg_rx = 0;

// flag universal ON OFF
int liga = 0;


// ===================================== FUNÇÃO PARA DESLIGAMENTO GERAL DA ESTUFA ===================================

void desliga_geral()
{
  //MENSAGEM DE DESLIGAMENTO DO SISTEMA
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Desligando");
  lcd.setCursor(0,1);
  lcd.print("sistema");
  _delay_ms(1000);
  lcd.clear(); // Limpia la pantalla
  lcd.blink(); // ativa blink do display
  _delay_ms(2000);
  lcd.noBlink(); // desativa blink do display
  _delay_ms(1500);
  lcd.clear();

  // motores
  PORTD &= ~BIT6;
  PORTD &= ~BIT7;
  // led
  PORTB &= ~BIT2;
  // buzer
  PORTB &= ~BIT5;
  //lcd
  PORTC &= ~BIT5;
  //LAMPADA EXTERNA
  PORTC &= ~BIT4;
  UART_Transmit("Sistema Desligado!\n");
  liga = 0;
}


// ===================================== INTERRUPÇÃO E TIMER PARA DESLIGAR SISTEMA APÓS 3SEG ===================================
int sec = 0;
ISR(TIMER1_COMPA_vect)
{
  if ((PINB & BIT1) == 0)
  {
    sec++;
    if (sec == 3)
    {
      desliga_geral();
      sec = 0;
    }
  }
  else
  {
    sec = 0;
  }
}

// ===================================== CONTROLES DE ILUMIAÇÃO, VENTILAÇÃO, IRRIGAÇÃO E ALARME ===================================
void controles(int temperatura, int umidade)
{
  // MOTOR DE IRRIGACAO = PD7
  // MOTOR DE VENTILACAO = PD6 (OCR0A)
  // motor do ventilador configurado com rotação de acordo com o nivel da temperatura

  //LIGA ILUMINAÇÃO
  if (temperatura <= 20){
    OCR0A = 0;
    PORTB |= BIT2;
  }
  else
  {
    // DESLIGA LUZ

    if(temperatura >= 40){

      // configura pwm no ventilador
      OCR0A = (int)((100 / 100) * 255); //(DC em Porcentagem)
    }
    else if(temperatura >= 35){

      // configura pwm no ventilador
      OCR0A = (int)((75 / 100) * 255); //(DC em Porcentagem)
    }
    else if(temperatura >= 30){

      // configura pwm no ventilador
      OCR0A = (int)((50 / 100) * 255); //(DC em Porcentagem)
    }
    else if(temperatura >= 25){

      // configura pwm no ventilador
      OCR0A = (int)((25 / 100) * 255); //(DC em Porcentagem)
    }
    else{
      
      // configura pwm no ventilador
      OCR0A = (int)((0 / 100) * 255); //(DC em Porcentagem)
    }

    PORTB &= ~BIT2;
  }

  // DEFINIÇÃO DO BUZZER - liga se temperatura passar de 40 graus
  if (temperatura > 40)

    PORTB |= BIT5;
  else
    PORTB &= ~BIT5;

  // LIGA E DESLIGA IRRIGADOR - liga se umidade estiver menor que 50%
  if (umidade < 50)
    PORTD |= BIT7;
  else
    PORTD &= ~BIT7;
}

// ===================================== CONVERSORES ADC PARA TEMPERATURA E UMIDADE ===================================
int obterTemperatura()
{
  unsigned int leitura_AD;
  int temperatura;
  float tensao;
  int pino = 0;

  ADMUX = ((ADMUX & 0xF8) | pino);

  // Leitura do Conversor AD com média de conversão
  unsigned int SomaLeitura = 0, MediaLeitura;
  for (int i = 0; i < 5; i++)
  {
    ADCSRA |= BIT6;
    while ((ADCSRA & BIT6) == BIT6)
      ;

    leitura_AD = (ADCL | (ADCH << 8)); // Ou leitura_AD = ADC;
    SomaLeitura += leitura_AD;
  }
  MediaLeitura = SomaLeitura / 5;
  tensao = (leitura_AD * 5.0) / 1023.0; // Variável auxiliar para o cálculo
  temperatura = tensao * 10; //temperatura de 0 a 50 graus
  
  // UART UTILIZADA PARA ENVIAR LOGS DA TEMPERATURA 
  itoa(temperatura, msg_tx, 10);
  UART_Transmit("Leitura Temperatura: ");
  UART_Transmit(msg_tx);
  UART_Transmit("\n");
  return temperatura;
}

int obterUmidade()
{
  unsigned int leitura_AD;
  int umidade;
  float tensao;
  int pino = 1;

  ADMUX = ((ADMUX & 0xF8) | pino);

  unsigned int SomaLeitura = 0, MediaLeitura;
  for (int i = 0; i < 10; i++)
  {
    ADCSRA |= BIT6;
    while ((ADCSRA & BIT6) == BIT6)
      ;

    leitura_AD = (ADCL | (ADCH << 8)); // Ou leitura_AD = ADC;
    SomaLeitura += leitura_AD;
  }
  MediaLeitura = SomaLeitura / 10;
  tensao = (MediaLeitura * 5) / 1023.0; // Variável auxiliar para o cálculo
  umidade = tensao * 20; //umidade de 0 a 100%
  
  // UART UTILIZADA PARA ENVIAR LOGS DA UMIDADE 
  itoa(umidade, msg_tx, 10);
  UART_Transmit("Leitura Umidade: ");
  UART_Transmit(msg_tx);
  UART_Transmit("\n");
  return umidade;
}

// ===================================== FUNÇÃO PRINCIPAL ===================================

int main()
{

  // Inicialização da Serial
  UART_config(MYUBRR);

  // Determinação do pino de leitura do conversor AD
  ADMUX = BIT6;
  // Configuração do Conversor AD
  ADCSRA = (BIT7 + BIT2 + BIT1 + BIT0); // Habilitação do Conversor e Prescaler de 128
  ADCSRB = 0;                           // Garantia de conversão única
  DIDR0;                                // Desabilitação dos PINOS usados no AD para entrada digital - Não obrigatório

  // saidas
  DDRD = BIT6 + BIT7;
  DDRB = BIT2 + BIT5;
  DDRC = BIT4 + BIT5;

  // habilitando pull-up para botões
  PORTB = BIT0 + BIT1;

  // configurando PWM
  TCCR0A = 0b10000011; // Temporizador no modo PWM não-invertido
  TCCR0B = 0b00000100; //- Prescaler de 256 - Frequencia de 62, 5 kHz - Período de 16 us

  // set timer1 interrupt at 1Hz
  TCCR1A = 0; // set entire TCCR1A register to 0
  TCCR1B = 0; // same for TCCR1B
  TCNT1 = 0;  // initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 15624; // = (16*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  // Interrupção global
  sei();

  //mensagem inicial via UART
  UART_Transmit("Sistema Desligado!\n");
  UART_Transmit("                    ");

  for (;;)
  {
    // A0 = Temperatura
    // A1 = Umidade
    if ((PINB & BIT0) == 0)
    {
      liga = 1;
      //LIGA  DISPLAY
      PORTC |= BIT5;
      //LIGA LAMPADA EXTERNA
      PORTC |= BIT4;
      // inicializando lcd
      lcd.begin(16, 2);
      //MENSAGEM DE INICIALIZAÇÃO DO SISTEMA
      lcd.print("Inicializando");
      lcd.setCursor(0,1);
      lcd.print("sistema");
      _delay_ms(1000);
      lcd.clear(); // Limpia la pantalla
      lcd.blink(); // ativa blink do display
      _delay_ms(2000);
      lcd.noBlink(); // desativa blink do display
      lcd.setCursor(0,0);
      lcd.print("Sistema");
      lcd.setCursor(0,1);
      lcd.print("Iniciado!");
      _delay_ms(1500);
      lcd.clear();

      UART_Transmit("Sistema Ligado!\n");
    }

    if (liga == 1)
    {


      // verifica temperatura e umidade para controles
      int temperatura = obterTemperatura(); // 0-9
      int umidade = obterUmidade();
      display_data(temperatura, umidade);
      controles(temperatura, umidade);
      // log da temperatura via serial
    }
  }
}


// ===================================== CONFIGURAÇÕES UART ===================================

// Transmissão de Dados Serial
void UART_Transmit(char *dados)
{
  // Envia todos os caracteres do buffer dados ate chegar um final de linha
  while (*dados != 0)
  {
    while ((UCSR0A & BIT5) == 0)
      ; // Aguarda a transmissão acabar

    // Escreve o caractere no registro de tranmissão
    UDR0 = *dados;
    // Passa para o próximo caractere do buffer dados
    dados++;
  }
}

// Configuração da Serial
void UART_config(unsigned int ubrr)
{
  // Configura a  baud rate
  UBRR0H = (unsigned char)(ubrr >> 8);
  UBRR0L = (unsigned char)ubrr;
  // Habilita a recepcao, tranmissao e interrupcao na recepcao */
  UCSR0B = (BIT7 + BIT4 + BIT3);
  // Configura o formato da mensagem: 8 bits de dados e 1 bits de stop */
  UCSR0C = (BIT2 + BIT1);
}
