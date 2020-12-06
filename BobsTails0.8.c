    /***********************************************************************************************
   /
  /    Programa escrito por Guilherme "Bob" Renoldi para controle de sumo de 500g "Tails"
 /
/***********************************************************************************************/


#include <18F2431.h>

#device adc=10

#fuses HS, NOWDT, NOPROTECT, NOBROWNOUT, PUT, NOLVP

#define POWER_PWM_PERIOD 499

#define TRUE 1

#define pmd1 PIN_C4
#define pmd2 PIN_C5
#define pme1 PIN_C6
#define pme2 PIN_C7

#use rs232(BAUD = 9600, FORCE_SW, XMIT = PIN_C0, RCV = PIN_C1, PARITY = N, BITS = 8, STREAM = BT)

#use delay(clock=20000000)


// variaveis globais usadas 
int16 leitura;
int8 entrada = 0;
int motorUltimaOrdem_D;
int motorUltimaOrdem_E;
int16 velocidadeUsadaDireita;
int16 velocidadeUsadaEsquerda;
const int1 frente = 1;
const int1 atras = 0;
const int1 direita = 1;
const int1 esquerda = 0;

const int linThreshold =  818  // sensor de linha
const int frontThreshold = 307 // senor de distancia

// HEADERS
void lerDados(int8 canal, int8 bit);
void lerFrontal();
void lerLinha();
int8 estrategia();
void moverMotor(int motor, int16 velocidade, int sentido);
void buscaFixa(int sentido);
void charge(int16 velocidade);
void curva(int x, int y);
void pararMotor(int motor);
void inverterMovimento();
void rotina();


// FUNCOES

void lerDados(int8 canal, int8 bit){
    switch (canal){
           case 0: set_adc_channel(0); break;
           case 1: set_adc_channel(1); break;
           case 2: set_adc_channel(2); break;
           case 3: set_adc_channel(3); break;
    }

    delay_us(10);

    leitura = read_adc();
    if (canal == 0 || canal == 1){
        if(leitura < frontThreshold) bit_set(entrada, bit);
        else bit_clear(entrada, bit);
        } 
    if (canal == 2|| canal == 3){
        if(leitura < linThreshold) bit_set(entrada, bit);
        else bit_clear(entrada, bit);
        }
    }

void lerFrontal(){
    lerDados(2, 0); //sensor da direita
    lerDados(3, 1); //sensor da esquerda
    }

void lerLinha(){
    lerDados(0, 2); //sensor da direita
    lerDados(1, 3); //sensor da esquerda
    }

void moverMotor(int1 motor, int16 velocidade, int1 sentido){
    if(motor){
        switch(sentido){
            case frente: output_high(pmd1); 
                         output_low(pmd2); 
                         motorUltimaOrdem_D = frente; 
                         break;

            case atras:  output_low(pmd1); 
                         output_high(pmd2); 
                         motorUltimaOrdem_D = atras; 
                         break;
        }
        set_power_pwm0_duty((int16)((POWER_PWM_PERIOD*4)*velocidade));
        velocidadeUsadaDireita = velocidade;
    }

    else{
        switch(sentido){
            case frente: output_high(pme1); 
                         output_low(pme2);
                         motorUltimaOrdem_E = frente; 
                         break;

            case atras:  output_low(pme1); 
                         output_high(pme2); 
                         motorUltimaOrdem_E = atras; 
                         break;
        }
        set_power_pwm2_duty((int16)((POWER_PWM_PERIOD*4)*velocidade));
        velocidadeUsadaEsquerda = velocidade;
    }
}

void buscaFixa(int1 sentido){
    if(sentido) {                                  //se (sentido == direita == 1):
        moverMotor(direita, 50, atras);           //roda em torno do proprio eixo sentido horario
        moverMotor(esquerda, 50, frente);        //
    }                                           //
                                               //
    else {                                    // caso contrario:
        moverMotor(direita, 50, frente);     // sentido anti-horario
        moverMotor(esquerda, 50, atras);    //
        }
    }

void charge(int16 velocidade){
    moverMotor(direita, velocidade, frente);
    moverMotor(esquerda, velocidade, frente);
    }

void curva(int1 x, int1 y){
    if(y){
        if (x){
            moverMotor(direita, 60, frente);
            moverMotor(esquerda, 80, frente);
            }
        else{
            moverMotor(esquerda, 60, frente);
            moverMotor(direita, 80, frente);
            }
        }

    else{
        if (x){
            moverMotor(direita, 60, atras);
            moverMotor(esquerda, 80, atras);
            }
        else{
            moverMotor(esquerda, 60, atras);
            moverMotor(direita, 80, atras);
            }
        }
    }

void pararMotor(int1 motor){
    if(motor){
        output_low(pmd1);
        output_low(pmd2);
        set_power_pwm0_duty((int16)((POWER_PWM_PERIOD*4)*0));
    }
    else{
        output_low(pme1);
        output_low(pme2);
        set_power_pwm2_duty((int16)((POWER_PWM_PERIOD*4)*0));
    }
}

void inverterMovimento(){
    pararMotor(direita);
    pararMotor(esquerda);
    switch(motorUltimaOrdem_D){
        case frente: moverMotor(direita, velocidadeUsadaDireita, atras); break;
        case atras: moverMotor(direita, velocidadeUsadaDireita, frente); break;
        }

    switch(motorUltimaOrdem_E){
        case frente: moverMotor(esquerda, velocidadeUsadaEsquerda, atras); break;
        case atras: moverMotor(esquerda, velocidadeUsadaEsquerda, frente); break;
        }
    }

void rotina(int1 sentido){
    lerLinha();
    lerFrontal();
    switch(entrada){
        case  0: buscaFixa(sentido); break;      // 0 0 0 0 nao achou nada
        case  1: curva(direita, frente); break;  // 0 0 0 1 sensor frontal direita
        case  2: curva(esquerda, frente); break; // 0 0 1 0 sensor frontal esquerda
        case  3: charge(100); break;             // 0 0 1 1 os dois frontais
        case  4: inverterMovimento(); break;     // 0 1 0 0 encostou na linha pelo lado direito
        case  5: curva(direita, frente); break;  // 0 1 0 1 encostou na linha pela direita mas ve alvo a direita(decisao arriscada eu sei, mas vamos testar)
        case  6: curva(esquerda, frente); break; // 0 1 1 0 encostou na linha pela direita mas ve alvo a esquerda
        case  7: charge(100); break;             // 0 1 1 1 encostou na linha pela direita e ve alvo bem a frente
        case  8: inverterMovimento(); break;     // 1 0 0 0 escostou na linha pela esquerda
        case  9: curva(direita, frente); break;  // 1 0 0 1 escostou na linha pela esquerda mas ve alvo a direita
        case 10: curva(esquerda, frente); break; // 1 0 1 0 escostou na linha pela esquerda e ve alvo a esquerda
        case 11: charge(100); break;             // 1 0 1 1 encostou na linha pela esquerda e ve alvo bem a frente
        case 12: inverterMovimento(); break;     // 1 1 0 0 dois sensores de linha ativados /!\ 
        case 13: curva(direita, frente); break;  // 1 1 0 1 alvo a direita
        case 14: curva(esquerda, frente); break; // 1 1 1 0 alvo a esquerda
        case 15: charge(100); break;              // 1 1 1 1 SE SALVE, CORRA PARA AS COLINAS
        }
    }

// PROGRAMA PRINCIPAL

void main(){
    char BTRead = getc();
    int1 sentido;
    while(BTRead!='a' || BTRead!='b') BTRead = getc();
    
    delay_ms(5000);
    
    set_tris_a(0b11111111);
    set_tris_b(0b00000000);
    set_tris_c(0b01000110);
    
    setup_adc_ports(ALL_ANALOG);
    setup_adc(ADC_CLOCK_INTERNAL);
    
    setup_power_pwm_pins(PWM_BOTH_ON, PWM_BOTH_ON, PWM_OFF, PWM_OFF);
    setup_power_pwm(PWM_FREE_RUN,1,0,POWER_PWM_PERIOD,0,1,33);

    
    switch (BTRead){// estrategias
        case 'a':sentido = direita;
        case 'b':sentido = esquerda;
        }
    
    while (BTHit!='h'){
        rotina(sentido);
        if(kbhit(BT)) BTHit = getc();
    }
}
