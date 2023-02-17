
#include "F28x_Project.h"
#include "math.h"

/**
 * main.c
 */

#define pi 3.14159265358979323846
#define NDAC 400                 // Número de amostras do DAC

uint16_t index = 0, indexseno = 0, offset = 0, senotable[NDAC];

uint16_t adc1 = 0;
uint16_t adc2 = 0;

uint16_t plot[400];
uint16_t *adc = &adc1; // Ponteiro do adc

uint32_t count = 0;
int LigaDesliga = 0;

interrupt void isr_timer0(void);
interrupt void isr_adc(void);

void Setup_GPIO(void);
void Setup_DAC(void);
void Setup_ADC(void);
void Setup_ePWM1(void);
void Setup_ePWM10(void);

void Liga_LED(void);
void Desliga_LED(void);

int main(void)
{
    InitSysCtrl(); // Inicializa CPU (desabilita watchdog, define CPU)

    DINT;          // Desabilita interrupção global

    InitPieCtrl(); // Limpa as interrupções e flags de 1 à 12

    IER = 0x000;    // Apaga cada chamada de interrupção
    IFR = 0x000;    // Apaga flags

    InitPieVectTable(); // Cria tabela de interrupções, aqui aparece as mensagens para estouro de cada timer

    // Por default, as rotinas possuem laços infinitos
    // Foi criada a rotina de interrupção nesse código principal

    // Interrupções

    EALLOW;

    PieVectTable.TIMER0_INT = &isr_timer0;  // Nome da rotina para o timer 0, feito para não utilizar o código fonte principal
    PieVectTable.ADCA1_INT = &isr_adc;      // Interrupção do AD

    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;      // Habilita timer 0 (coluna 7)
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;      // Habilita interrupção do adc

    EDIS;

    IER |= M_INT1; // Habilita linha 1

    // Configura timer 0
    InitCpuTimers();                            // Inicia timers da interrupção
    ConfigCpuTimer(&CpuTimer0, 200, 50);        // Informa timer, velocidade e tempo de estouro (configura os registradores)
    CpuTimer0Regs.TCR.all = 0x4000;             // Chama interrupção a cada estouro do timer

    // GPIO
    Setup_GPIO();

    // DAC
    Setup_DAC();

    // ePWM1
    Setup_ePWM1();

    // ePWM10
    Setup_ePWM10();

    // AD
    Setup_ADC();

    // Ao receber o AD, criar um gráfico

    // Criando a tabela da senóide
    for(index = 0; index<400 ;index++)
    {
        // Soma 1 para retirar o offset
        // Multiplicação por 1000 para variar 0 - 1,5 V
        senotable[index] = (uint16_t)((1000.0*(1.0+sin((2*pi)/NDAC*(float)index))));   // Mudanças de variável para float e 1000
    }
    index = 0;

    EINT;    // Habilita interrupção global
    ERTM;

    GpioDataRegs.GPBDAT.bit.GPIO34 = 0;  // Valor inicial para o pino
    GpioDataRegs.GPADAT.bit.GPIO31 = 0;

    while(1)
    {
        for(count = 0; count < 0x00FFFFFF; count++){

        }
        GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;
        //LigaDesliga != 0 ? Desliga_LED():Liga_LED();  // Condição ? Não:Sim;
    }

}

void Setup_GPIO(void)
{
    EALLOW;             // Escreve em qualquer registrador

    // O LED é conectado no pino 34
    // Dois multiplexadores da tabela 8-7
    // As variáveis do multiplexador na tabela com os pinos que eles acionam

    //GPIO34

    GpioCtrlRegs.GPBGMUX1.bit.GPIO34 = 0; // Configuração para GPIO
    GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 0;
    GpioCtrlRegs.GPBPUD.bit.GPIO34 = 1;   // Registrador de pull-up
    GpioCtrlRegs.GPBDIR.bit.GPIO34 = 1;   // Define como saída (apenas para GPIO)

    //GPIO34

    GpioCtrlRegs.GPAGMUX2.bit.GPIO31 = 0; // Configuração para GPIO
    GpioCtrlRegs.GPAMUX2.bit.GPIO31 = 0;
    GpioCtrlRegs.GPAPUD.bit.GPIO31 = 1;   // Registrador de pull-up
    GpioCtrlRegs.GPADIR.bit.GPIO31 = 1;   // Define como saída (apenas para GPIO)

    //ePWM1a (pino 40)

    GpioCtrlRegs.GPAGMUX1.bit.GPIO0 = 0;            // Configuração para GPIO
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO0 = 1;              // Registrador de pull-up
    //GpioCtrlRegs.GPADIR.bit.GPIO31 = 1;           // Define como saída (apenas para GPIO)
    GpioCtrlRegs.GPACSEL1.bit.GPIO0 = GPIO_MUX_CPU1;// Recebe comando da cpu1

    //ePWM1b (pino 39)

    GpioCtrlRegs.GPAGMUX1.bit.GPIO1 = 0;            // Configuração para GPIO
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO1 = 1;              // Registrador de pull-up
    //GpioCtrlRegs.GPADIR.bit.GPIO31 = 1;           // Define como saída (apenas para GPIO)
    GpioCtrlRegs.GPACSEL1.bit.GPIO1 = GPIO_MUX_CPU1;

    EDIS;
}

void Setup_DAC()
{
    EALLOW;

    DacaRegs.DACCTL.all = 0x0001;         // Utiliza 0-3V de range
    DacaRegs.DACVALS.all = 0x0800;        // Registrador auxiliar
    DacaRegs.DACOUTEN.bit.DACOUTEN = 1;   // Habilita a sáida DAC
    DacaRegs.DACLOCK.all = 0x0000;        // Registradores DACVALS e DACLOCK podem ser livres a vontade

    EDIS;
}

/*void Liga_LED()
{
    EALLOW;

    GpioDataRegs.GPBDAT.bit.GPIO34 = 1;
    GpioDataRegs.GPADAT.bit.GPIO31 = 1;

    EDIS;
}

void Desliga_LED()
{
    EALLOW;

    GpioDataRegs.GPBDAT.bit.GPIO34 = 0;
    GpioDataRegs.GPADAT.bit.GPIO31 = 0;

    EDIS;
}*/

void Setup_ePWM1(void)
{
    EALLOW;

    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;               // Desabilita clock do contador

    EPwm1Regs.TBPRD = 50000;                           // Frequência de 1 KHz triangular
    EPwm1Regs.TBPHS.bit.TBPHS = 0;                     // Fase
    EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_DISABLE;    // Desabilita sincronismo entre saídas PWM
    EPwm1Regs.TBCTR = 0;                               // Limpa contador
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;      // Triangular
    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;             // Desabilita fase
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;            // Prescale
    EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    EPwm1Regs.CMPA.bit.CMPA = EPwm1Regs.TBPRD >> 1;     // 50% dc

    EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;         // Shadow para cada cruzamento com o CMPA
    EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO_PRD;
    EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO_PRD;

    // ePWMa
    EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;                // Quando cruzar, vai para nível lógico baixo
    EPwm1Regs.AQCTLA.bit.CAD = AQ_SET;                  // Quando cruzar, vai para alto
    EPwm1Regs.AQCTLA.bit.PRD = AQ_NO_ACTION;            // Não muda com o período
    EPwm1Regs.AQCTLA.bit.ZRO = AQ_NO_ACTION;            // Não muda com o zero

    // ePWMb
    EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;           // Ativa sinal complementar
    EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;      // Habilita dead-band
    EPwm1Regs.DBFED.bit.DBFED = 44;                     // Dead-time
    EPwm1Regs.DBRED.bit.DBRED = 9;

    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;               // Habilita clock do contador

    EDIS;
}

void Setup_ePWM10(void)
{
        EALLOW;
        CpuSysRegs.PCLKCR2.bit.EPWM10 = 1;           //habilitar clock

        CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;

        EPwm10Regs.TBPRD = 5000;                         //periodo (up/down) 10Khz Clock/4/fpwm LAB PWM

        //Largura do pulso 50%
        EPwm10Regs.CMPA.bit.CMPA = EPwm10Regs.TBPRD >> 1;

        EPwm10Regs.TBPHS.bit.TBPHS = 0;                  // Phase is 0
        EPwm10Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;
        EPwm10Regs.TBCTR = 0x0000;                       // Clear counter

        EPwm10Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;  // Count up/down
        EPwm10Regs.TBCTL.bit.PHSEN = TB_DISABLE;         // Disable phase loading

        EPwm10Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;        // Clock ratio to SYSCLKOUT
        EPwm10Regs.TBCTL.bit.CLKDIV = TB_DIV1;

        EPwm10Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;     // Load registers every ZERO
        EPwm10Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO_PRD;
        EPwm10Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;     // Load registers every ZERO
        EPwm10Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO_PRD;

        EPwm10Regs.AQCTLA.bit.PRD = AQ_NO_ACTION;
        EPwm10Regs.AQCTLA.bit.ZRO = AQ_NO_ACTION;
        EPwm10Regs.AQCTLA.bit.CAU = AQ_CLEAR;            // set actions for EPWM10A
        EPwm10Regs.AQCTLA.bit.CAD = AQ_SET;

        //Trigger ADC
        EPwm10Regs.ETSEL.bit.SOCAEN = 1;                 // Enable SOC on A group
        EPwm10Regs.ETSEL.bit.SOCASEL = ET_CTR_PRDZERO;   // Dispara ADC no topo
        EPwm10Regs.ETPS.bit.SOCAPRD = ET_1ST;            // Trigger on every event

        CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
        EDIS;
}

void Setup_ADC()
{
    Uint16 acqps;

            // Configurações mínimas , consultar datasheet pag 105.
            if( ADC_RESOLUTION_12BIT  == AdcaRegs.ADCCTL2.bit.RESOLUTION)
                acqps = 14;
            else
                acqps = 63;
    EALLOW;
    //CpuSysRegs.PCLKCR13.bit.ADC_A =1;   // Habilita o clock do módulo A do ADC.
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; // RECOMENDADO NO DATASHEET , ADCCLK = 50 Mhz

    AdcSetMode(ADC_ADCA,ADC_RESOLUTION_12BIT,ADC_SIGNALMODE_SINGLE);

    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1; // Gera interrupção um ciclo de clock antes do EOC.
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;   // Energiza o ADC A .
    DELAY_US(1000);  // 1ms de delay para ligar o módulo do ADC.

          // SOC and INTERRUPT config
          AdcaRegs.ADCSOC0CTL.bit.CHSEL = 3; // ADCINA3 - PINO 26 (J3).
          AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 0x17; // SOCA Epwm1.
          AdcaRegs.ADCSOC0CTL.bit.ACQPS = acqps; // 64 SYSCLK cycles to charge the capacitor. Recomendado no Datasheet , pag 105.

          AdcaRegs.ADCSOC1CTL.bit.CHSEL = 4; // ADCINA3 - PINO 26 (J3).
          AdcaRegs.ADCSOC1CTL.bit.TRIGSEL =  0x17; // SOCA Epwm1.
          AdcaRegs.ADCSOC1CTL.bit.ACQPS = acqps;

          AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0x01; // EOC DISPARA o  ADCINT1;
          AdcaRegs.ADCINTSEL1N2.bit.INT1E =1;   // Desabilita interrupções do ADC;
          AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 =1; //Make sure the INT1 flag is cleared.
          EDIS;
}

interrupt void isr_timer0(void)
{
    //GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;

    indexseno = (indexseno == 399) ? 0: (indexseno+1);

    DacaRegs.DACVALS.all = senotable[indexseno];

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

interrupt void isr_adc(void)
{
    adc1 = AdcaResultRegs.ADCRESULT0;  // Ler ADC
    adc2 = AdcaResultRegs.ADCRESULT1;

    index = (index == 399) ? 0 : (index+1);

    plot[index] = *adc;

    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}
