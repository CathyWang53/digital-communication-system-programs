//������ʱ�Ӳ����ڲ�RC������     DCO��8MHz,��CPUʱ��;  SMCLK��1MHz,����ʱ��ʱ��
#include <msp430g2553.h>
//#include <tm1638.h>  //��TM1638�йصı���������������ڸ�H�ļ���

//[3,1]�������ڿ��������ƽ��4·GPIO���Ų�������
#define CTL0_L  P1OUT&=~BIT0;
#define CTL0_H  P1OUT|=BIT0;
#define CTL1_H  P1OUT|=BIT1;
#define CTL1_L  P1OUT&=~BIT1;
#define CTL2_H  P1OUT|=BIT2;
#define CTL3_H  P1OUT|=BIT3;
#define CTL2_L  P1OUT&=~BIT2;
#define CTL3_L  P1OUT&=~BIT3;

    //m������� BIT4
    //����ʱ����� BIT5
    //���ڼǺ� BIT6
    //���������� BIT7

//////////////////////////////
//         ��������         //
//////////////////////////////

// 0.1s�����ʱ�����ֵ��5��20ms
#define V_T100ms	5
// 0.5s�����ʱ�����ֵ��25��20ms
#define V_T500ms	25

//[1,1]����ȼ�����
#define GAIN_STATENUM 15

//////////////////////////////
//       ��������           //
//////////////////////////////
//m4���еı���
int reg[4]={1,1,1,1};
int mout = 1;//m������� BIT0
int synClock = 1;//����ʱ����� BIT1
int period = 8;//���ڼǺ� BIT2
int peFlag=0;
int syFlag = 1;
int defOut = 1;//����������
int button_flag;


// �����ʱ������
unsigned char clock100ms=0;
unsigned char clock500ms=0;
// �����ʱ�������־
unsigned char clock100ms_flag=0;
unsigned char clock500ms_flag=0;
// �����ü�����
unsigned int test_counter=0;
// 8λ�������ʾ�����ֻ���ĸ����
// ע����������λ�������������Ϊ4��5��6��7��0��1��2��3
//[1,2]����λ�̶���ʾ�����桱��Ӣ��GAIN����������λ�̶���ʾ����
unsigned char digit[8]={' ','-',' ',' ','G','A','I','N'};
// 8λС���� 1��  0��
// ע����������λС����������������Ϊ4��5��6��7��0��1��2��3
unsigned char pnt=0x04;     //�����ʲô������
// 8��LEDָʾ��״̬��ÿ����4����ɫ״̬��0��1�̣�2�죬3�ȣ���+�̣�
// ע������ָʾ�ƴ������������Ϊ7��6��5��4��3��2��1��0
//     ��ӦԪ��LED8��LED7��LED6��LED5��LED4��LED3��LED2��LED1
//ledȫ��
unsigned char led[]={0,0,0,0,0,0,0,0};
//[1,4]����ȼ�ȡֵ����ֵΪ1����Ӧ����Ϊ0.1
unsigned char gain_state = 1;
// add two global variables concerned about buttons
unsigned char key_state = 0, key_flag = 1;
// ��ǰ����ֵ
unsigned char key_code=0;
//unsigned char key1 = 0;

void m4creat()
{
    int tmpM,tmpDef;

    if (period == 0)
        {
            peFlag = 1;
            period = 7;
        }
    else
        {
            period--;
            peFlag = 0;
        }

    if (syFlag) syFlag = 0;
    else syFlag = 1;

    if (syFlag)
    {
        mout = reg[0];
        tmpM = ( reg[3] + reg[0] ) % 2;
        reg[0] = reg[1];
        reg[1] = reg[2];
        reg[2] = reg[3];
        reg[3] = tmpM;

        tmpDef = defOut;
        defOut = (tmpDef + mout) % 2;
    }

}

void m3creat()
{
    int tmpM,tmpDef;

    if (period == 0)
        {
            peFlag = 1;
            period = 5;
        }
    else
        {
            period--;
            peFlag = 0;
        }

    if (syFlag) syFlag = 0;
    else syFlag = 1;

    if (syFlag)
    {
        mout = reg[0];
        tmpM = ( reg[2] + reg[0] ) % 2;
        reg[0] = reg[1];
        reg[1] = reg[2];
        //reg4[2] = reg4[3];
        reg[2] = tmpM;

        tmpDef = defOut;
        defOut = (tmpDef + mout) % 2;
    }

}



//////////////////////////////
//       ϵͳ��ʼ��         //
//////////////////////////////


//timer1 initialize
void timer1_init(void)
{
    //��ʼ��P2.1 ����Ϊ��ʱ��A1 ��PWM �������
    P2SEL |= BIT1;
    P2DIR |= BIT1;

    // ��ʼ��Timer1������440Hz �ķ����ź�
    TA1CTL = TASSEL_2 + MC_1; // Source: SMCLK=1MHz, PWM mode
    TA1CCTL1 = OUTMOD_7;
    TA1CCR0 = 1000000/440; //�趨����,1000000 Ϊ��ʱ��1 ʱ��Ƶ��,440 Ϊ��ƵƵ��
    TA1CCR1 = TA1CCR0/2; //����ռ�ձȵ���50%

    P1DIR&=~BIT7;   //��1.7������Ϊ���룬���ں����ź�
}

//  I/O�˿ں����ų�ʼ��
void Init_Ports(void)
{
	P2SEL &= ~(BIT7+BIT6);
    P2DIR |= BIT7 + BIT6 + BIT5;
    P2DIR &= ~(BIT3+BIT2+BIT0);
    P2REN |= BIT3+BIT2+BIT0;
    P2OUT |= BIT3+BIT2+BIT0;
    P1DIR &= ~BIT3;
    P1REN |= BIT3;
    P1IES |= BIT3;
    P1IE |=BIT3;
    P1SEL &= ~(BIT4+BIT5+BIT6+BIT7);
    P1DIR |= BIT4 + BIT5 +BIT6+BIT7;
}

//  ��ʱ��TIMER0��ʼ����ѭ����ʱ20ms
void Init_Timer0(void)
{
	TA0CTL = TASSEL_2 + MC_1 ;      // Source: SMCLK=1MHz, UP mode,
	TA0CCR0 = 1000;                // 1MHzʱ��,����20000��Ϊ 20ms         //Ӧ��Ϊ1000����20000��
	TA0CCTL0 = CCIE;                // TA0CCR0 interrupt enabled
}

//  MCU������ʼ����ע���������������
void Init_Devices(void)
{
	WDTCTL = WDTPW + WDTHOLD;     // Stop watchdog timer��ͣ�ÿ��Ź�
	if (CALBC1_8MHZ ==0xFF || CALDCO_8MHZ == 0xFF)
	{
		while(1);            // If calibration constants erased, trap CPU!!
	}

    //����ʱ�ӣ��ڲ�RC������     DCO��8MHz,��CPUʱ��;  SMCLK��1MHz,����ʱ��ʱ��
	BCSCTL1 = CALBC1_8MHZ; 	 // Set range
	DCOCTL = CALDCO_8MHZ;    // Set DCO step + modulation
	BCSCTL3 |= LFXT1S_2;     // LFXT1 = VLO
	IFG1 &= ~OFIFG;          // Clear OSCFault flag
	BCSCTL2 |= DIVS_3;       //  SMCLK = DCO/8

	timer1_init();
    Init_Ports();           //���ú�������ʼ��I/O��
    Init_Timer0();          //���ú�������ʼ����ʱ��0
    _BIS_SR(GIE);           //��ȫ���ж�
    _EINT();                                                                        //����ɶ��
   //all peripherals are now initialized
}


//////////////////////////////
//      �жϷ������        // �����˲������ֵĹ���
//////////////////////////////

// Timer0_A0 interrupt service routine
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer0_A0 (void)
{
    if (button_flag)m4creat();
    else m3creat();
    //m������� BIT4
    //����ʱ����� BIT5
    //���ڼǺ� BIT6 ������ʱ��Ϊ���m���н��� button_flag��Ϊ1��4�ף�
    //���������� BIT7
    if (mout) P1OUT |= BIT4;
    else P1OUT &= ~BIT4;
    if (syFlag) P1OUT |= BIT5;
    else P1OUT &= ~BIT5;
    if (peFlag) P1OUT |= BIT6;
    else P1OUT &= ~BIT6;
    if (defOut) P1OUT |= BIT7;
    else P1OUT &= ~BIT7;
}

#pragma vector = PORT1_VECTOR
__interrupt void port1(void)
{
    unsigned char PushKey;
    unsigned int i;

    PushKey=P1IFG & BIT3;

    for(i=0;i<50000;i++);
    if(!(P1IN&PushKey)==PushKey)
        {
            P1IFG = 0;
            return;
        }
    if(PushKey&BIT3)
        {
            if(button_flag == 1) button_flag=0;
            else button_flag = 1;
        }
    P1IFG = 0;
}

//////////////////////////////
//         ������           //
//////////////////////////////

int main(void)
{
	//unsigned char i=0,temp;
	Init_Devices( );

	while(1)
	{

	}
}
