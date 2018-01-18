//本程序时钟采用内部RC振荡器。     DCO：8MHz,供CPU时钟;  SMCLK：1MHz,供定时器时钟
#include <msp430g2553.h>
//#include <tm1638.h>  //与TM1638有关的变量及函数定义均在该H文件中

//[3,1]定义用于控制输出电平的4路GPIO引脚操作命令
#define CTL0_L  P1OUT&=~BIT0;
#define CTL0_H  P1OUT|=BIT0;
#define CTL1_H  P1OUT|=BIT1;
#define CTL1_L  P1OUT&=~BIT1;
#define CTL2_H  P1OUT|=BIT2;
#define CTL3_H  P1OUT|=BIT3;
#define CTL2_L  P1OUT&=~BIT2;
#define CTL3_L  P1OUT&=~BIT3;

    //m序列输出 BIT4
    //符号时钟输出 BIT5
    //周期记号 BIT6
    //差分序列输出 BIT7

//////////////////////////////
//         常量定义         //
//////////////////////////////

// 0.1s软件定时器溢出值，5个20ms
#define V_T100ms	5
// 0.5s软件定时器溢出值，25个20ms
#define V_T500ms	25

//[1,1]增益等级总数
#define GAIN_STATENUM 15

//////////////////////////////
//       变量定义           //
//////////////////////////////
//m4序列的变量
int reg[4]={1,1,1,1};
int mout = 1;//m序列输出 BIT0
int synClock = 1;//符号时钟输出 BIT1
int period = 8;//周期记号 BIT2
int peFlag=0;
int syFlag = 1;
int defOut = 1;//查分序列输出
int button_flag;


// 软件定时器计数
unsigned char clock100ms=0;
unsigned char clock500ms=0;
// 软件定时器溢出标志
unsigned char clock100ms_flag=0;
unsigned char clock500ms_flag=0;
// 测试用计数器
unsigned int test_counter=0;
// 8位数码管显示的数字或字母符号
// 注：板上数码位从左到右序号排列为4、5、6、7、0、1、2、3
//[1,2]左四位固定显示“增益”的英文GAIN，右数第三位固定显示负号
unsigned char digit[8]={' ','-',' ',' ','G','A','I','N'};
// 8位小数点 1亮  0灭
// 注：板上数码位小数点从左到右序号排列为4、5、6、7、0、1、2、3
unsigned char pnt=0x04;     //这句是什么……？
// 8个LED指示灯状态，每个灯4种颜色状态，0灭，1绿，2红，3橙（红+绿）
// 注：板上指示灯从左到右序号排列为7、6、5、4、3、2、1、0
//     对应元件LED8、LED7、LED6、LED5、LED4、LED3、LED2、LED1
//led全灭
unsigned char led[]={0,0,0,0,0,0,0,0};
//[1,4]增益等级取值，初值为1，对应增益为0.1
unsigned char gain_state = 1;
// add two global variables concerned about buttons
unsigned char key_state = 0, key_flag = 1;
// 当前按键值
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
//       系统初始化         //
//////////////////////////////


//timer1 initialize
void timer1_init(void)
{
    //初始化P2.1 设置为定时器A1 的PWM 输出引脚
    P2SEL |= BIT1;
    P2DIR |= BIT1;

    // 初始化Timer1，产生440Hz 的方波信号
    TA1CTL = TASSEL_2 + MC_1; // Source: SMCLK=1MHz, PWM mode
    TA1CCTL1 = OUTMOD_7;
    TA1CCR0 = 1000000/440; //设定周期,1000000 为定时器1 时钟频率,440 为音频频率
    TA1CCR1 = TA1CCR0/2; //设置占空比等于50%

    P1DIR&=~BIT7;   //把1.7引脚设为输入，用于红外信号
}

//  I/O端口和引脚初始化
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

//  定时器TIMER0初始化，循环定时20ms
void Init_Timer0(void)
{
	TA0CTL = TASSEL_2 + MC_1 ;      // Source: SMCLK=1MHz, UP mode,
	TA0CCR0 = 1000;                // 1MHz时钟,计满20000次为 20ms         //应该为1000还是20000？
	TA0CCTL0 = CCIE;                // TA0CCR0 interrupt enabled
}

//  MCU器件初始化，注：会调用上述函数
void Init_Devices(void)
{
	WDTCTL = WDTPW + WDTHOLD;     // Stop watchdog timer，停用看门狗
	if (CALBC1_8MHZ ==0xFF || CALDCO_8MHZ == 0xFF)
	{
		while(1);            // If calibration constants erased, trap CPU!!
	}

    //设置时钟，内部RC振荡器。     DCO：8MHz,供CPU时钟;  SMCLK：1MHz,供定时器时钟
	BCSCTL1 = CALBC1_8MHZ; 	 // Set range
	DCOCTL = CALDCO_8MHZ;    // Set DCO step + modulation
	BCSCTL3 |= LFXT1S_2;     // LFXT1 = VLO
	IFG1 &= ~OFIFG;          // Clear OSCFault flag
	BCSCTL2 |= DIVS_3;       //  SMCLK = DCO/8

	timer1_init();
    Init_Ports();           //调用函数，初始化I/O口
    Init_Timer0();          //调用函数，初始化定时器0
    _BIS_SR(GIE);           //开全局中断
    _EINT();                                                                        //这是啥？
   //all peripherals are now initialized
}


//////////////////////////////
//      中断服务程序        // 加入了播放音乐的功能
//////////////////////////////

// Timer0_A0 interrupt service routine
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer0_A0 (void)
{
    if (button_flag)m4creat();
    else m3creat();
    //m序列输出 BIT4
    //符号时钟输出 BIT5
    //周期记号 BIT6 误码检测时改为输出m序列阶数 button_flag（为1则4阶）
    //差分序列输出 BIT7
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
//         主程序           //
//////////////////////////////

int main(void)
{
	//unsigned char i=0,temp;
	Init_Devices( );

	while(1)
	{

	}
}
