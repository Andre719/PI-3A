//本程序时钟采用内部RC振荡器。     DCO：8MHz,供CPU时钟;  SMCLK：1MHz,供定时器时钟
#include <msp430g2553.h>
#include <tm1638.h>  //与TM1638有关的变量及函数定义均在该H文件中

//////////////////////////////
//         常量定义         //
//////////////////////////////

// 0.1s软件定时器溢出值，5个20ms
#define V_T100ms	5
// 1.5s软件定时器溢出值，75个20ms
#define V_T1500ms	75

//DAC6571操作  P1.4接SDA(pin4),P1.5接SCL(pin5)
#define SCL_L       P1OUT&=~BIT5
#define SCL_H       P1OUT|=BIT5
#define SDA_L       P1OUT&=~BIT4
#define SDA_H       P1OUT|=BIT4
#define SDA_IN      P1OUT|=BIT4; P1DIR&=~BIT4; P1REN|=BIT4
#define SDA_OUT     P1DIR|=BIT4; P1REN&=~BIT4
#define DAC6571_voltage_max      499  //499x10mV
#define DAC6571_address         0x98  // 1001 10 A0 0  A0=0

//////////////////////////////
//       变量定义           //
//////////////////////////////

// 软件定时器计数
unsigned char clock100ms=0;
unsigned char clock1500ms=0;
// 软件定时器溢出标志
unsigned char clock100ms_flag=0;
unsigned char clock1500ms_flag=0;
// 8位数码管显示的数字或字母符号
// 注：板上数码位从左到右序号排列为4、5、6、7、0、1、2、3
//[1.2]左4位固定显示“增益”的英文GAIN,右数第3位固定显示“负号”
unsigned char digit[8]={' ',' ',' ',' ',' ',' ',' ',' '};
// 8位小数点 第1和第6亮
// 注：板上数码位小数点从左到右序号排列为4、5、6、7、0、1、2、3
unsigned char pnt=0x21;
// 8个LED指示灯状态，每个灯4种颜色状态，0灭，1绿，2红，3橙（红+绿）
// 注：板上指示灯从左到右序号排列为7、6、5、4、3、2、1、0
//     对应元件LED8、LED7、LED6、LED5、LED4、LED3、LED2、LED1
//LED灯设为全灭
unsigned char led[]={0,0,0,0,0,0,0,0};

//测量电压电流的变量
unsigned int ADC_mod =0;
unsigned int ADC_Volt[50];
unsigned int ADC_Current1[50];
unsigned int ADC_Current2[50];
unsigned int ADC_ptrV = 0;
unsigned int ADC_ptrC1 = 0;
unsigned int ADC_ptrC2 = 0;
signed int ADC_data;
unsigned long Volt = 0;  //AD转换后数值(P1.3) J2
unsigned long Current1 = 0;  //AD转换后数值(P1.6) J1
unsigned long Current2 = 0;  //AD转换后数值(P1.7) J1
int showmode = 1;
unsigned long show = 0;
// 当前按键值
unsigned char key_code=0;
unsigned char key_cnt=0;

// DAC6571
unsigned int dac6571_code=0x0800;
unsigned int dac6571_voltage=80;
unsigned char dac6571_flag=0;

//////////////////////////////
//       系统初始化         //
//////////////////////////////

//ADC初始化子程序，用于自动增益
void ADC10_Init(void)
{
	ADC10CTL0 = 0x00;						//关闭ADC
	ADC10CTL0 = ADC10ON + ADC10SHT_3+ SREF_1 + REFON;		//REF2_5V
	ADC10CTL1 = INCH_6 + CONSEQ_0 + ADC10DIV_0 + ADC10SSEL_3;	//通道A7，单通道单次采样，1分频，ADC10OSC
	ADC10CTL0 |= ENC + ADC10SC;				//开始采样
}
//  I/O端口和引脚初始化
void Init_Ports(void)
{
	P2SEL &= ~(BIT7+BIT6);       //P2.6、P2.7 设置为通用I/O端口
	  //因两者默认连接外晶振，故需此修改

	P2DIR |= BIT7 + BIT6 + BIT5; //P2.5、P2.6、P2.7 设置为输出
	  //本电路板中三者用于连接显示和键盘管理器TM1638，工作原理详见其DATASHEET

	//初始化P1.6、P1.7、P1.3作为输入
	P1DIR &= ~(BIT3 + BIT6 + BIT7);//AD转换后数值(P1.3)J2 (P1.6)J1 (P1.7)J1(板2)
	// DAC6571
	P1DIR |= BIT4 + BIT5;//P1.4接SDA(pin4),P1.5接SCL(pin5)
	P1OUT |= BIT4 + BIT5;
}

//  定时器TIMER0初始化，循环定时20ms
void Init_Timer0(void)
{
	TA0CTL = TASSEL_2 + MC_1 ;      // Source: SMCLK=1MHz, UP mode,
	TA0CCR0 = 20000;                // 1MHz时钟,计满20000次为 20ms
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

    Init_Ports();           //调用函数，初始化I/O口
    Init_Timer0();          //调用函数，初始化定时器0
    ADC10_Init();           //调用函数，初始化ADC
    _BIS_SR(GIE);           //开全局中断
   //all peripherals are now initialized
}
//////////////////////////////
//      功能子程序                                    //
//////////////////////////////
void dac6571_byte_transmission(unsigned char byte_data)
{
	unsigned char i,shelter;
	shelter = 0x80;

	for (i=1; i<=8; i++)
	{
		if ((byte_data & shelter)==0) SDA_L;
		else SDA_H;
		SCL_H; SCL_L;
		shelter >>= 1;
	}
	SDA_IN;
	SCL_H; SCL_L;
	SDA_OUT;
}

void dac6571_fastmode_operation(void)
{
	unsigned char msbyte,lsbyte;

	SCL_H; SDA_H; SDA_L; SCL_L;       // START condition
	dac6571_byte_transmission(DAC6571_address);
	msbyte = dac6571_code/256;
	lsbyte = dac6571_code - msbyte * 256;
	dac6571_byte_transmission(msbyte);
	dac6571_byte_transmission(lsbyte);

    SDA_L; SCL_H; SDA_H;        // STOP condition
}
//////////////////////////////
//      中断服务程序                                 //
//////////////////////////////

// Timer0_A0 interrupt service routine
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer0_A0 (void)
{
	// 0.1秒钟软定时器计数
	if (++clock100ms>=V_T100ms)
	{
		clock100ms_flag = 1; //当0.1秒到时，溢出标志置1
		clock100ms = 0;
	}
 	// 1.5秒钟软定时器计数
	if (++clock1500ms>=V_T1500ms)
	{
		clock1500ms_flag = 1; //当1.5秒到时，溢出标志置1
		clock1500ms = 0;
	}

	// 刷新全部数码管和LED指示灯
	TM1638_RefreshDIGIandLED(digit,pnt,led);

	//测量电压程序
	if(ADC_mod == 3)
		ADC_mod = 0;
	if(ADC_mod == 0)							//电流源采样
	{
		ADC_data = (int)(ADC10MEM)*1.511673-71.4659;	//定标
		ADC_Current1[ADC_ptrC1++] = ADC_data;
		if(ADC_ptrC1 == 50)
			ADC_ptrC1 = 0;
		ADC10CTL0 = 0x00;						//关闭ADC
		ADC10AE0 |= BIT7;						//使能通道A7
		ADC10CTL0 = ADC10ON + ADC10SHT_3 + SREF_1 + REFON;
		ADC10CTL1 = INCH_7 + CONSEQ_0 + ADC10DIV_0 + ADC10SSEL_0;	//通道A0，单通道单次采样，1分频，ADC10OSC
		ADC10CTL0 |= ENC + ADC10SC;				//开始采样
	}
	if(ADC_mod == 1)							//电流源采样
	{
		ADC_data = (int)(ADC10MEM)*1.515034+13.0829;	//定标
		ADC_Current2[ADC_ptrC2++] = ADC_data;
		if(ADC_ptrC2 == 50)
			ADC_ptrC2 = 0;
		ADC10CTL0 = 0x00;						//关闭ADC
		ADC10AE0 |= BIT3;
		ADC10CTL0 = ADC10ON + ADC10SHT_3 + SREF_1 + REFON;
		ADC10CTL1 = INCH_3 + CONSEQ_0 + ADC10DIV_0 + ADC10SSEL_0;	//通道A6，单通道单次采样，1分频，ADC10OSC
		ADC10CTL0 |= ENC + ADC10SC;				//开始采样
	}
	if(ADC_mod == 2)							//电压源采样
	{
		ADC_data = (int)(ADC10MEM)*5.791449-45.2117;	//定标
		ADC_Volt[ADC_ptrV++] = ADC_data;
		if(ADC_ptrV == 50)
			ADC_ptrV = 0;
		ADC10CTL0 = 0x00;						//关闭ADC
		ADC10AE0 |= BIT6;
		ADC10CTL0 = ADC10ON + ADC10SHT_3 + SREF_1 + REFON;
		ADC10CTL1 = INCH_6 + CONSEQ_0 + ADC10DIV_0 + ADC10SSEL_0;	//通道A6，单通道单次采样，1分频，ADC10OSC
		ADC10CTL0 |= ENC + ADC10SC;				//开始采样
	}
	ADC_mod++;

	unsigned char i = 0;
	Volt = 0;
	Current1 = 0;
	Current2 = 0;
	for(i=0; i<50; ++i)
	{
		Volt += ADC_Volt[i];
		Current1 += ADC_Current1[i];
		Current2 += ADC_Current2[i];
	}
	Volt = Volt/50;
	Current1 = Current1/50;
	Current2 = Current2/50;

	// 检查当前键盘输入，0代表无键操作，1-16表示有对应按键
	key_code=TM1638_Readkeyboard();
	if (key_code != 0)
	{
		if (key_cnt<4) key_cnt++;
		else if (key_cnt==4)
		{
			if (key_code==1)
			{
				showmode=1;
			}
			else if (key_code==2)
			{
				showmode=2;
			}
			else if (key_code==3)
			{
				showmode=3;
			}
			else if (key_code==5)
			{
				if (dac6571_voltage < DAC6571_voltage_max-10)
				{
					dac6571_voltage+=10;
					dac6571_flag =1;
				}
			}
			else if (key_code==6)
			{
				if (dac6571_voltage > 10)
				{
					dac6571_voltage-=10;
					dac6571_flag =1;
				}
			}
			else if (key_code==7)
			{
				if (dac6571_voltage < DAC6571_voltage_max)
				{
					dac6571_voltage++;
					dac6571_flag =1;
				}
			}
			else if (key_code==8)
			{
				if (dac6571_voltage > 0)
				{
					dac6571_voltage--;
					dac6571_flag =1;
				}
			}
			key_cnt =5;
		}
	}
	else key_cnt=0;
}

//////////////////////////////
//         主程序           //
//////////////////////////////

int main(void)
{
	//unsigned char i=0,temp;
	Init_Devices( );
	while (clock100ms<3);   // 延时60ms等待TM1638上电完成
	init_TM1638();	    //初始化TM1638
	dac6571_flag = 1;
	float temp;
	while(1)
	{
		if (dac6571_flag==1)   // 检查DAC电压是否要变
		{
			dac6571_flag=0;

			digit[5] = dac6571_voltage/100%10; 	//计算个位数
			digit[6] = dac6571_voltage/10%10; 	//计算十分位数
			digit[7] = dac6571_voltage%10;      //计算百分位数

			if (dac6571_voltage<=30)
			{
				if ((Volt/Current2)<7.5)
					temp = (dac6571_voltage-9.6286)/1.006857;
				else
					temp = (dac6571_voltage-9.9727)/1.065371;
			}
			else
			{
				temp = (dac6571_voltage-10.052)/0.9907;
			}
			dac6571_code = temp*4096.0/(DAC6571_voltage_max+1);
			dac6571_fastmode_operation();
		}
		if (clock1500ms_flag==1)   // 检查1.5秒定时是否到
		{
			clock1500ms_flag=0;
			if (Current2>Current1 && Current1>100 && key_code==0)
			{
				if (Current2-Current1>100)
				{
					dac6571_voltage-=10;
				    dac6571_flag =1;
				}
				else if (Current2-Current1>15 && key_code==0)
				{
					dac6571_voltage--;
					dac6571_flag =1;
				}
			}
			if (Current1>Current2 && Current1>100 && key_code==0)
		    {
				if (Current1-Current2>100)
				{
					dac6571_voltage+=10;
					dac6571_flag =1;
				}
				else if (Current1-Current2>15  && key_code==0)
				{
					dac6571_voltage++;
					dac6571_flag =1;
				}
			}
            if (showmode==1)
            {
                if (key_code==1)
                	show=(show/10)*10+Volt%10;
                else
                	show=Volt;
            }
            else if (showmode==2)
            {
            	if (key_code==2)
            	    show=(show/10)*10+Current1%10;
            	else
            	    show=Current1;
            }
            else if (showmode==3)
            {
            	if (key_code==3)
            	    show=(show/10)*10+Current2%10;
            	else
            	    show=Current2;
            }
			digit[0] = show/1000%10; 	    //计算千位数
			digit[1] = show/100%10; 	    //计算百位数
			digit[2] = show/10%10;	    //计算十位数
			digit[3] = show%10;        //计算个位数
		}
	}

}
