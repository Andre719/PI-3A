//������ʱ�Ӳ����ڲ�RC������     DCO��8MHz,��CPUʱ��;  SMCLK��1MHz,����ʱ��ʱ��
#include <msp430g2553.h>
#include <tm1638.h>  //��TM1638�йصı���������������ڸ�H�ļ���

//////////////////////////////
//         ��������         //
//////////////////////////////

// 0.1s�����ʱ�����ֵ��5��20ms
#define V_T100ms	5
// 1.5s�����ʱ�����ֵ��75��20ms
#define V_T1500ms	75

//DAC6571����  P1.4��SDA(pin4),P1.5��SCL(pin5)
#define SCL_L       P1OUT&=~BIT5
#define SCL_H       P1OUT|=BIT5
#define SDA_L       P1OUT&=~BIT4
#define SDA_H       P1OUT|=BIT4
#define SDA_IN      P1OUT|=BIT4; P1DIR&=~BIT4; P1REN|=BIT4
#define SDA_OUT     P1DIR|=BIT4; P1REN&=~BIT4
#define DAC6571_voltage_max      499  //499x10mV
#define DAC6571_address         0x98  // 1001 10 A0 0  A0=0

//////////////////////////////
//       ��������           //
//////////////////////////////

// �����ʱ������
unsigned char clock100ms=0;
unsigned char clock1500ms=0;
// �����ʱ�������־
unsigned char clock100ms_flag=0;
unsigned char clock1500ms_flag=0;
// 8λ�������ʾ�����ֻ���ĸ����
// ע����������λ�������������Ϊ4��5��6��7��0��1��2��3
//[1.2]��4λ�̶���ʾ�����桱��Ӣ��GAIN,������3λ�̶���ʾ�����š�
unsigned char digit[8]={' ',' ',' ',' ',' ',' ',' ',' '};
// 8λС���� ��1�͵�6��
// ע����������λС����������������Ϊ4��5��6��7��0��1��2��3
unsigned char pnt=0x21;
// 8��LEDָʾ��״̬��ÿ����4����ɫ״̬��0��1�̣�2�죬3�ȣ���+�̣�
// ע������ָʾ�ƴ������������Ϊ7��6��5��4��3��2��1��0
//     ��ӦԪ��LED8��LED7��LED6��LED5��LED4��LED3��LED2��LED1
//LED����Ϊȫ��
unsigned char led[]={0,0,0,0,0,0,0,0};

//������ѹ�����ı���
unsigned int ADC_mod =0;
unsigned int ADC_Volt[50];
unsigned int ADC_Current1[50];
unsigned int ADC_Current2[50];
unsigned int ADC_ptrV = 0;
unsigned int ADC_ptrC1 = 0;
unsigned int ADC_ptrC2 = 0;
signed int ADC_data;
unsigned long Volt = 0;  //ADת������ֵ(P1.3) J2
unsigned long Current1 = 0;  //ADת������ֵ(P1.6) J1
unsigned long Current2 = 0;  //ADת������ֵ(P1.7) J1
int showmode = 1;
unsigned long show = 0;
// ��ǰ����ֵ
unsigned char key_code=0;
unsigned char key_cnt=0;

// DAC6571
unsigned int dac6571_code=0x0800;
unsigned int dac6571_voltage=80;
unsigned char dac6571_flag=0;

//////////////////////////////
//       ϵͳ��ʼ��         //
//////////////////////////////

//ADC��ʼ���ӳ��������Զ�����
void ADC10_Init(void)
{
	ADC10CTL0 = 0x00;						//�ر�ADC
	ADC10CTL0 = ADC10ON + ADC10SHT_3+ SREF_1 + REFON;		//REF2_5V
	ADC10CTL1 = INCH_6 + CONSEQ_0 + ADC10DIV_0 + ADC10SSEL_3;	//ͨ��A7����ͨ�����β�����1��Ƶ��ADC10OSC
	ADC10CTL0 |= ENC + ADC10SC;				//��ʼ����
}
//  I/O�˿ں����ų�ʼ��
void Init_Ports(void)
{
	P2SEL &= ~(BIT7+BIT6);       //P2.6��P2.7 ����Ϊͨ��I/O�˿�
	  //������Ĭ�������⾧�񣬹�����޸�

	P2DIR |= BIT7 + BIT6 + BIT5; //P2.5��P2.6��P2.7 ����Ϊ���
	  //����·������������������ʾ�ͼ��̹�����TM1638������ԭ�������DATASHEET

	//��ʼ��P1.6��P1.7��P1.3��Ϊ����
	P1DIR &= ~(BIT3 + BIT6 + BIT7);//ADת������ֵ(P1.3)J2 (P1.6)J1 (P1.7)J1(��2)
	// DAC6571
	P1DIR |= BIT4 + BIT5;//P1.4��SDA(pin4),P1.5��SCL(pin5)
	P1OUT |= BIT4 + BIT5;
}

//  ��ʱ��TIMER0��ʼ����ѭ����ʱ20ms
void Init_Timer0(void)
{
	TA0CTL = TASSEL_2 + MC_1 ;      // Source: SMCLK=1MHz, UP mode,
	TA0CCR0 = 20000;                // 1MHzʱ��,����20000��Ϊ 20ms
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

    Init_Ports();           //���ú�������ʼ��I/O��
    Init_Timer0();          //���ú�������ʼ����ʱ��0
    ADC10_Init();           //���ú�������ʼ��ADC
    _BIS_SR(GIE);           //��ȫ���ж�
   //all peripherals are now initialized
}
//////////////////////////////
//      �����ӳ���                                    //
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
//      �жϷ������                                 //
//////////////////////////////

// Timer0_A0 interrupt service routine
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer0_A0 (void)
{
	// 0.1������ʱ������
	if (++clock100ms>=V_T100ms)
	{
		clock100ms_flag = 1; //��0.1�뵽ʱ�������־��1
		clock100ms = 0;
	}
 	// 1.5������ʱ������
	if (++clock1500ms>=V_T1500ms)
	{
		clock1500ms_flag = 1; //��1.5�뵽ʱ�������־��1
		clock1500ms = 0;
	}

	// ˢ��ȫ������ܺ�LEDָʾ��
	TM1638_RefreshDIGIandLED(digit,pnt,led);

	//������ѹ����
	if(ADC_mod == 3)
		ADC_mod = 0;
	if(ADC_mod == 0)							//����Դ����
	{
		ADC_data = (int)(ADC10MEM)*1.511673-71.4659;	//����
		ADC_Current1[ADC_ptrC1++] = ADC_data;
		if(ADC_ptrC1 == 50)
			ADC_ptrC1 = 0;
		ADC10CTL0 = 0x00;						//�ر�ADC
		ADC10AE0 |= BIT7;						//ʹ��ͨ��A7
		ADC10CTL0 = ADC10ON + ADC10SHT_3 + SREF_1 + REFON;
		ADC10CTL1 = INCH_7 + CONSEQ_0 + ADC10DIV_0 + ADC10SSEL_0;	//ͨ��A0����ͨ�����β�����1��Ƶ��ADC10OSC
		ADC10CTL0 |= ENC + ADC10SC;				//��ʼ����
	}
	if(ADC_mod == 1)							//����Դ����
	{
		ADC_data = (int)(ADC10MEM)*1.515034+13.0829;	//����
		ADC_Current2[ADC_ptrC2++] = ADC_data;
		if(ADC_ptrC2 == 50)
			ADC_ptrC2 = 0;
		ADC10CTL0 = 0x00;						//�ر�ADC
		ADC10AE0 |= BIT3;
		ADC10CTL0 = ADC10ON + ADC10SHT_3 + SREF_1 + REFON;
		ADC10CTL1 = INCH_3 + CONSEQ_0 + ADC10DIV_0 + ADC10SSEL_0;	//ͨ��A6����ͨ�����β�����1��Ƶ��ADC10OSC
		ADC10CTL0 |= ENC + ADC10SC;				//��ʼ����
	}
	if(ADC_mod == 2)							//��ѹԴ����
	{
		ADC_data = (int)(ADC10MEM)*5.791449-45.2117;	//����
		ADC_Volt[ADC_ptrV++] = ADC_data;
		if(ADC_ptrV == 50)
			ADC_ptrV = 0;
		ADC10CTL0 = 0x00;						//�ر�ADC
		ADC10AE0 |= BIT6;
		ADC10CTL0 = ADC10ON + ADC10SHT_3 + SREF_1 + REFON;
		ADC10CTL1 = INCH_6 + CONSEQ_0 + ADC10DIV_0 + ADC10SSEL_0;	//ͨ��A6����ͨ�����β�����1��Ƶ��ADC10OSC
		ADC10CTL0 |= ENC + ADC10SC;				//��ʼ����
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

	// ��鵱ǰ�������룬0�����޼�������1-16��ʾ�ж�Ӧ����
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
//         ������           //
//////////////////////////////

int main(void)
{
	//unsigned char i=0,temp;
	Init_Devices( );
	while (clock100ms<3);   // ��ʱ60ms�ȴ�TM1638�ϵ����
	init_TM1638();	    //��ʼ��TM1638
	dac6571_flag = 1;
	float temp;
	while(1)
	{
		if (dac6571_flag==1)   // ���DAC��ѹ�Ƿ�Ҫ��
		{
			dac6571_flag=0;

			digit[5] = dac6571_voltage/100%10; 	//�����λ��
			digit[6] = dac6571_voltage/10%10; 	//����ʮ��λ��
			digit[7] = dac6571_voltage%10;      //����ٷ�λ��

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
		if (clock1500ms_flag==1)   // ���1.5�붨ʱ�Ƿ�
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
			digit[0] = show/1000%10; 	    //����ǧλ��
			digit[1] = show/100%10; 	    //�����λ��
			digit[2] = show/10%10;	    //����ʮλ��
			digit[3] = show%10;        //�����λ��
		}
	}

}
