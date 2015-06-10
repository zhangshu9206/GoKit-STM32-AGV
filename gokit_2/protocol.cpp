#include <MemoryFree.h>
#include <SSD1306.h>
#include "protocol.h"
#include "GoKit.h"
#include "MotorCar.h" 
extern SSD1306 oled;

unsigned long                   check_status_time=0;
unsigned long                   report_status_idle_time=0;
static unsigned char rs_rec_flag = 0; 
static unsigned char rs_buffer[3]; 
static unsigned char rs_i; 

int comtemp;
int IRR=8;//定义右侧寻线传感器接口
int IRM=9;//定义中间寻线传感器接口
int IRL=10;//定义左侧寻线传感器接口

unsigned char CheckSum( unsigned char *buf, int packLen )
{
  int       i;
  unsigned char  sum;
  
  if(buf == NULL || packLen <= 0) return 0;

  sum = 0;
  for(i=2; i<packLen-1; i++) sum += buf[i];

  return sum;
}
void  CmdSendMcuP0(uint8_t *buf)
{
  uint8_t   tmp_cmd_buf;
  byte Readmv_stop[5]={255,00,00,00,255};//停止
  if(buf == NULL) return ;
  
  memcpy(&m_w2m_controlMcu, buf, sizeof(w2m_controlMcu));
  //m_w2m_controlMcu.status_w.motor_speed = exchangeBytes(m_w2m_controlMcu.status_w.motor_speed);
  
  if(m_w2m_controlMcu.sub_cmd == SUB_CMD_REQUIRE_STATUS) gokit_ReportStatus(REQUEST_STATUS);
  
  if(m_w2m_controlMcu.sub_cmd == SUB_CMD_CONTROL_MCU)
  {
    //发送P0接收确认
    SendCommonCmd(CMD_SEND_MCU_P0_ACK, m_w2m_controlMcu.head_part.sn);
    
    #if(DEBUG==1)
    mySerial.print("m_w2m_controlMcu.cmd_tag[0] :");
    mySerial.println(m_w2m_controlMcu.cmd_tag[0]);
    mySerial.print("m_w2m_controlMcu.cmd_tag[1] :");
    mySerial.println(m_w2m_controlMcu.cmd_tag[1]);
    mySerial.print("m_w2m_controlMcu.cmd_tag[2] :");
    mySerial.println(m_w2m_controlMcu.cmd_tag[2]);
    #endif
    
    //control LED R && 手动控制模式
    if((m_w2m_controlMcu.cmd_tag[2] & 0x01) == 0x01)
    {
		//0 bit, 1: R on, 0: R off;
		if((m_w2m_controlMcu.status_w.cmd_byte[1] & 0x01) == 0x01)
		{
			byte Readmv[5]={255,19,00,00,255};
			#if (DEBUG==1)
			mySerial.println("set RED !!!!");
			#endif
			
			gokit_setColorRGB(254, 0, 0);
			m_m2w_mcuStatus.status_w.cmd_byte[1] = (m_m2w_mcuStatus.status_w.cmd_byte[1] | 0x01);

		}
		else
		{
			gokit_setColorRGB(0, 0, 0);
			m_m2w_mcuStatus.status_w.cmd_byte[1] = (m_m2w_mcuStatus.status_w.cmd_byte[1] & 0xFE);

		}
    }
	//前进
	if((m_w2m_controlMcu.cmd_tag[2] & 0x02) == 0x02)
    {
		if((m_w2m_controlMcu.status_w.cmd_byte[1] & 0x02) == 0x02)
		{
			byte Readmv[5]={255,00,02,00,255};
			#if (DEBUG==1)
			mySerial.println("Mode_forward");
			#endif
			
            Motor.forward(250);
			m_m2w_mcuStatus.status_w.cmd_byte[1] = (m_m2w_mcuStatus.status_w.cmd_byte[1] | 0x02);

		}
		else
		{
            Motor.stop();
			m_m2w_mcuStatus.status_w.cmd_byte[1] = (m_m2w_mcuStatus.status_w.cmd_byte[1] & 0xFD);

		}
		
	}
	//后退
	if((m_w2m_controlMcu.cmd_tag[2] & 0x04) == 0x04)
    {
		if((m_w2m_controlMcu.status_w.cmd_byte[1] & 0x04) == 0x04)
		{
			byte Readmv[5]={255,00,01,00,255};
			#if (DEBUG==1)
			mySerial.println("Mode_back");
			#endif
			
            Motor.back(250);
			m_m2w_mcuStatus.status_w.cmd_byte[1] = (m_m2w_mcuStatus.status_w.cmd_byte[1] | 0x04);

		}
		else
		{
            Motor.stop();
			m_m2w_mcuStatus.status_w.cmd_byte[1] = (m_m2w_mcuStatus.status_w.cmd_byte[1] & 0xFB);

		}
		
	}
    
	//原地左转
	if((m_w2m_controlMcu.cmd_tag[2] & 0x20) == 0x20)
    {
		if((m_w2m_controlMcu.status_w.cmd_byte[1] & 0x20) == 0x20)
		{
			byte Readmv[5]={255,00,03,00,255};
			#if (DEBUG==1)
			mySerial.println("Mode_turnLeftOrigin");
			#endif
			
            Motor.turnLeftOrigin(250);
			m_m2w_mcuStatus.status_w.cmd_byte[1] = (m_m2w_mcuStatus.status_w.cmd_byte[1] | 0x20);

		}
		else
		{
            Motor.stop();
			m_m2w_mcuStatus.status_w.cmd_byte[1] = (m_m2w_mcuStatus.status_w.cmd_byte[1] & 0xDF);

		}

	}

	//原地右转
	if((m_w2m_controlMcu.cmd_tag[2] & 0x40) == 0x40)
    {
		if((m_w2m_controlMcu.status_w.cmd_byte[1] & 0x40) == 0x40)
		{
			byte Readmv[5]={255,00,04,00,255};
			#if (DEBUG==1)
			mySerial.println("Mode_turnRightOrigin");
			#endif
			
            Motor.turnRightOrigin(250);
			m_m2w_mcuStatus.status_w.cmd_byte[1] = (m_m2w_mcuStatus.status_w.cmd_byte[1] | 0x40);

		}
		else
		{
            Motor.stop();
			m_m2w_mcuStatus.status_w.cmd_byte[1] = (m_m2w_mcuStatus.status_w.cmd_byte[1] & 0xFFBF);

		}

	}
	
	//循迹模式
	if((m_w2m_controlMcu.cmd_tag[1] & 0x08) == 0x08)
    {
		if((m_w2m_controlMcu.status_w.cmd_byte[0] & 0x08) == 0x08)
		{
			byte Readmv[5]={255,19,02,00,255};
			#if (DEBUG==1)
			mySerial.println("Mode_tracking");
			#endif
			
            Tracking_section();
			m_m2w_mcuStatus.status_w.cmd_byte[0] = (m_m2w_mcuStatus.status_w.cmd_byte[0] | 0x08);

		}
		else
		{
			byte Readmv[5]={255,19,00,00,255};
			//中断
            Motor.stop();
			m_m2w_mcuStatus.status_w.cmd_byte[0] = (m_m2w_mcuStatus.status_w.cmd_byte[0] & 0xF7);

		}

	}
	
    //设置LED组合颜色（自、黄、紫、粉）
    if((m_w2m_controlMcu.cmd_tag[1] & 0x10) == 0x10)
    {
		tmp_cmd_buf = (m_w2m_controlMcu.status_w.cmd_byte[0] & 0x30) >> 4;
		#if (DEBUG==1)
		mySerial.print("tmp_cmd_buf:");
		mySerial.println(tmp_cmd_buf);
		//00：user define, 01: yellow, 10: purple, 11: pink	
		#endif
		
		if(tmp_cmd_buf == 0x00)
		{
			#if (DEBUG==1)
			mySerial.println("SET LED:Custom");
			#endif
			
			gokit_setColorRGB(m_w2m_controlMcu.status_w.led_r, m_m2w_mcuStatus.status_w.led_g, m_m2w_mcuStatus.status_w.led_b); 
			m_m2w_mcuStatus.status_w.cmd_byte[0] = (m_m2w_mcuStatus.status_w.cmd_byte[0] & 0xCF);
		}
		else if(tmp_cmd_buf == 0x01)
		{   
			#if (DEBUG==1)
			mySerial.println("SET LED:yellow");
			#endif
			
			gokit_setColorRGB(254, 70, 0);
			m_m2w_mcuStatus.status_w.cmd_byte[0] = (m_m2w_mcuStatus.status_w.cmd_byte[0] | 0x10);
			m_m2w_mcuStatus.status_w.cmd_byte[0] = (m_m2w_mcuStatus.status_w.cmd_byte[0] & 0xDF);
		}
		else if(tmp_cmd_buf == 0x02)
		{
			#if (DEBUG==1)
			mySerial.println("SET LED:Purple");
			#endif
			
			gokit_setColorRGB(254, 0, 70);  
			m_m2w_mcuStatus.status_w.cmd_byte[0] = (m_m2w_mcuStatus.status_w.cmd_byte[0] | 0x20);
			m_m2w_mcuStatus.status_w.cmd_byte[0] = (m_m2w_mcuStatus.status_w.cmd_byte[0] & 0xEF);
		}
		else if(tmp_cmd_buf == 0x03)
		{
			#if (DEBUG==1)
			mySerial.println("SET LED:Pink");
			#endif
			
			gokit_setColorRGB(238, 30, 30);
			m_m2w_mcuStatus.status_w.cmd_byte[0] = (m_m2w_mcuStatus.status_w.cmd_byte[0] | 0x30);
		}
    }
	
	tmp_cmd_buf = (m_w2m_controlMcu.status_w.cmd_byte[0] & 0x30) >> 4;
	
	//电机转速设置
//  if((m_w2m_controlMcu.cmd_tag[1] & 0x20) == 0x20)
//  {
//  	//gokit_motorstatus(m_w2m_controlMcu.status_w.motor_speed);
//  	#if (DEBUG==1)
//  	mySerial.print("motorstatus:");
//  	mySerial.println(m_w2m_controlMcu.status_w.motor_speed);
//  	#endif
//
//  	m_m2w_mcuStatus.status_w.motor_speed = m_w2m_controlMcu.status_w.motor_speed;
//  }
	
	//自定义色彩：R
	if((m_w2m_controlMcu.cmd_tag[1] & 0x40) == 0x40)
	{
		if(tmp_cmd_buf == 0x00){
			gokit_setColorRGB(m_w2m_controlMcu.status_w.led_r, m_m2w_mcuStatus.status_w.led_g, m_m2w_mcuStatus.status_w.led_b);     
			m_m2w_mcuStatus.status_w.led_r = m_w2m_controlMcu.status_w.led_r;
		}
	}

	//自定义色彩：G
	if((m_w2m_controlMcu.cmd_tag[1] & 0x80) == 0x80)
	{
		if(tmp_cmd_buf == 0x00){
			gokit_setColorRGB(m_m2w_mcuStatus.status_w.led_r, m_w2m_controlMcu.status_w.led_g, m_m2w_mcuStatus.status_w.led_b);     
			m_m2w_mcuStatus.status_w.led_g = m_w2m_controlMcu.status_w.led_g;
		}
	}

	//自定义色彩：B
	if((m_w2m_controlMcu.cmd_tag[0] & 0x01) == 0x01)
	{
		if(tmp_cmd_buf == 0x00){
			gokit_setColorRGB(m_m2w_mcuStatus.status_w.led_r, m_m2w_mcuStatus.status_w.led_g, m_w2m_controlMcu.status_w.led_b);     
			m_m2w_mcuStatus.status_w.led_b = m_w2m_controlMcu.status_w.led_b;
		}
	}
	
	gokit_ReportStatus(REPORT_STATUS);

  }
}

//循迹部分
void Tracking_section()
{
	do
	{
		int r,m,l;
		r=digitalRead(IRR);
		m=digitalRead(IRM);
		l=digitalRead(IRL);
		comtemp=Serial.read();

		if(l==HIGH &&m==LOW && r==HIGH)          /*********判断前进***********/
		{
			Motor.forward(250);
			delay(2); 
			while(1)                    //正向过程中判断是否转入转向
			{
				//循环判断两侧传感器度数
				r=digitalRead(IRR);
				l=digitalRead(IRL);
				if(r==LOW)
				{
					break;
				}
				else if(l==LOW)
				{
					break;
				}
				else
					Motor.forward(200);                     //检测到l==0或r==0说明转到两侧位置，跳出循环，检测三个传感器的状态再做出相应动作
			}
		}  
		else if(l==LOW && m==HIGH && r==HIGH)    /*********判断左转***********/
		{
			Motor.turnLeft(250);
			delay(2);
			while(1)                    //转向过程中判断是否转入正向
			{
				m=digitalRead(IRM);         //循环判断中间传感器度数，
				if(m==HIGH)
				{
					Motor.turnLeft(250);       //如果m==1说明还没有转到中间位置，继续左转
					delay(2);
				}
				else
					break;                     //检测到m==0说明转过头了，跳出循环，检测三个传感器的状态再做出相应动作
			}
		}
		else if(l==HIGH && m==HIGH && r==LOW)    /*********判断右转***********/
		{
			Motor.turnRight(250);
			delay(2);
			while(1)                    //转向过程中判断是否转入正向
			{
				m=digitalRead(IRM);         //循环判断中间传感器度数，
				if(m==HIGH)
				{
					Motor.turnRight(250);       //如果m==1说明还没有转到中间位置，继续左转
					delay(2);
				}
				else
					break;                     //检测到m==0说明转过头了，跳出循环，检测三个传感器的状态再做出相应动作
			}
		}
		else if(l==LOW && m==LOW && r==LOW)     /*********判断停止***********/
		{
			Motor.stop();
			delay(2);
		}
		else                        /*********无检测下进行人为控制模式***********/
		{
			Motor.stop();
			while(1)
			{
				int r,m,l;
				int comtemp2;
				r=digitalRead(IRR);
				m=digitalRead(IRM);
				l=digitalRead(IRL);
				if( Serial.available())
				{
					comtemp2=Serial.read();
					switch(comtemp2)
					{
						case 'w':
								Motor.forward(200);
								//前进避障功能
								Avoidance_section(comtemp2);
								delay(1000);
								break;
						case 's':
								Motor.back(200);
								delay(1000);
								break;
						case 'a':
								Motor.turnLeftOrigin(200);
								delay(1000);
								break;
						case 'd':
								Motor.turnRightOrigin(200);
								delay(1000);
								break; 
						case 'q':
								Motor.stop();
								delay(1000);
								break;
						default:
								Motor.stop();
								break;
					}
				}
				else if(comtemp2 == 't')
				{
					break;
				}
				else
					break;//循迹传感器在接收到线的信号后退出“无检测下进行人为控制”模式
			}
		}
	}
	while(comtemp != 't');
}

void Handle_uartdata(unsigned char *buf,int len)
{
    if( len < 4 ) return ;

    pro_headPart  tmp_headPart;   
    memset(&tmp_headPart, 0, sizeof(pro_headPart));
    memcpy(&tmp_headPart, buf, sizeof(pro_headPart));
    if(CheckSum(buf,len)!=buf[len-1])
    {
        SendErrorCmd(ERROR_CHECKSUM, tmp_headPart.sn);
        #if(DEBUG==1)
        Serial.println("CheckSum error!");
        mySerial.println("CheckSum error!");
        #endif
        return ;
    }    
    #if(DEBUG==1)
    Serial.print("sn = ");
    Serial.println(tmp_headPart.sn);
   
    mySerial.println("receive:");
    for (int i = 0; i < len; ++i)
    {
      mySerial.print(" "); mySerial.print(buf[i],HEX);
    }
    mySerial.println("");
    #endif
    switch( tmp_headPart.cmd )
    {
      
      case CMD_GET_MCU_INFO :
          #if(DEBUG==1)
          Serial.println("CMD_GET_MCU_INFO");
          mySerial.println("CMD_GET_MCU_INFO");
          #endif
          CmdGetMcuInfo(tmp_headPart.sn);
      
      break;
      
      case CMD_SEND_MCU_P0 :
            #if(DEBUG==1)
            Serial.println("CMD_SEND_MCU_P0");
            mySerial.println("CMD_SEND_MCU_P0");
            #endif
            CmdSendMcuP0(buf);
      break;
      case CMD_SEND_HEARTBEAT:
            #if(DEBUG==1)
            Serial.println("CMD_SEND_HEARTBEAT");
            mySerial.println("CMD_SEND_HEARTBEAT");
            #endif
            SendCommonCmd(CMD_SEND_HEARTBEAT_ACK,tmp_headPart.sn);
      break;
      /*
      case CMD_REPORT_MODULE_STATUS:
            #if(DEBUG==1)
            Serial.println("CMD_REPORT_MODULE_STATUS");
            #endif
      break;
      */
      default:
            #if(DEBUG==1)
            //Serial.println("default");
            #endif
            //SendErrorCmd(ERROR_CMD, tmp_headPart.sn);
      break;
    }
  
}
void Handle_keyeven()
{
  /*  长按是指按住按键3s以上   */
  switch(gokit_keydown())
  {
    case KEY1_SHORT_PRESS:
       #if (DEBUG==1) 
        Serial.println("KEY1_SHORT_PRESS");
       #endif

    break;
    case KEY1_LONG_PRESS:
      #if (DEBUG==1) 
        Serial.println("KEY1_LONG_PRESS");
       #endif
        gokit_ResetWiFi();
    break;
    case KEY2_SHORT_PRESS:
       #if (DEBUG==1) 
        Serial.println("KEY2_SHORT_PRESS");
       #endif
          gokit_sendAirlink();
    break;
    case KEY2_LONG_PRESS:
       #if (DEBUG==1) 
        Serial.println("KEY2_LONG_PRESS");
       #endif
       gokit_sendApCmd();
    break;
    default: break;
  }
  //KEY1 : D6
  //KEY2 : D7
}
void Check_Status()
{
  int         i, diff;
  uint8_t     *index_new, *index_old;
  
  diff = 0;
  gokit_DHT11_Read_Data(&m_m2w_mcuStatus.status_r.temputure, &m_m2w_mcuStatus.status_r.humidity);
  
  if(gokit_time_s()-check_status_time < 10 ) return ;
    
  check_status_time = gokit_time_s();
  index_new = (uint8_t *)&(m_m2w_mcuStatus.status_w);
  index_old = (uint8_t *)&(m_m2w_mcuStatus_reported.status_w);
    
  for(i=0; i<sizeof(status_writable); i++)
  {
    if(*(index_new+i) != *(index_old+i)) 
      {
         diff += 1;
         #if(DEBUG==1)
         Serial.print("status_w ");Serial.println(i,DEC);
         #endif
      }

  }
    
  if(diff == 0)
  {
    index_new = (uint8_t *)&(m_m2w_mcuStatus.status_r);
    index_old = (uint8_t *)&(m_m2w_mcuStatus_reported.status_r);
      
    for(i=0; i<sizeof(status_readonly); i++)
    {
      if(*(index_new+i) != *(index_old+i))
            {
                diff += 1;
                #if(DEBUG==1)
                Serial.print("status_r "); Serial.println(i, DEC);
                Serial.print("old: "); Serial.println(index_old[i]);
                Serial.print("new: "); Serial.println(index_new[i]);
                Serial.print("temp:"); Serial.println(m_m2w_mcuStatus.status_r.temputure, DEC);
                Serial.print("hum: "); Serial.println(m_m2w_mcuStatus.status_r.humidity, DEC);
                #endif
            }
        }
    }
    if(diff > 0 || gokit_time_s() - report_status_idle_time > GOKIT_REPORT_TIME)
    {
        gokit_ReportStatus(REPORT_STATUS);
        report_status_idle_time = gokit_time_s();
    }

}

void rs_Communication_Decode(void)
{
    if(rs_buffer[0] == '4')
    {
        switch(rs_buffer[1])
        {
        case '1':
			#if (MYSERIAL1_DATA==1)
			//mySerial.write(&Readmv_stop[0],5);//"FF410000FF"
            mySerial_1.println("#1GC1\r\n");
			#endif	

            return;
        case '2':
			#if (MYSERIAL1_DATA==1)
			//mySerial.write(&Readmv_stop[0],5);//"FF420000FF"
            mySerial_1.println("#2GC1\r\n");
			#endif

            return;
        case '0':
			#if (MYSERIAL1_DATA==1)
			//mySerial.write(&Readmv_stop[0],5);//"FF400000FF"
            mySerial_1.println("#3GC1\r\n");
			#endif

            return;

        case '3':

            return;
        default:
            return;
        }
    }
    else
    {
        return;
    }
}

void Handle_uartss_data(void)
{
    unsigned char outputByte; 

    if(mySerial.available() > 0 )
    {
        unsigned char outputByte = 0; 
        outputByte = mySerial.read(); 
        
        //Serial.print("outputByte:");
        //Serial.println(outputByte);
        
        if(rs_rec_flag == 0)
        {
            if(outputByte == 'F')
            {
                rs_rec_flag = 1;
                rs_i = 0;
            }
        }
        else
        {
            if(outputByte == 'F')
            {
                rs_rec_flag = 0;
                if(rs_i == 3)
                {
                    rs_Communication_Decode();
                    //UART_init(); //	???????????????
                }
                rs_i = 0;
            }
            else
            {
                rs_buffer[rs_i] = outputByte;
                rs_i++;
            }
        } 
    } 
  
}

void GoKit_Handle(void)
{
  Handle_uartdata(  uart_buf,get_onepackage(uart_buf));
  Handle_keyeven();
  Check_Status();
  Handle_uartss_data();
}

