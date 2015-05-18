#include <MemoryFree.h>
#include <SSD1306.h>
#include "protocol.h"
#include "GoKit.h"
#include "MotorCar.h"
extern SSD1306 oled;

unsigned long                   check_status_time=0;
unsigned long                   report_status_idle_time=0;

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
			#if MYSERIAL_DATA
			//mySerial.write("FF 00 02 00 FF");//向小车发送手动模式的对应指令FF130000FF
			mySerial.write(&Readmv[0],5);
			#endif
		}
		else
		{
			gokit_setColorRGB(0, 0, 0);
			m_m2w_mcuStatus.status_w.cmd_byte[1] = (m_m2w_mcuStatus.status_w.cmd_byte[1] & 0xFE);
			#if MYSERIAL_DATA
			//mySerial.write(&Readmv_stop[0],5);//停止FF000000FF
            mySerial.write(&Readmv_stop[0],5);
			#endif
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
			#if MYSERIAL_DATA
			mySerial.write(&Readmv[0],5);//FF000200FF
			#endif
		}
		else
		{
			Motor.stop();
			m_m2w_mcuStatus.status_w.cmd_byte[1] = (m_m2w_mcuStatus.status_w.cmd_byte[1] & 0xFD);
			#if MYSERIAL_DATA
			mySerial.write(&Readmv_stop[0],5);
			#endif
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
			#if MYSERIAL_DATA
			mySerial.write(&Readmv[0],5);
			#endif
		}
		else
		{
			Motor.stop();
			m_m2w_mcuStatus.status_w.cmd_byte[1] = (m_m2w_mcuStatus.status_w.cmd_byte[1] & 0xFB);
			#if MYSERIAL_DATA
			mySerial.write(&Readmv_stop[0],5);
			#endif
		}
		
	}
	//左转
	if((m_w2m_controlMcu.cmd_tag[2] & 0x08) == 0x08)
    {
		if((m_w2m_controlMcu.status_w.cmd_byte[1] & 0x08) == 0x08)
		{
			byte Readmv[5]={255,00,05,00,255};
			#if (DEBUG==1)
			mySerial.println("Mode_turnLeft");
			#endif
			
			Motor.turnLeft(250);
			m_m2w_mcuStatus.status_w.cmd_byte[1] = (m_m2w_mcuStatus.status_w.cmd_byte[1] | 0x08);
			#if MYSERIAL_DATA
			mySerial.write(&Readmv[0],5);
			#endif
		}
		else
		{
			Motor.stop();
			m_m2w_mcuStatus.status_w.cmd_byte[1] = (m_m2w_mcuStatus.status_w.cmd_byte[1] & 0xF7);
			#if MYSERIAL_DATA
			mySerial.write(&Readmv_stop[0],5);
			#endif
		}
		
	}

	//右转
	if((m_w2m_controlMcu.cmd_tag[2] & 0x10) == 0x10)
    {
		if((m_w2m_controlMcu.status_w.cmd_byte[1] & 0x10) == 0x10)
		{
			byte Readmv[5]={255,00,06,00,255};
			#if (DEBUG==1)
			mySerial.println("Mode_turnRight");
			#endif
			
			Motor.turnRight(250);
			m_m2w_mcuStatus.status_w.cmd_byte[1] = (m_m2w_mcuStatus.status_w.cmd_byte[1] | 0x10);
			#if MYSERIAL_DATA
			mySerial.write(&Readmv[0],5);
			#endif
		}
		else
		{
			Motor.stop();
			m_m2w_mcuStatus.status_w.cmd_byte[1] = (m_m2w_mcuStatus.status_w.cmd_byte[1] & 0xEF);
			#if MYSERIAL_DATA
			mySerial.write(&Readmv_stop[0],5);
			#endif
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
			#if MYSERIAL_DATA
			mySerial.write(&Readmv[0],5);
			#endif
		}
		else
		{
			Motor.stop();
			m_m2w_mcuStatus.status_w.cmd_byte[1] = (m_m2w_mcuStatus.status_w.cmd_byte[1] & 0xDF);
			#if MYSERIAL_DATA
			mySerial.write(&Readmv_stop[0],5);
			#endif
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
			#if MYSERIAL_DATA
			mySerial.write(&Readmv[0],5);
			#endif
		}
		else
		{
			Motor.stop();
			m_m2w_mcuStatus.status_w.cmd_byte[1] = (m_m2w_mcuStatus.status_w.cmd_byte[1] & 0xFFBF);
			#if MYSERIAL_DATA
			mySerial.write(&Readmv_stop[0],5);
			#endif
		}

	}

	//停止
	if((m_w2m_controlMcu.cmd_tag[2] & 0x80) == 0x80)
    {
		if((m_w2m_controlMcu.status_w.cmd_byte[1] & 0x80) == 0x80)
		{
			mySerial.println("Mode_stop");
			Motor.stop();
			m_m2w_mcuStatus.status_w.cmd_byte[1] = (m_m2w_mcuStatus.status_w.cmd_byte[1] | 0x80);
			#if MYSERIAL_DATA
			mySerial.write(&Readmv_stop[0],5);
			#endif
		}
		else
		{
			Motor.stop();
			m_m2w_mcuStatus.status_w.cmd_byte[1] = (m_m2w_mcuStatus.status_w.cmd_byte[1] & 0x7F);
			#if MYSERIAL_DATA
			mySerial.write(&Readmv_stop[0],5);
			#endif
		}

	}

	//执行机械臂动作组一
	if((m_w2m_controlMcu.cmd_tag[1] & 0x01) == 0x01)
    {
		byte Readmv[5]={255,65,00,00,255};
		if((m_w2m_controlMcu.status_w.cmd_byte[0] & 0x01) == 0x01)
		{
			#if (DEBUG==1)
			mySerial.println("Action_group1");
			#endif
			
			Serial.println("#1GC1\r\n");
			m_m2w_mcuStatus.status_w.cmd_byte[0] = (m_m2w_mcuStatus.status_w.cmd_byte[0] | 0x01);
			#if MYSERIAL_DATA
			mySerial.write(&Readmv_stop[0],5);//"FF410000FF"
			#endif
		}
		else
		{
			#if (DEBUG==1)
			Serial.println("#1GC1\r\n");
			#endif
			
			m_m2w_mcuStatus.status_w.cmd_byte[0] = (m_m2w_mcuStatus.status_w.cmd_byte[0] & 0xFE);
			#if MYSERIAL_DATA
			mySerial.write(&Readmv_stop[0],5);//"FF410000FF"
			#endif
		}

	}

	//执行机械臂动作组二
	if((m_w2m_controlMcu.cmd_tag[1] & 0x02) == 0x02)
    {
		byte Readmv[5]={255,66,00,00,255};
		if((m_w2m_controlMcu.status_w.cmd_byte[0] & 0x02) == 0x02)
		{
			#if (DEBUG==1)
			mySerial.println("Action_group2");
			#endif
			
			Serial.println("#2GC1\r\n");
			m_m2w_mcuStatus.status_w.cmd_byte[0] = (m_m2w_mcuStatus.status_w.cmd_byte[0] | 0x02);
			#if MYSERIAL_DATA
			mySerial.write(&Readmv_stop[0],5);//"FF420000FF"
			#endif
		}
		else
		{
			Serial.println("#2GC1\r\n");
			m_m2w_mcuStatus.status_w.cmd_byte[0] = (m_m2w_mcuStatus.status_w.cmd_byte[0] & 0xFD);
			#if MYSERIAL_DATA
			mySerial.write(&Readmv_stop[0],5);//"FF420000FF"
			#endif
		}

	}
	
	//执行机械臂动作还原
	if((m_w2m_controlMcu.cmd_tag[1] & 0x04) == 0x04)
    {
		byte Readmv[5]={255,64,00,00,255};
		if((m_w2m_controlMcu.status_w.cmd_byte[0] & 0x04) == 0x04)
		{
			#if (DEBUG==1)
			mySerial.println("Action_group_Reduction");
			#endif
			
			Serial.println("#2GC1\r\n");
			m_m2w_mcuStatus.status_w.cmd_byte[0] = (m_m2w_mcuStatus.status_w.cmd_byte[0] | 0x04);
			#if MYSERIAL_DATA
			mySerial.write(&Readmv[0],5);//"FF400000FF"
			#endif
		}
		else
		{
			Serial.println("#2GC1\r\n");
			m_m2w_mcuStatus.status_w.cmd_byte[0] = (m_m2w_mcuStatus.status_w.cmd_byte[0] & 0xFB);
			#if MYSERIAL_DATA
			mySerial.write(&Readmv[0],5);//"FF400000FF"
			#endif
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
			
			//Tracking_section();
			m_m2w_mcuStatus.status_w.cmd_byte[0] = (m_m2w_mcuStatus.status_w.cmd_byte[0] | 0x08);
			#if MYSERIAL_DATA
			mySerial.write(&Readmv[0],5);
			#endif
		}
		else
		{
			byte Readmv[5]={255,19,00,00,255};
			//中断
			Motor.stop();
			m_m2w_mcuStatus.status_w.cmd_byte[0] = (m_m2w_mcuStatus.status_w.cmd_byte[0] & 0xF7);
			#if MYSERIAL_DATA
			mySerial.write(&Readmv_stop[0],5);
			mySerial.write(&Readmv[0],5);
			#endif
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
    if((m_w2m_controlMcu.cmd_tag[1] & 0x20) == 0x20)
    {
		//gokit_motorstatus(m_w2m_controlMcu.status_w.motor_speed);
		#if (DEBUG==1)
		mySerial.print("motorstatus:");
		mySerial.println(m_w2m_controlMcu.status_w.motor_speed);
		#endif

		m_m2w_mcuStatus.status_w.motor_speed = m_w2m_controlMcu.status_w.motor_speed;
    }
	
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
        Serial.print("status_r ");Serial.println(i,DEC);
        Serial.print("old: ");Serial.println(index_old[i]);
        Serial.print("new: ");Serial.println(index_new[i]);
        Serial.print("temp:");Serial.println(m_m2w_mcuStatus.status_r.temputure,DEC);
        Serial.print("hum: ");Serial.println(m_m2w_mcuStatus.status_r.humidity,DEC);
      #endif
      }
    }
  }
  if(diff > 0 || gokit_time_s()-report_status_idle_time > GOKIT_REPORT_TIME)
  {
   gokit_ReportStatus(REPORT_STATUS);
   report_status_idle_time = gokit_time_s(); 
  }

}
void GoKit_Handle()
{
  Handle_uartdata(  uart_buf,get_onepackage(uart_buf));
  Handle_keyeven();
  Check_Status();
}

