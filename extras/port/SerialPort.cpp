//#include "StdAfx.h"
#include "SerialPort.h"
#include <process.h>
#include <iostream>

/** 线程退出标志 */ 
bool CSerialPort::s_bExit = false;
/** 当串口无数据时,sleep至下次查询间隔的时间,单位:秒 */ 
const UINT SLEEP_TIME_INTERVAL = 5;

///////////////////////////////////////
int a=0;
int c=0;
int d[9];
int e[9];
int efuzhi=0;



//three finger joint angles

int finger_one_joint[6];			//thumb

int finger_two_joint[6];			//index

int finger_three_joint[6];		//middle

int finger_one[6];

int finger_two[6];

int finger_three[6];




CSerialPort::CSerialPort(void)
: m_hListenThread(INVALID_HANDLE_VALUE)
{
	m_hComm = INVALID_HANDLE_VALUE;
	m_hListenThread = INVALID_HANDLE_VALUE;

	InitializeCriticalSection(&m_csCommunicationSync);

}

CSerialPort::~CSerialPort(void)
{
	CloseListenTread();
	ClosePort();
	DeleteCriticalSection(&m_csCommunicationSync);
}

bool CSerialPort::InitPort( UINT portNo /*= 1*/,UINT baud /*= CBR_9600*/,char parity /*= 'N'*/,
						    UINT databits /*= 8*/, UINT stopsbits /*= 1*/,DWORD dwCommEvents /*= EV_RXCHAR*/ )
{

	/** 临时变量,将制定参数转化为字符串形式,以构造DCB结构 */ 
	char szDCBparam[50];
	sprintf_s(szDCBparam, "baud=%d parity=%c data=%d stop=%d", baud, parity, databits, stopsbits);

	/** 打开指定串口,该函数内部已经有临界区保护,上面请不要加保护 */ 
	if (!openPort(portNo))
	{
		return false;
	}

	/** 进入临界段 */ 
	EnterCriticalSection(&m_csCommunicationSync);

	/** 是否有错误发生 */ 
	BOOL bIsSuccess = TRUE;

    /** 在此可以设置输入输出的缓冲区大小,如果不设置,则系统会设置默认值.
	 *  自己设置缓冲区大小时,要注意设置稍大一些,避免缓冲区溢出
	 */
	if (bIsSuccess )
	{
		bIsSuccess = SetupComm(m_hComm,8,8);
	}

	/** 设置串口的超时时间,均设为0,表示不使用超时限制 */
	COMMTIMEOUTS  CommTimeouts;
	CommTimeouts.ReadIntervalTimeout         = 0;
	CommTimeouts.ReadTotalTimeoutMultiplier  = 0;
	CommTimeouts.ReadTotalTimeoutConstant    = 0;
	CommTimeouts.WriteTotalTimeoutMultiplier = 0;
	CommTimeouts.WriteTotalTimeoutConstant   = 0; 
	if ( bIsSuccess)
	{
		bIsSuccess = SetCommTimeouts(m_hComm, &CommTimeouts);
	}

	DCB  dcb;
	if ( bIsSuccess )
	{
		// 将ANSI字符串转换为UNICODE字符串
		DWORD dwNum = MultiByteToWideChar (CP_ACP, 0, szDCBparam, -1, NULL, 0);
		wchar_t *pwText = new wchar_t[dwNum] ;
		if (!MultiByteToWideChar (CP_ACP, 0, szDCBparam, -1, pwText, dwNum))
		{
			bIsSuccess = TRUE;
		}

		/** 获取当前串口配置参数,并且构造串口DCB参数 */ 
		bIsSuccess = GetCommState(m_hComm, &dcb) && BuildCommDCB(pwText, &dcb) ;
		/** 开启RTS flow控制 */ 
		dcb.fRtsControl = RTS_CONTROL_ENABLE; 

		/** 释放内存空间 */ 
		delete [] pwText;
	}

	if ( bIsSuccess )
	{
		/** 使用DCB参数配置串口状态 */ 
		bIsSuccess = SetCommState(m_hComm, &dcb);
	}
		
	/**  清空串口缓冲区 */
	PurgeComm(m_hComm, PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_TXABORT);

	/** 离开临界段 */ 
	LeaveCriticalSection(&m_csCommunicationSync);

	return bIsSuccess==TRUE;
}

bool CSerialPort::InitPort( UINT portNo ,const LPDCB& plDCB )
{
	/** 打开指定串口,该函数内部已经有临界区保护,上面请不要加保护 */ 
	if (!openPort(portNo))
	{
		return false;
	}
	
	/** 进入临界段 */ 
	EnterCriticalSection(&m_csCommunicationSync);

	/** 配置串口参数 */ 
	if (!SetCommState(m_hComm, plDCB))
	{
		return false;
	}

	/**  清空串口缓冲区 */
	PurgeComm(m_hComm, PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_TXABORT);

	/** 离开临界段 */ 
	LeaveCriticalSection(&m_csCommunicationSync);

	return true;
}

void CSerialPort::ClosePort()
{
	/** 如果有串口被打开，关闭它 */
	if( m_hComm != INVALID_HANDLE_VALUE )
	{
		CloseHandle( m_hComm );
		m_hComm = INVALID_HANDLE_VALUE;
	}
}

bool CSerialPort::openPort( UINT portNo )
{
	/** 进入临界段 */ 
	EnterCriticalSection(&m_csCommunicationSync);

	/** 把串口的编号转换为设备名 */ 
    char szPort[50];
	sprintf_s(szPort, "COM%d", portNo);

	/** 打开指定的串口 */ 
	m_hComm = CreateFileA(szPort,		                /** 设备名,COM1,COM2等 */ 
						 GENERIC_READ | GENERIC_WRITE,  /** 访问模式,可同时读写 */   
						 0,                             /** 共享模式,0表示不共享 */ 
					     NULL,							/** 安全性设置,一般使用NULL */ 
					     OPEN_EXISTING,					/** 该参数表示设备必须存在,否则创建失败 */ 
						 0,    
						 0);    

	/** 如果打开失败，释放资源并返回 */ 
	if (m_hComm == INVALID_HANDLE_VALUE)
	{
		LeaveCriticalSection(&m_csCommunicationSync);
		return false;
	}

	/** 退出临界区 */ 
	LeaveCriticalSection(&m_csCommunicationSync);

	return true;
}

bool CSerialPort::OpenListenThread()
{
	/** 检测线程是否已经开启了 */ 
	if (m_hListenThread != INVALID_HANDLE_VALUE)
	{
		/** 线程已经开启 */ 
		return false;
	}

	s_bExit = false;
	/** 线程ID */ 
	UINT threadId;
	/** 开启串口数据监听线程 */ 
	m_hListenThread = (HANDLE)_beginthreadex(NULL, 0, ListenThread, this, 0, &threadId);
	if (!m_hListenThread)
	{
		return false;
	}
	/** 设置线程的优先级,高于普通线程 */ 
	if (!SetThreadPriority(m_hListenThread, THREAD_PRIORITY_ABOVE_NORMAL))
	{
		return false;
	}

	return true;
}

bool CSerialPort::CloseListenTread()
{	
	if (m_hListenThread != INVALID_HANDLE_VALUE)
	{
		/** 通知线程退出 */ 
		s_bExit = true;

		/** 等待线程退出 */ 
		Sleep(10);

		/** 置线程句柄无效 */ 
		CloseHandle( m_hListenThread );
		m_hListenThread = INVALID_HANDLE_VALUE;
	}  
	return true;
}

UINT CSerialPort::GetBytesInCOM()
{

	DWORD dwError = 0;	/** 错误码 */ 
	COMSTAT  comstat;   /** COMSTAT结构体,记录通信设备的状态信息 */ 
	//byte buf[];
	//int Test_Data_X;
	//int Test_Data_Y;
	//int Test_Data_Z;
	memset(&comstat, 0, sizeof(COMSTAT));//赋值之前初始化、将指针变量所指向的前 n 字节的内存单元用一个“整数”替换

	UINT BytesInQue = 0;

	DWORD BytesRead = 0;

	
	/** 在调用ReadFile和WriteFile之前,通过本函数清除以前遗留的错误标志 */ 
	if ( ClearCommError(m_hComm, &dwError, &comstat) )
	{
		BytesInQue = comstat.cbInQue; /** 获取在输入缓冲区中的字节数 */ 
	}
		
	if ( BytesInQue == 0 )
		{
			Sleep(SLEEP_TIME_INTERVAL);
		}

		///** 读取输入缓冲区中的数据并输出显示 */
		//char cRecved1 = 0x00;
		//do
		//{
		//	cRecved1 = 0x00;
		//	if(ReadChar(cRecved1) == true)
		//	{
		//	
		//		std::cout << cRecved1 ; 
		//		continue;
		//	}
		//}while(--BytesInQue);
	

	//Test_Data_X = comstat.cbInQue[1];
	//std::cout <<comstat.cbInQue <<std::endl;
	/*	std::cout <<BytesInQue<<std::endl;*/
	return BytesInQue;
}



UINT WINAPI CSerialPort::ListenThread( void* pParam )
{
	/** 得到本类的指针 */ 
	CSerialPort *pSerialPort = reinterpret_cast<CSerialPort*>(pParam);

	// 线程循环,轮询方式读取串口数据
	while (!pSerialPort->s_bExit) 
	{
		UINT BytesInQue = pSerialPort->GetBytesInCOM();
		/** 如果串口输入缓冲区中无数据,则休息一会再查询 */ 
		if ( BytesInQue == 0 )
		{
			Sleep(SLEEP_TIME_INTERVAL);
			continue;
		}



		/** 读取输入缓冲区中的数据并输出显示 */
		int xunhuan = 1;
		char cRecved = 0x00;
		//strtod(cRecved,NULL);

		do {
		
			if (xunhuan <= 7)//读取位置
			{
				cRecved = 0x00;
				if (pSerialPort->ReadChar(cRecved) == true)
				{
					//	std::cout << cRecved<<std::endl ; 				
					int b = cRecved - '0';
					//std::cout << b<<std::endl ; 
					if (-10 < b && b < 10)
					{
						finger_one_joint[0] = finger_one_joint[0] * 10 + b;
						std::cout << finger_one_joint[0] << std::endl;
					}
					xunhuan++;

				}
			};
			if (xunhuan > 7 && xunhuan <= 14)
			{
				cRecved = 0x00;
				if (pSerialPort->ReadChar(cRecved) == true)
				{
					int b = cRecved - '0';
					if (-10 < b && b < 10)
					{
						finger_one_joint[1] = finger_one_joint[1] * 10 + b;
					}
					xunhuan++;
				}
			};
			if (xunhuan > 14 && xunhuan <= 21)
			{
				cRecved = 0x00;
				if (pSerialPort->ReadChar(cRecved) == true)
				{
					int b = cRecved - '0';
					if (-10 < b && b < 10)
					{
						finger_one_joint[2] = finger_one_joint[2] * 10 + b;
					}
					xunhuan++;
				}
			};
			if (xunhuan > 21 && xunhuan <= 28)
			{
				cRecved = 0x00;
				if (pSerialPort->ReadChar(cRecved) == true)
				{
					int b = cRecved - '0';
					if (-10 < b && b < 10)
					{
						finger_one_joint[3] = finger_one_joint[3] * 10 + b;
					}
					xunhuan++;
				}
			};
			if (xunhuan > 28 && xunhuan <= 35)
			{
				cRecved = 0x00;
				if (pSerialPort->ReadChar(cRecved) == true)
				{
					int b = cRecved - '0';
					if (-10 < b && b < 10)
					{
						finger_one_joint[4] = finger_one_joint[4] * 10 + b;
					}
					xunhuan++;
				}
			};
			if (xunhuan > 35 && xunhuan <= 42)
			{
				cRecved = 0x00;
				if (pSerialPort->ReadChar(cRecved) == true)
				{
					int b = cRecved - '0';
					if (-10 < b && b < 10)
					{
						finger_one_joint[5] = finger_one_joint[5] * 10 + b;
					}
					xunhuan++;
				}
			};

			


			/*
			if (xunhuan > 43 && xunhuan <= 50)
			{
				cRecved = 0x00;
				if (pSerialPort->ReadChar(cRecved) == true)
				{
					int b = cRecved - '0';
					if (-10 < b && b < 10)
					{
						finger_two_joint[0] = finger_two_joint[0] * 10 + b;
					}
					xunhuan++;
				}
			};
			if (xunhuan > 50 && xunhuan <= 57)
			{
				cRecved = 0x00;
				if (pSerialPort->ReadChar(cRecved) == true)
				{
					int b = cRecved - '0';
					if (-10 < b && b < 10)
					{
						finger_two_joint[1] = finger_two_joint[1] * 10 + b;
					}
					xunhuan++;
				}
			};
			if (xunhuan > 57 && xunhuan <= 64)
			{
				cRecved = 0x00;
				if (pSerialPort->ReadChar(cRecved) == true)
				{
					int b = cRecved - '0';
					if (-10 < b && b < 10)
					{
						finger_two_joint[2] = finger_two_joint[2] * 10 + b;
					}
					xunhuan++;
				}
			};
			if (xunhuan > 64 && xunhuan <= 71)
			{
				cRecved = 0x00;
				if (pSerialPort->ReadChar(cRecved) == true)
				{
					int b = cRecved - '0';
					if (-10 < b && b < 10)
					{
						finger_two_joint[3] = finger_two_joint[3] * 10 + b;
					}
					xunhuan++;
				}
			};
			if (xunhuan > 71 && xunhuan <= 78)
			{
				cRecved = 0x00;
				if (pSerialPort->ReadChar(cRecved) == true)
				{
					int b = cRecved - '0';
					if (-10 < b && b < 10)
					{
						finger_two_joint[4] = finger_two_joint[4] * 10 + b;
					}
					xunhuan++;
				}
			};
			if (xunhuan > 78 && xunhuan <= 85)
			{
				cRecved = 0x00;
				if (pSerialPort->ReadChar(cRecved) == true)
				{
					int b = cRecved - '0';
					if (-10 < b && b < 10)
					{
						finger_two_joint[5] = finger_two_joint[5] * 10 + b;
					}
					xunhuan++;
				}
			};
			if (xunhuan > 85 && xunhuan <= 92)
			{
				cRecved = 0x00;
				if (pSerialPort->ReadChar(cRecved) == true)
				{
					int b = cRecved - '0';
					if (-10 < b && b < 10)
					{
						finger_three_joint[0] = finger_three_joint[0] * 10 + b;
					}
					xunhuan++;
				}
			};
			if (xunhuan > 92 && xunhuan <= 99)
			{
				cRecved = 0x00;
				if (pSerialPort->ReadChar(cRecved) == true)
				{
					int b = cRecved - '0';
					if (-10 < b && b < 10)
					{
						finger_three_joint[1] = finger_three_joint[1] * 10 + b;
					}
					xunhuan++;
				}
			};
			if (xunhuan > 99 && xunhuan <= 106)
			{
				cRecved = 0x00;
				if (pSerialPort->ReadChar(cRecved) == true)
				{
					int b = cRecved - '0';
					if (-10 < b && b < 10)
					{
						finger_three_joint[2] = finger_three_joint[2] * 10 + b;
					}
					xunhuan++;
				}
			};
			if (xunhuan > 106 && xunhuan <= 113)
			{
				cRecved = 0x00;
				if (pSerialPort->ReadChar(cRecved) == true)
				{
					int b = cRecved - '0';
					if (-10 < b && b < 10)
					{
						finger_three_joint[3] = finger_three_joint[3] * 10 + b;
					}
					xunhuan++;
				}
			};
			if (xunhuan > 113 && xunhuan <= 120)
			{
				cRecved = 0x00;
				if (pSerialPort->ReadChar(cRecved) == true)
				{
					int b = cRecved - '0';
					if (-10 < b && b < 10)
					{
						finger_three_joint[4] = finger_three_joint[4] * 10 + b;
					}
					xunhuan++;
				}
			};
			if (xunhuan > 120 && xunhuan <= 127)
			{
				cRecved = 0x00;
				if (pSerialPort->ReadChar(cRecved) == true)
				{
					int b = cRecved - '0';
					if (-10 < b && b < 10)
					{
						finger_three_joint[5] = finger_three_joint[5] * 10 + b;
					}
					xunhuan++;
				}
			};
			*/
		
		}while (--BytesInQue);		//


		for (efuzhi = 0; efuzhi < 6; efuzhi++)
		{
			if (finger_one_joint[efuzhi] != 0)
			{
				finger_one[efuzhi] = finger_one_joint[efuzhi];
				finger_one_joint[efuzhi] = 0;
			}
			/*
			if (finger_two_joint[i] != 0)
			{
				finger_two[i] = finger_two_joint[i];
				finger_two_joint[i] = 0;
			}
			if (finger_three_joint[i] != 0)
			{
				finger_three[i] = finger_three_joint[i];
				finger_three_joint[i] = 0;
			}
			*/
		}


		/*
		do
		{
			

	     if(xunhuan<=10)//读取位置
		 {
				cRecved = 0x00;			
				if(pSerialPort->ReadChar(cRecved) == true)
				{
				//	std::cout << cRecved<<std::endl ; 				
					int b=cRecved-'0';
					//std::cout << b<<std::endl ; 
					if(-10<b && b<10)
					{
						a = a*10+b;
					}
				    xunhuan++;
								
				}
		 };
		 if(xunhuan>10 && xunhuan<=17)
		 {
				cRecved = 0x00;			
				if(pSerialPort->ReadChar(cRecved) == true)
				{
						
					int b=cRecved-'0';
				
					if(-10<b && b<10)
					{
						d[0] = d[0]*10+b;
					}
				    xunhuan++;
								
				}
		 };
		 if(xunhuan>17 && xunhuan<=24)
		 {
				cRecved = 0x00;			
				if(pSerialPort->ReadChar(cRecved) == true)
				{
						
					int b=cRecved-'0';
				
					if(-10<b && b<10)
					{
						d[1] = d[1]*10+b;
					}
				    xunhuan++;
								
				}
		 };
		 if(xunhuan>24 && xunhuan<=31)
		 {
				cRecved = 0x00;			
				if(pSerialPort->ReadChar(cRecved) == true)
				{
						
					int b=cRecved-'0';
				
					if(-10<b && b<10)
					{
						d[2] = d[2]*10+b;
					}
				    xunhuan++;
								
				}
		 };
		 if(xunhuan>31 && xunhuan<=38)
		 {
				cRecved = 0x00;			
				if(pSerialPort->ReadChar(cRecved) == true)
				{
						
					int b=cRecved-'0';
				
					if(-10<b && b<10)
					{
						d[3] = d[3]*10+b;
					}
				    xunhuan++;
								
				}
		 };
		 if(xunhuan>38 && xunhuan<=45)
		 {
				cRecved = 0x00;			
				if(pSerialPort->ReadChar(cRecved) == true)
				{
						
					int b=cRecved-'0';
				
					if(-10<b && b<10)
					{
						d[4] = d[4]*10+b;
					}
				    xunhuan++;
								
				}
		 };
		 if(xunhuan>45 && xunhuan<=52)
		 {
				cRecved = 0x00;			
				if(pSerialPort->ReadChar(cRecved) == true)
				{
						
					int b=cRecved-'0';
				
					if(-10<b && b<10)
					{
						d[5] = d[5]*10+b;
					}
				    xunhuan++;
								
				}
		 };
		 if(xunhuan>52 && xunhuan<=59)
		 {
				cRecved = 0x00;			
				if(pSerialPort->ReadChar(cRecved) == true)
				{
						
					int b=cRecved-'0';
				
					if(-10<b && b<10)
					{
						d[6] = d[6]*10+b;
					}
				    xunhuan++;
								
				}
		 };
		 if(xunhuan>59 && xunhuan<=66)
		 {
				cRecved = 0x00;			
				if(pSerialPort->ReadChar(cRecved) == true)
				{
						
					int b=cRecved-'0';
				
					if(-10<b && b<10)
					{
						d[7] = d[7]*10+b;
					}
				    xunhuan++;
								
				}
		 };
		 if(xunhuan>66 && xunhuan<=73)
		 {
				cRecved = 0x00;			
				if(pSerialPort->ReadChar(cRecved) == true)
				{
						
					int b=cRecved-'0';
				
					if(-10<b && b<10)
					{
						d[8] = d[8]*10+b;
					}
				    xunhuan++;
								
				}
		 };	

		}while(--BytesInQue);
	
		c=a;a=0;//赋值POSITION
		for(efuzhi=0;efuzhi<9;efuzhi++)//赋值ROTATION
		{
			if(d[efuzhi] != 0)
			{
			e[efuzhi]=d[efuzhi];
			d[efuzhi]=0;
			}
		}	

		*/
	}

	return 0;
}

INT CSerialPort::ReturnPositions()
{
	
	return c;
}

INT CSerialPort::ReturnRosition0()
{
	
	return e[0];
}
INT CSerialPort::ReturnRosition1()
{
	
	return e[1];
}
INT CSerialPort::ReturnRosition2()
{
	
	return e[2];
}
INT CSerialPort::ReturnRosition3()
{
	
	return e[3];
}
INT CSerialPort::ReturnRosition4()
{
	
	return e[4];
}
INT CSerialPort::ReturnRosition5()
{
	
	return e[5];
}
INT CSerialPort::ReturnRosition6()
{
	
	return e[6];
}
INT CSerialPort::ReturnRosition7()
{
	
	return e[7];
}
INT CSerialPort::ReturnRosition8()
{
	
	return e[8];
}

/*	------------------------------------------
	Created by YuanSu on 2022/10/13

	Get three fingers joint angle (string)
	------------------------------------------ */

void CSerialPort::Get_Finger_One_Joint(int * finger_one_)
{
	finger_one_[0] = finger_one[0];
	finger_one_[1] = finger_one[1];
	finger_one_[2] = finger_one[2];
	finger_one_[3] = finger_one[3];
	finger_one_[4] = finger_one[4];
	finger_one_[5] = finger_one[5];

}
void CSerialPort::Get_Finger_Two_Joint(int * finger_two_)
{
	finger_two_[0] = finger_two[0];
	finger_two_[1] = finger_two[1];
	finger_two_[2] = finger_two[2];
	finger_two_[3] = finger_two[3];
	finger_two_[4] = finger_two[4];
	finger_two_[5] = finger_two[5];
}
void CSerialPort::Get_Finger_Three_Joint(int * finger_three_)
{
	finger_three_[0] = finger_two[0];
	finger_three_[1] = finger_two[1];
	finger_three_[2] = finger_two[2];
	finger_three_[3] = finger_two[3];
	finger_three_[4] = finger_two[4];
	finger_three_[5] = finger_two[5];
}



bool CSerialPort::ReadChar( char &cRecved )
{
	BOOL  bResult     = TRUE;
	DWORD BytesRead   = 0;
	if(m_hComm == INVALID_HANDLE_VALUE)
	{
		return false;
	}

	/** 临界区保护 */ 
	EnterCriticalSection(&m_csCommunicationSync);

	/** 从缓冲区读取一个字节的数据 */ 
	bResult = ReadFile(m_hComm, &cRecved, 1, &BytesRead, NULL);
	//std::cout <<cRecved<<"g"<< std::endl;
	if ((!bResult))
	{ 
		/** 获取错误码,可以根据该错误码查出错误原因 */ 
		DWORD dwError = GetLastError();

		/** 清空串口缓冲区 */ 
		PurgeComm(m_hComm, PURGE_RXCLEAR | PURGE_RXABORT);
		LeaveCriticalSection(&m_csCommunicationSync);

		return false;
	}

	/** 离开临界区 */ 
	LeaveCriticalSection(&m_csCommunicationSync);

	return (BytesRead == 1);

}

bool CSerialPort::WriteData( unsigned char* pData, unsigned int length )
{
	BOOL   bResult     = TRUE;
	DWORD  BytesToSend = 0;
	if(m_hComm == INVALID_HANDLE_VALUE)
	{
		return false;
	}

	/** 临界区保护 */ 
	EnterCriticalSection(&m_csCommunicationSync);

	/** 向缓冲区写入指定量的数据 */ 
	bResult = WriteFile(m_hComm, pData, length, &BytesToSend, NULL);
	if (!bResult)  
	{
		DWORD dwError = GetLastError();
		/** 清空串口缓冲区 */ 
		PurgeComm(m_hComm, PURGE_RXCLEAR | PURGE_RXABORT);
		LeaveCriticalSection(&m_csCommunicationSync);

		return false;
	}

	/** 离开临界区 */ 
	LeaveCriticalSection(&m_csCommunicationSync);

	return true;
}


