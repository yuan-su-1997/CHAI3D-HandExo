//#include "StdAfx.h"
#include "SerialPort.h"
#include <process.h>
#include <iostream>

/** �߳��˳���־ */ 
bool CSerialPort::s_bExit = false;
/** ������������ʱ,sleep���´β�ѯ�����ʱ��,��λ:�� */ 
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

	/** ��ʱ����,���ƶ�����ת��Ϊ�ַ�����ʽ,�Թ���DCB�ṹ */ 
	char szDCBparam[50];
	sprintf_s(szDCBparam, "baud=%d parity=%c data=%d stop=%d", baud, parity, databits, stopsbits);

	/** ��ָ������,�ú����ڲ��Ѿ����ٽ�������,�����벻Ҫ�ӱ��� */ 
	if (!openPort(portNo))
	{
		return false;
	}

	/** �����ٽ�� */ 
	EnterCriticalSection(&m_csCommunicationSync);

	/** �Ƿ��д����� */ 
	BOOL bIsSuccess = TRUE;

    /** �ڴ˿���������������Ļ�������С,���������,��ϵͳ������Ĭ��ֵ.
	 *  �Լ����û�������Сʱ,Ҫע�������Դ�һЩ,���⻺�������
	 */
	if (bIsSuccess )
	{
		bIsSuccess = SetupComm(m_hComm,8,8);
	}

	/** ���ô��ڵĳ�ʱʱ��,����Ϊ0,��ʾ��ʹ�ó�ʱ���� */
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
		// ��ANSI�ַ���ת��ΪUNICODE�ַ���
		DWORD dwNum = MultiByteToWideChar (CP_ACP, 0, szDCBparam, -1, NULL, 0);
		wchar_t *pwText = new wchar_t[dwNum] ;
		if (!MultiByteToWideChar (CP_ACP, 0, szDCBparam, -1, pwText, dwNum))
		{
			bIsSuccess = TRUE;
		}

		/** ��ȡ��ǰ�������ò���,���ҹ��촮��DCB���� */ 
		bIsSuccess = GetCommState(m_hComm, &dcb) && BuildCommDCB(pwText, &dcb) ;
		/** ����RTS flow���� */ 
		dcb.fRtsControl = RTS_CONTROL_ENABLE; 

		/** �ͷ��ڴ�ռ� */ 
		delete [] pwText;
	}

	if ( bIsSuccess )
	{
		/** ʹ��DCB�������ô���״̬ */ 
		bIsSuccess = SetCommState(m_hComm, &dcb);
	}
		
	/**  ��մ��ڻ����� */
	PurgeComm(m_hComm, PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_TXABORT);

	/** �뿪�ٽ�� */ 
	LeaveCriticalSection(&m_csCommunicationSync);

	return bIsSuccess==TRUE;
}

bool CSerialPort::InitPort( UINT portNo ,const LPDCB& plDCB )
{
	/** ��ָ������,�ú����ڲ��Ѿ����ٽ�������,�����벻Ҫ�ӱ��� */ 
	if (!openPort(portNo))
	{
		return false;
	}
	
	/** �����ٽ�� */ 
	EnterCriticalSection(&m_csCommunicationSync);

	/** ���ô��ڲ��� */ 
	if (!SetCommState(m_hComm, plDCB))
	{
		return false;
	}

	/**  ��մ��ڻ����� */
	PurgeComm(m_hComm, PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_TXABORT);

	/** �뿪�ٽ�� */ 
	LeaveCriticalSection(&m_csCommunicationSync);

	return true;
}

void CSerialPort::ClosePort()
{
	/** ����д��ڱ��򿪣��ر��� */
	if( m_hComm != INVALID_HANDLE_VALUE )
	{
		CloseHandle( m_hComm );
		m_hComm = INVALID_HANDLE_VALUE;
	}
}

bool CSerialPort::openPort( UINT portNo )
{
	/** �����ٽ�� */ 
	EnterCriticalSection(&m_csCommunicationSync);

	/** �Ѵ��ڵı��ת��Ϊ�豸�� */ 
    char szPort[50];
	sprintf_s(szPort, "COM%d", portNo);

	/** ��ָ���Ĵ��� */ 
	m_hComm = CreateFileA(szPort,		                /** �豸��,COM1,COM2�� */ 
						 GENERIC_READ | GENERIC_WRITE,  /** ����ģʽ,��ͬʱ��д */   
						 0,                             /** ����ģʽ,0��ʾ������ */ 
					     NULL,							/** ��ȫ������,һ��ʹ��NULL */ 
					     OPEN_EXISTING,					/** �ò�����ʾ�豸�������,���򴴽�ʧ�� */ 
						 0,    
						 0);    

	/** �����ʧ�ܣ��ͷ���Դ������ */ 
	if (m_hComm == INVALID_HANDLE_VALUE)
	{
		LeaveCriticalSection(&m_csCommunicationSync);
		return false;
	}

	/** �˳��ٽ��� */ 
	LeaveCriticalSection(&m_csCommunicationSync);

	return true;
}

bool CSerialPort::OpenListenThread()
{
	/** ����߳��Ƿ��Ѿ������� */ 
	if (m_hListenThread != INVALID_HANDLE_VALUE)
	{
		/** �߳��Ѿ����� */ 
		return false;
	}

	s_bExit = false;
	/** �߳�ID */ 
	UINT threadId;
	/** �����������ݼ����߳� */ 
	m_hListenThread = (HANDLE)_beginthreadex(NULL, 0, ListenThread, this, 0, &threadId);
	if (!m_hListenThread)
	{
		return false;
	}
	/** �����̵߳����ȼ�,������ͨ�߳� */ 
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
		/** ֪ͨ�߳��˳� */ 
		s_bExit = true;

		/** �ȴ��߳��˳� */ 
		Sleep(10);

		/** ���߳̾����Ч */ 
		CloseHandle( m_hListenThread );
		m_hListenThread = INVALID_HANDLE_VALUE;
	}  
	return true;
}

UINT CSerialPort::GetBytesInCOM()
{

	DWORD dwError = 0;	/** ������ */ 
	COMSTAT  comstat;   /** COMSTAT�ṹ��,��¼ͨ���豸��״̬��Ϣ */ 
	//byte buf[];
	//int Test_Data_X;
	//int Test_Data_Y;
	//int Test_Data_Z;
	memset(&comstat, 0, sizeof(COMSTAT));//��ֵ֮ǰ��ʼ������ָ�������ָ���ǰ n �ֽڵ��ڴ浥Ԫ��һ�����������滻

	UINT BytesInQue = 0;

	DWORD BytesRead = 0;

	
	/** �ڵ���ReadFile��WriteFile֮ǰ,ͨ�������������ǰ�����Ĵ����־ */ 
	if ( ClearCommError(m_hComm, &dwError, &comstat) )
	{
		BytesInQue = comstat.cbInQue; /** ��ȡ�����뻺�����е��ֽ��� */ 
	}
		
	if ( BytesInQue == 0 )
		{
			Sleep(SLEEP_TIME_INTERVAL);
		}

		///** ��ȡ���뻺�����е����ݲ������ʾ */
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
	/** �õ������ָ�� */ 
	CSerialPort *pSerialPort = reinterpret_cast<CSerialPort*>(pParam);

	// �߳�ѭ��,��ѯ��ʽ��ȡ��������
	while (!pSerialPort->s_bExit) 
	{
		UINT BytesInQue = pSerialPort->GetBytesInCOM();
		/** ����������뻺������������,����Ϣһ���ٲ�ѯ */ 
		if ( BytesInQue == 0 )
		{
			Sleep(SLEEP_TIME_INTERVAL);
			continue;
		}



		/** ��ȡ���뻺�����е����ݲ������ʾ */
		int xunhuan = 1;
		char cRecved = 0x00;
		//strtod(cRecved,NULL);

		do {
		
			if (xunhuan <= 7)//��ȡλ��
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
			

	     if(xunhuan<=10)//��ȡλ��
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
	
		c=a;a=0;//��ֵPOSITION
		for(efuzhi=0;efuzhi<9;efuzhi++)//��ֵROTATION
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

	/** �ٽ������� */ 
	EnterCriticalSection(&m_csCommunicationSync);

	/** �ӻ�������ȡһ���ֽڵ����� */ 
	bResult = ReadFile(m_hComm, &cRecved, 1, &BytesRead, NULL);
	//std::cout <<cRecved<<"g"<< std::endl;
	if ((!bResult))
	{ 
		/** ��ȡ������,���Ը��ݸô�����������ԭ�� */ 
		DWORD dwError = GetLastError();

		/** ��մ��ڻ����� */ 
		PurgeComm(m_hComm, PURGE_RXCLEAR | PURGE_RXABORT);
		LeaveCriticalSection(&m_csCommunicationSync);

		return false;
	}

	/** �뿪�ٽ��� */ 
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

	/** �ٽ������� */ 
	EnterCriticalSection(&m_csCommunicationSync);

	/** �򻺳���д��ָ���������� */ 
	bResult = WriteFile(m_hComm, pData, length, &BytesToSend, NULL);
	if (!bResult)  
	{
		DWORD dwError = GetLastError();
		/** ��մ��ڻ����� */ 
		PurgeComm(m_hComm, PURGE_RXCLEAR | PURGE_RXABORT);
		LeaveCriticalSection(&m_csCommunicationSync);

		return false;
	}

	/** �뿪�ٽ��� */ 
	LeaveCriticalSection(&m_csCommunicationSync);

	return true;
}


