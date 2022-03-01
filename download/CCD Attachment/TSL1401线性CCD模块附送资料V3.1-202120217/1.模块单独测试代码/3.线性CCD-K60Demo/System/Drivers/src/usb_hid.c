#include "usb_hid.h"
#include "usb.h"
#include "string.h"
/***********************************************************************************************
//CH_Kinetis������  V2.3   
//����    :YANDLD 
//E-MAIL  :yandld@126.com
//�޸�����:2013/2/14
//�汾��V2.3
//�Ա���http://upcmcu.taobao.com
//QQ    1453363089
//Copyright(C) YANDLD 2012-2022
//All rights reserved
************************************************************************************************/
//HID���ݻ���
uint8_t USB_HID_RecFlag = 0;  //���ձ�־
uint8_t USB_HID_SendBuffer[32];
uint8_t USB_HID_SendLen = 0;
static  MessageType_t* pMsg;         /* ��Ϣָ�� */
/***********************************************************************************************
 ���ܣ�HID ��������
 �βΣ�buf:�û�����  ���͹��ĳ���        
 ���أ�0
 ��⣺�û�����
************************************************************************************************/
void USB_HID_SendData(uint8_t* buf,uint8_t len)
{
	memcpy(USB_HID_SendBuffer,buf,len);
	USB_HID_SendLen = len;
}

/***********************************************************************************************
 ���ܣ�USB��귢������
 �βΣ�0          
 ���أ�0
 ��⣺��Ҫ��Ӧ�޸Ķ�Ӧ������������װUSB_HID_SendData ����
************************************************************************************************/

void USB_HID_SetMouse(uint8_t HOffset,uint8_t VOffset,uint8_t SOffset,uint8_t Key)
{
	uint8_t buf[4];
	buf[1] = HOffset;
	buf[2] = VOffset;
	buf[3] = SOffset;
	buf[0] = Key;
	USB_HID_SendData(buf,4);
}

//ͨ������ı����������Ķ��壬����֪�����ص����뱨�����8�ֽڡ�
//��һ�ֽڵ�8��bit������ʾ������Ƿ��£�����Shift��Alt�ȼ�����
//�ڶ��ֽ�Ϊ����ֵ��ֵΪ����0���������ڰ��ֽ���һ����ͨ����ֵ��
//���飬��û�м�����ʱ��ȫ��6���ֽ�ֵ��Ϊ0����ֻ��һ����ͨ������ʱ��
//�������ֽ��еĵ�һ�ֽ�ֵ��Ϊ�ð����ļ�ֵ������ļ�ֵ�뿴HID��
//��;���ĵ��������ж����ͨ��ͬʱ����ʱ����ͬʱ������Щ���ļ�ֵ��
//������µļ�̫�࣬���������ֽڶ�Ϊ0xFF�����ܷ���0x00����������
//����ϵͳ��Ϊ���м����Ѿ��ͷţ������ڼ�ֵ�������е��Ⱥ�˳����
//����ν�ģ�����ϵͳ�Ḻ�����Ƿ����¼����¡�����Ӧ�����ж϶˵�1
//�а�������ĸ�ʽ����ʵ�ʵļ������ݡ����⣬�����л�������һ���ֽ�
//��������棬����������LED����ġ�ֻʹ���˵�7λ����1λ�Ǳ���ֵ0��
//��ĳλ��ֵΪ1ʱ�����ʾ��Ӧ��LEDҪ����������ϵͳ�Ḻ��ͬ������
//����֮���LED����������������̣�һ������ּ��̵���ʱ����һ��
//Ҳ������������̱�����Ҫ�жϸ���LEDӦ�ú�ʱ������ֻ�ǵȴ�����
//���ͱ��������Ȼ����ݱ���ֵ��������Ӧ��LED�������ڶ˵�1����ж�
//�ж�����1�ֽڵ�������棬Ȼ�����ȡ������Ϊѧϰ���ϵ�LED�ǵ͵�ƽʱ
//������ֱ�ӷ��͵�LED�ϡ�����main�����а�������LED�Ĵ���Ͳ���Ҫ�ˡ�
//�������ݷ��ͺ���

//USBHID ���ü�ֵ �����ðٶ���
/*
��HID�ĵ���ժ�����ģ�����Ҳο�һ��
0 00 Reserved (no event indicated)9 N/A �� �� �� 4/101/104
1 01 Keyboard ErrorRollOver9 N/A �� �� �� 4/101/104
2 02 Keyboard POSTFail9 N/A �� �� �� 4/101/104
3 03 Keyboard ErrorUndefined9 N/A �� �� �� 4/101/104
4 04 Keyboard a and A4 31 �� �� �� 4/101/104
5 05 Keyboard b and B 50 �� �� �� 4/101/104
6 06 Keyboard c and C4 48 �� �� �� 4/101/104
7 07 Keyboard d and D 33 �� �� �� 4/101/104
8 08 Keyboard e and E 19 �� �� �� 4/101/104
9 09 Keyboard f and F 34 �� �� �� 4/101/104
10 0A Keyboard g and G 35 �� �� �� 4/101/104
11 0B Keyboard h and H 36 �� �� �� 4/101/104
12 0C Keyboard i and I 24 �� �� �� 4/101/104
13 0D Keyboard j and J 37 �� �� �� 4/101/104
14 0E Keyboard k and K 38 �� �� �� 4/101/104
15 0F Keyboard l and L 39 �� �� �� 4/101/104
16 10 Keyboard m and M4 52 �� �� �� 4/101/104
17 11 Keyboard n and N 51 �� �� �� 4/101/104
18 12 Keyboard o and O4 25 �� �� �� 4/101/104
19 13 Keyboard p and P4 26 �� �� �� 4/101/104
20 14 Keyboard q and Q4 17 �� �� �� 4/101/104
21 15 Keyboard r and R 20 �� �� �� 4/101/104
22 16 Keyboard s and S4 32 �� �� �� 4/101/104
23 17 Keyboard t and T 21 �� �� �� 4/101/104
24 18 Keyboard u and U 23 �� �� �� 4/101/104
25 19 Keyboard v and V 49 �� �� �� 4/101/104
26 1A Keyboard w and W4 18 �� �� �� 4/101/104
27 1B Keyboard x and X4 47 �� �� �� 4/101/104
28 1C Keyboard y and Y4 22 �� �� �� 4/101/104
29 1D Keyboard z and Z4 46 �� �� �� 4/101/104
30 1E Keyboard 1 and !4 2 �� �� �� 4/101/104
31 1F Keyboard 2 and @4 3 �� �� �� 4/101/104
32 20 Keyboard 3 and #4 4 �� �� �� 4/101/104
33 21 Keyboard 4 and $4 5 �� �� �� 4/101/104
34 22 Keyboard 5 and %4 6 �� �� �� 4/101/104
35 23 Keyboard 6 and ^4 7 �� �� �� 4/101/104
36 24 Keyboard 7 and &4 8 �� �� �� 4/101/104
37 25 Keyboard 8 and *4 9 �� �� �� 4/101/104
38 26 Keyboard 9 and (4 10 �� �� �� 4/101/104
39 27 Keyboard 0 and )4 11 �� �� �� 4/101/104

*/
//����KeyBoard����
//Fn Key�����⹦�ܼ��� ���λ1Ctrl ����ֱ�Alt Tab
//�����Ǽ��̻������� ÿ��ֻ��6������ͬʱ����
uint8_t USB_HID_SetKeyBoard(uint8_t FnKey,uint8_t *Keybuf)
{
	uint8_t i;
	if(Keybuf == NULL) return 1;
	USB_HID_SendBuffer[0] = FnKey;
	USB_HID_SendLen = 8;
	for(i=2;i<8;i++) 
	{
		USB_HID_SendBuffer[i] = Keybuf[i-2]; 
	}
}

/***********************************************************************************************
 ���ܣ�HID �豸������
 �βΣ�0          
 ���أ�0
 ��⣺��Ҫ�û��� �����в��ϵĵ���
************************************************************************************************/
void HID_Proc(void)
{
	uint8_t i;
	pMsg = NULL;
	if(fn_msg_exist()) //���������Ϣ
	{
			pMsg = fn_msg_pop();
			if(pMsg->m_Command == USB_DEVICE_CLASS_HID)
			{
				if(pMsg->m_MessageType == fIN) //��Ҫ��������
				{
					USB_EP_IN_Transfer(EP2,USB_HID_SendBuffer,USB_HID_SendLen); //��������
					memset(USB_HID_SendBuffer,0,USB_HID_SendLen);  //�������
				}
				else if(pMsg->m_MessageType == fOUT) //���յ�������
				{
					 USB_HID_RecFlag = 1;
				}
			}
	}
}
/***********************************************************************************************
 ���ܣ�USB_HID_���ݽ��պ���
 �βΣ�buf: ���ڽ������ݵĻ�����     
 ���أ����յ������ݳ���
 ��⣺�û��ӿ�
************************************************************************************************/
uint8_t USB_HID_RecData(uint8_t* buf)
{
	if(USB_HID_RecFlag == 1)
	{
		memcpy(buf,pMsg->pMessage,pMsg->m_MsgLen); //��������
		USB_HID_RecFlag = 0;
		return pMsg->m_MsgLen;
	}
	else return 0;
}
