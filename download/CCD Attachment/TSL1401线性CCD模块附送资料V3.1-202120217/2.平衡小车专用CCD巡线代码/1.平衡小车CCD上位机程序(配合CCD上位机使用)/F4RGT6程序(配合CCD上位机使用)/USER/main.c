/***********************************************
公司：轮趣科技（东莞）有限公司
品牌：WHEELTEC
官网：wheeltec.net
淘宝店铺：shop114407458.taobao.com 
速卖通: https://minibalance.aliexpress.com/store/4455017
版本：V1.0
修改时间：2021-12-3

Company: WHEELTEC Co.Ltd
Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com 
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: V1.0
Update：2021-12-3

All rights reserved
***********************************************/
#include "sys.h"
#include "system.h"

int main(void)
{ 
	systemInit();					//Initialization function     初始化函数
	while(1)
		{ 
      sendToPc();   //发送信息至上位机			
		}

}





