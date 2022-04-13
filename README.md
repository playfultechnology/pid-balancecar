# pid-balancecar
Code, usage notes for operation of an STM32-controlled "self-balancing car", as shown below:

<img src="images/balancecar.jpg"/>

There are several similar models available from different suppliers on AliExpress, BangGood etc.

## Operation
- Long press on USER button: toggle operation mode
  - "putong" (普通) = "ordinary" (i.e. static balance)
  - "bizhang" (蔽障) = "obstacle" (i.e. moves to avoid obstacles placed in front of ultrasonic sensor)
 - Short press on USER button: start/stop balancing
 
## Hardware
- 2x [MG513P3012V](https://www.aliexpress.com/item/4000996252848.html) Motors, 30:1 gear ratio
<img src="images/motor_spec.jpg"/>

## Adding the CCD Line-Following Module
- Compile the Keil uVision project with CCD support contained in pid-balancecar\download\CCD Attachment\TSL1401线性CCD模块附送资料V3.1-202120217\2.平衡小车专用CCD巡线代码\2.STM32平衡小车 CCD巡线\CCD巡线代码【高配版平衡小车STM32F405RGT6】
- If required, install support for the STM32F405RGT6 board
- To upload the code (without using the STLink dongle), load the FlyMCU application.
- Set the COM port and bps:76800, then hit the big (P) button to program the board.
