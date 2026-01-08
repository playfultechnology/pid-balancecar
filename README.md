# pid-balancecar
Code, usage notes for operation of an STM32-controlled "self-balancing car", as shown below:

<img src="images/balancecar.jpg"/>

There are several similar models available from different suppliers on AliExpress, BangGood etc.

## Source Code
The "open source" code was supplied by the vendor as a baidu link. 
Link: https://pan.baidu.com/s/1M-D3hrumQRwJgep1K78iIw   
Password: bx5w
The problem is you can't download from baidu without a baidu account, and you can't create a baidu account without supplying a Chinese phone number. 
I previously made use of a workaround (https://oneleaf.icu/), but that no longer seems active, so I have uploaded the original supplied code to the download/ folder of this repository.

The code/ directory contains several versions of the project:
 - [Main](https://github.com/playfultechnology/pid-balancecar/tree/main/code/MiniBalanceV5.0%E3%80%90%E9%AB%98%E9%85%8D%E7%89%88%E3%80%91%E5%B9%B3%E8%A1%A1%E5%B0%8F%E8%BD%A6PID%E5%AD%A6%E4%B9%A0%E6%BA%90%E7%A0%81) 
 - [DMP Version](https://github.com/playfultechnology/pid-balancecar/tree/main/code/MiniBalanceV5.0%E3%80%90%E9%AB%98%E9%85%8D%E7%89%88%E3%80%91%E5%B9%B3%E8%A1%A1%E5%B0%8F%E8%BD%A6%E5%BA%93%E5%87%BD%E6%95%B0%E7%89%88%E6%BA%90%E7%A0%81%EF%BC%88DMP%E7%89%88%EF%BC%89)
 - [Complentary Filter Version](https://github.com/playfultechnology/pid-balancecar/tree/main/code/MiniBalanceV5.0%E3%80%90%E9%AB%98%E9%85%8D%E7%89%88%E3%80%91%E5%B9%B3%E8%A1%A1%E5%B0%8F%E8%BD%A6%E5%BA%93%E5%87%BD%E6%95%B0%E7%89%88%E6%BA%90%E7%A0%81%EF%BC%88%E4%BA%92%E8%A1%A5%E6%BB%A4%E6%B3%A2%E7%89%88%EF%BC%89)
 - [Kalman Filter Version](https://github.com/playfultechnology/pid-balancecar/tree/main/code/MiniBalanceV5.0%E3%80%90%E9%AB%98%E9%85%8D%E7%89%88%E3%80%91%E5%B9%B3%E8%A1%A1%E5%B0%8F%E8%BD%A6%E5%BA%93%E5%87%BD%E6%95%B0%E7%89%88%E6%BA%90%E7%A0%81%EF%BC%88%E5%8D%A1%E5%B0%94%E6%9B%BC%E6%BB%A4%E6%B3%A2%E7%89%88%EF%BC%89)
 - [Simplified Beginner's Version](https://github.com/playfultechnology/pid-balancecar/tree/main/code/MiniBalanceV5.0%E3%80%90%E9%AB%98%E9%85%8D%E7%89%88%E3%80%91%E5%B9%B3%E8%A1%A1%E5%B0%8F%E8%BD%A6%E5%BA%93%E5%87%BD%E6%95%B0%E7%89%88%E6%BA%90%E7%A0%81%EF%BC%88%E7%B2%BE%E7%AE%80%E5%85%A5%E9%97%A8%E7%89%88%EF%BC%89)
 - [Arduino Version](https://github.com/playfultechnology/pid-balancecar/tree/main/code/%E6%8B%93%E5%B1%95%E6%BA%90%E7%A0%81/MinibalancV5.0%20%E3%80%90%E9%AB%98%E9%85%8D%E7%89%88%E3%80%91Arduino%20nano%E5%B9%B3%E8%A1%A1%E5%B0%8F%E8%BD%A6%E6%BA%90%E7%A0%81/Minibalance_for_Arduino_Final)

## Operation
- Long press on USER button: toggle operation mode
  - "putong" (普通) = "ordinary" (i.e. static balance)
  - "bizhang" (蔽障) = "obstacle" (i.e. moves to avoid obstacles placed in front of ultrasonic sensor)
 - Short press on USER button: start/stop balancing

# Connecting via BlueTooth
 - Pair with the BT04-A module (passcode 1234) to be able to send/receive data via BT Classic serial interface

 
## Hardware
- 2x [MG513P3012V](https://www.aliexpress.com/item/4000996252848.html) Motors, 30:1 gear ratio
<img src="images/motor_spec.jpg"/>

## Adding the CCD Line-Following Module
- Compile the Keil uVision project with CCD support contained in pid-balancecar\download\CCD Attachment\TSL1401线性CCD模块附送资料V3.1-202120217\2.平衡小车专用CCD巡线代码\2.STM32平衡小车 CCD巡线\CCD巡线代码【高配版平衡小车STM32F405RGT6】
- If required, install support for the STM32F405RGT6 board
- To upload the code (without using the STLink dongle), load the FlyMCU application.
- Set the COM port and bps:76800, then hit the big (P) button to program the board.
