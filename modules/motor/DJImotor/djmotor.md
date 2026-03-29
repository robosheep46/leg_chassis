# djmotor

## 拨码开关设置
### GM6020
![id设置](image.png)

![指示灯](image-4.png)

### M2006
![id设置](image-7.png)

![指示灯状态](image-6.png)

### M3508
![id设置](image-1.png)

![指示灯状态](image-3.png) 

## CAN通信协议
### GM6020 
![控制报文](image-10.png)
![反馈报文](image-11.png)
控制(tx_id):0x1ff,0x2ff
反馈(rx_id):0x204+id 

### M2006/M3508
![控制报文](image-8.png)
![反馈报文](image-9.png)
控制(tx_id):0x1ff,0x200;
反馈(rx_id):0x200+id