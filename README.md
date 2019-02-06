#EnvironmentMonitor
基于STM32的PM,CO2,环境温湿度监测器.使用STM32CubeMX+Keil uvision4开发  
部分代码参考自网络  
##接口说明  
温湿度传感器(DHT22/AM2303) -> PA1  
PM2.5(G7 PMS7003) -> USART2(PA3 PA4)
CO2(DS-CO2-20) -> USART3(PE14 PE15)  
SD -> SDIO
##功能说明  
每隔一分钟检测一次环境参数,并存储入SD卡中.  
##存储文件格式:  
文件名:  
Sensor_yyyy_mm_dd_hh_mm.txt  
文件格式:  
[yyyy-mm-dd hh:mm:ss]  
Temp : _%d.%d_ degree Celsius  
RH : _%d.%d_ %  
CO2 : _%4d_ ppm  
PM2.5 : _%4d_ ug/m3  
PM10 : _%4d_ ug/m3  

