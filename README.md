# My_4010_BLDC_DRiver
ESP32_WIFI_Module+DRV9313+AS5600+USB_Typec+CAN, fit with 4010 BLDC Motor

经测试，功能正常， ESP32通过 RS232 接口通讯，功能正常，需修改simplefoc library , disable com port echo. CAN通讯时间关系，还没有测试。

To be otpimized: 
1)Typec USB 接口插入又方向性，应是随意方向都能使用 
2)增加Power Mosfet,适合大流量应用 
3)去掉wifi功能，适合单面板，降低成本

appearance icon
