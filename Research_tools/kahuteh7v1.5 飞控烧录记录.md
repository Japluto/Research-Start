## 问题描述
---
在烧录kahute-h7 v1.5这款飞控时，出现反复进入Bootloader的情况，表证为：
> 只有变成dfu才能在设备管理器找到，但是变成dfu不会弹弹窗(qgc里的刷固件弹窗)
> 不摁boot变dfu可以在qgc里面找到并且弹窗刷固件，但是刷完侧边栏没有更新，没有显示包括机架选型等，而且此时设备管理器里面找不到stm32设备

---
主要是kahute的bootloader出厂烧录的是betaflight的固件，所以需要先用stm32programmer连接USB，找到对应dfu（摁住boot键接上电脑，然后2s后拔掉，此时不会亮强蓝光），擦除原有flash的内容，然后烧录[这个链接](https://docs.holybro.com/autopilot/kakute-h7-v1-v2-mini/kakute-h7-v1/supported-firmware)里的对应版本px4的bootloader 
**(应该是个.hex文件，这个可以直接烧录，会自动识别烧录的地址；如果是.bin文件:STM32CubeProgrammer 的起始地址必须填 0x08000000)** 

---
烧录好对应的px4的bootloader之后，接下来要做的就是不用进dfu，直接启动qgc firmware，然后刷固件校准就行了。