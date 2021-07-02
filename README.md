



# 脑电放大器说明文档

version：V1.0.0         author：lyh



## 准备

- windows系统电脑一台

- 下载并安装[CH340驱动](http://www.wch.cn/downloads/file/65.html?time=2020-10-26%2015:34:02&code=aVN9CyCuR4TqbKSTNDEmppEj6408df6NMeNtbwHk)

- 下载[Openbci_GUI V4.2.0](https://github.com/OpenBCI/OpenBCI_GUI/releases/tag/v4.2.0)，更高版本目前还无法支持

- 放大器硬件接口说明

  1. **SWD程序下载接口**：烧录固件使用
  2. **电源接口**：电源输出，测试使用.`AVDD`,`AVSS`分别为`ads1299`正负输入电源，为`+-2.5V`
  3. **电极接口**：`INxP`,`INxN`(x=1,2,...,8)分别为8个通道的正负输入，`REF`为参考电极输入，`BIAS`为右腿驱动输入
  4. **电源开关**：按下为OFF，**弹起**为**ON**
  5. **`HC05`蓝牙接口**：若使用`HC05`蓝牙，将7中跳线帽选择`HC05`，同时蓝牙模块插入时应注意方向，模块朝板内，也可根据模块上的丝印匹配来插入
  6. **电池接口(背面)**：3.7V锂电池输入，**请勿反接或短路**
  7. **蓝牙选择**：利用跳线帽选择蓝牙为`nRF52832`or`HC05`
  8. **通信线路跳线帽**：连接`ads1299`和`stm32f407`,取下跳线帽则对应通信断开
  9. **充电指示灯**：充电时亮红灯，充满亮绿灯
  10. **LED显示**：放大器上丝印有误，从上往下分别为`PWR`,`LED1`,`LED0`，**电源ON**时`PWR`点亮
  11. **`MicroUSB`接口**：供电以及对电池充电，还可作为有线串口使用
  12. **按键**：`RESET`控制程序复位，`KEY1`,`KEY2`为自定义按键，编程使用

  <img src="C:\Users\Huiever\AppData\Roaming\Typora\typora-user-images\image-20201026165031114.png" alt="image-20201026165031114"  />

## 开始

1. 打开`Openbci_GUI`，将microUSB线一端插入电脑，另一端接入放大器

2. 对脑电电极涂上导电膏，固定在头部，另一端根据需要接在放大器的指定位置.采用单极导联的时候，参考电极接`REF`，其余电极接`INxP`.双极导联的时候`INxP`,`INxN`交替连接.程序默认为单极导联模式

3. 打开放大器的电源开关

4. 在`Openbci_GUI`界面，依次点选`LIVE(from Cyton)`,`Serial(from Dongle)`，最后点击`AUTO_CONNECT`等待数秒钟即可启动放大器并进入波形显示界面

   <img src="C:\Users\Huiever\AppData\Roaming\Typora\typora-user-images\image-20201026160246735.png" alt="image-20201026160246735"  />
   
   <img src="C:\Users\Huiever\AppData\Roaming\Typora\typora-user-images\image-20201026164251574.png" alt="image-20201026164251574"  />
