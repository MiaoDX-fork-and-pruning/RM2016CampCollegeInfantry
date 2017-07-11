This is a fork of [mobangjack/RM2016CampCollegeInfantry](https://github.com/mobangjack/RM2016CampCollegeInfantry), the original project is for the control board STM32F405RGT6, and this project is porting for new board STM32F427IIH6.

The main changes are:

# General change

This part is for porting project via (any) different boards. And please follow the [document on robomaster forum](http://bbs.robomasters.com/thread-4177-1-1.html).

The System Clock part is more than crucial, so make sure these variables are changed accordingly:

* HSE Frequency: 25000000 -> 12000000
* PLL_M: 25 -> 12
* PLL_N: 336 -> 360 (At first, I did not change this, but the CAN bus just can not receive any thing)

The `Device` part, we choose `STM32F427II` -> `STM32F427IIHx` since we are using STM32F427IIH6.

In the `Options for target (魔术棒)` -> `Utilities` -> `Settings` -> `Flash Download`, we check `Reset and Run` to make sure our project can be running after the download (no restart needed).

## Possible errors

* ..\lib\stm32f4xx.h(246): error:  #5: cannot open source input file "core_cm4.h": No such file or directory

Just include the `CMSIS` in the installation folder.


# Board Specific

## LED

``` cpp
/*----GREEN LED----PA6-----'0' is on,'1' is off */
/*----RED LED----PA7-----'0' is on,'1' is off */
```

## USART3

``` vi
# OLD
-----USART3_TX-----PB10-----
-----USART3_RX-----PB11-----

# NEW
-----USART3_TX-----PD8-----
-----USART3_RX-----PD9-----                                     
```

The usart with function `int fputc(int ch, FILE *f)` will be treated as `STOUT`.

## CAN1 BUS

``` vi
# OLD
----CAN1_TX-----PA12----
----CAN1_RX-----PA11----

# NEW
----CAN1_TX-----PD1----
----CAN1_RX-----PD0----
```