EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:stm32_theapi
LIBS:project-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "LCD Reader"
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L STM32L031F4Px U1
U 1 1 5A9FF0B9
P 5675 3925
F 0 "U1" H 5325 3025 50  0000 L BNN
F 1 "STM32L031F4Px" H 6325 4875 50  0000 R BNN
F 2 "Housings_SSOP:TSSOP-20_4.4x6.5mm_Pitch0.65mm" H 6675 2975 50  0001 R TNN
F 3 "" H 5675 3925 50  0001 C CNN
	1    5675 3925
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X05 J1
U 1 1 5A9FF0BA
P 2825 3625
F 0 "J1" H 2825 3925 50  0000 C CNN
F 1 "SWD" V 2925 3625 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x05_Pitch2.54mm" H 2825 3625 50  0001 C CNN
F 3 "" H 2825 3625 50  0001 C CNN
	1    2825 3625
	-1   0    0    1   
$EndComp
$Comp
L R R1
U 1 1 5A9FF0BB
P 4950 3775
F 0 "R1" V 5030 3775 50  0000 C CNN
F 1 "10K" V 4950 3775 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 4880 3775 50  0001 C CNN
F 3 "" H 4950 3775 50  0001 C CNN
	1    4950 3775
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR13
U 1 1 5A9FF0BC
P 4950 3925
F 0 "#PWR13" H 4950 3675 50  0001 C CNN
F 1 "GND" H 4950 3775 50  0000 C CNN
F 2 "" H 4950 3925 50  0001 C CNN
F 3 "" H 4950 3925 50  0001 C CNN
	1    4950 3925
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR16
U 1 1 5A9FF0BD
P 5675 4825
F 0 "#PWR16" H 5675 4575 50  0001 C CNN
F 1 "GND" H 5675 4675 50  0000 C CNN
F 2 "" H 5675 4825 50  0001 C CNN
F 3 "" H 5675 4825 50  0001 C CNN
	1    5675 4825
	1    0    0    -1  
$EndComp
Text GLabel 3025 3525 2    60   Input ~ 0
SWCLK
Text GLabel 3025 3725 2    60   Input ~ 0
SWDIO
Text GLabel 3025 3825 2    60   Input ~ 0
NRST
Text GLabel 6175 4525 2    60   Input ~ 0
SWCLK
Text GLabel 6175 4425 2    60   Input ~ 0
SWDIO
$Comp
L +BATT #PWR4
U 1 1 5A9FF0BE
P 5625 2325
F 0 "#PWR4" H 5625 2175 50  0001 C CNN
F 1 "+BATT" H 5625 2465 50  0000 C CNN
F 2 "" H 5625 2325 50  0001 C CNN
F 3 "" H 5625 2325 50  0001 C CNN
	1    5625 2325
	1    0    0    -1  
$EndComp
$Comp
L +BATT #PWR10
U 1 1 5A9FF0BF
P 3025 3425
F 0 "#PWR10" H 3025 3275 50  0001 C CNN
F 1 "+BATT" H 3025 3565 50  0000 C CNN
F 2 "" H 3025 3425 50  0001 C CNN
F 3 "" H 3025 3425 50  0001 C CNN
	1    3025 3425
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR11
U 1 1 5A9FF0C0
P 3525 3625
F 0 "#PWR11" H 3525 3375 50  0001 C CNN
F 1 "GND" H 3525 3475 50  0000 C CNN
F 2 "" H 3525 3625 50  0001 C CNN
F 3 "" H 3525 3625 50  0001 C CNN
	1    3525 3625
	1    0    0    -1  
$EndComp
$Comp
L Battery_Cell BT1
U 1 1 5A9FF0C1
P 4050 1575
F 0 "BT1" H 4150 1675 50  0000 L CNN
F 1 "Battery_Cell" H 4150 1575 50  0000 L CNN
F 2 "Pin_Headers:Pin_Header_Straight_2x01_Pitch2.54mm" V 4050 1635 50  0001 C CNN
F 3 "" V 4050 1635 50  0001 C CNN
	1    4050 1575
	1    0    0    -1  
$EndComp
$Comp
L +BATT #PWR1
U 1 1 5A9FF0C2
P 4050 1375
F 0 "#PWR1" H 4050 1225 50  0001 C CNN
F 1 "+BATT" H 4050 1515 50  0000 C CNN
F 2 "" H 4050 1375 50  0001 C CNN
F 3 "" H 4050 1375 50  0001 C CNN
	1    4050 1375
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR2
U 1 1 5A9FF0C3
P 4050 1675
F 0 "#PWR2" H 4050 1425 50  0001 C CNN
F 1 "GND" H 4050 1525 50  0000 C CNN
F 2 "" H 4050 1675 50  0001 C CNN
F 3 "" H 4050 1675 50  0001 C CNN
	1    4050 1675
	1    0    0    -1  
$EndComp
$Comp
L C_Small C2
U 1 1 5A9FF0C4
P 5475 2525
F 0 "C2" V 5475 2275 50  0000 L CNN
F 1 "0.1UF" V 5400 2250 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 5475 2525 50  0001 C CNN
F 3 "" H 5475 2525 50  0001 C CNN
	1    5475 2525
	0    1    1    0   
$EndComp
$Comp
L C_Small C3
U 1 1 5A9FF0C5
P 5775 2525
F 0 "C3" V 5775 2700 50  0000 L CNN
F 1 "0.1UF" V 5700 2600 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 5775 2525 50  0001 C CNN
F 3 "" H 5775 2525 50  0001 C CNN
	1    5775 2525
	0    1    1    0   
$EndComp
$Comp
L GND #PWR5
U 1 1 5A9FF0C6
P 5375 2525
F 0 "#PWR5" H 5375 2275 50  0001 C CNN
F 1 "GND" H 5375 2375 50  0000 C CNN
F 2 "" H 5375 2525 50  0001 C CNN
F 3 "" H 5375 2525 50  0001 C CNN
	1    5375 2525
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR6
U 1 1 5A9FF0C7
P 5875 2525
F 0 "#PWR6" H 5875 2275 50  0001 C CNN
F 1 "GND" H 5875 2375 50  0000 C CNN
F 2 "" H 5875 2525 50  0001 C CNN
F 3 "" H 5875 2525 50  0001 C CNN
	1    5875 2525
	1    0    0    -1  
$EndComp
$Comp
L C_Small C1
U 1 1 5A9FF0C8
P 3775 1525
F 0 "C1" H 3650 1625 50  0000 L CNN
F 1 "10UF" H 3550 1450 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 3775 1525 50  0001 C CNN
F 3 "" H 3775 1525 50  0001 C CNN
	1    3775 1525
	1    0    0    -1  
$EndComp
Text GLabel 5175 3425 0    60   Input ~ 0
NRST
Wire Wire Line
	5175 3625 4950 3625
Wire Wire Line
	3025 3625 3525 3625
Wire Wire Line
	5575 2325 5575 2525
Wire Wire Line
	5575 2525 5575 2925
Wire Wire Line
	5675 2325 5675 2525
Wire Wire Line
	5675 2525 5675 2925
Wire Wire Line
	5575 2325 5625 2325
Wire Wire Line
	5625 2325 5675 2325
Connection ~ 5625 2325
Connection ~ 5675 2525
Connection ~ 5575 2525
Wire Wire Line
	4050 1375 3775 1375
Wire Wire Line
	3775 1375 3775 1425
Wire Wire Line
	4050 1675 3775 1675
Wire Wire Line
	3775 1675 3775 1625
$Comp
L LED D1
U 1 1 5A9FF1D7
P 9025 2175
F 0 "D1" H 9025 2275 50  0000 C CNN
F 1 "LED" H 9025 2075 50  0000 C CNN
F 2 "" H 9025 2175 50  0001 C CNN
F 3 "" H 9025 2175 50  0001 C CNN
	1    9025 2175
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR3
U 1 1 5A9FF286
P 9175 2175
F 0 "#PWR3" H 9175 1925 50  0001 C CNN
F 1 "GND" H 9175 2025 50  0000 C CNN
F 2 "" H 9175 2175 50  0001 C CNN
F 3 "" H 9175 2175 50  0001 C CNN
	1    9175 2175
	1    0    0    -1  
$EndComp
$Comp
L R R2
U 1 1 5A9FF2CB
P 8725 2175
F 0 "R2" V 8805 2175 50  0000 C CNN
F 1 "1k" V 8725 2175 50  0000 C CNN
F 2 "" V 8655 2175 50  0001 C CNN
F 3 "" H 8725 2175 50  0001 C CNN
	1    8725 2175
	0    1    1    0   
$EndComp
$Comp
L LED D2
U 1 1 5A9FF74C
P 9025 2550
F 0 "D2" H 9025 2650 50  0000 C CNN
F 1 "LED" H 9025 2450 50  0000 C CNN
F 2 "" H 9025 2550 50  0001 C CNN
F 3 "" H 9025 2550 50  0001 C CNN
	1    9025 2550
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR7
U 1 1 5A9FF752
P 9175 2550
F 0 "#PWR7" H 9175 2300 50  0001 C CNN
F 1 "GND" H 9175 2400 50  0000 C CNN
F 2 "" H 9175 2550 50  0001 C CNN
F 3 "" H 9175 2550 50  0001 C CNN
	1    9175 2550
	1    0    0    -1  
$EndComp
$Comp
L R R3
U 1 1 5A9FF758
P 8725 2550
F 0 "R3" V 8805 2550 50  0000 C CNN
F 1 "1k" V 8725 2550 50  0000 C CNN
F 2 "" V 8655 2550 50  0001 C CNN
F 3 "" H 8725 2550 50  0001 C CNN
	1    8725 2550
	0    1    1    0   
$EndComp
$Comp
L LED D3
U 1 1 5A9FF7F4
P 9025 2950
F 0 "D3" H 9025 3050 50  0000 C CNN
F 1 "LED" H 9025 2850 50  0000 C CNN
F 2 "" H 9025 2950 50  0001 C CNN
F 3 "" H 9025 2950 50  0001 C CNN
	1    9025 2950
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR8
U 1 1 5A9FF7FA
P 9175 2950
F 0 "#PWR8" H 9175 2700 50  0001 C CNN
F 1 "GND" H 9175 2800 50  0000 C CNN
F 2 "" H 9175 2950 50  0001 C CNN
F 3 "" H 9175 2950 50  0001 C CNN
	1    9175 2950
	1    0    0    -1  
$EndComp
$Comp
L R R4
U 1 1 5A9FF800
P 8725 2950
F 0 "R4" V 8805 2950 50  0000 C CNN
F 1 "1k" V 8725 2950 50  0000 C CNN
F 2 "" V 8655 2950 50  0001 C CNN
F 3 "" H 8725 2950 50  0001 C CNN
	1    8725 2950
	0    1    1    0   
$EndComp
$Comp
L LED D4
U 1 1 5A9FF806
P 9025 3325
F 0 "D4" H 9025 3425 50  0000 C CNN
F 1 "LED" H 9025 3225 50  0000 C CNN
F 2 "" H 9025 3325 50  0001 C CNN
F 3 "" H 9025 3325 50  0001 C CNN
	1    9025 3325
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR9
U 1 1 5A9FF80C
P 9175 3325
F 0 "#PWR9" H 9175 3075 50  0001 C CNN
F 1 "GND" H 9175 3175 50  0000 C CNN
F 2 "" H 9175 3325 50  0001 C CNN
F 3 "" H 9175 3325 50  0001 C CNN
	1    9175 3325
	1    0    0    -1  
$EndComp
$Comp
L R R5
U 1 1 5A9FF812
P 8725 3325
F 0 "R5" V 8805 3325 50  0000 C CNN
F 1 "1k" V 8725 3325 50  0000 C CNN
F 2 "" V 8655 3325 50  0001 C CNN
F 3 "" H 8725 3325 50  0001 C CNN
	1    8725 3325
	0    1    1    0   
$EndComp
$Comp
L LED D5
U 1 1 5A9FFBF9
P 9025 3700
F 0 "D5" H 9025 3800 50  0000 C CNN
F 1 "LED" H 9025 3600 50  0000 C CNN
F 2 "" H 9025 3700 50  0001 C CNN
F 3 "" H 9025 3700 50  0001 C CNN
	1    9025 3700
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR12
U 1 1 5A9FFBFF
P 9175 3700
F 0 "#PWR12" H 9175 3450 50  0001 C CNN
F 1 "GND" H 9175 3550 50  0000 C CNN
F 2 "" H 9175 3700 50  0001 C CNN
F 3 "" H 9175 3700 50  0001 C CNN
	1    9175 3700
	1    0    0    -1  
$EndComp
$Comp
L R R6
U 1 1 5A9FFC05
P 8725 3700
F 0 "R6" V 8805 3700 50  0000 C CNN
F 1 "1k" V 8725 3700 50  0000 C CNN
F 2 "" V 8655 3700 50  0001 C CNN
F 3 "" H 8725 3700 50  0001 C CNN
	1    8725 3700
	0    1    1    0   
$EndComp
$Comp
L LED D6
U 1 1 5A9FFC0B
P 9025 4075
F 0 "D6" H 9025 4175 50  0000 C CNN
F 1 "LED" H 9025 3975 50  0000 C CNN
F 2 "" H 9025 4075 50  0001 C CNN
F 3 "" H 9025 4075 50  0001 C CNN
	1    9025 4075
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR14
U 1 1 5A9FFC11
P 9175 4075
F 0 "#PWR14" H 9175 3825 50  0001 C CNN
F 1 "GND" H 9175 3925 50  0000 C CNN
F 2 "" H 9175 4075 50  0001 C CNN
F 3 "" H 9175 4075 50  0001 C CNN
	1    9175 4075
	1    0    0    -1  
$EndComp
$Comp
L R R7
U 1 1 5A9FFC17
P 8725 4075
F 0 "R7" V 8805 4075 50  0000 C CNN
F 1 "1k" V 8725 4075 50  0000 C CNN
F 2 "" V 8655 4075 50  0001 C CNN
F 3 "" H 8725 4075 50  0001 C CNN
	1    8725 4075
	0    1    1    0   
$EndComp
$Comp
L LED D7
U 1 1 5A9FFC1D
P 9025 4475
F 0 "D7" H 9025 4575 50  0000 C CNN
F 1 "LED" H 9025 4375 50  0000 C CNN
F 2 "" H 9025 4475 50  0001 C CNN
F 3 "" H 9025 4475 50  0001 C CNN
	1    9025 4475
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR15
U 1 1 5A9FFC23
P 9175 4475
F 0 "#PWR15" H 9175 4225 50  0001 C CNN
F 1 "GND" H 9175 4325 50  0000 C CNN
F 2 "" H 9175 4475 50  0001 C CNN
F 3 "" H 9175 4475 50  0001 C CNN
	1    9175 4475
	1    0    0    -1  
$EndComp
$Comp
L R R8
U 1 1 5A9FFC29
P 8725 4475
F 0 "R8" V 8805 4475 50  0000 C CNN
F 1 "1k" V 8725 4475 50  0000 C CNN
F 2 "" V 8655 4475 50  0001 C CNN
F 3 "" H 8725 4475 50  0001 C CNN
	1    8725 4475
	0    1    1    0   
$EndComp
$Comp
L LED D8
U 1 1 5A9FFC2F
P 9025 4850
F 0 "D8" H 9025 4950 50  0000 C CNN
F 1 "LED" H 9025 4750 50  0000 C CNN
F 2 "" H 9025 4850 50  0001 C CNN
F 3 "" H 9025 4850 50  0001 C CNN
	1    9025 4850
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR17
U 1 1 5A9FFC35
P 9175 4850
F 0 "#PWR17" H 9175 4600 50  0001 C CNN
F 1 "GND" H 9175 4700 50  0000 C CNN
F 2 "" H 9175 4850 50  0001 C CNN
F 3 "" H 9175 4850 50  0001 C CNN
	1    9175 4850
	1    0    0    -1  
$EndComp
$Comp
L R R9
U 1 1 5A9FFC3B
P 8725 4850
F 0 "R9" V 8805 4850 50  0000 C CNN
F 1 "1k" V 8725 4850 50  0000 C CNN
F 2 "" V 8655 4850 50  0001 C CNN
F 3 "" H 8725 4850 50  0001 C CNN
	1    8725 4850
	0    1    1    0   
$EndComp
$EndSCHEMATC