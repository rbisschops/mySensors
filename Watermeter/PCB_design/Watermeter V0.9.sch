EESchema Schematic File Version 2
LIBS:Watermeter V0.9-rescue
LIBS:Power
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
LIBS:MySensors
LIBS:Watermeter V0.9-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "Water meter"
Date "2016-02-27"
Rev "0.9"
Comp "MySensors Boards"
Comment1 "R. Bisschops"
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Arduino_Mini_Pro REF1
U 1 1 560EB9E5
P 2600 6400
F 0 "REF1" H 2600 8050 50  0000 C CNN
F 1 "Arduino_Mini_Pro 5V" V 3000 7100 50  0000 C CNN
F 2 "MySensors:Arduino_Pro_Mini" H 2700 5700 60  0001 C CNN
F 3 "" H 2700 5700 60  0000 C CNN
	1    2600 6400
	1    0    0    -1  
$EndComp
$Comp
L NRF24L01 REF2
U 1 1 560EBABF
P 6450 1250
F 0 "REF2" H 6450 1550 50  0000 C CNN
F 1 "NRF24L01" H 6450 950 50  0000 C CNN
F 2 "MySensors:NRF24L01_STD" H 6450 50  60  0001 C CNN
F 3 "" H 6450 50  60  0000 C CNN
	1    6450 1250
	1    0    0    -1  
$EndComp
$Comp
L AP1117D33 U2
U 1 1 560EBBDE
P 4100 1250
F 0 "U2" H 4200 1000 50  0000 C CNN
F 1 "AP1117D33" H 4100 1500 50  0000 C CNN
F 2 "TO_SOT_Packages_SMD:SOT-223" H 4100 1250 60  0001 C CNN
F 3 "" H 4100 1250 60  0000 C CNN
	1    4100 1250
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR01
U 1 1 560EC06B
P 6150 2150
F 0 "#PWR01" H 6150 1900 50  0001 C CNN
F 1 "GND" H 6150 2000 50  0000 C CNN
F 2 "" H 6150 2150 60  0000 C CNN
F 3 "" H 6150 2150 60  0000 C CNN
	1    6150 2150
	1    0    0    -1  
$EndComp
Text GLabel 2100 5450 0    47   Input ~ 0
RAD_IRQ
Text GLabel 2100 6150 0    47   Input ~ 0
RAD_CE
Text GLabel 3900 6150 2    47   Input ~ 0
RAD_CSN
Text GLabel 3900 5850 2    47   Input ~ 0
SCK
Text GLabel 3900 6050 2    47   Input ~ 0
MOSI
Text GLabel 3900 5950 2    47   Input ~ 0
MISO
Text GLabel 7100 1350 2    47   Input ~ 0
RAD_IRQ
Text GLabel 6100 1150 0    47   Input ~ 0
RAD_CE
Text GLabel 7100 1150 2    47   Input ~ 0
RAD_CSN
Text GLabel 7100 1250 2    47   Input ~ 0
MOSI
Text GLabel 6100 1350 0    47   Input ~ 0
MISO
Text GLabel 6100 1250 0    47   Input ~ 0
SCK
$Comp
L CP_Small C9
U 1 1 560ED129
P 7050 1700
F 0 "C9" H 7060 1770 50  0000 L CNN
F 1 "4u7" H 7060 1620 50  0000 L CNN
F 2 "Capacitors_SMD:c_elec_5x5.3" H 7050 1700 60  0001 C CNN
F 3 "" H 7050 1700 60  0000 C CNN
	1    7050 1700
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR02
U 1 1 560ED2C4
P 7050 2150
F 0 "#PWR02" H 7050 1900 50  0001 C CNN
F 1 "GND" H 7050 2000 50  0000 C CNN
F 2 "" H 7050 2150 60  0000 C CNN
F 3 "" H 7050 2150 60  0000 C CNN
	1    7050 2150
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR03
U 1 1 560EDA7E
P 9150 2150
F 0 "#PWR03" H 9150 1900 50  0001 C CNN
F 1 "GND" H 9150 2000 50  0000 C CNN
F 2 "" H 9150 2150 60  0000 C CNN
F 3 "" H 9150 2150 60  0000 C CNN
	1    9150 2150
	1    0    0    -1  
$EndComp
$Comp
L R R3
U 1 1 560EE3C2
P 8550 1100
F 0 "R3" V 8630 1100 50  0000 C CNN
F 1 "56K" V 8550 1100 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 8480 1100 30  0001 C CNN
F 3 "" H 8550 1100 30  0000 C CNN
	1    8550 1100
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X02 CONN1
U 1 1 560EEBCF
P 800 1300
F 0 "CONN1" H 800 1450 50  0000 C CNN
F 1 "CONN_01X02" V 900 1300 50  0001 C CNN
F 2 "MySensors:Socket_Jack_5.5-2.1MM" H 800 1300 60  0001 C CNN
F 3 "" H 800 1300 60  0000 C CNN
	1    800  1300
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR04
U 1 1 560EECF1
P 1100 2150
F 0 "#PWR04" H 1100 1900 50  0001 C CNN
F 1 "GND" H 1100 2000 50  0000 C CNN
F 2 "" H 1100 2150 60  0000 C CNN
F 3 "" H 1100 2150 60  0000 C CNN
	1    1100 2150
	1    0    0    -1  
$EndComp
Text Notes 750  1100 0    47   ~ 0
RAW input \n(7V - 12V for 5V)
$Comp
L GND #PWR05
U 1 1 560F00D2
P 3650 6400
F 0 "#PWR05" H 3650 6150 50  0001 C CNN
F 1 "GND" H 3650 6250 50  0000 C CNN
F 2 "" H 3650 6400 60  0000 C CNN
F 3 "" H 3650 6400 60  0000 C CNN
	1    3650 6400
	1    0    0    -1  
$EndComp
$Comp
L D D1
U 1 1 56116884
P 1300 1250
F 0 "D1" H 1300 1150 50  0000 C CNN
F 1 "1N4007 M7" H 1250 1350 50  0000 C CNN
F 2 "MySensors:DO-214AC" H 1300 1250 60  0001 C CNN
F 3 "" H 1300 1250 60  0000 C CNN
	1    1300 1250
	-1   0    0    1   
$EndComp
$Comp
L C_Small C1
U 1 1 56116990
P 1950 1450
F 0 "C1" H 1975 1550 50  0000 L CNN
F 1 "100nF" H 1975 1350 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 1988 1300 30  0001 C CNN
F 3 "" H 1950 1450 60  0000 C CNN
	1    1950 1450
	1    0    0    -1  
$EndComp
Text GLabel 3900 5450 2    47   Input ~ 0
SEC-SDA
$Comp
L GND #PWR06
U 1 1 5611787C
P 1950 2150
F 0 "#PWR06" H 1950 1900 50  0001 C CNN
F 1 "GND" H 1950 2000 50  0000 C CNN
F 2 "" H 1950 2150 60  0000 C CNN
F 3 "" H 1950 2150 60  0000 C CNN
	1    1950 2150
	1    0    0    -1  
$EndComp
$Comp
L PWR_FLAG #FLG07
U 1 1 56117C0D
P 1700 1300
F 0 "#FLG07" H 1700 1395 50  0001 C CNN
F 1 "PWR_FLAG" H 1700 1480 50  0000 C CNN
F 2 "" H 1700 1300 60  0000 C CNN
F 3 "" H 1700 1300 60  0000 C CNN
	1    1700 1300
	-1   0    0    1   
$EndComp
$Comp
L PWR_FLAG #FLG08
U 1 1 56117EE4
P 1700 2100
F 0 "#FLG08" H 1700 2195 50  0001 C CNN
F 1 "PWR_FLAG" H 1700 2280 50  0000 C CNN
F 2 "" H 1700 2100 60  0000 C CNN
F 3 "" H 1700 2100 60  0000 C CNN
	1    1700 2100
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR09
U 1 1 56118144
P 1700 2150
F 0 "#PWR09" H 1700 1900 50  0001 C CNN
F 1 "GND" H 1700 2000 50  0000 C CNN
F 2 "" H 1700 2150 60  0000 C CNN
F 3 "" H 1700 2150 60  0000 C CNN
	1    1700 2150
	1    0    0    -1  
$EndComp
$Comp
L ATSHA240A U3
U 1 1 56119452
P 3850 3550
F 0 "U3" H 3900 4000 60  0000 C CNN
F 1 "ATSHA240A" H 4050 3500 60  0000 C CNN
F 2 "TO_SOT_Packages_SMD:SOT-23" H 3850 3550 60  0001 C CNN
F 3 "" H 3850 3550 60  0000 C CNN
	1    3850 3550
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR010
U 1 1 561195B2
P 4450 3750
F 0 "#PWR010" H 4450 3500 50  0001 C CNN
F 1 "GND" H 4450 3600 50  0000 C CNN
F 2 "" H 4450 3750 60  0000 C CNN
F 3 "" H 4450 3750 60  0000 C CNN
	1    4450 3750
	1    0    0    -1  
$EndComp
Text GLabel 3450 3350 0    47   Input ~ 0
SEC-SDA
$Comp
L C_Small C5
U 1 1 5611AF0C
P 4500 1550
F 0 "C5" H 4550 1650 50  0000 L CNN
F 1 "100nF" H 4525 1450 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 4538 1400 30  0001 C CNN
F 3 "" H 4500 1550 60  0000 C CNN
	1    4500 1550
	1    0    0    -1  
$EndComp
$Comp
L C_Small C4
U 1 1 5611AF9D
P 3700 1550
F 0 "C4" H 3725 1650 50  0000 L CNN
F 1 "100nF" H 3725 1450 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 3738 1400 30  0001 C CNN
F 3 "" H 3700 1550 60  0000 C CNN
	1    3700 1550
	1    0    0    -1  
$EndComp
$Comp
L CP_Small C2
U 1 1 5611AFFC
P 2250 1550
F 0 "C2" H 2275 1650 50  0000 L CNN
F 1 "47uF" H 2275 1450 50  0000 L CNN
F 2 "Capacitors_SMD:c_elec_6.3x5.3" H 2288 1400 30  0001 C CNN
F 3 "" H 2250 1550 60  0000 C CNN
	1    2250 1550
	1    0    0    -1  
$EndComp
$Comp
L R R1
U 1 1 5611B051
P 3150 1800
F 0 "R1" V 3230 1800 50  0000 C CNN
F 1 "470" V 3150 1800 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 3080 1800 30  0001 C CNN
F 3 "" H 3150 1800 30  0000 C CNN
	1    3150 1800
	1    0    0    -1  
$EndComp
$Comp
L LED D2
U 1 1 5611B0FA
P 3150 1450
F 0 "D2" H 3150 1550 50  0000 C CNN
F 1 "PWR" H 3150 1350 50  0000 C CNN
F 2 "LEDs:LED_1206" H 3150 1450 60  0001 C CNN
F 3 "" H 3150 1450 60  0000 C CNN
	1    3150 1450
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR011
U 1 1 5611BB5D
P 2250 2150
F 0 "#PWR011" H 2250 1900 50  0001 C CNN
F 1 "GND" H 2250 2000 50  0000 C CNN
F 2 "" H 2250 2150 60  0000 C CNN
F 3 "" H 2250 2150 60  0000 C CNN
	1    2250 2150
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR012
U 1 1 5611BB9F
P 3700 2150
F 0 "#PWR012" H 3700 1900 50  0001 C CNN
F 1 "GND" H 3700 2000 50  0000 C CNN
F 2 "" H 3700 2150 60  0000 C CNN
F 3 "" H 3700 2150 60  0000 C CNN
	1    3700 2150
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR013
U 1 1 5611BBE1
P 4100 2150
F 0 "#PWR013" H 4100 1900 50  0001 C CNN
F 1 "GND" H 4100 2000 50  0000 C CNN
F 2 "" H 4100 2150 60  0000 C CNN
F 3 "" H 4100 2150 60  0000 C CNN
	1    4100 2150
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR014
U 1 1 5611BC23
P 4500 2150
F 0 "#PWR014" H 4500 1900 50  0001 C CNN
F 1 "GND" H 4500 2000 50  0000 C CNN
F 2 "" H 4500 2150 60  0000 C CNN
F 3 "" H 4500 2150 60  0000 C CNN
	1    4500 2150
	1    0    0    -1  
$EndComp
Text GLabel 2100 5550 0    47   Input ~ 0
D3
NoConn ~ 2450 5050
NoConn ~ 2450 5150
NoConn ~ 3300 6550
NoConn ~ 3200 6550
NoConn ~ 2800 4650
NoConn ~ 2900 4650
NoConn ~ 3000 4650
NoConn ~ 3100 4650
NoConn ~ 3200 4650
NoConn ~ 3300 4650
NoConn ~ 3550 5550
NoConn ~ 3550 5650
$Comp
L LM393 U6
U 2 1 566B8C1B
P 8700 3700
F 0 "U6" H 8450 3950 60  0000 C CNN
F 1 "LM393" H 8850 3550 60  0000 C CNN
F 2 "SMD_Packages:SOIC-8-N" H 8700 3700 60  0001 C CNN
F 3 "" H 8700 3700 60  0000 C CNN
	2    8700 3700
	1    0    0    -1  
$EndComp
$Comp
L R R7
U 1 1 566B9420
P 9200 3450
F 0 "R7" V 9280 3450 50  0000 C CNN
F 1 "10K" V 9200 3450 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 9130 3450 30  0001 C CNN
F 3 "" H 9200 3450 30  0000 C CNN
	1    9200 3450
	1    0    0    -1  
$EndComp
$Comp
L R R5
U 1 1 566B9C99
P 7700 3450
F 0 "R5" V 7780 3450 50  0000 C CNN
F 1 "10K" V 7700 3450 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 7630 3450 30  0001 C CNN
F 3 "" H 7700 3450 30  0000 C CNN
	1    7700 3450
	1    0    0    -1  
$EndComp
$Comp
L R R4
U 1 1 566B9CF8
P 6850 3200
F 0 "R4" V 6930 3200 50  0000 C CNN
F 1 "1K" V 6850 3200 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 6780 3200 30  0001 C CNN
F 3 "" H 6850 3200 30  0000 C CNN
	1    6850 3200
	1    0    0    -1  
$EndComp
$Comp
L TCRT5000 U5
U 1 1 566B9D7F
P 7200 4250
F 0 "U5" H 7000 4450 50  0000 L CNN
F 1 "TCRT5000" H 7000 4050 50  0000 L CNN
F 2 "MySensors:TCRT5000" H 7000 4050 50  0001 L CIN
F 3 "" H 7200 4250 50  0000 L CNN
	1    7200 4250
	1    0    0    -1  
$EndComp
$Comp
L POT R6
U 1 1 566B9E6A
P 8050 3400
F 0 "R6" H 8050 3300 50  0000 C CNN
F 1 "10K" H 8050 3400 50  0000 C CNN
F 2 "Potentiometers:Potentiometer_Bourns_3296W_3-8Zoll_Inline_ScrewUp" H 8050 3400 60  0001 C CNN
F 3 "" H 8050 3400 60  0000 C CNN
	1    8050 3400
	0    1    1    0   
$EndComp
$Comp
L LED D3
U 1 1 566BB624
P 9600 3100
F 0 "D3" H 9600 3200 50  0000 C CNN
F 1 "PULSE" H 9600 3000 50  0000 C CNN
F 2 "LEDs:LED_1206" H 9600 3100 60  0001 C CNN
F 3 "" H 9600 3100 60  0000 C CNN
	1    9600 3100
	0    -1   -1   0   
$EndComp
$Comp
L R R8
U 1 1 566BB774
P 9600 3450
F 0 "R8" V 9680 3450 50  0000 C CNN
F 1 "1K" V 9600 3450 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 9530 3450 30  0001 C CNN
F 3 "" H 9600 3450 30  0000 C CNN
	1    9600 3450
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR015
U 1 1 566BCDF2
P 8600 4600
F 0 "#PWR015" H 8600 4350 50  0001 C CNN
F 1 "GND" H 8600 4450 50  0000 C CNN
F 2 "" H 8600 4600 60  0000 C CNN
F 3 "" H 8600 4600 60  0000 C CNN
	1    8600 4600
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR016
U 1 1 566BED8A
P 8050 4600
F 0 "#PWR016" H 8050 4350 50  0001 C CNN
F 1 "GND" H 8050 4450 50  0000 C CNN
F 2 "" H 8050 4600 60  0000 C CNN
F 3 "" H 8050 4600 60  0000 C CNN
	1    8050 4600
	1    0    0    -1  
$EndComp
Text GLabel 9800 3700 2    47   Input ~ 0
D3
$Comp
L GND #PWR017
U 1 1 566C2D2F
P 6850 4600
F 0 "#PWR017" H 6850 4350 50  0001 C CNN
F 1 "GND" H 6850 4450 50  0000 C CNN
F 2 "" H 6850 4600 60  0000 C CNN
F 3 "" H 6850 4600 60  0000 C CNN
	1    6850 4600
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR018
U 1 1 566C2D83
P 7500 4600
F 0 "#PWR018" H 7500 4350 50  0001 C CNN
F 1 "GND" H 7500 4450 50  0000 C CNN
F 2 "" H 7500 4600 60  0000 C CNN
F 3 "" H 7500 4600 60  0000 C CNN
	1    7500 4600
	1    0    0    -1  
$EndComp
$Comp
L C_Small C13
U 1 1 566C4816
P 7700 4350
F 0 "C13" H 7725 4450 50  0000 L CNN
F 1 "100nF" H 7725 4250 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 7738 4200 30  0001 C CNN
F 3 "" H 7700 4350 60  0000 C CNN
	1    7700 4350
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR019
U 1 1 566C491C
P 7700 4600
F 0 "#PWR019" H 7700 4350 50  0001 C CNN
F 1 "GND" H 7700 4450 50  0000 C CNN
F 2 "" H 7700 4600 60  0000 C CNN
F 3 "" H 7700 4600 60  0000 C CNN
	1    7700 4600
	1    0    0    -1  
$EndComp
Wire Wire Line
	7000 1050 7050 1050
Wire Wire Line
	7050 900  7050 1600
Wire Wire Line
	6150 2150 6150 1050
Wire Wire Line
	6150 1050 6200 1050
Wire Wire Line
	2100 6150 2450 6150
Wire Wire Line
	2450 5450 2100 5450
Wire Wire Line
	3900 6150 3550 6150
Wire Wire Line
	3550 5950 3900 5950
Wire Wire Line
	3550 6050 3900 6050
Wire Wire Line
	3550 5850 3900 5850
Wire Wire Line
	7000 1150 7100 1150
Wire Wire Line
	7000 1250 7100 1250
Wire Wire Line
	7000 1350 7100 1350
Wire Wire Line
	6100 1150 6200 1150
Wire Wire Line
	6100 1350 6200 1350
Wire Wire Line
	6100 1250 6200 1250
Connection ~ 7050 1050
Wire Wire Line
	7050 2150 7050 1800
Wire Wire Line
	9150 2100 9150 2150
Wire Wire Line
	9150 950  9150 1200
Wire Wire Line
	8550 1250 8550 1500
Wire Wire Line
	1000 1350 1100 1350
Wire Wire Line
	1100 1350 1100 2150
Wire Wire Line
	1000 1250 1150 1250
Wire Wire Line
	1450 1250 2450 1250
Wire Wire Line
	1950 1250 1950 1350
Wire Wire Line
	3550 5350 3800 5350
Wire Wire Line
	3800 4600 3800 6200
Wire Wire Line
	3550 5450 3900 5450
Wire Wire Line
	1950 1550 1950 2150
Wire Wire Line
	1700 2100 1700 2150
Wire Wire Line
	4450 3200 4650 3200
Wire Wire Line
	4650 2900 4650 3450
Wire Wire Line
	4450 3500 4450 3750
Wire Wire Line
	2250 1250 2250 1450
Wire Wire Line
	3050 1250 3800 1250
Wire Wire Line
	3700 1450 3700 1250
Connection ~ 3700 1250
Wire Wire Line
	4400 1250 4800 1250
Wire Wire Line
	4500 1250 4500 1450
Connection ~ 4500 1250
Wire Wire Line
	4500 1650 4500 2150
Wire Wire Line
	4100 1550 4100 2150
Wire Wire Line
	3700 1650 3700 2150
Wire Wire Line
	2250 1650 2250 2150
Wire Wire Line
	2100 5550 2450 5550
Wire Wire Line
	9600 3600 9600 3700
Wire Wire Line
	9200 3600 9200 3700
Wire Wire Line
	8200 3400 8200 3600
Wire Wire Line
	8600 2900 8600 3300
Wire Wire Line
	9200 2900 9200 3300
Wire Wire Line
	8050 2900 8050 3150
Wire Wire Line
	8050 3650 8050 4600
Wire Wire Line
	8600 4100 8600 4600
Wire Wire Line
	9200 3700 9800 3700
Connection ~ 9600 3700
Connection ~ 9200 3700
Wire Wire Line
	7700 2900 7700 3300
Wire Wire Line
	7700 3600 7700 4250
Wire Wire Line
	7700 4150 7500 4150
Wire Wire Line
	7700 3800 8200 3800
Wire Wire Line
	6850 2900 6850 3050
Wire Wire Line
	6850 4150 6850 3350
Connection ~ 7700 3800
Wire Wire Line
	7500 4350 7500 4350
Wire Wire Line
	7500 4350 7500 4600
Wire Wire Line
	6900 4350 6850 4350
Wire Wire Line
	6850 4350 6850 4600
Connection ~ 7700 4150
Wire Wire Line
	7700 4450 7700 4600
Wire Notes Line
	10050 2600 10050 4850
Wire Notes Line
	5400 2600 5400 4850
$Comp
L C_Small C7
U 1 1 566D105E
P 4650 3550
F 0 "C7" H 4675 3650 50  0000 L CNN
F 1 "100nF" H 4675 3450 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 4688 3400 30  0001 C CNN
F 3 "" H 4650 3550 60  0000 C CNN
	1    4650 3550
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR020
U 1 1 566D1123
P 4650 3750
F 0 "#PWR020" H 4650 3500 50  0001 C CNN
F 1 "GND" H 4650 3600 50  0000 C CNN
F 2 "" H 4650 3750 60  0000 C CNN
F 3 "" H 4650 3750 60  0000 C CNN
	1    4650 3750
	1    0    0    -1  
$EndComp
Wire Wire Line
	4650 3650 4650 3750
Connection ~ 4650 3200
Text Notes 2700 2700 0    60   ~ 0
Authentication
Text Notes 5750 700  0    60   ~ 0
NRF24L01 Radio Module
Text Notes 5450 2700 0    60   ~ 0
Sensor logic
Wire Notes Line
	10050 2400 7850 2400
Wire Notes Line
	7850 2400 7850 600 
Wire Notes Line
	7850 600  10050 600 
Wire Notes Line
	10050 600  10050 2400
Text Notes 7900 700  0    60   ~ 0
External memory
Wire Notes Line
	600  2400 5150 2400
Wire Notes Line
	5150 2400 5150 600 
Wire Notes Line
	5150 600  600  600 
Text Notes 650  700  0    60   ~ 0
Power supply
$Comp
L C_Small C10
U 1 1 566D8CB1
P 7300 1700
F 0 "C10" H 7310 1770 50  0000 L CNN
F 1 "100nF" H 7310 1620 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 7300 1700 60  0001 C CNN
F 3 "" H 7300 1700 60  0000 C CNN
	1    7300 1700
	1    0    0    -1  
$EndComp
Wire Wire Line
	7300 1600 7300 1450
Wire Wire Line
	7300 1450 7050 1450
Connection ~ 7050 1450
$Comp
L GND #PWR021
U 1 1 566D90F4
P 7300 2150
F 0 "#PWR021" H 7300 1900 50  0001 C CNN
F 1 "GND" H 7300 2000 50  0000 C CNN
F 2 "" H 7300 2150 60  0000 C CNN
F 3 "" H 7300 2150 60  0000 C CNN
	1    7300 2150
	1    0    0    -1  
$EndComp
Wire Wire Line
	7300 2150 7300 1800
Connection ~ 1950 1250
Wire Notes Line
	600  600  600  2400
Wire Notes Line
	5150 2600 5150 4000
Wire Notes Line
	2650 4000 2650 2600
Wire Notes Line
	5400 600  7650 600 
Wire Notes Line
	7650 2400 5400 2400
Wire Notes Line
	10050 2600 5400 2600
Wire Notes Line
	5400 4850 10050 4850
Wire Notes Line
	600  4200 600  7000
Wire Notes Line
	5150 7000 5150 4200
Text Notes 650  4350 0    60   ~ 0
Arduino Mini-Pro
Wire Notes Line
	5150 4200 600  4200
Wire Notes Line
	600  7000 5150 7000
$Comp
L C_Small C8
U 1 1 566D8483
P 3800 6300
F 0 "C8" H 3810 6370 50  0000 L CNN
F 1 "100nF" H 3810 6220 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 3800 6300 60  0001 C CNN
F 3 "" H 3800 6300 60  0000 C CNN
	1    3800 6300
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR022
U 1 1 566D854C
P 3800 6400
F 0 "#PWR022" H 3800 6150 50  0001 C CNN
F 1 "GND" H 3800 6250 50  0000 C CNN
F 2 "" H 3800 6400 60  0000 C CNN
F 3 "" H 3800 6400 60  0000 C CNN
	1    3800 6400
	1    0    0    -1  
$EndComp
Connection ~ 3800 5350
$Comp
L GND #PWR023
U 1 1 566DB8F4
P 2350 6700
F 0 "#PWR023" H 2350 6450 50  0001 C CNN
F 1 "GND" H 2350 6550 50  0000 C CNN
F 2 "" H 2350 6700 60  0000 C CNN
F 3 "" H 2350 6700 60  0000 C CNN
	1    2350 6700
	1    0    0    -1  
$EndComp
Wire Wire Line
	2450 5350 2350 5350
Wire Wire Line
	2350 5350 2350 6700
Wire Wire Line
	2800 6550 2350 6550
Connection ~ 2350 6550
$Comp
L R R2
U 1 1 566DECC5
P 3550 3050
F 0 "R2" V 3630 3050 50  0000 C CNN
F 1 "56K" V 3550 3050 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 3480 3050 30  0001 C CNN
F 3 "" H 3550 3050 30  0000 C CNN
	1    3550 3050
	1    0    0    -1  
$EndComp
Wire Wire Line
	3550 3200 3550 3350
Wire Wire Line
	3450 3350 3650 3350
Connection ~ 3550 3350
Wire Notes Line
	5150 4000 2650 4000
Wire Notes Line
	2650 2600 5150 2600
$Comp
L CP_Small C6
U 1 1 566E4C09
P 4800 1550
F 0 "C6" H 4850 1650 50  0000 L CNN
F 1 "10uF" H 4810 1470 50  0000 L CNN
F 2 "Capacitors_SMD:c_elec_5x5.3" H 4800 1550 60  0001 C CNN
F 3 "" H 4800 1550 60  0000 C CNN
	1    4800 1550
	1    0    0    -1  
$EndComp
Wire Wire Line
	4800 950  4800 1450
Connection ~ 4800 1250
Wire Wire Line
	4800 1650 4800 2150
$Comp
L GND #PWR024
U 1 1 566E506A
P 4800 2150
F 0 "#PWR024" H 4800 1900 50  0001 C CNN
F 1 "GND" H 4800 2000 50  0000 C CNN
F 2 "" H 4800 2150 60  0000 C CNN
F 3 "" H 4800 2150 60  0000 C CNN
	1    4800 2150
	1    0    0    -1  
$EndComp
Wire Wire Line
	1700 1250 1700 1300
Connection ~ 1700 1250
$Comp
L CONN_02X03 P1
U 1 1 566EFDD7
P 1500 3250
F 0 "P1" H 1500 3450 50  0000 C CNN
F 1 "CONN_02X03" H 1500 3050 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_2x03" H 1500 2050 60  0001 C CNN
F 3 "" H 1500 2050 60  0000 C CNN
	1    1500 3250
	1    0    0    -1  
$EndComp
Text GLabel 1150 3150 0    47   Input ~ 0
MISO
Text GLabel 1150 3250 0    47   Input ~ 0
SCK
Text GLabel 1150 3350 0    47   Input ~ 0
RESET
Text GLabel 1850 3250 2    47   Input ~ 0
MOSI
$Comp
L GND #PWR025
U 1 1 566F6866
P 1850 3750
F 0 "#PWR025" H 1850 3500 50  0001 C CNN
F 1 "GND" H 1850 3600 50  0000 C CNN
F 2 "" H 1850 3750 60  0000 C CNN
F 3 "" H 1850 3750 60  0000 C CNN
	1    1850 3750
	1    0    0    -1  
$EndComp
Wire Wire Line
	1850 2900 1850 3150
Wire Wire Line
	1850 3150 1750 3150
Wire Wire Line
	1750 3250 1850 3250
Wire Wire Line
	1750 3350 1850 3350
Wire Wire Line
	1850 3350 1850 3750
Wire Wire Line
	1250 3350 1150 3350
Wire Wire Line
	1150 3250 1250 3250
Wire Wire Line
	1150 3150 1250 3150
Text Notes 650  2750 0    60   ~ 0
AVR ISP
Wire Notes Line
	7650 600  7650 2400
Wire Notes Line
	5400 2400 5400 600 
$Comp
L C_Small C12
U 1 1 566FD39D
P 8800 3250
F 0 "C12" H 8810 3320 50  0000 L CNN
F 1 "100nF" H 8810 3170 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 8800 3250 60  0001 C CNN
F 3 "" H 8800 3250 60  0000 C CNN
	1    8800 3250
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR026
U 1 1 566FD420
P 8800 3350
F 0 "#PWR026" H 8800 3100 50  0001 C CNN
F 1 "GND" H 8800 3200 50  0000 C CNN
F 2 "" H 8800 3350 60  0000 C CNN
F 3 "" H 8800 3350 60  0000 C CNN
	1    8800 3350
	1    0    0    -1  
$EndComp
Wire Wire Line
	8800 3150 8600 3150
Connection ~ 8600 3150
$Comp
L AT25DF512C U4
U 1 1 56707563
P 8750 1900
F 0 "U4" H 8800 2450 60  0000 C CNN
F 1 "AT25DF512C" H 9450 1850 60  0000 C CNN
F 2 "SMD_Packages:SOIC-8-N" H 9600 50  60  0001 C CNN
F 3 "" H 9600 50  60  0000 C CNN
	1    8750 1900
	1    0    0    -1  
$EndComp
Wire Wire Line
	9800 1600 9750 1600
Wire Wire Line
	9800 950  9800 1600
Wire Wire Line
	9800 1500 9750 1500
Connection ~ 9800 1500
$Comp
L C_Small C11
U 1 1 56707DA3
P 9400 1100
F 0 "C11" H 9410 1170 50  0000 L CNN
F 1 "100nF" H 9410 1020 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 9400 1100 60  0001 C CNN
F 3 "" H 9400 1100 60  0000 C CNN
	1    9400 1100
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR027
U 1 1 56707EB7
P 9400 1200
F 0 "#PWR027" H 9400 950 50  0001 C CNN
F 1 "GND" H 9400 1050 50  0000 C CNN
F 2 "" H 9400 1200 60  0000 C CNN
F 3 "" H 9400 1200 60  0000 C CNN
	1    9400 1200
	1    0    0    -1  
$EndComp
Wire Wire Line
	9400 1000 9150 1000
Connection ~ 9150 1000
Text GLabel 2100 6050 0    47   Input ~ 0
FLASH-CSN
Wire Wire Line
	2100 6050 2450 6050
NoConn ~ 2450 5750
Text GLabel 8450 1500 0    47   Input ~ 0
FLASH-CSN
Text GLabel 8450 1600 0    47   Input ~ 0
MISO
Text GLabel 8450 1700 0    47   Input ~ 0
MOSI
Text GLabel 8450 1800 0    47   Input ~ 0
SCK
Wire Wire Line
	8550 1500 8450 1500
Wire Wire Line
	8450 1600 8550 1600
Wire Wire Line
	8450 1700 8550 1700
Wire Wire Line
	8450 1800 8550 1800
Connection ~ 8550 1500
Wire Notes Line
	2450 2600 600  2600
Wire Notes Line
	600  2600 600  4000
Wire Notes Line
	600  4000 2450 4000
Wire Notes Line
	2450 4000 2450 2600
Text GLabel 3900 5250 2    47   Input ~ 0
RESET
NoConn ~ 3550 5050
$Comp
L GND #PWR028
U 1 1 5674ABC0
P 3150 2150
F 0 "#PWR028" H 3150 1900 50  0001 C CNN
F 1 "GND" H 3150 2000 50  0000 C CNN
F 2 "" H 3150 2150 60  0000 C CNN
F 3 "" H 3150 2150 60  0000 C CNN
	1    3150 2150
	1    0    0    -1  
$EndComp
NoConn ~ 2450 5650
NoConn ~ 2450 5850
NoConn ~ 2450 5950
$Comp
L LM393 U6
U 1 1 566F46EA
P 6100 3700
F 0 "U6" H 6250 3850 60  0000 C CNN
F 1 "LM393" H 6300 3500 60  0000 C CNN
F 2 "" H 6100 3700 60  0000 C CNN
F 3 "" H 6100 3700 60  0000 C CNN
	1    6100 3700
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR029
U 1 1 566F4A6D
P 5550 4250
F 0 "#PWR029" H 5550 4000 50  0001 C CNN
F 1 "GND" H 5550 4100 50  0000 C CNN
F 2 "" H 5550 4250 60  0000 C CNN
F 3 "" H 5550 4250 60  0000 C CNN
	1    5550 4250
	1    0    0    -1  
$EndComp
NoConn ~ 6600 3700
Wire Wire Line
	3150 1950 3150 2150
Wire Wire Line
	6850 4150 6900 4150
Wire Wire Line
	5600 3800 5550 3800
Wire Wire Line
	5550 3600 5550 4250
Wire Wire Line
	5600 3600 5550 3600
Connection ~ 5550 3800
Wire Wire Line
	7700 4050 9800 4050
Connection ~ 7700 4050
Text GLabel 9800 4050 2    47   Input ~ 0
A0
Text GLabel 3900 5750 2    47   Input ~ 0
A0
Wire Wire Line
	3550 5750 3900 5750
$Comp
L AP1117D33 U1
U 1 1 568D26B3
P 2750 1250
F 0 "U1" H 2850 1000 50  0000 C CNN
F 1 "AP1117D50" H 2750 1500 50  0000 C CNN
F 2 "TO_SOT_Packages_SMD:SOT-223" H 2750 1250 60  0001 C CNN
F 3 "" H 2750 1250 60  0000 C CNN
	1    2750 1250
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR030
U 1 1 568D27A8
P 2750 2150
F 0 "#PWR030" H 2750 1900 50  0001 C CNN
F 1 "GND" H 2750 2000 50  0000 C CNN
F 2 "" H 2750 2150 60  0000 C CNN
F 3 "" H 2750 2150 60  0000 C CNN
	1    2750 2150
	1    0    0    -1  
$EndComp
$Comp
L CP_Small C3
U 1 1 568D28FB
P 3400 1550
F 0 "C3" H 3425 1650 50  0000 L CNN
F 1 "10uF" H 3425 1450 50  0000 L CNN
F 2 "Capacitors_SMD:c_elec_5x5.3" H 3438 1400 30  0001 C CNN
F 3 "" H 3400 1550 60  0000 C CNN
	1    3400 1550
	1    0    0    -1  
$EndComp
Wire Wire Line
	3400 1250 3400 1450
Wire Wire Line
	3400 1650 3400 2150
Wire Wire Line
	2750 2150 2750 1550
Connection ~ 3400 1250
$Comp
L GND #PWR031
U 1 1 568D2E11
P 3400 2150
F 0 "#PWR031" H 3400 1900 50  0001 C CNN
F 1 "GND" H 3400 2000 50  0000 C CNN
F 2 "" H 3400 2150 60  0000 C CNN
F 3 "" H 3400 2150 60  0000 C CNN
	1    3400 2150
	1    0    0    -1  
$EndComp
Connection ~ 2250 1250
Wire Wire Line
	3300 950  3300 1250
Connection ~ 3300 1250
Connection ~ 3150 1250
$Comp
L +3.3V-RESCUE-Watermeter_V0.4 #PWR032
U 1 1 56922E34
P 4800 950
F 0 "#PWR032" H 4800 800 50  0001 C CNN
F 1 "+3.3V" H 4800 1100 50  0000 C CNN
F 2 "" H 4800 950 60  0000 C CNN
F 3 "" H 4800 950 60  0000 C CNN
	1    4800 950 
	1    0    0    -1  
$EndComp
$Comp
L +3.3V-RESCUE-Watermeter_V0.4 #PWR033
U 1 1 56924A2B
P 7050 900
F 0 "#PWR033" H 7050 750 50  0001 C CNN
F 1 "+3.3V" H 7050 1050 50  0000 C CNN
F 2 "" H 7050 900 60  0000 C CNN
F 3 "" H 7050 900 60  0000 C CNN
	1    7050 900 
	1    0    0    -1  
$EndComp
$Comp
L +5V-RESCUE-Watermeter_V0.9 #PWR034
U 1 1 569259C2
P 3300 950
F 0 "#PWR034" H 3300 800 50  0001 C CNN
F 1 "+5V" H 3300 1100 50  0000 C CNN
F 2 "" H 3300 950 60  0000 C CNN
F 3 "" H 3300 950 60  0000 C CNN
	1    3300 950 
	1    0    0    -1  
$EndComp
$Comp
L +5V-RESCUE-Watermeter_V0.9 #PWR035
U 1 1 56928B61
P 8550 950
F 0 "#PWR035" H 8550 800 50  0001 C CNN
F 1 "+5V" H 8550 1100 50  0000 C CNN
F 2 "" H 8550 950 60  0000 C CNN
F 3 "" H 8550 950 60  0000 C CNN
	1    8550 950 
	1    0    0    -1  
$EndComp
$Comp
L +5V-RESCUE-Watermeter_V0.9 #PWR036
U 1 1 56928BBD
P 9150 950
F 0 "#PWR036" H 9150 800 50  0001 C CNN
F 1 "+5V" H 9150 1100 50  0000 C CNN
F 2 "" H 9150 950 60  0000 C CNN
F 3 "" H 9150 950 60  0000 C CNN
	1    9150 950 
	1    0    0    -1  
$EndComp
$Comp
L +5V-RESCUE-Watermeter_V0.9 #PWR037
U 1 1 56928C19
P 9800 950
F 0 "#PWR037" H 9800 800 50  0001 C CNN
F 1 "+5V" H 9800 1100 50  0000 C CNN
F 2 "" H 9800 950 60  0000 C CNN
F 3 "" H 9800 950 60  0000 C CNN
	1    9800 950 
	1    0    0    -1  
$EndComp
$Comp
L +5V-RESCUE-Watermeter_V0.9 #PWR038
U 1 1 5692B55B
P 1850 2900
F 0 "#PWR038" H 1850 2750 50  0001 C CNN
F 1 "+5V" H 1850 3050 50  0000 C CNN
F 2 "" H 1850 2900 60  0000 C CNN
F 3 "" H 1850 2900 60  0000 C CNN
	1    1850 2900
	1    0    0    -1  
$EndComp
$Comp
L +5V-RESCUE-Watermeter_V0.9 #PWR039
U 1 1 5692CE9C
P 8600 2900
F 0 "#PWR039" H 8600 2750 50  0001 C CNN
F 1 "+5V" H 8600 3050 50  0000 C CNN
F 2 "" H 8600 2900 60  0000 C CNN
F 3 "" H 8600 2900 60  0000 C CNN
	1    8600 2900
	1    0    0    -1  
$EndComp
$Comp
L +5V-RESCUE-Watermeter_V0.9 #PWR040
U 1 1 5692D01D
P 3550 2900
F 0 "#PWR040" H 3550 2750 50  0001 C CNN
F 1 "+5V" H 3550 3050 50  0000 C CNN
F 2 "" H 3550 2900 60  0000 C CNN
F 3 "" H 3550 2900 60  0000 C CNN
	1    3550 2900
	1    0    0    -1  
$EndComp
$Comp
L +5V-RESCUE-Watermeter_V0.9 #PWR041
U 1 1 5692D0E6
P 7700 2900
F 0 "#PWR041" H 7700 2750 50  0001 C CNN
F 1 "+5V" H 7700 3050 50  0000 C CNN
F 2 "" H 7700 2900 60  0000 C CNN
F 3 "" H 7700 2900 60  0000 C CNN
	1    7700 2900
	1    0    0    -1  
$EndComp
$Comp
L +5V-RESCUE-Watermeter_V0.9 #PWR042
U 1 1 5692D1F9
P 3800 4600
F 0 "#PWR042" H 3800 4450 50  0001 C CNN
F 1 "+5V" H 3800 4750 50  0000 C CNN
F 2 "" H 3800 4600 60  0000 C CNN
F 3 "" H 3800 4600 60  0000 C CNN
	1    3800 4600
	1    0    0    -1  
$EndComp
$Comp
L +5V-RESCUE-Watermeter_V0.9 #PWR043
U 1 1 5692D2C2
P 9200 2900
F 0 "#PWR043" H 9200 2750 50  0001 C CNN
F 1 "+5V" H 9200 3050 50  0000 C CNN
F 2 "" H 9200 2900 60  0000 C CNN
F 3 "" H 9200 2900 60  0000 C CNN
	1    9200 2900
	1    0    0    -1  
$EndComp
$Comp
L +5V-RESCUE-Watermeter_V0.9 #PWR044
U 1 1 5692D38F
P 9600 2900
F 0 "#PWR044" H 9600 2750 50  0001 C CNN
F 1 "+5V" H 9600 3050 50  0000 C CNN
F 2 "" H 9600 2900 60  0000 C CNN
F 3 "" H 9600 2900 60  0000 C CNN
	1    9600 2900
	1    0    0    -1  
$EndComp
$Comp
L +5V-RESCUE-Watermeter_V0.9 #PWR045
U 1 1 5692D458
P 4650 2900
F 0 "#PWR045" H 4650 2750 50  0001 C CNN
F 1 "+5V" H 4650 3050 50  0000 C CNN
F 2 "" H 4650 2900 60  0000 C CNN
F 3 "" H 4650 2900 60  0000 C CNN
	1    4650 2900
	1    0    0    -1  
$EndComp
$Comp
L +5V-RESCUE-Watermeter_V0.9 #PWR046
U 1 1 5692D546
P 8050 2900
F 0 "#PWR046" H 8050 2750 50  0001 C CNN
F 1 "+5V" H 8050 3050 50  0000 C CNN
F 2 "" H 8050 2900 60  0000 C CNN
F 3 "" H 8050 2900 60  0000 C CNN
	1    8050 2900
	1    0    0    -1  
$EndComp
$Comp
L +5V-RESCUE-Watermeter_V0.9 #PWR047
U 1 1 5692D634
P 6850 2900
F 0 "#PWR047" H 6850 2750 50  0001 C CNN
F 1 "+5V" H 6850 3050 50  0000 C CNN
F 2 "" H 6850 2900 60  0000 C CNN
F 3 "" H 6850 2900 60  0000 C CNN
	1    6850 2900
	1    0    0    -1  
$EndComp
Wire Notes Line
	5400 5050 5400 6450
Wire Notes Line
	5400 6450 7000 6450
Wire Notes Line
	7000 6450 7000 5050
Wire Notes Line
	7000 5050 5400 5050
Text Notes 5450 5150 0    60   ~ 0
Temperature sensor (option)
$Comp
L SI7021 U7
U 1 1 56A1555E
P 6150 6000
F 0 "U7" H 6200 6450 47  0000 C CNN
F 1 "SI7021" H 6550 5950 47  0000 C CNN
F 2 "MySensors:SI7021" H 6150 6000 47  0000 C CNN
F 3 "" H 6150 6000 47  0000 C CNN
	1    6150 6000
	1    0    0    -1  
$EndComp
$Comp
L +3.3V-RESCUE-Watermeter_V0.7 #PWR048
U 1 1 56A15C52
P 6400 5400
F 0 "#PWR048" H 6400 5250 50  0001 C CNN
F 1 "+3.3V" H 6400 5550 50  0000 C CNN
F 2 "" H 6400 5400 60  0000 C CNN
F 3 "" H 6400 5400 60  0000 C CNN
	1    6400 5400
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR049
U 1 1 56A15CB0
P 6400 6200
F 0 "#PWR049" H 6400 5950 50  0001 C CNN
F 1 "GND" H 6400 6050 50  0000 C CNN
F 2 "" H 6400 6200 60  0000 C CNN
F 3 "" H 6400 6200 60  0000 C CNN
	1    6400 6200
	1    0    0    -1  
$EndComp
Text GLabel 5950 5700 0    47   Input ~ 0
TMP_SCL
Text GLabel 5950 5900 0    47   Input ~ 0
TMP_SDA
Text GLabel 3150 6700 2    47   Input ~ 0
TMP_SCL
Text GLabel 3150 6800 2    47   Input ~ 0
TMP_SDA
Wire Wire Line
	3150 6700 3100 6700
Wire Wire Line
	3100 6700 3100 6550
Wire Wire Line
	3000 6550 3000 6800
Wire Wire Line
	3000 6800 3150 6800
Wire Wire Line
	3550 5150 3650 5150
Wire Wire Line
	3650 5150 3650 6400
Wire Wire Line
	3550 5250 3900 5250
Text GLabel 2100 5250 0    47   Input ~ 0
RESET
Wire Wire Line
	2450 5250 2100 5250
$EndSCHEMATC
