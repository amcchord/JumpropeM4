# RS03 Instruction Manual

## Precautions
1. Please use according to the working parameters specified in this article, otherwise it may cause serious damage to the product!
2. Do not switch the control mode when the joint is running. If you need to switch, send the command to stop the operation before switching.
3. Check whether the parts are in good condition before use. If the parts are missing or damaged, contact technical support in time.
4. Do not disassemble the motor at will, so as to avoid unrecoverable failure.
5. Ensure that there is no short circuit when the motor is connected, and the interface is correctly connected as required.

## Legal Statement
Before using this product, please read this manual carefully and operate the product according to the contents of this manual. If the user violates the contents of this manual to use this product, resulting in any property damage, personal injury accident, the company does not assume any responsibility. Because this product is composed of many parts, do not allow children to touch this product to avoid accidents. In order to prolong the service life of the product, do not use this product in high temperature and high pressure environment. This manual has been printed to the extent possible to include a description of the functions and instructions for use. However, due to the continuous improvement of product functions, design changes, etc., there may still be discrepancies with the products purchased by users.

The color and appearance of this manual may differ from the actual product. Please refer to the actual product. This manual is published by Beijing Lingfoot Times Technology Co., LTD. (hereinafter referred to as Lingfoot), and Lingfoot may at any time make necessary improvements and changes to the inaccurate and up-to-date information in this manual, or make improvements to procedures and/or equipment. Such changes will be uploaded to the company's official website in electronic format. Details can be found in the download center (www.robstride.com). All images are for reference only. Please refer to actual objects.

## After-sales Policy
The after-sales service of this product is implemented in strict accordance with the Law of the People's Republic of China on the Protection of Consumer Rights and Interests and the Product Quality Law of the People's Republic of China. The service content is as follows:

### 1. Warranty period and contents
1. Users who place orders on the online channel to purchase this product can enjoy the return service without reason within seven days from the day after signing. When returning goods, the user must present a valid proof of purchase and return the invoice. The user must ensure that the returned goods maintain the original quality and function, the appearance is intact, the trademarks and various logos of the goods themselves and accessories are complete, and if there are gifts, they should be returned together. If the goods are artificially damaged, artificially disassembled, missing packaging boxes, missing parts and accessories, they will not be returned. The logistics cost incurred during the return shall be borne by the user (see "After-sales Service Fee Standard"). If the user does not settle the logistics cost, it will be deducted from the refund amount according to the actual amount incurred. Refund the amount paid to the user within seven days from the date of receipt of the returned item. Refund method is the same as payment method. The specific arrival date may be affected by factors such as banks and payment institutions.
2. The warranty period of this product is 1 year.
3. Within 7 days after the user signs for the next day, non-human damage performance failure occurs, through the Lingzhu after-sales service center test and confirmation, for the user to handle the return business, the user must present a valid purchase voucher, and return the invoice. Any freebies should be returned.
4. From 7 days to 15 days after the user signs for the next day, non-human damage performance failure occurs, through the Lingfoot after-sales service center test and confirmation, for the user to replace the whole set of goods. After the replacement, the three guarantee period of the goods themselves is recalculated.
5. From 15 days to 365 days after the user signed the next day, after the inspection and confirmation of the Lingfoot after-sales service center, it is a quality fault of the product itself, and can provide free maintenance services. The replacement of the faulty product is owned by Lingzu Company. The product is not faulty and will be returned as is. This product has been strictly tested after the factory, if there is a quality fault other than the product itself, we will have the right to refuse the user's return demand.

### 2. Non-warranty regulations 
The following circumstances are not covered by the warranty:
3. Exceed the warranty period specified in the warranty terms.
4. Failure to follow the instructions, resulting in product damage caused by wrong use.
5. Damage caused by improper operation, maintenance, installation, modification, testing and other improper use.
6. Non-quality failure caused by conventional mechanical loss, wear.
7. Damage caused by abnormal working conditions, including but not limited to falling, impact, liquid immersion, violent impact, etc.
8. Damage caused by natural disasters (such as floods, fires, lightning strikes, earthquakes, etc.) or incapacitated forces.
9. Damage caused by exceeding peak torque.
10. Damage caused by exceeding peak torque.
11. Failure or damage caused by other non-product design, technology, manufacturing, quality and other problems.
12. Use this product for commercial purposes.

In the case of the above situation, the user must pay the cost.

## Motor specification

### Outline and mounting dimensions
When fixing, the screw depth should not exceed the depth of the casing thread

### Standard usage status
1. Rated voltage: 48 VDC
2. Operating voltage range: 24V-60 VDC
3. Rated load (CW) : 20 N.m
4. Operation direction: CW/CCW from the direction of the exit shaft
5. Use posture: the direction of the exit axis is horizontal or vertical
6. Standard operating temperature: 25±5℃
7. Operating temperature range: -20 ~ 50℃
8. Standard operating humidity: 65%
9. Humidity range: 5 ~ 85%, no condensation
10. Storage temperature range: -30 ~ 70℃
11. Insulation Class: Class B

### Electrical characteristics
1. No load speed: 200 rpm±10%
2. No load current: 2 Arms
3. Rated load: 20 N.m
4. Rated load speed: 180rpm±10%
5. Rated load phase current (peak) : 13Apk±10%
6. Peak load: 60 N.m
7. Maximum load phase current (peak) : 43Apk±10%
8. Insulation resistance/stator winding: DC 500VAC, 100M Ohms
9. High voltage/stator and housing: 600 VAC, 1s, 2mA
10. Motor back potential: 17Vrms/krpm±10%
11. Torque constant: 2.36N.m/Arms
12. T-N curve
13. Maximum overload curve

### Test conditions
- Ambient temperature: 25℃
- Winding limit temperature: 130℃ (this is the constraint temperature, the actual is 180 degrees)
- Speed: 24rpm

### Test data
| Load | Operating time(s) |
|------|-------------------|
| 60   | 7                 |
| 50   | 12                |
| 40   | 26                |
| 30   | 189               |
| 20   | rated             |

### Mechanical properties
1. Weight: 880g±20g
2. Number of poles: 42
3. Phase number: 3 phases
4. Drive mode: FOC
5. Deceleration ratio: 9:1

## Drive product information

### Drive product specifications
| Project | Data |
|---------|------|
| The rated working voltage | 48VDC |
| The maximum allowable voltage | 60VDC |
| Rated working phase current | 12Apk |
| Maximum allowable phase current | 43Apk |
| Standby power | ≤40mA |
| CAN bus bit rate | 1Mbps |
| Dimensions | Φ70mm |
| Working environment temperature | -20℃ to 50℃ |
| The maximum allowable temperature of the control board | 105℃ |
| Encoder resolution | 14bit (absolute turn) |

## Driver interface definition

### Driver interface recommended brand and model
| Board end model | Brand manufacturer | Line end model | Brand manufacturer |
|-----------------|-------------------|----------------|-------------------|
| XT30APW-M | AMASS (Ams) | XT30UW-F | AMASS (AMS) |
| GH1.25-2PWT | any | GH1.25-T | any |

### Driver function pin and device description

#### 1. Power supply and CAN communication
| Pin | Description |
|-----|-------------|
| 1 | The positive electrode of the power supply (+) |
| 2 | Negative electrode of the power supply (-) |
| 3 | CAN CAN_L |
| 4 | CAN the high side of the communication CAN_H |

#### 2. Download port
| Pin | Description |
|-----|-------------|
| 1 | SWDIO (data) |
| 2 | SWCLK (clock) |
| 3 | 3V3 (positive 3.3V) |
| 4 | GND |

#### 3. Indicator light
| Pin | Description |
|-----|-------------|
| 1 | If the blue indicator blinks, the program is running normally |
| 2 | Power indicator. If the indicator is red, the power supply to the entire network is normal |

## Main devices and specifications
| No. | Item | Specifications | Quantity |
|-----|------|---------------|----------|
| 1 | MCU chip | GD32F303RET6 | 1 PCS |
| 2 | Driver chip | DRV8353SRTAT | 1 PCS |
| 3 | Magnetic encoder chip | AS5047P | 2 PCS |
| 4 | The thermistor | LTS00-104J395T19E010/ NCP18XH103F03RB | 2 PCS |
| 5 | Power MOS | ISC030N12NM6 | 12 PCS |

## Upper computer instructions
Please go to www.robstride.com website download center

## Hardware disposition
The articulated motor uses the CAN communication mode and has two communication cables. It is connected to the debugger through the can to USB tool. The debugger needs to be installed with the ch340 driver in advance and works in AT mode by default.

It should be noted that we are based on the specific can to USB tool development of the debugger, so we need to use our recommended serial port tool to debug the debugger, if you want to transplant to other debugger platform can refer to the third chapter of the instructions for development.

The CAN to USB tool is recommended to use the official USB-CAN module of Lingzu Times. The frame header of the corresponding serial port protocol is 41 54, and the frame tail is 0D 0A.

## Upper computer interface and description
It mainly includes:

### A. Select a module
- Device module
- Disposition module
- Analysis module
- Help Module

### B. Select a submodule
- Connect or disconnect motor equipment
- Motor equipment information
- Motor encoder calibration
- Modify the motor CAN ID
- Set the mechanical zero position of the motor
- Motor program upgrade
- Parameter table, you can view and modify the motor parameters
- Upload parameters. The parameters in the motor can be uploaded to the parameter table
- Download parameters, you can download the data in the parameter table to the motor
- Export parameters. You can download data in the parameter table to a local computer
- Restore the data in the parameter table to factory defaults
- Clear warning, can clear motor errors, such as high temperature

### Analysis modules include:
- Oscilloscope, you can view the curve of parameter change with time
- Frequency: You can adjust the frequency of viewing data
- The channel can be disposition to view the data
- Start and stop drawing
- Output waveform data locally

### Help modules include:
- Instructions, you can open the instruction manual
- Yes, you can check the software information

### C. Motor information query
- Device information
- Parameter table information

### D. Data field
- Log information
- Communication information

### E. Run the debugging area
- Select equipment
- Convenient operation area, can quickly control the positive and negative rotation of the motor
- Motion control area, which can control the motor operation according to various modes

### F. Submodule display area

## Motor setup

### Motor connection setup
Connect the CAN-to-USB tool (Install the ch340 driver, which works in AT mode by default), click the connection submodule in the device module, select the corresponding serial port connection and motor type, and click Connect.

### Basic setup
1. Change the motor id.
2. Motor magnetic coding calibration, motor board and motor re-installation, or motor three-phase line re-sequential connection, need to be re-calibrated magnetic coding.
3. Set the zero position (power loss) to 0.
4. Motor program upgrade, when the motor program is updated, click the upgrade button to select the upgrade file to upgrade.

## Parameter list
After the motor is successfully connected, click the parameter table module in disposition module. The log will show that all parameters are loaded successfully, indicating that the relevant parameters of the motor are successfully read (Note: The parameter table is required for disposition under the standby state of the motor. If the motor is in the running state, the parameter table cannot be refreshed), the interface will display the relevant parameters of the motor. The parameters in blue are the stored parameters in the motor, which can be modified in the current value bar after the corresponding parameters. Click to download parameters to download the parameters in the debugger to the motor, click to upload parameters to upload the parameters in the motor to the debugger, and the green parameters of the motor are observed parameters, which are collected parameters and can be observed in real time.

**Note: Please do not change the torque limit, protection temperature and overtemperature time of the motor. Our company will not bear any legal responsibility for any damage to human body or irreversible damage to joints caused by illegal operation of this product.**

## Actively report (Upgrade 0.0.2.7 to obtain the function)
The motor automatically reports off by default, and reports on type 24.
The reporting type is Type 2. The default reporting interval is 10ms. You can change the reporting period by using EPScan_time of type 18.

## Zero flag bit description (Upgrade 0.0.2.7 to obtain the function)
The zero_sta flag bit is modified by the host computer or type 18, where the modification by type 18 needs to be saved by communication type 22.
The default motor flag bit is 0, and the default position after power-on is 0-2π.
If the flag bit is set to 1, the default position after power-on is -π-π.

## Type 2 Change Description (Upgrade 0.0.2.7 to obtain the function)
Type 2 is changed to a periodic cycle -4π-4π, by which the number of turns can be counted.
Note that the location interface needs to be changed:
- P_MIN is -12.57f
- P_MAX is 12.57f

### Function Code Parameters
| Function code | Name | Parameter type | Attribute | Maximum value | Minimum value | Current value (for reference) | Notes |
|---------------|------|----------------|-----------|---------------|--------------|----------------------------|-------|
| 0X0000 | Name | String | Read/Write | | | ÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿ | |
| 0X0001 | BarCode | String | Read/Write | | | ÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿ | |
| 0X1000 | BootCodeVersion | String | Read only | | | 0.1.5 | |
| 0X1001 | BootBuildDate | String | Read only | | | Mar 16 2022 | |
| 0X1002 | BootBuildTime | String | Read only | | | 20:22:09 | |
| 0X1003 | AppCodeVersion | String | Read only | | | 0.0.0.1 | Motor program version number |
| 0X1004 | AppGitVersion | String | Read only | | | 7b844b0fM | |
| 0X1005 | AppBuildDate | String | Read only | | | Apr 14 2022 | |
| 0X1006 | AppBuildTime | String | Read only | | | 20:30:22 | |
| 0X1007 | AppCodeName | String | Read only | | | Lingzu_motor | |
| 0X2000 | echoPara1 | uint16 | Disposition | 74 | 5 | 5 | |
| 0X2001 | echoPara2 | uint16 | Disposition | 74 | 5 | 5 | |
| 0X2002 | echoPara3 | uint16 | Disposition | 74 | 5 | 5 | |
| 0X2003 | echoPara4 | uint16 | Disposition | 74 | 5 | 5 | |
| 0X2004 | echoFreHz | uint32 | Read/Write | 10000 | 1 | 500 | |
| 0X2005 | MechOffset | float | Settings | 7 | -7 | 4.619583 | Motor magnetic encoder Angle offset |
| 0X2006 | MechPos_init | float | Read/Write | 50 | -50 | 4.52 | Reserved parameter |
| 0X2007 | limit_torque | float | Read/Write | 17 | 0 | 17 | Torque limitation |
| 0X2008 | I_FW_MAX | float | Read/Write | 33 | 0 | 0 | Weak magnetic current value, default 0 |
| 0X2009 | motor_baud | uint8 | Settings | 20 | 0 | 1 | Motor index, marking the joint position of the motor |
| 0X200a | CAN_ID | uint8 | Settings | 127 | 0 | 1 | id of this object |
| 0X200b | CAN_MASTER | uint8 | Settings | 127 | 0 | 0 | can host id |
| 0X200c | CAN_TIMEOUT | uint32 | Read/Write | 100000 | 0 | 0 | can timeout threshold. The default value is 0 |
| 0X200d | status2 | int16 | Read/Write | 1500 | 0 | 800 | Reserved parameter |
| 0X200e | status3 | uint32 | Read/Write | 1000000 | 1000 | 20000 | Reserved parameter |
| 0X200f | status1 | float | Read/Write | 64 | 1 | 7.75 | Reserved parameter |
| 0X2010 | Status6 | uint8 | Read/Write | 1 | 0 | 1 | Reserved parameter |
| 0X2011 | cur_filt_gain | float | Read/Write | 1 | 0 | 0.9 | Current filtering parameter |
| 0X2012 | cur_kp | float | Read/Write | 200 | 0 | 0.025 | Current kp |
| 0X2013 | cur_ki | float | Read/Write | 200 | 0 | 0.0258 | Current ki |
| 0X2014 | spd_kp | float | Read/Write | 200 | 0 | 2 | Velocity kp |
| 0X2015 | spd_ki | float | Read/Write | 200 | 0 | 0.021 | Speed ki |
| 0X2016 | loc_kp | float | Read/Write | 200 | 0 | 30 | Position kp |
| 0X2017 | spd_filt_gain | float | Read/Write | 1 | 0 | 0.1 | Velocity filter parameter |
| 0X2018 | limit_spd | float | Read/Write | 200 | 0 | 2 | Location mode speed limit |
| 0X2019 | limit_cur | float | Read/Write | 23 | 0 | 23 | Position, Velocity mode current limit |
| 0X201a | loc_ref_filt_gain | float | Read/Write | 100 | 0 | 0 | Reserved parameter |
| 0X201b | limit_loc | float | Read/Write | 100 | 0 | 0 | Reserved parameter |
| 0X201c | position_offset | float | Read/Write | 27 | 0 | 0 | High speed segment offset |
| 0X201d | chasu_angle_offset | float | Read/Write | 27 | 0 | 0 | The low end is offset |
| 0X201e | spd_step_value | float | Read/Write | 150 | 0 | | Velocity-mode acceleration |
| 0X201f | vel_max | float | Read/Write | 20 | 0 | | PP mode speed |
| 0X2020 | acc_set | float | Read/Write | 1000 | 0 | | PP mode acceleration |
| 0X2021 | zero_sta | float | Read/Write | 100 | 0 | 0 | Zero marker |
| 0X3000 | timeUse0 | uint16 | Read only | | | 5 | |
| 0X3001 | timeUse1 | uint16 | Read only | | | 0 | |
| 0X3002 | timeUse2 | uint16 | Read only | | | 10 | |
| 0X3003 | timeUse3 | uint16 | Read only | | | 0 | |
| 0X3004 | encoderRaw | int16 | Read only | | | 11396 | Magnetic encoder sampling value |
| 0X3005 | mcuTemp | int16 | Read only | | | 337 | mcu internal temperature, *10 |
| 0X3006 | motorTemp | int16 | Read only | | | 333 | Motor ntc temperature, *10 |
| 0X3007 | vBus(mv) | uint16 | Read only | | | 24195 | Bus voltage |
| 0X3008 | adc1Offset | int32 | Read only | | | 2084 | adc sampling channel 1 Zero current bias |
| 0X3009 | adc2Offset | int32 | Read only | | | 2084 | adc sampling channel 2 Zero current bias |
| 0X300a | adc1Raw | uint16 | Read only | | | 1232 | adc sampling value 1 |
| 0X300b | adc2Raw | uint16 | Read only | | | 1212 | adc sampling value 2 |
| 0X300c | VBUS | float | Read only | | | 36 | Bus voltage V |
| 0X300d | cmdId | float | Read only | | | 0 | id ring instruction, A |
| 0X300e | cmdIq | float | Read only | | | 0 | iq ring command, A |
| 0X300f | cmdlocref | float | Read only | | | 0 | Position loop command, rad |
| 0X3010 | cmdspdref | float | Read only | | | 0 | Speed loop command, rad/s |
| 0X3011 | cmdTorque | float | Read only | | | 0 | Torque instruction, nm |
| 0X3012 | cmdPos | float | Read only | | | 0 | mit Protocol Angle instruction |
| 0X3013 | cmdVel | float | Read only | | | 0 | mit Protocol Speed instruction |
| 0X3014 | rotation | int16 | Read only | | | 1 | Number of turns |
| 0X3015 | modPos | float | Read only | | | 4.363409 | Motor uncounted coil mechanical Angle, rad |
| 0X3016 | mechPos | float | Read only | | | 0.777679 | Load end loop mechanical Angle, rad |
| 0X3017 | mechVel | float | Read only | | | 0.036618 | Load speed: rad/s |
| 0X3018 | elecPos | float | Read only | | | 4.714761 | Electrical Angle |
| 0X3019 | ia | float | Read only | | | 0 | U-wire current, A |
| 0X301a | ib | float | Read only | | | 0 | V-wire current, A |
| 0X301b | ic | float | Read only | | | 0 | W-wire current, A |
| 0X301c | timeout | uint32 | Read only | | | 31600 | Timeout counter value |
| 0X301d | phaseOrder | uint8 | Read only | | | 0 | Directional marking |
| 0X301e | iqf | float | Read only | | | 0 | iq filter value，A |
| 0X301f | boardTemp | int16 | Read only | | | 359 | Plate temperature，*10 |
| 0X3020 | iq | float | Read only | | | 0 | iq Original value，A |
| 0X3021 | id | float | Read only | | | 0 | id Original value，A |
| 0X3022 | faultSta | uint32 | Read only | | | 0 | Fault status value |
| 0X3023 | warnSta | uint32 | Read only | | | 0 | Warning status value |
| 0X3024 | drv_fault | uint16 | Read only | | | 0 | The driver chip fault value is 1 |
| 0X3025 | drv_temp | int16 | Read only | | | 48 | The driver chip fault value is 2 |
| 0X3026 | Uq | float | Read only | | | 0 | Q-axis voltage |
| 0X3027 | Ud | float | Read only | | | 0 | D-axis voltage |
| 0X3028 | dtc_u | float | Read only | | | 0 | The duty cycle of the U-phase output |
| 0X3029 | dtc_v | float | Read only | | | 0 | The duty cycle of the V-phase output |
| 0X302a | dtc_w | float | Read only | | | 0 | The duty cycle of the W-phase output |
| 0X302b | v_bus | float | Read only | | | 24.195 | Vbus in the closed loop |
| 0X302c | torque_fdb | float | Read only | | | 0 | Torque feedback value，nm |
| 0X302d | rated_i | float | Read only | | | 8 | Rated current of motor |
| 0X302e | limit_i | float | Read only | | | 27 | The motor limits the maximum current |
| 0X302f | spd_ref | float | Read only | | | 0 | Motor speed expectation |
| 0X3030 | spd_reff | float | Read only | | | 0 | Motor speed expectation 2 |
| 0X3031 | zero_fault | float | Read only | | | 0 | Motor position determination parameters |
| 0X3032 | chasu_coder_raw | float | Read only | | | 0 | Motor position determination parameters |
| 0X3033 | chasu_angle | float | Read only | | | 0 | Motor position determination parameters |
| 0X3034 | as_angle | float | Read only | | | 0 | Motor position determination parameters |
| 0X3035 | vel_max | float | Read only | | | 0 | Motor position determination parameters |
| 0X3036 | judge | float | Read only | | | 0 | Motor position determination parameters |
| 0X3037 | fault1 | uint32 | Read only | | | 0 | Log failure |
| 0X3038 | fault2 | uint32 | Read only | | | 0 | Log failure |
| 0X3039 | fault3 | uint32 | Read only | | | 0 | Log failure |
| 0X303a | fault4 | uint32 | Read only | | | 0 | Log failure |
| 0X303b | fault5 | uint32 | Read only | | | 0 | Log failure |
| 0X303c | fault6 | uint32 | Read only | | | 0 | Log failure |
| 0X303d | fault7 | uint32 | Read only | | | 0 | Log failure |
| 0X303e | fault8 | uint32 | Read only | | | 0 | Log failure |
| 0X303f | ElecOffset | float | Read only | | | 0 | electrical Angle offset |
| 0X3040 | mcOverTemp | int16 | Read only | | | 0 | Overtemperature threshold |
| 0X3041 | Kt_Nm/Amp | float | Read only | | | 0 | Moment coefficient |
| 0X3042 | Tqcali_Type | uint8 | Read only | | | 0 | Motor type |
| 0X3043 | low_position | float | Read only | | | 0 | Motor position determination parameters |
| 0X3044 | theta_mech_1 | float | Read only | | | 0 | Type 2 Low speed Angle |
| 0X3045 | instep | float | Read only | | | 0 | Motor protection decision parameters |

## Oscilloscope
The interface supports viewing and observing the graph generated by real-time data, including motor Id/Iq current, temperature, real-time speed at the output end, rotor (encoder) position, output end position, etc.

Click on the oscilloscope module in the analysis module, select the appropriate parameters in the channel (parameter meaning can be referred to the parameter table), set the output frequency, click on the start plot to observe the data graph, stop the plot to stop the observation graph.

# Control demo

## Jog running
Set the maximum speed, click Run, click JOG run to make the motor run forward and backward.

## Control mode switching
The motor control mode can be changed in the motion mode interface.

## Operation control mode
Click the switch button on the right, then set five parameter values, click Start or continuous send, the motor will return the feedback frame and run according to the target instruction; Click the switch button on the right side again, and the motor will stop.

## Current mode
Manually switch the current mode, click the switch button on the right side, then set the Iq current command value, start or continue to send, the motor will follow the current command, click the switch button on the right side again, the motor will stop.

Click the switch button on the right side of the control mode, enter the amplitude and frequency of the sinusoidal automatic test, then click the switch button on the right side of the sinusoidal automatic test, and the iq (A) of the motor will run according to the amplitude and frequency of the Settings.

## Velocity mode
Manually cut the Velocity mode, click the right switch button, then set the speed command value, start or continue to send, the motor will follow the speed command, click the right switch button again, the motor will stop.

Click the switch button on the right side of the control mode, enter the amplitude and frequency of the sinusoidal automatic test, then click the switch button on the right side of the sinusoidal automatic test, and the motor speed (rad/s) will run according to the amplitude and frequency of the Settings.

## Location Mode (CSP)
Manually switch the position mode (CSP), click the right switch button, then set the position instruction value (rad), start or continuous transmission, the motor will follow the target position instruction, click the right switch button again, the motor will stop. You can set the speed to change the maximum speed for following the position.

Click the switch button on the right side of the control mode, enter the amplitude and frequency of the sinusoidal automatic test, then click the switch button on the right side of the sinusoidal automatic test, and the motor position (rad) will run according to the amplitude and frequency of the Settings.

## Location Mode (PP)
Manually switch the position mode (PP), click the switch button on the right side, and then set the position instruction value (rad), speed setting instruction value (rad/s), acceleration setting (rad/s^2) to start or continue to send, the motor will follow the target position instruction to run, click the switch button on the right side again, the motor will stop. You can modify the maximum speed and acceleration followed by the position by setting the speed.

## Firmware update
First, click Upgrade of device module and select bin file to burn; The second step is to confirm the upgrade, and the motor starts to update the firmware. After the progress is completed, the motor is updated and automatically restarts.

## Driver protocol and instructions
The motor communication is the CAN 2.0 communication interface, the baud rate is 1Mbps, and the extended frame format is adopted as follows:

| Data field | 29-bit ID | | | 8Byte data field |
|------------|-----------|--|--|----------------|
| Size | Bit28~bit24 | bit23~8 | bit7~0 | Byte0~Byte7 |
| Description | Communication type | Data area 2 | Destination address | Data area 1 |

## Communication box instruction example

```
41 54 90 07 e8 0c 08 05 70 00 00 01 00 00 00 0d 0a
```

The meaning is as follows:

| 41 54 | 90 07 e8 0c | 08 | 05 70 00 00 01 00 00 00 | 0d 0a |
|-------|-------------|----|-----------------------|-------|
| frame header | Number of data bits | extended frame | data frame | frame tail |

The translation of extended frame canid into real canid requires the following transformations:
90 07 e8 0c converts to binary as 1001 0000 0000 0111 1110 1000 0000 1100, remove the 100 on the right and it becomes 1 0010 0000 0000 1111 1101 0000 0001, convert it to hexadecimal, It is 12 00 FD 01. According to the communication protocol, the meaning is as follows:

| 12 in hexadecimal | 00 | FD | 01 |
|-------------------|----|----|-----|
| Communication type 18 (in decimal base) | No meaning | host id | motor canid |

# Description of the communication protocol type

## Communication type 0: Get device ID
Gets the device's ID and 64-bit MCU unique identifier

Reply frame:
| Data field | 29-bit ID | 8Byte data field |
|------------|-----------|------------------|
| Size | Bit28~bit24 | bit23~8 | bit7~0 | Byte0~Byte7 |
| Description | 0x0 | bit15~8: identifies host CAN_ID | target motor CAN_ID | 0 |

| Data field | 29-bit ID | 8Byte data field |
|------------|-----------|------------------|
| Size | Bit28~bit24 | bit23~8 | bit7~0 | Byte0~Byte7 |
| Description | 0x0 | target motor CAN_ID | 0XFE | 64-bit MCU unique identifier |

## Communication Type 1: Operation control mode motor control instruction
| Data field | 29-bit ID | 8Byte data field |
|------------|-----------|------------------|
| Description | 0x1 | Byte2: Torque (0~65535) corresponds to (-60Nm~60Nm) | target motor CAN_ID | Byte0~1: target Angle [0~65535] corresponds to (-4π~4π) Byte2~3: Target angular velocity [0~65535] corresponds to (-20rad/s~20rad/s) Byte4~5: Kp [0~65535] corresponds to (0.0~5000.0) Byte6~7: Kd [0 to 65535] corresponds to the above data (0.0 to 100.0). After the conversion, the high byte is in front and the low byte is in |

Response frame: Response motor feedback frame (see communication type 2)

## Communication Type 2: Motor feedback data
| Data field | 29-bit ID | 8Byte data field |
|------------|-----------|------------------|
| Size | Bit28~bit24 | bit23~8 | bit7~0 | Byte0~Byte7 |
| Description | 0x2 | Bit8~Bit15: CAN ID of the current motor bit21~16: fault information (0 none 1 has) bit21: uncalibrated bit20: Uncalibrated bit20: Gridlock overload fault bit19: magnetic coding fault bit18: overtemperature bit17: overcurrent bit16: undervoltage fault bit22~23: Mode status 0: Reset mode [reset] 1: Cali mode [calibration] 2: Motor mode [Run] | host CAN_ID | Byte0~1: The current Angle [0~65535] Corresponding to (-4π~4π) Byte2~3: Current angular velocity [0~65535] corresponds to (-20rad/s~20rad/s) Byte4~5: Current torque [0~65535] corresponds to (-60Nm~60Nm) Byte6~7: Current temperature: Temp(Celsius) *10 If the value is higher than 10, the high byte is first and the low byte is last |

## Communication Type 3: Motor enabled to run
| Data field | 29-bit ID | 8Byte data field |
|------------|-----------|------------------|
| Size | Bit28~bit24 | bit23~8 | bit7~0 | Byte0~Byte7 |
| Description | 0x3 | bit15~8: identifies the main CAN_ID | target motor CAN_ID | |

Response frame: Response motor feedback frame (see communication type 2)

## Communication Type 4: Motor stops running
| Data field | 29-bit ID | 8Byte data field |
|------------|-----------|------------------|
| Size | Bit28~bit24 | bit23~8 | bit7~0 | Byte0~Byte7 |
| Description | 0x4 | bit15~8: used to identify the main CAN_ID | target motor CAN_ID | When the motor is running normally, 0 must be cleared in the data field. Byte[0]=1: The fault is cleared. |

Response frame: Response motor feedback frame (see communication type 2)

## Communication type 6: Set motor mechanical zero
| Data field | 29-bit ID | 8Byte data field |
|------------|-----------|------------------|
| Size | Bit28~bit24 | bit23~8 | bit7~0 | Byte0~Byte7 |
| Description | 0x6 | bit15~8: Identifies the main CAN_ID | target motor CAN_ID | Byte[0]=1 |

Response frame: Response motor feedback frame (see communication type 2)

## Communication type 7: Set motor CAN_ID
Change the current motor CAN_ID, effective immediately.

| Data field | 29-bit ID | 8Byte data field |
|------------|-----------|------------------|
| Size | Bit28~bit24 | bit23~8 | bit7~0 | Byte0~Byte7 |
| Description | 0x7 | bit15~8: used to identify main CAN_ID Bit16~23: preset CAN_ID | Target motor CAN_ID | |

Answer frame: Answer motor broadcast frame (see communication type 0)

## Communication type 17: Single parameter read
| Data field | 29-bit ID | 8Byte data field |
|------------|-----------|------------------|
| Size | Bit28~bit24 | bit23~8 | bit7~0 | Byte0~Byte7 |
| Description | 0x11 | bit15~8: Used to identify the main CAN_ID | target motor CAN_ID | Byte0~1: index. For details, see the readability parameter table below Byte2~3: 00 Byte4~7: In data above 00, the low byte is first and the high byte is second |

Reply frame:
| Data field | 29-bit ID | 8Byte data field |
|------------|-----------|------------------|
| Size | Bit28~bit24 | bit23~8 | bit7~0 | Byte0~Byte7 |
| Description | 0x11 | bit15~8: indicates that the master CAN_ID Bit23~16:00 indicates that the master CAN_ID is successfully read. 01 indicates that the master can_ID | | Byte0~1: Byte2~3: 00 Byte4~7: Parameter data. 1 byte of data above Byte4 is preceded by low bytes and followed by high bytes at |

## Communication type 18: Single parameter write (lost in power failure)
With type 22, the parameter starting with function code 0x20 of the parameter table in the upper computer module can be saved.

| Data field | 29-bit ID | 8Byte data field |
|------------|-----------|------------------|
| Size | Bit28~bit24 | bit23~8 | bit7~0 | Byte0~Byte7 |
| Description | 0x12 | bit15~8: Used to identify the main CAN_ID | target motor CAN_ID | Byte0~1: index. For details, see the readability parameter table below Byte2~3: 00 Byte4~7: Parameter data In the preceding data, the low byte is in the front and the high byte is in the rear |

## Communication type 21: Fault feedback frame
| Data field | 29-bit ID | 8Byte data field |
|------------|-----------|------------------|
| Size | Bit28~bit24 | bit23~8 | bit7~0 | Byte0~Byte7 |
| Description | 0x15 | bit15~8: motor CAN_ID identifies the main CAN_ID | | Byte0~3: fault value (non-0: faulty; 0: faulty). Normal) bit14: gridlock i square t overload fault bit7: encoder not calibrated bit3: overvoltage fault bit2: undervoltage fault bit1: driver chip fault bit0: motor overtemperature fault, Default 145 ° C Byte4~7: warning Value bit0: motor overtemperature warning, the default is 135 ° c |

## Communication type 22: Motor data save frame
| Data field | 29-bit ID | 8Byte data field |
|------------|-----------|------------------|
| Size | Bit28~bit24 | bit23~8 | bit7~0 | Byte0~Byte7 |
| Description | 0x16 | bit15~8: identifies the main CAN_ID | target motor CAN_ID | |

Response frame: Response motor feedback frame (see communication type 2)

## Communication type 23: Motor baud rate modification frame (re-power-on effect)
| Data field | 29-bit ID | 8Byte data field |
|------------|-----------|------------------|
| Size | Bit28~bit24 | bit23~8 | bit7~0 | Byte0~Byte7 |
| Description | 0x17 | bit15~8: used to identify the main CAN_ID | target motor CAN_ID | Byte0: 01 is 1M 02 is 500K 03 is 250K 04 is 100K |

Response frame: Response motor feedback frame (see communication type 0)

## Communication type 24: The motor actively reports frames
| Data field | 29-bit ID | 8Byte data field |
|------------|-----------|------------------|
| Size | Bit28~bit24 | bit23~8 | bit7~0 | Byte0~Byte7 |
| Description | 0x18 | bit15~8: identifies the main CAN_ID | target motor CAN_ID | Byte0: 00 indicates that active reporting is disabled (by default). 01 indicates that active reporting is enabled. The default reporting interval is 10ms |

Response frame: Response motor feedback frame (see communication type 2)

## Read and write a single parameter list
| Index | Description | Type | Number of bytes | R/W Read and write permission |
|-------|-------------|------|-----------------|------------------------------|
| 0X7005 | run_mode 0: operation mode 1: position mode (PP) 2: Velocity mode 3: Operation mode Current mode 5: Position mode (CSP) | uint8 | 1 | W/R |
| 0X7006 | iq_ref Current mode Iq command | float | 4 | -43 to 43A W/R |
| 0X700A | spd_ref Rotational Velocity mode Rotational speed command | float | 4 | -20to 20rad/s W/R |
| 0X700B | limit_torque torque limit | float | 4 | 0 to 60Nm W/R |
| 0X7010 | cur_kp Kp | float | 4 | The default value is 0.17 W/R |
| 0X7011 | cur_ki Ki | float | 4 | The default value is 0.012 W/R |
| 0X7014 | cur_filt_gain filt_gain | float | 4 | 0 to 1.0, The default value is 0.1 W/R |
| 0X7016 | loc_ref Position Mode Angle instruction | float | 4 | rad W/R |
| 0X7017 | limit_spd Location mode (CSP) speed limit | float | 4 | 0 to 20rad/s W/R |
| 0X7018 | limit_cur Velocity position mode Current limitation | float | 4 | 0 to 43A W/R |
| 0x7019 | mechPos Mechanical Angle of the loading coil | float | 4 | rad R |
| 0x701A | iqf iq Filter | float | 4 | -43 to 43A R |
| 0x701B | mechVel Speed of the load | float | 4 | -20 to 20rad/s R |
| 0x701C | VBUS Bus voltage | float | 4 | V R |
| 0x701E | loc_kp kp at | float | 4 | The default value is 60 W/R |
| 0x701F | spd_kp Indicates the speed kp | float | 4 | The default value is 6 W/R |
| 0x7020 | spd_ki ki | float | 4 | The default value is 0.02 W/R |
| 0x7021 | spd_filt_gain Speed filter value | float | 4 | The default value is 0.1 W |
| 0x7022 | acc_rad velocity mode acceleration | float | 4 | The default value is 20rad/s^2 W |
| 0x7024 | vel_max Location mode (PP) speed | float | 4 | The default value is 10rad/s W |
| 0x7025 | acc_set Location mode (PP) acceleration | float | 4 | The default value is 10rad/s^2 W |
| 0x7026 | EPScan_time Indicates the report time. 1 indicates 10ms. Plus 1 increments by 5ms | uint16 | 2 | The default value is 1 W |
| 0x7028 | canTimeout can The timeout threshold, 20000 is 1s | uint32 | 4 | The default value is 0 W |
| 0x7029 | zero_sta Indicates the zero flag bit, 0 means 0-2π and 1 means -π-π | uint8 | 1 | The default is 0 W |

## Read example
Take reading loc_kp as an example:

Read instruction is:
| Size | Bit28~bit24 | bit23~8 | bit7~0 | Byte0~Byte7 |
|------|-------------|--------|--------|-------------|
| 0x11 | 0x00FD | 0x7F | 1E 70 00 00 00 00 00 00 |

The feedback instruction is:
| Size | Bit28~bit24 | bit23~8 | bit7~0 | Byte0~Byte7 |
|------|-------------|--------|--------|-------------|
| 0x11 | 0x007F | 0xFD | 1E 70 00 00 00 00 F0 41 |
| Description | Type 17 | bit15~8: Target motor CAN_ID 7F | Host id 0xFD | Byte0~1: index, corresponding to loc_kp Byte4~7:loc_kp value 30, high right byte, (32-bit single precision) hexadecimal IEEE-754 standard floating point number |

## CAN communication failure protection
When the value of CAN_TIMEOUT is 0, this function is disabled
When the CAN_TIMEOUT value is non-0, when the motor does not receive the can command within a certain period of time, the motor enters the reset mode, and 20000 is 1s

# Control mode instructions

## Program sample
Examples of various mode control motors are provided below (take gd32f303 as an example).

The following are library, function, and macro definitions for the various instances:

```c
#define P_MIN -12.57f //0.3.0.5及之前为12.5，之后为12.57
#define P_MAX 12.57f //0.3.0.5及之前为12.5，之后为12.57
#define V_MIN -20.0f
#define V_MAX 20.0f
#define KP_MIN 0.0f
#define KP_MAX 5000.0f
#define KD_MIN 0.0f
#define KD_MAX 100.0f
#define T_MIN -60.0f
#define T_MAX 60.0f

struct exCanIdInfo{
  uint32_t id:8;
  uint32_t data:16;
  uint32_t mode:5;
  uint32_t res:3;
};

can_receive_message_struct rxMsg;
can_trasnmit_message_struct txMsg={
  .tx_sfid = 0,
  .tx_efid = 0xff,
  .tx_ft = CAN_FT_DATA,
  .tx_ff = CAN_FF_EXTENDED,
  .tx_dlen = 8,
};

#define txCanIdEx (*((struct exCanIdInfo*)&(txMsg.tx_efid)))
#define rxCanIdEx (*((struct exCanIdInfo*)&(rxMsg.rx_efid))) //将扩展帧id解析为自定义数据结构

int float_to_uint(float x, float x_min, float x_max, int bits){
  float span = x_max - x_min;
  float offset = x_min;
  if(x > x_max) x=x_max;
  else if(x < x_min) x= x_min;
  return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}
```

The following lists the common types of communication sent:

## Motor Enabled Run frame (communication type 3)
```c
#define can_txd() can_message_transmit(CAN0, &txMsg)
#define can_rxd() can_message_receive(CAN0, CAN_FIFO1, &rxMsg)

void motor_enable(uint8_t id, uint16_t master_id)
{
  txCanIdEx.mode = 3;
  txCanIdEx.id = id;
  txCanIdEx.res = 0;
  txCanIdEx.data = master_id;
  txMsg.tx_dlen = 8;
  txCanIdEx.data = 0;
  can_txd();
}
```

## Operation control mode Motor control instruction (communication type 1)
```c
void motor_controlmode(uint8_t id, float torque, float MechPosition, float speed, float kp, float kd)
{
  txCanIdEx.mode = 1;
  txCanIdEx.id = id;
  txCanIdEx.res = 0;
  txCanIdEx.data = float_to_uint(torque,T_MIN,T_MAX,16);
  txMsg.tx_dlen = 8;
  txMsg.tx_data[0]=float_to_uint(MechPosition,P_MIN,P_MAX,16)>>8;
  txMsg.tx_data[1]=float_to_uint(MechPosition,P_MIN,P_MAX,16);
  txMsg.tx_data[2]=float_to_uint(speed,V_MIN,V_MAX,16)>>8;
  txMsg.tx_data[3]=float_to_uint(speed,V_MIN,V_MAX,16);
  txMsg.tx_data[4]=float_to_uint(kp,KP_MIN,KP_MAX,16)>>8;
  txMsg.tx_data[5]=float_to_uint(kp,KP_MIN,KP_MAX,16);
  txMsg.tx_data[6]=float_to_uint(kd,KD_MIN,KD_MAX,16)>>8;
  txMsg.tx_data[7]=float_to_uint(kd,KD_MIN,KD_MAX,16);
  can_txd();
}
```

## Motor reset command (communication type 4)
```c
void motor_reset(uint8_t id, uint16_t master_id)
{
  txCanIdEx.mode = 4;
  txCanIdEx.id = id;
  txCanIdEx.res = 0;
  txCanIdEx.data = master_id;
  txMsg.tx_dlen = 8;
  for(uint8_t i=0;i<8;i++)
  {
    txMsg.tx_data[i]=0;
  }
  can_txd();
}
```

## Motor mode parameter write command (communication type 18, running mode switch)
```c
uint8_t runmode;
uint16_t index;

void motor_modechange(uint8_t id, uint16_t master_id)
{
  txCanIdEx.mode = 0x12;
  txCanIdEx.id = id;
  txCanIdEx.res = 0;
  txCanIdEx.data = master_id;
  txMsg.tx_dlen = 8;
  for(uint8_t i=0;i<8;i++)
  {
    txMsg.tx_data[i]=0;
  }
  memcpy(&txMsg.tx_data[0],&index,2);
  memcpy(&txMsg.tx_data[4],&runmode, 1);
  can_txd();
}
```

## Motor mode parameter write command (communication type 18, control parameter write)
```c
uint16_t index;
float ref;

void motor_write(uint8_t id, uint16_t master_id)
{
  txCanIdEx.mode = 0x12;
  txCanIdEx.id = id;
  txCanIdEx.res = 0;
  txCanIdEx.data = master_id;
  txMsg.tx_dlen = 8;
  for(uint8_t i=0;i<8;i++)
  {
    txMsg.tx_data[i]=0;
  }
  memcpy(&txMsg.tx_data[0],&index,2);
  memcpy(&txMsg.tx_data[4],&ref,4);
  can_txd();
}
```

Function code 0x3024 is driver chip fault code 1. The specific faults are as follows:

| Bit | Field | Type | Default | Description |
|-----|-------|------|---------|-------------|
| 10 | FAULT | R | 0b | Logic OR of FAULT status registers. Mirrors nFAULT pin. |
| 9 | VDS_OCP | R | 0b | Indicates VDS monitor overcurrent fault condition |
| 8 | GDF | R | 0b | Indicates gate drive fault condition |
| 7 | UVLO | R | 0b | Indicates undervoltage lockout fault condition |
| 6 | OTSD | R | 0b | Indicates overtemperature shutdown |
| 5 | VDS_HA | R | 0b | Indicates VDS overcurrent fault on the A high-side MOSFET |
| 4 | VDS_LA | R | 0b | Indicates VDS overcurrent fault on the A low-side MOSFET |
| 3 | VDS_HB | R | 0b | Indicates VDS overcurrent fault on the B high-side MOSFET |
| 2 | VDS_LB | R | 0b | Indicates VDS overcurrent fault on the B low-side MOSFET |
| 1 | VDS_HC | R | 0b | Indicates VDS overcurrent fault on the C high-side MOSFET |
| 0 | VDS_LC | R | 0b | Indicates VDS overcurrent fault on the C low-side MOSFET |

Function code 0x3025 is driver chip fault code 2. The specific faults are as follows:

| Bit | Field | Type | Default | Description |
|-----|-------|------|---------|-------------|
| 10 | SA_OC | R | 0b | Indicates overcurrent on phase A sense amplifier (DRV8353xS) |
| 9 | SB_OC | R | 0b | Indicates overcurrent on phase B sense amplifier (DRV8353xS) |
| 8 | SC_OC | R | 0b | Indicates overcurrent on phase C sense amplifier (DRV8353xS) |
| 7 | OTW | R | 0b | Indicates overtemperature warning |
| 6 | GDUV | R | 0b | Indicates VCP charge pump and/or VGLS undervoltage fault condition |
| 5 | VGS_HA | R | 0b | Indicates gate drive fault on the A high-side MOSFET |
| 4 | VGS_LA | R | 0b | Indicates gate drive fault on the A low-side MOSFET |
| 3 | VGS_HB | R | 0b | Indicates gate drive fault on the B high-side MOSFET |
| 2 | VGS_LB | R | 0b | Indicates gate drive fault on the B low-side MOSFET |
| 1 | VGS_HC | R | 0b | Indicates gate drive fault on the C high-side MOSFET |
| 0 | VGS_LC | R | 0b | Indicates gate drive fault on the C low-side MOSFET |