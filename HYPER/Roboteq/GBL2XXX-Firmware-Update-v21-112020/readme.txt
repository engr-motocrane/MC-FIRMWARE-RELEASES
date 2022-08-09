DISCLAIMER:
-----------
Unless it is marked as Beta, the Firmware is tested and validated by Roboteq and is believed to be fault-free. 
The possibility always exists, however, that the particular configuration and/or use condition uncovers a fault 
that escaped our validation test coverage. Always extensively test the firmware under your use conditions prior 
to deploying it in the field.

2.1 11/20/2020
--------------
 - Fixed Temperature measurement fluctuation in SBL1360A and SIM1360.
 - Implemented SBL(M)2XXXTD(S), SDC(M)3XXXTD, SIM(M)2XXXTD(S) and SSM(M)2XXXTDS.
 - Fixed brake functionality (Motor is on) in closed loop modes and with noisy sensor.
 - Fixed CANOpen parameter defaulting when in bus-off error.
 - Fixed erroneous speed measurement when resolver sensor is used.
 - Implemented automatic motor/sensor setup in encoder sinusoidal when recovering from fault state.
 - Fixed false positive short faults in SBL(M)2/SDC(M)3/SIM(M)2/SSM(M)2XXX v1.1 products.
 - Implemented Minimum RPM configuration command.
 - Optimised dead time in all products.
 - Fixed USB packets loss.
 - Implemented SSM1XXX product.
 - Fixed node guarding in stopped mode.
 - Implemented Pulse Command Min Max Safety feature (PMS).
 - Fixed erroneous behaviour at 100% duty cycle for SBL2XXX and SBL1XXXA product families.
 - Fixed stall error to trigger only for the affected motor channel.
 - Fixed intermittent crosstalk between 2 channels in Hall trapezoidal mode.
 - Optimised boot time.
 - Adapted CANOpen Autostart functionality as local signal based on CiA301 standard.
 - Optimised torque control on all modes.
 - Implemented DS402 cyclic sync Torque, Velocity and Position modes.
 - Implmented Raw Redirect, ASCII and RTU, for RS232 and RS485 interfaces.
 - Implemented Modbus RTU for RS232 and RS485 interfaces.
 - Implemented bypass ramp/trajectory.
 - Optimised speed limiting in torque mode.
 - Conformed DS402 FSA according to standard.
 - Introduced new baud rates for RS232 and RS485.
 - Fixed torque loop in position modes.
 - Optimised CANOpen stack response time.
 - Implemented the separation of the PID gains to Speed, Position, Flux and Torque gains.
 - Fixed Minimum speed implementation for Closed Loop Speed and Closed Loop Speed Position modes.
 - Implemented reset application based on CiA301 CANOpen standard.
 - Average current sampling for monitoring change from 64 to 16 samples.
 - Fixed Torque mode in case of speed limiting and when clearing faults.
 - Set the PID gains to be by default 0.
 - First implementation of SST1XXX.
 - Improved accuracy in current, torque and speed commands and queries.
 - Conformed DS402 commands variable size and their functionality.
 - Implemented Consumer Heartbeat Lost Action (CHLA) configuration command.
 - Implemented Consumer Heartbeat Status (CHS) query.
 - Optimisation of the SVM modulator and saturation technique in FOC.
 - Implemented SetVPM scripting function.
 - Fixed noisy pulse reading on hard STO pins.
 - Fixed CANOpen TPDO transmission type and send rate value update.
 - Fixed sensorless commutation on high load command changes.
 - Implemented Index Encoder Home Process for SBL1XXXA.
 - Implemented configurable STO inputs for SoftSTO.
 - Implemented IPM motor control algorithm (MTPA operation) at sinusoidal commutation.
 - Fixed issues in position modes when amps limiting is triggered.
 - Fixed Safety Stop fault reset in Closed Loop Speed Position.
 - Fixed SSI not working when ethernet is activated.
 - Fixed counter roll-over when SSI is used in Closed Loop Speed Position.
 - Fixed Ramp in Velocity modes when DS402 FSA is enabled.
 - Fixed board recognition bug (v1.0/v1.1) in SBL2XXX.
 - Fixed motor/sensor setup when both channels are set to ewncoder sinusoidal mode.
 - Added the Sin^2 + Cos^2 = 1 support.
 - Added the ICAP command support.
 - Fixed variable sizes in CANOpen objects.
 - Fixed CAN filtering in CANOPEN.
 - Fixed Digital Inputs' Level in GBL2XXX.
 - Fixed Resolver sensor sampling.
 - Fixed Safety stop clearance in closed loop Speed Position.
 - Optimised Amps Limiting function.
 - Fixed fault read of MGM through MultiPWM.
 - Fixed false positive short fault in channel 2 for HDC2XXX.
 - Optimised Firmware performance.
 - Fixed false positive short detection in trapezoidal mode with hall sensor disconnected.
 - Fixed false positive Short fault on GBL2XXX and KBL1XXX.
 - Implemented RGBL1XXX v2.3 board support.
 - Fixed erroneous current measurement after continuous use.
 - Fixed Brake engaged in case of fault.
 - Implemented run-away protection in Hall+Encoder mode when encoder counts are lost.
 - Fixed issues in Closed Loop modes in KBL1XXX.
 - Fixed Amps triggering when current is negative.
 - Implemented configurable rate in RoboCAN queries.
 - Implemented MOSFET damage test.
 - Fixed MOTCMD and BLSPEED CANOPEN queries.
 - Implemented Quick Short Detection to float the MOSFETs much quicker and set default to Quick.
 - Implemented power recovery after activating floated MOSFETs for brushless motor controllers.
 - Fixed S command and acceleration/deceleration implementation in Closed Loop Position Relative.
 - Implemented PWM Brake for SBLM2XXX.
 - Implemented Safety Stop run-time command (SFT).
 - Fixed HallC/DIN3 when Molex Input is SSI sensors on SBL1/MBL1XXXA and KBL1XXX.
 - Fixed Heatsink Temperature noise distortion.
 - Fixed the analog input distortion in GBL2/GDC3/GIM2XXX boards.
 - Fixed runaway when changing direction and Power Limit is on in Closed Loop Speed Position.
 - Fixed motor current measurement in single channel controllers (S).
 - Fixed Forward and Reverse Limit for Closed Loop Speed Position.
 - Fixed Runaway issues in case of current control malfunction.
 - Fixed CANOpen boot-up message missing.
 - Implemented Hall sequence error detection.
 - Implemented fail detection in Hall (+Encoder) Sensor Setup.
 - Fixed Forward Limit Switch in Closed Loop Speed
 - Fixed Reference Seek Power.
 - Improved SinCos Sensor Setup.
 - Optimised Hall Sinusoidal mode.
 - Fixed Encoder functionality in XDC2XXX.
 - Fixed Emergency Stop when STO is enabled.
 - Fixed Pulse Input 6 for SIM2360(S)
 - Fixed Counter reset when switching to Closed Loop Speed Position.
 - Fixed spikes in Battery and Controller Volts measurement.
 - Implemented C, CB and CSS change even in closed loop and used as feedback.
 - Enabled DIN9, DIN10 for GBL2xxx, GIM2xxx, GDC3xxx.
 - Implemented Fault Deceleration in Closed Loop Torque mode based on speed.
 - Fixed Runaway when doing Motor/Sensor Setup with Motor Direction Inverted.
 - Fixed runaway when moving from Closed Loop Count Position to Closed Loop Speed mode.
 - Fixed FOC Angle Correction when motor idle.
 - Fixed Emergency Stop when STO is enabled.
 - Implemented CANOpen hardware filters.
 - Implemented Extended RawCAN.
 
2.01 11/14/2019
--------------
 - Fixed script loss bug.

2.0 06/21/2019
--------------
 - Implemented SBLM2XXX, SDCM3XXX, SSMM2XXXS and SIMM2XXX.
 - Optimised MOSFETs protection.
 - Implemented Sensor Phase Shift Angle for SinCos/Resolver sensors.
 - Implemented Sensor Linearity Correction Table Interface for SinCos, Resolver and SSI sensors.
 - Implemented SBL13XXA, SDC3XXX, SIM2XXX(S), SSM2XXXS, KBL1XXX, KSM1660, FSM2360(A)(E)S, GDC3XXX, GIM2XXX, HIM2XXX(S), HBL23XXAS, RGSM1XXX
 - Implemented high voltage products SBL2396(S), HBL23120(A), GBL26120, GDC36120, GIM26120
 - Implemented floating point PID and increrased PID Gain resolution (10^6).
 - Implemented Hall Trapezoidal Tuning.
 - Implemented Sinusoidal Sensor Tuning.
 - Implemented ethernet module detection method.
 - Implemented TCP interface.
 - Implemented RS485 interface.
 - Implemented Modbus (TCP, TCP_RTU, RS232_ASCII, RS485_ASCII).
 - Implemented STO circuit support and soft-STO.
 - Implemented FlowSensor MultiPWM protocols and FLW query.
 - Implemented BMS MultiPWM protocols and BMC, BMF, BMS, and BSC queries.
 - Implemented SSI sensors as feedback sensors and respective queries SS, SSR, CSS, CSR and command CSS.
 - Implemented variable SSI sensor resolution to be configurable.
 - Implemented support of DS402 specification.
 - Implemented Node Guarding, Dynamic PDO Mapping and Transmission Type in CANOpen.
 - Enabled RawCAN in MiniCAN mode and sending RawCAN messages in both MiniCAN and RoboCAN modes.
 - Implemented Fast query log streams using console commands and history.
 - Implemented fault logs.
 - Implemented FlowSensor MultiPWM protocols and FLX, FLY queries.
 - Implemented BMS MultiPWM protocols and BMC, BMF, BMS, and BSC queries.
 - Implemented new sensorless method (BEMF integration) and Closed Loop Speed Mode for sensorless.
 - Implemented Ramp Bypass if Acceleration and Deceleration values are zero.
 - Implemented hardcoded highest command priority for Script and CAN motor commands.
 - Implemented MLX configuration command in order to set which sensors will be connected to molex connector.
 - Implemented Emergency Deceleration in case of fault or safety stop.
 - Implemented outer sensor calibration for RGBL1XXX.
 - Implemented speed limit in torque mode.
 - Implemented over temperature cut off limit (OTL).
 - Implemented Magsensor Tape Cross (MGX).
 - Implemented outer sensor calibration for RGBL1XXX.
 - Increased Flash protection.

1.8 07/12/2017
--------------     
 - Implemented FIM23XX and RGIM18XX for AC Induction Motors.
 - Implemented HBL23XXA, MBL1XXXA and SBL2XXX.  
 - Implemented HBL1696, HBL2396, HDC2496 and HBL120AC.
 - Current Measurement Optimisations for trapezoidal and sinusoidal modes.
 - Implemented Hall Sensor Map for (FBL23XX, MBL1XXXA, HBL23XXA, RGBL1XXX).
 - Optimised Raw Amps Data collection and fixed Short detection for RGBL/RGDC/RGIM1XXX.
 - Restored BADJ to Configuration space (EE_RCBCV).
 - Fixed Short detection Method.
 - Introduced Winding Swap (SWD).
 - Fixed number of script timers for SDC2XXX, RCB503, HDC2XXX (8).
 - Fixed bug on script Atan function.
 - Fixed sign propagation in Script's shift operators.
 - Removed AC Induction functionality from FBL23XX and RGB18XX controllers.
 - Implemented Regeneration Amps Limit and new Amps measurement method.
 - Fixed Amps Trigger bug for channels 2 and 3.
 - Fixed no clock bug when only channel 2 is set as sinusoidal/SPI sensor.
 - Released Resolver for FBL2XXX v2.4.
 - Fixed ANA8, DIN9, DIN10 in FBL2XXX. 
 - Fixed DOUT3, ANA8 in FDC3260.
 - Fixed bug in FM query when encoder stall detection take place. 
 - Optimised Encoder Stall Detection Mechanism. 
 - Implemented Hall Position (HPO) and Hall 60 degrees support0. 
 - Implemented Hall Sensor Map feature.
 - Implemented Battery Amps Averaging.
 - Optimised Hall Sinusoidal and Hall+Encoder Sinusoidal modes. 
 - Fixed Hall/Hall+Encoder Sinusoidal functionality after power-up.   
 - Implemented Anti-Glitch mechanism for Encoder and Hall+Encoder Sinusoidal mode.
 - Fixed minor bug for hall state synchroniation.
 - Optimisation of sensorless runtime under speed and load change.  
 - Fixed memory overflow in FDC3XXX.
 - Fixed Closed Loop Speed mode bug in VBLXXXX and HBLXXXXX boards.
 - Fixed Roll-over cases in case of closed loop speed position.
 - Fixed maximum current and default current limit values for FBL2360(S).
 - Fixed Motor Jerk when loading configuration in closed loop and Encoder as feedback (EncTimer reconfiguration).
 - Fixed Integrator bug and Integrator bleeding in Closed Loop Speed Mode (OldDoPID).
 - Increased PWM Timer resolution.
 - Fixed ICL when accessed via script.
 - Fixed bug in order to execute P command even when Desired Position is same to track.
 - Fixed bug when Current Limit takes place in Closed Loop Position modes.
 - Fixed BND process when no motor connected.
 - Changed default Sinusoidal Angle Sensor (BFBK) to Hall.
 - Fixed DR query.
 - Fixed Battery Volts fluctuation for SBL1XXX.
 - Added Averaging for the Battery Volts in FBL2XXX, FDC3XXX, RGBL1XXX, RGDC1XXX, F3MBL1XXX, XDC1XXX.
 - Modified default undervoltage limits.
 - Fixed SPI Enable functionality.
 - Fixed buffer overflow in encoder sinusoidal when PPR is too big.
 - Fixed Encoder Sinusoidal Mode when PPR is negative.
 - Fixed Motor Direction functionality in Encoder Sinusoidal and Hall+Encoder Sinusoidal modes.
 - Enabled SIN/COS Filters for RGBL1XXX.
 - Fixed Query History when arguments set.
 - Fixed jumps in Closed Loop Count Position when in motion and P is executed.
 - Fixed Timing in CPLD Update in RCB503.  
 - Ignore RoboCAN commands for deactivated nodes.
 - Fixed Stall when CANOpen TPDO sending fails.
 - Fixed MiniCAN TPDO Send Rate roll-over.
 - CANOpen EDS: Fixed 0x200E(MS) structure, 0x2135(FC) variable size and added 0x2136(SL).
 - Fixed CANOPEN TPDO bug when loading configuration (Load from Controller).

1.7 10/13/2016
--------------
- Implemented Sensorless Mode for FBL1XXX, RGBL1XXX and MBL1XXX v4.1.
- Implemented Open Loop Scalar mode for AC Induction Motors for FBL1XXX, RGBL1XXX and MBL1XXX v4.1.   
- Implemented Backup Registers and Time for FBL2XXx, FDC3XXXX, F3MBL1XXX, HBLXXXX, HDC2XXX, RGBL1XXX and RGDC1XXX.   
- Implemented Clear Loop Error Mechanism. 
- Implemented minimal CANOpen Node Guarding. 
- Updated CANOpen interface (v5.0 EDS).
- Implemented Anti-Glitch mechanism for Sinusoidal modes.          
- Optimised FOC mechanism.
- Optimised Hall+Encoder Sinusoidal mode in runtime. 
- Optimised Hall Sinusoidal Mode's Perfomance.  
- Optimised Zero Angle Reference Mechanism (BND).
- Optimised Sin/Cos Sensor Mode's Angle Resolution and Calculation.
- Fixed Motor Direction bug in Closed Loop Speed Position mode.
- Fixed bugs in Close Loop Modes (Resume from Safety Stop, Iteration, Deceleration) 
- Fixed FM bugs (Safety Stop, MDC2XXX).   
- Fixed MS for Closed Loop Tracking Position and Closed Loop Relative Position.
- Fixed RSBR bugs and Inverted USART.
- Fixed CANOpen bugs (AutoStart, Configuration, Processing, S Command).
- Fixed MiniCAN Heartbeat Counter Roll-over. 
- Fixed Pulse Input bugs (Motor Command, Enable/Disable).
- Fixed Sin/Cos and SPI mode bugs (Acceleration/Deceleration, Speed, Interference). 
- Fixed Roborun bug with #EESAV and !EES. 
- Fixed Crash when reseting to default (after power cycling with non-default configuration).
- Fixed Motor Amps sign for FOC supporting boards. 

1.6 04/14/2016
--------------
- Optimised Closed Loop modes (Emergency Stop, Start-up, Track Overflow).
- Fixed Closed Loop Count Position in MDC2XXX V3.0.
- Fixed Single Channel Motor Amps.
- Added BADJ to Calibration (STM32F30X).
- Implemented Encoder Polarity when setting negative PPR.
- Fixed Encoder Stall bug when Stall detection disabled.
- Fixed Status LED on Digital Out.
- Increased Maximum PWM frequency to 50kHz.
- Fixed serial interface crash in RCB500.
- Optimised inputs RCB503 v1.2 (Encoders, Pulse and digital inputs).
- Optimised RoboCAN (buffer overflow, crash).
- Enabled Automatic Bus-Off Recovery in CAN.
- Implemented FC (Read FOC correction) query.
- Implemented ASI (Read Raw Sensor ADC values) query.
- Implemented ICL (Read if Node is alive) query.   
- Fixed DO query.
- Fixed SCC query.
- Implemented Motor Direction Configuration (MDIR).
- Implemented OVH configuration command.  
- Optimised in Sin/Cos Sensor mode. 
- Modified SPOL as Sensor Poles.
- Fixed Action At Min/Max for Hall Sensors.  
- Set Hall Inputs as Pull-Down.  
- Optimised startup in Hall+Encoder Sinusoidal mode.
- Optimised Hall Sinusoidal Mode.                   
- Optimised BND process (6 states, Response, Closed looop bugs).
- Implemented Index Encoder Home Counter for FBL2XXX and SBL1XXX.
- Fixed sin and cos function.
- Implemented Auto recovery system when script crashes.
- Fixed script downloading.

1.6beta 10/16/2015
--------------
- Added library functions sin, cos, sqrt and atan.
- Fixed bug when configuring both Hall and Encoder as feedback (full fix).
- Increased script size to 32k and sript var size to 4k for STM32F30X boards.
- Fixed Count Position in GBL1XXX.
- Added SPOL configuration command (Sensor Pole Ratio).
- Added SCC query (Script CRC32).
- Fixed script download bug.
- Fixed V30/V60 pin bug in F3SDC21XX.

1.5 10/02/2015
--------------
- Fixed sporadical USART crash.
- Fixed USB LED in FBL2XXX and FDC3XXX.
- Implemented Sin/Cos sensore mode.
- Added CTRIM command.
- FIxed SCRO, CR and BCR commands.
- Implemented Stall Detection when Encoder is disconnected.
- Fixed Hall Speed fluctuation.
- Implemented Hall Sinusoidal mode.
- Fixed DI for CANOpen interface.
- Removed noise on startup for BL motors.
- Support of more than one MagSensors in single Controller.
- Implemented Sensor Offset calculation when idle.
- Implemented Amps Limit during BIND process.
- Adapted default configuration for digital outputs.
- Implemented CANOpen for all supporting boards (STM32F30X).
- Fixed USB Serial Number(board type dependent).
- Implemented FOC.
- Fixed Script download overflow bugs.
- Fixed crash when configuring SPI Sensor Mode.
- Fixed overflow and several bugs in Count Position mode.
- Fixed Overvolt error in F3SDC2XXX.
- Fixed motor squeal when enabling encoders and Encoder Counters in FDC3XXX.
- Fixed crash when configuring to Encoder Sinusoidal in MBL1XXX/SBL1XXX.
- Fixed delayed pulse in capture.
- Fixed Short problem for MDC2XXX rev.3.2
- Implemented MiniJ1939 CAN mode.
- Fixed CANOpen bugs concerning node id 32 and TPDOs 3-4.
- Fixed angle adjust and added stall detection in Encoder Sinusoidal mode
- Fixed Battery Amps for single channel boards with only one sensor.
- Fixed deadtime in PWM in FBL2XXX.
- Implemented unified RoboCAN for all boards, RIOX and Magsensor.

1.4b 03/31/2015
---------------
- MCU overheat error no longer sticks
- Remove Short fault when powering up from power control.
- Fixed Battery Amps for single channel boards
- Added error logs for SDCard
- Fixed Amps averaging on FBL2xxx
- Added Inverted USART support
- Added %sopen, %swrit SDCard functions
- Fixed MOSFETS Off on MDC1xxx, XDC2xxx
- Fixed knocking on FBL2360
- Added support for FBL2360v21
- !AC now works same as in ^MAC
- Progressive Angle Advance in Sinusoidal mode
- Patched Motor Amps sign in Sinusoidal mode
- Fixed CANOpen TPDO Send
- Fixed Short Circuit disable on XDC2xxx
- Added MDC1230
- Removed zero search on Sinusoidal with SPI feedback
- Fixed channel 2 speed capture on FBL2xxx
- Fixed abrupt speed change in Position Count mode
- Fixed negative number of brushless poles in Speed Position, and Position Count
- Sinusoidal with SPI sensor mode
- FDC3xxx support

1.4a 11/21/2014
---------------
- Fixed Count Position bugs
- Relaxed MaxPWM Freq to 30kHz
- Detect Amps calibration loss
- Fixed BEE range
- Added Speed Position mode
- Fixed position count mode bug
- Temperature capture on Analog in for RCB500
- Added continuous count position tracking
- Added script download via RoboCAN
- Added user storage in battery backed RAM
- CPLD Programming works in RS232 mode
- Removed Selftest function

v1.4 09/09/2014
---------------
- Restored CPLD Programming in RS232 mode
- Fixed RoboCAN startup problem
- Fixed CANOpen TPDO and RPDO
- Restructured source files
- Reduced filtering on pulse input to allow encoder capture > 50kHz
- Improved fbl2xxx current sense

v1.4 08/01/2014
---------------
- RoboCAN implementation
- Enable pull ups on RC inputs on XDC2xxx
- Fixed crashing firmware during script exec and motor spin on MBLxxx
- Speed capture is averaged over 10ms on all models. Changed HDC, HBL and MDC

v1.3 05/15/2014
---------------
- Fixed Encoder and Brushless soft limit switches
- Fixed Query History bug
- Added script redirection support to TTL serial port on SDCxxxx
- Added Magsensor Gyroscope in multiPWM
- Improved Short Circuit detection & protection on XDC2xxx and MDC1xxx
- Added SENT protocol
- Added 16-point linearization option on inputs
- Added XDC models
- Added support for MBL1xxx v3.0
- Added filter and FIFO on CAN
- Widen PWM motor drive frequency range
- Fixed clicking on MBL/LBL

v1.3 03/03/2014
---------------
- Added up down selftest
- Added !S direct speed command in closed loop speed mode


v1.3 01/09/2014
---------------
- Fixed difference between ch1 and ch2 in position count mode

v1.3 01/07/2014
---------------
- Fixed Count Position mode when high PPR encoder
- Fixed missed stops in Count Position mode

v1.3 12/12/2013
---------------
- Fixed motor flag error

v1.3 12/04/2013
---------------
- Added MBL1330 support
- Added sinusoidal switching to SBL13xx

v1.3 11/19/2013
---------------
- Encoder sinusoidal beta release

v1.3 10/23/2013
---------------
- Increaded absolute max motor acceleration from 100ms to 10ms to max RPM
- Eliminated MOSFail oversensitivity

v1.3 10/10/2013
---------------
- LDC1450/1460 auto detection
- Added MDC2460 support
- Fixed TPDO and RPDO in CANOpen
- Added CTPS config parameter
- Modified CANOpen EDS
- Added ?FM motor status query

v1.2 08/26/2013
---------------
- Fixed LDC14XX pulse capture on RC3 RC4 RC5 RC6
- Added SBL1XXX V2.0 support

v1.2 08/12/2013
---------------
- Added averaging of Ana 1 and Ana 2 on VIAL1450
- Fixed !m 0 0 crashing on single channel models

v1.2 07/20/2013
---------------
- Fixed Amps capture error on -S versions of dual channel controllers
- Changed SDC2150x to SDC2160x

v1.2 07/10/2013
---------------
- Added immediate script autostart configuration

v1.2 06/12/2013
---------------
- Fixed closed loop speed with BL Hall feedback

v1.2 06/10/2013
---------------
- Added 60V detect and models
- Added Track query for position relative mode

v1.2 06/03/2013
---------------
- Increased Encoder PPR limit to 32000
- Fixed Echo Off on TTL serial port of SDC1130/SDC21XX

v1.2 05/31/2013
---------------
- Added MDC/LDC2260 support

v1.2 05/25/2013
---------------
- Enabled CAN on SBL1360 and MBL1650 (requires wire fix. Disables Din5-6)
- Eliminated SBL/LBL/MBL knocking introduced on 3/15

v1.2 05/12/2013
---------------
- Fixed Overvolt, Unser volt and other fault Shut Off on MBL/LBL/SBL
- Fixed return to zero with Pulse feedback
- Fixed CANOpen heartbeat
- Fixed LED flashing pattern while in CANopen
- Updated CANOpen object dictionary
- Fixed CAN_GO and other commands
- Fixed DOut2 on LBL1350 and MBL1650
- Fixed CANopen on SDC21x0N
- Fixed analog capture on LDC225x0
- Fixed negative BL count on MBL,LBL,SBL
- Fixed PIC query
- Fixing count position mode with negative BL pole number
- Change Hall counter direction when negative pole number
- replaced SBL1350 with SBL1360
- Enabled and fixed count position mode using Hall sensors
- Enabled count position mode on LDC1450, 
- Corrected PWM frequency on LDC22x0

v1.2 02/09/2013
---------------
- Added SBL1350 support
- Corrected wrong MDC2xxx indentification bug
- Fixed Microbasic

v1.2 01/24/2013
---------------
- Gradual powerdown at overtemp. Starting at 70o
- Fixed Ch2 default Closed Loop error detect on SDC2xxx
- Fixed microbasic crash when goto and gosub

v1.2 12/04/2012
---------------
- Fixed startup kick on MBL1650/LBL1350
- Fixed microbasic on MBL1650

v1.2 11/27/2012
---------------
- Added 16-message buffer in RawCAN
- 12K Microbasic scripting Flash on HDC/HBL
- fixe 29-bit CAN identifier bug
- Added Microbasic variable inspections
- Fixed calibration issue on LDC1450, LBL1350 and MBL1650
- Current counter value becomes destination when switching to position count mode


v1.2 10/28/2012
---------------
- Added SDCard file delete. SDIR reports file sizes.
- Added LBL/HBL1650 versions 60V and 72V
- Added LDC1450 versions 30V
- Added MDC/LBC versions 30V and HE
- Make current position be destination when switching to closed loop
- Fixed pulse capture on LBL1350
- Fixed limit switches when in Count Position and Position Relative mode
- Reseting integrator at every closed loop mode changes
- Fixed SDCard watchdog timeout, Read bug
- Changed SDCard output redirection

v1.2 10/05/2012
---------------
- Fixed hardware encoder counter when Pulse input 5 is used
- Fixed RawCAN bug in sdc21x0 and MDC2150

v1.2 09/24/2012
---------------
- Added Ferno configs
- Added script output config parameter. Not used yet
- Added 60V and 72V support to HDC/VDC2400
- Added 60V and 72V support to HBL/VBL2300
- Added new FNB2000 parameters
- Added Integrator anti windup
- Fixed Integrator cap

v1.2 09/03/2012
---------------
- Clear Integrator when limit switchs are activated

v1.2 08/25/2012
---------------
- Added option to "no command change" at power up

v1.2 08/15/2012
---------------
- Fixed potential issue with single channel controllers

v1.2 08/14/2012
---------------
- Fixed Ch1 Hall speed capture on HBL2350
- Fixed speed capture accuracy on HBLxxxx
- Fixed ESTOP and MGO

v1.2 08/06/2012
---------------
- Added Serial Port bit rate change
- Fixed smooth change from open loop to close loop position tracking

v1.2 07/27/2012
---------------
- Added support for MagSensor
- Added Digital Filter on pulse inputs

v1.2 07/23/2012
---------------
- Fixed Microbasic bug on LDCxxx LBLxxx and SDC1130
- Added LBL1350 support

v1.2 05/22/2012
---------------
- Fixed potential bug on LDC1450 and SDC1130

v1.2 05/01/2012
---------------
- Fixed acceleration setting and improved in closed loop position
- Added MOSFail safety
- Removed Analog Mode from default command priorities
- Replaced Uncal LED with RunScript LED in Roborun status (new roborun needed)
- Fixed CAN2 Receive mode
- Improved Close Loop Count Position mode
- Added CAN Support & Autoswitch USB/CAN
- Improved USB communication reliability

v1.2 03/29/2012
---------------
- fixed speed capture in SDCxxxx
- fixed 1s DOut active at boot on MDC2250
- Added Auto-Switch CAN-USB on HDCxxxx/HBLxxxx
- Added !BND Spektrum BIND runtime command
- Added negative Hall sensor speed capture on xBLxxx boards
- Fixed pulse captures on MDC2250
- Fixed encoder speed capture on MDC2250 and HBL/HDC models
- Changed MDC2250 max encoder frequency from 254kHz to 508kHz
- Restored jumper detect for SDC2150
- Changed control loop to 4ms and fixed acceleration ramp on SDC1130 and LDCxxxx models
- Fixed feedback on LDC1450
- Fixed pulse capture problem on SDC1130
- Fixed crash in SDC1130 when changing EMOD configuration
- Added user nonvolatile storage using EE config and EES command
- Fixed microbasic getvalue crash bug
- Working beta of MINICAN and RAWCAN
- Fixed close loop bug on BL controllers at low speed

v1.2 12/28/2011
---------------
- Reliable RTC startup
- Extended safety wdog timeout to 3x
- Fixed Update for microbasic getvalue()
- Added Read/Write to battery backed up RAM
- Change set/get time to single 32-bit seconds counter
- Fixed encoder speed capure bug introduced by previous release

v1.2 12/05/2011
---------------
- Added support for new contoller models

v1.2 10/31/2011
---------------
- Fixed overrange bug when changing !s in mode 3

v1.2 10/19/2011
---------------
- Fixed Encoder counter on single channel SDC2130/50
- improved speed capture accuracy in SDC2130/50

v1.2 09/07/2011
---------------
- Added AIC and PIC queries

v1.2 09/01/2011
---------------
- Added Ramp preload command !RMP

v1.2 08/24/2011
---------------
- Corrected Encoder enable at startup on SDC2130

v1.2 08/12/2011
---------------
- Corrected Short Circuit detect on MDC2250

v1.2 08/12/2011
---------------
- Corrected Short Circuit release bug
- Corrected controller names
- Changed default short circuit threshold to 1

v1.2 08/01/2011
---------------
- Fixed pulse capture on RCIN1 on MDC2250
- Beta RC Output feature on MDC2250

v1.2 07/24/2011
---------------
- Disable Ain4 and Pin2 default in single channel SDC2130

v1.2 07/24/2011
---------------
- Fixed SDC2130 Encoder bug

v1.2 07/15/2011
---------------
- Fixed SDC2130 RC input 5

v1.2 07/09/2011
---------------
- Fixed SDC2130 bug with pulse capture
- Fixed LED status on digital output
- Single channel amps measure on MDC2250 and HDC2450
- Amps lim raise to 150A/300A on HDC2450

v1.2 05/08/2011
---------------
- BL Hall speed capture on HDC


v1.2 03/30/2011
---------------
- Fixed Mixed mode in Closed Loop speed control
- Changed to latest StdPeriph & USB libraries

v1.2 03/20/2011
---------------
- Added single channel support on SDC2130/60
- Improved Encoder capture on single channel SDC2130/50

v1.2 03/06/2011
---------------
- Fixed MicroBasic large script size bug

v1.2 02/20/2011
---------------
- Chained EncPos motions

v1.2 02/08/2011
---------------
- Fixes HDC2450S CPLD reporting

v1.2 02/03/2011
---------------
- Changed Soft Encoder 2 Pinout
- Fixed closed loop error

v1.2 02/02/2011
---------------
- Fixed MBL amps offset

v1.2 01/16/2011
---------------
- Assembly version of softencoder
- Enable/Disable softencoder

v1.2 01/14/2011
---------------
- Speed Capture on SDC2XXX
- Spektrum Radio support added

v1.2 01/05/2011
---------------
- Fixed _DIN getvalue
- Added TTL USART on SDC2XXX
- Dual BLDC Speed fix
- Added softencoder
- BLDC negative speed fix
- SDC2XXX & MDC2XXX pulse capture