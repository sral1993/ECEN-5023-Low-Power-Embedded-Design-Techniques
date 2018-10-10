# A Solar Powered State Highway Speed Detector
<br>
<p> <b> OVERVIEW </b> </p>

This application intended to measure the speed of every vehicle that passes on a state highway and check if the speed exceeds a threshold. If so, the respective vehicle  number plate is transmitted using Zigbee which is then stored in an NFC Tag. The data could be retrieved any time using an NFC enabled phone.
<br>
<img src="https://github.com/sral1993/ECEN-5023-Low-Power-Embedded-Design-Techniques/blob/master/OurPCB.png" alt="OurPCB"> 

<p> <b> MOTIVATION </b> </p>

People often tend to speed up in highways and not obey the speed limits which often result in accidents. According to data compiled by the National Highway Traffic Safety Administration (NHTSA), in 2016, 37,461 people were killed in 34,436 crashes, an average of 102 per day.<br>
(https://en.wikipedia.org/wiki/List_of_motor_vehicle_deaths_in_U.S._by_year#Motor_vehicle_deaths_in_U.S._by_year)

This calls for enforcing a discipline in staying within speed limit that can only be done by issuing exorbitant speeding tickets by the cops. This led to the idea of developing a module that would measure speed and provide information such as the number plate of the car to the cop using NFC Tags, that would allow the cop to issue a speeding ticket.

<br>
<b> HARDWARE FUNCTIONAL BLOCK DIAGRAM  </b>

 <br>
 <p> <b> SPEED DETECTOR HARDWARE COMPONENTS </b> </p>
 <b> Power Side: </b>
 <br>
1.	PMIC - BQ25570 <br>
2.	Energy Storage Element- Ultralife UBP001	<br>
3.	Solar Cell (Primary Energy harvesting source) - MikroElektronika MIKROE-651  <br>
4.	Micro USB 2.0 Charging (Secondary Energy harvesting source) - Amphenol FCI 10103594-0001LF  <br>
5.	Bulk Capacitance - GRM31CC80G476ME19L <br>

<b> Digital Side: </b> <br>
6.	MCU - Pearl Gecko EFM32PG12B500F1024GM48-B   <br>
7.	Ultrasonic Sensor-LV MaxSonar EZ4-Narrow Beam  <br>
8.	NFC NT3H2111 <br>
9.	XBee Module(series 1 : 802.15.4) <br>
10.	MOSFET Based Load Switch - TPS22922  <br>
11.	ESD Protection Device - Semtech Corporation SMF3.3.TCT  <br>
12.	High Frequency Clock - AVX Corp/Kyocera Corp CX2016DB40000D0FLJCC  <br>
13.	Low Frequency Clock - Abracon LLC ABS07-32.768KHZ-7-T  <br>
14. Miscellaneous library of resitors, capacitors and inductors
<br>

<p> <b> SOFTWARE FUNCTIONAL BLOCK DIAGRAM  </b> </p>
<img src="https://github.com/sral1993/ECEN-5023-Low-Power-Embedded-Design-Techniques/blob/master/Software_Blockdiagram.png" alt="SBD"> 

<p> <b> SPEED DETECTOR USE CASE </b> </p>
<img src="https://github.com/sral1993/ECEN-5023-Low-Power-Embedded-Design-Techniques/blob/master/Energy_Usecase_diagram.png" alt="Use Case"> 

<p> <b> EXCITING APPLICATION FACTOR </b> </p>
	
We intended to provide a low cost solution to speed detection and overspeed control. Vehicle speed could be measured multiple ways, such as using expensive speed sensors and LiDARS which are normally used. However, ultrasonic sensors provide a good range and are low cost too. This led to designing a model with two ultrasonic sensors angled at 90 degrees with respect to each other. The operation engineered is given below:

<br>
<img src="https://github.com/sral1993/ECEN-5023-Low-Power-Embedded-Design-Techniques/blob/master/Speed_Using_Ultrasonic.png" alt="Speed sing US"> 
<br>
1.	Ultrasonic sensor 1 gives distance measured to an ADC channel of the Pearl Gecko <br>
2.	The ADC output value is checked for being greater than 1.8ft. <br>
3.	The timer is initiated <br>
4.	Ultrasonic sensor 2 gives distance measure to another ADC channel of the Pearl Gecko <br>
5.	Once the value is taken, the timer is stopped and speed is calculated using the below formula <br>
 <br>
Speed  =  (Known Distance(D))/Timer value 
<br>
Another impressive factor of our application is the use of load switches to introduce load power management. This is done by directing power (3.3V) to external components only when needed. A load switch is connected to Zigbee and the two ultrasonic sensors and the load switch is controlled using the Pearl Gecko GPIO Pin. This ensures that power is given to modules only when intended to.



  
