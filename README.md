# openGaugeSimplified
MPG from car using arduino (ISO9141 k-line protocol) 


While a lot of people are making interfaces to connect an Arduino microcontroller to their cars, I wanted to get in on the action. Unfortunately for me, my car is a 2005 Toyota Camry which uses the older K-line interface for communication. Since 2008, all cars went to CAN BUS standard which is what all of the Arduino shields available are made to work with. The hardware available will not work with older cars and there is not much information available on the older OBD interfaces for pre-2008 vehicles.

Hardware you will need
Arduino Uno Microcontroller $2
MC33290, ISO K Line Serial Link Interface Chip (converts the 12 volt logic of the car computer to 5 volt that is compatible with the Arduino. Essentially takes the place of the Can Bus shield) $2
SOT23 to DIP10 Adapter PCB Board Convertor (makes hand soldering to SMD chip possible) $3
LCD display $10
OBD port connector $8
Wires $4


Connecting the Arduino to the Car
Step 1: prepare OBD cable

If you bought a cheap OBD cable it is probably wired to connect to a CAN BUS enabled vehicle. You will need to cut it open and make sure the wires connect to pins 4, 7, and 16 of the OBD port. I recommend using a multimeter to check the wiring.

Step 2: solder the MC33290 chip

The MC33290 comes in a SMD mount package which is not so useful to us because the pins are small and break easily. Solder the chip to a DIP adapter board so you can solder wires to it reliably.

Step 3: Connect all wiring between components

I used a prototyping breadboard and prototyping wires to connect everything at first.

Step 4: Upload source code to Arduino board

Download the source code on github: 

The code is based closely off the obduino project found on github.





How to Calculate Miles Per Gallon MPG from MAF (Mass Air Flow Sensor)
It is a common feature on newer cars to display information like gas mileage and average fuel consumption for a trip. How does the car know how much fuel is being consumed at any given moment? In this case, we are calculating the figure using a commonly available PID code MAF or mass air flow rate. It tells us how much air is coming into the engine in grams per second. Assuming the engine is attaining an ideal air to fuel ratio as well as a few other constants, we can use an equation to calculate fuel burned per second and convert it to miles per gallon:

MPG = (14.7 * 6.17 * 4.54 * VSS * 0.621371) / (3600 * MAF / 100)

14.7 grams of air to 1 gram of gasoline - ideal air/fuel ratio
6.17 pounds per gallon - density of gasoline
4.54 grams per pound - conversion
VSS - vehicle speed in kilometers per hour
0.621371 miles per hour/kilometers per hour - conversion
3600 seconds per hour - conversion
MAF - mass air flow rate in 100 grams per second
100 - to correct MAF to give grams per second
