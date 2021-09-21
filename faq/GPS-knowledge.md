# GPS Knowledge
## August Soderberg
### This is a collection all current knowledge I have found in my GPS research including our plans for setting up a Brandeis Campus GPS base station.

### Stylistic note:
I will use the term GPS on this page for readability but the more accurate term would be Global Navigation Satellite System (GNSS). GPS refers to the satellite constellation owned and operated by the United States but many countries have created global satellite constellations such as Russia's GLONASS, Europe's Galileo, and China's BeiDou. Any "GPS" work we are doing will actually use more than just the United State's GPS constellation.

## GPS Correction
### Why do we need to correct the data?
Each GPS satellite is basically sending radio signals towards earth which encode the exact time that signal was sent. The GPS receiver uses the time it received each radio signal with knowledge of where the satellites are, to find its own position. The primary issue is that the ionosphere and troposphere (primarily) significantly affect the time it takes the radio signal to travel towards earth creating inaccuracies in the final location. These inaccuracies are in the 5 - 15 meter range and without sophisticated GPS correction, that is the best localization any GPS sensor could do.

### Multi-band vs single-band
We will need this information going forward so I will just add this here. Satellites transmit multiple radio frequencies to help shed light on the effects the ionosphere and troposphere are having on the radio signals. Multi-band GPS means that you are listening to multiple radio frequencies which can improve accuracy overall, single-band GPS is not as good. More on this later.

### Non-corrected GPS data
For less than $20 you can buy a cheap plug and play GPS sensor which does not attempt to correct its GPS data in any way. We purchased the *G72 G-Mouse USB GPS Dongle* to take some preliminary results. Below we see 5 minutes of continuous GPS data (in blue) taken from a fixed GPS location (in red). I will note it was a cloudy rainy day when the data was recorded and the true GPS location was under a large concrete overhang outdoors near other buildings. This is a particularly difficult situation which lead to the larger than normal maximum inaccuracy of ~25 meters. 

![GPS data](https://i.imgur.com/5wtUIYS.png)

Important time saving note: no matter how fancy or expensive your GPS sensor is, if it is not being corrected by some kind of secondary remote device, you will not see good accuracy. This is confusing because a lot of GPS sensors tout their "centimeter accuracy in seconds" which would imply you could just plug it in and achieve that accuracy. 

**There are no shortcuts, for centimeter accuracy you need to correct your GPS data with and outside source.**

### How to correct GPS data
The most common and most accurate way to correct GPS data is by utilizing two GPS sensors in a process called Differential GPS (DGPS). There are three ways to achieve a differential GPS system according to [RACELOGIC](https://en.racelogic.support/VBOX_Automotive/01General_Information/Knowledge_Base/How_Does_DGPS_(Differential_GPS)_Work%3F):

* SBAS – Correction messages are sent from Geostationary Satellites, for example, EGNOS or WASS.
* RTCMv2 – Correction messages are sent from a static base station, giving 40 – 80 cm accuracy.
* RTK – Correction messages are sent from a static base station signal giving <2cm accuracy on RTK enabled units. 

We will ignore SBAS and RTCMv2 (see above source for more detail) and focus entirely on RTK correction because it is the most common and most accurate differential GPS correction method.

RTK stands for Real-Time Kinematic and provides ~1cm accuracy when it is set up properly and operated in reasonable conditions. This is our ticket to highly accurate GPS data.

### How RTK works
RTK correction relies on two GPS sensors to provide our ~1cm accuracy. One sensor is part of the "base station" and the other is a "rover".

### Base station
The base station consists of a GPS sensor in an accurately known, fixed location on earth which is continually reading in the radio signals from the satellites and it uses its known location to determine the precise timing error that each satellite signal is experiencing. This is an incredibly complex calculation to figure out which timing errors each individual radio signal is experiencing. We cannot simply say that the measurement is off by 4 meters and assume that all nearby GPS sensors will experience the same 4 meter error vector. The base station computer must look at each satellite signal it is using to calculate location, look at the total error in location, and then reverse engineer the timing error that each radio signal exhibits. (Accurate GPS requires 3-4 different satellites to determine location, our calculation will thus produce at least 3-4 timing error values, one for each satellite).

The base station will then send these timing errors in the form of an RTCM message (this is the standard RTK error message) to the "rover" so that the rover can perform its own calculations based on which satellites it is currently using. This will ultimately achieve the ~1cm accuracy.

### Base station types
There are two ways to acquire RTK corrections. You can either set up a local base station (by the time you are reading this we might have a base station on the Brandeis Campus) or you can utilize RTK corrections from various public or subscription based base stations around the country.

#### Why you should set up a local base station
The subscription based base stations are often quite expensive and difficult to find, the good news is that most states have state owned public base stations you can receive RTK correction data from. The problem is that these base stations are often old and not very high quality. They often use solely single-band antenna which means that to have accurate RTK correction you need be within 10km of the public base station and the correction values get drastically better the closer you are. If you set up your own base station you will be able to use multi-band signals for higher accuracy, you will be much closer, and this is where you will see ~1cm accuracies. 

## Setting up a (great and inexpensive) local base station
This will take you through the process we plan to follow (or may have already followed) to set up the Brandeis local GPS base station. 

Even if you don't plan on following our steps, this includes a ton of information and notes on things that will help inform any GPS application you are interested in.

### GPS module
The GPS module is what processes all of the information read in from the antenna and gives you actual usable data. For cost and performance reasons we have selected the [ZED-F9P module](https://www.u-blox.com/en/product/zed-f9p-module) from u-blox. More specifically, we have selected the developer kit from SparkFun which is called the [SparkFun GPS-RTK2 Board - ZED-F9P](https://www.sparkfun.com/products/15136) which is a board containing the ZED-F9P module and has convenient ports attached for everything you will need to plug in.

Important information about this board:
* A GPS-RTK2 board and a GPS-RTK board are not the same! Do not mix RTK2 and RTK, it will not work.
* The board must be connected to an antenna (more below) to receive the radio signals.
* This is a multi-band module which will allow us to have much more accurate data even if our rover goes near building, under trees, or far away from the base station.
* The board plugs into a Raspberry Pi with a USB cable and is powered and sends data through that single cable.

We will require two of these boards, one for the base station and one for the rover.

### Antenna
We need a quality multi-band antenna to receive the multi-band signal, these can get very fancy and very expensive, we will be using this [ArduSimple antenna.](https://www.mouser.com/ProductDetail/ArduSimple/AS-ANT2B-SUR-L1L2-25SMA-00?qs=vHuUswq2%252BswBsit1qMDm7Q%3D%3D)

If you use the same antenna on your base station and your rover it will marginally improve accuracy since the noise characteristics will be very similar. 

### Communication between Raspberry Pi and GPS module
The GPS module will send data through USB to the Raspberry Pi and appear as serial port. You can watch [this video](https://youtu.be/LVRXa8pX0Oc) to see what these GPS communications look like to the Raspberry Pi and how to process them in a python script.

### Configuring the GPS module
All configuration of the GPS module can be done while connected to a computer running u-center which is a u-blox application. You will not need to figure the GPS module while it is connected to the raspberry pi. This configuration is important because it will establish whether your GPS module is a rover or a base station and will allow you to set the base station's known location etc.

### Physical location of the base station
The base station will need to have a very precise known location for the antenna. This should be as close to your rover on average as possible. To find the exact location of your antenna you can either hire a surveyor, find a friend who is really into geological surveys and ready to do some math, place the base station directly on top of a known US geological survey marker of a known location, or simply use the u-center application to put your GPS module into Time Mode and then select Survey-In which will record GPS data for a long period of time to estimate its own known location. You will need this precise location to properly set up your GPS system.

### Sending RTK corrections from base station to the rover
Your base station will output RTCM messages which are the standard RTK correction messages which a rover type GPS module will be able to use to correct its own GPS data. These RTCM messages will be outputted over the serial port to the base station Raspberry Pi and you will need to set up some kind of messaging protocol to get these messages from the base station to the rover. A lot of companies have services that will do this for you, or you could set up something like an MQTT server/client.

### Receiving RTK corrections
Once the rover u-blox module is continually receiving RTK corrections as RTCM messages, it will use these messages to perform calculations and in turn output over serial port the ~1cm accurate GPS data in the form of an NMEA message. These NMEA messages are simple to parse and will clearly provide latitude and longitude data as well as a lot more information for more complex applications. The Raspberry Pi will be able to read these messages (as described in [the video above](https://youtu.be/LVRXa8pX0Oc)) and now you have incredibly accurate GPS data to do with as you wish.