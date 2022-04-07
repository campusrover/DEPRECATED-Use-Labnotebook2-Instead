# GPS Knowledge
## August Soderberg
### This is a collection all current knowledge I have found in my GPS research. Some sections take a very high level approach and others are quite technical. Please read prior sections if any sections are difficult to understand.

## How does GPS work?
The high level of how GPS functions relies on several GPS satellites sending out a radio signals which encode the time at which the signal was sent and the position of the satellite at that time. You should imagine this radio signal as a sphere of information expanding out at the speed of light with the satellite, which emitted the signal, at the center.

If I were just looking at a single satellite, I would receive this radio signal and be able to calculate the difference in time between the moment I received it, and the time the signal left the satellite which again is encoded in the radio signal. Let’s say I calculate that the signal traveled 10,000 miles before I received it. That would indicate to me that I could be in any position in space exactly 10,000 miles away from the satellite which sent the signal. Notice that this is a giant sphere of radius 10,000 miles; I could be standing anywhere on this massive imaginary sphere.  Thus, GPS is not very useful with a single satellite.

Now let’s say I receive a signal from 2 satellites, I know their positions when they sent their messages and the time it took each message to reach me. Each satellite defines a sphere on which I could be standing, however, with both spheres I now know I must be somewhere on the intersection of these two spheres. As you may be able to picture, the intersection of two spheres is a circle in space, this means with 2 satellites I could be standing anywhere on this circle in space, still not very useful.

Now if I manage to receive a signal from 3 satellites, I suddenly have three spheres of possible locations which all intersect. Because 3 spheres will intersect in a single point, I now have my exact point in space where I must be standing.

This is how GPS works. The name of the game is calculating how far I am from several satellites at once, and finding the intersection; luckily for us, people do all of these calculations for us.

## How many GPS satellites are there?
This is a bit of a trick question since technically GPS refers specifically to the global positioning satellites which the United States have put in orbit. The general term here is Global Navigation Satellite System (GNSS) which encompasses all satellite constellations owned and operated by any country. This includes the US’s GPS, Russia's GLONASS, Europe's Galileo, and China's BeiDou. Any GPS sensor I reference here is actually using all of these satellites to localize itself, not just the GPS constellation.

## Why is GPS inaccurate?
There is a big problem with GPS. The problem lies in the fact that the radio signals are not traveling in a vacuum to us, they are passing through our atmosphere! The changing layers of our atmosphere will change the amount of time it takes the radio signal to travel to us. This is a huge problem when we rely on the time it took the signal to reach us to calculate our distance from each satellite. If we look at a really cheap GPS sensor (there is one in the lab, I will mention it later) you will find that our location usually has an error of up to 20 meters. This is why people will “correct” their GPS signals.

## Broad Hardware overview
There are two parts to every GPS sensor: the antenna and the module. The GPS antenna is responsible for receiving the radio signals from the satellites and passing them to the module which will perform all of the complex calculations and output a position. High quality antennas and modules are almost always purchased separately and can be quite expensive. There are also cheap all in one sensors which combine the antenna and module into something as small as a USB drive.

### Multi-band vs single-band antenna
Satellites transmit multiple radio frequencies to help the GPS module account for the timing errors created by our atmosphere. Multi-band GPS means that you are listening to multiple radio frequencies (L1 and L2 refer to these different radio frequencies emitted by a single satellite) which can improve accuracy overall, single-band GPS is not as good. Keep in mind that both the antenna and the module will need to be multi-band and both units will be significantly more expensive because of that.

## Plugging a GPS sensor into a linux computer
When you plug any kind of GPS sensor into a linux computer using a USB cable you will see the device appear as a serial port. This can be located in the directory `/dev` most of the time it will be called `/dev/ttyACM0` or `/dev/ttyACM1`. If you run `$ cat /dev/ttyACM0` in your terminal you will see all of the raw information coming off of the GPS sensor. Usually this is just NMEA messages which are what show you the location of the GPS as well as lots of other information.

### NMEA messages
NMEA messages provide almost all of the information about our GPS that we could desire. There are several types of NMEA messages all with 5 character names. They will all begin GP or GN depending on whether they are using just the GPS constellation or all satellite constellations, respectively. Sometimes the first 2 characters are noted Gx for simplicity. The last 3 characters will show you the type of message. [Here is a complete list of their definitions.](http://aprs.gids.nl/nmea/#interp) The most important message for us is the GxGGA message, the complete definition of which you can view in the previous link. It will include the latitude, latitude direction (north or south), longitude, and longitude direction (east or west). There are several different ways of writing latitude and longitude values but online converters can convert between any of them, and they can always be viewed on Google Maps for verification.  
The other important piece of information in the GxGGA message is the "fix quality". This value tells you what mode your GPS is currently operating in. 0 indicates no valid position and 1 indicates uncorrected position (this is the standard GPS mode). 2, 4, and 5 are only present when you are correcting your GPS data, more on what this means later.  
You can use [this Python script](https://github.com/augustSoderberg/rover_files/blob/master/nmea/gps.py) to read all of the NMEA messages and print relevant data to console. Obviously this can be edited to do much more complex things with this data.


## GPS Correction

### Non-corrected GPS data
For less than $20 you can buy a cheap plug and play GPS sensor which does not attempt to correct its GPS data in any way. We purchased the *G72 G-Mouse USB GPS Dongle* to take some preliminary results. Below we see 5 minutes of continuous GPS data (in blue) taken from a fixed GPS location (in red). I will note it was a cloudy rainy day when the data was recorded and the true GPS location was under a large concrete overhang outdoors near other buildings. This is a particularly difficult situation which lead to the larger than normal maximum inaccuracy of ~25 meters. 

![GPS data](https://i.imgur.com/5wtUIYS.png)

NOTE: no matter how fancy or expensive your GPS sensor is, if it is not being corrected by some kind of secondary remote device, you will not see good accuracy. This is confusing because a lot of GPS sensors tout their "centimeter accuracy in seconds" which would imply you could just plug it in and achieve that accuracy. 

**There are no shortcuts, for centimeter accuracy you need to correct your GPS data with an outside source.**

### How to correct GPS data
The most common and most accurate way to correct GPS data is by utilizing two GPS sensors in a process called Differential GPS (DGPS). There are three ways to achieve a differential GPS system according to [RACELOGIC](https://bit.ly/3CyITvP):

* SBAS – Correction messages are sent from Geostationary Satellites, for example, EGNOS or WASS.
* RTCMv2 – Correction messages are sent from a static base station, giving 40 – 80 cm accuracy.
* RTK – Correction messages are sent from a static base station signal giving <2cm accuracy on RTK enabled units. 

We will ignore SBAS and RTCMv2 (see above source for more detail) and focus entirely on RTK correction because it is the most common and most accurate differential GPS correction method.

RTK stands for Real-Time Kinematic and provides ~1cm accuracy when it is set up properly and operated in reasonable conditions. This is our ticket to highly accurate GPS data.

### How RTK works
RTK correction relies on two GPS sensors to provide our ~1cm accuracy. One sensor is part of the "base station" and the other is a "rover".

### Base station
The base station consists of a GPS sensor in an accurately known, fixed location on earth which is continually reading in the radio signals from the satellites. The goal of the base station GPS is to compute, in real time, the error in the amount of time it takes the radio signal from each satellite to reach the base station. This is an incredibly complex calculation to figure out which timing errors each individual radio signal is experiencing. We cannot simply say that the measurement is off by 4 meters and assume that all nearby GPS sensors will experience the same 4 meter error vector. The base station computer must look at each satellite signal it is using to calculate location, look at the total error in location, and then reverse engineer the timing error that each radio signal exhibits. (Accurate GPS requires 3-4 different satellites to determine location, our calculation will thus produce at least 3-4 timing error values, one for each satellite).

The base station will then send these timing errors in the form of an RTCM message (this is the standard RTK error message) to the "rover" so that the rover can perform its own calculations based on which satellites it is currently using. This will ultimately achieve the ~1cm accuracy.

To summarize, RTK correction requires a fixed base station to determine the error in the amount of time it takes each radio signal from all satellites in view to reach the sensor. It then sends this list of errors to the rover GPS. The rover GPS will look at all of the radio signals it is using to calculate its position, adjust each time value by the error sent from the base station, and calculate a very accurate position.

### Base station types
There are two ways to acquire RTK corrections. You can either set up a local base station or you can utilize RTK corrections from various public or subscription based base stations around the country.

#### Why you would want to set up a local base station
The subscription based base stations are often quite expensive and difficult to find, the good news is that most states have state owned public base stations you can receive RTK correction data from, [here is the Massachusetts public base stations site.](https://www.mass.gov/how-to/the-massachusetts-continuously-operating-reference-station-network-macors) and [here is a national Continuously Operating Reference Stations map](https://www.ngs.noaa.gov/CORS_Map/) The problem is that these base stations are often old and not very high quality. They often use solely single-band antenna which means that to have accurate RTK correction you need be within 10km of the public base station and the correction values get drastically better the closer you are. If you set up your own base station you will be able to use multi-band signals for higher accuracy, you will be much closer, and this is where you will see ~1cm accuracies. That being said, Olin College of Engineering uses public base stations for their work.

## Setting up a local base station
This will take you through the process of setting up a local Brandeis base station.

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
All configuration of the GPS module can be done while connected to a Windows computer running u-center which is a u-blox application. Sadly u-center only runs on Windows. This configuration is important because it will establish whether your GPS module is a rover or a base station and will allow you to set the base station's known location etc.

### Physical location of the base station
The base station will need to have a very precise known location for the antenna. This should be as close to your rover on average as possible. To find the exact location, you can use the Survey-in mode on the GPS or use a fixed location determined by Google Maps, the configuration video will cover this.

### Sending RTK corrections from base station to the rover
Your base station will output RTCM messages which are the standard RTK correction messages which a rover type GPS module will be able to use to correct its own GPS data. These RTCM messages will be outputted over the serial port to the base station Raspberry Pi and you will need to set up some kind of messaging protocol to get these messages from the base station to the rover. I recommend using rtk2go.com to handle this message passing. More on this in the configuration video.

### Receiving RTK corrections
Once the rover u-blox module is continually receiving RTK corrections as RTCM messages, it will use these messages to perform calculations and in turn output over serial port the ~1cm accurate GPS data in the form of an NMEA message. These NMEA messages are simple to parse and will clearly provide latitude and longitude data as well as a lot more information for more complex applications. The Raspberry Pi will be able to read these messages (as described in [the video above](https://youtu.be/LVRXa8pX0Oc)) and now you have incredibly accurate GPS data to do with as you wish.

### Credentials
For rtk2go.com, username: asoderberg@brandeis.edu
password: [the standard lab password]

For macorsrtk.massdot.state.ma.us, username: asod614
password: mzrP8idxpiU9UWC

### NTRIP Client script
`/home/rover/RTKLIB/app/str2str/gcc/str2str -in ntrip://rover:ROSlab134@rtk2go.com:2101/brandeis -out serial://ttyACM0:115200`

### Configuration and hardware overview video
https://youtu.be/qZ2at1xV8DY