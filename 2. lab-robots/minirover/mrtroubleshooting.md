# Troubleshooting

**Author: Pito Salas**

## Gemeral

* An excellent and very detailed [troubleshooting guide for the Raspberry Pi](https://elinux.org/R-Pi_Troubleshooting)

## LED on the Red Board

* Blinking green is the “starting up” indication.
* Solid green is the “ready” indication.*
* Solid yellow is the “low battery” caution indication.
* Solid red is the “low battery” warning.
* Blinking red is the “shutting down” indication.
* Blinking purple is (AFAIK) the “I don’t know what is going on” indication, but I could be wrong.

But, note that all of these indications are only valid when using a Dexter Industries / Modular Robotics operating system. (i.e. Raspbian for Robots, GoPiGo OS, or Dexter OS). It will continue to blink green when using any other O/S. The Minirover configuration is Ubuntu! Also see: https://www.dexterindustries.com/GoPiGo/get-started-with-the-gopigo3-raspberry-pi-robot/2-connect-to-the-gopigo-3/

## Python default

To Change the default python on ubuntu (assuming you want it to be python3.4)

`sudo update-alternatives --install /usr/bin/python python /usr/bin/python3.4 1`

## Check whether spi is working on the pi

```pythons = 
>>> import spidev
>>> spi = spidev.SpiDev()
>>> spi.open(0,1)
Traceback (most recent call last):
  File "<stdin>", line 1, in <module>
PermissionError: [Errno 13] Permission denied
```
