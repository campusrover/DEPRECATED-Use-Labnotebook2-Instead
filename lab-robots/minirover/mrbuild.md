# mrbuild.md

## GoPiGo3 Basic Kit

We recommend that you start with a [GoPiGo3 Basic Kit](https://www.dexterindustries.com/store/gopigo3-base-kit/). Build it following the instructions. There is a lot written and a lot to google about it. Here are some useful links:

* [Dexter Forum](https://www.dexterindustries.com/store/gopigo3-base-kit/) - where you can get many questions answered
* [Build your GoPiGo3](https://edu.workbencheducation.com/cwists/preview/26659x) - where you find step by step instructions.
* [MiniRover Purchase Program](http://cosi119r.s3-website-us-west-2.amazonaws.com/content/background/13\_gpg\_purchase.md/) - instructions for students taking Cosi119a at Brandeis University to order a miniRover.

In addition to what you received with the Basic Kit you will need the following to get through the instructions.

* A Raspberry Pi 3+
* A battery pack (with Y-cable and charger)
* A MicroSD card
* A Lidar
* A very short USB cable

The Battery pack is different from what the instructions talk about. The Y-Cable is used to connect the barrel connector of the battery pack with the corresponding battery connector on the "Red Board" which in turn will power the Raspberry Pi.

The robot will come with a microSD card with the dexter software. There is some very very simple calibration that you need to do.

### Battery Pack

* The battery pack is specific in terms of voltage and capacity. Don't substitute it for another one.
* Note that the battery pack needs to be in the "on" position in order to have the charger do anything. And obviously it has to be in "ON" for the robot to work.
* It also comes with a charger and a y-cable. The y-cable is about 10" long and has three barrel connectors on it in a y-configuration.
* We use only two of the 3 ends of the Y-connector. One goes into the barrel connector on the red board, and one goes into the barrel connector on the battery pack.
* We then connect the very short usb cable with one end in the battery pack and the other end in the "power" connection of the Lidar board. The Lidar board is tiny about 1.5" square. On the one side you have the funny Lidar wires and on the other side you have two micro usb connectors. If you look very carefully one is marked data and one is marked power. The one marked data is connected with a short usb cable to one of the usb connectors of the pi. The one marked power is connected with another short usb cable to the battery pack.

#### Camera

You will have bought the camera separately, but the instructions talk about how to mount it to the robot. Make sure it is centered in all three dimensions. A good spot is on the front of the top plexiglass part. It can be do without any extra parts, but [Dexter has this bracket](https://www.dexterindustries.com/store/camera-distance-sensor-mount/) which works nicely too.

#### Lidar

The Lidar needs to be mounted on the top plexiglass part. You will need to drill two holes. Make sure that the Lidar is exactly centered and pointed forward. As far as the drilling: I kept it to a minimum and drilled only two. I think it doesn't matter much. Mostly that it be straight and centered. Note that the narrower part of the Lidar is the front. You want it centered and facing forward (which on the other side of the ball caster.) I drilled holes for the two posts at the back of the Lidar (the wider part.) I don't see that it matters much though, that was what was easiest for me.

![Top View of MiniRover](<topview (1).jpg>) ![Side View of MiniRover](<sideview (1).jpg>)
