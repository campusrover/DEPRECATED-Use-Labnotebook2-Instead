# Validating

When a new robot is built, there are a number of steps needed to see that it is properly configured.

## Encoders

Encoders need to be connected correctly. The left and right can be swapped, and also they may be backwards. These issues lead to very crazy and incomprehensible behavir, so it's best to check them first if your new robot is acting weird.

## Motors

Motors also need to be connected correctly. They can be left-right swapped or forward-reverse swapped. It is worth again to test them separately. The kind of incomprehenible behavor from this kind of problem is totally different from the one when the encoders are backwards or swaps.

## Front and back

It is very important that you know what the front of the robot is. Otherwise nothing makes sense. Our robots have the big wheels in front and the casters in back.

## IMU

The IMU is connceted via an I2C Quick Connector to the Teensy. We have seen problems when the IMU doesn't work that were caused by the placement or length of the wire, so keep an eye out for that case.

