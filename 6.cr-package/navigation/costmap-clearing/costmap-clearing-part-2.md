# Costmap Clearing 2

I investigated further with the max\_range of camera and found that it was indeed 10 meters. When the camera is more than 10 meters away from an obstacle, the range readings in the `/scan` topic corresponding to the angle of the obstacle will be `nan`. Also, when an obstacle is within the minimum range of camera or the surface of the obstacle does not reflect any laser, the laser readings will be `nan`. These `nan` readings make the move base think there’s something wrong with laser and will not unmark an obstacle once it’s gone... I wrote a filter node called `scan_filter.py` which will replace the `nan` readings with 9.9 \(a number slightly smaller than max\_range\), and publish to a new topic called `/scan_filtered`. Then I passes an argument to the move base in our launch file to make the cost map in move base subscribe to `/scan_filtered`. However, amcl should still subscribe to the original `/scan` topic because localization relies on unfiltered readings.

At first I changed all the `nan` readings to 9.90, but later Alex help me notice that the `nan` readings at the beginning and end of the array should not be changed, because they correspond to the laser being blocked by robot's body. Therefore I chose not to change these `nan` readings.

Now the robot will immediately unmark an obstacle on cost map once it is gone even the camera is out of range.

## _Huaigu Lin 11/21/2018_
