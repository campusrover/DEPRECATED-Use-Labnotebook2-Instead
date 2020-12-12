### Intro - Pre-built models in Gazebo

A lot of times we may use the simple pre-built models within Gazebo to simulate real world objects without putting much effort in mimicing the details of the objects.
At the top tool bar, you may click and drag either a cube, cylinder or sphere into the model. You can easily size the object by changing its configuration by selecting 
it and entering information on the left tool bar but you may wonder how to actually change the visual of such objects.

### Changing the color of basic model in Gazebo

Although this simple tutorial will help you learn how to change the visual of the object, you may follow similar stepes to alter other physical properties without the 
need to actually dig into the verbose XML files.

First, you would need to highlight the obejct and right click. Select the option `Edit Model`. You have now entered the model editor. Select the object again and right click
the option `Link Inspector`. You should see something like this:

![alt text](https://github.com/campusrover/labnotebook/blob/master/images/LinkInspector.png "Link Inspector")

Click on Visual tab and scroll down until you see three options for RGB input - Ambient, Diffuse and Specular. Ambient refers to the color you would see if no light is
directly pointing at the object, which is the color you would see if the object is in shadow. Diffuse refers to the color you would see if there is a pure white light
pointing at the object, which is the color you would see if the object is in direct sunlight. Specular deals with the color intensity of the reflection, something we may
not be interested in for the sake of changing color of the object. 

Now you have the option to enter the RGB range or you can click on the three dots to the right and bring up the color panel for easy color selection.

![alt text](https://github.com/campusrover/labnotebook/blob/master/images/ChangeVisual.png "Color Panel")

The diffuse will change automatically with ambient color. This may be problematic sometimes if you are trying to create a mask for camera image as the HSV value would
change for the object depending on the angle. To solve this problem you can manually change the ambient to match with diffuse.

Save the model and you would see the color of your cube changed! Now keep doing your OpenCV masks!

![alt text](https://github.com/campusrover/labnotebook/blob/master/images/ColorChangedObject.png "Colored Object")
