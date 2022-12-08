# TrackingElectricalComponent
Projet 3A Sigma. Track Electrical component using an OAK-D POE camera

This script is using tow powerfull algorithms to detect corners on an image : Harris and Shi-tomasi.
After the corners detected, we only delete all corners outside of the detection zone. Like that, we better control what is going on.

![Screenshot from 2022-12-08 15-00-07](https://user-images.githubusercontent.com/90550358/206523758-843f8b7d-4c24-45c6-9367-f38c8150c3bf.png)

How does it work ?
The two powerfull algorithms return the psition of each corner of the image. We delete all corners which are outside of the detection zone and after:

## 1.
In the rest of the script, averages represent position of electrical component centers.
The first detected corner represent the first average.

## 2.
After, we compare this first average with the second corner. If both are closed, we stock the sum of both and the number (of corners) they are on the sum (in a way to make a average at the end) here 2. If both aren't close enough (based on a simple interval) we create a new "average" composed by this second corner value.
We repeat the second step for all corners in the detection zone. In fact, we do not want to create a lot of new averages (because they reprsent a component) each time the tested corner isn't close to the fisrt average, so we parcour all the averages and if the tesed corner isn't close to any averages, we coonsidere as a new one (and by the way as a new component).

## 3.
We can now add the found averages on the preview image and watch the result.

## précision regarding the camera

The OAK-D POE camera is based on three cameras: One RGB and tow GRAY.
Here, after some tests, we conclude that it was a very good idea to miw the result off the two gray cameras to optain better result.
The main issus it that the two cameras aren't at the same position. If we want to compare both, we need to transform the result gave by one of the two. We decided to transform the result from the right camera. To find the transformation, it is very simple. We put an object (with corners) in front of the camera and get the postion of the same corner on the two camera. The different on x and y axis represent the transformation.

## Precision regarding averages at the end

If you understand how the algorithm work, after tested all corners, each average represent an electrical component. In reality, it isn't realy the case because some time, little (and very long) pins of a component can be interpreted by the script as a component. To avoid this issue, at the end of the computation we only keep averages composed by a minimum of corner values
