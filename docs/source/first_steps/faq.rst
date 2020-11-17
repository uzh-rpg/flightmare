.. _faq-first-steps:

Frequently Asked Questions
==========================

How can we get the calibration matrix?
--------------------------------------

You don't extract the calibration matrix from Unity, instead, it is calculated on the client-side (ROS/C++).

Since both the image dimensions (width x height) and field of view (FOV) are defined by the user. (see https://github.com/uzh-rpg/flightmare/blob/master/flightlib/include/flightlib/sensors/rgb_camera.hpp#L59-L61)

you can compute the focal length using this formula: **f = ( image.height / 2.0 ) / tan( (M_PI * FOV/180.0 )/2.0 )**
and fx=fy. Hence, the camera intrinsic matrix is **[ [fx, 0, image.width/2], [0, fy, image.height/2], [0, 0, 1] ]**.

How can we publish an image?
----------------------------

Follow the example in :ref:`Tutorials <tut-camera>`.

Depth image rendering format results in information loss?
---------------------------------------------------------

The `issue <https://github.com/uzh-rpg/flightmare/issues/51>`_ has been mentioned by another user, but is unfortunately still unresolved.
It's not straight forward to output images that are not in a 8-bit format.


