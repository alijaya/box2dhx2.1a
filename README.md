Box2D 2.1a finally ported!!! :D...

But there is some known bugs... The Continous Collision Detection is bugging. So be sure to turn off the CCD feature with world.SetContinuousPhysics(false);

If you want to send a patch for fixes, contact me in alijayameilio[at]gmail[dot]com or just hit the issues. I will be glad if you help me :D

For complete information about Box2D 2.1a you can see in http://www.box2dflash.org/

There are differences between the flash version and haXe version... this is the list:

1. the package name is in lower case
2. the class name is in upper case (so it's B2World not b2World)
3. Number.MAX_VALUE and Number.MIN_VALUE is changed to B2Math.MAX_VALUE and Number.MIN_VALUE respectively

For note... this library hasn't been optimized, so the next thing must do is:

1. optimized this library
2. change Vector to Array or another kind container, so it can be crossplatform