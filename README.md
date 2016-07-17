# h's homemade Motion Control: firmware for Arduino plus Canon EF Lens
Introduction

This code is designed to be uploaded to an Arduino, which is in turn connected to a Canon EF-series autofocus lens. The microcontroller can then be connected to a Mac via USB, and the Motion Control
Server app run, which will allow full (and kinematically safe) manual control of the lens focus via on-screen sliders, and the playback of 
sequences created in Blender.

This firmware uses undocumented Canon protocols to control the lens, and has variable levels of success depending on the actual lens
used. For best results, use a Canon ring-type Ultrasonic lens; these are the only ones that seem to offer good repeatability. I get excellent results
with a Canon EF-S 17-55 IS USM lens. (Note that not all lenses marked "Ultrasonic" actually offer the true benefits of a ring-type motor)

This code does nothing on its own, and needs the Mac app in order to do anything useful. 

To see it working, watch this: https://www.youtube.com/watch?v=rWuKmWKicro
For the server code, see this project: https://github.com/howiemnet/MotionControl

Warnings

This code contains more bugs than working lines of code. I will not be offering support or help in understanding
what the hell is going on beyond what I can get written up for my blog. It's just too big and complex a project: it relies 
on not just this app working correctly, but the right hardware wiring up, and the appropriate firmware running on
the various microcontrollers involved.

That said, it's my intention to try and tidy this up and document it well enough that the project will be reproducable, in the 
hope that others may be able to hack together motion control systems without having to re-invent these wheels.

A lot more documentation will come: this is just the first commit to get it all up there - feel free to poke around and laugh
at my appalling coding style. Just keep it to yourself, y'all

:)

h 17/7/2016
