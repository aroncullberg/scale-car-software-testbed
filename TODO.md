# TODO

tings to improve and or add

--

## Fix how leds work,

 want a dysnamic system that works for any size led and can programtaitcally controll them via state machines and patterns you give it rather than hardcoded shits.

I want to be able to define a led as like type strip, and it has x leds or as type ring and it has x leds in cirlce and then yes/no if it has center led and i can then define patterns for it and have it run those patterns based on the shape of led. 

--

## why duplicates of EscDriver? 

Both managed component EscDriver and the motor component has stuff like dshot_rmt_encoder. which one is most updated, and why are there two?

> Think answer is that the managed component is dead code and i foled it in ot my codebase, fork is gone. 

--

## I want config in files not in code

I wish i could use sdkconfig and the idf.py menuconfig but if i change that i 1. have to reflash the whole thing 2. it requires rebuilding the WHOLE (like all 1k plus build steps) project. 

This makes me look to putting a config on the sd card prehaps that i can then read and change data too. 

Needs more research. 

--

## move things to a config file out of code.

even better if the config file can live on the sd card. will wait because of that posisiblity.

---

## Change over to AM32 esc

Current motor driver related code was written for a 4in1 bluejay (8bit, not important) esc. We are going to prepare for integration of an AM32 esc instead. The AM32 esc will have a featured called RC-car mode enabled. This mode gives it freewheeling and proportioanl breaking capabilities. It splits the DShot throttle range into two 1000-step halves; 48–1047 is one direction, and 1048–2047 is the other, and DShot value 0 is neutral/stop. Uncertain if inbetween those two are idle or not (i.e talking about 48, 1048). Point being is that the esc behaves as if 3d-mode was tunred on on a traditional drone esc. 

We are going to remove thing such as `tick_braking()` and `tick_reverse()`. The two existing functions to drive the motors `throttle(float, Direction)` and `brake(float)` become ill suited for the new esc (imo). Instead i think it is better to give each motor (object) its own direction (enum), and a `drive(float)` & `brake(float)` function. Uncertain about if it should take float or not since its an esp32s3, probably no real reason for it to use float atm but i'll leave it in the todo for now. 

What requires extra attention now is how the brake fucntion behaves, it will send the opsoite of direction to the esc and the esc will brake (as long as it spinning in the direction of *direction*). However when we brake we do not want to start to reverse so we need to make sure we stop sending brake after the motor has reached a standstill, or possibly even a little bit before that. We get motor speed via erpm but its spotty at low rpms and does not tell direction. 

How to brake? We are going to introduce a new channel for this, that so when that is pulled we will brake, even if throttle is on (TBD?). I use a stick radio but will not have center stick to drive a car so i will have forward/reverse on a separate channel. I say this to not make you think we split the radio channel into forward/backwards, that would be stupid.

What we need to consider is KISS telemetry, right now we assume all edt telemetry is of type erpm, and as such cannot support receiving current/temp data over the *singal* wire. If we want that we have two options, change the dshot generator to allow parsing of that and pay a small cut in how often we get rpm updates (probably non-essential since we cant use the data fast enoug anyways) or we can use KISS-telemetry from the 4in1 esc. All 4 esc's on the 4in1 share the same uart so we would have to command each esc to send telemetry in sequence. 

worth mentioning is that `Motor::rpm()` maps several things (some of which shouln't be mapped) to the 0. need a way to better do this. but is tbd. 

---

## SD card over SD 1-bit

Add an SD card to the vehicle as the storage target for system logging.

- It should use SD mode in 1-bit. (SPI is slower and takes up more pins)
- Should use some kind of filesystem (which one is TBD) so that the card can be pulled and read on any laptop.
- telem offloading, either by sd card or connect cable to usb (labled usb not com) port and have it show up as usb drive.

---

## Architect a better way to log data

There is no guarantee that the logging will have a SD card to dump data to immediately, so we need to add a way to sink to void.

Need to rename it, its not a blackboard. Closest existing thing is uORB but i dont want to copy px4 naming. 

I like how we have publish send data to both ringbuffer and snapshot, since it allows for non blocking logging. But is it really the best way to do it if we have a bunch of data sources each at different speeds? We need to consider how to handle this. 

Honestly I would like to re-write the whole logging system even if we end up at the same point as now. 

I like the idea of doing it more bus-like but i guess the current setup also works but its worh considering. 

---

## GPS debugging

Can we configure the gps to output data faster? Lets look into it and see how much we can configure it from the esp. Right now the data is way behind what we measure from the erpm data.

> yes yes we can, its in progress.

---

## partial dead reckoning (well i acually only really care about speed but it could be useful for torque vectoring)

We have a lot of noise in imu data but we can use erpm data to help with drift. this only works for speed. If gps wont update in time and we cannot get it to update in time this is a good backup to have to have. 

Not sure how it will work when we implement toruqe vectoring and tc but i hope it will still work. 

Can i argue why this is needed when we already get the speed from erpm data? well no but part of me thinks that it can be useful if we want to implement traction control or at least launch control. 

---

## (this is a todo in the todo) go over and see what is debt/dead code and can be removed

Remove the tcp sink and the wifi that comes with it. Cant do this until the logging rewrite above is sorted since its the only sink we have right now. It also sits on 4mb of psram which is absurd for a debug link.

The fs/dashboard stuff should go too. We reserve two littlefs partitions but there is no dashboard component and there never was one, just a commented out line in the root CMakeLists. Thats about 1mb of flash for nothing. Means a reflash.
