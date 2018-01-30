# Hexapod Simulator
A simulator for Hexapods forked from billhsu github (https://github.com/billhsu/hexapod-sim)
[![Demo video](http://img.youtube.com/vi/59QpA3tUnTU/0.jpg)](http://www.youtube.com/watch?v=59QpA3tUnTU)

## How to build

```
./build.sh
```

## How to run
```
cd Hexapod
./Hexapod               Regular mode, accepts command from terminal and require "\r \n" on the end of every command
./Hexapod -s            Simple mode, accepts command from terminal and DO NOT require "\r \n" on the end of every command
./Hexapod /dev/ttyUSB0  Regular mode, but accepts commands from provided Serial device
```

Hint:
You can emulate Serial device with commands:
```
socat -d -d pty,raw,echo=0 pty,raw,echo=0
```
You will get something like this:
```
2018/01/30 13:54:17 socat[179615] N PTY is /dev/pts/5          <- read from
2018/01/30 13:54:17 socat[179615] N PTY is /dev/pts/6          <- write to
```

Now you can write to virtual Serial device:
```
echo "#2P2000\r\n" > /dev/pts/6
```

And read from it:
```
cat < /dev/pts/5
```
But it is better to connect Hexapod simulator to this virtual device by:
```
./Hexapod /dev/pts/5
```

Now you can write commands to it:
```
echo "#2P2000\r\n" > /dev/pts/6
```
And see the results on the screen

```
# Accepted commands (standard for SSC32 and Veyron 24 Servo Controller)
```
#{servoId}P{PWM}
```
{servoId} is from 1 to 9 and from 32 to 24, {PWM} is from 500 to 2500.

Sample
```
#1P1500#2P3000\r\n
```

You can config the servo mapping in Hexapod/config.txt.

# Screenshot
![Screenshot](screenshots/screenshot.png)


