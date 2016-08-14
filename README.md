# quadcopter
This is my Arduino Due based quadcopter. The language is C++ with about 2500 lines of code.

If you want to use code from this repo, you use it at your own risk. I am not responsible for any damage or injury resulting from anybody using this software. Building a quadcopter from scratch can become dangerous. The props turn very fast and sooner or later, you will make a mistake.

Be patient. Be careful. Be a good engineer.

## MIQ: Does it fly?
Yes, it does. Check this out: [YouTube Video] (https://www.youtube.com/watch?v=DGXSOvx3GmY)

## How does it look like?
[Pic0](https://cloud.githubusercontent.com/assets/20499620/17651703/e24e91ae-626c-11e6-9fed-11d581fa1a50.jpg)
[Pic1](https://cloud.githubusercontent.com/assets/20499620/17651707/ec92b8c0-626c-11e6-837b-2d71fa09c990.jpg)
[Pic2](https://cloud.githubusercontent.com/assets/20499620/17651710/f2587f4c-626c-11e6-8e57-1e0f6f156db3.jpg)
![eye_catcher](https://cloud.githubusercontent.com/assets/20499620/17651702/ddfb5e66-626c-11e6-9d9b-9204bb1c9ec1.jpg)

## Software
The quadcopter uses a 1st order complementary filter for sensor fusion with body-to-earth-frame transformations. It measures roll, pitch, yaw, rotating rates, height, climbrate and accelerations. Also, the complementary filter does not use constant filter coefficients, but changes them during flight to trust the accelerometer or the gyroscope more.

The PID controller consists of two stages. The 1st stage takes the actual angles from sensor fusion and the desired angle from the radio controller and determines a desired rotating rate. The second stage takes this desired totating rate and compares it to the acutal rate. The output is send to the motors. This two stage design is a very common.

## Hardware
- **Flightcontroller**: Arduino Due (SainSmart clone) + software presented here
- **Sensor board**: GY-86 10 DOF MPU6050 + HMC5883L + MS5611
- **Radio**: Flysky-i6 (6 channels) with a FS-iA6 receiver
- **Motors**: EMAX Multi Copter MT2213 Brushless Motor
- **Props**: came with the motors (10x4,5 Emax)
- **Frame**: Q450 V3
- **PDB**: integrated in frame
- **ESCs**: HobbyKing 20A BlueSeries Brushless Speed Controller (flashed with SimonK)
- **ESC flash tool**: [Atmel Atmega Socket Firmware Flashing Tool] (http://www.hobbyking.com/hobbyking/store/__27195__Atmel_Atmega_Socket_Firmware_Flashing_Tool.html)
- **Battery**: ZIPPY Compact 4000mAh 3S 25C Lipo Pack
- **Battery charger**: IMax B6AC V2

## Damages during build
- 4 props
- Once I accidentally started the quadcopter with props mounted in full throttle about one meter next to me. Fortunately, nothing happened. It flew in the opposite direction and crashed.
- At an early stage of the software, the quadcopter fell two times from a height of about 5 to 10 meters. I never found out why.
