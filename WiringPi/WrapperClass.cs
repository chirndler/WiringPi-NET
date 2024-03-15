namespace WiringPi
{
	using System.Runtime.InteropServices;

	/// <summary>
	/// Used to initialise Gordon's library, there's 4 different ways to initialise and we're going to support all 4
	/// </summary>
	public static class Init
	{
		[DllImport("libwiringPi.so", EntryPoint = "wiringPiSetup")]     //This is an example of how to call a method / function in a c library from c#
		public static extern int WiringPiSetup();

		[DllImport("libwiringPi.so", EntryPoint = "wiringPiSetupGpio")]
		public static extern int WiringPiSetupGpio();

		[DllImport("libwiringPi.so", EntryPoint = "wiringPiSetupSys")]
		public static extern int WiringPiSetupSys();

		[DllImport("libwiringPi.so", EntryPoint = "wiringPiSetupPhys")]
		public static extern int WiringPiSetupPhys();
	}

	/// <summary>
	/// Used to configure a GPIO pin's direction and provide read & write functions to a GPIO pin
	/// </summary>
	public static class GPIO
	{
		[DllImport("libwiringPi.so", EntryPoint = "pinMode")]           //Uses Gpio pin numbers
		public static extern void PinMode(int pin, int mode);

		[DllImport("libwiringPi.so", EntryPoint = "digitalWrite")]      //Uses Gpio pin numbers
		public static extern void DigitalWrite(int pin, int value);

		[DllImport("libwiringPi.so", EntryPoint = "digitalWriteByte")]      //Uses Gpio pin numbers
		public static extern void DigitalWriteByte(int value);

		[DllImport("libwiringPi.so", EntryPoint = "digitalRead")]           //Uses Gpio pin numbers
		public static extern int DigitalRead(int pin);

		[DllImport("libwiringPi.so", EntryPoint = "pullUpDnControl")]         //Uses Gpio pin numbers  
		public static extern void PullUpDnControl(int pin, int pud);

		//This pwm mode cannot be used when using GpioSys mode!!
		[DllImport("libwiringPi.so", EntryPoint = "pwmWrite")]              //Uses Gpio pin numbers
		public static extern void PwmWrite(int pin, int value);

		[DllImport("libwiringPi.so", EntryPoint = "pwmSetMode")]             //Uses Gpio pin numbers
		public static extern void PwmSetMode(int mode);

		[DllImport("libwiringPi.so", EntryPoint = "pwmSetRange")]             //Uses Gpio pin numbers
		public static extern void PwmSetRange(uint range);

		[DllImport("libwiringPi.so", EntryPoint = "pwmSetClock")]             //Uses Gpio pin numbers
		public static extern void PwmSetClock(int divisor);

		[DllImport("libwiringPi.so", EntryPoint = "gpioClockSet")]              //Uses Gpio pin numbers
		public static extern void ClockSetGpio(int pin, int freq);

		public enum GPIOpinmode
		{
			Input = 0,
			Output = 1,
			PWMOutput = 2,
			GPIOClock = 3,
			SoftPWMOutput = 4,
			SoftToneOutput = 5,
			PWMToneOutput = 6
		}

		public enum GPIOpinvalue
		{
			High = 1,
			Low = 0
		}

		public enum PullUpDnValue
		{
			Off = 0,
			Down = 1,
			Up = 2
		}
	}

	public static class SoftPwm
	{
		[DllImport("libwiringPi.so", EntryPoint = "softPwmCreate")]
		public static extern int Create(int pin, int initialValue, int pwmRange);

		[DllImport("libwiringPi.so", EntryPoint = "softPwmWrite")]
		public static extern void Write(int pin, int value);

		[DllImport("libwiringPi.so", EntryPoint = "softPwmStop")]
		public static extern void Stop(int pin);
	}

	/// <summary>
	/// Provides use of the Timing functions such as delays
	/// </summary>
	public static class Timing
	{
		[DllImport("libwiringPi.so", EntryPoint = "millis")]
		public static extern uint Millis();

		[DllImport("libwiringPi.so", EntryPoint = "micros")]
		public static extern uint Micros();

		[DllImport("libwiringPi.so", EntryPoint = "delay")]
		public static extern void Delay(uint howLong);

		[DllImport("libwiringPi.so", EntryPoint = "delayMicroseconds")]
		public static extern void DelayMicroseconds(uint howLong);
	}

	/// <summary>
	/// Provides access to the Thread priority and interrupts for IO
	/// </summary>
	public static class PiThreadInterrupts
	{
		[DllImport("libwiringPi.so", EntryPoint = "piHiPri")]
		public static extern int PiHiPri(int priority);

		[DllImport("libwiringPi.so", EntryPoint = "waitForInterrupt")]
		public static extern int WaitForInterrupt(int pin, int timeout);

		//This is the C# equivelant to "void (*function)(void))" required by wiringPi to define a callback method
		public delegate void ISRCallback();

		[DllImport("libwiringPi.so", EntryPoint = "wiringPiISR")]
		public static extern int WiringPiISR(int pin, int mode, ISRCallback method);

		public enum InterruptLevels
		{
			INT_EDGE_SETUP = 0,
			INT_EDGE_FALLING = 1,
			INT_EDGE_RISING = 2,
			INT_EDGE_BOTH = 3
		}
	}

	public static class MiscFunctions
	{
		[DllImport("libwiringPi.so", EntryPoint = "piBoardRev")]
		public static extern int PiBoardRev();

		[DllImport("libwiringPi.so", EntryPoint = "wpiPinToGpio")]
		public static extern int WpiPinToGpio(int wPiPin);

		[DllImport("libwiringPi.so", EntryPoint = "physPinToGpio")]
		public static extern int PhysPinToGpio(int physPin);

		[DllImport("libwiringPi.so", EntryPoint = "setPadDrive")]
		public static extern int SetPadDrive(int group, int value);
	}

	/// <summary>
	/// Provides SPI port functionality
	/// </summary>
	public static class SPI
	{
		/// <summary>
		/// Configures the SPI channel specified on the Raspberry Pi
		/// </summary>
		/// <param name="channel">Selects either Channel 0 or 1 for use</param>
		/// <param name="speed">Selects speed, 500,000 to 32,000,000</param>
		/// <returns>-1 for an error, or the linux file descriptor the channel uses</returns>
		[DllImport("libwiringPi.so", EntryPoint = "wiringPiSPISetup")]
		public static extern int WiringPiSPISetup(int channel, int speed);

		/// <summary>
		/// Read and Write data over the SPI bus, don't forget to configure it first
		/// </summary>
		/// <param name="channel">Selects Channel 0 or Channel 1 for this operation</param>
		/// <param name="data">signed byte array pointer which holds the data to send and will then hold the received data</param>
		/// <param name="len">How many bytes to write and read</param>
		/// <returns>-1 for an error, or the linux file descriptor the channel uses</returns>
		[DllImport("libwiringPi.so", EntryPoint = "wiringPiSPIDataRW")]
		public static unsafe extern int WiringPiSPIDataRW(int channel, byte* data, int len);  //char is a signed byte
	}

	/// <summary>
	/// Provides access to the I2C port
	/// </summary>
	public static class I2C
	{
		[DllImport("libwiringPi.so", EntryPoint = "wiringPiI2CSetup")]
		public static extern int WiringPiI2CSetup(int devId);

		[DllImport("libwiringPi.so", EntryPoint = "wiringPiI2CRead")]
		public static extern int WiringPiI2CRead(int fd);

		[DllImport("libwiringPi.so", EntryPoint = "wiringPiI2CWrite")]
		public static extern int WiringPiI2CWrite(int fd, int data);

		[DllImport("libwiringPi.so", EntryPoint = "wiringPiI2CWriteReg8")]
		public static extern int WiringPiI2CWriteReg8(int fd, int reg, int data);

		[DllImport("libwiringPi.so", EntryPoint = "wiringPiI2CWriteReg16")]
		public static extern int WiringPiI2CWriteReg16(int fd, int reg, int data);

		[DllImport("libwiringPi.so", EntryPoint = "wiringPiI2CReadReg8")]
		public static extern int WiringPiI2CReadReg8(int fd, int reg);

		[DllImport("libwiringPi.so", EntryPoint = "wiringPiI2CReadReg16")]
		public static extern int WiringPiI2CReadReg16(int fd, int reg);
	}

	/// <summary>
	///  Provides the ability to use the Software Tone functions in WiringPi
	/// </summary>
	public static class Tone
	{
		[DllImport("libwiringPi.so", EntryPoint = "softToneCreate")]
		public static extern int SoftToneCreate(int pin);

		[DllImport("libwiringPi.so", EntryPoint = "softToneWrite")]
		public static extern void SoftToneWrite(int pin, int freq);
	}
}