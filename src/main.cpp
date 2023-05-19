#include <Arduino.h>
#include <WiFi.h>

#define ENCA 32
#define ENCB 33

#define IN1 23
#define IN2 19

#define PWM 25

#define PWM1_Ch 0
#define PWM1_Res 8
#define PWM1_Freq 1000

#define BTN1_IN 26
#define BTN1_OUT 27

#define BTN2_IN 14
#define BTN2_OUT 12

#define POW_IN 22
#define POW_OUT 21

#define MAX_POS 21200
#define MIN_POS 0

#define LED_BUILTIN 5

long pos = 0;
long previousTime = 0;
float errorPrevious = 0;
float integral = 0;

bool isOpen = false;
bool isRunning = false;

bool openTable = false;
bool closeTable = false;

/// Cont wifi
const char *ssid = "Redmi";
const char *password = "luigi123";

#pragma region Motor

#define FORWARD -1
#define BACKWARD 1
#define STOP 0

void setMotor(int dir, int pwmVal, int pwm = PWM, int in1 = IN1, int in2 = IN2)
{
	// analogWrite(pwm, pwmVal);
	ledcWrite(PWM1_Ch, pwmVal);

	if (dir == FORWARD)
	{
		digitalWrite(in1, HIGH);
		digitalWrite(in2, LOW);
	}
	else if (dir == BACKWARD)
	{
		digitalWrite(in1, LOW);
		digitalWrite(in2, HIGH);
	}
	else if (dir == STOP)
	{
		digitalWrite(in1, LOW);
		digitalWrite(in2, LOW);
	}
}

void readEncoder()
{
	int b = digitalRead(ENCB);

	if (b > 0)
		pos++;
	else
		pos--;
}

#pragma endregion

void PID()
{
	int target = 10000;

	// PID constants
	float kp = 0.5;
	float ki = 0;
	float kd = 0.08;

	// time difference
	long currentTime = micros();
	float deltaT = ((float)(currentTime - previousTime)) / 1.0e6;
	previousTime = currentTime;

	// error
	float error = -target + pos;

	// integral
	integral += error * deltaT;

	// derivative
	float derivative = (error - errorPrevious) / deltaT;

	// output
	float output = kp * error + ki * integral + kd * derivative;

	// motor power
	float pwr = fabs(output);

	if (pwr > 255)
		pwr = 255;

	// motor direction
	int dir = output >= 0 ? 1 : -1;

	if (abs(target - pos) < (0.01 * target))
	{
		pwr = 0;
		dir = 0;
	}

	// signal the motor
	setMotor(dir, pwr);

	// store prev error
	errorPrevious = error;

	Serial.print(target);
	Serial.print(" ");
	Serial.print(pos);
	Serial.println();
}

void oneButton()
{
	if (digitalRead(BTN1_IN) == LOW)
	{
		isRunning = !isRunning;
	}

	if (isRunning)
	{
		if (!isOpen)
		{
			while (pos < MAX_POS)
			{
				setMotor(BACKWARD, 255);

				Serial.print("backward ");
				Serial.println(pos);
			}
			isOpen = true;
		}
		else
		{
			while (pos > MIN_POS)
			{
				setMotor(FORWARD, 255);

				Serial.print("forward ");
				Serial.println(pos);
			}
			isOpen = false;
		}

		isRunning = false;
	}
	else
	{
		setMotor(STOP, 0);
	}
}

void twoButton()
{
	if (digitalRead(BTN1_IN) == LOW)
	{
		openTable = true;
		closeTable = false;

		Serial.println("OPEN");

		isRunning = true;
	}
	else if (digitalRead(BTN2_IN) == LOW)
	{
		closeTable = true;
		openTable = false;

		Serial.println("CLOSE");

		isRunning = true;
	}

	if (isRunning)
	{
		Serial.println("RUNNING");

		if (openTable)
		{
			// while (pos < pos + MAX_POS)
			{
				setMotor(BACKWARD, 255);
				Serial.println("MOTOR BACKWARD");
			}
		}
		else if (closeTable)
		{
			// while (pos > pos - MAX_POS)
			{
				setMotor(FORWARD, 255);
				Serial.println("MOTOR FORWARD");
			}
		}

		openTable = false;
		closeTable = false;
		isRunning = false;

		Serial.println("STOPPED");
	}
	else
	{
		setMotor(STOP, 0);
	}
}

void openCloseButtons()
{
	if (digitalRead(BTN1_IN) == LOW)
	{
		setMotor(BACKWARD, 255);
		Serial.print("BACKWARD ");
		Serial.println(pos);
	}
	else if (digitalRead(BTN2_IN) == LOW)
	{
		setMotor(FORWARD, 255);
		Serial.print("FORWARD ");
		Serial.println(pos);
	}
	else
	{
		setMotor(STOP, 0);
	}
}

void setup()
{

	pinMode(LED_BUILTIN, OUTPUT);
	/// Conexiune Wi-Fi
	Serial.begin(115200);

	WiFi.mode(WIFI_STA);
	WiFi.begin(ssid, password);
	Serial.println("\nConnecting to WiFi Network ..");

	while (WiFi.status() != WL_CONNECTED)
	{
		digitalWrite(LED_BUILTIN, HIGH);
		Serial.print(".");
		delay(100);
		digitalWrite(LED_BUILTIN, LOW);
		delay(100);
	}

	digitalWrite(LED_BUILTIN, LOW);

	Serial.println("\nConnected to the WiFi network");
	Serial.print("Local ESP32 IP: ");
	Serial.println(WiFi.localIP());
	/// Configurare pini

	pinMode(IN1, OUTPUT);
	pinMode(IN2, OUTPUT);

	// pinMode(PWM, OUTPUT);
	ledcAttachPin(PWM, PWM1_Ch);
	ledcSetup(PWM1_Ch, PWM1_Freq, PWM1_Res);

	pinMode(ENCA, INPUT);
	pinMode(ENCB, INPUT);

	pinMode(BTN1_IN, INPUT_PULLUP);
	pinMode(BTN1_OUT, OUTPUT);

	pinMode(BTN2_IN, INPUT_PULLUP);
	pinMode(BTN2_OUT, OUTPUT);

	pinMode(POW_IN, INPUT_PULLUP);
	pinMode(POW_OUT, OUTPUT);

	attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
}

void loop()
{
	// PID();

	// if the power button is turned off the motor will not be available
	if (digitalRead(POW_IN) == HIGH)
	{
		setMotor(STOP, 0);

		return;
	}

	// oneButton();
	twoButton();
	// openCloseButtons();
}
