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

#define MAX_POS -21200
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
const char *ssid = "DIGI-D5bj";
const char *password = "2874qX3n";

WiFiServer server(80);

String header;

bool toggleTableOnline = false;

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

void oneButton()
{
	if (digitalRead(BTN1_IN) == LOW || toggleTableOnline)
	{
		isRunning = !isRunning;
	}

	if (isRunning)
	{
		if (!isOpen)
		{
			while (pos > MAX_POS)
			{
				setMotor(BACKWARD, 255);

				Serial.print("backward ");
				Serial.println(pos);
			}
			isOpen = true;
		}
		else
		{
			while (pos < MIN_POS)
			{
				setMotor(FORWARD, 255);

				Serial.print("forward ");
				Serial.println(pos);
			}
			isOpen = false;
		}

		isRunning = false;
		toggleTableOnline = false;
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
				// Serial.println("MOTOR BACKWARD");
				Serial.println(pos);
			}
		}
		else if (closeTable)
		{
			// while (pos > pos - MAX_POS)
			{
				setMotor(FORWARD, 255);
				// Serial.println("MOTOR FORWARD");
				Serial.println(pos);
			}
		}

		openTable = false;
		closeTable = false;
		isRunning = false;

		// Serial.println("STOPPED");
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

	server.begin();

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
	WiFiClient client = server.available();

	if (client)
	{
		Serial.println("New Client.");
		String currentLine = "";
		while (client.connected())
		{
			if (client.available())
			{
				char c = client.read();
				Serial.write(c);
				header += c;
				if (c == '\n')
				{
					if (currentLine.length() == 0)
					{
						client.println("HTTP/1.1 200 OK");
						client.println("Content-type:text/html");
						client.println("Connection: close");
						client.println();

						if (header.indexOf("GET /toggle-table") >= 0)
						{
							if (digitalRead(POW_IN) != HIGH)
							{
								toggleTableOnline = true;
							}
						}

						client.println("<!DOCTYPE html><html>");
						client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
						client.println("<link rel=\"icon\" href=\"data:,\">");
						client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
						client.println(".button { background-color: #195B6A; border: none; color: white; padding: 16px 40px;");
						client.println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
						client.println(".button2 {background-color: #77878A;}</style></head>");

						client.println("<body><h1>ESP32 Web Server</h1>");
						client.println("<p>Click to open or close the table.</p>");
						client.println("<p><a href=\"/toggle-table\"><button class=\"button\">TOGGLE</button></a></p>");

						client.println("</body></html>");
						client.println();
						break;
					}
					else
					{
						currentLine = "";
					}
				}
				else if (c != '\r')
				{
					currentLine += c;
				}
			}
		}
		client.stop();
		Serial.println("Client disconnected.");
	}

	// if the power button is turned off the motor will not be available
	if (digitalRead(POW_IN) == HIGH)
	{
		setMotor(STOP, 0);

		return;
	}

	oneButton();
	// twoButton();
	// openCloseButtons();
}
