#include "WProgram.h"

#include <map>
#include <string>

#include <SPI.h>
#include <Ethernet2.h>
#include <util.h>
#include <utility/w5500.h>
#include <Bounce2.h>
#include <FastLED.h>
#include <Encoder.h>

#include <json11.hpp>

#define CPU_RESTART_ADDR (uint32_t *)0xE000ED0C
#define CPU_RESTART_VAL 0x5FA0004
#define CPU_RESTART (*CPU_RESTART_ADDR = CPU_RESTART_VAL);

byte mac[] = {
  0x00, 0xAA, 0xBB, 0xCC, 0xDE, 0x02
};

// Initialize the Ethernet client library
// with the IP address and port of the server
// that you want to connect to (port 80 is default for HTTP):
//EthernetClient client;

void log(const char *msg)
{
	char logline[1024];
	unsigned long t = millis();
	unsigned long millis = t % 1000;
	t /= 1000;
	unsigned long seconds = t % 60;
	t /= 60;
	unsigned long minutes = t % 60;
	t /= 60;
	unsigned long hours = t;
	snprintf(logline, 1024, "[%03lu:%02lu:%02lu.%03lu] %s", hours, minutes, seconds, millis, msg);
	Serial.println(logline);
}

namespace shoe {

class PushButton
{
public:
	PushButton(int pin)
		: pin_(pin)
	{
		bounce_.attach(pin, INPUT_PULLUP);
		bounce_.interval(5);
	}

	int pin()
	{
		return pin_;
	}

	void update()
	{
		bounce_.update();
	}

	bool up()
	{
		return bounce_.fell();
	}

	bool down()
	{
		return bounce_.rose();
	}

private:
	Bounce bounce_;
	int pin_;
};
std::vector<PushButton> g_push_buttons;

class EncoderWithPins {
public:
	EncoderWithPins(uint8_t pin1, uint8_t pin2)
	 : encoder_(pin1, pin2)
	 , pin1_(pin1)
	 , pin2_(pin2)
	{
	}
	int32_t readAndReset() {
		int32_t diff = encoder_.readAndReset();
		unsigned long now = millis();
		// filter out any flip-flops within small time ranges
		if (diff != 0 && last_read_time_ != 0 && last_read_ != 0 && (now - last_read_time_) < 500) {
			bool last_read_pos = last_read_ > 0;
			bool diff_pos = diff > 0;
			if (last_read_pos != diff_pos) {
				diff = 0;
			}
		}
		last_read_ = diff;
		last_read_time_ = now;
		return diff;
	}
	uint8_t pin1() { return pin1_; }
	uint8_t pin2() { return pin2_; }
private:
	Encoder encoder_;
	uint8_t pin1_, pin2_;
	int32_t last_read_ = 0;
	unsigned long last_read_time_ = 0;
};
std::vector<EncoderWithPins> g_rotary_encoders;

class InputPin
{
public:
	InputPin(int pin, bool analog)
	 : pin_(pin)
	 , analog_(analog)
	{
		pinMode(pin, INPUT);
	}
	int read()
	{
		int val = value_;
		if (analog_) {
			val = analogRead(pin_);
			int diff = value_ - val;
			changed_ = diff > 10 || diff < -10;
		}
		else {
			val = digitalRead(pin_);
			changed_ = val != value_;
		}

		if (changed_) {
			value_ = val;
		}
		return val;
	}
	bool changed() { return changed_; }
	int value() { return value_; }
	int pin() { return pin_; }
	bool analog() { return analog_; }
private:
	int pin_ = 0;
	int value_ = LOW;
	bool changed_ = false;
	bool analog_ = false;
};
std::vector<InputPin> g_input_pins;

static constexpr int NUM_LEDS = 60;
class NeopixelBase
{
public:
	virtual void init() = 0;
	virtual int pin() = 0;
	virtual void set_colors(CRGB colors[NUM_LEDS]) = 0;
};

template <uint8_t DATA_PIN>
class Neopixel : public NeopixelBase
{
public:
	Neopixel()
	{
		memset(leds_, 0, sizeof(leds_));
	}

	void init()
	{
		FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds_, NUM_LEDS);
		redraw();
	}

	int pin() override
	{
		return DATA_PIN;
	}

	void set_colors(CRGB colors[NUM_LEDS])
	{
		log("set_colors");
		memcpy(leds_, colors, sizeof(leds_));
		redraw();
	}

private:
	void redraw()
	{
		log("redraw");
		num_++;
		//FastLED.clear();
		/*
		for (int i = 0; i < NUM_LEDS; i++) {
			leds_[i] = CRGB::Black;
		}
		for (int i = 0; i < num_; i++) {
			leds_[i] = CRGB::Green;
		}
		*/
		/*
		for (int i = 0; i < NUM_LEDS; i++) {
			leds_[i] = CRGB(0, 0, i*4);
		}
		*/
		/*
		leds_[0] = CRGB::Red;
		leds_[1] = CRGB::Green;
		leds_[2] = CRGB::Blue;
		leds_[3] = CRGB(255, 0, 0);
		*/
		FastLED.show();
	}
	CRGB leds_[NUM_LEDS];
	int num_{0};
};
Neopixel<4> _np4;
std::array<NeopixelBase*, 5> g_neopixels = {
	nullptr,
	nullptr,
	nullptr,
	nullptr,
	&_np4,
};

class HubClient
{
public:
	HubClient()
	{
		// 192.168.138.216
		client_.connect(IPAddress(192, 168, 138, 216), 5350);
		json11::Json::object info{{"mac_address", "e6:43:d8:e9:d4:04"}};
		send_msg("hail", info);
	}

	json11::Json read_msg()
	{
		if (!client_.available()) {
			unsigned long t2 = millis();
			if (last_recv_millis_ > 0 && (t2 - last_recv_millis_) > 3000) {
				Serial.println(last_recv_millis_);
				Serial.println(t2);
				log("restarting");
				delay(100);
				CPU_RESTART
		//last_recv_millis_ = millis();
			}
			return json11::Json();
		}

		char recvd = 0;
		while ((recvd = client_.read()) == 255);
		char type = recvd;
		if (type != 0) {
			log("unknown type");
			return json11::Json();
		}
		int size = 0;
		for (int i=0; i < 4; i++) {
			while ((recvd = client_.read()) == 255);
			size = size << 8;
			size = size + recvd;
		}
		long buf_size = size;
		if (buf_size > (4 * 1024)) {
			log("buf_size is too big");
			return json11::Json();
		}
		char buffer[buf_size + 1];
		int offset = 0;
		while (offset < buf_size) {
			while ((recvd = client_.read()) == 255);
			buffer[offset] = recvd;
			offset++;
		}
		buffer[buf_size] = '\0';

	//	log(buffer);
	//	char size[128];
	//	snprintf(size, 128, "%d", offset);
	//	log(offset);
		last_recv_millis_ = millis();
		std::string err;
		return json11::Json::parse(buffer, err);
	}

	void send_msg(const std::string& type, const json11::Json& info)
	{
		json11::Json msg{{{"type", type}, {"info", info}}};
		std::string msg_str;
		msg.dump(msg_str);
		char msg_type = 0; // JSON
		int msg_size = htonl(msg_str.size());
		client_.write(&msg_type, 1);
		client_.write((char*)&msg_size, 4);
		client_.write(msg_str.c_str(), msg_str.size());
		client_.flush();
		last_recv_millis_ = millis();
		/*
		Serial.println("flushed");
		Serial.println(msg_str.c_str());
		*/
	}

	void send_event(const std::string& type, const json11::Json::object args)
	{
		json11::Json::object info{{"type", type}, {"args", args}};
		send_msg("event", info);
	}

private:
	EthernetClient client_;
	unsigned long last_recv_millis_ = 0;
};

void handle_command(const json11::Json &type, const json11::Json &args) {
	if (!type.is_string()) {
		log("command is not string");
		return;
	}
	const auto& command = type.string_value();

	if (command == "pin_mode") {
		log("pin_mode");
		if (!args["pin"].is_number() || !args["mode"].is_string()) {
			log("pin_mode: bad args");
			return;
		}
		auto val = args["mode"].string_value() == "output" ? OUTPUT : INPUT;
		pinMode(args["pin"].int_value(), val);
	}
	else if (command == "analog_write") {
		log("analog_write");
		if (!args["pin"].is_number() || !args["value"].is_number()) {
			log("analog_write: bad args");
			return;
		}
		analogWrite(args["pin"].int_value(), args["value"].int_value());
	}
	else if (command == "analog_write_frequency") {
		log("analog_write_frequency");
		if (!args["pin"].is_number() || !args["value"].is_number()) {
			log("analog_write_frequency: bad args");
			return;
		}
		analogWriteFrequency(args["pin"].int_value(), args["value"].int_value());
	}
	else if (command == "digital_write") {
		log("digital_write");
		if (!args["pin"].is_number() || !args["value"].is_number()) {
			log("digital_write: bad args");
			return;
		}
		digitalWrite(args["pin"].int_value(), args["value"].int_value());
	}
	else if (command == "push_button") {
		log("push_button");
		if (!args["pin"].is_number()) {
			log("push_button: bad args");
			return;
		}
		g_push_buttons.emplace_back(args["pin"].int_value());
	}
	else if (command == "rotary_encoder") {
		log("rotary_encoder");
		if (!args["pin1"].is_number() || !args["pin2"].is_number()) {
			log("rotary_encoder: bad args");
			return;
		}
		g_rotary_encoders.emplace_back(args["pin1"].int_value(), args["pin2"].int_value());
	}
	else if (command == "input_pin") {
		log("input_pin");
		if (!args["pin"].is_number()) {
			log("digital_input_pin: bad args");
			return;
		}
		bool analog = false;
		if (args["analog"].is_bool()) {
			analog = args["analog"].bool_value();
		}
		g_input_pins.emplace_back(args["pin"].int_value(), analog);
	}
	else if (command == "neopixel") {
		log("neopixel");
		if (!args["pin"].is_number()) {
			log("neopixel: bad args");
			return;
		}
		int pin = args["pin"].int_value();
		if (g_neopixels[pin]) {
			g_neopixels[pin]->init();
		}
	}
	else if (command == "neopixel_colors") {
		log("command neopixel_colors");
		if (!args["pin"].is_number() || !args["colors"].is_array()) {
			log("neopixel_colors: bad args");
			return;
		}
		int pin = args["pin"].int_value();
		auto colors = args["colors"].array_items();
		char logmsg[256];
		snprintf(logmsg, 256, "colors.size: %d", colors.size());
		log(logmsg);
		CRGB leds[NUM_LEDS];
		memset(leds, 0, sizeof(leds));
		int curr = 0;
		for (auto const& color : colors) {
			if (color.is_number()) {
				leds[curr] = static_cast<const uint32_t>(color.int_value());
			}
			else {
				log("color is not a number");
			}
			curr++;
		}
		if (g_neopixels[pin]) {
			g_neopixels[pin]->set_colors(leds);
		}
	}
	else {
		log("unknown command");
		log(command.c_str());
	}
}

void process_messages(HubClient& client)
{
	json11::Json msg = client.read_msg();
	if (msg.is_object())
	{
		if (msg["type"] == "config") {
			if (msg["info"].is_object() && msg["info"]["commands"].is_array()) {
				for (auto const& command : msg["info"]["commands"].array_items()) {
					handle_command(command["command"], command["args"]);
				}
			}
			client.send_msg("configured", json11::Json::object{});
		}
		else if (msg["type"] == "heartbeat") {
			log("heartbeat");
			client.send_msg("heartbeat", json11::Json::object{});
			/*
			const char *msg = "{\"type\":\"heartbeat\",\"info\":{}}";
			client.write(msg, strlen(msg));
			client.flush();
			*/
		}
		else if (msg["type"] == "rehail") {
			json11::Json::object info{{"mac_address", "e6:43:d8:e9:d4:04"}};
			client.send_msg("hail", info);
		}
		else if (msg["type"] == "commands") {
			// commands message format
			/*
			{
				type: "commands",
				info: {
					commands: [
						{command: "analog_write", args: {pin: 10, value: 200}}
					]
				}
			}
			*/
			if (msg["info"].is_object() && msg["info"]["commands"].is_array()) {
				for (auto const& command : msg["info"]["commands"].array_items()) {
					handle_command(command["command"], command["args"]);
				}
			}
			client.send_msg("commands", json11::Json::object{{"result", "ok"}});
		}
	}
}

void process_pushbuttons(HubClient& client)
{
	for (auto& button : g_push_buttons) {
		button.update();

		if (button.down()) {
			log("button down");
			client.send_event("down", {{"pin", button.pin()}});
		}
		if (button.up()) {
			log("button up");
			client.send_event("up", {{"pin", button.pin()}});
		}
	}
}

void process_rotary_encoders(HubClient& client)
{
	for (auto& encoder : g_rotary_encoders) {
		int diff = encoder.readAndReset();

		if (diff != 0) {
			log("rotary encoder changed");
			char diffstr[128];
			snprintf(diffstr, 128, "%d", diff);
			log(diffstr);
			client.send_event("rotary_encoder_changed",
				{{"pin1", encoder.pin1()}, {"pin2", encoder.pin2()}, {"value", diff}});
		}
	}
}

void process_input_pins(HubClient& client)
{
	for (auto& pin : g_input_pins) {
		int val = pin.read();
		if (pin.changed())
		{
			log("pin changed");
			client.send_event("pin_change", {{"pin", pin.pin()}, {"value", val}});
		}
	}
}

extern "C" int main(void)
{
//#ifdef USING_MAKEFILE

	// To use Teensy 3.0 without Arduino, simply put your code here.
	// For example:

	delay(5000);
	Serial.begin(9600);
//	pinMode(13, OUTPUT);
	// this check is only needed on the Leonardo:
	// wait for serial port to connect. Needed for Leonardo only
//	while (!Serial);

//	digitalWriteFast(13, LOW);
	Serial.println("about to begin");
	auto ethernet_begin = Ethernet.begin(mac);
	Serial.println("done with begin");
	if (ethernet_begin == 0) {
		Serial.println("Ethernet.begin failed");
	}
	else {
		Serial.println("Ethernet.begin succeeded");
	}
	IPAddress ip = Ethernet.localIP();
	char ipstr[128];
	snprintf(ipstr, 128, "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
	Serial.println(ipstr);
	/*
	std::map<int, int> foobar = {{10, 10}, {20, 30}};
	foobar[30] = 50;

	std::string oof = "foobar";
	std::map<int, std::string> foobaz;
	foobaz[30] = "fifty";

	json11::Json j{{{"foobar", 10}, {"foobaz", "twenty"}}};
	*/

	/*
	constexpr int NUM_LEDS = 50;
	constexpr int DATA_PIN = 4;
	CRGB leds[NUM_LEDS];
	FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
	for (int i = 0; i < NUM_LEDS; i++) {
		leds[i] = CRGB::Black;
	}
	leds[0] = CRGB::Green;
	leds[1] = CRGB::Red;
	FastLED.show();
	*/
	//g_neopixels.emplace_back(4);
//	EthernetClient client;
//	const char *msg = "{\"type\":\"hail\",\"info\":{\"mac_address\":\"e6:43:d8:e9:d4:04\"}}";
//	client.connect(IPAddress(10, 0, 1, 13), 5350);
//	client.write(msg, strlen(msg));
//	client.flush();
	HubClient client;
//	json11::Json::object info{{"mac_address", "e6:43:d8:e9:d4:04"}};
//	send_msg(client, "hail", info);

	while (1) {
		process_messages(client);
		process_pushbuttons(client);
		process_rotary_encoders(client);
		process_input_pins(client);
		/*
		digitalWriteFast(13, HIGH);
		delay(500);
		digitalWriteFast(13, LOW);
		delay(500);
		Serial.println("Hello Max");
		Serial.println(foobar[20]);
		Serial.println(foobaz[30].c_str());
		Serial.println(oof.c_str());
		Serial.println(j["foobaz"].string_value().c_str());
		*/
	}


/*
#else
	// Arduino's main() function just calls setup() and loop()....
//	setup();
	while (1) {
//		loop();
//		yield();
	}
#endif
*/
}
} // namespace shoe

