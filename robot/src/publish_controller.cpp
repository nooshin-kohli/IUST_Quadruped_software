#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <linux/joystick.h>
#include <lcm/lcm-cpp.hpp>
#include "../../lcm-types/cpp/gamepad_lcmt.hpp"

class GamepadPublisher {
public:
    GamepadPublisher() {
        lcm = new lcm::LCM("udpm://239.255.76.67:7667?ttl=1");
        if (!lcm->good()) {
            std::cerr << "LCM initialization failed" << std::endl;
            exit(1);
        }
    }

    ~GamepadPublisher() {
        delete lcm; // Ensure proper memory cleanup
    }

    void publishEvent(const gamepad_lcmt& msg) {
        lcm->publish("interface", &msg);
    }

private:
    lcm::LCM* lcm;
};

int main() {
    // Open the joystick device
    const char* devicePath = "/dev/input/js0";
    int fd = open(devicePath, O_RDONLY);

    if (fd < 0) {
        std::cerr << "Error opening device: " << devicePath << std::endl;
        return 1;
    }

    GamepadPublisher publisher;
    gamepad_lcmt msg;

    // Structure to hold joystick event data
    struct js_event_local {
        uint32_t time;     // Event timestamp in milliseconds
        int16_t value;     // Value of the axis/button
        uint8_t type;      // Event type (button or axis)
        uint8_t number;    // Axis/button number
    };

    js_event_local e;

    // Read joystick events
    while (read(fd, &e, sizeof(e)) > 0) {
        // Update the `msg` object based on the event type and number
        if (e.type == JS_EVENT_BUTTON) {
            switch (e.number) {
                case 0: msg.a = e.value; break;
                case 1: msg.b = e.value; break;
                case 2: msg.x = e.value; break;
                case 3: msg.y = e.value; break;
                case 4: msg.leftBumper = e.value; break;
                case 5: msg.rightBumper = e.value; break;
                case 6: msg.back = e.value; break;
                case 7: msg.start = e.value; break;
                case 8: msg.leftStickButton = e.value; break;
                case 9: msg.rightStickButton = e.value; break;
            }
        } else if (e.type == JS_EVENT_AXIS) {
            if (e.number == 0) msg.leftStickAnalog[0] = e.value / 32767.0; // Normalize to [-1, 1]
            if (e.number == 1) msg.leftStickAnalog[1] = e.value / 32767.0; // Normalize to [-1, 1]
            if (e.number == 3) msg.rightStickAnalog[0] = e.value / 32767.0; // Normalize to [-1, 1]
            if (e.number == 4) msg.rightStickAnalog[1] = e.value / 32767.0; // Normalize to [-1, 1]
            if (e.number == 2) msg.leftTriggerAnalog = (e.value + 32767.0) / 65534.0; // Normalize to [0, 1]
            if (e.number == 5) msg.rightTriggerAnalog = (e.value + 32767.0) / 65534.0; // Normalize to [0, 1]
        }

        // Publish the updated message
        printf("Publishing gamepad event\n");
        publisher.publishEvent(msg);
    }

    // Close the file descriptor
    close(fd);
    return 0;
}
