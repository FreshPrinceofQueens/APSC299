// Comms.h - Teams 04, 05 and 06 communications library (Arduino side).

#include <stdint.h>
#include <string.h>
#include <WiFiNINA.h>

class CommsLink final {
public:
    enum State {
        IDLE, // Not driving, waiting for command.
        SEARCHING, // Driving along search path, looking for platform.
        FOUND, // This robot found platform and others are waiting.
        DRIVING, // Driving to the final target.
    };

private:
    const char *wifi_ssid;
    const char *wifi_pass;
    IPAddress server_ip;
    WiFiClient client;
    float target_x, target_y;
    State state = IDLE;
    uint8_t buffer[40];
    int len = 0;

public:
    CommsLink(const char *wifi_ssid, const char *wifi_pass, IPAddress server_ip)
        : wifi_ssid(wifi_ssid), wifi_pass(wifi_pass), server_ip(server_ip) {}

    bool init() {
        int status = WiFi.begin(wifi_ssid, wifi_pass);
        if (status != (WL_CONNECTED)) {
            // Failed to connect to network.
            return false;
        }
        if (!client.connect(server_ip, 22222)) {
            // Failed to connect to server.
            return false;
        }

        return true;
    }

    State update() {
        while (len < sizeof(buffer) && client.available()) {
            uint8_t c = client.read();
            if (c == '\n') {
                // Message received; parse it.
                switch (buffer[0]) {
                    case 'I': // Command from server to idle.
                        state = IDLE;
                        break;
                    case 'S': // Command from server to begin search.
                        state = SEARCHING;
                        break;
                    case 'D': { // Message from other robot, drive to point.
                        // Example command: D12.7,8.5

                        // Locate comma.
                        int i;
                        for (i = 1; i < len && buffer[i] != ','; ++i);
                        if (i >= len) {
                            // Not enough data.
                            break;
                        }

                        // Replace comma with null-terminator to cut strings.
                        buffer[i] = '\0';

                        // Parse out floats.
                        target_x = atof(buffer + 1);
                        target_y = atof(buffer + i + 1);

                        state = DRIVING;
                        break;
                    }
                }
                len = 0;
            } else {
                buffer[len++] = c;
            }
        }
        if (len >= sizeof(buffer)) {
            // Buffer overrun; drop data.
            len = 0;
        }
        return state;
    }

    float get_target_x() const {
        return target_x;
    }

    float get_target_y() const {
        return target_y;
    }

    void force_idle() {
        state = IDLE;
    }

    void notify_found() {
        client.print('F');
        client.print('\n');
        state = FOUND;
    }

    void force_driving(float x, float y) {
        target_x = x;
        target_y = y;
        state = DRIVING;
    }

    void notify_target(int robot, float x, float y) {
        // Example command: D5 12.7,8.5
        client.print('D');
        client.print(robot);
        client.print(' ');
        client.print(x);
        client.print(',');
        client.print(y);
        client.print('\n');
    }
};
