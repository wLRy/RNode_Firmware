#ifndef LORA_TO_GPIO_H
#define LORA_TO_GPIO_H

const char * GPIO_CMD_PREFIX = "gpiocmd";
const int GPIO_CMD_PREFIX_LEN = 7;
const uint8_t CMD_WRITE = 1;
const uint8_t CMD_READ = 2;
const uint8_t CMD_DISPLAY_INTENSITY = 3;
const uint8_t CMD_READ_BATTERY = 4;
const uint8_t CMD_RESTART = 5;
const uint8_t CMD_ANALOG_READ = 6;
const uint8_t CMD_TIME_TO_OFF = 7;
const char * GPIO_RESP_PREFIX = "gpiorsp";
const int GPIO_RESP_PREFIX_LEN = 7;
const int MY_LORA_TO_GPIO_ID_SIZE = 4;

void serial_callback(uint8_t sbyte);
void di_conf_save(uint8_t dint);

void parseLoRaPacketAndExecGpioCommand(const uint8_t* buf, uint16_t len) {
    int i = 0;
    for (; i < GPIO_CMD_PREFIX_LEN; i++) {
        if (buf[i] != GPIO_CMD_PREFIX[i] || i >= len) {
            return;
        }
    }
    for (; i < GPIO_CMD_PREFIX_LEN + MY_LORA_TO_GPIO_ID_SIZE; i++) {
        if (buf[i] != MY_LORA_TO_GPIO_ID[i - GPIO_CMD_PREFIX_LEN] || i >= len) {
            return;
        }
    }
    if (i+4 > len) {
        last_gpio_command = 255;
        return;
    }

    last_gpio_nonce = buf[i++];
    last_gpio_nonce |= (buf[i++] << 8);
    last_gpio_nonce |= (buf[i++] << 16);
    last_gpio_nonce |= (buf[i++] << 24);

    if (i >= len) {
        last_gpio_command = 255;
        return;
    }
    last_gpio_command = buf[i++];
    if (last_gpio_command == CMD_RESTART) {
        reset_millis = millis() + 3000;
        return;
    }
    if (last_gpio_command == CMD_DISPLAY_INTENSITY) {
        display_intensity = buf[i++];
        if (i < len) {
            bool saveConf = buf[i++];
            if (saveConf) {
                di_conf_save(display_intensity);
            }
        }
        return;
    }
    if (i >= len) {
        return;
    }
    last_gpio = buf[i++];
    if (last_gpio_command == CMD_WRITE) {
        last_gpio_value = buf[i++];
        pinMode(last_gpio, OUTPUT);
        if (last_gpio_value) {
            digitalWrite(last_gpio, HIGH);
            // schedule an off-event if a little-endian duration is present
            if (i + 4 <= len && gpioOffEventCount < MAX_GPIO_EVENTS) {
                uint32_t durationMillis = buf[i++];
                durationMillis |= (buf[i++] << 8);
                durationMillis |= (buf[i++] << 16);
                durationMillis |= (buf[i++] << 24);
                gpioOffEvents[gpioOffEventCount++] = { last_gpio, millis() + durationMillis };
            }
        } else {
            // explicit low: cancel any pending off‐event for this same pin
            for (int j = 0; j < gpioOffEventCount; ++j) {
                if (gpioOffEvents[j].pin == last_gpio) {
                    // remove event at j
                    for (int k = j; k + 1 < gpioOffEventCount; ++k) {
                        gpioOffEvents[k] = gpioOffEvents[k+1];
                    }
                    --gpioOffEventCount;
                    break;
                }
            }
            digitalWrite(last_gpio, LOW);
        }
    }
}

void updateLoraToGpio() {
    uint32_t currentTime = millis();
    // process our off-event queue
    for (int idx = 0; idx < gpioOffEventCount; ) {
        if (currentTime > gpioOffEvents[idx].offMillis) {
            pinMode(gpioOffEvents[idx].pin, OUTPUT);
            digitalWrite(gpioOffEvents[idx].pin, LOW);
            // remove this event
            for (int j = idx; j + 1 < gpioOffEventCount; ++j) {
                gpioOffEvents[j] = gpioOffEvents[j+1];
            }
            --gpioOffEventCount;
            // do not advance idx, since entries shifted
        } else {
            ++idx;
        }
    }
    if (currentTime > reset_millis) {
        reset_millis = 0xFFFFFFFF;
        hard_reset();
    }

    if (last_gpio_nonce != 0) {
        //simulate serial input
        serial_callback(FEND);
        serial_callback(CMD_DATA);
        for (int i = 0; i < GPIO_RESP_PREFIX_LEN; i++) {
            serial_callback(GPIO_RESP_PREFIX[i]);
        }
        for (int i = 0; i < MY_LORA_TO_GPIO_ID_SIZE; i++) {
            serial_callback(MY_LORA_TO_GPIO_ID[i]);
        }
        serial_callback(last_gpio_nonce & 0xff);
        serial_callback((last_gpio_nonce >> 8) & 0xff);
        serial_callback((last_gpio_nonce >> 16) & 0xff);
        serial_callback((last_gpio_nonce >> 24) & 0xff);
        if (last_gpio_command == CMD_READ) {
            pinMode(last_gpio, INPUT);
            last_gpio_value = digitalRead(last_gpio) == HIGH;
            serial_callback(last_gpio_value);
        } else if (last_gpio_command == CMD_READ_BATTERY) {
            last_gpio_value = (uint8_t)int(battery_percent);
            serial_callback(last_gpio_value);
            uint32_t voltage = uint32_t(battery_voltage * 1000);
            serial_callback(voltage & 0xff);
            serial_callback((voltage >> 8) & 0xff);
        } else if (last_gpio_command == CMD_ANALOG_READ) {
            uint32_t result = analogRead(last_gpio);
            serial_callback(result & 0xff);
            serial_callback((result >> 8) & 0xff);
        } else if (last_gpio_command == CMD_TIME_TO_OFF) {
            // find any pending off‐event for last_gpio
            uint32_t remaining = 0;
            for (int j = 0; j < gpioOffEventCount; ++j) {
                if (gpioOffEvents[j].pin == last_gpio) {
                    uint32_t offTime = gpioOffEvents[j].offMillis;
                    if (offTime > currentTime) {
                        remaining = offTime - currentTime;
                    }
                    break;
                }
            }
            // return remaining ms as little‐endian 32‐bit
            serial_callback(remaining & 0xff);
            serial_callback((remaining >> 8) & 0xff);
            serial_callback((remaining >> 16) & 0xff);
            serial_callback((remaining >> 24) & 0xff);
        }
        serial_callback(currentTime & 0xff);
        serial_callback((currentTime >> 8) & 0xff);
        serial_callback((currentTime >> 16) & 0xff);
        serial_callback((currentTime >> 24) & 0xff);

        serial_callback(FEND);
        last_gpio_nonce = 0;
    }
}

#endif