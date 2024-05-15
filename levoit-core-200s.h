#define USE_UART_DEBUGGER

#include "esphome.h"
#include <vector>
#include "esphome/components/uart/uart_debugger.h"
#include "esphome/core/helpers.h"

static const char *TAG = "component.core200s_UART";

class component_core200sUART :
        public PollingComponent,
        public UARTDevice,
        public CustomAPIDevice {

  public:
    TextSensor *textsensor_fanSpeed;
    TextSensor *textsensor_Power;
    TextSensor *textsensor_mcuFW;
    TextSensor *textsensor_SleepMode;
    TextSensor *textsensor_DisplayLit;
    TextSensor *textsensor_DisplayState;
    TextSensor *textsensor_DisplayLocked;
    TextSensor *textsensor_NightLight;
    TextSensor *textsensor_FilterState;

    static component_core200sUART* instance(UARTComponent *parent) {
      static component_core200sUART* INSTANCE = new component_core200sUART(parent);
      return INSTANCE;
    }

    // delay setup() so that core200s board has booted completely before
    // starting to rx/tx on bus
    float get_setup_priority() const override {
      return esphome::setup_priority::LATE;
    }

    void setup() override {
      ESP_LOGD(TAG, "setup()");

      rx_buf.reserve(128);
      tx_buf.reserve(32);

      textsensor_fanSpeed->publish_state("Unknown");
      textsensor_Power->publish_state("Unknown");
      textsensor_SleepMode->publish_state("Unknown");
      textsensor_DisplayLit->publish_state("Unknown");
      textsensor_DisplayLocked->publish_state("Unknown");
      textsensor_mcuFW->publish_state("Unknown");
      textsensor_DisplayState->publish_state("Unknown");
      textsensor_NightLight->publish_state("Unknown");
      textsensor_FilterState->publish_state("Unknown");

      set_update_interval(5);

      send_command(cmd_get_status);
    }

    void update() override {
      while (available()) {
        read_byte(&b);

        // look for start byte
        if ((b == 0xA5)) {
          rx_buf.clear();
          rx_buf.push_back(b);

          // read until checksum
          for (int i = 0; i < 5; i++) {
            if (read_byte(&b)) {
              rx_buf.push_back(b);
            } else {
              break;
            }
          }

          if (rx_buf.size() == 6) {
            // read rest of packet
            for (int i = 0; i < rx_buf[3]; i++) {
              if (read_byte(&b)) {
                rx_buf.push_back(b);
              } else {
                break;
              }
            }

            // process packet if payload was read complete
            if (rx_buf.size() == (rx_buf[3] + 6)) {
              process_packet();
            }
          }
        }
      }
    }

    void power_on(void) {
      ESP_LOGD(TAG, "power_on");
      send_command(cmd_power_on);
    }

    void power_off(void) {
      ESP_LOGD(TAG, "power_off");
      send_command(cmd_power_off);
    }

    void set_fan_manual_high(void) {
      ESP_LOGD(TAG, "set_fan_manual_high");
      send_command(cmd_set_fan_manual_high);
    }

    void set_fan_manual_medium(void) {
      ESP_LOGD(TAG, "set_fan_manual_medium");
      send_command(cmd_set_fan_manual_medium);
    }

    void set_fan_manual_low(void) {
      ESP_LOGD(TAG, "set_fan_manual_low");
      send_command(cmd_set_fan_manual_low);
    }

    void sleep_mode_off(void) {
      ESP_LOGD(TAG, "sleep_mode_off");
      send_command(cmd_sleep_mode_off);
    }

    void sleep_mode_on(void) {
      ESP_LOGD(TAG, "sleep_mode_on");
      send_command(cmd_sleep_mode_on);
    }

    void lock_display(void) {
      ESP_LOGD(TAG, "lock_display");
      send_command(cmd_lock_display);
    }

    void unlock_display(void) {
      ESP_LOGD(TAG, "unlock_display");
      send_command(cmd_unlock_display);
    }

    void display_on(void) {
      ESP_LOGD(TAG, "display_on");
      send_command(cmd_display_on);
    }

    void display_off(void) {
      ESP_LOGD(TAG, "display_off");
      send_command(cmd_display_off);
    }

    void wifi_led_off(void) {
      if (wifi_led == 0) {
        return;
      }

      ESP_LOGD(TAG, "wifi_led_off");
      send_command(cmd_wifi_led_off);
      wifi_led = 0;
    }

    void wifi_led_on(void) {
      if (wifi_led == 1) {
        return;
      }

      ESP_LOGD(TAG, "wifi_led_on");
      send_command(cmd_wifi_led_on);
      wifi_led = 1;
    }

    void filter_led_on(void) {
      ESP_LOGD(TAG, "filter_led_on");
      send_command(cmd_filter_led_on);
    }

    void filter_led_off(void) {
      ESP_LOGD(TAG, "filter_led_off");
      send_command(cmd_filter_led_off);
    }

    void nightlight_high(void) {
      ESP_LOGD(TAG, "nightlight_high");
      send_command(cmd_nightlight_high);
    }

    void nightlight_low(void) {
      ESP_LOGD(TAG, "nightlight_low");
      send_command(cmd_nightlight_low);
    }

    void nightlight_off(void) {
      ESP_LOGD(TAG, "nightlight_off");
      send_command(cmd_nightlight_off);
    }

    void get_status(void) {
      ESP_LOGD(TAG, "get_status");
      send_command(cmd_get_status);
    }

  private:
    std::vector<uint8_t> rx_buf, tx_buf;
    uint8_t b = 0;
    uint8_t tx_seq_num = 0;
    uint8_t wifi_led = 0;

    std::vector<uint8_t> cmd_get_status            = { 0XA5, 0X22, 0XFF, 0X04, 0X00, 0XFF, 0X01, 0X31, 0X40, 0X00 };

    std::vector<uint8_t> cmd_power_on              = { 0XA5, 0X22, 0XFF, 0X05, 0X00, 0XFF, 0X01, 0X00, 0XA0, 0X00, 0X01 };
    std::vector<uint8_t> cmd_power_off             = { 0XA5, 0X22, 0XFF, 0X05, 0X00, 0XFF, 0X01, 0X00, 0xA0, 0X00, 0X00 };

    std::vector<uint8_t> cmd_set_fan_manual_high   = { 0XA5, 0X22, 0XFF, 0X07, 0X00, 0XFF, 0X01, 0X60, 0XA2, 0X00, 0X00, 0X01, 0X03 };
    std::vector<uint8_t> cmd_set_fan_manual_medium = { 0XA5, 0X22, 0XFF, 0X07, 0X00, 0XFF, 0X01, 0X60, 0XA2, 0X00, 0X00, 0X01, 0X02 };
    std::vector<uint8_t> cmd_set_fan_manual_low    = { 0XA5, 0X22, 0XFF, 0X07, 0X00, 0XFF, 0X01, 0X60, 0XA2, 0X00, 0X00, 0X01, 0X01 };

    std::vector<uint8_t> cmd_sleep_mode_off        = { 0XA5, 0X22, 0XFF, 0X05, 0X00, 0XFF, 0X01, 0XE0, 0XA5, 0X00, 0X00 };
    std::vector<uint8_t> cmd_sleep_mode_on         = { 0XA5, 0X22, 0XFF, 0X05, 0X00, 0XFF, 0X01, 0XE0, 0XA5, 0X00, 0X01 };

    std::vector<uint8_t> cmd_lock_display          = { 0XA5, 0X22, 0XFF, 0X05, 0X00, 0XFF, 0X01, 0X00, 0XD1, 0X00, 0X01 };
    std::vector<uint8_t> cmd_unlock_display        = { 0XA5, 0X22, 0XFF, 0X05, 0X00, 0XFF, 0X01, 0X00, 0XD1, 0X00, 0X00 };

    std::vector<uint8_t> cmd_display_on            = { 0XA5, 0X22, 0XFF, 0X05, 0X00, 0XFF, 0X01, 0X05, 0XA1, 0X00, 0X64 };
    std::vector<uint8_t> cmd_display_off           = { 0XA5, 0X22, 0XFF, 0X05, 0X00, 0XFF, 0X01, 0X05, 0XA1, 0X00, 0X00 };

    std::vector<uint8_t> cmd_wifi_led_off          = { 0XA5, 0X22, 0XFF, 0X0A, 0X00, 0XFF, 0X01, 0X29, 0XA1, 0X00, 0X00, 0XF4, 0X01, 0XF4, 0X01, 0X00 };
    std::vector<uint8_t> cmd_wifi_led_on           = { 0XA5, 0X22, 0XFF, 0X0A, 0X00, 0XFF, 0X01, 0X29, 0XA1, 0X00, 0X01, 0X7D, 0X00, 0X7D, 0X00, 0X00 };

    std::vector<uint8_t> cmd_filter_led_on         = { 0XA5, 0X22, 0XFF, 0X05, 0X00, 0XFF, 0X01, 0XE2, 0XA5, 0X00, 0X01 };
    std::vector<uint8_t> cmd_filter_led_off        = { 0XA5, 0X22, 0XFF, 0X05, 0X00, 0XFF, 0X01, 0XE2, 0xA5, 0X00, 0X00 };

    std::vector<uint8_t> cmd_nightlight_high       = { 0XA5, 0X22, 0XFF, 0X05, 0X00, 0XFF, 0X01, 0x03, 0xA0, 0x00, 0x00, 0x64 };
    std::vector<uint8_t> cmd_nightlight_low        = { 0XA5, 0X22, 0XFF, 0X05, 0X00, 0XFF, 0X01, 0x03, 0xA0, 0x00, 0x00, 0x32 };
    std::vector<uint8_t> cmd_nightlight_off        = { 0XA5, 0X22, 0XFF, 0X05, 0X00, 0XFF, 0X01, 0x03, 0xA0, 0x00, 0x00, 0x00 };

    component_core200sUART(UARTComponent *parent) : PollingComponent(200), UARTDevice(parent) {
      this->textsensor_fanSpeed       = new TextSensor();
      this->textsensor_Power          = new TextSensor();
      this->textsensor_mcuFW          = new TextSensor();
      this->textsensor_SleepMode      = new TextSensor();
      this->textsensor_DisplayLit     = new TextSensor();
      this->textsensor_DisplayLocked  = new TextSensor();
      this->textsensor_DisplayState   = new TextSensor();
      this->textsensor_NightLight     = new TextSensor();
      this->textsensor_FilterState    = new TextSensor();
    }

    void send_command(std::vector< uint8_t > &cmd) {
      cmd[2] = tx_seq_num++;
      cmd[5] = 0; // zero checksum byte
      uint8_t cs = 255;

      for (int i = 0; i < cmd.size(); i++) {
        cs -= cmd[i];
      }

      cmd[5] = cs;
      write_array(cmd);
    }

    void acknowledge_packet() {
      tx_buf.clear();
      tx_buf.assign(rx_buf.begin(), rx_buf.begin() + 9);
      tx_buf.push_back(0);
      tx_buf[1] = 0x12;  // ack byte
      tx_buf[3] = 4;     // payload length
      tx_buf[5] = 0;     // checksum byte

      uint8_t cs = 255;

      for (int i = 0; i < tx_buf.size(); i++) {
        cs -= tx_buf[i];
      }

      tx_buf[5] = cs;
      write_array(tx_buf);
    }

    void process_packet() {
      char buf[32];
      unsigned int packet_type=0;
      packet_type = (rx_buf[6] << 16) + (rx_buf[7] << 8) + rx_buf[8];

      std::string formattedValue = format_hex_pretty(packet_type).c_str();
      std::replace(formattedValue.begin(), formattedValue.end(), '.', ':');

      ESP_LOGW(TAG, "process_packet: %s", formattedValue.c_str());

      if (rx_buf[1] == 0x22) {
        acknowledge_packet();
      }

      switch (packet_type) {
        // status packet
        case 0x016040:
          // check length of packet
          if (rx_buf.size() != 22) {
            return;
          }

          // firmware version
          sprintf(buf, "%d.%d.%d", rx_buf[12], rx_buf[11], rx_buf[10]);
          textsensor_mcuFW->publish_state(buf);

          // power
          switch (rx_buf[13]) {
            case 0x00:
              textsensor_Power->publish_state("Off");
              break;

            case 0x01:
              textsensor_Power->publish_state("On");
              break;

            default:
              sprintf(buf, "Error %X", rx_buf[13]);
              textsensor_Power->publish_state(buf);
          }

          // sleep mode
          switch (rx_buf[14]) {
            case 0x00:
              textsensor_SleepMode->publish_state("Off");
              break;

            case 0x01:
              textsensor_SleepMode->publish_state("On");
              break;

            default:
              sprintf(buf, "Error %X", rx_buf[14]);
              textsensor_SleepMode->publish_state(buf);
          }

          // fan speed
          switch (rx_buf[15]) {
            case 0x01:
              textsensor_fanSpeed->publish_state("Low");
              break;

            case 0x02:
              textsensor_fanSpeed->publish_state("Medium");
              break;

            case 0x03:
              textsensor_fanSpeed->publish_state("High");
              break;

            default:
              sprintf(buf, "Error %X", rx_buf[15]);
              textsensor_fanSpeed->publish_state(buf);
          }

          // display brightness
          switch (rx_buf[16]) {
            case 0x00:
              textsensor_DisplayLit->publish_state("Off");
              break;

            case 0x64:
              textsensor_DisplayLit->publish_state("On");
              break;

            default:
              sprintf(buf, "Error %X", rx_buf[16]);
              textsensor_DisplayLit->publish_state(buf);
          }

          // display state
          switch (rx_buf[17]) {
            case 0x00:
              textsensor_DisplayState->publish_state("Off");
              break;

            case 0x01:
              textsensor_DisplayState->publish_state("On");
              break;

            default:
              sprintf(buf, "Error %X", rx_buf[17]);
              textsensor_DisplayState->publish_state(buf);
          }

          // filter LED state
          switch(rx_buf[19]) {
            case 0x00:
              textsensor_FilterState->publish_state("Off");
              break;

            case 0x01:
              textsensor_FilterState->publish_state("On");
              break;

            default:
              sprintf(buf, "Error %X", rx_buf[19]);
              textsensor_FilterState->publish_state(buf);
          }

          // display lock state
          switch (rx_buf[20]) {
            case 0x00:
              textsensor_DisplayLocked->publish_state("Off");
              break;

            case 0x01:
              textsensor_DisplayLocked->publish_state("On");
              break;

            default:
              sprintf(buf, "Error %X", rx_buf[20]);
              textsensor_DisplayLocked->publish_state(buf);
          }

          // nightlight state
          switch (rx_buf[21]) {
            case 0x00:
              textsensor_NightLight->publish_state("Off");
              break;

            case 0x32:
              textsensor_NightLight->publish_state("Low");
              break;

            case 0x64:
              textsensor_NightLight->publish_state("High");
              break;

            default:
              sprintf(buf, "Error %X", rx_buf[21]);
              textsensor_NightLight->publish_state(buf);
          }

          break;
      }
    }
};
