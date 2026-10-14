#include "pin_test.h"

#include <driver/gpio.h>
#include <esp_log.h>
#include <esp_rom_sys.h>
#include <soc/soc_caps.h>

#include "esp_private/esp_gpio_reserve.h"

static const char *TAG = "pin_test";

namespace pin_test {
namespace {

constexpr uint32_t PULL_SETTLE_US = 2000;
constexpr uint32_t DRIVE_SETTLE_US = 200;

enum class Verdict { Ok, OutputDead, HeldHigh, HeldLow, Inconclusive };

struct Result {
  int up;
  int down;
  int drive_hi;
  int drive_lo;
  bool drive_tested;
};

bool exists(int gpio) { return SOC_GPIO_VALID_GPIO_MASK & (1ULL << gpio); }

const char *skip_reason(int gpio) {
  if (esp_gpio_is_reserved(1ULL << gpio)) {
    return "claimed by flash or psram";
  }
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
  if (gpio == 19 || gpio == 20) {
    return "usb console";
  }
#endif
  return nullptr;
}

const char *board_note(int gpio) {
  switch (gpio) {
  case 0:
    return "strap, boot button on devkits";
  case 3:
    return "strap, jtag select";
  case 43:
  case 44:
    return "uart0, wired to the usb bridge on some devkits";
  case 45:
    return "strap, vdd_spi";
  case 46:
    return "strap, boot mode";
  case 48:
    return "rgb led on devkits";
  default:
    return "";
  }
}

Result probe(gpio_num_t gpio) {
  Result r{};

  gpio_config_t cfg = {
      .pin_bit_mask = 1ULL << gpio,
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_ENABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  gpio_config(&cfg);
  esp_rom_delay_us(PULL_SETTLE_US);
  r.up = gpio_get_level(gpio);

  cfg.pull_up_en = GPIO_PULLUP_DISABLE;
  cfg.pull_down_en = GPIO_PULLDOWN_ENABLE;
  gpio_config(&cfg);
  esp_rom_delay_us(PULL_SETTLE_US);
  r.down = gpio_get_level(gpio);

  // A pad that ignored both pulls is either dead or held by something outside
  // the chip. Driving it would put our output stage against that something, so
  // stop here rather than risk the contention this test exists to detect.
  r.drive_tested = (r.up == 1 && r.down == 0);
  if (!r.drive_tested) {
    gpio_reset_pin(gpio);
    return r;
  }

  // Drive against the opposite internal pull. A driver that has gone weak
  // loses that fight, where a plain readback would still report its own level.
  gpio_set_level(gpio, 1);
  cfg.mode = GPIO_MODE_INPUT_OUTPUT;
  gpio_config(&cfg);
  esp_rom_delay_us(DRIVE_SETTLE_US);
  r.drive_hi = gpio_get_level(gpio);

  gpio_set_level(gpio, 0);
  cfg.pull_up_en = GPIO_PULLUP_ENABLE;
  cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
  gpio_config(&cfg);
  esp_rom_delay_us(DRIVE_SETTLE_US);
  r.drive_lo = gpio_get_level(gpio);

  gpio_reset_pin(gpio);
  return r;
}

Verdict judge(const Result &r) {
  if (!r.drive_tested) {
    if (r.up == r.down) {
      return r.up ? Verdict::HeldHigh : Verdict::HeldLow;
    }
    return Verdict::Inconclusive;
  }
  if (r.drive_hi == 1 && r.drive_lo == 0) {
    return Verdict::Ok;
  }
  return Verdict::OutputDead;
}

const char *verdict_text(Verdict v) {
  switch (v) {
  case Verdict::Ok:
    return "OK";
  case Verdict::OutputDead:
    return "OUTPUT DEAD";
  case Verdict::HeldHigh:
    return "HELD HIGH";
  case Verdict::HeldLow:
    return "HELD LOW";
  default:
    return "INCONCLUSIVE";
  }
}

} // namespace

void run_all() {
  ESP_LOGI(TAG, "================================================");
  ESP_LOGI(TAG, "GPIO pad test, expects nothing wired to the board");
  ESP_LOGI(TAG, "  pin  pull u/d  drive h/l  verdict       note");

  int tested = 0;
  int ok = 0;

  for (int gpio = 0; gpio <= SOC_GPIO_IN_RANGE_MAX; gpio++) {
    if (!exists(gpio)) {
      continue;
    }
    if (const char *skip = skip_reason(gpio)) {
      ESP_LOGI(TAG, "  %-4d   -/-       -/-        SKIPPED       %s", gpio, skip);
      continue;
    }

    const Result r = probe(static_cast<gpio_num_t>(gpio));
    const Verdict v = judge(r);
    tested++;
    if (v == Verdict::Ok) {
      ok++;
    }

    if (r.drive_tested) {
      ESP_LOGI(TAG, "  %-4d   %d/%d       %d/%d        %-13s %s", gpio, r.up,
               r.down, r.drive_hi, r.drive_lo, verdict_text(v),
               board_note(gpio));
    } else {
      ESP_LOGI(TAG, "  %-4d   %d/%d       skipped    %-13s %s", gpio, r.up,
               r.down, verdict_text(v), board_note(gpio));
    }
  }

  ESP_LOGI(TAG, "%d pins tested, %d fully good, %d flagged", tested, ok,
           tested - ok);
  ESP_LOGI(TAG, "HELD on a bare board means the input buffer is not responding");
  ESP_LOGI(TAG, "================================================");
}

} // namespace pin_test
