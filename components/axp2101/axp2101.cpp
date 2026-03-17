#include "axp2101.h"
#include "esp_sleep.h"
#include "esphome/core/log.h"
// #include <Esp.h>

#ifndef CONFIG_PMU_IRQ
#define CONFIG_PMU_IRQ 35
#endif

static bool pmu_flag = false;
static const uint8_t pmu_irq_pin = CONFIG_PMU_IRQ;

void IRAM_ATTR setFlag(void)
{
    pmu_flag = true;
}

namespace esphome {
namespace axp2101 {

static const char *TAG = "axp2101.sensor";

// ── Register helper implementations ─────────────────────────────────────────

void AXP2101Component::setRegisterBit(uint8_t reg, uint8_t bit) {
    uint8_t val = Read8bit(reg);
    Write1Byte(reg, val | (1u << bit));
}

void AXP2101Component::clrRegisterBit(uint8_t reg, uint8_t bit) {
    uint8_t val = Read8bit(reg);
    Write1Byte(reg, val & ~(1u << bit));
}

bool AXP2101Component::getRegisterBit(uint8_t reg, uint8_t bit) {
    return (Read8bit(reg) >> bit) & 0x01;
}

// Set LDO voltage: reg = voltage register (0x92-0x9A), 500-3500mV, 100mV steps
// Writes to bits 4:0, preserves bits 7:5
void AXP2101Component::setLdoVoltage(uint8_t reg, uint16_t millivolt) {
    if (millivolt < 500) millivolt = 500;
    if (millivolt > 3500) millivolt = 3500;
    uint8_t val = (Read8bit(reg) & 0xE0) | (((millivolt - 500) / 100) & 0x1F);
    Write1Byte(reg, val);
}

void AXP2101Component::setLdoEnabled(uint8_t reg, uint8_t bit, bool on) {
    if (on) {
        setRegisterBit(reg, bit);
    } else {
        clrRegisterBit(reg, bit);
    }
}

// ── Battery status ──────────────────────────────────────────────────────────

bool AXP2101Component::isBatteryConnect() {
    return getRegisterBit(AXP2101_STATUS1, 3);
}

int AXP2101Component::getBatteryPercent() {
    if (!isBatteryConnect()) return -1;
    return Read8bit(AXP2101_BAT_PERCENT);
}

bool AXP2101Component::isCharging() {
    uint8_t val = Read8bit(AXP2101_STATUS2);
    return ((val >> 5) & 0x07) == 0x01;
}

float AXP2101Component::GetBatVoltage() {
    if (!isBatteryConnect()) return 0;
    uint8_t h = Read8bit(AXP2101_BAT_VOLTAGE_H);
    uint8_t l = Read8bit(AXP2101_BAT_VOLTAGE_L);
    return ((h & 0x1F) << 8) | l;  // millivolts
}

// ── Setup ───────────────────────────────────────────────────────────────────

void AXP2101Component::setup()
{
    // Verify chip ID
    uint8_t chip_id = Read8bit(AXP2101_IC_TYPE);
    ESP_LOGCONFIG(TAG, "getID:0x%x", chip_id);
    if (chip_id != 0x4B && chip_id != 0x4A) {
        ESP_LOGE(TAG, "AXP2101 not found at address 0x%02X (got chip ID 0x%02X)", AXP2101_ADDR, chip_id);
        this->mark_failed();
        return;
    }
    ESP_LOGCONFIG(TAG, "AXP2101 initialized successfully");

    // Set VBUS input voltage limit to 4.36V (register 0x15, bits 3:0)
    uint8_t val = (Read8bit(AXP2101_INPUT_VOL_LIMIT) & 0xF0) | (VBUS_VOL_LIM_4V36 & 0x0F);
    Write1Byte(AXP2101_INPUT_VOL_LIMIT, val);

    // Set VBUS input current limit to 1500mA (register 0x16, bits 2:0)
    val = (Read8bit(AXP2101_INPUT_CUR_LIMIT) & 0xF8) | (VBUS_CUR_LIM_1500MA & 0x07);
    Write1Byte(AXP2101_INPUT_CUR_LIMIT, val);

    // Set system power-down voltage to 2600mV (register 0x24, bits 2:0)
    // Formula: reg_val = (mV - 2600) / 100 = 0
    uint16_t sys_off_mv = 2600;
    val = (Read8bit(AXP2101_VOFF_SET) & 0xF8) | (((sys_off_mv - 2600) / 100) & 0x07);
    Write1Byte(AXP2101_VOFF_SET, val);

    uint8_t voff_reg = Read8bit(AXP2101_VOFF_SET) & 0x07;
    ESP_LOGCONFIG(TAG, "->  getSysPowerDownVoltage:%u", (unsigned)(voff_reg * 100 + 2600));

    // ── Set DC voltages ─────────────────────────────────────────────────

    // DC1: 3300mV (register 0x82, bits 4:0, (mV-1500)/100)
    Write1Byte(AXP2101_DC1_VOLTAGE, (3300 - 1500) / 100);
    ESP_LOGCONFIG(TAG, "DC1  : %s   Voltage:%u mV",
        getRegisterBit(AXP2101_DC_ONOFF_CTRL, 0) ? "+" : "-",
        (unsigned)((Read8bit(AXP2101_DC1_VOLTAGE) & 0x1F) * 100 + 1500));

    // DC2: 1000mV (register 0x83, bits 6:0, range1: (mV-500)/10)
    val = (Read8bit(AXP2101_DC2_VOLTAGE) & 0x80) | (((1000 - 500) / 10) & 0x7F);
    Write1Byte(AXP2101_DC2_VOLTAGE, val);

    // DC3: 3300mV (register 0x84, bits 6:0, range3: (mV-1600)/100 + 88)
    val = (Read8bit(AXP2101_DC3_VOLTAGE) & 0x80) | ((((3300 - 1600) / 100) + 88) & 0x7F);
    Write1Byte(AXP2101_DC3_VOLTAGE, val);

    // DC4: 1000mV (register 0x85, bits 6:0, range1: (mV-500)/10)
    val = (Read8bit(AXP2101_DC4_VOLTAGE) & 0x80) | (((1000 - 500) / 10) & 0x7F);
    Write1Byte(AXP2101_DC4_VOLTAGE, val);

    // DC5: 3300mV (register 0x86, bits 4:0, (mV-1400)/100)
    val = (Read8bit(AXP2101_DC5_VOLTAGE) & 0xE0) | (((3300 - 1400) / 100) & 0x1F);
    Write1Byte(AXP2101_DC5_VOLTAGE, val);

    // ── Set LDO voltages ────────────────────────────────────────────────
    setLdoVoltage(AXP2101_ALDO1_VOLTAGE, 3300);
    setLdoVoltage(AXP2101_ALDO2_VOLTAGE, 3300);
    // ALDO3 (speaker) - leave as is
    setLdoVoltage(AXP2101_ALDO4_VOLTAGE, 3300);  // Display power
    setLdoVoltage(AXP2101_BLDO1_VOLTAGE, 3300);  // LCD backlight
    setLdoVoltage(AXP2101_BLDO2_VOLTAGE, 3300);

    // CPUSLDO: 1000mV, 50mV steps: (1000-500)/50 = 10
    val = (Read8bit(AXP2101_CPUSLDO_VOLTAGE) & 0xE0) | (((1000 - 500) / 50) & 0x1F);
    Write1Byte(AXP2101_CPUSLDO_VOLTAGE, val);

    // ── Enable/disable power rails ──────────────────────────────────────

    // DC rails (register 0x80)
    // DC1: leave as-is (don't enable)
    setRegisterBit(AXP2101_DC_ONOFF_CTRL, 1);  // DC2
    setRegisterBit(AXP2101_DC_ONOFF_CTRL, 2);  // DC3
    setRegisterBit(AXP2101_DC_ONOFF_CTRL, 3);  // DC4
    setRegisterBit(AXP2101_DC_ONOFF_CTRL, 4);  // DC5

    // LDO rails (register 0x90)
    setLdoEnabled(AXP2101_LDO_ONOFF_CTRL0, AXP2101_LDO_ALDO1_BIT, true);
    setLdoEnabled(AXP2101_LDO_ONOFF_CTRL0, AXP2101_LDO_ALDO2_BIT, true);
    // ALDO3 (speaker) - leave disabled
    setLdoEnabled(AXP2101_LDO_ONOFF_CTRL0, AXP2101_LDO_ALDO4_BIT, true);   // Display power
    // BLDO1 (backlight) - controlled by UpdateBrightness()
    setLdoEnabled(AXP2101_LDO_ONOFF_CTRL0, AXP2101_LDO_BLDO2_BIT, true);
    setLdoEnabled(AXP2101_LDO_ONOFF_CTRL0, AXP2101_LDO_CPUSLDO_BIT, true);
    // DLDO1 (vibration) - leave disabled
    // DLDO2 - leave disabled

    // ── Log all rail states ─────────────────────────────────────────────
    ESP_LOGCONFIG(TAG, "DC1  : %s   Voltage:%u mV",
        getRegisterBit(AXP2101_DC_ONOFF_CTRL, 0) ? "+" : "-",
        (unsigned)((Read8bit(AXP2101_DC1_VOLTAGE) & 0x1F) * 100 + 1500));
    ESP_LOGCONFIG(TAG, "DC2  : %s",  getRegisterBit(AXP2101_DC_ONOFF_CTRL, 1) ? "+" : "-");
    ESP_LOGCONFIG(TAG, "DC3  : %s",  getRegisterBit(AXP2101_DC_ONOFF_CTRL, 2) ? "+" : "-");
    ESP_LOGCONFIG(TAG, "DC4  : %s",  getRegisterBit(AXP2101_DC_ONOFF_CTRL, 3) ? "+" : "-");
    ESP_LOGCONFIG(TAG, "DC5  : %s",  getRegisterBit(AXP2101_DC_ONOFF_CTRL, 4) ? "+" : "-");

    uint8_t ldo_ctrl = Read8bit(AXP2101_LDO_ONOFF_CTRL0);
    ESP_LOGCONFIG(TAG, "ALDO1: %s   Voltage:%u mV", (ldo_ctrl & (1<<0)) ? "+" : "-",
        (unsigned)((Read8bit(AXP2101_ALDO1_VOLTAGE) & 0x1F) * 100 + 500));
    ESP_LOGCONFIG(TAG, "ALDO2: %s   Voltage:%u mV", (ldo_ctrl & (1<<1)) ? "+" : "-",
        (unsigned)((Read8bit(AXP2101_ALDO2_VOLTAGE) & 0x1F) * 100 + 500));
    ESP_LOGCONFIG(TAG, "ALDO3: %s   Voltage:%u mV", (ldo_ctrl & (1<<2)) ? "+" : "-",
        (unsigned)((Read8bit(AXP2101_ALDO3_VOLTAGE) & 0x1F) * 100 + 500));
    ESP_LOGCONFIG(TAG, "ALDO4: %s   Voltage:%u mV", (ldo_ctrl & (1<<3)) ? "+" : "-",
        (unsigned)((Read8bit(AXP2101_ALDO4_VOLTAGE) & 0x1F) * 100 + 500));
    ESP_LOGCONFIG(TAG, "BLDO1: %s   Voltage:%u mV", (ldo_ctrl & (1<<4)) ? "+" : "-",
        (unsigned)((Read8bit(AXP2101_BLDO1_VOLTAGE) & 0x1F) * 100 + 500));
    ESP_LOGCONFIG(TAG, "BLDO2: %s   Voltage:%u mV", (ldo_ctrl & (1<<5)) ? "+" : "-",
        (unsigned)((Read8bit(AXP2101_BLDO2_VOLTAGE) & 0x1F) * 100 + 500));
    ESP_LOGCONFIG(TAG, "CPUSLDO: %s Voltage:%u mV", (ldo_ctrl & (1<<6)) ? "+" : "-",
        (unsigned)((Read8bit(AXP2101_CPUSLDO_VOLTAGE) & 0x1F) * 50 + 500));
    ESP_LOGCONFIG(TAG, "DLDO1: %s   Voltage:%u mV", (ldo_ctrl & (1<<7)) ? "+" : "-",
        (unsigned)((Read8bit(AXP2101_DLDO1_VOLTAGE) & 0x1F) * 100 + 500));
    ESP_LOGCONFIG(TAG, "DLDO2: %s   Voltage:%u mV",
        getRegisterBit(AXP2101_LDO_ONOFF_CTRL1, 0) ? "+" : "-",
        (unsigned)((Read8bit(AXP2101_DLDO2_VOLTAGE) & 0x1F) * 100 + 500));

    // ── Power key timing ────────────────────────────────────────────────
    // Press-off time: 4s (register 0x27, bits 3:2, value 0)
    val = (Read8bit(AXP2101_IRQ_OFF_ON_LEVEL) & 0xF3) | ((POWEROFF_4S & 0x03) << 2);
    Write1Byte(AXP2101_IRQ_OFF_ON_LEVEL, val);

    // Press-on time: 128ms (register 0x27, bits 1:0, value 0)
    val = (Read8bit(AXP2101_IRQ_OFF_ON_LEVEL) & 0xFC) | (POWERON_128MS & 0x03);
    Write1Byte(AXP2101_IRQ_OFF_ON_LEVEL, val);

    uint8_t pkey = Read8bit(AXP2101_IRQ_OFF_ON_LEVEL);
    uint8_t off_time = (pkey >> 2) & 0x03;
    uint8_t on_time = pkey & 0x03;
    const char *off_str[] = {"4 Second", "6 Second", "8 Second", "10 Second"};
    const char *on_str[] = {"128 Ms", "512 Ms", "1 Second", "2 Second"};
    ESP_LOGCONFIG(TAG, "PowerKeyPressOffTime: %s", off_str[off_time]);
    ESP_LOGCONFIG(TAG, "PowerKeyPressOnTime: %s", on_str[on_time]);

    // ── DC over/under-voltage protection status ─────────────────────────
    ESP_LOGCONFIG(TAG, "getDCHighVoltagePowerDownEn: %s",
        getRegisterBit(AXP2101_DC_OVP_UVP_CTRL, 5) ? "ENABLE" : "DISABLE");
    for (int i = 0; i < 5; i++) {
        ESP_LOGCONFIG(TAG, "getDC%dLowVoltagePowerDownEn: %s", i + 1,
            getRegisterBit(AXP2101_DC_OVP_UVP_CTRL, i) ? "ENABLE" : "DISABLE");
    }

    // ── TS pin: disable temperature sensing (no battery thermistor) ─────
    val = (Read8bit(AXP2101_TS_PIN_CTRL) & 0xF0) | 0x10;
    Write1Byte(AXP2101_TS_PIN_CTRL, val);
    clrRegisterBit(AXP2101_ADC_CHANNEL_CTRL, 1);  // Disable TS ADC

    // Enable temperature measurement (internal AXP2101 temp)
    setRegisterBit(AXP2101_ADC_CHANNEL_CTRL, 4);

    // Enable ADC channels
    setRegisterBit(AXP2101_BAT_DET_CTRL, 0);       // Battery detection
    setRegisterBit(AXP2101_ADC_CHANNEL_CTRL, 2);    // VBUS voltage
    setRegisterBit(AXP2101_ADC_CHANNEL_CTRL, 0);    // Battery voltage
    setRegisterBit(AXP2101_ADC_CHANNEL_CTRL, 3);    // System voltage

    // ── IRQ pin setup ───────────────────────────────────────────────────
    pinMode(pmu_irq_pin, INPUT_PULLUP);
    attachInterrupt(pmu_irq_pin, setFlag, FALLING);

    // Disable all interrupts
    Write1Byte(AXP2101_INTEN1, 0x00);
    Write1Byte(AXP2101_INTEN2, 0x00);
    Write1Byte(AXP2101_INTEN3, 0x00);

    // Clear all interrupt flags
    Write1Byte(AXP2101_INTSTS1, 0xFF);
    Write1Byte(AXP2101_INTSTS2, 0xFF);
    Write1Byte(AXP2101_INTSTS3, 0xFF);

    // Enable desired interrupts:
    // INTEN1 (0x40): BAT_INSERT(bit 0), BAT_REMOVE(bit 1), VBUS_INSERT(bit 2), VBUS_REMOVE(bit 3)
    //                PKEY_SHORT(bit 4), PKEY_LONG(bit 5)
    // INTEN2 (0x41): BAT_CHG_START(bit 0), BAT_CHG_DONE(bit 1)
    Write1Byte(AXP2101_INTEN1, 0x3F);
    Write1Byte(AXP2101_INTEN2, 0x03);

    // ── Charger configuration ───────────────────────────────────────────
    // Precharge: 50mA (register 0x61, bits 1:0)
    val = (Read8bit(AXP2101_IPRECHG_SET) & 0xFC) | (PRECHARGE_50MA & 0x03);
    Write1Byte(AXP2101_IPRECHG_SET, val);

    // Constant current: 200mA (register 0x62, bits 4:0)
    val = (Read8bit(AXP2101_ICC_CHG_SET) & 0xE0) | (CHG_CUR_200MA & 0x1F);
    Write1Byte(AXP2101_ICC_CHG_SET, val);

    // Termination current: 25mA (register 0x63, bits 3:0)
    val = (Read8bit(AXP2101_ITERM_CHG_SET) & 0xF0) | (CHG_ITERM_25MA & 0x0F);
    Write1Byte(AXP2101_ITERM_CHG_SET, val);

    // Charge target voltage: 4.1V (register 0x64, bits 2:0)
    val = (Read8bit(AXP2101_CV_CHG_VOL_SET) & 0xF8) | (CHG_VOL_4V1 & 0x07);
    Write1Byte(AXP2101_CV_CHG_VOL_SET, val);

    // ── Watchdog ────────────────────────────────────────────────────────
    // Disable PMU hardware watchdog — ESPHome has its own software WDT.
    // The 4s PMU watchdog was causing reboot loops during setup.
    clrRegisterBit(AXP2101_CHARGE_GAUGE_WDT, 0);

    // ── Button battery ──────────────────────────────────────────────────
    // Enable button battery charge (register 0x18, bit 2)
    setRegisterBit(AXP2101_CHARGE_GAUGE_WDT, 2);

    // Set button battery charge voltage: 3300mV (register 0x6A, bits 2:0)
    // Formula: (3300-2600)/100 = 7
    val = (Read8bit(AXP2101_BTN_BAT_CHG_VOL) & 0xF8) | (7 & 0x07);
    Write1Byte(AXP2101_BTN_BAT_CHG_VOL, val);
}

// ── Screen backlight enable/disable ─────────────────────────────────────────

void AXP2101Component::set_lcd_enabled(bool on) {
    setLdoEnabled(AXP2101_LDO_ONOFF_CTRL0, AXP2101_LDO_BLDO1_BIT, on);
}

// ── Charge LED control ──────────────────────────────────────────────────────

void AXP2101Component::set_blue_led(bool on) {
    uint8_t val = Read8bit(AXP2101_CHGLED_SET_CTRL);
    if (on) {
        // Manual control, LED ON: mask 0xC8, set manual enable (0x05), mode ON (3 << 4)
        val = (val & 0xC8) | 0x05 | (CHG_LED_ON << 4);
    } else {
        // Manual control, LED OFF: mask 0xC8, set manual enable (0x05), mode OFF (0 << 4)
        val = (val & 0xC8) | 0x05 | (CHG_LED_OFF << 4);
    }
    Write1Byte(AXP2101_CHGLED_SET_CTRL, val);
}

// ── Speaker power (ALDO3) ───────────────────────────────────────────────────

void AXP2101Component::set_speaker_enabled(bool on) {
    setLdoEnabled(AXP2101_LDO_ONOFF_CTRL0, AXP2101_LDO_ALDO3_BIT, on);
}

// ── Config dump ─────────────────────────────────────────────────────────────

void AXP2101Component::dump_config() {
    ESP_LOGCONFIG(TAG, "AXP2101:");
    LOG_I2C_DEVICE(this);
    LOG_SENSOR("  ", "Battery Voltage", this->batteryvoltage_sensor_);
    LOG_SENSOR("  ", "Battery Level", this->batterylevel_sensor_);
    LOG_BINARY_SENSOR("  ", "Battery Charging", this->batterycharging_bsensor_);
}

float AXP2101Component::get_setup_priority() const { return setup_priority::DATA; }

// ── Periodic update ─────────────────────────────────────────────────────────

void AXP2101Component::update() {

    if (this->batterylevel_sensor_ != nullptr) {
        float vbat = GetBatVoltage();
        ESP_LOGD(TAG, "Got Battery Voltage=%f", vbat);
        this->batteryvoltage_sensor_->publish_state(vbat / 1000.0f);

        float batterylevel;
        if (isBatteryConnect()) {
            batterylevel = getBatteryPercent();
        } else {
            batterylevel = 100.0f * ((vbat - 3000.0f) / (4100.0f - 3000.0f));
        }

        ESP_LOGD(TAG, "Got Battery Level=%f", batterylevel);
        if (batterylevel > 100.0f) {
            batterylevel = 100.0f;
        }
        this->batterylevel_sensor_->publish_state(batterylevel);
    }

    if (this->batterycharging_bsensor_ != nullptr) {
        bool vcharging = isCharging();
        ESP_LOGD(TAG, "Got Battery Charging=%s", vcharging ? "true" : "false");
        this->batterycharging_bsensor_->publish_state(vcharging);
    }

    UpdateBrightness();
}

// ── Register I/O ────────────────────────────────────────────────────────────

void AXP2101Component::Write1Byte(uint8_t Addr, uint8_t Data)
{
    this->write_byte(Addr, Data);
}

uint8_t AXP2101Component::Read8bit(uint8_t Addr)
{
    uint8_t data;
    this->read_byte(Addr, &data);
    return data;
}

uint16_t AXP2101Component::Read12Bit(uint8_t Addr)
{
    uint16_t Data = 0;
    uint8_t buf[2];
    ReadBuff(Addr, 2, buf);
    Data = ((buf[0] << 4) + buf[1]);
    return Data;
}

uint16_t AXP2101Component::Read13Bit(uint8_t Addr)
{
    uint16_t Data = 0;
    uint8_t buf[2];
    ReadBuff(Addr, 2, buf);
    Data = ((buf[0] << 5) + buf[1]);
    return Data;
}

uint16_t AXP2101Component::Read16bit(uint8_t Addr)
{
    uint32_t ReData = 0;
    uint8_t Buff[2];
    this->read_bytes(Addr, Buff, sizeof(Buff));
    for (int i = 0; i < (int)sizeof(Buff); i++) {
        ReData <<= 8;
        ReData |= Buff[i];
    }
    return ReData;
}

uint32_t AXP2101Component::Read24bit(uint8_t Addr)
{
    uint32_t ReData = 0;
    uint8_t Buff[3];
    this->read_bytes(Addr, Buff, sizeof(Buff));
    for (int i = 0; i < (int)sizeof(Buff); i++) {
        ReData <<= 8;
        ReData |= Buff[i];
    }
    return ReData;
}

uint32_t AXP2101Component::Read32bit(uint8_t Addr)
{
    uint32_t ReData = 0;
    uint8_t Buff[4];
    this->read_bytes(Addr, Buff, sizeof(Buff));
    for (int i = 0; i < (int)sizeof(Buff); i++) {
        ReData <<= 8;
        ReData |= Buff[i];
    }
    return ReData;
}

void AXP2101Component::ReadBuff(uint8_t Addr, uint8_t Size, uint8_t *Buff)
{
    this->read_bytes(Addr, Buff, Size);
}

// ── Brightness / Backlight ──────────────────────────────────────────────────

void AXP2101Component::UpdateBrightness() {
    if (brightness_ == curr_brightness_) return;
    ESP_LOGD(TAG, "Brightness=%f (Curr: %f)", brightness_, curr_brightness_);
    curr_brightness_ = brightness_;

    if (brightness_ <= 0.0f) {
        ESP_LOGD(TAG, "Brightness zero; disabling BLDO1");
        setLdoEnabled(AXP2101_LDO_ONOFF_CTRL0, AXP2101_LDO_BLDO1_BIT, false);
        return;
    }

    // Use actual visible range [20..30] for BLDO1_CFG steps
    const uint8_t min_vis_step = 20;
    const uint8_t max_step     = 30;
    uint8_t step = static_cast<uint8_t>(brightness_ * (max_step - min_vis_step) + 0.5f) + min_vis_step;
    if (step > max_step) step = max_step;

    uint8_t reg = (Read8bit(AXP2101_BLDO1_VOLTAGE) & 0xE0) | (step & 0x1F);
    ESP_LOGD(TAG, "Setting BLDO1 step %u for brightness=%f", step, brightness_);
    Write1Byte(AXP2101_BLDO1_VOLTAGE, reg);
    setLdoEnabled(AXP2101_LDO_ONOFF_CTRL0, AXP2101_LDO_BLDO1_BIT, true);
}

// ── Coulomb counter ─────────────────────────────────────────────────────────

void AXP2101Component::EnableCoulombcounter(void)  { Write1Byte(0xB8, 0x80); }
void AXP2101Component::DisableCoulombcounter(void) { Write1Byte(0xB8, 0x00); }
void AXP2101Component::StopCoulombcounter(void)    { Write1Byte(0xB8, 0xC0); }
void AXP2101Component::ClearCoulombcounter(void)   { Write1Byte(0xB8, 0xA0); }
uint32_t AXP2101Component::GetCoulombchargeData(void)    { return Read32bit(0xB0); }
uint32_t AXP2101Component::GetCoulombdischargeData(void) { return Read32bit(0xB4); }

float AXP2101Component::GetCoulombData(void)
{
    uint32_t coin = GetCoulombchargeData();
    uint32_t coout = GetCoulombdischargeData();
    float ccc = 65536 * 0.5 * (coin - coout) / 3600.0 / 25.0;
    return ccc;
}

// ── Measurement helpers ─────────────────────────────────────────────────────

float AXP2101Component::GetBatCurrent()
{
    float ADCLSB = 0.5;
    uint16_t CurrentIn = Read13Bit(0x7A);
    uint16_t CurrentOut = Read13Bit(0x7C);
    return (CurrentIn - CurrentOut) * ADCLSB;
}

float AXP2101Component::GetVinVoltage()
{
    float ADCLSB = 1.7 / 1000.0;
    uint16_t ReData = Read12Bit(0x56);
    return ReData * ADCLSB;
}

float AXP2101Component::GetVinCurrent()
{
    float ADCLSB = 0.625;
    uint16_t ReData = Read12Bit(0x58);
    return ReData * ADCLSB;
}

float AXP2101Component::GetVBusVoltage()
{
    float ADCLSB = 1.7 / 1000.0;
    uint16_t ReData = Read12Bit(0x5A);
    return ReData * ADCLSB;
}

float AXP2101Component::GetVBusCurrent()
{
    float ADCLSB = 0.375;
    uint16_t ReData = Read12Bit(0x5C);
    return ReData * ADCLSB;
}

float AXP2101Component::GetTempInAXP2101()
{
    float ADCLSB = 0.1;
    const float OFFSET_DEG_C = -144.7;
    uint16_t ReData = Read12Bit(0x5E);
    return OFFSET_DEG_C + ReData * ADCLSB;
}

float AXP2101Component::GetBatPower()
{
    float VoltageLSB = 1.1;
    float CurrentLCS = 0.5;
    uint32_t ReData = Read24bit(0x70);
    return VoltageLSB * CurrentLCS * ReData / 1000.0;
}

float AXP2101Component::GetBatChargeCurrent()
{
    float ADCLSB = 0.5;
    uint16_t ReData = Read13Bit(0x7A);
    return ReData * ADCLSB;
}

float AXP2101Component::GetAPSVoltage()
{
    float ADCLSB = 1.4 / 1000.0;
    uint16_t ReData = Read12Bit(0x7E);
    return ReData * ADCLSB;
}

float AXP2101Component::GetBatCoulombInput()
{
    uint32_t ReData = Read32bit(0xB0);
    return ReData * 65536 * 0.5 / 3600 / 25.0;
}

float AXP2101Component::GetBatCoulombOut()
{
    uint32_t ReData = Read32bit(0xB4);
    return ReData * 65536 * 0.5 / 3600 / 25.0;
}

void AXP2101Component::SetCoulombClear() { Write1Byte(0xB8, 0x20); }

uint8_t AXP2101Component::GetWarningLevel(void) { return Read8bit(0x47) & 0x01; }

uint8_t AXP2101Component::GetBtnPress()
{
    uint8_t state = Read8bit(0x46);
    if (state) {
        Write1Byte(0x46, 0x03);
    }
    return state;
}

// ── Sleep ───────────────────────────────────────────────────────────────────

void AXP2101Component::SetSleep(void)
{
    Write1Byte(0x31, Read8bit(0x31) | (1 << 3)); // Power off voltage 3.0v
    Write1Byte(0x90, Read8bit(0x90) | 0x07);     // GPIO1 floating
    Write1Byte(0x82, 0x00);                       // Disable ADCs
    Write1Byte(0x12, Read8bit(0x12) & 0xA1);     // Disable all outputs but DCDC1
}

void AXP2101Component::DeepSleep(uint64_t time_in_us)
{
    SetSleep();
    esp_sleep_enable_ext0_wakeup((gpio_num_t)39, 0);
    if (time_in_us > 0) {
        esp_sleep_enable_timer_wakeup(time_in_us);
    } else {
        esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
    }
    (time_in_us == 0) ? esp_deep_sleep_start() : esp_deep_sleep(time_in_us);
}

void AXP2101Component::LightSleep(uint64_t time_in_us)
{
    if (time_in_us > 0) {
        esp_sleep_enable_timer_wakeup(time_in_us);
    } else {
        esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
    }
    esp_light_sleep_start();
}

// ── Misc ────────────────────────────────────────────────────────────────────

void AXP2101Component::SetChargeCurrent(uint8_t current)
{
    uint8_t buf = Read8bit(0x33);
    buf = (buf & 0xf0) | (current & 0x07);
    Write1Byte(0x33, buf);
}

void AXP2101Component::PowerOff()
{
    Write1Byte(0x32, Read8bit(0x32) | 0x80);
}

void AXP2101Component::SetAdcState(bool state)
{
    Write1Byte(0x82, state ? 0xff : 0x00);
}

std::string AXP2101Component::GetStartupReason() {
    esp_reset_reason_t reset_reason = ::esp_reset_reason();
    if (reset_reason == ESP_RST_DEEPSLEEP) {
        esp_sleep_source_t wake_reason = esp_sleep_get_wakeup_cause();
        if (wake_reason == ESP_SLEEP_WAKEUP_EXT0)
            return "ESP_SLEEP_WAKEUP_EXT0";
        if (wake_reason == ESP_SLEEP_WAKEUP_EXT1)
            return "ESP_SLEEP_WAKEUP_EXT1";
        if (wake_reason == ESP_SLEEP_WAKEUP_TIMER)
            return "ESP_SLEEP_WAKEUP_TIMER";
        if (wake_reason == ESP_SLEEP_WAKEUP_TOUCHPAD)
            return "ESP_SLEEP_WAKEUP_TOUCHPAD";
        if (wake_reason == ESP_SLEEP_WAKEUP_ULP)
            return "ESP_SLEEP_WAKEUP_ULP";
        if (wake_reason == ESP_SLEEP_WAKEUP_GPIO)
            return "ESP_SLEEP_WAKEUP_GPIO";
        if (wake_reason == ESP_SLEEP_WAKEUP_UART)
            return "ESP_SLEEP_WAKEUP_UART";
        return std::string{"WAKEUP_UNKNOWN_REASON"};
    }

    if (reset_reason == ESP_RST_UNKNOWN)  return "ESP_RST_UNKNOWN";
    if (reset_reason == ESP_RST_POWERON)  return "ESP_RST_POWERON";
    if (reset_reason == ESP_RST_SW)       return "ESP_RST_SW";
    if (reset_reason == ESP_RST_PANIC)    return "ESP_RST_PANIC";
    if (reset_reason == ESP_RST_INT_WDT)  return "ESP_RST_INT_WDT";
    if (reset_reason == ESP_RST_TASK_WDT) return "ESP_RST_TASK_WDT";
    if (reset_reason == ESP_RST_WDT)      return "ESP_RST_WDT";
    if (reset_reason == ESP_RST_BROWNOUT) return "ESP_RST_BROWNOUT";
    if (reset_reason == ESP_RST_SDIO)     return "ESP_RST_SDIO";
    return std::string{"RESET_UNKNOWN_REASON"};
}

}
}
