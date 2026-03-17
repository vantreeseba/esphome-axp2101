# ESPHome AXP2101 Component

This custom component implements AXP2101 support for the M5Stack Core2 V1.1, building on top of https://github.com/martydingo/esphome-axp192 and https://github.com/lewisxhe/XPowersLib. The Core2 V1.1 uses an AXP2101 while the older Core2 uses an AXP192.

> [!IMPORTANT]
> The xPowers library has been removed from this component in favor of direct register writes due to conflicts with ESPHome's move to ESP-IDF's native I2C driver

*This component does not offer full functionality yet, it only covers part of the AXP2101 features and is not fully tested.*  

## Installation

Copy the components to a custom_components directory next to your .yaml configuration file, or include directly from this repository.

## Configuration

Sample configurations are found in the `/sample-config` folder.

This component adds a new model configuration to the AXP2101 sensor which determines which registers are needed for each device. The only available model is `model: M5CORE2`.

### Include AXP2101 Component

```yaml
external_components:
  - source: github://vantreeseba/esphome-axp2101
    components: [ axp2101 ]
```

### M5Stack Core2 V1.1

```yaml
sensor:
  - platform: axp2101
    model: M5CORE2
    address: 0x34
    i2c_id: bus_a
    update_interval: 30s
    brightness: 75%
    battery_voltage:
      name: "Battery Voltage"
    battery_level:
      name: "Battery Level"
    battery_charging:
      name: "Battery Charging"
```

The display component required for the M5Stack Core2 V1.1 is as follows:

```yaml
font:
  - file: "gfonts://Roboto"
    id: roboto
    size: 24

display:
  - platform: ili9xxx
    model: M5STACK
    dimensions: 320x240
    cs_pin: GPIO5
    dc_pin: GPIO15
    lambda: |-
      it.print(0, 0, id(roboto), "Hello World");
```
