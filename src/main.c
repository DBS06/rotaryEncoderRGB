#include <inttypes.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#define BTN_NODE DT_ALIAS(rot_btn)
#if !DT_NODE_HAS_STATUS(BTN_NODE, okay)
#error "Unsupported board: sw0 devicetree alias is not defined"
#endif

/**
 * From:
 * https://github.com/sparkfun/Rotary_Encoder_Breakout-Illuminated/blob/main/Firmware/RGB_Rotary_Encoder/RGB_Rotary_Encoder.ino
 *
 * Rotary encoder pin A to digital pin 3*
 * Rotary encoder pin B to analog pin 3
 * Rotary encoder pin C to ground
 *
 * This sketch implements software debounce, but you can further
 * improve performance by placing 0.1uF capacitors between
 * A and ground, and B and ground.
 *
 * If you wish to use the RGB LED and button functions of
 * SparkFun part number COM-10982, use the following connections:
 *
 * Rotary encoder pin 1 (red cathode) to digital pin 10
 * Rotary encoder pin 2 (green cathode) to analog pin 1
 * Rotary encoder pin 3 (button) to digital pin 4
 * Rotary encoder pin 4 (blue cathode) to digital pin 5
 * Rotary encoder pin 5 (common anode) to VCC (3.3V or 5V)
 *
 * Note that because this is a common anode device,
 * the pushbutton requires an external 1K-10K pullDOWN resistor
 * to operate.
 */

static const struct gpio_dt_spec rotBtn = GPIO_DT_SPEC_GET_OR(BTN_NODE, gpios, {0});

static const struct pwm_dt_spec rotLedRed   = PWM_DT_SPEC_GET(DT_ALIAS(rot_led_red));
static const struct pwm_dt_spec rotLedGreen = PWM_DT_SPEC_GET(DT_ALIAS(rot_led_green));
static const struct pwm_dt_spec rotLedBlue  = PWM_DT_SPEC_GET(DT_ALIAS(rot_led_blue));

#define NUM_STEPS 50U
#define SLEEP_MSEC 25U

static struct gpio_callback rotButtonCbData;

void rotBtnPressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
int ledFadeInOut(const struct pwm_dt_spec *led, const uint32_t numSteps, const size_t timeoutMs);

int main(void)
{
    int ret;

    printk("Test Rotary Encoder - Illuminated (RGB)\n");

    if (!pwm_is_ready_dt(&rotLedRed))
    {
        printk("Error: PWM device %s is not ready\n", rotLedRed.dev->name);
        return 0;
    }

    if (!pwm_is_ready_dt(&rotLedGreen))
    {
        printk("Error: PWM device %s is not ready\n", rotLedGreen.dev->name);
        return 0;
    }

    if (!pwm_is_ready_dt(&rotLedBlue))
    {
        printk("Error: PWM device %s is not ready\n", rotLedBlue.dev->name);
        return 0;
    }

    if (!gpio_is_ready_dt(&rotBtn))
    {
        printk("Error: rotBtn device %s is not ready\n", rotBtn.port->name);
        return 0;
    }
    ret = gpio_pin_configure_dt(&rotBtn, GPIO_INPUT);
    if (ret != 0)
    {
        printk("Error %d: failed to configure %s pin %d\n", ret, rotBtn.port->name, rotBtn.pin);
        return 0;
    }

    ret = gpio_pin_interrupt_configure_dt(&rotBtn, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret != 0)
    {
        printk("Error %d: failed to configure interrupt on %s pin %d\n", ret, rotBtn.port->name, rotBtn.pin);
        return 0;
    }

    gpio_init_callback(&rotButtonCbData, rotBtnPressed, BIT(rotBtn.pin));
    gpio_add_callback(rotBtn.port, &rotButtonCbData);
    printk("Set up button at %s pin %d\n", rotBtn.port->name, rotBtn.pin);

    while (1)
    {
        ret = ledFadeInOut(&rotLedRed, NUM_STEPS, SLEEP_MSEC);
        if (ret)
        {
            printk("Error %d: failed to set pulse width\n", ret);
            return 0;
        }

        ret = ledFadeInOut(&rotLedGreen, NUM_STEPS, SLEEP_MSEC);
        if (ret)
        {
            printk("Error %d: failed to set pulse width\n", ret);
            return 0;
        }

        ret = ledFadeInOut(&rotLedBlue, NUM_STEPS, SLEEP_MSEC);
        if (ret)
        {
            printk("Error %d: failed to set pulse width\n", ret);
            return 0;
        }
    }
    return 0;
}

void rotBtnPressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    printk("Button pressed at %" PRIu32 "\n", k_cycle_get_32());
}

int ledFadeInOut(const struct pwm_dt_spec *led, const uint32_t numSteps, const size_t timeoutMs)
{
    uint32_t step = led->period / numSteps;
    int ret       = 0;
    for (uint32_t i = 0; i < led->period; i += step)
    {
        ret = pwm_set_pulse_dt(led, i);
        if (ret)
        {
            return ret;
        }
        k_sleep(K_MSEC(timeoutMs));
    }

    for (uint32_t i = led->period; i > 0; i -= step)
    {
        ret = pwm_set_pulse_dt(led, i);
        if (ret)
        {
            return ret;
        }
        k_sleep(K_MSEC(timeoutMs));
    }
    ret = pwm_set_pulse_dt(led, 0);
    return ret;
}