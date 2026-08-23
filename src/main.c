/* Bare-metal LED blink for FRDM-MCXA153 (MCX A153, Arm Cortex-M33).
 *
 * The on-board RGB LED is wired to GPIO3, active-low:
 *   red   = GPIO3 pin 12
 *   green = GPIO3 pin 13
 *   blue  = GPIO3 pin 0
 * (per NXP's board files / Zephyr's frdm_mcxa153 devicetree).
 */
#include "fsl_device_registers.h"
#include "rtt.h"

#define LED_RED_PIN   12U
#define LED_GREEN_PIN 13U
#define LED_BLUE_PIN  0U

static void delay(volatile uint32_t count)
{
    while (count--)
    {
        __asm volatile("nop");
    }
}

int main(void)
{
    rtt_init();
    rtt_puts("main() entered\r\n");

    /* MRCC clock/reset control registers are write-protected by a lock bit
     * in SYSCON; must unlock before touching them and re-lock afterward
     * (mirrors what NXP's CLOCK_EnableClock/RESET_ReleasePeripheralReset do). */
    SYSCON->CLKUNLOCK &= ~SYSCON_CLKUNLOCK_UNLOCK_MASK;

    /* Enable clocks to PORT3 (pin mux) and GPIO3. */
    MRCC0->MRCC_GLB_CC1_SET = MRCC_MRCC_GLB_CC1_PORT3_MASK | MRCC_MRCC_GLB_CC1_GPIO3_MASK;
    rtt_puts("clocks enabled\r\n");

    /* Release PORT3 and GPIO3 from reset. NOTE: on this MRCC block, writing
     * the _SET register sets the "out of reset" bit to 1 (i.e. _SET releases
     * the peripheral, _CLR would re-assert reset) -- confirmed against NXP's
     * RESET_ReleasePeripheralReset(), which calls RESET_SetPeripheralReset(). */
    MRCC0->MRCC_GLB_RST1_SET = MRCC_MRCC_GLB_RST1_PORT3_MASK | MRCC_MRCC_GLB_RST1_GPIO3_MASK;
    rtt_puts("reset released\r\n");

    SYSCON->CLKUNLOCK |= SYSCON_CLKUNLOCK_UNLOCK_MASK;

    /* Route the three LED pins to GPIO (MUX = ALT0 on this device). */
    PORT3->PCR[LED_RED_PIN]   = PORT_PCR_MUX(0U);
    PORT3->PCR[LED_GREEN_PIN] = PORT_PCR_MUX(0U);
    PORT3->PCR[LED_BLUE_PIN]  = PORT_PCR_MUX(0U);
    rtt_puts("pinmux done\r\n");

    /* Drive LEDs high (off, active-low) and set them as outputs. */
    GPIO3->PSOR = (1U << LED_RED_PIN) | (1U << LED_GREEN_PIN) | (1U << LED_BLUE_PIN);
    GPIO3->PDDR |= (1U << LED_RED_PIN) | (1U << LED_GREEN_PIN) | (1U << LED_BLUE_PIN);
    rtt_puts("gpio configured, entering loop\r\n");

    while (1)
    {
        GPIO3->PTOR = (1U << LED_RED_PIN);
        rtt_puts("toggle\r\n");
        delay(1500000U);
    }
}
