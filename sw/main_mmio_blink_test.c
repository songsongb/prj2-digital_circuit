#define MMIO32(addr) (*(volatile unsigned int *)(addr))

#define HEX_DEBUG  MMIO32(0x10000020u)
#define LED_DEBUG  MMIO32(0x10000024u)

static void delay(void)
{
    volatile unsigned int i;

    for (i = 0u; i < 5000000u; i++) {
        ;
    }
}

int main(void)
{
    while (1) {
        LED_DEBUG = 0x00015555u;
        HEX_DEBUG = 0x00001234u;
        delay();

        LED_DEBUG = 0x0002AAAAu;
        HEX_DEBUG = 0x00004321u;
        delay();
    }
}
