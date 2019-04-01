#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "nrf.h"
#include "SEGGER_RTT.h"

#define CHANNEL 90

typedef struct
{
    uint8_t base_addr_p0[4];        /**< Base address for pipe 0 encoded in big endian. */
    uint8_t base_addr_p1[4];        /**< Base address for pipe 1-7 encoded in big endian. */
    uint8_t pipe_prefixes[8];       /**< Address prefix for pipe 0 to 7. */
    uint8_t num_pipes;              /**< Number of pipes available. */
    uint8_t addr_length;            /**< Length of the address including the prefix. */
    uint8_t rx_pipes_enabled;       /**< Bitfield for enabled pipes. */
    uint8_t rf_channel;             /**< Channel to use (must be between 0 and 100). */
} nrf_esb_address_t;

__ALIGN(4) static nrf_esb_address_t m_esb_addr = {
  //.base_addr_p0       = { 0xE7, 0xE7, 0xE7, 0xE7 },
  .base_addr_p0       = { 0xE7, 0x83, 0x9A, 0x70 },
  .base_addr_p1       = { 0xC2, 0xC2, 0xC2, 0xC2 },
  .pipe_prefixes      = { 0xE7, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8 },
  .addr_length        = 5,
  .num_pipes          = 8,
  .rf_channel         = 2,
  .rx_pipes_enabled   = 0xFF
};

#define RADIO_BUFFER_SIZE 260
#define RADIO_RINGBUF_ELEMENTS_COUNT 32

typedef struct {
  uint8_t buffer[RADIO_BUFFER_SIZE];
  //uint8_t rssi;
} radio_ringbuf_element_t;

typedef struct {
  radio_ringbuf_element_t elements[RADIO_RINGBUF_ELEMENTS_COUNT];
  uint32_t write_index;
  uint32_t read_index;
} radio_ringbuf_t;

static radio_ringbuf_t radio_ringbuf;

static volatile uint32_t overflow_count = 0;
static volatile uint32_t depth_max = 0;

// Function to do bytewise bit-swap on an unsigned 32-bit value
static uint32_t bytewise_bit_swap(uint8_t const * p_inp)
{
    uint32_t inp = (*(uint32_t*)p_inp);
#if __CORTEX_M == (0x04U)
    return __REV((uint32_t)__RBIT(inp)); //lint -esym(628, __rev) -esym(526, __rev) -esym(628, __rbit) -esym(526, __rbit) */
#else
    inp = (inp & 0xF0F0F0F0) >> 4 | (inp & 0x0F0F0F0F) << 4;
    inp = (inp & 0xCCCCCCCC) >> 2 | (inp & 0x33333333) << 2;
    inp = (inp & 0xAAAAAAAA) >> 1 | (inp & 0x55555555) << 1;
    return inp;
#endif
}


// Internal function to convert base addresses from nRF24L type addressing to nRF51 type addressing
static uint32_t addr_conv(uint8_t const* p_addr)
{
    return __REV(bytewise_bit_swap(p_addr)); //lint -esym(628, __rev) -esym(526, __rev) */
}

static void radio_configure(void)
{
  NRF_RADIO->TXPOWER   = RADIO_TXPOWER_TXPOWER_0dBm;
  NRF_RADIO->FREQUENCY = CHANNEL;
  NRF_RADIO->MODE      = (RADIO_MODE_MODE_Ble_2Mbit << RADIO_MODE_MODE_Pos);
  NRF_RADIO->MODECNF0  = (NRF_RADIO->MODECNF0 & ~RADIO_MODECNF0_RU_Msk) |
                         (RADIO_MODECNF0_RU_Fast << RADIO_MODECNF0_RU_Pos);

  NRF_RADIO->PCNF0 = (0 << RADIO_PCNF0_S0LEN_Pos) |
                     (8 << RADIO_PCNF0_LFLEN_Pos) |
                     (3 << RADIO_PCNF0_S1LEN_Pos) ;

  NRF_RADIO->PCNF1 = (RADIO_PCNF1_WHITEEN_Disabled    << RADIO_PCNF1_WHITEEN_Pos) |
                     (RADIO_PCNF1_ENDIAN_Big          << RADIO_PCNF1_ENDIAN_Pos)  |
                     ((m_esb_addr.addr_length - 1)    << RADIO_PCNF1_BALEN_Pos)   |
                     (0                               << RADIO_PCNF1_STATLEN_Pos) |
                     (252                             << RADIO_PCNF1_MAXLEN_Pos);

  NRF_RADIO->BASE0 = addr_conv(m_esb_addr.base_addr_p0);
  NRF_RADIO->BASE1 = addr_conv(m_esb_addr.base_addr_p1);
  NRF_RADIO->PREFIX0 = bytewise_bit_swap(&m_esb_addr.pipe_prefixes[0]);
  NRF_RADIO->PREFIX1 = bytewise_bit_swap(&m_esb_addr.pipe_prefixes[4]);

  NRF_RADIO->TXADDRESS   = 0x00;
  NRF_RADIO->RXADDRESSES = 0xFF;

  NRF_RADIO->CRCINIT = 0xFFFFUL;      // Initial value
  NRF_RADIO->CRCPOLY = 0x11021UL;     // CRC poly: x^16+x^12^x^5+1
  NRF_RADIO->CRCCNF = RADIO_CRCCNF_LEN_Two << RADIO_CRCCNF_LEN_Pos;
}

static void clock_initialization(void)
{
    /* Start 16 MHz crystal oscillator */
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART    = 1;

    /* Wait for the external oscillator to start up */
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
    {
        // Do nothing.
    }

    /* Start low frequency crystal oscillator for app_timer(used by bsp)*/
    NRF_CLOCK->LFCLKSRC            = (CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos);
    NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_LFCLKSTART    = 1;

    while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0)
    {
        // Do nothing.
    }
}


static int read_packet(void)
{
  int ret = -1;

    NRF_RADIO->EVENTS_READY = 0U;
    // Enable radio and wait for ready
    NRF_RADIO->TASKS_RXEN = 1U;

    while (NRF_RADIO->EVENTS_READY == 0U)
    {
        // wait
    }
    NRF_RADIO->EVENTS_END = 0U;
    // Start listening and wait for address received event
    NRF_RADIO->TASKS_START = 1U;

    // Wait for end of packet or buttons state changed
    while (NRF_RADIO->EVENTS_END == 0U)
    {
        // wait
    }

    if (NRF_RADIO->CRCSTATUS == 1U)
    {
      ret = 0;
    }
    NRF_RADIO->EVENTS_DISABLED = 0U;
    // Disable radio
    NRF_RADIO->TASKS_DISABLE = 1U;

    while (NRF_RADIO->EVENTS_DISABLED == 0U)
    {
        // wait
    }
    return ret;
}

static void radio_prepare(void)
{
  NRF_RADIO->EVENTS_READY = 0;
  NRF_RADIO->TASKS_RXEN = 1;
  while (NRF_RADIO->EVENTS_READY == 0) {
    ;
  }

  NRF_RADIO->PACKETPTR =
    (uint32_t)radio_ringbuf.elements[radio_ringbuf.write_index].buffer;

  NRF_RADIO->EVENTS_END = 0;
  NRF_RADIO->TASKS_START = 1;
}

void RADIO_IRQHandler(void)
{
  if (NRF_RADIO->EVENTS_END) {
    NRF_RADIO->EVENTS_END = 0;

    uint32_t write_index_next =
      (radio_ringbuf.write_index + 1) % RADIO_RINGBUF_ELEMENTS_COUNT;

    if (write_index_next != radio_ringbuf.read_index) {
      radio_ringbuf.write_index = write_index_next;
    } else {
      // Overflow - just restart
      overflow_count++;
    }

    uint32_t depth = (radio_ringbuf.write_index +
                      RADIO_RINGBUF_ELEMENTS_COUNT -
                      radio_ringbuf.read_index) % RADIO_RINGBUF_ELEMENTS_COUNT;
    if (depth > depth_max) {
      depth_max = depth;
    }

    NRF_RADIO->PACKETPTR =
      (uint32_t)radio_ringbuf.elements[radio_ringbuf.write_index].buffer;
    NRF_RADIO->TASKS_START = 1;
  }
}

volatile uint8_t rx_lens[1024] = { 0 };
volatile uint32_t rx_lens_i = 0;

int main(void)
{
    clock_initialization();

    //err_code = NRF_LOG_INIT(NULL);
    //APP_ERROR_CHECK(err_code);
    //NRF_LOG_DEFAULT_BACKENDS_INIT();

    radio_ringbuf.write_index = 0;
    radio_ringbuf.read_index = 0;

    // Set radio configuration parameters
    radio_configure();
    //NRF_RADIO->PACKETPTR = (uint32_t)radio_packet;

    NVIC_SetPriority(RADIO_IRQn, 1);
    NVIC_EnableIRQ(RADIO_IRQn);
    NRF_RADIO->INTENSET = RADIO_INTENSET_END_Msk;

    radio_prepare();

    SEGGER_RTT_printf(0, "RF sniffer startup\r\n");
    //NRF_LOG_INFO("Wait for first packet");
    //NRF_LOG_FLUSH();

    uint32_t rx_count = 0;
    while (1) {
      __disable_irq();
      uint32_t write_index = radio_ringbuf.write_index;
      __enable_irq();
      __DMB();

      while (radio_ringbuf.read_index != write_index) {
        rx_count++;

        uint8_t length =
          radio_ringbuf.elements[radio_ringbuf.read_index].buffer[0];

        SEGGER_RTT_printf(0, "%6u: index = %u, length = %u\r\n",
                             rx_count, radio_ringbuf.read_index, length);
        SEGGER_RTT_printf(0, "\tovf_cnt = %u, depth_max = %u\r\n",
                             overflow_count, depth_max);

        //NRF_LOG_INFO("%6u: index = %u, length = %u",
        //             rx_count, radio_ringbuf.read_index, length);
        //NRF_LOG_INFO("\t\tovf_cnt = %u, depth_max = %u",
        //             overflow_count, depth_max);
        //NRF_LOG_FLUSH();

        uint32_t read_index_next =
          (radio_ringbuf.read_index + 1) % RADIO_RINGBUF_ELEMENTS_COUNT;

        __DMB();
        __disable_irq();
        radio_ringbuf.read_index = read_index_next;
        __enable_irq();
      }

    }



    while (true)
    {
        if (read_packet() == 0) {
          //uint8_t packet[sizeof(radio_packet)];
          //memcpy(packet, radio_packet, sizeof(packet));

          rx_count++;
          //bsp_indication_set(BSP_INDICATE_RCV_OK);
          //nrf_gpio_pin_toggle(17);
          //SEGGER_RTT_printf(0, "%6u: len = %u\r\n", rx_count, radio_packet[0]);
          //NRF_LOG_INFO("%6u: len = %u", rx_count, radio_packet[0]);
          //NRF_LOG_INFO("%6u: len = %u", rx_count, packet[0]);
          //NRF_LOG_INFO("%6u", rx_count);
          //NRF_LOG_FLUSH();
          //for (volatile uint32_t i = 0; i < 10; i++) {
          //  ;
          //}
          //nrf_delay_us(5);
          //if (rx_lens_i < sizeof(rx_lens)) {
          //  rx_lens[rx_lens_i++] = radio_packet[0];
          //  __asm volatile("" ::: "memory");
          //}
        }
    }
}
