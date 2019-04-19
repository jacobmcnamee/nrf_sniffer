#include <stdint.h>
#include <stdbool.h>

#include "nrf.h"
#include "crc.h"
#include "SEGGER_RTT.h"

//#define CHANNEL 90
//#define CHANNEL 65
#define CHANNEL 0

#define RADIO_BUFFER_SIZE 264
#define RADIO_RINGBUF_ELEMENTS_COUNT 32

typedef struct {
  uint8_t buffer[RADIO_BUFFER_SIZE];
  uint32_t timestamp;
  uint32_t rx_count;
  uint32_t overflow_count;
  uint8_t fifo_depth_max;
  uint8_t rssi;
} radio_ringbuf_element_t;

typedef struct {
  radio_ringbuf_element_t elements[RADIO_RINGBUF_ELEMENTS_COUNT];
  uint32_t write_index;
  uint32_t read_index;
} radio_ringbuf_t;

static radio_ringbuf_t radio_ringbuf = { .write_index = 0, .read_index = 0 };

static uint32_t rx_count = 0;
static uint32_t overflow_count = 0;
static uint32_t fifo_depth_max = 0;

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
  //.base_addr_p0       = { 0xE7, 0x83, 0x9A, 0x70 },
  .base_addr_p0       = { 0xE7, 0x12, 0xEE, 0x31 },
  .base_addr_p1       = { 0xC2, 0xC2, 0xC2, 0xC2 },
  .pipe_prefixes      = { 0xE7, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8 },
  .addr_length        = 5,
  .num_pipes          = 8,
  .rf_channel         = 2,
  .rx_pipes_enabled   = 0xFF
};

static uint32_t bytewise_bit_swap(uint8_t const * p_inp)
{
  uint32_t inp = (*(uint32_t*)p_inp);
#if __CORTEX_M == (0x04U)
  return __REV((uint32_t)__RBIT(inp));
#else
  inp = (inp & 0xF0F0F0F0) >> 4 | (inp & 0x0F0F0F0F) << 4;
  inp = (inp & 0xCCCCCCCC) >> 2 | (inp & 0x33333333) << 2;
  inp = (inp & 0xAAAAAAAA) >> 1 | (inp & 0x55555555) << 1;
  return inp;
#endif
}

static uint32_t addr_conv(uint8_t const* p_addr)
{
  return __REV(bytewise_bit_swap(p_addr));
}

static void radio_configure(void)
{
  NRF_RADIO->TXPOWER   = RADIO_TXPOWER_TXPOWER_0dBm;
  NRF_RADIO->FREQUENCY = CHANNEL;
  NRF_RADIO->MODE      = (RADIO_MODE_MODE_Ble_2Mbit << RADIO_MODE_MODE_Pos);
  NRF_RADIO->MODECNF0  = (NRF_RADIO->MODECNF0 & ~RADIO_MODECNF0_RU_Msk) |
                         (RADIO_MODECNF0_RU_Fast << RADIO_MODECNF0_RU_Pos);

  NRF_RADIO->PCNF0 = (1 << RADIO_PCNF0_S0LEN_Pos) |
                     (8 << RADIO_PCNF0_LFLEN_Pos) |
                     (3 << RADIO_PCNF0_S1LEN_Pos) ;

  NRF_RADIO->PCNF1 = (RADIO_PCNF1_WHITEEN_Disabled    << RADIO_PCNF1_WHITEEN_Pos) |
                     (RADIO_PCNF1_ENDIAN_Big          << RADIO_PCNF1_ENDIAN_Pos)  |
                     ((m_esb_addr.addr_length - 1)    << RADIO_PCNF1_BALEN_Pos)   |
                     (0                               << RADIO_PCNF1_STATLEN_Pos) |
                     (255                             << RADIO_PCNF1_MAXLEN_Pos);

  NRF_RADIO->BASE0 = addr_conv(m_esb_addr.base_addr_p0);
  NRF_RADIO->BASE1 = addr_conv(m_esb_addr.base_addr_p1);
  NRF_RADIO->PREFIX0 = bytewise_bit_swap(&m_esb_addr.pipe_prefixes[0]);
  NRF_RADIO->PREFIX1 = bytewise_bit_swap(&m_esb_addr.pipe_prefixes[4]);

  NRF_RADIO->TXADDRESS   = 0x00;
  NRF_RADIO->RXADDRESSES = 0xFF;

  NRF_RADIO->CRCINIT = 0xFFFFUL;      // Initial value
  NRF_RADIO->CRCPOLY = 0x11021UL;     // CRC poly: x^16+x^12^x^5+1
  NRF_RADIO->CRCCNF = RADIO_CRCCNF_LEN_Two << RADIO_CRCCNF_LEN_Pos;

  NRF_RADIO->SHORTS = RADIO_SHORTS_ADDRESS_RSSISTART_Msk |
                      RADIO_SHORTS_DISABLED_RSSISTOP_Msk;
}

static void radio_start(void)
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

  NVIC_SetPriority(RADIO_IRQn, 1);
  NVIC_EnableIRQ(RADIO_IRQn);
  NRF_RADIO->INTENSET = RADIO_INTENSET_END_Msk;
}

static void hw_init(void)
{
  // Start 16MHz crystal oscillator
  NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
  NRF_CLOCK->TASKS_HFCLKSTART = 1;
  while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) {
    ;
  }
}

static void timer_start(void)
{
  // Start TIMER0
  NRF_TIMER0->MODE = TIMER_MODE_MODE_Timer << TIMER_MODE_MODE_Pos;
  NRF_TIMER0->BITMODE =
    TIMER_BITMODE_BITMODE_32Bit << TIMER_BITMODE_BITMODE_Pos;
  NRF_TIMER0->PRESCALER = 4 << TIMER_PRESCALER_PRESCALER_Pos;
  NRF_TIMER0->TASKS_START = 1;
}

static void output_element(const radio_ringbuf_element_t *e)
{
  uint32_t buffer_length = 3 + e->buffer[1];

#ifdef TEXT_OUTPUT
  SEGGER_RTT_printf(0, "%6u: index = %u, length = %u, rssi = -%udBm\r\n",
                       e->rx_count, radio_ringbuf.read_index, buffer_length,
                       e->rssi);
  SEGGER_RTT_printf(0, "\ttimestamp = %u, ovf_cnt = %u, depth_max = %u\r\n",
                       e->timestamp, e->overflow_count, e->fifo_depth_max);

  SEGGER_RTT_printf(0, "\t");
  for (uint32_t i = 0; i < buffer_length; i++) {
    SEGGER_RTT_printf(0, "%02x ", e->buffer[i]);
  }
  SEGGER_RTT_printf(0, "\r\n");

  return;
#endif

  // Populate data
  uint8_t data[14];
  data[0] =  (e->timestamp >>  0) & 0xff;
  data[1] =  (e->timestamp >>  8) & 0xff;
  data[2] =  (e->timestamp >> 16) & 0xff;
  data[3] =  (e->timestamp >> 24) & 0xff;
  data[4] =  (e->rx_count >>  0) & 0xff;
  data[5] =  (e->rx_count >>  8) & 0xff;
  data[6] =  (e->rx_count >> 16) & 0xff;
  data[7] =  (e->rx_count >> 24) & 0xff;
  data[8] =  (e->overflow_count >>  0) & 0xff;
  data[9] =  (e->overflow_count >>  8) & 0xff;
  data[10] = (e->overflow_count >> 16) & 0xff;
  data[11] = (e->overflow_count >> 24) & 0xff;
  data[12] = e->fifo_depth_max;
  data[13] = e->rssi;

  // Populate header
  uint8_t header[8];
  uint16_t packet_length = sizeof(data) + buffer_length;
  header[0] = 'S';
  header[1] = 'F';
  header[2] = (packet_length >> 0) & 0xff;
  header[3] = (packet_length >> 8) & 0xff;

  // Populate header CRC
  uint32_t crc = crc_compute(header, 4);
  crc = crc_continue(data, sizeof(data), crc);
  crc = crc_continue(e->buffer, buffer_length, crc);
  header[4] = (crc >>  0) & 0xff;
  header[5] = (crc >>  8) & 0xff;
  header[6] = (crc >> 16) & 0xff;
  header[7] = (crc >> 24) & 0xff;

  // Write to RTT
  SEGGER_RTT_Write(0, header, sizeof(header));
  SEGGER_RTT_Write(0, data, sizeof(data));
  SEGGER_RTT_Write(0, e->buffer, buffer_length);
}

static void radio_ringbuf_process(void)
{
  // Atomic read of write index
  __disable_irq();
  uint32_t write_index = radio_ringbuf.write_index;
  __enable_irq();
  __DMB();

  while (radio_ringbuf.read_index != write_index) {

    const radio_ringbuf_element_t *e =
      &radio_ringbuf.elements[radio_ringbuf.read_index];

    output_element(e);

    uint32_t read_index_next =
      (radio_ringbuf.read_index + 1) % RADIO_RINGBUF_ELEMENTS_COUNT;

    // Atomic update of read index
    __DMB();
    __disable_irq();
    radio_ringbuf.read_index = read_index_next;
    __enable_irq();
  }
}

int main(void)
{
  hw_init();
  timer_start();

  SEGGER_RTT_printf(0, "RF sniffer startup\r\n");

  radio_configure();
  radio_start();

  while (1) {
    radio_ringbuf_process();
  }
}

void RADIO_IRQHandler(void)
{
  if (NRF_RADIO->EVENTS_END) {
    NRF_RADIO->EVENTS_END = 0;

    radio_ringbuf_element_t *e =
      &radio_ringbuf.elements[radio_ringbuf.write_index];

    // Populate timestamp
    NRF_TIMER0->TASKS_CAPTURE[0] = 1;
    e->timestamp = NRF_TIMER0->CC[0];

    // Populate RSSI
    e->rssi = NRF_RADIO->RSSISAMPLE;

    // Update write index
    uint32_t write_index_next =
      (radio_ringbuf.write_index + 1) % RADIO_RINGBUF_ELEMENTS_COUNT;

    if (write_index_next != radio_ringbuf.read_index) {
      radio_ringbuf.write_index = write_index_next;
    } else {
      // Overflow - just restart
      overflow_count++;
    }

    // Start next reception
    NRF_RADIO->PACKETPTR =
      (uint32_t)radio_ringbuf.elements[radio_ringbuf.write_index].buffer;
    NRF_RADIO->TASKS_START = 1;

    // Update max depth
    uint32_t depth = (radio_ringbuf.write_index +
                      RADIO_RINGBUF_ELEMENTS_COUNT -
                      radio_ringbuf.read_index) % RADIO_RINGBUF_ELEMENTS_COUNT;
    if (depth > fifo_depth_max) {
      fifo_depth_max = depth;
    }

    // Populate RX count
    e->rx_count = rx_count++;

    // Populate max depth
    e->fifo_depth_max = fifo_depth_max;

    // Populate overflow count
    e->overflow_count = overflow_count;
  }
}
