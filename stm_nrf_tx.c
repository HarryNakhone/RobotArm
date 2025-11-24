#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include <zephyr/random/random.h>
#include <zephyr/sys/printk.h>
#include "flexsen.h"
#include <string.h>

/// Stack for reading Flex sensors thread
#define STACK_SIZE 1024
static K_THREAD_STACK_DEFINE(adc_stack, STACK_SIZE);
static struct k_thread adc_thread_data;

/// Stack for printing the thread
static K_THREAD_STACK_DEFINE(print_stack, STACK_SIZE);
static struct k_thread print_thread_data;

/// Struct to hold flex sensor data (from flexsensor.h)
Data_sen local_flex_data;

/// nrf24 node from devicetree
#define NRF24_NODE        DT_NODELABEL(nrf24)
#define NRF24_SPI_NODE    DT_PARENT(NRF24_NODE)

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

static const struct device *spi_dev = DEVICE_DT_GET(NRF24_SPI_NODE);

// CE pin from ce_pin node
static const struct gpio_dt_spec ce_gpio =
    GPIO_DT_SPEC_GET(DT_NODELABEL(ce_pin), gpios);

// CSN pin from SPI controller's cs-gpios
static struct spi_cs_control spi_cs = {
    .gpio = GPIO_DT_SPEC_GET(NRF24_SPI_NODE, cs_gpios),
    .delay = 0,
};

// SPI config
static const struct spi_config spi_cfg = {
    .frequency = 500000,
    .operation = SPI_WORD_SET(8) |
                 SPI_TRANSFER_MSB |
                 SPI_OP_MODE_MASTER,
    .slave = DT_REG_ADDR(NRF24_NODE),
    .cs = &spi_cs,
};
typedef union{
    struct {
       uint8_t f_1;
       uint8_t f_2;
       uint8_t f_3;
       uint8_t f_4;
       uint8_t f_5;
    //    uint8_t yaw;
    //    uint8_t pitch;
    } message_t;

    uint8_t data[5];
} union_message_t;

///Declare global message data
 union_message_t message_data;

#define PAYLOAD_LENGTH 5

#define R_REGISTER     0x00
#define W_REGISTER     0x20
#define REGISTER_MASK  0x1F
#define R_RX_PAYLOAD   0x61
#define W_TX_PAYLOAD   0xA0
#define FLUSH_RX       0xE2
#define FLUSH_TX       0xE1
#define CONFIG_REG     0x00
                                /// R_REGISTER | (reg & REGISTER_MASK); 0b0000 0111  0b0001 1111
                                /// 0x00 0b0000 0000 
#define EN_AA          0x01
#define EN_RXADDR      0x02
#define SETUP_AW       0x03
#define SETUP_RETR     0x04
#define RF_CH          0x05
#define RF_SETUP       0x06
#define STATUS_REG     0x07
#define RX_ADDR_P0     0x0A
#define TX_ADDR        0x10
#define RX_PW_P0       0x11
#define FIFO_STATUS    0x17

#define NRF_NOP        0xFF


static inline void ce_high(void)  { gpio_pin_set_dt(&ce_gpio, 1); }
static inline void ce_low(void)   { gpio_pin_set_dt(&ce_gpio, 0); }


static int spi_txrx(uint8_t *tx, uint8_t *rx, uint8_t value)
{
    struct spi_buf txb = { .buf = tx, .len = 1 };
    struct spi_buf_set tx_set = { .buffers = &txb, .count = 1 };

    struct spi_buf rxb = { .buf = rx, .len = 1 };
    struct spi_buf_set rx_set = { .buffers = &rxb, .count = 1 };
    
    tx[0] = value;

    return spi_transceive(spi_dev, &spi_cfg, &tx_set, &rx_set);
}

static int nrf_read_reg(uint8_t reg, uint8_t *value)
{
    uint8_t tx_buf[2] = {
        R_REGISTER | (reg & REGISTER_MASK),
        NRF_NOP
    };
    uint8_t rx_buf[2];

    struct spi_buf tx = { .buf = tx_buf, .len = 2 };
    struct spi_buf_set txs = { .buffers = &tx, .count = 1 };

    struct spi_buf rx = { .buf = rx_buf, .len = 2 };
    struct spi_buf_set rxs = { .buffers = &rx, .count = 1 };

    int ret = spi_transceive(spi_dev, &spi_cfg, &txs, &rxs);
    if (ret == 0) {
        *value = rx_buf[1];
    }

    return ret;
}

static int nrf_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t tx_buf[2] = {
        W_REGISTER | (reg & REGISTER_MASK),
        value
    };

    struct spi_buf tx = { .buf = tx_buf, .len = 2 };
    struct spi_buf_set txs = { .buffers = &tx, .count = 1 };

    return spi_write(spi_dev, &spi_cfg, &txs);
}

static int nrf_write_buffer(uint8_t reg, const uint8_t *buf, size_t len){

    uint8_t frame = W_REGISTER | (reg & REGISTER_MASK);


    uint8_t tx_buf[len + 1];

    struct spi_buf tx = {.buf = tx_buf, .len = len + 1};
    struct spi_buf_set txs =  {.buffers  =&tx, .count =1};

    tx_buf[0] = frame;

    memcpy(&tx_buf[1], buf, len);



    return spi_write(spi_dev, &spi_cfg, &txs);

}

static void nrf_cmd(uint8_t reg){
    uint8_t tx[1];
    uint8_t rx[1];

    int ret = spi_txrx(tx, rx, reg);

    if (ret != 0){
        printk("nrf cmd failed\n");
        return;
    }

}

static void nrf_write_payload(uint8_t *buffer, uint8_t len){

    uint8_t tx_buf[len + 1];

    struct spi_buf tx = {
        .buf = tx_buf,
        .len = len + 1
    };

    struct spi_buf_set txs = {
        .buffers = &tx,
        .count = 1
    };

    tx_buf[0] = W_TX_PAYLOAD;

    memcpy(&tx_buf[1], buffer, len);
    
    int ret = spi_write(spi_dev, &spi_cfg, &txs);

    if (ret != 0){
        printk("nrf write payload failed\n");
        return;
    }

}

static void nrf_send_payload(uint8_t *buffer, uint8_t len){

    ce_low();

    nrf_write_payload(buffer, len);

    ce_high();

    k_busy_wait(20);

    ce_low();

    k_busy_wait(320);

    nrf_write_reg(STATUS_REG, 0x70);

    nrf_cmd(FLUSH_TX);


}

static void nrf_init(const uint8_t addr[], uint8_t len, uint8_t data_size){
    ce_low();
    nrf_write_reg(STATUS_REG, 0x70);
    nrf_cmd(FLUSH_RX);
    nrf_cmd(FLUSH_TX);


    nrf_write_reg(EN_AA, 0x00);
    nrf_write_reg(EN_RXADDR, 0x01);
    nrf_write_reg(SETUP_AW, 0x03);
    nrf_write_reg(SETUP_RETR, 0x00);
    nrf_write_reg(RF_CH, 2);
    nrf_write_reg(RF_SETUP, 0x06);

    nrf_write_buffer(TX_ADDR, addr, len);
    nrf_write_buffer(RX_ADDR_P0, addr, len);
    nrf_write_reg(RX_PW_P0, data_size);

    nrf_write_reg(CONFIG_REG, 0x02);
    k_msleep(16);

}

///***************** Int converter ************////
uint8_t convert_to_u8(int32_t adc, int16_t min, int16_t max)
{

    if (adc < min) adc = min;
    if (adc > max) adc = max;

    return (uint8_t)((adc - min) * 255 / (max - min));
}


///***************** Flex Sensor ************////

static void flex_thread (void *p1, void *p2, void *p3){
    flex_read_all();
}

static void print_sensor(void *p1, void *p2, void *p3){
    while (1){
        if(k_mutex_lock(&sensor_mutex, K_MSEC(5))==0){
            //local_flex_data = data_sen;

            message_data.message_t.f_1 = convert_to_u8(data_sen.values[0], 620, 1220);
            message_data.message_t.f_2 = convert_to_u8(data_sen.values[1], 600, 1230);
            message_data.message_t.f_3 = convert_to_u8(data_sen.values[2], 470, 1020);
            message_data.message_t.f_4 = convert_to_u8(data_sen.values[3], 540, 1140);
            message_data.message_t.f_5 = convert_to_u8(data_sen.values[4], 480, 1220);

            nrf_send_payload(message_data.data, sizeof(message_data.data));

            printk("Data: %d, %d, %d, %d, %d \n", message_data.message_t.f_1, message_data.message_t.f_2, message_data.message_t.f_3, message_data.message_t.f_4, message_data.message_t.f_5);
            k_mutex_unlock(&sensor_mutex);

        } else{
            printk("Could not lock mutex\n");   
        }
        k_sleep(K_MSEC(20));
    }
}

void main(void)
{

    uint8_t val;
    printk("NRF24 Zephyr-based test starting...\n");

    if (!device_is_ready(spi_dev)) {
        printk("SPI DEV NOT READY!\n");
        return;
    }

    // Configure CE pin
    if (!device_is_ready(ce_gpio.port)) {
        printk("CE GPIO not ready!\n");
        return;
    }
    gpio_pin_configure_dt(&ce_gpio, GPIO_OUTPUT_INACTIVE);
    ce_low();

    k_msleep(1000);
    uint8_t addr[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};

    nrf_init(addr, 5, 5); /// previously 7

    if (nrf_read_reg(STATUS_REG, &val) == 0) {
            printk("STATUS_REG After = 0x%02X\n", val);
    }

    k_msleep(150);

     if (nrf_read_reg(CONFIG_REG, &val) == 0) {
            printk("CONFIG_REG After = 0x%02X\n", val);
    }

     if (!flex_init()){

        printk("Unable to set up flex init");
        return;
    }

    k_tid_t adc_tid = k_thread_create(&adc_thread_data, adc_stack, K_THREAD_STACK_SIZEOF(adc_stack), flex_thread, NULL, NULL, NULL, 4, 0, K_NO_WAIT);

    k_tid_t print_tid = k_thread_create(&print_thread_data, print_stack, K_THREAD_STACK_SIZEOF(print_stack), print_sensor, NULL, NULL, NULL, 5, 0, K_NO_WAIT);

    // while (1){
    //     message_data.message_t.f_1 = sys_rand8_get();
    //     message_data.message_t.f_2 = sys_rand8_get();
    //     message_data.message_t.f_3 = sys_rand8_get();
    //     message_data.message_t.f_4 = sys_rand8_get();
    //     message_data.message_t.f_5 = sys_rand8_get();
      
    //     printk("Sending payload...\n");

    //     nrf_send_payload(message_data.data, sizeof(message_data.data));


    //     printk("Payload: ");

    //     LOG_HEXDUMP_INF(message_data.data, sizeof(message_data.data), "SENT: ");
      

    //     k_msleep(2000);
    
    // }
}
