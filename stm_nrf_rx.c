#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>
#include "pca96.h"

#define NRF24_NODE        DT_NODELABEL(nrf24)
#define NRF24_SPI_NODE    DT_PARENT(NRF24_NODE)

#define MAX_PWM 2500000
#define MIN_PWM 500000
#define MAX_DA 255
#define MIN_DA 0

DataDecoded data_pca;

/// Stack for receving  thread
#define STACK_SIZE 1024
static K_THREAD_STACK_DEFINE(pca_receive_stack, STACK_SIZE);
static struct k_thread pca_receive_data;

static const struct device *spi_dev = DEVICE_DT_GET(NRF24_SPI_NODE);

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

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

// =========================================================
// Helper macros
// =========================================================
#define PAYLOAD_LENGTH 5 /// previosuly 7

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
    uint8_t tx_buf[2] = {0};

    struct spi_buf tx = { .buf = tx_buf, .len = 2 };
    struct spi_buf_set txs = { .buffers = &tx, .count = 1 };

    tx_buf[0] = W_REGISTER | (reg & REGISTER_MASK);
    tx_buf[1] = value;

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


static void nrf_read_payload(uint8_t *buffer, uint8_t len){

    uint8_t tx_buffer[len+1];
    uint8_t rx_buffer[len+1];

    struct spi_buf tx = {
        .buf = tx_buffer,
        .len = len + 1
    };

    struct spi_buf_set txs = {
        .buffers = &tx,
        .count = 1
    };

    struct spi_buf rx = {
        .buf = rx_buffer,
        .len = len + 1
    };

    struct spi_buf_set rxs = {
        .buffers = &rx,
        .count = 1
    };

    tx_buffer[0] = R_RX_PAYLOAD;
    memset(&tx_buffer[1], NRF_NOP, len);

    int ret = spi_transceive(spi_dev, &spi_cfg, &txs, &rxs);

    if (ret != 0){
        printk("nrf read payload failed\n");
        return;
    }

    memcpy(buffer, &rx_buffer[1], len);

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

    nrf_write_buffer(RX_ADDR_P0, addr, len);
    nrf_write_reg(RX_PW_P0, data_size);

     // CONFIG: PWR_UP=1, PRIM_RX=1, EN_CRC=0
    nrf_write_reg(CONFIG_REG, 0x03);
    k_msleep(10);
    ce_high();

}

void pca_receive_thread(void *p1, void *p2, void *p3){


    uint8_t temp_buf[10];
    
     while (1){ 
        uint8_t status = 0; 
        nrf_read_reg(STATUS_REG, &status); 
        if (status & (1<<6)){ 
            nrf_read_payload(temp_buf, PAYLOAD_LENGTH); 
            nrf_write_reg(STATUS_REG, 1<<6); 
            temp_buf[PAYLOAD_LENGTH] = '\0'; 
        
            data_pca.f1 = temp_buf[0];
            data_pca.f2 = temp_buf[1];
            data_pca.f3 = temp_buf[2];
            data_pca.f4 = temp_buf[3];
            data_pca.f5 = temp_buf[4];

            pca_pwm_control(&data_pca, MIN_DA, MAX_DA, MIN_PWM, MAX_PWM);

            printk("Payload received and sent: %d, %d, %d, %d, %d \n", temp_buf[0], temp_buf[1], temp_buf[2], temp_buf[3], temp_buf[4]); 

       
    } 
    k_msleep(5); 
}

}


void main(void)
{
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

    pca_init();
    k_msleep(100);
    gpio_pin_configure_dt(&ce_gpio, GPIO_OUTPUT_INACTIVE);
    //ce_low();

    uint8_t val;

    k_msleep(150);

     if (nrf_read_reg(STATUS_REG, &val) == 0) {
            printk("STATUS_REG = 0x%02X\n", val);
    }

    k_msleep(150);

     if (nrf_read_reg(CONFIG_REG, &val) == 0) {
            printk("CONFIG_REG = 0x%02X\n", val);
    }

    k_msleep(1000);
    uint8_t addr[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};

    nrf_init(addr,5, 5);

    if (nrf_read_reg(STATUS_REG, &val) == 0) {
            printk("STATUS_REG After = 0x%02X\n", val);
    }

    k_msleep(150);

     if (nrf_read_reg(CONFIG_REG, &val) == 0) {
            printk("CONFIG_REG After = 0x%02X\n", val);
    }

    //uint8_t temp_buf[10];
    
    printk("Start reading payload...\n"); 
//     while (1){ 
//         uint8_t status = 0; 
//         nrf_read_reg(STATUS_REG, &status); 
//         if (status & (1<<6)){ 
//             nrf_read_payload(temp_buf, PAYLOAD_LENGTH); 
//             nrf_write_reg(STATUS_REG, 1<<6); 
//             temp_buf[PAYLOAD_LENGTH] = '\0'; 
        
//         if (sizeof(temp_buf) == 5){ 
//             printk("Payload received: %d, %d, %d, %d, %d \n", temp_buf[0], temp_buf[1], temp_buf[2], temp_buf[3], temp_buf[4]); 
//         } 
//         else{ 
//             printk("Payload received: %d, %d, %d, %d, %d \n", temp_buf[0], temp_buf[1], temp_buf[2], temp_buf[3], temp_buf[4]); 
        
//         } 
//     } 
//     k_msleep(5); 
// }
    
     k_tid_t print_tid = k_thread_create(&pca_receive_data, pca_receive_stack, K_THREAD_STACK_SIZEOF(pca_receive_stack), pca_receive_thread, NULL, NULL, NULL, 3, 0, K_NO_WAIT);
    
}
