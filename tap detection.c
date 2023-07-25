#include <stdio.h>
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define INT_1                18
#define INT_2                19
#define SAMPLE_PERIOD_MS        100             // Sample period delay
#define I2C_SCL_IO          22// GPIO pin for I2C Wire Master Clock
#define I2C_SDA_IO          21// GPIO pin for I2C Wire Data Input/Output
#define I2C_FREQ_HZ             100000// Clock Frequency for I2C
#define I2C_PORT_NUM            I2C_NUM_1// Master Port Enable Macro
#define I2C_TX_BUF_DISABLE      0 // I2C Master don't need buffer
#define I2C_RX_BUF_DISABLE      0// I2C master don't need buffer

//I2C PROTOCAL
#define WRITE_BIT               I2C_MASTER_WRITE
#define READ_BIT                I2C_MASTER_READ
#define ACK_CHECK_EN            0x1
#define ACK_CHECK_DIS           0x0
#define ACK_VAL                 0x0
#define NACK_VAL                0x1

#define ADXL345_ADDR    0x53
#define DEVICE_ID       0x00 // Device ID, 0 by default


#define PWR_CTL_REG     0x2D
#define INT_ENABLE      0x2E //Interrupts
#define INT_MAP         0x2F //Interrupts
#define INT_SOURCE      0x30 //Interrupts
#define THRESH_TAP      0x1D
#define DUR             0x21
#define LATENT          0x22
#define WINDOW          0x23
#define TAP_AXES        0x2A
#define ACT_TAP_STATUS  0x2B 
#define DATA_FORMAT     0x31
int single_flag=0;
int double_flag=0;
uint8_t A;


void I2C_Master_Initialize()
{
    int i2c_master_port = I2C_PORT_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode,I2C_RX_BUF_DISABLE, I2C_TX_BUF_DISABLE, 0);
    printf ("INITIALIZED");
}

esp_err_t I2C_Master_Write_Slave_Reg(i2c_port_t i2c_num, uint8_t i2c_addr, uint8_t i2c_reg, uint8_t* data_wr, size_t size)
{
            i2c_cmd_handle_t cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            // first, send device address (indicating write) & register to be written
            i2c_master_write_byte(cmd, (i2c_addr << 1 ) | WRITE_BIT, ACK_CHECK_EN);
            //send register we want
            i2c_master_write_byte(cmd, i2c_reg, ACK_CHECK_EN);
            // write the data
            i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
            i2c_master_stop(cmd);
            esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
            i2c_cmd_link_delete(cmd);
             printf ("WRITTEN \n");
            return ret;
}

esp_err_t I2C_Master_Read_Slave_Reg(i2c_port_t i2c_num, uint8_t i2c_addr, uint8_t i2c_reg, uint8_t* data_rd, size_t size)
{
if (size == 0) {
    return ESP_OK;
}
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    // first, send device address (indicating write) & register to be read
    i2c_master_write_byte(cmd, ( i2c_addr << 1 ), ACK_CHECK_EN);
             //send register we wa
    i2c_master_write_byte(cmd, i2c_reg, ACK_CHECK_EN);
             //Send repeated start
    i2c_master_start(cmd);
             // now send device address (indicating read) & read data
    i2c_master_write_byte(cmd, ( i2c_addr << 1 ) | READ_BIT, ACK_CHECK_EN);
    if (size > 1)
  {
    i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
  }
  i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
  i2c_master_stop(cmd);
  esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  printf ("READ DONE\n");
  return ret;
}

esp_err_t ADXL345_REG_READER( uint8_t REG, uint8_t *P_DATA, uint8_t COUNT )
{
    return( I2C_Master_Read_Slave_Reg( I2C_PORT_NUM, ADXL345_ADDR,  REG, P_DATA, COUNT ) );

}


esp_err_t ADXL345_REG_WRITER( uint8_t reg, uint8_t *pdata, uint8_t count )
{
    return( I2C_Master_Write_Slave_Reg( I2C_PORT_NUM, ADXL345_ADDR,  reg, pdata, count ) );
}


void ADXL345_Initialize()
{
    //configuring reg
    uint8_t val;
    char *TAG = "ADXL345";
    ADXL345_REG_READER(DEVICE_ID, &(val), 1);
        if (val == 0xE5)
        {
            ESP_LOGI( TAG, "ADXL345 ID:0x%X (OK)", val );
        }
        else {
            ESP_LOGE( TAG, "ADXL345 ID:0x%X !!!! (NOT correct; should be 0xE5)", val );
        }
    val =0x08;
    ADXL345_REG_WRITER(PWR_CTL_REG,&(val), 1 );
    printf ("%d \n",val);
    val=0x0B;
    ADXL345_REG_WRITER(DATA_FORMAT,&(val), 1 );
    printf ("%d \n",val);
    val = 0x20;//int1=single and int2=double 
    ADXL345_REG_WRITER(INT_MAP, &(val), 1 );
    printf ("%d \n",val);
    val = 0x00;
    ADXL345_REG_WRITER(INT_ENABLE, &(val), 1 );
    val = 0x60;
    ADXL345_REG_WRITER(INT_ENABLE, &(val), 1 );
    printf ("%d \n",val);
    val = 0x60;//to trigger act and inact interrupts
    ADXL345_REG_WRITER(INT_SOURCE, &(val), 1 );
    printf ("%d \n",val);
    val=0x77;
    ADXL345_REG_WRITER(ACT_TAP_STATUS, &(val), 1 );
    printf ("%d \n",val);
    val=0x07;
    ADXL345_REG_WRITER(TAP_AXES,&(val), 1 );
    printf ("%d \n",val);
    val =0x20;
    ADXL345_REG_WRITER(THRESH_TAP, &(val), 1 );
    printf ("%d \n",val);
    val=0x10;
    ADXL345_REG_WRITER(DUR,&(val), 1 );
    printf ("%d \n",val);
    val=0x10;
    ADXL345_REG_WRITER(LATENT,&(val), 1 );
    printf ("%d \n",val);
    val=0xFF;
    ADXL345_REG_WRITER(WINDOW,&(val), 1 );
    printf ("%d \n",val);
    printf ("configured \n");

}

void IRAM_ATTR single_isr_handler(void* arg)
{
            single_flag = 1;
           //       gpio_isr_handler_remove(INT_1); // Remove the interrupt handler temporarily 
}

void IRAM_ATTR double_isr_handler(void* arg)
{
            double_flag = 1;
            single_flag = 0;
            //       gpio_isr_handler_remove(INT_2); // Remove the interrupt handler temporarily
}
void def()
{
        gpio_config_t io_conf;
        io_conf.intr_type=GPIO_INTR_ANYEDGE;
        io_conf.pin_bit_mask = (1ULL << INT_1);
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        gpio_config(&io_conf);

        io_conf.intr_type=GPIO_INTR_POSEDGE;
        io_conf.pin_bit_mask = (1ULL << INT_2);
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        gpio_config(&io_conf); 
        
       gpio_install_isr_service(0);
       gpio_isr_handler_add(INT_1, single_isr_handler, (void*)INT_1);
       gpio_isr_handler_add(INT_2, double_isr_handler, (void*)INT_2);
        printf ("gpio pins are configured\n");
}




void app_main(){
    I2C_Master_Initialize();
    ADXL345_Initialize();
    def();
    printf("done\n");
    int i,j;
    while(1)
    {
          ADXL345_REG_READER(INT_SOURCE,&(A),1);
          i=(1<<6)&A;
          j=(1<<5)&A;

        printf("%d\n",single_flag);
        printf("%d\n",double_flag);
        if (single_flag)
        {
           printf("single tap detected!\n");
           printf("%d ,%d\n",single_flag,i);
           single_flag=0;
        } 
        if (double_flag)
        {
           printf("double tap detected!\n");
           printf("%d ,%d\n",double_flag,j);
           double_flag=0;
        }
        
        printf("loop time\n");
        printf("%d\n",A);
         printf("%d\n",i);
          printf("%d\n",j);
    vTaskDelay(500);
    }
}


