#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>
#include <math.h>

#include "pico/stdlib.h"
#include <stdio.h>

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "mpu6050.h"

#include "Fusion.h"
#define SAMPLE_PERIOD (0.01f) 

typedef struct {
    int eixo;
    int valor;
} adc_t;

QueueHandle_t xQueueADC;

const int MPU_ADDRESS = 0x68;
const int I2C_SDA_GPIO = 4;
const int I2C_SCL_GPIO = 5;
const int PIN_BTN_RESET = 16;
const int PIN_BTN_CLICK = 19;
const int PIN_MACRO = 10;
const int PIN_MUTE = 22;

void btn_callback(uint gpio, uint32_t events) {
    adc_t adc;
    if (events & GPIO_IRQ_EDGE_FALL) { 
        if (gpio == PIN_BTN_CLICK) {
            adc.eixo = 2;
        } else if (gpio == PIN_BTN_RESET){
            adc.eixo = -1;
        } else if (gpio == PIN_MUTE){
            adc.eixo = 6;
        } else if (gpio==PIN_MACRO){
            adc.eixo = 5;
        }
    } else {
        if (gpio == PIN_BTN_CLICK){
            adc.eixo = 10;
        } 
    }
    xQueueSendFromISR(xQueueADC, &adc, 0);
}

static void mpu6050_reset() {
    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    uint8_t buf[] = {0x6B, 0x00};
    i2c_write_blocking(i2c_default, MPU_ADDRESS, buf, 2, false);
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.

    uint8_t buffer[6];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    uint8_t val = 0x3B;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true); // true to keep master control of bus
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);

    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Now gyro data from reg 0x43 for 6 bytes
    // The register is auto incrementing on each read
    val = 0x43;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);  // False - finished with bus

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);;
    }

    // Now temperature from reg 0x41 for 2 bytes
    // The register is auto incrementing on each read
    val = 0x41;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 2, false);  // False - finished with bus

    *temp = buffer[0] << 8 | buffer[1];
}

void mpu6050_task(void *p) {
    // configuracao do I2C
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(I2C_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_GPIO);
    gpio_pull_up(I2C_SCL_GPIO);

    mpu6050_reset();
    int16_t acceleration[3], gyro[3], temp;
    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);
    while(1) {
        mpu6050_read_raw(acceleration, gyro, &temp);

        FusionVector gyroscope = {
            .axis.x = gyro[0] / 131.0f,
            .axis.y = gyro[1] / 131.0f,
            .axis.z = gyro[2] / 131.0f,
        };
    
        FusionVector accelerometer = {
            .axis.x = acceleration[0] / 16384.0f,
            .axis.y = acceleration[1] / 16384.0f,
            .axis.z = acceleration[2] / 16384.0f,
        };
    
        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);
    
        const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

        adc_t adc;
        adc.eixo = 0;
        adc.valor = euler.angle.pitch*0.5;
        xQueueSend(xQueueADC, &adc, 0);

        adc.eixo = 1;
        adc.valor = euler.angle.roll * -1*0.5;
        xQueueSend(xQueueADC, &adc, 0);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void uart_task(void *p) {
    adc_t data;
    while (1) {
        if (xQueueReceive(xQueueADC, &data, 100)) {
            if (data.eixo == -1) {
                mpu6050_reset();
            } else {
                uint8_t bytes[4];
                bytes[0] = 0xFF; 
                bytes[1] = (uint8_t)(data.eixo);
                bytes[2] = (data.valor >> 8) & 0xFF;
                bytes[3] = data.valor & 0xFF;
                uart_write_blocking(uart0, bytes, 4);
            }
        }
    }
}

int main() {

    stdio_init_all();
    xQueueADC = xQueueCreate(4, sizeof(adc_t));

    gpio_init(PIN_BTN_CLICK);
    gpio_set_dir(PIN_BTN_CLICK, GPIO_IN);
    gpio_pull_up(PIN_BTN_CLICK);

    gpio_init(PIN_BTN_RESET);
    gpio_set_dir(PIN_BTN_RESET, GPIO_IN);
    gpio_pull_up(PIN_BTN_RESET);

    gpio_init(PIN_MUTE);
    gpio_set_dir(PIN_MUTE, GPIO_IN);
    gpio_pull_up(PIN_MUTE);

    gpio_init(PIN_MACRO);
    gpio_set_dir(PIN_MACRO, GPIO_IN);
    gpio_pull_up(PIN_MACRO);

    gpio_set_irq_enabled_with_callback(PIN_BTN_CLICK, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, &btn_callback);
    gpio_set_irq_enabled_with_callback(PIN_MUTE, GPIO_IRQ_EDGE_FALL, true, &btn_callback);
    gpio_set_irq_enabled_with_callback(PIN_BTN_RESET, GPIO_IRQ_EDGE_FALL, true, &btn_callback);
    gpio_set_irq_enabled_with_callback(PIN_MACRO, GPIO_IRQ_EDGE_FALL, true, &btn_callback);

    xTaskCreate(mpu6050_task, "mpu6050_Task 1", 8192, NULL, 1, NULL);
    xTaskCreate(uart_task, "uart task", 4095, NULL, 1, NULL);

    vTaskStartScheduler();

    while (true)
        ;
}
