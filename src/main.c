#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <stdlib.h>

LOG_MODULE_REGISTER(aq_acel, LOG_LEVEL_INF);

#define STACK_SIZE 1024
#define PRIORITY_AQ 4   
#define PRIORITY_COM 5  

// === Endereço e registradores do MMA8451Q ===
#define MMA8451Q_I2C_ADDR    0x1D
#define MMA8451Q_CTRL_REG1   0x2A
#define MMA8451Q_ACTIVE_BIT  0x01
#define MMA8451Q_ODR         (0x0 << 3)  // 100 Hz conforme datasheet (DR=100b)

// Dispositivos
static const struct device *const accel = DEVICE_DT_GET(DT_NODELABEL(mma8451q));
static const struct device *const i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));

// Estrutura de dados agora com X, Y e Z filtrados
struct acel_data_item {
    uint32_t timestamp;
    int32_t raw_x;
    int32_t raw_y;
    int32_t raw_z;
    int32_t filtered_x;
    int32_t filtered_y;
    int32_t filtered_z;
};
K_MSGQ_DEFINE(acel_msgq, sizeof(struct acel_data_item), 30, 4);

// Configuração do Filtro FIR (Média Móvel de 6 pontos para bater com o Python)
volatile bool usar_filtro = false;
#define FIR_TAPS 6
int32_t fir_buffer_x[FIR_TAPS] = {0};
int32_t fir_buffer_y[FIR_TAPS] = {0};
int32_t fir_buffer_z[FIR_TAPS] = {0};

// Botão (PTA4)
static const struct gpio_dt_spec button = {
    .port = DEVICE_DT_GET(DT_NODELABEL(gpioa)),
    .pin  = 4,
    .dt_flags = 0,
};
static struct gpio_callback button_cb_data;

void buttonIsr(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    usar_filtro = !usar_filtro;
    printk("Filtros FIR (XYZ): %s\n", usar_filtro ? "ATIVADOS" : "DESATIVADOS");
}

// Função genérica de Filtro FIR que aceita qualquer eixo
int32_t aplicar_filtro_fir(int32_t nova_amostra, int32_t *buffer) {
    int64_t soma = 0;
    for (int i = FIR_TAPS - 1; i > 0; i--) {
        buffer[i] = buffer[i - 1];
        soma += buffer[i];
    }
    buffer[0] = nova_amostra;
    soma += nova_amostra;
    return (int32_t)(soma / FIR_TAPS);
}

// === Configuração I2C ===
void mma8451q_configurar_odr(void) {
    uint8_t buf[2];
    int ret;

    buf[0] = MMA8451Q_CTRL_REG1;
    buf[1] = 0x00;
    i2c_write(i2c_dev, buf, 2, MMA8451Q_I2C_ADDR);

    buf[0] = MMA8451Q_CTRL_REG1;
    buf[1] = MMA8451Q_ODR;
    i2c_write(i2c_dev, buf, 2, MMA8451Q_I2C_ADDR);

    buf[0] = MMA8451Q_CTRL_REG1;
    buf[1] = MMA8451Q_ODR | MMA8451Q_ACTIVE_BIT;
    i2c_write(i2c_dev, buf, 2, MMA8451Q_I2C_ADDR);

    LOG_INF("MMA8451Q configurado para 100 Hz via I2C.");
}

// --- THREAD 1: AQUISIÇÃO ---
void thread_aquisicao(void *arg1, void *arg2, void *arg3) {
    struct sensor_value val_x, val_y, val_z;
    struct acel_data_item dados;

    if (!device_is_ready(accel) || !device_is_ready(i2c_dev)) {
        LOG_ERR("Erro: Dispositivos não prontos!");
        return;
    }

    mma8451q_configurar_odr();
    k_msleep(200); 

    while (1) {
        int ret = sensor_sample_fetch(accel);
        if (ret == 0) {
            dados.timestamp = k_cycle_get_32(); 

            sensor_channel_get(accel, SENSOR_CHAN_ACCEL_X, &val_x);
            sensor_channel_get(accel, SENSOR_CHAN_ACCEL_Y, &val_y);
            sensor_channel_get(accel, SENSOR_CHAN_ACCEL_Z, &val_z);

            dados.raw_x = (val_x.val1 * 1000) + (val_x.val2 / 1000);
            dados.raw_y = (val_y.val1 * 1000) + (val_y.val2 / 1000);
            dados.raw_z = (val_z.val1 * 1000) + (val_z.val2 / 1000);

            if (usar_filtro) {
                // Aplica o filtro nos 3 buffers separados
                dados.filtered_x = aplicar_filtro_fir(dados.raw_x, fir_buffer_x);
                dados.filtered_y = aplicar_filtro_fir(dados.raw_y, fir_buffer_y);
                dados.filtered_z = aplicar_filtro_fir(dados.raw_z, fir_buffer_z);
            } else {
                dados.filtered_x = dados.raw_x;
                dados.filtered_y = dados.raw_y;
                dados.filtered_z = dados.raw_z;
            }

            k_msgq_put(&acel_msgq, &dados, K_NO_WAIT);
        }
        k_msleep(5); 
    }
}

// --- THREAD 2: COMUNICAÇÃO ---
void thread_comunicacao(void *arg1, void *arg2, void *arg3) {
    struct acel_data_item rx;

    while (1) {
        if (k_msgq_get(&acel_msgq, &rx, K_FOREVER) == 0) {
            // Imprime exatamente no formato esperado pelo Regex do Python
            LOG_INF("XR:%d YR:%d ZR:%d XF:%d YF:%d ZF:%d", 
                    rx.raw_x, rx.raw_y, rx.raw_z, 
                    rx.filtered_x, rx.filtered_y, rx.filtered_z);
        }
    }
}

K_THREAD_DEFINE(acq_tid, STACK_SIZE, thread_aquisicao, NULL, NULL, NULL, PRIORITY_AQ, 0, 0);
K_THREAD_DEFINE(com_tid, STACK_SIZE, thread_comunicacao, NULL, NULL, NULL, PRIORITY_COM, 0, 0);

int main(void) {
    gpio_pin_configure(button.port, button.pin, GPIO_INPUT | GPIO_PULL_UP);
    gpio_pin_interrupt_configure(button.port, button.pin, GPIO_INT_EDGE_FALLING);
    gpio_init_callback(&button_cb_data, buttonIsr, BIT(button.pin));
    gpio_add_callback(button.port, &button_cb_data);

    LOG_INF("Sistema Pronto.");
    return 0; 
}