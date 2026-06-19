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

// Fila de mensagens e Estrutura de dados
struct acel_data_item {
    uint32_t timestamp;
    int32_t raw_x;
    int32_t filtered_x;
};
K_MSGQ_DEFINE(acel_msgq, sizeof(struct acel_data_item), 30, 4);

// Configuração do Filtro FIR (Média Móvel de 4 pontos)
volatile bool usar_filtro = false;
#define FIR_TAPS 4
int32_t fir_buffer_x[FIR_TAPS] = {0};

// Botão (PTA4)
static const struct gpio_dt_spec button = {
    .port = DEVICE_DT_GET(DT_NODELABEL(gpioa)),
    .pin  = 4,
    .dt_flags = 0,
};
static struct gpio_callback button_cb_data;

void buttonIsr(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    usar_filtro = !usar_filtro;
    printk("Filtro FIR no Eixo X: %s\n", usar_filtro ? "ATIVADO" : "DESATIVADO");
}

int32_t aplicar_filtro_fir_x(int32_t nova_amostra) {
    int64_t soma = 0;
    for (int i = FIR_TAPS - 1; i > 0; i--) {
        fir_buffer_x[i] = fir_buffer_x[i - 1];
        soma += fir_buffer_x[i];
    }
    fir_buffer_x[0] = nova_amostra;
    soma += nova_amostra;
    return (int32_t)(soma / FIR_TAPS);
}

// === Função de Configuração I2C que você criou ===
void mma8451q_configurar_odr(void) {
    uint8_t buf[2];
    int ret;

    buf[0] = MMA8451Q_CTRL_REG1;
    buf[1] = 0x00;
    ret = i2c_write(i2c_dev, buf, 2, MMA8451Q_I2C_ADDR);
    if (ret) { LOG_ERR("ERRO standby: %d", ret); return; }

    buf[0] = MMA8451Q_CTRL_REG1;
    buf[1] = MMA8451Q_ODR;
    ret = i2c_write(i2c_dev, buf, 2, MMA8451Q_I2C_ADDR);
    if (ret) { LOG_ERR("ERRO config ODR: %d", ret); return; }

    buf[0] = MMA8451Q_CTRL_REG1;
    buf[1] = MMA8451Q_ODR | MMA8451Q_ACTIVE_BIT;
    ret = i2c_write(i2c_dev, buf, 2, MMA8451Q_I2C_ADDR);
    if (ret) { LOG_ERR("ERRO ativar: %d", ret); return; }

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
    k_msleep(200); // Estabilizar

    while (1) {
        int ret = sensor_sample_fetch(accel);
        if (ret == 0) {
            dados.timestamp = k_cycle_get_32(); 

            sensor_channel_get(accel, SENSOR_CHAN_ACCEL_X, &val_x);
            // Convertendo para inteiro para o filtro FIR
            dados.raw_x = (val_x.val1 * 1000) + (val_x.val2 / 1000);

            if (usar_filtro) {
                dados.filtered_x = aplicar_filtro_fir_x(dados.raw_x);
            } else {
                dados.filtered_x = dados.raw_x;
            }

            // Envia para a fila
            k_msgq_put(&acel_msgq, &dados, K_NO_WAIT);
        }
        k_msleep(5); // Mantendo o seu delay de aquisição de dados
    }
}

// --- THREAD 2: COMUNICAÇÃO ---
void thread_comunicacao(void *arg1, void *arg2, void *arg3) {
    struct acel_data_item rx_dados;

    while (1) {
        if (k_msgq_get(&acel_msgq, &rx_dados, K_FOREVER) == 0) {
            // Formato que o Python entende (ACEL,timestamp,raw_x,filtered_x,0,0)
            LOG_INF("ACEL,%u,%d,%d,0,0", rx_dados.timestamp, rx_dados.raw_x, rx_dados.filtered_x);
        }
    }
}

K_THREAD_DEFINE(acq_tid, STACK_SIZE, thread_aquisicao, NULL, NULL, NULL, PRIORITY_AQ, 0, 0);
K_THREAD_DEFINE(com_tid, STACK_SIZE, thread_comunicacao, NULL, NULL, NULL, PRIORITY_COM, 0, 0);

// === MAIN CORRIGIDO (Retornando int) ===
int main(void) {
    gpio_pin_configure(button.port, button.pin, GPIO_INPUT | GPIO_PULL_UP);
    gpio_pin_interrupt_configure(button.port, button.pin, GPIO_INT_EDGE_FALLING);
    gpio_init_callback(&button_cb_data, buttonIsr, BIT(button.pin));
    gpio_add_callback(button.port, &button_cb_data);

    LOG_INF("Sistema Pronto.");
    
    return 0; // Exigência do compilador C moderno
}