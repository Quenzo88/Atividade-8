#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <stdlib.h>

LOG_MODULE_REGISTER(aq_acel, LOG_LEVEL_INF);

#define STACK_SIZE 1024
#define PRIORITY_AQ 4   // Maior prioridade para a aquisição
#define PRIORITY_COM 5  // Menor prioridade para a comunicação

// Definição do Acelerômetro
static const struct device *const accel = DEVICE_DT_GET(DT_NODELABEL(mma8451q));

// Estrutura do pacote de dados (Foco no Eixo X para o Filtro)
struct acel_data_item {
    uint32_t timestamp;
    int32_t raw_x;
    int32_t filtered_x;
    int32_t raw_y;
    int32_t raw_z;
};

// Fila de mensagens para comunicação entre threads
K_MSGQ_DEFINE(acel_msgq, sizeof(struct acel_data_item), 30, 4);

// Configuração do Filtro FIR (Média Móvel de 4 pontos)
volatile bool usar_filtro = false;
#define FIR_TAPS 4
int32_t fir_buffer_x[FIR_TAPS] = {0};

// Botão (PTA4) para alternar o filtro
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

// Filtro FIR básico para o Eixo X
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

// --- THREAD 1: AQUISIÇÃO (SENSOR FETCH) ---
void thread_aquisicao(void *arg1, void *arg2, void *arg3) {
    struct sensor_value val_x, val_y, val_z;
    struct acel_data_item dados;

    if (!device_is_ready(accel)) {
        LOG_ERR("Erro: Acelerômetro MMA8451Q não está pronto!");
        return;
    }

    LOG_INF("Thread de Aquisição do Acelerômetro Iniciada.");

    while (1) {
        // Realiza o fetch dos dados via barramento I2C
        int ret = sensor_sample_fetch(accel);
        if (ret == 0) {
            dados.timestamp = k_cycle_get_32(); // Timestamp de alta resolução por ciclos de clock

            sensor_channel_get(accel, SENSOR_CHAN_ACCEL_X, &val_x);
            sensor_channel_get(accel, SENSOR_CHAN_ACCEL_Y, &val_y);
            sensor_channel_get(accel, SENSOR_CHAN_ACCEL_Z, &val_z);

            // Converte m/s² (val1 + val2) para um inteiro em mili-m/s² para facilitar o FIR
            dados.raw_x = (val_x.val1 * 1000) + (val_x.val2 / 1000);
            dados.raw_y = (val_y.val1 * 1000) + (val_y.val2 / 1000);
            dados.raw_z = (val_z.val1 * 1000) + (val_z.val2 / 1000);

            // Aplica o filtro se estiver ativado
            if (usar_filtro) {
                dados.filtered_x = aplicar_filtro_fir_x(dados.raw_x);
            } else {
                dados.filtered_x = dados.raw_x;
            }

            // Envia para a fila de comunicação (Não bloqueia se estiver cheia)
            if (k_msgq_put(&acel_msgq, &dados, K_NO_WAIT) != 0) {
                // Se cair aqui, a fila encheu (gargalo de comunicação UART)
            }
        } else {
            LOG_WRN("Falha ao ler o acelerômetro: %d", ret);
        }

        // CONTROLADOR DA TAXA DE AQUISIÇÃO:
        // Modifique este tempo para testar os limites do sistema (Ex: K_MSEC(10), K_MSEC(2), K_NO_WAIT)
        k_msleep(10); 
    }
}

// --- THREAD 2: COMUNICAÇÃO (UART LOGGING) ---
void thread_comunicacao(void *arg1, void *arg2, void *arg3) {
    struct acel_data_item rx_dados;

    LOG_INF("Thread de Comunicação Iniciada.");

    while (1) {
        // Aguarda indefinidamente até que um dado surja na fila
        if (k_msgq_get(&acel_msgq, &rx_dados, K_FOREVER) == 0) {
            // Formato de saída limpo para o script Python capturar
            // Padrão: ACEL,timestamp,raw_x,filtered_x,raw_y,raw_z
            LOG_INF("ACEL,%u,%d,%d,%d,%d", 
                    rx_dados.timestamp, 
                    rx_dados.raw_x, 
                    rx_dados.filtered_x, 
                    rx_dados.raw_y, 
                    rx_dados.raw_z);
        }
    }
}

K_THREAD_DEFINE(acq_tid, STACK_SIZE, thread_aquisicao, NULL, NULL, NULL, PRIORITY_AQ, 0, 0);
K_THREAD_DEFINE(com_tid, STACK_SIZE, thread_comunicacao, NULL, NULL, NULL, PRIORITY_COM, 0, 0);

void main(void) {
    gpio_pin_configure(button.port, button.pin, GPIO_INPUT | GPIO_PULL_UP);
    gpio_pin_interrupt_configure(button.port, button.pin, GPIO_INT_EDGE_FALLING);
    gpio_init_callback(&button_cb_data, buttonIsr, BIT(button.pin));
    gpio_add_callback(button.port, &button_cb_data);

    LOG_INF("Sistema Pronto. Use o botão PTA4 para ligar/desligar o Filtro FIR.");
}