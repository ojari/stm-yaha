#include "hts221.h"
#include "lps22hb.h"


void read_temp_hum(HTS221_device* device) {
    StatusReg status;

    hts221_set_CTRL_REG2(device, 0, 0, 1);  // one-shot

    hts221_read_STATUS_REG(device, &status);
    while (status.t_da == 0) {
        delay(1);
        hts221_read_STATUS_REG(device, &status);
    }
    float t = hts221_read_temperature(device);
    Serial_printf(t);
    Serial_print("C ");

    while (status.t_da == 0) {
        delay(1);
        hts221_read_STATUS_REG(device, &status);
    }
    float h = hts221_read_humidity(device);
    Serial_printf(h);
    Serial_print("%\r\n");
}

void read_pressure(LPS22HB_device* device) {
    int16_t temp = lps22hb_read_temperature(device);
    Serial_printf(temp);
    Serial_print("C ");

    int32_t pressure = lps22hb_read_pressure(device);
    Serial_printf(pressure / 100.0);
    Serial_print("hPa\r\n");
}

void task_main(HTS221_device *hts221, LPS22HB_device *lps22hb) {
    hts221_begin(hts221);

    hts221_set_CTRL_REG1(hts221, 1, 1, 0);
    delay(5);

    lps22hb_begin(lps22hb);

    while (1) {
        read_temp_hum(hts221);
        read_pressure(lps22hb);

        digitalWrite(LED2, HIGH);
        digitalWrite(LED1, LOW);
        delay(2000);

        digitalWrite(LED2, LOW);
        digitalWrite(LED1, HIGH);
        delay(2000);
    }
}
