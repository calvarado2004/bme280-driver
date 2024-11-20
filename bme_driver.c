#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <linux/device.h>

#define DEVICE_NAME "bme280"
#define CLASS_NAME "bme280_class"
#define IOCTL_GET_TEMPERATURE _IOR('B', 1, int)
#define IOCTL_GET_HUMIDITY _IOR('B', 2, int)
#define IOCTL_GET_PRESSURE _IOR('B', 3, int)

// Register addresses for BME280
#define BME280_REG_TEMP_MSB 0xFA
#define BME280_REG_TEMP_LSB 0xFB
#define BME280_REG_TEMP_XLSB 0xFC
#define BME280_REG_PRESS_MSB 0xF7
#define BME280_REG_PRESS_LSB 0xF8
#define BME280_REG_PRESS_XLSB 0xF9
#define BME280_REG_HUM_MSB 0xFD
#define BME280_REG_HUM_LSB 0xFE
#define BME280_CALIB_START 0x88
#define BME280_CALIB_END 0xA1
#define BME280_CALIB_HUM_START 0xE1
#define BME280_CALIB_HUM_END 0xE7

struct bme280_calib_data {
    unsigned short dig_T1;
    signed short dig_T2;
    signed short dig_T3;
    unsigned short dig_P1;
    signed short dig_P2;
    signed short dig_P3;
    signed short dig_P4;
    signed short dig_P5;
    signed short dig_P6;
    signed short dig_P7;
    signed short dig_P8;
    signed short dig_P9;
    unsigned char dig_H1;
    signed short dig_H2;
    unsigned char dig_H3;
    signed short dig_H4;
    signed short dig_H5;
    signed char dig_H6;
};

static struct i2c_client *bme280_client;
static struct class *bme280_class;
static dev_t dev_num;
static struct cdev bme280_cdev;
static struct bme280_calib_data calib_data;
static int t_fine;

static int bme280_read_calibration_data(void) {
    int ret;
    unsigned char calib[BME280_CALIB_END - BME280_CALIB_START + 1];
    unsigned char calib_hum[BME280_CALIB_HUM_END - BME280_CALIB_HUM_START + 1];

    ret = i2c_smbus_read_i2c_block_data(bme280_client, BME280_CALIB_START, sizeof(calib), calib);
    if (ret < 0) return ret;

    calib_data.dig_T1 = (calib[1] << 8) | calib[0];
    calib_data.dig_T2 = (calib[3] << 8) | calib[2];
    calib_data.dig_T3 = (calib[5] << 8) | calib[4];
    calib_data.dig_P1 = (calib[7] << 8) | calib[6];
    calib_data.dig_P2 = (calib[9] << 8) | calib[8];
    calib_data.dig_P3 = (calib[11] << 8) | calib[10];
    calib_data.dig_P4 = (calib[13] << 8) | calib[12];
    calib_data.dig_P5 = (calib[15] << 8) | calib[14];
    calib_data.dig_P6 = (calib[17] << 8) | calib[16];
    calib_data.dig_P7 = (calib[19] << 8) | calib[18];
    calib_data.dig_P8 = (calib[21] << 8) | calib[20];
    calib_data.dig_P9 = (calib[23] << 8) | calib[22];
    calib_data.dig_H1 = calib[25];

    ret = i2c_smbus_read_i2c_block_data(bme280_client, BME280_CALIB_HUM_START, sizeof(calib_hum), calib_hum);
    if (ret < 0) return ret;

    calib_data.dig_H2 = (calib_hum[1] << 8) | calib_hum[0];
    calib_data.dig_H3 = calib_hum[2];
    calib_data.dig_H4 = (calib_hum[3] << 4) | (calib_hum[4] & 0x0F);
    calib_data.dig_H5 = (calib_hum[5] << 4) | (calib_hum[4] >> 4);
    calib_data.dig_H6 = calib_hum[6];

    return 0;
}

static int bme280_read_raw_data(int reg_msb, int reg_lsb, int reg_xlsb) {
    int msb = i2c_smbus_read_byte_data(bme280_client, reg_msb);
    int lsb = i2c_smbus_read_byte_data(bme280_client, reg_lsb);
    int xlsb = i2c_smbus_read_byte_data(bme280_client, reg_xlsb);

    if (msb < 0 || lsb < 0 || xlsb < 0) return -1;

    return (msb << 12) | (lsb << 4) | (xlsb >> 4);
}

static int bme280_compensate_temperature(int adc_T) {
    int var1, var2, T;
    var1 = ((((adc_T >> 3) - ((int)calib_data.dig_T1 << 1))) * ((int)calib_data.dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int)calib_data.dig_T1)) * ((adc_T >> 4) - ((int)calib_data.dig_T1))) >> 12) *
            ((int)calib_data.dig_T3)) >>
           14;

    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T;
}

static int bme280_compensate_pressure(int adc_P) {
    long long var1, var2, p;
    var1 = ((long long)t_fine) - 128000;
    var2 = var1 * var1 * (long long)calib_data.dig_P6;
    var2 = var2 + ((var1 * (long long)calib_data.dig_P5) << 17);
    var2 = var2 + (((long long)calib_data.dig_P4) << 35);
    var1 = ((var1 * var1 * (long long)calib_data.dig_P3) >> 8) + ((var1 * (long long)calib_data.dig_P2) << 12);
    var1 = (((((long long)1) << 47) + var1)) * ((long long)calib_data.dig_P1) >> 33;

    if (var1 == 0) {
        return 0;  // Avoid division by zero
    }

    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((long long)calib_data.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((long long)calib_data.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((long long)calib_data.dig_P7) << 4);
    return (int)p / 256;
}

static int bme280_compensate_humidity(int adc_H) {
    int v_x1_u32r;

    // Temperature fine resolution adjustment
    v_x1_u32r = t_fine - ((int)76800);

    // Intermediate calculations for humidity compensation
    v_x1_u32r = (((((adc_H << 14) - (((int)calib_data.dig_H4) << 20) -
                    (((int)calib_data.dig_H5) * v_x1_u32r)) +
                   ((int)16384)) >>
                  15) *
                 (((((((v_x1_u32r * ((int)calib_data.dig_H6)) >> 10) *
                      (((v_x1_u32r * ((int)calib_data.dig_H3)) >> 11) +
                       ((int)32768))) >>
                     10) +
                    ((int)2097152)) *
                   ((int)calib_data.dig_H2) +
                   8192) >>
                  14));

    // Final adjustment for humidity
    v_x1_u32r = v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
                              ((int)calib_data.dig_H1)) >>
                             4);

    // Constrain to valid range
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);

    // Return percentage (10.24 scale)
    return (v_x1_u32r >> 12) / 1024;
}

static long bme280_ioctl(struct file *file, unsigned int cmd, unsigned long arg) {
    int value;
    int raw_data;

    switch (cmd) {
        case IOCTL_GET_TEMPERATURE:
            raw_data = bme280_read_raw_data(BME280_REG_TEMP_MSB, BME280_REG_TEMP_LSB, BME280_REG_TEMP_XLSB);
            if (raw_data < 0) return -EFAULT;
            value = bme280_compensate_temperature(raw_data);
            break;
        case IOCTL_GET_HUMIDITY:
            raw_data = i2c_smbus_read_word_data(bme280_client, BME280_REG_HUM_MSB);
            if (raw_data < 0) {
                pr_err("Failed to read raw humidity data\n");
                return -EFAULT;
            }
            //pr_info("Raw humidity data: %d\n", raw_data);  // Debug print
            value = bme280_compensate_humidity(raw_data);
            break;
        case IOCTL_GET_PRESSURE:
            raw_data = bme280_read_raw_data(BME280_REG_PRESS_MSB, BME280_REG_PRESS_LSB, BME280_REG_PRESS_XLSB);
            if (raw_data < 0) return -EFAULT;
            value = bme280_compensate_pressure(raw_data);
            break;
        default:
            return -EINVAL;
    }

    if (copy_to_user((int __user *)arg, &value, sizeof(int))) {
        return -EFAULT;
    }
    return 0;
}

static struct file_operations fops = {
    .owner = THIS_MODULE,
    .unlocked_ioctl = bme280_ioctl,
};

static int bme280_probe(struct i2c_client *client) {
    bme280_client = client;
    if (bme280_read_calibration_data() < 0) {
        pr_err("Failed to read calibration data\n");
        return -EIO;
    }

    if (alloc_chrdev_region(&dev_num, 0, 1, DEVICE_NAME) < 0) {
        pr_err("Failed to allocate character device region\n");
        return -1;
    }
    cdev_init(&bme280_cdev, &fops);
    if (cdev_add(&bme280_cdev, dev_num, 1) == -1) {
        unregister_chrdev_region(dev_num, 1);
        pr_err("Failed to add cdev\n");
        return -1;
    }

    bme280_class = class_create(CLASS_NAME);
    if (IS_ERR(bme280_class)) {
        cdev_del(&bme280_cdev);
        unregister_chrdev_region(dev_num, 1);
        pr_err("Failed to create class\n");
        return PTR_ERR(bme280_class);
    }

    if (IS_ERR(device_create(bme280_class, NULL, dev_num, NULL, DEVICE_NAME))) {
        class_destroy(bme280_class);
        cdev_del(&bme280_cdev);
        unregister_chrdev_region(dev_num, 1);
        pr_err("Failed to create device\n");
        return PTR_ERR(bme280_class);
    }

    pr_info("BME280 driver initialized\n");
    return 0;
}

static void bme280_remove(struct i2c_client *client) {
    device_destroy(bme280_class, dev_num);
    class_destroy(bme280_class);
    cdev_del(&bme280_cdev);
    unregister_chrdev_region(dev_num, 1);
    pr_info("BME280 driver removed\n");
}

static const struct i2c_device_id bme280_id[] = {
    { "bme280", 0 },
    { }
};

MODULE_DEVICE_TABLE(i2c, bme280_id);

static struct i2c_driver bme280_driver = {
    .driver = {
        .name = "bme280_driver",
        .owner = THIS_MODULE,
    },
    .probe = bme280_probe,
    .remove = bme280_remove,
    .id_table = bme280_id,
};

module_i2c_driver(bme280_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Carlos Alvarado Martinez");
MODULE_DESCRIPTION("Enhanced BME280 driver with ioctl support for temperature, humidity, and pressure");
