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
#define BME280_HUMIDITY_CALIB_DATA_ADDR 0xE1
#define BME280_CTRL_HUM 0xF2
#define BME280_CTRL_MEAS 0xF4

#define BME280_CONCAT_BYTES(msb, lsb) (((uint16_t)(msb) << 8) | (uint16_t)(lsb))

static struct i2c_client *bme280_client;
static struct class *bme280_class;
static dev_t dev_num;
static struct cdev bme280_cdev;
static int32_t t_fine;

// Calibration data structure
struct bme280_calib_data {
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;
    uint8_t dig_H1;
    int16_t dig_H2;
    uint8_t dig_H3;
    int16_t dig_H4;
    int16_t dig_H5;
    int8_t dig_H6;
};

static struct bme280_calib_data calib_data;

// Function to read calibration data
static int bme280_read_calibration_data(void) {
    uint8_t calib[26], calib_hum[7];
    int ret;

    // Read temperature and pressure calibration data
    ret = i2c_smbus_read_i2c_block_data(bme280_client, BME280_CALIB_START, sizeof(calib), calib);
    if (ret < 0) {
        pr_err("Failed to read temperature/pressure calibration data\n");
        return ret;
    }

    calib_data.dig_T1 = BME280_CONCAT_BYTES(calib[1], calib[0]);
    calib_data.dig_T2 = (int16_t)BME280_CONCAT_BYTES(calib[3], calib[2]);
    calib_data.dig_T3 = (int16_t)BME280_CONCAT_BYTES(calib[5], calib[4]);
    calib_data.dig_P1 = BME280_CONCAT_BYTES(calib[7], calib[6]);
    calib_data.dig_P2 = (int16_t)BME280_CONCAT_BYTES(calib[9], calib[8]);
    calib_data.dig_P3 = (int16_t)BME280_CONCAT_BYTES(calib[11], calib[10]);
    calib_data.dig_P4 = (int16_t)BME280_CONCAT_BYTES(calib[13], calib[12]);
    calib_data.dig_P5 = (int16_t)BME280_CONCAT_BYTES(calib[15], calib[14]);
    calib_data.dig_P6 = (int16_t)BME280_CONCAT_BYTES(calib[17], calib[16]);
    calib_data.dig_P7 = (int16_t)BME280_CONCAT_BYTES(calib[19], calib[18]);
    calib_data.dig_P8 = (int16_t)BME280_CONCAT_BYTES(calib[21], calib[20]);
    calib_data.dig_P9 = (int16_t)BME280_CONCAT_BYTES(calib[23], calib[22]);
    calib_data.dig_H1 = calib[25];

    // Read humidity calibration data
    ret = i2c_smbus_read_i2c_block_data(bme280_client, BME280_HUMIDITY_CALIB_DATA_ADDR, sizeof(calib_hum), calib_hum);
    if (ret < 0) {
        pr_err("Failed to read humidity calibration data\n");
        return ret;
    }

    calib_data.dig_H2 = (int16_t)BME280_CONCAT_BYTES(calib_hum[1], calib_hum[0]);
    calib_data.dig_H3 = calib_hum[2];
    calib_data.dig_H4 = (int16_t)((calib_hum[3] << 4) | (calib_hum[4] & 0x0F));
    calib_data.dig_H5 = (int16_t)((calib_hum[5] << 4) | (calib_hum[4] >> 4));
    calib_data.dig_H6 = (int8_t)calib_hum[6];

    pr_info("Temperature calibration data: dig_T1=%d, dig_T2=%d, dig_T3=%d\n", calib_data.dig_T1, calib_data.dig_T2, calib_data.dig_T3);
    pr_info("Pressure calibration data: dig_P1=%d, dig_P2=%d, dig_P3=%d, dig_P4=%d, dig_P5=%d, dig_P6=%d, dig_P7=%d, dig_P8=%d, dig_P9=%d\n",
            calib_data.dig_P1, calib_data.dig_P2, calib_data.dig_P3, calib_data.dig_P4, calib_data.dig_P5, calib_data.dig_P6, calib_data.dig_P7, calib_data.dig_P8, calib_data.dig_P9);
    pr_info("Humidity calibration data: dig_H1=%d, dig_H2=%d, dig_H3=%d, dig_H4=%d, dig_H5=%d, dig_H6=%d\n",
            calib_data.dig_H1, calib_data.dig_H2, calib_data.dig_H3, calib_data.dig_H4, calib_data.dig_H5, calib_data.dig_H6);

    pr_info("Calibration data successfully loaded.\n");
    return 0;
}

// Function to read raw sensor data
static int bme280_read_raw_data(int reg_msb, int reg_lsb, int reg_xlsb) {
    int msb = i2c_smbus_read_byte_data(bme280_client, reg_msb);
    int lsb = i2c_smbus_read_byte_data(bme280_client, reg_lsb);
    int xlsb = (reg_xlsb != 0) ? i2c_smbus_read_byte_data(bme280_client, reg_xlsb) : 0;

    if (msb < 0 || lsb < 0 || (xlsb < 0 && reg_xlsb != 0))
        return -1;

    return (msb << 12) | (lsb << 4) | (xlsb >> 4);
}

// Temperature compensation
static int bme280_compensate_temperature(int adc_T) {
    int var1 = (((adc_T >> 3) - ((int)calib_data.dig_T1 << 1)) * (int)calib_data.dig_T2) >> 11;
    int var2 = (((((adc_T >> 4) - (int)calib_data.dig_T1) *
                  ((adc_T >> 4) - (int)calib_data.dig_T1)) >> 12) *
                (int)calib_data.dig_T3) >>
               14;

    t_fine = var1 + var2;
    int T = (t_fine * 5 + 128) >> 8;

    return T; // Return temperature in 0.01°C
}

// Pressure compensation
static int bme280_compensate_pressure(int adc_P) {
    int64_t var1 = ((int64_t)t_fine) - 128000;
    int64_t var2 = var1 * var1 * (int64_t)calib_data.dig_P6;
    var2 += (var1 * (int64_t)calib_data.dig_P5) << 17;
    var2 += ((int64_t)calib_data.dig_P4) << 35;
    var1 = ((var1 * var1 * (int64_t)calib_data.dig_P3) >> 8) + ((var1 * (int64_t)calib_data.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1) * (int64_t)calib_data.dig_P1) >> 33;

    if (var1 == 0) {
        return 0;  // Avoid division by zero
    }

    int64_t p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)calib_data.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)calib_data.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)calib_data.dig_P7) << 4);
    return (int)(p / 256);
}

// Humidity compensation
static uint32_t bme280_compensate_humidity(int32_t adc_H) {

    int32_t v_x1_u32r = t_fine - ((int32_t)76800);

    v_x1_u32r = (((((adc_H << 14) -
                    (((int32_t)calib_data.dig_H4) << 20) -
                    (((int32_t)calib_data.dig_H5) * v_x1_u32r)) +
                   ((int32_t)16384)) >>
                  15) *
                 (((((((v_x1_u32r * ((int32_t)calib_data.dig_H6)) >> 10) *
                      (((v_x1_u32r * ((int32_t)calib_data.dig_H3)) >> 11) +
                       ((int32_t)32768))) >>
                     10) +
                    ((int32_t)2097152)) *
                       ((int32_t)calib_data.dig_H2) +
                   ((int32_t)8192)) >>
                  14));

    v_x1_u32r = v_x1_u32r -
                (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
                  ((int32_t)calib_data.dig_H1)) >>
                 4);

    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);

    return (uint32_t)(v_x1_u32r >> 12);
}

// IOCTL handler
static long bme280_ioctl(struct file *file, unsigned int cmd, unsigned long arg) {
    int raw_data, value = 0;

    switch (cmd) {
    case IOCTL_GET_TEMPERATURE:
        raw_data = bme280_read_raw_data(BME280_REG_TEMP_MSB, BME280_REG_TEMP_LSB, BME280_REG_TEMP_XLSB);
        if (raw_data < 0)
            return -EFAULT;
        value = bme280_compensate_temperature(raw_data);
        break;
    case IOCTL_GET_HUMIDITY:
        raw_data = bme280_read_raw_data(BME280_REG_HUM_MSB, BME280_REG_HUM_LSB, 0);
        if (raw_data < 0)
            return -EFAULT;
        value = bme280_compensate_humidity(raw_data);
        printk("Humidity, raw value: %d\n", value);
        break;
    case IOCTL_GET_PRESSURE:
        raw_data = bme280_read_raw_data(BME280_REG_PRESS_MSB, BME280_REG_PRESS_LSB, BME280_REG_PRESS_XLSB);
        if (raw_data < 0)
            return -EFAULT;
        value = bme280_compensate_pressure(raw_data);
        break;
    default:
        return -EINVAL;
    }

    if (copy_to_user((int32_t __user *)arg, &value, sizeof(value)))
        return -EFAULT;

    return 0;
}

// File operations
static const struct file_operations fops = {
    .owner = THIS_MODULE,
    .unlocked_ioctl = bme280_ioctl,
};

// Probe function
static int bme280_probe(struct i2c_client *client) {
    bme280_client = client;

    if (bme280_read_calibration_data() < 0) {
        pr_err("Failed to read calibration data\n");
        return -EIO;
    }

    if (alloc_chrdev_region(&dev_num, 0, 1, DEVICE_NAME) < 0)
        return -1;

    cdev_init(&bme280_cdev, &fops);
    if (cdev_add(&bme280_cdev, dev_num, 1) == -1) {
        unregister_chrdev_region(dev_num, 1);
        return -1;
    }

    bme280_class = class_create(CLASS_NAME);
    if (IS_ERR(bme280_class)) {
        cdev_del(&bme280_cdev);
        unregister_chrdev_region(dev_num, 1);
        return PTR_ERR(bme280_class);
    }

    if (!device_create(bme280_class, NULL, dev_num, NULL, DEVICE_NAME)) {
        class_destroy(bme280_class);
        cdev_del(&bme280_cdev);
        unregister_chrdev_region(dev_num, 1);
        return -ENOMEM;
    }

    pr_info("BME280 driver initialized successfully\n");
    return 0;
}

// Remove function
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
    },
    .probe = bme280_probe,
    .remove = bme280_remove,
    .id_table = bme280_id,
};

module_i2c_driver(bme280_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Carlos Alvarado Martinez");
MODULE_DESCRIPTION("Enhanced BME280 driver with IOCTL support for temperature, humidity, and pressure");