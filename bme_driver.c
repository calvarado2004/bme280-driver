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
#define BME280_HUMIDITY_CALIB_DATA_ADDR 0xE1
#define BME280_HUMIDITY_CALIB_DATA_LEN 7

// Calibration data length
#define BME280_TEMP_PRESS_CALIB_DATA_LEN 26

// Macro to combine bytes
#define BME280_CONCAT_BYTES(msb, lsb) (((uint16_t)(msb) << 8) | (uint16_t)(lsb))

static s32 t_fine = 22381;

// Structure to hold calibration data
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
    int64_t dig_H1;
    int64_t dig_H2;
    int64_t dig_H3;
    int64_t dig_H4;
    int64_t dig_H5;
    int64_t dig_H6;
};

static struct i2c_client *bme280_client;
static struct class *bme280_class;
static dev_t dev_num;
static struct cdev bme280_cdev;
static struct bme280_calib_data calib_data;
static int t_fine;

// Function to read calibration data
static int bme280_read_calibration_data(void) {
    uint8_t calib[BME280_TEMP_PRESS_CALIB_DATA_LEN];
    uint8_t calib_hum[BME280_HUMIDITY_CALIB_DATA_LEN];
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

    //print temperature, humidity and pressure calibration data, one line per data type
    pr_info("Temperature calibration data: " \
            "dig_T1=%d, dig_T2=%d, dig_T3=%d\n", \
            calib_data.dig_T1, calib_data.dig_T2, calib_data.dig_T3);

    pr_info("Humidity calibration data: " \
            "dig_H1=%d, dig_H2=%d, dig_H3=%d, dig_H4=%d, dig_H5=%d, dig_H6=%d\n", \
            calib_data.dig_H1, calib_data.dig_H2, calib_data.dig_H3, calib_data.dig_H4, calib_data.dig_H5, calib_data.dig_H6);

    pr_info("Pressure calibration data: " \
            "dig_P1=%d, dig_P2=%d, dig_P3=%d, dig_P4=%d, dig_P5=%d, dig_P6=%d, dig_P7=%d, dig_P8=%d, dig_P9=%d\n", \
            calib_data.dig_P1, calib_data.dig_P2, calib_data.dig_P3, calib_data.dig_P4, calib_data.dig_P5, calib_data.dig_P6, calib_data.dig_P7, calib_data.dig_P8, calib_data.dig_P9);

    pr_info("Calibration data read successfully\n");

    return 0;
}


static int bme280_read_raw_data(int reg_msb, int reg_lsb, int reg_xlsb) {
    int msb = i2c_smbus_read_byte_data(bme280_client, reg_msb);
    int lsb = i2c_smbus_read_byte_data(bme280_client, reg_lsb);
    int xlsb = i2c_smbus_read_byte_data(bme280_client, reg_xlsb);

    if (msb < 0 || lsb < 0 || xlsb < 0) return -1;

    return (msb << 12) | (lsb << 4) | (xlsb >> 4);
}

// Compensation functions
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

// Constants for calculations
const int64_t HUM_VAR1_OFFSET = 76800;
const int64_t HUM_CALIB_SCALE = 1048576;
const int64_t HUM_VAR3_SCALE = 4096;
const int64_t HUM_VAR4_SCALE = 8192;
const int64_t HUM_FINAL_SCALE = 100;

// Function to compensate humidity
static int64_t bme280_compensate_humidity(int64_t adc_H) {
    int64_t temp_diff = t_fine - HUM_VAR1_OFFSET;

    // Adjust humidity based on calibration data
    int64_t humidity_scaled = (adc_H * 16384)
                        - (calib_data.dig_H4 * HUM_CALIB_SCALE)
                        - ((calib_data.dig_H5 * temp_diff) / 1024);
    int64_t humidity_uncomp = (humidity_scaled + 16384) / 32768;

    // Apply further compensations
    int64_t var3 = (humidity_uncomp * humidity_uncomp * calib_data.dig_H1) / HUM_VAR3_SCALE;
    int64_t var4 = (humidity_uncomp * calib_data.dig_H2) / HUM_VAR4_SCALE;
    int64_t compensation_result = (((var4 + 2097152) * calib_data.dig_H3) / 16384) + var3;

    // Return humidity as an integer percentage with two decimal places
    return compensation_result / HUM_FINAL_SCALE;
}


static long bme280_ioctl(struct file *file, unsigned int cmd, unsigned long arg) {
    int value;
    int raw_data;
    s64 int_humidity;

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
            int_humidity = bme280_compensate_humidity(raw_data);
            if (copy_to_user((int __user *)arg, &int_humidity, sizeof(int))) {
                pr_err("Failed to copy data to user space\n");
                return -EFAULT;
            }
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
        return -1;
    }
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
    device_create(bme280_class, NULL, dev_num, NULL, DEVICE_NAME);

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

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Carlos Alvarado Martinez");
MODULE_DESCRIPTION("Enhanced BME280 driver with ioctl support for temperature, humidity, and pressure");