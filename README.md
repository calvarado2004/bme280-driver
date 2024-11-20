# BME280 Driver README

## Overview

This project provides a kernel space driver for the BME280 sensor, which measures temperature, humidity, and pressure. The driver communicates with the BME280 sensor over the I2C bus and uses the `ioctl` interface to allow user-space applications to access sensor readings.

## Features

- **Kernel Space Driver**: Runs in kernel mode and interfaces with the hardware directly via I2C headers.
- **IOCTL Interface**: User-space applications can use `ioctl` commands to read temperature, humidity, and pressure.
- **I2C Communication**: The driver uses I2C headers for communication with the BME280 sensor.
- **Device Node**: A character device node (`/dev/bme280`) is created for user-space interaction.
- **Calibration Support**: The driver reads and applies sensor calibration data for accurate measurements.

## IOCTL Commands

The driver defines the following `ioctl` commands:

- `IOCTL_GET_TEMPERATURE`: Read the temperature in degrees Celsius (integer representation).
- `IOCTL_GET_HUMIDITY`: Read the humidity in percentage (integer representation).
- `IOCTL_GET_PRESSURE`: Read the pressure in Pascals (integer representation).

### IOCTL Command Definitions

```c
#define IOCTL_GET_TEMPERATURE _IOR('B', 1, int)
#define IOCTL_GET_HUMIDITY _IOR('B', 2, int)
#define IOCTL_GET_PRESSURE _IOR('B', 3, int)
```

## Usage

### Building the Driver

1. **Ensure your kernel headers are installed**:
   ```
   sudo apt-get install linux-headers-$(uname -r)
   ```

2. **Compile the driver**:
   ```
   make
   ```

3. **Insert the driver**:
   ```
   sudo insmod bme280.ko
   ```

4. **Create a device node**:
   ```
   sudo mknod /dev/bme280 c <major_number> 0
   ```

### User-Space Interaction

To interact with the driver, use a simple C program or a utility like `ioctl` in combination with `open()` and `close()` system calls.

Example code snippet for reading temperature:
```c
#include <stdio.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#define IOCTL_GET_TEMPERATURE _IOR('B', 1, int)

int main() {
    int fd = open("/dev/bme280", O_RDONLY);
    if (fd < 0) {
        perror("Failed to open /dev/bme280");
        return -1;
    }

    int temperature;
    if (ioctl(fd, IOCTL_GET_TEMPERATURE, &temperature) == -1) {
        perror("Failed to read temperature");
        close(fd);
        return -1;
    }

    printf("Temperature: %d.%03d°C\n", temperature / 1000, temperature % 1000);
    close(fd);
    return 0;
}
```

## Driver Details

### Calibration Data

The BME280 sensor requires calibration data stored in its registers to provide accurate readings. The driver reads this data during initialization and uses it for compensation algorithms in temperature, humidity, and pressure readings.

### Compensation Functions

The driver includes functions to read and compensate:
- **Temperature (`bme280_compensate_temperature`)**
- **Humidity (`bme280_compensate_humidity`)**
- **Pressure (`bme280_compensate_pressure`)**

### I2C Communication

The driver uses `i2c_smbus_read_byte_data` and `i2c_smbus_read_i2c_block_data` functions to read data from the BME280 sensor.

## Troubleshooting

- **Device Not Created**: Ensure the character device node is created using `mknod` with the correct major number.
- **Permissions**: Make sure the `/dev/bme280` has the necessary permissions.
- **Kernel Logs**: Check `dmesg` for relevant driver logs:
  ```
  dmesg | grep bme280
  ```

## License

This driver is licensed under GPL v2. 

## Author

Developed by Carlos Alvarado Martinez.
