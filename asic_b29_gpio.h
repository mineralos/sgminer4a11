#ifndef _ASIC_B29_GPIO_
#define _ASIC_B29_GPIO_

#include <stdlib.h>
#include <assert.h>
#include <fcntl.h>
#include <limits.h>
#include <unistd.h>
#include <stdbool.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/ioctl.h>


#define SYSFS_VID_DEV "/dev/vidgen0.0"

#define SYSFS_GPIO_EXPORT   "/sys/class/gpio/export"
#define SYSFS_GPIO_DIR_STR  "/sys/class/gpio/gpio%d/direction"
#define SYSFS_GPIO_VAL_STR  "/sys/class/gpio/gpio%d/value"

#define SYSFS_SPI_EXPORT "/sys/devices/soc0/amba/f8007000.devcfg/fclk_export"
#define SYSFS_SPI_VAL_STR "/sys/devices/soc0/amba/f8007000.devcfg/fclk/fclk1/set_rate"

#define SYSFS_GPIO_DIR_OUT  "out"
#define SYSFS_GPIO_DIR_IN   "in"

#define SYSFS_GPIO_VAL_LOW  "0"
#define SYSFS_GPIO_VAL_HIGH "1"

#define GPIO_DIR_INPUT      (1)
#define GPIO_DIR_OUTPUT     (0)


extern int SPI_PIN_POWER_EN[];
extern int SPI_PIN_START_EN[];
extern int SPI_PIN_RESET[];
extern int SPI_PIN_LED[];
extern int SPI_PIN_PLUG[];


void loop_blink_led(int pos, int cnt);


#endif
