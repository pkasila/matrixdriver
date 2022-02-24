/**
 * Example Device-Tree entry:
 * ----------------------------
 *    shift_register {
 *       compatible = "texasinstruments,matrixdriver";
 *       status = "okay";
 *       ser-in-gpios = <&gpio6 18 GPIO_ACTIVE_HIGH>;
 *       srck-gpios = <&gpio7 1 GPIO_ACTIVE_HIGH>;
 *       rck-gpios = <&gpio7 7 GPIO_ACTIVE_HIGH>;
 *    };
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/of.h>
#include <linux/gpio/consumer.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/kthread.h>
#include <linux/uaccess.h>
#include "matrixdriver.h"

#define matrixdriver_DEV_MAJOR 191     /* major-number of this device */
#define matrixdriver_DEV_MINORS 32     /* allow up to 32 devices */
#define matrixdriver_BITBANG_UDELAY 10
#define matrixdriver_BUFFER_SIZE 8     /* size of the write-buffer per device */

static struct class *matrixdriver_class;  /* drivers class, initialized in the __init routine */
static LIST_HEAD(device_list);         /* list of all devices managed by this driver */
static DEFINE_MUTEX(device_list_lock); /* synchronizes access to device_list */
static DECLARE_BITMAP(minors, matrixdriver_DEV_MINORS); /* use bitmap to manage minor-numbers of the device */

/* structure to represent a matrixdriver device */
struct matrixdriver_dev {
    struct list_head device_entry;  /* entry of this device in the device-list */

    struct device    *dev;
    struct cdev      cdev;
    dev_t            dev_t;

    struct mutex     mutex;         /* used to synchronize write-access */
    int              users;         /* count of users, e.g. open file-handles */
    u8               *buf;          /* buffer used for write-operations */
    u8               *curval;       /* current value of the device, e.g. last written value */
    long             curval_ptr;    /* index of the start of curval */
    long             dev_size;      /* size of the device in bytes */
    long             bitbang_delay; /* time to wait between gpio-state changes */

    struct gpio_desc *gpio_srck;
    struct gpio_desc *gpio_rck;
    struct gpio_desc *gpio_ser_in;

    struct task_struct *thread;
};

/*
 * forward-declarations
 */
static int matrixdriver_open(struct inode *node, struct file *fd);
static ssize_t matrixdriver_write(struct file* fd, const char __user *buf, size_t len, loff_t *off);
static long matrixdriver_ioctl(struct file *fd, unsigned int req, unsigned long arg);

static int row_lookup_table[8] = {3, 2, 1, 0, 4, 5, 6, 7};

/**
 * Writes the devices current buffer to the device. This function
 * does the actual gpio-bitbanging to communicate with the hardware.
 *
 * @param dev the dev to write its buffer to
 * @return count of bytes written
 */
static ssize_t write_buffer(struct matrixdriver_dev *dev, size_t len) {
    int i, j, val;

    /* write byte for byte in the buffer */
    for (i = len - 1; i >= 0; i--) {

        u8 row = 1 << i;

        /* write every bit in the byte for columns, lsb */
        for (j = 7; j >= 0; j--) {

            /* set serial-data pin */
            val = ((unsigned char)row & (1u << j)) > 0;
            gpiod_set_value(dev->gpio_ser_in, val);

            /* set SRCK */
            udelay(dev->bitbang_delay);
            gpiod_set_value(dev->gpio_srck, 1);
            udelay(dev->bitbang_delay);
            gpiod_set_value(dev->gpio_srck, 0);
        }

        /* write every bit in the byte, lsb */
        for (j = 0; j < 8; j++) {

            /* set serial-data pin */
            val = ((unsigned char)dev->buf[i] & (1u << row_lookup_table[j])) > 0;
            gpiod_set_value(dev->gpio_ser_in, !val);

            /* set SRCK */
            udelay(dev->bitbang_delay);
            gpiod_set_value(dev->gpio_srck, 1);
            udelay(dev->bitbang_delay);
            gpiod_set_value(dev->gpio_srck, 0);
        }

        /* set & unset RCK */
        gpiod_set_value(dev->gpio_rck, 1);
        udelay(dev->bitbang_delay);
        gpiod_set_value(dev->gpio_rck, 0);
    }

    return len;
}

int device_thread(void *dev_void) {
    struct matrixdriver_dev *dev = (struct matrixdriver_dev *) dev_void;
    dev_info(dev->dev, "thread started\n");

    while(!kthread_should_stop()) {
        mutex_lock(&dev->mutex);
        write_buffer(dev, 8);
        mutex_unlock(&dev->mutex);

        usleep_range(dev->bitbang_delay, dev->bitbang_delay);
    }

    return 0;
}

/**
 * Opens the device for use from userspace with the specified
 * file handle.
 *
 * @param node the node that contains the maj/min
 * @param fd   the file to add device-information to
 * @return 0 for success,
 *         -ENXIO on failure
 */
static int matrixdriver_open(struct inode *node, struct file *fd) {
    struct matrixdriver_dev *dev;
    int err = -ENXIO;

    mutex_lock(&device_list_lock);

    /* find device-entry for the device that is being opened */
    list_for_each_entry(dev, &device_list, device_entry) {
        if (dev->dev_t == node->i_rdev) {
            err = 0;
            break;
        }
    }

    /* associate file with device and increment users */
    if (err == 0) {
        fd->private_data = dev;
        dev->users++;
    }
    else
        pr_err("failed to open unassigned minor\n");

    mutex_unlock(&device_list_lock);

    return err;
}

/**
 * Closes the device to be used from userspace with
 * the specified file-handle.
 *
 * @param node the node that contains the maj/min
 * @param fd   the file to close
 * @return 0 on success,
 *         -ENXIO on failure
 */
static int matrixdriver_release(struct inode *node, struct file *fd) {
    struct matrixdriver_dev *dev = fd->private_data;

    if (!dev)
        return -ENXIO;

    /* file was closed, decrement users */
    mutex_lock(&device_list_lock);
    fd->private_data = NULL;
    dev->users--;
    mutex_unlock(&device_list_lock);

    return 0;
}

/**
 * Writes to the device.
 *
 * @param fd  the file-handle to write to
 * @param buf the buffer to write
 * @param len the length of the buffer
 * @param off the offset within the buffer, not supported
 * @return count of bytes written on success,
 *         -ENXIO or -EBADE on failure
 */
static ssize_t matrixdriver_write(struct file* fd, const char __user *buf, size_t len, loff_t *off) {
    struct matrixdriver_dev *dev = fd->private_data;
    size_t total, cur, tx = 0;

    if (!dev)
        return -ENXIO;

    for (total = 0; total < len; total += matrixdriver_BUFFER_SIZE) {

        /* copy buffer_size at most per transfer */
        cur = len - total > matrixdriver_BUFFER_SIZE ? matrixdriver_BUFFER_SIZE : len - total;
        if (copy_from_user(dev->buf, &buf[total], cur) != 0) {
            dev_err(dev->dev, "failed to copy from user\n");
            return -EBADE;
        }

        if(dev->thread == NULL) {
            dev->thread = kthread_create(device_thread, (void*)dev, "kthread_dev");
            if(dev->thread != NULL){
                /* Let's start the thread */
                wake_up_process(dev->thread);
                dev_info(dev->dev, "Thread was created and is running now!\n");
            } else {
                dev_err(dev->dev, "Thread could not be created!\n");
                return -ENXIO;
            }
        }

        /* save to current value */
        dev->curval = dev->buf;

        /* write the current buffer */
        tx += len;
    }

    return tx;
}

/**
 * Reads the current value of the device. This doesn't
 * perform any actual hardware-read. It returns the bytes
 * that were last written to the device.
 *
 * @param fd  the file-handle to read from
 * @param buf the buffer to read into
 * @return count of bytes read on success,
 *         -ENXIO or -EBADE on failure
 */
static ssize_t matrixdriver_read(struct file *fd, char __user *buf, size_t len, loff_t *f_pos) {
    ssize_t res;
    struct matrixdriver_dev *dev = fd->private_data;

    if (!dev)
        return -ENXIO;

    // TODO: fix

    /* copy up to dev->dev_size bytes */
    mutex_lock(&dev->mutex);
    res = len < dev->dev_size ? len : dev->dev_size;
    if (copy_to_user(buf, dev->curval, res) != 0) {
        dev_err(dev->dev, "failed to copy to user\n");
        res = -EBADE;
    }
    mutex_unlock(&dev->mutex);

    return res;
}

/**
 * Sets the size of the device. This will also allocate
 * the buffers of the device if needed.
 *
 * @param dev  the device to set the size for
 * @param size the size to set
 * @return 0 on success,
 *         -ENOMEM on failure
 */
static int matrixdriver_set_devsize(struct matrixdriver_dev *dev, long size) {
    int err = 0;

    /* set devices size and reallocate curval-buffer if necessary */
    mutex_lock(&dev->mutex);
    dev->dev_size = size;
    if (dev->dev_size < size) {
        dev->curval = krealloc(dev->curval, size, GFP_KERNEL);

        if (!dev->dev_size)
            err = -ENOMEM;
    }
    mutex_unlock(&dev->mutex);

    return err;
}

/**
 * Configures the device. @see matrixdriver.h for supported requests
 * and their return-values.
 *
 * @param fd  the device to configure
 * @param req the configuration request
 * @param arg the configuration payload
 * @return 0 on success,
 *         -EINVAL on failure
 */
static long matrixdriver_ioctl(struct file *fd, unsigned int req, unsigned long arg) {
    long ret = 0;
    struct matrixdriver_dev *dev = fd->private_data;

    switch (req) {

        case matrixdriver_IOCTL_BITBANG_DELAY:
            dev_info(dev->dev, "setting bitbang_delay=%ld", arg);
            dev->bitbang_delay = arg;
            break;

        case matrixdriver_IOCTL_SIZE:
            dev_info(dev->dev, "setting dev_size=%ld", arg);
            matrixdriver_set_devsize(dev, arg);
            break;

        default:
            ret = -EINVAL;
            break;
    }

    return ret;
}

static const struct file_operations matrixdriver_fops = {
        .write          = matrixdriver_write,    /* write to the device */
        .read           = matrixdriver_read,     /* current value of the device, e.g. last written bytes */
        .open           = matrixdriver_open,     /* open the device */
        .release        = matrixdriver_release,  /* close the device */
        .llseek         = no_llseek,          /* don't support seek */
        .unlocked_ioctl = matrixdriver_ioctl,    /* primary interface to access the driver from userspace */
};

/* used to map device-tree entries to the matching driver */
static const struct of_device_id tpic_dt_ids[] = {
        { .compatible = "texasinstruments,matrixdriver" },
        { /* sentinel */ },
};

/**
 * Probing. Responsible for the initialization of a concrete
 * device after a device-tree-entry was matched to a device
 * managed by this driver.
 *
 * @param pdev the platform_device to probe
 * @return zero on success,
 *         non-zero error-code on failure
 */
static int matrixdriver_probe(struct platform_device *pdev) {
    int err = 0;
    struct matrixdriver_dev *tpic_dev;
    unsigned long minor;

    pr_info("probing for matrixdriver\n");

    /* allocate memory for device */
    tpic_dev = kzalloc(sizeof(struct matrixdriver_dev), GFP_KERNEL);
    if (!tpic_dev)
        return -ENOMEM;

    /* allocate the devices buffer */
    tpic_dev->buf = kzalloc(matrixdriver_BUFFER_SIZE, GFP_KERNEL);
    if (!tpic_dev->buf) {
        err = -ENOMEM;
        goto err_free_dev;
    }

    /* init list-entry, lock the list and find the next available minor-number */
    INIT_LIST_HEAD(&tpic_dev->device_entry);
    mutex_lock(&device_list_lock);
    minor = find_first_zero_bit(minors, matrixdriver_DEV_MINORS);

    /* if available minor found, create device*/
    if (minor >= matrixdriver_DEV_MINORS)
        err = -ENODEV;
    else {
        tpic_dev->dev_t = MKDEV(matrixdriver_DEV_MAJOR, minor);
        tpic_dev->dev = device_create(matrixdriver_class, &pdev->dev, tpic_dev->dev_t, tpic_dev, "matrixdriver.%ld", minor);
        err = PTR_ERR_OR_ZERO(tpic_dev->dev);

        /* if device was created, reserve the minor-number and add the device to list*/
        if (err == 0) {
            set_bit(minor, minors);
            list_add(&tpic_dev->device_entry, &device_list);

            /* set private driver data for dev */
            dev_set_drvdata(tpic_dev->dev, tpic_dev);
        }
    }
    mutex_unlock(&device_list_lock);

    /* return early if failed to create device */
    if (err != 0) {
        pr_err("no minor numbers available or failed to create device\n");
        goto err_free_buf;
    }

    /* init & add cdev */
    cdev_init(&tpic_dev->cdev, &matrixdriver_fops);
    if ((err = cdev_add(&tpic_dev->cdev, tpic_dev->dev_t, 1)) < 0) {
        pr_err("failed to add character device\n");
        goto err_free_buf;
    }

    /* set bitbang-delay and allocate devices cur-val buffer */
    tpic_dev->bitbang_delay   = matrixdriver_BITBANG_UDELAY;
    if (matrixdriver_set_devsize(tpic_dev, 1) < 0) {
        pr_err("failed to set devsize\n");
        goto err_free_buf;
    }

    /* init device mutex */
    mutex_init(&tpic_dev->mutex);

    /* gpio_srck */
    tpic_dev->gpio_srck = gpiod_get(&pdev->dev, "srck", GPIOD_OUT_HIGH);
    if (IS_ERR(tpic_dev->gpio_srck)) {
        err = PTR_ERR(tpic_dev->gpio_srck);
        pr_err("failed to get gpio_srck\n");
        goto err_free_curval;
    }

    /* gpio_rck */
    tpic_dev->gpio_rck = gpiod_get(&pdev->dev, "rck", GPIOD_OUT_HIGH);
    if (IS_ERR(tpic_dev->gpio_rck)) {
        err = PTR_ERR(tpic_dev->gpio_rck);
        pr_err("failed to get gpio_rck\n");
        goto err_free_curval;
    }

    /* gpio_ser_in */
    tpic_dev->gpio_ser_in = gpiod_get(&pdev->dev, "ser-in", GPIOD_OUT_HIGH);
    if (IS_ERR(tpic_dev->gpio_ser_in)) {
        err = PTR_ERR(tpic_dev->gpio_ser_in);
        pr_err("failed to get gpio_ser_in\n");
        goto err_free_curval;
    }

    /* set platform driver data */
    platform_set_drvdata(pdev, tpic_dev);

    pr_info("matrixdriver probe done\n");

    return 0;

    /* free memory on failure */
    err_free_curval:
    kfree(tpic_dev->curval);
    err_free_buf:
    kfree(tpic_dev->buf);
    err_free_dev:
    kfree(tpic_dev);
    return err;
}

/**
 * Removes a previously probed device. This frees all resources
 * associated with the given device.
 *
 * @param pdev the platform-device to remove
 * @return zero on success,
 *         non-zero error-code on failure
 */
static int matrixdriver_remove(struct platform_device *pdev) {
    struct matrixdriver_dev *dev = platform_get_drvdata(pdev);

    if (!dev) {
        dev_err(&pdev->dev, "failed to get driver-data\n");
        return -ENODEV;
    }

    mutex_lock(&device_list_lock);

    /* delete list-entry and destroy the device, free the minor number */
    list_del(&dev->device_entry);
    device_destroy(matrixdriver_class, dev->dev_t);
    clear_bit(MINOR(dev->dev_t), minors);

    /* free gpios */
    if (dev->gpio_rck)
        gpiod_put(dev->gpio_rck);
    if (dev->gpio_ser_in)
        gpiod_put(dev->gpio_ser_in);
    if (dev->gpio_srck)
        gpiod_put(dev->gpio_srck);

    kthread_stop(dev->thread);

    /* free memory */
    kfree(dev);

    mutex_unlock(&device_list_lock);

    pr_info("removed matrixdriver\n");

    return 0;
}

/* platform-driver representation of this driver */
static struct platform_driver matrixdriver_driver = {
        .driver = {
                .name           = "matrixdriver",
                .of_match_table = of_match_ptr(tpic_dt_ids),
                .owner          = THIS_MODULE,
        },
        .probe  = matrixdriver_probe,
        .remove = matrixdriver_remove,
};

/**
 * Initializes this driver. E.g. creates it's class, registers with the system
 * as a character-device etc.
 *
 * @return zero on success,
 *         non-zero error-code on failure
 */
static int __init matrixdriver_init(void) {
    int status;

    /* create class */
    matrixdriver_class = class_create(THIS_MODULE, matrixdriver_driver.driver.name);
    if (IS_ERR(matrixdriver_class)) {
        pr_err("failed to create class\n");
        return PTR_ERR(matrixdriver_class);
    }

    /* register character-device for access from userspace */
    status = register_chrdev(matrixdriver_DEV_MAJOR, matrixdriver_driver.driver.name, &matrixdriver_fops);
    if (status < 0) {
        class_destroy(matrixdriver_class);
        pr_err("failed to register character-device\n");
        return status;
    }

    /* register platform driver, this will result in probing */
    status = platform_driver_register(&matrixdriver_driver);
    if (status < 0) {
        unregister_chrdev(matrixdriver_DEV_MAJOR, matrixdriver_driver.driver.name);
        class_destroy(matrixdriver_class);
        pr_err("failed to register platform driver\n");
        return status;
    }

    pr_info("initialized matrixdriver\n");

    return 0;
}
module_init(matrixdriver_init);

/**
 * Uninitializes this driver, e.g. undoes everything
 * that was done during initialization.
 */
static void __exit matrixdriver_exit(void) {
    /* unregister platform-driver */
    platform_driver_unregister(&matrixdriver_driver);

    /* unregister character-device */
    unregister_chrdev(matrixdriver_DEV_MAJOR, matrixdriver_driver.driver.name);

    /* unregister class */
    class_destroy(matrixdriver_class);

    pr_info("exit matrixdriver\n");
}
module_exit(matrixdriver_exit);

MODULE_DESCRIPTION("Driver to control matrix of LEDs with Texas Instruments 8-bit shift registers");
MODULE_AUTHOR("Pavel Kasila, <pavel.kasila@gmail.com>");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
