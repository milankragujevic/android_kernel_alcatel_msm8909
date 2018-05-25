#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/uaccess.h>
#include <linux/proc_fs.h>
#include <linux/module.h>
#include <linux/lcd_monitor.h>

#define BASE_DIR_NAME    "tct_debug"
#define TP_DIR_NAME      "tp"
#define LCD_DIR_NAME     "lcd"
#ifdef CONFIG_SENSORS_HALL
#define HALL_DIR_NAME    "hall"
#endif

struct ctp_vendor_info {
    char md5[10];
    char name[32];
    u8   id;
};

static struct proc_dir_entry *base;
static struct proc_dir_entry *tp_dir, *lcd_dir;
static struct proc_dir_entry *tp_entry, *lcd_entry, *gesture_entry, *palm_entry;
static struct proc_dir_entry *gtp_cfg_entry, *gtp_fw_entry, *gtp_id_entry;
#ifdef CONFIG_SENSORS_HALL
static struct proc_dir_entry *hall_dir, *hall_status_entry;
#endif

static struct ctp_vendor_info all_ctp_vendor_info[] = {
#ifdef KERNEL_FIRST_TP
    {KERNEL_FIRST_TP, "80701-0A5908B", 0},
#endif
#ifdef KERNEL_SECOND_TP
    {KERNEL_SECOND_TP, "DPT80701-0A5879E", 0x34},
#endif
#ifdef KERNEL_THIRD_TP
    {KERNEL_THIRD_TP, "CTPXXXXXXXXX", 0xFF},
#endif
};

char tp_info[512];
EXPORT_SYMBOL(tp_info);

char lcd_info[512];
EXPORT_SYMBOL(lcd_info);

int gesture_wakeup_flag = 0;
EXPORT_SYMBOL(gesture_wakeup_flag);

bool  palm_suspend_flag = false;
EXPORT_SYMBOL(palm_suspend_flag);

unsigned int gtp_cfg_version = 0;
EXPORT_SYMBOL(gtp_cfg_version);

unsigned int gtp_fw_version = 0;
EXPORT_SYMBOL(gtp_fw_version);

u8 gtp_sensor_id = 0;
EXPORT_SYMBOL(gtp_sensor_id);

#ifdef CONFIG_SENSORS_HALL
u16 g_hall_status = 250;
EXPORT_SYMBOL(g_hall_status);
#endif

u16 g_ctp_is_founded = 0;
EXPORT_SYMBOL(g_ctp_is_founded);

static ssize_t tp_info_read(struct file *filp, char __user *page,
                            size_t size, loff_t *ppos)
{
    char *ptr = page;
    static bool is_tp_info_loaded = false;
    int i, count = -1, len;
    count = sizeof(all_ctp_vendor_info) / sizeof(all_ctp_vendor_info[0]);

    if (*ppos)
        return 0;

    if (strlen(tp_info) == 0) {
        for (i = 0; i < count; i++) {
            len = strlen(all_ctp_vendor_info[i].name);

            if (len != 0)
                strncat(tp_info, all_ctp_vendor_info[i].name, len);

            else
                strncat(tp_info, "CTPXXXX", 7);

            strncat(tp_info, "_", 1);
            len = strlen(all_ctp_vendor_info[i].md5);

            if (len != 0 && len >= 8)
                strncat(tp_info, all_ctp_vendor_info[i].md5, 8);

            else
                strncat(tp_info, "MD5XXXX", 7);

            if (gtp_sensor_id == all_ctp_vendor_info[i].id)
                strncat(tp_info, "*", 1);

            strncat(tp_info, "\n", 1);
            is_tp_info_loaded = true;
        }
    }

    if (is_tp_info_loaded == false) {
        strncat(tp_info, "UnKown\n", 7);
        is_tp_info_loaded = true;
    }

    ptr += sprintf(ptr, "%s", tp_info);
    *ppos += (ptr - page);
    return (ptr - page);
}

static const struct file_operations tp_info_proc_ops = {
    .owner = THIS_MODULE,
    .read = tp_info_read,
    .write = NULL,
};

static ssize_t lcd_info_read(struct file *filp, char __user *page,
                             size_t size, loff_t *ppos)
{
    char *ptr = page;

    if (*ppos)
        return 0;

    ptr += sprintf(ptr, "Unknow");
    *ppos += (ptr - page);

    return (ptr - page);
}


static const struct file_operations lcd_info_proc_ops = {
    .owner = THIS_MODULE,
    .read = lcd_info_read,
    .write = NULL,
};

static ssize_t gesture_wakeup_read(struct file *filp, char __user *page,
                                   size_t size, loff_t *ppos)
{
    char *ptr = page;

    if (*ppos)
        return 0;

    ptr += sprintf(ptr, "%d\n", gesture_wakeup_flag);
    *ppos += (ptr - page);
    return (ptr - page);
}

static ssize_t gesture_wakeup_write(struct file *filp, const char __user *page,
                                    size_t size, loff_t *ppos)
{
    char buf[64];
    int val, ret;

    if (size >= sizeof(buf))
        return -EINVAL;

    if (copy_from_user(&buf, page, size))
        return -EFAULT;

    buf[size] = 0;
    ret = strict_strtoul(buf, 10, (unsigned long *)&val);

    if (val == 0) {
        gesture_wakeup_flag = 0;

    } else if (val == 1) {
        gesture_wakeup_flag = 1;
    }

    if (ret < 0)
        return ret;

    pr_err("%s gesture wakeup function!\n", (gesture_wakeup_flag ? "Enable" : "Disable"));
    return size;
}

static struct file_operations gesture_wakeup_ops = {
    .owner = THIS_MODULE,
    .read = gesture_wakeup_read,
    .write = gesture_wakeup_write,
};

static ssize_t palm_suspend_read(struct file *filp, char __user *page,
                                 size_t size, loff_t *ppos)
{
    char *ptr = page;

    if (*ppos)
        return 0;

    ptr += sprintf(ptr, "%d\n", palm_suspend_flag);
    *ppos += (ptr - page);
    return (ptr - page);
}

static ssize_t palm_suspend_write(struct file *filp, const char __user *page,
                                  size_t size, loff_t *ppos)
{
    char buf[64];
    int val, ret;

    if (size >= sizeof(buf))
        return -EINVAL;

    if (copy_from_user(&buf, page, size))
        return -EFAULT;

    buf[size] = 0;
    ret = strict_strtoul(buf, 10, (unsigned long *)&val);

    if (val == 0) {
        palm_suspend_flag = false;

    } else if (val == 1) {
        palm_suspend_flag = true;
    }

    if (ret < 0)
        return ret;

    pr_err("%s palm suspend function!\n", (palm_suspend_flag ? "Enable" : "Disable"));
    return size;
}

static struct file_operations palm_suspend_ops = {
    .owner = THIS_MODULE,
    .read = palm_suspend_read,
    .write = palm_suspend_write,
};

static ssize_t gtp_cfg_version_read(struct file *filp, char __user *page,
                                    size_t size, loff_t *ppos)
{
    char *ptr = page;

    if (*ppos)
        return 0;

    ptr += sprintf(ptr, "0x%x\n", gtp_cfg_version);
    *ppos += (ptr - page);
    return (ptr - page);
}

static struct file_operations gtp_cfg_ops = {
    .owner = THIS_MODULE,
    .read = gtp_cfg_version_read,
    .write = NULL,
};

static ssize_t gtp_fw_version_read(struct file *filp, char __user *page,
                                   size_t size, loff_t *ppos)
{
    char *ptr = page;

    if (*ppos)
        return 0;

    ptr += sprintf(ptr, "%u\n", gtp_fw_version);
    *ppos += (ptr - page);
    return (ptr - page);
}

static struct file_operations gtp_fw_ops = {
    .owner = THIS_MODULE,
    .read = gtp_fw_version_read,
    .write = NULL,
};

static ssize_t gtp_sensor_id_read(struct file *filp, char __user *page,
                                  size_t size, loff_t *ppos)
{
    char *ptr = page;

    if (*ppos)
        return 0;

    ptr += sprintf(ptr, "%x\n", gtp_sensor_id);
    *ppos += (ptr - page);
    return (ptr - page);
}

static struct file_operations gtp_id_ops = {
    .owner = THIS_MODULE,
    .read = gtp_sensor_id_read,
    .write = NULL,
};

#ifdef CONFIG_SENSORS_HALL
static ssize_t hall_status_read(struct file *filp, char __user *page,
                                size_t size, loff_t *ppos)
{
    char *ptr = page;

    if (*ppos)
        return 0;

    ptr += sprintf(ptr, "%d\n", g_hall_status);
    *ppos += (ptr - page);
    return (ptr - page);
}

static struct file_operations hall_status_ops = {
    .owner = THIS_MODULE,
    .read = hall_status_read,
    .write = NULL,
};
#endif

static int __init tct_debug_init(void)
{
    base = proc_mkdir(BASE_DIR_NAME, NULL);
    tp_dir = proc_mkdir(TP_DIR_NAME, base);
    lcd_dir = proc_mkdir(LCD_DIR_NAME, base);
#ifdef CONFIG_SENSORS_HALL
    hall_dir = proc_mkdir(HALL_DIR_NAME, base);
#endif
    tp_entry = proc_create("tp_fw_cfg.ver", 0444, tp_dir, &tp_info_proc_ops);

    if (tp_entry == NULL) {
        pr_err("Create tp proc entry failed!\n");
        goto err_create_tp_info_entry;
    }

    lcd_entry = proc_create("initcode.ver", 0444, lcd_dir, &lcd_info_proc_ops);

    if (lcd_entry == NULL) {
        pr_err("Create lcd proc entry failed!\n");
        goto err_create_lcd_info_entry;
    }

    gesture_entry = proc_create("gesture_enable", 0666, tp_dir,
                                &gesture_wakeup_ops);

    if (gesture_entry == NULL) {
        pr_err("Create gesture enable/disable entry failed!\n");
        goto err_create_gesture_wakeup_entry;
    }

    palm_entry = proc_create("palm_suspend_enable", 0666, tp_dir,
                             &palm_suspend_ops);

    if (palm_entry == NULL) {
        pr_err("Create palm suspend enable/disable entry failed!\n");
        goto err_create_palm_suspend_entry;
    }

    gtp_cfg_entry = proc_create("cfg_version", 0664, tp_dir,
                                &gtp_cfg_ops);

    if (gtp_cfg_entry == NULL) {
        pr_err("Create gtp cfg version entery failed!\n");
        goto err_create_gtp_cfg_entry;
    }

    gtp_fw_entry = proc_create("firmware_version", 0664, tp_dir,
                               &gtp_fw_ops);

    if (gtp_fw_entry == NULL) {
        pr_err("Create gtp firmware version entery failed!\n");
        goto err_create_gtp_fw_entry;
    }

    gtp_id_entry = proc_create("sensor_id", 0664, tp_dir,
                               &gtp_id_ops);

    if (gtp_id_entry == NULL) {
        pr_err("Create gtp sensor id entry failed!\n");
        goto err_create_gtp_sensor_id_entry;
    }

#ifdef CONFIG_SENSORS_HALL
    hall_status_entry = proc_create("hall_status", 0664, hall_dir,
                                    &hall_status_ops);

    if (hall_status_entry == NULL) {
        pr_err("Create hall status entry failed!\n");
        goto err_create_hall_status_entry;
    }

#endif
    return 0;
#ifdef CONFIG_SENSORS_HALL
err_create_hall_status_entry:
    proc_remove(gtp_id_entry);
#endif
err_create_gtp_sensor_id_entry:
    proc_remove(gtp_fw_entry);
err_create_gtp_fw_entry:
    proc_remove(gtp_cfg_entry);
err_create_gtp_cfg_entry:
    proc_remove(palm_entry);
err_create_palm_suspend_entry:
    proc_remove(gesture_entry);
err_create_gesture_wakeup_entry:
    proc_remove(lcd_entry);
err_create_lcd_info_entry:
    proc_remove(tp_entry);
err_create_tp_info_entry:
    proc_remove(lcd_dir);
    proc_remove(tp_dir);
    proc_remove(base);
    return -1;
}

static void __exit tct_debug_exit(void)
{
#ifdef CONFIG_SENSORS_HALL
    proc_remove(hall_status_entry);
#endif
    proc_remove(gtp_id_entry);
    proc_remove(gtp_fw_entry);
    proc_remove(gtp_cfg_entry);
    proc_remove(gesture_entry);
    proc_remove(palm_entry);
    proc_remove(lcd_entry);
    proc_remove(tp_entry);
    proc_remove(lcd_dir);
    proc_remove(tp_dir);
    proc_remove(base);
    return;
}

module_init(tct_debug_init);
module_exit(tct_debug_exit);
MODULE_LICENSE("GPL");

