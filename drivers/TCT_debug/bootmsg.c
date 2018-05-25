/*
 * bootmsg.c - keep the first BOOTMSG_BUFFER_SZ bytes of kernel log to memory
 *
 * Add bootmsg by wenzhao.guo for PR 937829 on 03/30/2015
 *
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/console.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/proc_fs.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <soc/qcom/smem.h>

#define BOOTMSG_BUFFER_SZ (256 << 10)
#define RELEASE_RETRY_DELAY (msecs_to_jiffies(60 * MSEC_PER_SEC))

static long  bootmsg_buffer_phys = 0x0;
static char *bootmsg_buffer = NULL;
static int   bootmsg_len    = 0;

static bool should_release_console = false;

/*yongzhong.cheng:PR-907051,use usb ID to print log,Start*/
#define DELAY_TIME 102

#if defined(JRD_PROJECT_POP45)
#define SBL1_USB_ID_GPIO (23+911)
#else
#define SBL1_USB_ID_GPIO (65+911)
#endif

#define RETRY_DELAY_CYZ (msecs_to_jiffies(60 * MSEC_PER_SEC))  //60 is ok

char *dst;
char *printstr = NULL;
unsigned long flags;
spinlock_t	tct_uart_lock;
static int  printstr_len=0;
struct delayed_work		usb_uart_work;


void write_byte(unsigned char tct_char)
{
	unsigned char i = 8;

	spin_lock_irqsave(&tct_uart_lock, flags);
	gpio_direction_output(SBL1_USB_ID_GPIO, 0);
	udelay(DELAY_TIME);

	while(i--)
	{
		if(tct_char & 0x1)
			gpio_direction_output(SBL1_USB_ID_GPIO, 1);
		else
			gpio_direction_output(SBL1_USB_ID_GPIO, 0);

		udelay(DELAY_TIME);
		tct_char = tct_char >> 1;

	}
	gpio_direction_output(SBL1_USB_ID_GPIO, 1);
	udelay(DELAY_TIME);
	spin_unlock_irqrestore(&tct_uart_lock,flags);
}


void usb_uart_fn(struct work_struct *work)
{
	int i = 0;

	memcpy(printstr, (bootmsg_buffer+printstr_len),
		(strlen(bootmsg_buffer)-printstr_len));
	printstr_len=strlen(printstr)+printstr_len;

	for ( i = 0; i < printstr_len; i++) {
		write_byte(*printstr++);
		if(10==(int)*printstr){
			//10 ASCII is newline
			write_byte(0X0D);
		}
	}
	schedule_delayed_work(container_of(work, struct delayed_work, work),
				RETRY_DELAY_CYZ);
}
DECLARE_DELAYED_WORK(usb_uart_work,usb_uart_fn);

int tct_uart_puts(const char *str,int len)
{
	int ret,i;
	static int initonce=0;

	if(initonce==0){
		ret = gpio_request(SBL1_USB_ID_GPIO,"usbgpio");	 
		if(ret < 0)
		{
			printk(KERN_ERR "%s: gpio_request SBL1_USB_ID_GPIO, err=%d", __func__, ret);
			return 0;
		}

		initonce++;
		printstr = kzalloc(BOOTMSG_BUFFER_SZ, GFP_KERNEL);
		spin_lock_init(&tct_uart_lock);
		INIT_DELAYED_WORK(&usb_uart_work,usb_uart_fn);
	}

	memcpy(printstr, (bootmsg_buffer), (strlen(bootmsg_buffer)));
	printstr_len=strlen(printstr)+printstr_len;

	for ( i = 0; i < printstr_len; i++) {
		write_byte(*printstr++);
		if(10==(int)*printstr){
			//10 ASCII is newline
			write_byte(0X0D);
		}
	}

	schedule_delayed_work(&usb_uart_work, RETRY_DELAY_CYZ);
	return 0;
}
/*yongzhong.cheng:PR-907051,use usb ID to print log,Start*/

static void
bootmsg_console_write(struct console *console, const char *s, unsigned int count)
{
	int len = min(count, (unsigned int) (BOOTMSG_BUFFER_SZ - bootmsg_len));
	if (len) {
		memcpy(bootmsg_buffer + bootmsg_len, s, len);
		bootmsg_len += len;
	}

	if (bootmsg_len >= BOOTMSG_BUFFER_SZ) {
		console->flags &= ~CON_ENABLED;
		should_release_console = true;
	}
}

static struct console bootmsg_console = {
	.name	= "bootmsg",
	.write	= bootmsg_console_write,
	.flags	= CON_PRINTBUFFER | CON_ENABLED | CON_ANYTIME,
	.index	= -1,
};

static ssize_t bootmsg_knob_read(struct file *f, char __user *buf,
				       size_t count, loff_t *ppos)
{
	return simple_read_from_buffer(buf, count, ppos, bootmsg_buffer, bootmsg_len);
}

static const struct file_operations bootmsg_fops = {
	.open	= simple_open,
	.read	= bootmsg_knob_read,	
};


static void release_console_work_fn(struct work_struct *work)
{
	if (should_release_console) {
		unregister_console(&bootmsg_console);
		pr_info("bootmsg is disabled (bootmsg_buffer_phys=%08lx)\n",
				bootmsg_buffer_phys);
	} else
		schedule_delayed_work(container_of(work, struct delayed_work, work),
				RELEASE_RETRY_DELAY);
}
static DECLARE_DELAYED_WORK(release_console_work, release_console_work_fn);

//yongzhong.cheng@tcl.com,TASK-1220939,2015/12/24,trace otg uart log
uint32_t get_hack_otg_uart_flag(void)
{   
	int *temp_ret; 
	int hack_otg_uart_flag=-2;
	temp_ret = smem_alloc(SMEM_BATTERY_VOLTAGE, sizeof(int),0,SMEM_ANY_HOST_FLAG);
	if(temp_ret!=NULL)
	{
		hack_otg_uart_flag = *(temp_ret+1);
	}
	
	printk("hack_otg_uart_flag=%u\n",hack_otg_uart_flag);
	return  hack_otg_uart_flag;
}
//yongzhong.cheng@tcl.com,TASK-1220939,2015/12/24,trace otg uart log

static int __init bootmsg_release_console_work_init(void)
{
	schedule_delayed_work(&release_console_work, RELEASE_RETRY_DELAY);

	/*yongzhong.cheng:PR-907051,use usb ID to print log,Start*/
	//if(1==get_hack_otg_uart_flag()){
		//tct_uart_puts(bootmsg_buffer,strlen(bootmsg_buffer));
	//}
	/*yongzhong.cheng:PR-907051,use usb ID to print log,end*/

	return 0;
}
late_initcall(bootmsg_release_console_work_init);

static int __init bootmsg_console_init(void)
{
	WARN_ON(BOOTMSG_BUFFER_SZ % PAGE_SIZE);
	bootmsg_buffer = kzalloc(BOOTMSG_BUFFER_SZ, GFP_KERNEL);
	if (bootmsg_buffer) {
		bootmsg_buffer_phys = virt_to_phys(bootmsg_buffer);
		printk("bootmsg is enable %p(phys=%08lx)\n", bootmsg_buffer, bootmsg_buffer_phys);
		register_console(&bootmsg_console);
	} else
		pr_err("%s: fail to alloc buffer\n", __func__);

        if (!proc_create("bootmsg", (S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH ), NULL,&bootmsg_fops)){
		printk("bootmsg: creat bootmsg  fail!\n");
		return 0;
       }
	return 0;
}
core_initcall(bootmsg_console_init);
