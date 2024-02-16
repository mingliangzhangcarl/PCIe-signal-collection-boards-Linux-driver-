#include <linux/init.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/dmaengine.h>
#include <linux/list.h>
#include <linux/cdev.h>
#include <linux/mutex.h>

#include "echodev-cmd.h"

#define DEVNR 64
#define DEVNRNAME "echodev"

#define VID 0x1234
#define DID 0xbeef

#define DMA_SRC 0x10
#define DMA_DST 0x18
#define DMA_CNT 0x20
#define DMA_CMD 0x28
#define DMA_RUN 1


#define timeout_send_ms  		50
#define timeout_process_ms 		105
#define timeout_recv_ms  		10

#define TO_DEVICE_FRAME_LENGTH 	14
#define SEND_BUFFER_SIZE    	14
#define RECV_BUFFER_SIZE		4096
#define DEVICE_BAR_SIZE			4096

#define LOG_INFO(fmt, ...) pr_info("%s %s() line %d " fmt "\n", __FILE__, __FUNCTION__, __LINE__, ##__VA_ARGS__)
#define LOG_ERR(fmt, ...) pr_err("%s %s() line %d " fmt "\n", __FILE__, __FUNCTION__, __LINE__, ##__VA_ARGS__)

typedef enum {
    INITING,
    SENDING,
    PROCESSING,
    RECEIVING,
    RECEIVED,
    FINISH,
    FAILED
} State;

struct control_cdev{
    struct echo_dev*		pedev;
    dev_t 					cdev_nr;			/* character device major:minor */
    struct cdev 			cdev;			/* character device embedded struct */
    char* 					buffer;
    // struct device 		*sys_device;	/* sysfs device */
    spinlock_t 				buffer_lock;
};

struct event_cdev{
    struct echo_dev* 		pedev;
    dev_t 					cdev_nr;			/* character device major:minor */
    struct cdev 			cdev;			/* character device embedded struct */
    int 					event;
    // struct device 		*sys_device;	/* sysfs device */
	spinlock_t 				event_lock;
};

struct engine{
	struct work_struct      work;
	struct echo_dev* 		pedev;
	const char __user *     user_buffer;
    void *   				send_buffer;
	bool					is_copied;
    uint32_t 				send_count;
    dma_addr_t 				send_dma_addr;
    void *   				recv_buffer;
    uint32_t 				recv_count;
    dma_addr_t 				recv_dma_addr;
    State      				cur_state;
	spinlock_t 				state_lock;
};

struct echo_dev {
	struct pci_dev 			*pdev;
	int 					card_idx;
	void __iomem 			*ptr_bar0;
	struct list_head 		list;
	struct cdev 			cdev;
	int   					irq_nr;
	struct control_cdev 	ctrl_cdev;
    struct event_cdev 		event_cdev;
	struct engine			engine;
	wait_queue_head_t 		irq_wq;
	wait_queue_head_t 		write_wq;
	wait_queue_head_t 		event_wq;
};

/* Global Variables */
LIST_HEAD(card_list);
static struct mutex lock;
static int minor_count = 0;
static int card_count = 0;


static irqreturn_t echo_irq_handler(int irq_nr, void *data)
{
	unsigned long flags;
	struct echo_dev *echo = (struct echo_dev *) data;
    uint32_t temp = ioread32(echo->ptr_bar0 + 8);
	iowrite32(0xffffffff, echo->ptr_bar0 + 8);
    LOG_INFO("echo_dev-drv - Legacy IRQ triggered ! interrput reg = %lx", temp);
	spin_lock_irqsave(&echo->engine.state_lock, flags);
    LOG_INFO("echo->cur_state->cur_state = %d",echo->engine.cur_state);
    if(temp & 0x1) {
		LOG_INFO(" dma translation finish!");
		if (echo->engine.cur_state == SENDING || echo->engine.cur_state == RECEIVING){
			echo->engine.cur_state++;
		}else{
			echo->engine.cur_state = FAILED;
		}
    }else if (temp & 0x2){
		LOG_INFO(" process finish!");
		if (echo->engine.cur_state == PROCESSING){
			echo->engine.cur_state++;
		}else{
			echo->engine.cur_state = FAILED;
		} 
    }else{
		echo->engine.cur_state = FAILED;
		LOG_INFO(" undefined interruput status");
    }
	spin_unlock_irqrestore(&echo->engine.state_lock, flags);
    wake_up_interruptible(&(echo->irq_wq));
    
    return IRQ_HANDLED;
}



static void work_handler(struct work_struct *work) 
{
    int rv,count;
	unsigned long flags;
    struct engine *engine_p = container_of(work, struct engine, work);
    struct echo_dev* echo_p = engine_p->pedev;
    struct control_cdev* ctrl_cdev_p = &echo_p->ctrl_cdev;
    struct event_cdev* event_cdev_p = &echo_p->event_cdev;
    LOG_INFO("into work_handler function");
    // step 1 : copy data from user buffer to send buffer 
    rv = copy_from_user(engine_p->send_buffer, engine_p->user_buffer, engine_p->send_count);
    engine_p->is_copied = 1;
    // step 2 : notify write_wq functions that data copy was finished
    wake_up_interruptible(&echo_p->write_wq);
	// step 3 : Setup the DMA controller */
	iowrite32(engine_p->send_count, echo_p->ptr_bar0 + DMA_CNT);
	iowrite32(engine_p->send_dma_addr, echo_p->ptr_bar0 + DMA_SRC);
	iowrite32(0, echo_p->ptr_bar0 + DMA_DST);
	spin_lock_irqsave(&engine_p->state_lock, flags);
	engine_p->cur_state = SENDING;
	spin_unlock_irqrestore(&engine_p->state_lock, flags);
	iowrite32(DMA_RUN | DMA_TO_DEVICE, echo_p->ptr_bar0 + DMA_CMD);
    // step 4 : wait the data send finish
    rv = wait_event_interruptible_timeout(echo_p->irq_wq,engine_p->cur_state != SENDING, msecs_to_jiffies(timeout_send_ms));
    spin_lock_irqsave(&engine_p->state_lock, flags);
	if (rv == 0 || engine_p->cur_state == FAILED || engine_p->cur_state == SENDING){
        LOG_ERR("data send failed rv = %d, cur_statue = %d", rv,engine_p->cur_state);
		spin_unlock_irqrestore(&engine_p->state_lock, flags);
		count = -1;
        goto failed;
    }
	spin_unlock_irqrestore(&engine_p->state_lock, flags);
	// step 5 : fire up process
	iowrite32(1, echo_p->ptr_bar0 + 0x32);
    // step 6 : wait process finish
    rv = wait_event_interruptible_timeout(echo_p->irq_wq,engine_p->cur_state != PROCESSING, msecs_to_jiffies(timeout_process_ms));
    spin_lock_irqsave(&engine_p->state_lock, flags);
	if (rv == 0 || engine_p->cur_state == FAILED || engine_p->cur_state == PROCESSING){
		LOG_ERR("process failed rv = %d, cur_statue = %d", rv,engine_p->cur_state);
		spin_unlock_irqrestore(&engine_p->state_lock, flags);
		count = -2;
        goto failed;
    }
	spin_unlock_irqrestore(&engine_p->state_lock, flags);
	// step 7 : read register get the length of signal
	count = ioread32(echo_p->ptr_bar0 + DMA_CNT);
	LOG_INFO("device to host data length = %x",count);
    // step 8 : config dma register && state trans && fire dma
	iowrite32(0, echo_p->ptr_bar0 + DMA_SRC);
	iowrite32(engine_p->recv_dma_addr, echo_p->ptr_bar0 + DMA_DST);
	/* Let's fire the dma */
	iowrite32(DMA_RUN | DMA_FROM_DEVICE, echo_p->ptr_bar0 + DMA_CMD);
	// step 9 : wait dma trans finish
	rv = wait_event_interruptible_timeout(echo_p->irq_wq,engine_p->cur_state != RECEIVING, msecs_to_jiffies(timeout_recv_ms));
    spin_lock_irqsave(&engine_p->state_lock, flags);
	if (rv == 0 || engine_p->cur_state == FAILED || engine_p->cur_state == RECEIVING){
		LOG_ERR("recv data error rv = %d, cur_statue = %d", rv,engine_p->cur_state);
		spin_unlock_irqrestore(&engine_p->state_lock, flags);
		count = -3;
        goto failed;
    }
	spin_unlock_irqrestore(&engine_p->state_lock, flags);
	// step 10 : copy data to write buffer
	spin_lock_irqsave(&echo_p->ctrl_cdev.buffer_lock, flags);
	memcpy(echo_p->ctrl_cdev.buffer, engine_p->recv_buffer, count);
	spin_unlock_irqrestore(&echo_p->ctrl_cdev.buffer_lock, flags);
	// step 12 : change state
	spin_lock_irqsave(&engine_p->state_lock, flags);
	engine_p->cur_state = FINISH;
	spin_unlock_irqrestore(&engine_p->state_lock, flags);
	// step 11 : notify event_cdev
	spin_lock_irqsave(&echo_p->event_cdev.event_lock, flags);
	echo_p->event_cdev.event = count;
	spin_unlock_irqrestore(&echo_p->event_cdev.event_lock, flags);
	wake_up_interruptible(&echo_p->event_wq);
	LOG_INFO("work finish");
	return;
failed:
	// step failed : free resource && set status && notify wake_wq
	spin_lock_irqsave(&engine_p->state_lock, flags);
	engine_p->cur_state = FAILED;
	spin_unlock_irqrestore(&engine_p->state_lock, flags);

	spin_lock_irqsave(&echo_p->event_cdev.event_lock, flags);
	echo_p->event_cdev.event = count;
	spin_unlock_irqrestore(&echo_p->event_cdev.event_lock, flags);
	wake_up_interruptible(&echo_p->event_wq);
	LOG_INFO("work failed error code = %d", count);
    return;
}


static int dma_transfer(struct echo_dev *echo, void *buffer, int count, dma_addr_t addr, enum dma_data_direction dir)
{
	dma_addr_t buffer_dma_addr = dma_map_single(&echo->pdev->dev, buffer, count, dir);

	/* Setup the DMA controller */
	iowrite32(count, echo->ptr_bar0 + DMA_CNT);

	switch(dir) {
		case DMA_TO_DEVICE: /* 1 */
			iowrite32(buffer_dma_addr, echo->ptr_bar0 + DMA_SRC);
			iowrite32(addr, echo->ptr_bar0 + DMA_DST);
			break;
		case DMA_FROM_DEVICE: /* 2 */
			iowrite32(buffer_dma_addr, echo->ptr_bar0 + DMA_DST);
			iowrite32(addr, echo->ptr_bar0 + DMA_SRC);
			break;
		default:
			return -EFAULT;
	}

	/* Let's fire the dma */
	iowrite32(DMA_RUN | dir, echo->ptr_bar0 + DMA_CMD);
	mdelay(100);
	dma_unmap_single(&echo->pdev->dev, buffer_dma_addr, count, dir);
	return 0;
}
void test(struct echo_dev* echo);


static int control_cdev_open(struct inode *inode, struct file *file)
{
    struct echo_dev *echo;  
	dev_t dev_nr = inode->i_rdev;
    LOG_INFO("dev_nr = %d open_file",dev_nr);
	mutex_lock(&lock);
	list_for_each_entry(echo, &card_list, list) {
		if(echo->ctrl_cdev.cdev_nr == dev_nr) {			
			file->private_data = &echo->ctrl_cdev;
            mutex_unlock(&lock);
			return 0;
		}
	}
    mutex_unlock(&lock);
	return -ENODEV;
}

static int event_cdev_open(struct inode *inode, struct file *file)
{
    struct echo_dev *echo;  
	dev_t dev_nr = inode->i_rdev;
    LOG_INFO("dev_nr = %d open_file",dev_nr);
	mutex_lock(&lock);
	list_for_each_entry(echo, &card_list, list) {
		if(echo->event_cdev.cdev_nr == dev_nr) {			
			file->private_data = &echo->event_cdev;
            mutex_unlock(&lock);
			return 0;
		}
	}
    mutex_unlock(&lock);
	return -ENODEV;
}


static ssize_t event_cdev_read(struct file *file, char __user *user_buffer, size_t count, loff_t *offs)
{
	int rv;
	u32 events_user;
	unsigned long flags;
	struct event_cdev *cdev = (struct event_cdev *)file->private_data;
	struct echo_dev* echo = cdev->pedev; 
	if (count != 4)
		return -EPROTO;

	if (*offs & 3)
		return -EPROTO;

	/*
	 * sleep until any interrupt events have occurred,
	 * or a signal arrived
	 */
	rv = wait_event_interruptible(echo->event_wq,
			cdev->event != 0);
	if (rv)
		LOG_INFO("wait_event_interruptible=%d", rv);

	/* wait_event_interruptible() was interrupted by a signal */
	if (rv == -ERESTARTSYS)
		return -ERESTARTSYS;

	/* atomically decide which events are passed to the user */
	spin_lock_irqsave(&cdev->event_lock, flags);
	events_user = cdev->event;
	cdev->event = 0;
	spin_unlock_irqrestore(&cdev->event_lock, flags);

	rv = copy_to_user(user_buffer, &events_user, 4);
	if (rv)
		LOG_ERR("Copy to user failed but continuing rv = %d",rv);

	return 4;
}



static ssize_t control_cdev_write(struct file *file, const char __user *user_buffer, size_t count, loff_t *offs)
{
    int rv;
    void* send_buffer;
    dma_addr_t send_dma_addr;
	unsigned long flags;
    struct control_cdev *ctrl_cdev = (struct control_cdev *) file->private_data;
    struct echo_dev* echo = ctrl_cdev->pedev;
    LOG_INFO("into control_cdev_write function\n");
    LOG_INFO("check count\n");
    if (count != 14){
        LOG_INFO("into control_cdev_write function count != 14 \n");
	    return -EPROTO;
    }


	LOG_INFO("check offs\n");
    if (*offs != 0){
        LOG_ERR("into control_cdev_write function  *offs != 0 \n");
        return -EPROTO;
    }
        
    LOG_INFO("check engine_->cur_state\n");
	spin_lock_irqsave(&echo->engine.state_lock, flags);
	if (echo->engine.cur_state != FINISH && echo->engine.cur_state != FAILED){
        LOG_INFO("EBUSY cur_state = %d",echo->engine.cur_state);
		spin_unlock_irqrestore(&echo->engine.state_lock, flags);
        return -EBUSY;
    }else{
		echo->engine.cur_state = INITING;
	}
	spin_unlock_irqrestore(&echo->engine.state_lock, flags);
	// send_buffer = kmalloc(count, GFP_KERNEL);
    // if (!send_buffer) {
    //     return;
    // }
    // // step 2 : copy data from user buffer to send buffer 
    // copy_from_user(send_buffer, user_buffer, count);
	// dma_transfer(echo, send_buffer,0,count, DMA_TO_DEVICE);

	echo->engine.user_buffer = user_buffer;
	echo->engine.is_copied = 0;
	schedule_work(&echo->engine.work);
	rv = wait_event_interruptible_timeout(echo->write_wq,echo->engine.is_copied != 0, msecs_to_jiffies(1000));
	if (rv)
		LOG_INFO("wait_event_interruptible=%d", rv);

	/* wait_event_interruptible() was interrupted by a signal */
	if (rv == -ERESTARTSYS)
		return -ERESTARTSYS;
	*offs += 14;
	return 14;
}

static ssize_t control_cdev_read(struct file *file, char __user *user_buffer, size_t count, loff_t *offs)
{
    unsigned long flags;
    struct control_cdev *ctrl_cdev = (struct control_cdev *) file->private_data;
	int not_copied, to_copy = (count + *offs < RECV_BUFFER_SIZE) ? count : RECV_BUFFER_SIZE - *offs;
	LOG_INFO("count = %d offs = %d", count,*offs);   
    spin_lock_irqsave(&ctrl_cdev->buffer_lock, flags);
	not_copied = copy_to_user(user_buffer, ctrl_cdev->buffer, to_copy);
    spin_unlock_irqrestore(&ctrl_cdev->buffer_lock, flags);
	LOG_INFO("read finish");
	*offs += to_copy - not_copied;
    return to_copy - not_copied;
}

static int echo_open(struct inode *inode, struct file *file)
{
	struct echo_dev *echo;
	dev_t dev_nr = inode->i_rdev;

	LOG_INFO("dev_nr = %d open_file", dev_nr);
	mutex_lock(&lock);
	list_for_each_entry(echo, &card_list, list) {
		if(echo->ctrl_cdev.cdev_nr == dev_nr) {			
			file->private_data = echo;
            mutex_unlock(&lock);
			test(echo);
			return 0;
		}
	}
    mutex_unlock(&lock);
	return -ENODEV;
}

// static ssize_t echo_write(struct file *file, const char __user *user_buffer, size_t count, loff_t *offs)
// {
//         char *buf;
//         struct echo_dev *echo = (struct echo_dev *) file->private_data;
// 	if (echo->cur_task && echo->cur_task->cur_state != FINISH)
// 		return -EBUSY;
//         int not_copied, to_copy = (count + *offs < pci_resource_len(echo->pdev, 1)) ? count : pci_resource_len(echo->pdev, 1) - *offs;

//         if(to_copy == 0)
//                 return 0;
		
//         buf = kzalloc(to_copy, GFP_ATOMIC);
//     	not_copied = copy_from_user(buf, user_buffer, to_copy);
// 		echo->cur_task = kzalloc(sizeof(struct Task), GFP_ATOMIC);
//     	echo->cur_task->id = 123;
//     	echo->cur_task->send_buffer = buf;
//     	echo->cur_task->send_count = to_copy;
//         task_process(echo);
//         *offs += to_copy - not_copied;
//         return to_copy - not_copied;
// }

void test(struct echo_dev* echo)
{
	int send_count = 14;
    char *send_buf = kzalloc(send_count, GFP_ATOMIC);
	LOG_INFO("test start");
	send_buf[0] = 0xaa;

	send_buf[1] = 0x64; 
	send_buf[2] = 00; 
	send_buf[3] = 00;
	send_buf[4] = 00;

	send_buf[5] = 00;
	send_buf[6] = 0x10;
	send_buf[7] = 00;
	send_buf[8] = 00;

	send_buf[9] = 0x10;
	send_buf[10] = 0x10;
	send_buf[11] = 00;
	send_buf[12] = 00;

	send_buf[13] = 0xee;

	dma_transfer(echo, send_buf, send_count,0, DMA_TO_DEVICE);
	kfree(send_buf);
	LOG_INFO("test end");
}



/*
static ssize_t echo_write(struct file *file, const char __user *user_buffer, size_t count, loff_t *offs)
{
        char *buf;
        int not_copied, to_copy = (count + *offs < 4096) ? count : 4096 - *offs;
        struct echodev *echo = (struct echodev *) file->private_data;

        if(*offs >= pci_resource_len(echo->pdev, 1))
                return 0;

        buf = kmalloc(to_copy, GFP_ATOMIC);
        not_copied = copy_from_user(buf, user_buffer, to_copy);

        dma_transfer(echo, buf, to_copy, *offs, DMA_TO_DEVICE);

        kfree(buf);
        *offs += to_copy - not_copied;
        return to_copy - not_copied;
}
*/

// static ssize_t echo_read(struct file *file, char __user *user_buffer, size_t count, loff_t *offs)
// {
//         char *buf;
//         struct echo_dev *echo = (struct echo_dev *) file->private_data;
// 	if (!echo)
// 		return -1;
// 	if (echo->cur_task->cur_state == FAILED)
// 		return -2;
// 	if (echo->cur_task->cur_state != RECEIVED)
// 		return -3;
// 	printk("echo_read - count = %x \t *offs = %x\n", count, *offs);
	
//         int not_copied, to_copy = (count + *offs < echo->cur_task->recv_count) ? count : echo->cur_task->recv_count - *offs;

// 	printk("echo_read - to_copy = %x\n", to_copy);
// 	echo->cur_task->cur_state = FINISH;
//         not_copied = copy_to_user(user_buffer, echo->cur_task->recv_buffer, to_copy);
// 	dma_unmap_single(&echo->pdev->dev, echo->cur_task->recv_dma_addr, echo->cur_task->recv_count, DMA_TO_DEVICE);
// 	kfree(echo->cur_task->recv_buffer);
//         *offs += to_copy - not_copied;
//         return to_copy - not_copied;
// }

static int echo_mmap(struct file *file, struct vm_area_struct *vma)
{
	int status;
	struct echo_dev *echo = (struct echo_dev *) file->private_data;

	vma->vm_pgoff = pci_resource_start(echo->pdev, 1) >> PAGE_SHIFT;

	status = io_remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff, vma->vm_end
	- vma->vm_start, vma->vm_page_prot);
	if(status) {
		printk("echo_dev-drv - Error allocating device number\n");
		return -status;
	}
	return 0;
}

static long int echo_ioctl(struct file *file, unsigned cmd, unsigned long arg)
{
	struct echo_dev *echo = (struct echo_dev *) file->private_data;
	u32 val;

	switch(cmd) {
		case GET_ID:
			val = ioread32(echo->ptr_bar0 + 0x00);
			return copy_to_user((u32 *) arg, &val, sizeof(val));
		case GET_INV:
			val = ioread32(echo->ptr_bar0 + 0x04);
			return copy_to_user((u32 *) arg, &val, sizeof(val));
		case GET_RAND:
			val = ioread32(echo->ptr_bar0 + 0x0C);
			return copy_to_user((u32 *) arg, &val, sizeof(val));
		case SET_INV:
			if(0 != copy_from_user(&val, (u32 *) arg, sizeof(val)))
				return -EFAULT;
			iowrite32(val, echo->ptr_bar0 + 0x4);
			return 0;
		case IRQ:
			iowrite32(1, echo->ptr_bar0 + 0x8);
			return 0;
		case PROCESS:
                        iowrite32(1, echo->ptr_bar0 + 0x32);
                        return 0;
		default:
			return -EINVAL;
	}
}


/*
 * character device file operations for echo_device
 */
static loff_t control_cdev_llseek(struct file *file, loff_t off, int whence)
{
	loff_t newpos = 0;
	LOG_INFO("echo_llseek is called\n");
	switch (whence) {
	case 0: /* SEEK_SET */
		newpos = off;
		break;
	case 1: /* SEEK_CUR */
		newpos = file->f_pos + off;
		break;
	case 2: /* SEEK_END, @TODO should work from end of address space */
		newpos = UINT_MAX + off;
		break;
	default: /* can't happen */
		return -EINVAL;
	}
	if (newpos < 0)
		return -EINVAL;
	file->f_pos = newpos;
	return newpos;
}

static struct file_operations control_cdev_fops = {
	.owner = THIS_MODULE,
	.open = control_cdev_open,
	.write = control_cdev_write,
	.read = control_cdev_read,
	.llseek = control_cdev_llseek,
	// .mmap = echo_mmap,
	// .read = echo_read,
	// .write = echo_write,
	// .unlocked_ioctl = echo_ioctl,
	// .llseek = echo_llseek,
};


static struct file_operations event_cdev_fops = {
	.owner = THIS_MODULE,
	.open = event_cdev_open,
	.read = event_cdev_read,
	// .mmap = echo_mmap,
	// .read = echo_read,
	// .write = echo_write,
	// .unlocked_ioctl = echo_ioctl,
	
};


// static struct file_operations fops = {
// 	.owner = THIS_MODULE,
// 	.open = echo_open,
// 	.mmap = echo_mmap,
// 	.read = echo_read,
// 	.write = echo_write,
// 	.unlocked_ioctl = echo_ioctl,
// 	.llseek = echo_llseek,
// };

static struct pci_device_id echo_ids[] = {
	{PCI_DEVICE(VID, DID)},
	{},
};
MODULE_DEVICE_TABLE(pci, echo_ids);


/*
 * 负责自身初始化与外界无关
 */
static struct echo_dev* alloc_dev_instance(struct pci_dev *pdev)
{
    struct echo_dev *edev;
    if (!pdev) {
		LOG_ERR("Invalid pdev");
		return NULL;
    }
    edev = devm_kzalloc(&pdev->dev, sizeof(struct echo_dev), GFP_KERNEL);
    if (!edev) {
        LOG_ERR("OOM, xdma_dev.");
		return NULL;
    }
    // initialize echo_dev
    edev->pdev = pdev;
    edev->card_idx = -1;
	init_waitqueue_head(&(edev->event_wq));
    init_waitqueue_head(&(edev->write_wq));
    init_waitqueue_head(&(edev->irq_wq));
    // initialize echo_dev->ctrl_cdev
    edev->ctrl_cdev.pedev = edev;
	edev->ctrl_cdev.cdev.owner = THIS_MODULE;
	cdev_init(&(edev->ctrl_cdev.cdev), &control_cdev_fops);
	edev->ctrl_cdev.buffer = devm_kzalloc(&pdev->dev, RECV_BUFFER_SIZE, GFP_KERNEL);
    spin_lock_init(&(edev->ctrl_cdev.buffer_lock));
    // initialize echo_dev->event_cdev
    edev->event_cdev.pedev = edev;
    edev->event_cdev.event = 0;
	edev->event_cdev.cdev.owner = THIS_MODULE;
	cdev_init(&(edev->event_cdev.cdev), &event_cdev_fops);
    spin_lock_init(&(edev->event_cdev.event_lock));
    // initialize echo_dev->engine
    edev->engine.pedev = edev;
    edev->engine.cur_state = FINISH;
	edev->engine.send_count = SEND_BUFFER_SIZE;
	edev->engine.send_buffer = devm_kzalloc(&pdev->dev, edev->engine.send_count, GFP_KERNEL);
	edev->engine.recv_count = DEVICE_BAR_SIZE;
	edev->engine.recv_buffer = devm_kzalloc(&pdev->dev, edev->engine.recv_count, GFP_KERNEL);
	edev->engine.send_dma_addr = dma_map_single(&edev->pdev->dev, edev->engine.send_buffer, edev->engine.send_count, DMA_TO_DEVICE);
	edev->engine.recv_dma_addr = dma_map_single(&edev->pdev->dev, edev->engine.recv_buffer, edev->engine.recv_count, DMA_FROM_DEVICE);
	spin_lock_init(&(edev->engine.state_lock));
    INIT_WORK(&edev->engine.work, work_handler);

    return edev;
}



static void free_dev_instance(struct echo_dev* edev)
{
    if (!edev) {
		LOG_ERR("Invalid pdev");
		return NULL;
    }
    // Because devm_kzalloc function be used, fewer resources need to be released manually
    
    // free echo_dev->engine
	dma_unmap_single(&edev->pdev->dev, edev->engine.send_buffer, edev->engine.send_count, DMA_TO_DEVICE);
	dma_unmap_single(&edev->pdev->dev, edev->engine.recv_buffer, edev->engine.recv_count, DMA_FROM_DEVICE);
	// free echo_dev->event_cdev	Currently, no resources need to be manually released
	// free echo_dev->ctrl_cdev		Currently, no resources need to be manually released
	// free echo_dev				Currently, no resources need to be manually released
}

/*
 * 负责全局链表的维护
 * 负责设备号的维护
 * 负责当前设备pci使能/设置/io映射/中断处理函数注册/
 */
static int echo_device_open(struct echo_dev *echo)
{
    int status;
    struct pci_dev *pdev = echo->pdev;
    // 维护全局链表
    mutex_lock(&lock);
	echo->card_idx = card_count++;
	echo->ctrl_cdev.cdev_nr = MKDEV(DEVNR, minor_count++);
	echo->event_cdev.cdev_nr = MKDEV(DEVNR, minor_count++);
	list_add_tail(&echo->list, &card_list);
	mutex_unlock(&lock);
	LOG_INFO("card id = %d ", echo->card_idx);
	LOG_INFO("ctrl_cdev cdev_id  = %d ", echo->ctrl_cdev.cdev_nr);
	LOG_INFO("event_cdev cdev_id = %d ", echo->event_cdev.cdev_nr);
	
	status = pcim_enable_device(pdev);
	if(status != 0) {
		LOG_ERR("card %d - Error enabling device", echo->card_idx);
		goto fdev;
	}

	pci_set_master(pdev);

	echo->ptr_bar0 = pcim_iomap(pdev, 0, pci_resource_len(pdev, 0));
	if(!echo->ptr_bar0) {
		LOG_ERR("card %d - Error mapping BAR0",echo->card_idx);
		status = -ENODEV;
		goto fdev;
	}

	pci_set_drvdata(pdev, echo);

	status = pci_alloc_irq_vectors(pdev, 1, 1, PCI_IRQ_ALL_TYPES);
	if(status != 1) {
		LOG_ERR("card %d - Error alloc_irq returned %d", echo->card_idx,status);
		status = -ENODEV;
		goto fdev;
	}

	echo->irq_nr = pci_irq_vector(pdev, 0);
	LOG_INFO("card %d - IRQ Number: %d", echo->card_idx,echo->irq_nr);

	status = devm_request_irq(&pdev->dev, echo->irq_nr, echo_irq_handler, 0,
	"echo_dev-irq", echo);
	if(status != 0) {
		LOG_ERR("card %d - Error requesting interrupt",echo->card_idx);
		goto fdev;
	}
fdev:
    return status;
}


static void echo_device_close(struct echo_dev *echo)
{
	if (!echo) {
		LOG_ERR("Invalid pdev");
		return;
    }
    struct pci_dev *pdev = echo->pdev;
	pci_free_irq_vectors(pdev);
	// Because I'm using the pcim_iomap function, I don't need to release it manually
	// pci_iounmap(pdev, echo->ptr_bar0); // bug point
	pci_disable_device(pdev);
	// 维护全局链表
	mutex_lock(&lock);
	list_del(&echo->list);
	mutex_unlock(&lock);
    return;
}

/*
 * 负责创建用户接口 就是创建sysfs
 */
static int edev_create_interfaces(struct echo_dev *echo)
{
    int status;
    status = cdev_add(&(echo->ctrl_cdev.cdev), echo->ctrl_cdev.cdev_nr, 1);
	if(status < 0) {
		LOG_ERR("card %d - Error adding ctrl cdev", echo->card_idx);
		return status;
	}
	status = cdev_add(&(echo->event_cdev.cdev), echo->event_cdev.cdev_nr, 1);
	if(status < 0) {
		LOG_ERR("card %d - Error adding event cdev", echo->card_idx);
		return status;
	}
    return 0;
}



/*
 * 
 */
static void edev_destroy_interfaces(struct echo_dev *echo)
{
	if (!echo) {
		LOG_ERR("Invalid pdev");
		return;
    }
	cdev_del(&(echo->ctrl_cdev.cdev));
	LOG_INFO("Removing device with Device Number %d:%d",MAJOR(echo->ctrl_cdev.cdev_nr), MINOR(echo->ctrl_cdev.cdev_nr));
	cdev_del(&(echo->event_cdev.cdev));
	LOG_INFO("Removing device with Device Number %d:%d",MAJOR(echo->event_cdev.cdev_nr), MINOR(echo->event_cdev.cdev_nr));
    return;
}


static int echo_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	int status;
	struct echo_dev *echo;

	// echo = devm_kzalloc(&pdev->dev, sizeof(struct echo_dev), GFP_KERNEL);
	echo = alloc_dev_instance(pdev);
	if(!echo)
		return -ENOMEM;

	// mutex_lock(&lock);
	// echo->card_idx = card_count++;
	// echo->ctrl_cdev.cdev_nr = MKDEV(DEVNR, minor_count++);
	// echo->event_cdev.cdev_nr = MKDEV(DEVNR, minor_count++);
	// list_add_tail(&echo->list, &card_list);
	// mutex_unlock(&lock);
	// LOG_INFO("card id = %d ", echo->card_idx);
	// LOG_INFO("ctrl_cdev cdev_id  = %d ", echo->ctrl_cdev.cdev_nr);
	// LOG_INFO("event_cdev cdev_id = %d ", echo->event_cdev.cdev_nr);
	// status = cdev_add(&(echo->ctrl_cdev.cdev), echo->ctrl_cdev.cdev_nr, 1);
	// if(status < 0) {
	// 	LOG_ERR("card %d - Error adding ctrl cdev", echo->card_idx);
	// 	return status;
	// }
	// status = cdev_add(&(echo->event_cdev.cdev), echo->event_cdev.cdev_nr, 1);
	// if(status < 0) {
	// 	LOG_ERR("card %d - Error adding event cdev", echo->card_idx);
	// 	return status;
	// }
	// // init_waitqueue_head(&echo->event_wq);
	// // init_waitqueue_head(&echo->irq_wq);
	// // init_waitqueue_head(&echo->write_wq);

	// echo->pdev = pdev;

	// status = pcim_enable_device(pdev);
	// if(status != 0) {
	// 	LOG_ERR("card %d - Error enabling device", echo->card_idx);
	// 	goto fdev;
	// }

	// pci_set_master(pdev);

	// echo->ptr_bar0 = pcim_iomap(pdev, 0, pci_resource_len(pdev, 0));
	// if(!echo->ptr_bar0) {
	// 	LOG_ERR("card %d - Error mapping BAR0",echo->card_idx);
	// 	status = -ENODEV;
	// 	goto fdev;
	// }

	// pci_set_drvdata(pdev, echo);

	// status = pci_alloc_irq_vectors(pdev, 1, 1, PCI_IRQ_ALL_TYPES);
	// if(status != 1) {
	// 	LOG_ERR("card %d - Error alloc_irq returned %d", echo->card_idx,status);
	// 	status = -ENODEV;
	// 	goto fdev;
	// }

	// echo->irq_nr = pci_irq_vector(pdev, 0);
	// LOG_INFO("card %d - IRQ Number: %d\n", echo->card_idx,echo->irq_nr);

	// status = devm_request_irq(&pdev->dev, echo->irq_nr, echo_irq_handler, 0,
	// "echo_dev-irq", echo);
	// if(status != 0) {
	// 	LOG_ERR("card %d - Error requesting interrupt",echo->card_idx);
	// 	goto fdev;
	// }
	status = echo_device_open(echo);
	if (status) return status;
	status = edev_create_interfaces(echo);
	if (status) return status;
	test(echo);
	LOG_ERR("card %d - init finish", echo->card_idx);
	return 0;

fdev:
	/* Removing echo from list is missing */
	cdev_del(&echo->cdev);
	return status;

}

static void echo_remove(struct pci_dev *pdev)
{
	struct echo_dev *echo = (struct echo_dev *) pci_get_drvdata(pdev);
	if(echo) {
		LOG_INFO("Removing the device with card id =  %d",echo->card_idx);
		edev_destroy_interfaces(echo);
		echo_device_close(echo);
		free_dev_instance(echo);
	}
	pci_free_irq_vectors(pdev);
}

static struct pci_driver echo_driver = {
	.name = "echo_dev-driver",
	.probe = echo_probe,
	.remove = echo_remove,
	.id_table = echo_ids,
};

static int __init echo_init(void)
{
	int status;
	dev_t dev_nr = MKDEV(DEVNR, 0);

	status = register_chrdev_region(dev_nr, MINORMASK + 1, DEVNRNAME);
	if(status < 0) {
		LOG_ERR("echo_dev-drv - Error registering Device numbers");
		return status;
	}

	mutex_init(&lock);

	status = pci_register_driver(&echo_driver);
	if(status < 0) {
		LOG_ERR("echo_dev-drv - Error registering driver");
		unregister_chrdev_region(dev_nr, MINORMASK + 1);
		return status;
	}
	return 0;
}

static void __exit echo_exit(void)
{
	dev_t dev_nr = MKDEV(DEVNR, 0);
	unregister_chrdev_region(dev_nr, MINORMASK + 1);
	pci_unregister_driver(&echo_driver);
}

module_init(echo_init);
module_exit(echo_exit);

MODULE_LICENSE("GPL");
