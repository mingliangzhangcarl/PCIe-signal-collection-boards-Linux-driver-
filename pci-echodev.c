#include "qemu/osdep.h"
#include "qemu/units.h"
#include "hw/pci/pci.h"
#include "hw/hw.h"
#include "hw/pci/msi.h"
#include "qemu/timer.h"
#include "qom/object.h"
#include "qemu/main-loop.h" /* iothread mutex */
#include "qemu/module.h"
#include "qapi/visitor.h"
#include <unistd.h>

#define TYPE_PCI_CUSTOM_DEVICE "pci-echodev"

#define ID_REGISTER        				0x0   
#define INV_REGISTER       				0x4
#define IRQ_REGISTER       				0x8
#define RANDVAL_REGISTER   				0xc
#define DMA_SRC		   					0x10
#define DMA_DST		   					0x18
#define DMA_CNT            				0x20
#define DMA_CMD            				0x28
#define PROCESS_REGISTER   				0x32


#define HOST_TO_DEV_FRAME_LEN       	0x13
#define FRAME_HEAD_OFF	   				0x0
#define FRAME_COM_OFF	   				0x1
#define FRAME_TIME_OFF     				0x2
#define FRAME_FRE_START_OFF 			0x6
#define FRAME_FRE_END_OFF 				0xa
#define FRAME_END_OFF     				0x12


#define DEV_TO_HOST_FRAME_LEN 			0x15
typedef struct PciechodevState PciechodevState;

//This macro provides the instance type cast functions for a QOM type.
//DECLARE_INSTANCE_CHECKER(PciechodevState, PCIECHODEV, TYPE_PCI_CUSTOM_DEVICE)
#define PCIECHODEV(obj) \
        OBJECT_CHECK(PciechodevState, (obj), TYPE_PCI_CUSTOM_DEVICE)

	//struct defining/descring the state
	//of the custom pci device.
	struct PciechodevState {
		PCIDevice pdev;
		MemoryRegion mmio_bar0;
		MemoryRegion mmio_bar1;
		uint32_t bar0[16];
		uint8_t bar1[4096];
		struct dma_state {
			dma_addr_t src;
			dma_addr_t dst;
			dma_addr_t cnt;
			dma_addr_t cmd;
		} *dma;
	};

#define DMA_RUN 1
#define DMA_DIR(cmd) (((cmd) & 2) >> 1)
#define DMA_TO_DEVICE 0
#define DMA_FROM_DEVICE 1
#define DMA_DONE (1<<31)
#define DMA_ERROR (1<<30)

static int check_range(uint64_t addr, uint64_t cnt)
{
	uint64_t end = addr + cnt;
	if(end > 4*1024)
		return -1;
	return 0;
}


static void start_process(PciechodevState *pciechodev, hwaddr addr, unsigned size){
	pciechodev->bar0[PROCESS_REGISTER/4] = 0;    // clear
	pciechodev->bar0[PROCESS_REGISTER/4] |= 0x01;// bits[1:0] = [0 1] 
	for (hwaddr i = addr; i < addr+size;++i){
		printf("PCIECHODEV - start_process - addr = %lx\tdata=%x\n",i,pciechodev->bar1[i]);
	}

	if (size != HOST_TO_DEV_FRAME_LEN){
		goto parsing_failure;
	}
	if (pciechodev->bar1[addr + FRAME_HEAD_OFF] != 0xaa){
		printf("PCIECHODEV - start_process - head error\n");
		goto parsing_failure;
	}
	if (pciechodev->bar1[addr + FRAME_END_OFF] != 0xee){
		printf("PCIECHODEV - start_process - tail error\n");
		goto parsing_failure;
	}
	if (pciechodev->bar1[addr + FRAME_COM_OFF] == 0x01){
		uint32_t time;
		uint32_t fre0,fre1;
		uint32_t* ptr;
		ptr = (uint32_t *) &pciechodev->bar1[addr + FRAME_TIME_OFF];
		time = *ptr;
		ptr = (uint32_t *) &pciechodev->bar1[addr + FRAME_FRE_START_OFF];
			fre0 = *ptr;
		ptr = (uint32_t *) &pciechodev->bar1[addr + FRAME_FRE_END_OFF];
			fre1 = *ptr;
		printf("time = %d\n", time);
		printf("fre0 = %d\n", fre0);
		printf("fre1 = %d\n", fre1);
		if (time > 1000 || fre0 < 1e3 || fre1 > 6e3 || fre0 >= fre1){
			printf("PCIECHODEV - start_process - range error\n");
			goto parsing_failure;
		}
		uint8_t* uint8_ptr = (uint8_t*) &pciechodev->bar1[addr];
		uint32_t count = rand() % 10 + 1; // [50,100]
		for (int i = 0; i < count;++i){
			*uint8_ptr = 0xbb;
			uint8_ptr += 1;
			*uint8_ptr = 0x02;
			uint8_ptr += 1;
			*(uint16_t*)uint8_ptr = i;
			uint8_ptr += 2;
			*(uint64_t*)uint8_ptr = rand() % time;// toa
			uint8_ptr += 8; // toa
			*(uint32_t*)uint8_ptr = rand() % (fre1 - fre0 + 1) + fre0; // fre
			uint8_ptr += 4; // fre
			*(uint32_t*)uint8_ptr = rand() % 200; // pw
			uint8_ptr += 4; // fre
			*uint8_ptr = 0xdd;
			uint8_ptr++;
		}
		// for (int i = 0; i < count*DEV_TO_HOST_FRAME_LEN;++i){
		// 	printf("%d\n", pciechodev->bar1[i]);
		// }
		usleep(1000*time);
		pciechodev->bar0[PROCESS_REGISTER/4] = 0;
		pciechodev->bar0[PROCESS_REGISTER/4] = 0x3; // bit[3:0] = [0 0 1 1]
		pciechodev->dma->cnt = count*DEV_TO_HOST_FRAME_LEN;
		pciechodev->dma->dst = addr;
		// pciechodev->bar0[IRQ_REGISTER] |= (1 << 1);
		pciechodev->bar0[IRQ_REGISTER/4] = 2;
		printf("pciechodev->bar0[IRQ_REGISTER]  == = %lx\n",pciechodev->bar0[IRQ_REGISTER]);
		pci_set_irq(&pciechodev->pdev, 1);
		return;
		
	}
	
	
parsing_failure:
	pciechodev->bar0[PROCESS_REGISTER/4] = 0;
	pciechodev->bar0[PROCESS_REGISTER/4] |= 0x6; // set [3:0] = [0,1,1,0]
	//pciechodev->bar0[IRQ_REGISTER] |= (1 << 1);
	pciechodev->bar0[IRQ_REGISTER/4] = 2;	
pci_set_irq(&pciechodev->pdev, 1);
	return;
}


static void fire_dma(PciechodevState *pciechodev)
{
	struct dma_state *dma = pciechodev->dma;
	dma->cmd &= ~(DMA_DONE | DMA_ERROR);

	if(DMA_DIR(dma->cmd) == DMA_TO_DEVICE) {
		printf("PCIECHODEV - Transfer Data from RC to EP\n");
		printf("pci_dma_read: src: %lx, dst: %lx, cnt: %ld, cmd: %lx\n",
			dma->src, dma->dst, dma->cnt, dma->cmd);
		if(check_range(dma->dst, dma->cnt) == 0) {
			pci_dma_read(&pciechodev->pdev, dma->src,
			pciechodev->bar1 + dma->dst, dma->cnt);
		} else
			dma->cmd |= (DMA_ERROR);
	} else {
		printf("PCIECHODEV - Transfer Data from EP to RC\n");
		printf("pci_dma_write: src: %lx, dst: %lx, cnt: %ld, cmd: %lx\n",
			dma->src, dma->dst, dma->cnt, dma->cmd);
		if(check_range(dma->src, dma->cnt) == 0) {
			pci_dma_write(&pciechodev->pdev, dma->dst,
			pciechodev->bar1 + dma->src, dma->cnt);
		} else
			dma->cmd |= (DMA_ERROR);
	}

	dma->cmd &= ~(DMA_RUN);
	dma->cmd |= (DMA_DONE);
	pciechodev->bar0[IRQ_REGISTER/4] = 1;
	pci_set_irq(&pciechodev->pdev, 1);
}

static uint64_t pciechodev_bar0_mmio_read(void *opaque, hwaddr addr, unsigned size)
{
	PciechodevState *pciechodev = opaque;
	printf("PCIECHODEV: BAR0 pciechodev_mmio_read() addr %lx size %x \n", addr, size);
	if(addr == RANDVAL_REGISTER)
		return rand();

	return pciechodev->bar0[addr/4];
}

static void pciechodev_bar0_mmio_write(void *opaque, hwaddr addr, uint64_t val,
		unsigned size)
{
	printf("PCIECHODEV: BAR0 pciechodev_mmio_write() addr %lx size %x val %lx \n", addr, size, val);
	PciechodevState *pciechodev = opaque;

	if(addr >= 64)
		return;

	switch(addr) {
		case ID_REGISTER:
		case RANDVAL_REGISTER:
			/* 0 and 12 are read only */
			break;
		case INV_REGISTER:
			pciechodev->bar0[1] = ~val;
			break;
		case DMA_CMD:
			pciechodev->dma->cmd = val;
			if(val & DMA_RUN)
				fire_dma(pciechodev);
			break;
		case IRQ_REGISTER:
			if(val & 1){
				pci_set_irq(&pciechodev->pdev, 1);
				pciechodev->bar0[addr/4] = 1;
			}
			if (val & 0xffffffff){
				pci_set_irq(&pciechodev->pdev, 0);
				printf("hello\n");
				pciechodev->bar0[addr/4] = 0;
			}
			break;
		case PROCESS_REGISTER:
			if (val & 1) start_process(pciechodev,0,HOST_TO_DEV_FRAME_LEN);
			pciechodev->bar0[addr/4] = val;
			break;
		default:
			pciechodev->bar0[addr/4] = val;
			break;
	}
}

///ops for the Memory Region.
static const MemoryRegionOps pciechodev_bar0_mmio_ops = {
	.read = pciechodev_bar0_mmio_read,
	.write = pciechodev_bar0_mmio_write,
	.endianness = DEVICE_NATIVE_ENDIAN,
	.valid = {
		.min_access_size = 4,
		.max_access_size = 4,
	},
	.impl = {
		.min_access_size = 4,
		.max_access_size = 4,
	},

};

static uint64_t pciechodev_bar1_mmio_read(void *opaque, hwaddr addr, unsigned size)
{
	PciechodevState *pciechodev = opaque;
	printf("PCIECHODEV: BAR1 pciechodev_mmio_read() addr %lx size %x \n", addr, size);

	if(size == 1) {
		return pciechodev->bar1[addr];
	} else if(size == 2) {
		uint16_t *ptr = (uint16_t *) &pciechodev->bar1[addr];
		return *ptr;
	} else if(size == 4) {
		uint32_t *ptr = (uint32_t *) &pciechodev->bar1[addr];
		return *ptr;
	} else if(size == 8) {
		uint64_t *ptr = (uint64_t *) &pciechodev->bar1[addr];
		return *ptr;
	}
	return 0xffffffffffffffL;
}

static void pciechodev_bar1_mmio_write(void *opaque, hwaddr addr, uint64_t val,
		unsigned size)
{
	printf("PCIECHODEV: BAR1 pciechodev_mmio_read() addr %lx size %x val %lx \n", addr, size, val);
	PciechodevState *pciechodev = opaque;

	if(size == 1) {
		pciechodev->bar1[addr] = (uint8_t) val;
	} else if(size == 2) {
		uint16_t *ptr = (uint16_t *) &pciechodev->bar1[addr];
		*ptr = (uint16_t) val;
	} else if(size == 4) {
		uint32_t *ptr = (uint32_t *) &pciechodev->bar1[addr];
		*ptr = (uint32_t) val;
	} else if(size == 8) {
		uint64_t *ptr = (uint64_t *) &pciechodev->bar1[addr];
		*ptr = (uint64_t) val;
	}
}

///ops for the Memory Region.
static const MemoryRegionOps pciechodev_bar1_mmio_ops = {
	.read = pciechodev_bar1_mmio_read,
	.write = pciechodev_bar1_mmio_write,
	.endianness = DEVICE_NATIVE_ENDIAN,
	.valid = {
		.min_access_size = 1,
		.max_access_size = 8,
	},
	.impl = {
		.min_access_size = 1,
		.max_access_size = 8,
	},
};


//implementation of the realize function.
static void pci_pciechodev_realize(PCIDevice *pdev, Error **errp)
{
	PciechodevState *pciechodev = PCIECHODEV(pdev);
	uint8_t *pci_conf = pdev->config;

	pci_config_set_interrupt_pin(pci_conf, 1);

	///initial configuration of devices registers.
	memset(pciechodev->bar0, 0, 64);
	memset(pciechodev->bar1, 0, 4096);
	pciechodev->bar0[0] = 0xcafeaffe;
	pciechodev->dma = (struct dma_state *) &pciechodev->bar0[4];

	// Initialize an I/O memory region(pciechodev->mmio). 
	// Accesses to this region will cause the callbacks 
	// of the pciechodev_mmio_ops to be called.
	memory_region_init_io(&pciechodev->mmio_bar0, OBJECT(pciechodev), &pciechodev_bar0_mmio_ops, pciechodev, "pciechodev-mmio", 64);
	// registering the pdev and all of the above configuration 
	// (actually filling a PCI-IO region with our configuration.
	pci_register_bar(pdev, 0, PCI_BASE_ADDRESS_SPACE_MEMORY, &pciechodev->mmio_bar0);
	/* BAR 1 */
	memory_region_init_io(&pciechodev->mmio_bar1, OBJECT(pciechodev), &pciechodev_bar1_mmio_ops, pciechodev, "pciechodev-mmio", 4096);
	pci_register_bar(pdev, 1, PCI_BASE_ADDRESS_SPACE_MEMORY, &pciechodev->mmio_bar1);
}

// uninitializing functions performed.
static void pci_pciechodev_uninit(PCIDevice *pdev)
{
	return;
}


///initialization of the device
static void pciechodev_instance_init(Object *obj)
{
	return ;
}

static void pciechodev_class_init(ObjectClass *class, void *data)
{
	DeviceClass *dc = DEVICE_CLASS(class);
	PCIDeviceClass *k = PCI_DEVICE_CLASS(class);

	//definition of realize func().
	k->realize = pci_pciechodev_realize;
	//definition of uninit func().
	k->exit = pci_pciechodev_uninit;
	k->vendor_id = PCI_VENDOR_ID_QEMU;
	k->device_id = 0xbeef; //our device id, 'beef' hexadecimal
	k->revision = 0x10;
	k->class_id = PCI_CLASS_OTHERS;

	/**
	 * set_bit - Set a bit in memory
	 * @nr: the bit to set
	 * @addr: the address to start counting from
	 */
	set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static void pci_custom_device_register_types(void)
{
	static InterfaceInfo interfaces[] = {
		{ INTERFACE_CONVENTIONAL_PCI_DEVICE },
		{ },
	};
	static const TypeInfo custom_pci_device_info = {
		.name          = TYPE_PCI_CUSTOM_DEVICE,
		.parent        = TYPE_PCI_DEVICE,
		.instance_size = sizeof(PciechodevState),
		.instance_init = pciechodev_instance_init,
		.class_init    = pciechodev_class_init,
		.interfaces = interfaces,
	};
	//registers the new type.
	type_register_static(&custom_pci_device_info);
}

type_init(pci_custom_device_register_types)
