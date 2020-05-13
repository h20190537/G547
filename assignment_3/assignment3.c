#include<linux/kernel.h>
#include<linux/init.h>
#include<linux/module.h>
#include<linux/usb.h>
#include<linux/slab.h>
#include<linux/string.h>
#include<linux/blkdev.h>
#include<linux/genhd.h>
#include<linux/spinlock.h>

//USB macros
#define RETRY_MAX 3
#define READ_CAPACITY_LENGTH 0x08
#define be_to_int32(buf) (((buf)[0]<<24)|((buf)[1]<<16)|((buf)[2]<<8)|(buf)[3])
#define ENDPOINT_IN 0x80
#define ENDPOINT_OUT 0x00
#define READ_CAPACITY_LEN 0x08

//block macros
#define SECTOR_SIZE 512
#define nsectors 15633408

//pendrive 1
#define SANDISK_A_VID 0x0781 //assigned by the USB organization(unique id)
#define SANDISK_A_PID 0x5567

//pendrive 2
#define SANDISK_B_VID 0x0781 //assigned by the USB organization(unique id)
#define SANDISK_B_PID 0x5590

//usb variables
static unsigned long user_block_size;
static uint8_t ep_in = 0;
static uint8_t ep_out = 0; 
static struct usb_device *device;
static uint8_t scsi_support_flag;

//block variables
int block_reg_flag =0;
int usb_major =0; //will contain the allocated minor number

struct usb_dev{ //structure defining the usb block device
	struct request_queue *queue; //pointer to the request queue of the device
	struct gendisk *gd;//pointer to the gendisk structure of the device
	spinlock_t lock;//lock for mutual exclusion
	struct workqueue_struct *dev_wq;//workqueue member for handling request
	int users;//to keep track of number of users
};

struct dev_work {
	struct work_struct work;
	struct request *req;
};

struct request *req;
static struct usb_dev *p_blkdev = NULL;

//usb structure
struct command_block_wrapper {
	uint8_t dCBWSignature[4];
	uint32_t dCBWTag;
	uint32_t dCBWDataTransferLength;
	uint8_t bmCBWFlags;
	uint8_t bCBWLUN;
	uint8_t bCBWCBLength;
	uint8_t CBWCB[16];
};

struct command_status_wrapper {
	uint8_t dCSWSignature[4];
	uint32_t dCSWTag;
	uint32_t dCSWDataResidue;
	uint8_t bCSWStatus;
};

static uint8_t cdb_length[256] = {
//	 0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F
	06,06,06,06,06,06,06,06,06,06,06,06,06,06,06,06,  //  0
	06,06,06,06,06,06,06,06,06,06,06,06,06,06,06,06,  //  1
	10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,  //  2
	10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,  //  3
	10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,  //  4
	10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,  //  5
	00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,  //  6
	00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,  //  7
	16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,  //  8
	16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,  //  9
	12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,  //  A
	12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,  //  B
	00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,  //  C
	00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,  //  D
	00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,  //  E
	00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,  //  F
};


static void blkdev_release(struct gendisk*, fmode_t);
static int blkdev_open(struct block_device*, fmode_t);
static int block_registration(void);
//-------------------------------------------------------USB functions-----------------------------------------
static struct usb_device_id usbdev_table[] = {
	{USB_DEVICE(SANDISK_A_VID,SANDISK_A_PID)},
	{USB_DEVICE(SANDISK_B_VID,SANDISK_B_PID)},
	{}
};

static void usbdev_disconnect(struct usb_interface *interface)
{
	printk(KERN_INFO "USBDEV Device Removed\n");
	return;
}

static int get_status(struct usb_device *device, uint8_t endpoint, uint32_t expected_tag)
{
	int retry, retval, size;
	struct command_status_wrapper *csw;
	if ( !(csw = (struct command_status_wrapper *)kmalloc(sizeof(struct command_status_wrapper), GFP_KERNEL)) ) {
		printk(KERN_ERR "Memory allocation to csw failed\n");
		return -1;
	}
	retry = 0;
	do {	//reading csw of exact 13 bytes
		retval = usb_bulk_msg(device,usb_rcvbulkpipe(device, endpoint),(unsigned char*)csw,13, &size, 2000);
		if (retval != 0 ) 
			usb_clear_halt(device,usb_rcvbulkpipe(device, endpoint));//use to clear stall/halt in bulk endpoint.
		retry++;
	} while ((retry<RETRY_MAX)&&(retval!=0));
		
	if (size != 13) {
		printk(KERN_ERR"   get_mass_storage_status: received %d bytes (expected 13)\n", size);
		return -1;
	}
	
	if (csw->dCSWTag != expected_tag) {
		printk(KERN_ERR"   get_mass_storage_status: mismatched tags (expected %d, received %d)\n",
			expected_tag, csw->dCSWTag);
		return -1;
	}
kfree(csw);
return 0;
}

static int send_mass_storage_command(struct usb_device *device, uint8_t endpoint, uint8_t lun,
	uint8_t *cdb, uint8_t direction, int data_length, uint32_t *ret_tag)
{
        static uint32_t tag = 1;
	uint8_t cdb_len;
	int i, r, size;
	struct command_block_wrapper *cbw;

	if ( !(cbw = (struct command_block_wrapper *)kmalloc(sizeof(struct command_block_wrapper), GFP_KERNEL)) ) {
		printk(KERN_ERR "Memory allocation to cbw failed\n");
		return -1;
	}

	if (cdb == NULL) {
		return -1;
	}

	if (endpoint & ENDPOINT_IN) {
		printk(KERN_ERR "send_mass_storage_command: cannot send command on IN endpoint\n");
		return -1;
	}

	cdb_len = cdb_length[cdb[0]];

	memset(cbw, 0, sizeof(struct command_block_wrapper));
	cbw->dCBWSignature[0] = 'U';
	cbw->dCBWSignature[1] = 'S';
	cbw->dCBWSignature[2] = 'B';
	cbw->dCBWSignature[3] = 'C';
	*ret_tag = tag;
	cbw->dCBWTag = tag++;
	cbw->dCBWDataTransferLength = data_length;
	cbw->bmCBWFlags = direction;
	cbw->bCBWLUN = lun;
	// Subclass is 1 or 6 => cdb_len
	cbw->bCBWCBLength = cdb_len;//defines the valid length of cdb
	memcpy(cbw->CBWCB, cdb, cdb_len);
	//usb_reset_device(device);
	i = 0;
	do {
		// The transfer length must always be exactly 31 bytes.
		r = usb_bulk_msg(device,usb_sndbulkpipe(device, endpoint), (unsigned char*)cbw, 31, &size, 2000);
		if (r != 0) {
			usb_clear_halt(device,usb_sndbulkpipe(device, endpoint));//use to clear stall/halt in bulk endpoint.		
		}
		i++;
	    } while ((i<RETRY_MAX)&&(r !=0));

	if (r != 0) {
		printk(KERN_INFO" Command send failure\n");
		return -1;
	}

	printk(KERN_INFO"   sent %d CDB bytes\n", cdb_len);
	kfree(cbw);	
	return 0;
}

static int read_capacity_command(void)
{
	int retval,count;
	uint32_t expected_tag;
	uint8_t lun = 0;//logical device number
	unsigned long max_lba;
	unsigned long device_size;
	uint8_t cdb[16];	// SCSI Command Descriptor Block
	uint8_t *buffer;

	if ( !(buffer = (uint8_t *)kmalloc(sizeof(uint8_t)*64, GFP_KERNEL)) ) {	
		printk(KERN_ERR "Memory allocation to buffer failed\n");
		return -1;
	}
	
	//read capacity	
	printk(KERN_INFO"Reading Capacity:\n");
	memset(buffer, 0, sizeof(uint8_t)*64);
	memset(cdb, 0, sizeof(cdb));
	cdb[0] = 0x25;	// Read Capacity

	retval = send_mass_storage_command(device, ep_out, lun, cdb, ENDPOINT_IN, READ_CAPACITY_LEN, &expected_tag);
	
	if(retval < 0){
		printk(KERN_ERR"READ Capacity command failed\n");
		return -1;
	}

	retval = usb_bulk_msg(device,usb_rcvbulkpipe(device, ep_in),(unsigned char*)buffer,READ_CAPACITY_LENGTH, &count, 2000);	
	if(retval < 0){
		printk(KERN_ERR"Error in receiving data\n");	
		return -1;
	}	

	max_lba =  be_to_int32(&buffer[0]);
	printk(KERN_INFO "No. of logic blocks : %ld\n",(max_lba+1));
	user_block_size = be_to_int32(&buffer[4]);
	printk(KERN_INFO "block size : %ld \n",user_block_size);
	device_size = ((max_lba+1)*user_block_size/(1024*1024*1024));
	printk(KERN_INFO "Pendrive size : %ld\n",device_size);
	
	if (get_status(device, ep_in, expected_tag) == -1) {
		printk(KERN_ERR"Error in reading status\n");
		return -1;	
	}
	
	kfree(buffer);
	return 0;	
}

static int read_command(sector_t start,uint16_t block_num, uint8_t *mem_buffer)
{

	int retval,count;
	uint32_t expected_tag;
	uint8_t lun = 0;//logical device number
	uint8_t cdb[16];// SCSI Command Descriptor Block

	memset(cdb, 0, sizeof(cdb));
	cdb[0] = 0x28;		// Read(10)
	cdb[2] = (start & 0xff000000)>>24;//MSB	
	cdb[3] = (start & 0x00ff0000)>>16;
	cdb[4] = (start & 0x0000ff00)>>8;
	cdb[5] = (start & 0x000000ff)>>0;//LSB	
	cdb[7] = (block_num & 0xff00)>>8;// defines the number of block to be read(MSB)
	cdb[8] = (block_num & 0x00ff)>>0;//(LSB) 

	retval = send_mass_storage_command(device, ep_out, lun, cdb, ENDPOINT_IN, block_num*SECTOR_SIZE, &expected_tag);
		
	if(retval < 0){
		printk(KERN_ERR"READ Capacity command failed\n");
		return -1;
	}

	retval = usb_bulk_msg(device,usb_rcvbulkpipe(device, ep_in),(unsigned char*)mem_buffer,block_num*SECTOR_SIZE, &count, 2000);
	
	if(retval < 0){
		printk(KERN_ERR"Error in reading data\n");	
		return -1;
	}

	printk(KERN_INFO"   READ: received %d bytes\n", count);
	if (get_status(device, ep_in, expected_tag) == -1) {
		printk(KERN_ERR"Error in reading status\n");
		return -1;	
	}

	//display_buffer_hex(mem_buffer,count);
	return 0;
}

static int write_command(sector_t start,uint16_t block_num, uint8_t *mem_buffer)
{
	int retval,count;
	uint32_t expected_tag;
	uint8_t lun = 0;//logical device number
	uint8_t cdb[16];// SCSI Command Descriptor Block

	memset(cdb, 0, sizeof(cdb));
	cdb[0] = 0x2A;		// Write(10)
	cdb[2] = (start & 0xff000000)>>24;//MSB	
	cdb[3] = (start & 0x00ff0000)>>16;
	cdb[4] = (start & 0x0000ff00)>>8;
	cdb[5] = (start & 0x000000ff)>>0;//LSB	
	cdb[7] = (block_num & 0xff00)>>8;// defines the number of block to be read(MSB)
	cdb[8] = (block_num & 0x00ff)>>0;//(LSB) 

	retval = send_mass_storage_command(device, ep_out, lun, cdb, ENDPOINT_OUT, block_num*user_block_size, &expected_tag);
		
	if(retval < 0){
		printk(KERN_ERR"Write command failed\n");
		return -1;
	}

	retval = usb_bulk_msg(device,usb_sndbulkpipe(device, ep_out),(unsigned char*)mem_buffer,block_num*user_block_size, &count, 2000);
	
	if(retval < 0){
		printk(KERN_ERR"Error in writing data\n");	
		return -1;
	}

	printk(KERN_INFO"   Write: Ent %d bytes\n", count);
	if (get_status(device, ep_in, expected_tag) == -1) {
		printk(KERN_ERR"Error in reading status\n");
		return -1;	
	}

	return 0;

}

static int usbdev_probe(struct usb_interface *interface, const struct usb_device_id *id)
{
	int i;
	uint8_t epAddr,epAttr;
	
	struct usb_endpoint_descriptor *ep_desc;
	
	printk(KERN_INFO "USB device pid : %x\n", id->idProduct);
	printk(KERN_INFO "USB device vid : %x\n", id->idVendor);
	
	device = interface_to_usbdev(interface); //retriving the usb_device structure from the interface descriptor.
	printk(KERN_INFO "USB_DEVICE_CLASS : %x\n",interface->cur_altsetting->desc.bInterfaceClass);
	printk(KERN_INFO "USB_DEVICE_SUBCLASS : %x\n", interface->cur_altsetting->desc.bInterfaceSubClass);
	printk(KERN_INFO "USB_DEVICE_PROTOCOL : %x\n",interface->cur_altsetting->desc.bInterfaceProtocol);
	//match the vid and pid of the connected USB device	
	if(id->idProduct == SANDISK_A_PID && id->idVendor == SANDISK_A_VID)
	{
		printk(KERN_INFO "cruzer blade plugged in....known device.\n");
	} 
	
	if(id->idProduct == SANDISK_B_PID && id->idVendor == SANDISK_B_VID)
	{
		printk(KERN_INFO "Sandisk plugged in known device.\n");
	} 
	
	printk(KERN_INFO "No. of Altsettings : %d\n",interface->num_altsetting);
	printk(KERN_INFO "No. of Endpoints : %d\n",interface->cur_altsetting->desc.bNumEndpoints);
	//nNumEndpoints doesn't count the Oth endpoint.
	//loop through all the endpoints and find out their type and direction
	for(i=0;i<interface->cur_altsetting->desc.bNumEndpoints;i++)
	{
		ep_desc = &interface->cur_altsetting->endpoint[i].desc;
		epAddr = ep_desc->bEndpointAddress;
		epAttr = ep_desc->bmAttributes;
		if((epAttr & USB_ENDPOINT_XFERTYPE_MASK)==USB_ENDPOINT_XFER_BULK)//mask for checking the bulk type endpoint.
		{
			if(epAddr & 0x80)
			{
				printk(KERN_INFO "EP %d is Bulk IN\n",i);
				ep_in = epAddr;		//storing the address of the bulk in endpoint	
			}		
				else
			{
				printk(KERN_INFO "EP %d is Bulk OUT\n",i);
				ep_out = epAddr;		//storing the address of the bulk out endpoint	
			}		
		}
	}
//-----------------------check for the SCSI command support--------------------------------	
	if(interface->cur_altsetting->desc.bInterfaceClass == 0x08 && interface->cur_altsetting->desc.bInterfaceSubClass == 0x06 && interface->cur_altsetting->desc.bInterfaceProtocol== 0x50)
	{
		printk(KERN_INFO "SCSI commands supported.....\n");
		scsi_support_flag = 1;
		if(read_capacity_command()<0)
			printk(KERN_ERR "Read capacity command failed\n");				
		
	}
	else
	{	
		printk(KERN_INFO "SCSI commands not supported");
		return -1;
	}	
	if(block_reg_flag == 0)
	{
		if(block_registration()<0)
		{
			printk(KERN_INFO "Block registration failed");
			return -1;
		}
		else
		{
			printk(KERN_ALERT"Block device registered\n");
			block_reg_flag =1;	
		}	
	}
	else
		printk("Block device already registered\n");	
	return 0;	
}

//usb operation block
static struct usb_driver usbdev_driver = {
	.name  = "usbdev",  //name of the device
	.probe = usbdev_probe, // Whenever Device is plugged in
	.disconnect = usbdev_disconnect, // When we remove a device
	.id_table = usbdev_table, //  List of devices served by this driver
};




//-------------------------------block functions--------------------------------------------

static int blkdev_open(struct block_device *device, fmode_t mode)
{
	printk(KERN_INFO "Block device open\n");
	return 0;
}

static void blkdev_release(struct gendisk *gd, fmode_t mode)
{

	printk(KERN_INFO "Block device release\n");

}

int process_data(struct request *req)
{
	int ret = 0;
	int direction = rq_data_dir(req);
	unsigned int total_sectors = blk_rq_sectors(req); // number of sectors to process

	struct bio_vec bvec;
	struct req_iterator iter;
	sector_t sector_offset = 0;
	unsigned int bvec_sectors;
	uint8_t *buffer = NULL;	
	uint8_t *local_buffer = NULL;

	rq_for_each_segment(bvec,req,iter) {
	 	local_buffer = (uint8_t*)kmalloc(total_sectors * SECTOR_SIZE,GFP_ATOMIC);
		if(local_buffer == NULL) {
			printk(KERN_ERR "Cannot allocate memoory to local buffer\n");
			return -1;
		}

		bvec_sectors = bvec.bv_len / SECTOR_SIZE; 

		if (direction == 0 ) {
			printk(KERN_INFO "Read Request\n");

			if(read_command(iter.iter.bi_sector, bvec_sectors, local_buffer) == 0)
				printk(KERN_INFO "Read Success\n"); 
			else
				printk(KERN_INFO "Read Failure\n");
			
			buffer = __bio_kmap_atomic(iter.bio, iter.iter);
			memcpy(buffer,local_buffer,bvec_sectors*SECTOR_SIZE);
			__bio_kunmap_atomic(buffer);
			kfree(local_buffer); 				
		}
		else{
			printk(KERN_INFO "Write request\n");
			buffer = __bio_kmap_atomic(iter.bio, iter.iter);
			if(write_command(iter.iter.bi_sector, bvec_sectors, buffer) == 0)
				printk(KERN_INFO "Write Success\n"); 
			else
				printk(KERN_INFO "Write Failure\n");
			__bio_kunmap_atomic(buffer);		
			}
		sector_offset += bvec_sectors; 
	}

	if ( sector_offset == total_sectors) 
		ret = 0;
	else {
		printk(KERN_INFO "Sectors remaining\n");
		ret = -1;
	}
	return ret;
}

void request_bottom_half(struct work_struct *work)
{
	struct dev_work *usb_work = NULL;
	struct request *req = NULL;
	unsigned long flags; 				
	int rval =0;

	usb_work = container_of(work, struct dev_work, work); 		// retriving my dev_work struct
	req = usb_work->req;

	rval = process_data(req);

	spin_lock_irqsave( &p_blkdev->lock, flags); 		
	__blk_end_request_all(req,rval);
	spin_unlock_irqrestore( &p_blkdev->lock, flags);

	kfree(usb_work);
	return;
}

void usb_request(struct request_queue *q)
{
	struct request *req;
	struct dev_work *work = NULL;

	while( (req = blk_fetch_request(q)) != NULL ) {
		if(req->cmd_type != REQ_TYPE_FS)//non fs request should be ended.
		{
			printk(KERN_NOTICE "Skip non fs request\n");
			__blk_end_request_all(req,0);
			req = blk_fetch_request(q);			
			continue;		
		} 
		work = (struct dev_work*)kmalloc(sizeof(struct dev_work), GFP_ATOMIC);  
		if ( work == NULL ) {
			printk("Memory allocation for deferred work failed\n");
			blk_end_request_all(req, 0); 		
			continue;
		}
		printk(KERN_INFO"Request handled\n");
		
		work->req = req;
		INIT_WORK(&work->work, request_bottom_half);
		queue_work( p_blkdev->dev_wq, &work->work);
	}

}

//block operation block
static struct block_device_operations blkdev_ops =
{
	owner: THIS_MODULE,
	open: blkdev_open,
	release: blkdev_release,
};

static int block_registration(void)
{
	//block device registration
	usb_major = register_blkdev(0, "USB DISK");
	if (usb_major < 0) 
		printk(KERN_WARNING "Unable to allocate major number for the device\n");
	
	p_blkdev = kmalloc(sizeof(struct usb_dev),GFP_KERNEL);
	if(!p_blkdev)
	{
		printk("Unable to allocate memory for the private block device structure\n");
		return -1;
	}
	
	memset(p_blkdev, 0, sizeof(struct usb_dev));
	spin_lock_init(&p_blkdev->lock);//used by the request queue
	
	p_blkdev->queue = blk_init_queue(usb_request, &p_blkdev->lock);//initializing the request queue and kernel will hold the lock when it calls this request function
	if(!p_blkdev->queue)
	{
		printk("Unable to allocate memory for the request queue\n");
		return -1;
	}

	p_blkdev->gd = alloc_disk(2);//allocating the gendisk structure having only one minor number(no partition).
	if(!p_blkdev->gd)
	{
		unregister_blkdev(usb_major, "USB DISK");
		blk_cleanup_queue(p_blkdev->queue);
		kfree(p_blkdev);
		printk(KERN_INFO "Disk allocation failure.\n");
		return -1;
	}
	
	p_blkdev->dev_wq = create_workqueue("Myqueue");	//workqueue for deffering the work
	
	p_blkdev->gd->major = usb_major;
	p_blkdev->gd->first_minor = 0;
	p_blkdev->gd->fops = &blkdev_ops;
	p_blkdev->gd->queue = p_blkdev->queue;
	p_blkdev->gd->private_data = p_blkdev;
	strcpy(p_blkdev->gd->disk_name, "USB DISK");
	set_capacity(p_blkdev->gd,nsectors); 				// in terms of number of sectors
	add_disk(p_blkdev->gd);
	printk(KERN_INFO "Registered disk\n");
	return 0;
}

int device_init(void)
{	int result;
	//usb registration
	result = usb_register(&usbdev_driver);
	if(result)
		printk(KERN_ALERT "USB driver registration failed\n");
	else
		printk(KERN_ALERT"USB device driver registered\n");
	return 0;
}

void device_exit(void)
{
	usb_deregister(&usbdev_driver);
	if(block_reg_flag ==1){
	block_reg_flag = 0;
	del_gendisk(p_blkdev->gd);
	flush_workqueue(p_blkdev->dev_wq);
	destroy_workqueue(p_blkdev->dev_wq);
	blk_cleanup_queue(p_blkdev->queue);
	kfree(p_blkdev);
	unregister_blkdev(usb_major, "USB DISK");
	}
	printk(KERN_NOTICE "Leaving Kernel\n");
}

module_init(device_init);
module_exit(device_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("SUDHANSHU");
MODULE_DESCRIPTION("USB block driver");

