Steps to compile the code:
1. Complie module using the make file.
2. Create a directory under /media folder.
3. Created block device is having 2 partition --> Mount the USB Disk1 partition using the below command
		sudo mount -t vfat /USB/DISK1 /media/<folder_name>
4. The mounted folder is having all the files that pendrive is having.
