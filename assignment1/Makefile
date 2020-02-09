obj-m := assignment1.o

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules
	gcc assignment1_user.c -o calluser
clean:
	rm -rf *.o *.mod.c *.symvers *.order *.ko
