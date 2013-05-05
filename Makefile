SOURCE_PATH=/home/liu/workspace/linux-3.0.1
EXTRA_CFLAGS=-g
obj-m:= wm9714.o
wm9714-objs:=main.o pcm.o
build:kernel_modules
kernel_modules:
	make -C $(SOURCE_PATH) M=$(PWD) modules
clean:
	make -C $(SOURCE_PATH) M=$(PWD) clean
