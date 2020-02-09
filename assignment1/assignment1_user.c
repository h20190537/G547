#include "ioctl1.h"
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>             
#include <unistd.h>      
#include <sys/ioctl.h>   

//functions for the ioctl calls
int adc_set_channel(int file_desc, int channel)
{
    int ret_val;
    ret_val = ioctl(file_desc, SET_CHANNEL, channel);
    printf("Setting up the ADC channel\n");
    if (ret_val < 0) {
        printf("Adc_set_channel failed:%d\n", ret_val);
        exit(-1);
    }
    return 0;
}

int adc_set_alignment(int file_desc, int setting)
{
    int ret_val;
    ret_val = ioctl(file_desc, SET_ALIGNMENT, setting);
    printf("Setting up the ADC Alignment\n");	
    if (ret_val < 0) {
        printf("Adc_set_alignment failed:%d\n", ret_val);
        exit(-1);
    }
    return 0;
}

int main()
{
	unsigned short adc_value;
	int file_desc;
	int set_channel;
	int set_align = 3;
	int choice    = 3;
	int count;


	while(1)
	{	
		while(1)
		{	printf("\nOpen adc?(1->yes/0->no)---->\n");
			scanf("%d",&choice);
			while(getchar()!='\n');
			if(choice == 1)
			{	choice=3;
				file_desc = open(DEVICE_FILE_NAME,O_RDONLY);
				if(file_desc == -1)
				{
					printf("Cannot open the source file\n");
					exit(1);
				}
				break;
			}
			else if(choice == 0){
				choice = 3;
				continue;}
			else{
				printf("\nInvalid choice\n");			
				choice=3;}
		}

		while(1)
		{
			printf("\nChoose bit alignment---->\n0----->lower bits\n1----->Upper bits\n");
			scanf("%d",&set_align);
			while(getchar()!='\n');
			if(set_align == 1 || set_align==0)
				{	
					adc_set_alignment(file_desc,set_align);
					set_align = 3;					
					break;
				}
			else
					{printf("\nInvalid choice\n");	
					set_align = 3;}		
		}
		
		while(1)
		{
			printf("Enter the channel number(0-7) of adc:");
			scanf("%d",&set_channel);
			while(getchar()!='\n');
			if(set_channel > 7)
				printf("\nInvalid channel number-----\n\nSelect in between (0-7)\n");
			else
				{
					adc_set_channel(file_desc,set_channel);
					break;
				}
		}
	
		while(1)
		{
				printf("\nContinue to read?(1->yes/0->no)---->\n\n");
				scanf("%d",&choice);
				while(getchar()!='\n');
				if(choice == 1)
					{	choice = 3;
						count = read(file_desc,&adc_value,sizeof(adc_value));
						if(count>0)
							printf("\nADC reading:%d\n",adc_value);
						else
							printf("\nRead Unsuccessful\n");
					
					}	
				else if(choice == 0)
					{
						printf("\nClose adc?(1->yes/0->no)---->\n");
						scanf("%d",&choice);
						while(getchar()!='\n');
						if(choice ==1)
							{choice=3;
							close(file_desc);
							break;}

						else if(choice == 0)
							{
							choice = 3;
							continue;
							}
						else
							{
							choice = 3;
							printf("\nInvalid choice\n");
							}	
					}					

				else
					{
					choice = 3;
					printf("\nInvalid choice\n");
					}
		}
	}
	return 0;
}



