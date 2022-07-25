#include<sys/io.h>
#include<sys/file.h>
#include<unistd.h>
#include<stdlib.h> //
#include<stdio.h>
#include<stdint.h>
#include<nvml.h>
#define EC_IO_PATH "/sys/kernel/debug/ec/ec0/io"
#define MSR_DIRECTORY "/dev/cpu/0/msr"
#define IA32_PACKAGE_THERM_STATUS 433

static int fd_msr = -1;
static int fd_ec = -1;
static uint8_t cpu_reg[8];
static nvmlDevice_t d_handle;

typedef struct {
	uint8_t downThresh;
	uint8_t upThresh;
	uint8_t speed;
}tempProfile;

typedef struct {
	uint8_t fanSpeed_read;
	uint8_t fanSpeed_write;
}fanRegisters;

/* Configuration */
const static fanRegisters cpu_fan_registers = {19, 55};
const static fanRegisters gpu_fan_registers = {21, 58};
const static tempProfile cpu_temp_profile[] = {{43, 55, 0}, {44, 60, 30}, {50, 65, 45}, {56, 70, 50}, {63, 80, 65}, {68, 85, 75}, {78, 90, 100}};
const static uint8_t cpu_temp_profile_size = 7;
const static tempProfile gpu_temp_profile[] = {{43, 55, 0}, {44, 60, 30}, {50, 65, 45}, {56, 70, 50}, {63, 80, 65}, {68, 85, 75}, {78, 90, 100}};
const static uint8_t gpu_temp_profile_size = 7;
/* Configuration */

static uint8_t current_cpu_temp=0, current_gpu_temp=0;
static tempProfile current_cpu_temp_profile={43, 55, 0}, current_gpu_temp_profile={43, 55, 0};

void write_byte_to_ec(uint8_t reg_val, uint8_t *buf){
	int l = flock(fd_ec, LOCK_EX|LOCK_NB);
	while(l == -1){
		l = flock(fd_ec, LOCK_EX|LOCK_NB);
	}
	lseek(fd_ec, 0, SEEK_SET);
	lseek(fd_ec, reg_val, SEEK_SET);
	write(fd_ec, buf, 1);
	flock(fd_ec, LOCK_UN);
}

void read_byte_from_ec(uint8_t reg_val, uint8_t *buf){
	int l = flock(fd_ec, LOCK_EX|LOCK_NB);
        while(l == -1){
                l = flock(fd_ec, LOCK_EX|LOCK_NB);
        }
        lseek(fd_ec, 0, SEEK_SET);
        lseek(fd_ec, reg_val, SEEK_SET);
        read(fd_ec, buf, 1);
        flock(fd_ec, LOCK_UN);
}

void read_fan_rpm(uint8_t reg_val, uint16_t *buf){
	uint8_t p, q;
	read_byte_from_ec(reg_val, &p);
	read_byte_from_ec(reg_val+1, &q);
	*buf = (q<<8)|p;
}

/*temperature_getters*/
void read_cpu_temp(uint8_t *buf){
        lseek(fd_msr, 0, SEEK_SET);
        lseek(fd_msr, IA32_PACKAGE_THERM_STATUS, SEEK_SET);
        read(fd_msr, cpu_reg, 8);
        *buf = (cpu_reg[2])<<1;
	*buf = (*buf)>>1;
	*buf = 98-(*buf);
}

void read_gpu_temp(uint8_t *buf){
        unsigned int t;
        nvmlDeviceGetTemperature(d_handle, 0, &t);
        *buf = t;
}
/*temperature_getters*/

tempProfile search_temp_profile(tempProfile *inp, uint8_t size, uint8_t temp){

	int start = 0, end = size-1;
	if(temp<inp[0].downThresh){
		return inp[0];
	}
	if(temp>inp[size-1].upThresh){
		return inp[size-1];
	}
	while(start<=end && start>=0 && end<size){
		int m = start + (end-start)/2;
		if(inp[m].downThresh<=temp && inp[m].upThresh>=temp){
			int t = m;
			while(t>=0 && inp[t].upThresh>=temp){
				t--;
			}
			if(inp[t].upThresh == temp){
				return inp[t];
			}
			else{
				return inp[t+1];
			}
		}
		else if(temp>inp[m].upThresh){
			start = m+1;
		}
		else{
			end = m-1;
		}
	}

}

void initialize(){

	fd_ec = open(EC_IO_PATH, O_RDWR| O_EXCL);
        fd_msr = open(MSR_DIRECTORY, O_RDONLY);
        nvmlInit_v2();
        nvmlDeviceGetHandleByIndex_v2(0, &d_handle);

}

void de_initialize(){

	close(fd_ec);
	close(fd_msr);
	nvmlShutdown();

}

void start_program(){

	read_cpu_temp(&current_cpu_temp);
	read_gpu_temp(&current_gpu_temp);
	if(current_cpu_temp > current_cpu_temp_profile.upThresh || current_cpu_temp < current_cpu_temp_profile.downThresh){
		current_cpu_temp_profile = search_temp_profile(cpu_temp_profile, cpu_temp_profile_size, current_cpu_temp);
	}
	if(current_gpu_temp > current_gpu_temp_profile.upThresh || current_gpu_temp < current_gpu_temp_profile.downThresh){
                current_gpu_temp_profile = search_temp_profile(gpu_temp_profile, gpu_temp_profile_size, current_gpu_temp);
        }
	write_byte_to_ec(cpu_fan_registers.fanSpeed_write, current_cpu_temp_profile.speed);
	write_byte_to_ec(gpu_fan_registers.fanSpeed_write, current_gpu_temp_profile.speed);

}

int main(){

	initialize();
	while(1){
		start_program();
		read_fan_rpm(cpu_fan_registers.fanSpeed_read, &cs);
		printf("%d\n", cs);
		sleep(3);
	}
	return 0;


}
