#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <pthread.h>


#include "echodev-cmd.h"


#define FRAME_LENGTH       	0xe


// 互斥锁和条件变量
pthread_mutex_t mutex;
pthread_cond_t cond;
int32_t count;
void *fileReader(void *arg) {
    char *filename = (char *)arg;
    int fd = open(filename, O_RDWR);
    if (!fd) {
        perror("Error opening file");
        pthread_exit(NULL);
    }

    char buffer[4];

    while (1) {
        read(fd, buffer,4);
        // 加锁
        pthread_mutex_lock(&mutex);
        // 将读取结果转发给主线程
        // ...
        count = *((int32_t*) buffer);
        printf("count = %d\n",count);
        // 唤醒主线程
        pthread_cond_signal(&cond);
        // 解锁
        pthread_mutex_unlock(&mutex);
    }

    fclose(fd);
    pthread_exit(NULL);
}




int main(int argc, char **argv)
{
	int fd, status;
	uint32_t value;

    if(argc != 3) {
            printf("Usage: %s <ctrlFile> <eventFile>\n", argv[0]);
            return 0;
    }

    // 初始化互斥锁和条件变量
    pthread_mutex_init(&mutex, NULL);
    pthread_cond_init(&cond, NULL);

    pthread_t thread;
    int ret = pthread_create(&thread, NULL, fileReader, argv[2]);
    if (ret) {
        perror("Error creating thread");
        exit(EXIT_FAILURE);
    }

    fd = open(argv[1], O_RDWR);
    if(fd < 0) {
            perror("open");
            return fd;
    }

    

    uint8_t databuf[FRAME_LENGTH] = { 0xaa, 0x64, 00, 00, 00, 0, 0x10, 00, 00, 0x10, 0x10, 00, 00, 0xee };

    status = write(fd, databuf, FRAME_LENGTH);
    if(status < 0){
        printf("write error!\r\n");
        close(fd);
        return -1;
    }
	// 加锁
    pthread_mutex_lock(&mutex);

    // 等待子线程唤醒
    pthread_cond_wait(&cond, &mutex);
    // 处理结果
    uint8_t* recv_buffer = malloc(count);
    memset(recv_buffer,0,count);
    status = read(fd, recv_buffer, count);
    if(status < 0){
        printf("read error!\r\n");
        close(fd);
        return -1;
    }
    // 解锁
    pthread_mutex_unlock(&mutex);
    uint32_t i = 0;
	uint32_t* ptr;
	while( i < status){
		if (recv_buffer[i] == 0xbb && recv_buffer[i+FRAME_LENGTH - 1] == 0xdd){
			i++;
			ptr = (uint32_t*) &recv_buffer[i];
			printf("toa = %d\t", *ptr);
			i += 4;
			ptr = (uint32_t*) &recv_buffer[i];
			printf("cf =  %d\t", *ptr);
			i += 4;
			ptr = (uint32_t*) &recv_buffer[i];
			printf("pw =  %d\t", *ptr);
			i += 4;
			printf("\n");
		}else{
			i++;
		}
	}

	// uint8_t* recv_buffer = malloc(1200);
	// memset(recv_buffer,0,1200);
	// status = lseek(fd, 0, SEEK_SET);
	// if(status < 0){
        //             printf("lseek_error error!\r\n");
        //             close(fd);
        //             return -1;
        // }
	// status = read(fd, recv_buffer, 1200);
        // if(status < 0){
        //             printf("read error!\r\n");
        //             close(fd);
        //             return -1;
        // }
	// printf("status = %x\n",status);
	// uint32_t i = 0;
	// uint32_t* ptr;
	// while( i < status){
	// 	if (recv_buffer[i] == 0xbb && recv_buffer[i+FRAME_LENGTH - 1] == 0xdd){
	// 		i++;
	// 		ptr = (uint32_t*) &recv_buffer[i];
	// 		printf("toa = %d\t", *ptr);
	// 		i += 4;
	// 		ptr = (uint32_t*) &recv_buffer[i];
	// 		printf("cf =  %d\t", *ptr);
	// 		i += 4;
	// 		ptr = (uint32_t*) &recv_buffer[i];
	// 		printf("pw =  %d\t", *ptr);
	// 		i += 4;
	// 		printf("\n");
	// 	}else{
	// 		i++;
	// 	}
	// }
	close(fd);
	return 0;
}
