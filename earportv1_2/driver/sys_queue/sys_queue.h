#ifndef APP_QUEUE_H__
#define APP_QUEUE_H__

#include "stdint.h"
#include "stdbool.h"

#define QUEUE_TYPE_BLE    1
#define QUEUE_TYPE_SPI    2
#define MAX_BUF           144  //每个buff内存储144字节
#define QUEUE_MAX         40   //每个队列内buff个数

typedef struct {
	int length;
  uint8_t cmd[MAX_BUF];
}queue_element_t;

typedef struct {
    queue_element_t data[QUEUE_MAX];
    int front;
    int rear;
    int size;
} queue_list_t;

void queue_init(queue_list_t* queue);
bool queue_full(queue_list_t* queue);
bool queue_empty(queue_list_t* queue);
void queue_in(queue_list_t* queue, uint8_t* item, int length);
int  queue_out(queue_list_t* queue, uint8_t* item);
int  queue_size(queue_list_t* queue);


#define QUEUE_MSG_PROCESS(QUEUE,FUNC)              \
    do {                                           \
            uint8_t out[MAX_BUF];                  \
            if(!queue_empty(QUEUE)) {              \
                int len = queue_out(QUEUE, out);   \
                if(len > 0) {                      \
                    FUNC(out, len);                \
            }                                      \
      }                                            \
    } while (0)


#define QUEUE_MSG_EMPTY(QUEUE)       (queue_empty(QUEUE))
#define QUEUE_MSG_NOT_EMPTY(QUEUE)   (!queue_empty(QUEUE))


#endif /* APP_QUEUE_H__ */

