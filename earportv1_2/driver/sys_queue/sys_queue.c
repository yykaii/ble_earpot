#include <string.h>
#include "sys_queue.h"
#include "stdbool.h"

void queue_init(queue_list_t* queue) {
    queue->front = -1;
    queue->rear = -1;
    queue->size = 0;
}

bool queue_full(queue_list_t* queue) {
    return (queue_size(queue) == QUEUE_MAX);
}

bool queue_empty(queue_list_t* queue) {
    return (queue_size(queue) == 0);
}

void queue_in(queue_list_t* queue, uint8_t* item, int length) {
    if (queue_full(queue)) {
        return;
    }
	length = length > MAX_BUF ? MAX_BUF : length;

    queue->rear++;
    queue->rear %= QUEUE_MAX;
    queue->size++;
    memset(&queue->data[queue->rear], 0, sizeof(queue_element_t));
    memcpy(queue->data[queue->rear].cmd, item, length);
    queue->data[queue->rear].length = length;
}

int queue_out(queue_list_t* queue, uint8_t* item) {
    if (queue_empty(queue)) {
        return 0;
    }
    queue->front++;
    queue->front %= QUEUE_MAX;
    queue->size--;
    memcpy(item, queue->data[queue->front].cmd, queue->data[queue->front].length);
    return queue->data[queue->front].length;
}

int queue_size(queue_list_t* queue) {
    return queue->size;
}

int queue_clear(queue_list_t* queue) {
    if (queue_empty(queue)) {
        return 0;
    }
    queue->front++;
    queue->front %= QUEUE_MAX;
    queue->size--;
    return queue->size;
}
