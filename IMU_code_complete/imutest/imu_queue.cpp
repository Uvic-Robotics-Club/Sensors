/*
 * A simple circular queue that overwrites itself as newer data is collected.
 * Used to compute running averages of data collected by an ST LSM9DS1 microcontroller.
 * Author: Andrew Rose
 */


#include "imu_queue.h"

IMUQueue::IMUQueue(int len){
  arr = (int16_t*)malloc(len*sizeof(int16_t));
  if (arr==NULL){
    Serial.print("Error allocating memory");
    exit(1);
  }
  tail = -1; //the tail of the array will be immediately bumped to 0, the first index
  q_len = len; //the length of the array
}

void IMUQueue::enqueue(int16_t item){
  tail = (tail + 1) % q_len;
  arr[tail] = item;
}

int16_t IMUQueue::getTail(){
  return arr[tail];
}

int16_t IMUQueue::getAvg(){
  int32_t sum;
  for (int i = 0; i < q_len; i++){
    sum += arr[i];
  }
  return (int16_t) sum/q_len;
}
