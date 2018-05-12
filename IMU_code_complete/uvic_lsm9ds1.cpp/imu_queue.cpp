/*
 * A simple circular queue that overwrites itself as newer data is collected.
 * Used to compute running averages of data collected by an ST LSM9DS1 microcontroller.
 * Author: Andrew Rose
 */


#include "imu_queue.h"

IMUQueue::IMUQueue(int q_size){
  array = (int16_t*)malloc(q_size*sizeof(int16_t));
  if (array==NULL){
    Serial.print("Error allocating memory");
    exit(1);
  }
  tail = -1; //the tail of the array will be immediately bumped to 0, the first index
  size = q_size; //the length of the array
}

void IMUQueue::enqueue(int16_t item){
  tail = (tail + 1) % size;
  array[tail] = item;
}

int16_t IMUQueue::getTail(){
  return array[tail];
}

int16_t getAvg(){
  int32_t sum;
  for (int i = 0; i < size; i++){
    sum += array[i];
  }
  return (int16_t) sum/size;
}
