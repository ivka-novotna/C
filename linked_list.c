#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <ctype.h>
#include "linked_list.h"
#define true 1     
#define false 0 

struct item {
    int value;
};
typedef struct queue {
    struct item data;
    struct queue* next;
} QUEUE;

QUEUE* myQueue = NULL;


_Bool is_empty(void) {
    return (myQueue == NULL);
}

_Bool push(int entry){
    if (entry < 0){
        return false;
    } else{
        QUEUE *new_q = (QUEUE*)malloc(sizeof(QUEUE));
        new_q->data.value =  entry;
        new_q->next = NULL;
        if(is_empty()){
            myQueue = new_q;
            return true;
        }  
        else {
            QUEUE *tmp = myQueue;
            while(tmp->next != NULL){
                tmp = tmp->next;
            }
            tmp->next = new_q;
            return true;     
        }
    }  
}
int pop(void){
    int element = -1;
    if (is_empty()) {
        return element;
    }
    element = myQueue->data.value;
    QUEUE *tmp = myQueue;
    tmp = tmp->next;
    free(myQueue);
    myQueue = tmp;
    return element;

}

_Bool insert(int entry){
   QUEUE *new_q = (QUEUE*)malloc(sizeof(QUEUE));
   new_q->data.value = entry;
   new_q->next = NULL;
   if(is_empty()){
        myQueue = new_q;
        return true;
    }  
    else if(entry > myQueue->data.value){
        new_q->next = myQueue;
        myQueue = new_q;
        return true;
    }
   else{
        QUEUE *tmp = myQueue;
        while(tmp->next != NULL && tmp->next->data.value > entry){
            tmp = tmp->next;
        }   
        if(tmp->next == NULL){
            tmp->next = new_q;
        }
        else{
            new_q->next = tmp->next;
            tmp->next = new_q;
        }   
      return true; 
    }
}  


_Bool erase(int entry){
    int counter = 0;
    int queue_size = size();
    QUEUE *tmp = myQueue;
    if(queue_size == 1){
        if (tmp->data.value == entry){
            clear();
            myQueue = NULL;
            return true;
        }else
            return false;
    }else if(queue_size == 2){
        if (tmp->data.value == entry ){
            if (tmp->next->data.value == entry){     
                clear();
                myQueue = NULL;
                return true;
            } else {
                tmp = tmp->next;
                free(myQueue);
                myQueue = tmp;
                return true;
            }
        }
    }else if (queue_size <= 0){
        return false;
    }


    while(tmp && tmp->data.value == entry ){
        tmp = tmp->next;
        free(myQueue);
        myQueue = tmp;
        counter++;
    }
    QUEUE *curr = myQueue;
    QUEUE *prev = NULL;
    QUEUE *tmp2 = NULL;
    while(curr){
        if (curr->data.value == entry){
            prev->next = curr->next;
            counter++;
            tmp2 = curr;
            curr = curr->next;
            free(tmp2);
            tmp2 = NULL;

        }
        else{
            prev = curr;
            curr = curr->next;
        }       
    }
    if (counter == 0)
        return false;
    else
        return true;   
}   



int getEntry(int idx){
    int element = -1;
    if(idx < 0 || idx >= size()){
        return element;
    }else{
        int i = 0;
        QUEUE *tmp = myQueue;
        while (i < idx) {
           tmp = tmp->next;
           i++;
        }
        element = tmp->data.value;
        return element;
    }
}
int size(void){
    int counter = 0;
    if(is_empty()){
        return counter;
    }
    QUEUE *tmp = myQueue;
    while (tmp != NULL) {
       tmp = tmp->next;
       counter++;
    }   
    return counter;   
}

void clear(){
    QUEUE *tmp = myQueue;
    while(tmp != NULL){
        tmp = tmp->next;
        free(myQueue);
        myQueue = tmp;
    }
}
