/**
 * @file  CircleList.c
 * @brief 循环链表实现，基于静态内存池分配节点
 */
#include "CircleList.h"
#include "AppState.h"
#include "ErrorHandler.h"
#include <stdio.h>

static CircleListNode circle_pool[MAX_CIRCLE_LIST_NODES];
static bool circle_pool_initialized = false;

void initCircleListPool(void){
    for (int i = 0; i < MAX_CIRCLE_LIST_NODES; i++){
        circle_pool[i].is_used = false;
        circle_pool[i].data = NULL;
        circle_pool[i].next = NULL;
    }
    circle_pool_initialized = true;
}

static CircleListNode *allocateCircleListNode(void){
    if (!circle_pool_initialized){
        initCircleListPool();
    }

    for (int i = 0; i < MAX_CIRCLE_LIST_NODES; i++){
        if (!circle_pool[i].is_used){
            circle_pool[i].is_used = true;
            circle_pool[i].data = NULL;
            circle_pool[i].next = NULL;
            return &circle_pool[i];
        }
    }
    return NULL;
}

static void freeCircleListNode(CircleListNode *node){
    if (node >= circle_pool && node < circle_pool + MAX_CIRCLE_LIST_NODES){
        node->is_used = false;
        node->data = NULL;
        node->next = NULL;
    }
}

void CircleList_Init(CircleList *list){
    list->head = NULL;
    list->tail = NULL;
}

int CircleList_Insert(CircleList *list, ModeTree *data){
    CircleListNode *new_node = allocateCircleListNode();
    if (new_node == NULL){
        sprintf(error_message, "No free circle list nodes available");
        error_handler();
        return -1;
    }

    new_node->data = data;
    if (list->head == NULL){
        list->head = new_node;
        list->tail = new_node;
        new_node->next = new_node;
    } else{
        list->tail->next = new_node;
        list->tail = new_node;
        new_node->next = list->head;
    }

    return 0;
}

void CircleList_Delete(CircleList *list, ModeTree *data){
    if (list->head == NULL){
        return;
    }

    CircleListNode *current = list->head;
    CircleListNode *previous = list->tail;

    do{
        if (current->data == data){
            if (current == list->head){
                if (list->head == list->tail){
                    freeCircleListNode(current);
                    list->head = NULL;
                    list->tail = NULL;
                } else{
                    list->tail->next = current->next;
                    list->head = current->next;
                    freeCircleListNode(current);
                }
            } else{
                previous->next = current->next;
                if (current == list->tail){
                    list->tail = previous;
                }
                freeCircleListNode(current);
            }
            return;
        }
        previous = current;
        current = current->next;
    } while (current != list->head);
}

void CircleList_Display(CircleList *list){
    (void)list;
}

void CircleList_Clear(CircleList *list){
    if (list->head == NULL){
        return;
    }

    CircleListNode *current = list->head;
    CircleListNode *next;

    do{
        next = current->next;
        freeCircleListNode(current);
        current = next;
    } while (current != list->head);

    list->head = NULL;
    list->tail = NULL;
}
