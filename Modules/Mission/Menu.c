/**
 * @file  Menu.c
 * @brief OLED 菜单系统实现，负责模式树构建、显示与选择
 */
#include "Menu.h"
#include "AppState.h"
#include "ErrorHandler.h"
#include "Initialize.h"
#include "Key.h"
#include "Mode.h"
#include <stdio.h>

static ModeTree *now_mode_tree = NULL;
char CircleNum = '0';

static char circle_name_1[] = "1";
static char circle_name_2[] = "2";
static char circle_name_3[] = "3";
static char circle_name_4[] = "4";
static char circle_name_5[] = "5";

static CircleList static_menu_list;

void menu_init(void){
    initModeTreePool();
    initCircleListPool();

    ModeNode root_node = {NULL, "Root"};
    now_mode_tree = createModeTree(root_node);
    if (now_mode_tree == NULL){
        sprintf(error_message, "Failed to create mode tree");
        error_handler();
    }

    now_mode_tree->parent = NULL;
    now_mode_tree->nodes.mode_function = menu_function;
    now_mode_tree->nodes.mode_name = "Main Menu";
    OLED_Clear();

    ModeNode problem_b_node = {menu_function, "Problem B"};
    ModeNode problem_h_node = {menu_function, "Problem H"};

    ModeTree *problem_b_menu = createModeTree(problem_b_node);
    ModeTree *problem_h_menu = createModeTree(problem_h_node);

    addChild(now_mode_tree, problem_b_menu);
    addChild(now_mode_tree, problem_h_menu);

#if PROJECT_ENABLE_TEST_MODES
    ModeNode test_node = {menu_function, "Test Menu"};
    ModeTree *test_menu = createModeTree(test_node);

    addChild(now_mode_tree, test_menu);

    ModeNode test_dis_mode = {mode_test_distance, "Test Distance"};
    ModeNode test_cordi_mode = {mode_test_coordinate, "Test Coordinate"};
    ModeNode test_circle_mode = {mode_test_circle, "Test Circle"};
    ModeNode test_track_mode = {mode_test_tracking, "Test Track"};
    ModeNode test_connect_mode = {mode_test_connection, "Test Connect"};

    addChild(test_menu, createModeTree(test_connect_mode));
    addChild(test_menu, createModeTree(test_dis_mode));
    addChild(test_menu, createModeTree(test_cordi_mode));
    addChild(test_menu, createModeTree(test_circle_mode));
    addChild(test_menu, createModeTree(test_track_mode));
#endif

    ModeNode problem_b1_menu_node = {menu_function, "ProB1"};
    ModeNode problem_b23_menu_node = {mode_problem_b_2_3, "ProB2/3"};
    ModeNode problem_h1_menu_node = {menu_function, "ProH1"};
    ModeNode problem_h2_menu_node = {mode_problem_h_2, "ProH2"};

    ModeTree *problem_b_menu_1 = createModeTree(problem_b1_menu_node);
    ModeTree *problem_b_menu_23 = createModeTree(problem_b23_menu_node);
    ModeTree *problem_h_menu_1 = createModeTree(problem_h1_menu_node);
    ModeTree *problem_h_menu_2 = createModeTree(problem_h2_menu_node);

    addChild(problem_b_menu, problem_b_menu_1);
    addChild(problem_b_menu, problem_b_menu_23);
    addChild(problem_h_menu, problem_h_menu_1);
    addChild(problem_h_menu, problem_h_menu_2);

    char *circle_names[] = {circle_name_1, circle_name_2, circle_name_3, circle_name_4, circle_name_5};

    for (int i = 0; i < 5; i++){
        ModeNode circle_node = {mode_problem_b_1, circle_names[i]};
        ModeTree *circle_menu = createModeTree(circle_node);

        addChild(problem_b_menu_1, circle_menu);
        if (circle_menu == NULL){
            sprintf(error_message, "Failed to create circle menu %d", i + 1);
            error_handler();
        }
    }

    for (int i = 0; i < 2; i++){
        ModeNode circle_node = {mode_problem_h_1, circle_names[i]};
        ModeTree *circle_menu = createModeTree(circle_node);

        addChild(problem_h_menu_1, circle_menu);
        if (circle_menu == NULL){
            sprintf(error_message, "Failed to create circle menu %d", i + 1);
            error_handler();
        }
    }
}

void menu_begin(void){
    now_mode_tree->nodes.mode_function();
}

void menu_function(void){
    OLED_Clear();
    CircleList_Clear(&static_menu_list);
    CircleList_Init(&static_menu_list);

    if (now_mode_tree == NULL){
        OLED_ShowString(0, 0, "Menu not initialized", 8);
        return;
    }

    if (now_mode_tree->parent == NULL && now_mode_tree->firstChild == NULL){
        OLED_ShowString(0, 0, "Empty Menu", 8);
        return;
    }

    if (now_mode_tree->parent != NULL){
        if (CircleList_Insert(&static_menu_list, now_mode_tree->parent) != 0){
            OLED_ShowString(0, 0, "Menu list full", 8);
            return;
        }
    }

    ModeTree *child = getFirstChild(now_mode_tree);
    while (child != NULL){
        if (CircleList_Insert(&static_menu_list, child) != 0){
            OLED_ShowString(0, 0, "Menu list full", 8);
            return;
        }
        child = getNextSibling(child);
    }

    CircleListNode *current = static_menu_list.head;
    int index = 0;

    do{
        if (current->data != NULL && current->data->nodes.mode_name != NULL){
            if (current->data == now_mode_tree->parent){
                OLED_ShowString(0, index, "..", 8);
            } else{
                OLED_ShowString(0, index, current->data->nodes.mode_name, 8);
            }
            index++;
        }
        current = current->next;
    } while (current != static_menu_list.head);

    int total = index;
    index = 0;
    OLED_ShowChar(END_X - 8, index, '<', 8);

    while (1){
        if (Key_long_press()){
            CircleNum = current->data->nodes.mode_name[0];
            select_menu(current->data);
            break;
        }

        if (Key_short_press()){
            current = current->next;
            OLED_ShowChar(END_X - 8, index, ' ', 8);
            index = (index + 1) % total;
            OLED_ShowChar(END_X - 8, index, '<', 8);
        }
    }
}

void select_menu(ModeTree *menu){
    if (menu == NULL){
        return;
    }

    OLED_Clear();
    now_mode_tree = menu;
    now_mode_tree->nodes.mode_function();
}

bool is_menu_node(ModeTree *node){
    return (node != NULL && node->nodes.mode_function != NULL) &&
           (node->nodes.mode_function == menu_function);
}
