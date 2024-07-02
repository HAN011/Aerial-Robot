/**
 * @file referee_UI.C
 * @author kidneygood (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2023-1-18
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "referee_UI.h"
#include "string.h"
#include "crc_ref.h"
#include "stdio.h"
#include "rm_referee.h"
#include "message_center.h"
#include "robot_cmd.h"
#include "gimbal.h"
#include "dji_motor.h"
#include "super_cap.h"

extern referee_info_t referee_info;                         // 裁判系统数据
extern Referee_Interactive_info_t Referee_Interactive_info; // 绘制UI所需的数据

uint8_t UI_Seq;                           // 包序号，供整个referee文件使用
static Graph_Data_t UI_shoot_dot[10];     // 射击准线
static Graph_Data_t UI_Deriction_line[4]; // 射击准线
static Graph_Data_t UI_Energy[3];         // 电容能量条
Graph_Data_t UI_Rectangle[10];     // 矩形
static Graph_Data_t UI_Circle_t[10];      // 圆形
static Graph_Data_t UI_Arco_t[10];        // 圆弧
static Graph_Data_t UI_Number_t[10];      // 数字
static String_Data_t UI_State_sta[10];    // 机器人状态,静态只需画一次
// static String_Data_t UI_State_dyn[6];							// 机器人状态,动态先add才能change

static Subscriber_t *gimbal_feed_sub;          // 云台反馈信息订阅者
static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息

uint8_t Super_condition;    // 超电的开关状态
float Super_condition_volt; // 超电的电压

float Pitch_Angle; // pitch角度（角度制)
float Yaw_Angle;   // yaw角度（角度制）

extern auto_shoot_mode_e AutoShooting_flag; // 自动射击标志位

/********************************************删除操作*************************************
**参数：_id 对应的id结构体
        Del_Operate  对应头文件删除操作
        Del_Layer    要删除的层 取值0-9
*****************************************************************************************/
void UIDelete(referee_id_t *_id, uint8_t Del_Operate, uint8_t Del_Layer)
{
    static UI_delete_t UI_delete_data;
    uint8_t temp_datalength = Interactive_Data_LEN_Head + UI_Operate_LEN_Del; // 计算交互数据长度

    UI_delete_data.FrameHeader.SOF        = REFEREE_SOF;
    UI_delete_data.FrameHeader.DataLength = temp_datalength;
    UI_delete_data.FrameHeader.Seq        = UI_Seq;
    UI_delete_data.FrameHeader.CRC8       = Get_CRC8_Check_Sum((uint8_t *)&UI_delete_data, LEN_CRC8, 0xFF);

    UI_delete_data.CmdID = ID_student_interactive;

    UI_delete_data.datahead.data_cmd_id = UI_Data_ID_Del;
    UI_delete_data.datahead.receiver_ID = _id->Cilent_ID;
    UI_delete_data.datahead.sender_ID   = _id->Robot_ID;

    UI_delete_data.Delete_Operate = Del_Operate; // 删除操作
    UI_delete_data.Layer          = Del_Layer;

    UI_delete_data.frametail = Get_CRC16_Check_Sum((uint8_t *)&UI_delete_data, LEN_HEADER + LEN_CMDID + temp_datalength, 0xFFFF);
    /* 填入0xFFFF,关于crc校验 */

    RefereeSend((uint8_t *)&UI_delete_data, LEN_HEADER + LEN_CMDID + temp_datalength + LEN_TAIL); // 发送

    UI_Seq++; // 包序号+1
}
/************************************************绘制直线*************************************************
**参数：*graph Graph_Data类型变量指针，用于存放图形数据
        graphname[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Width    图形线宽
        Start_x、Start_y  起点xy坐标
        End_x、End_y   终点xy坐标
**********************************************************************************************************/

void UILineDraw(Graph_Data_t *graph, char graphname[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color,
                uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t End_x, uint32_t End_y)
{
    int i;
    for (i = 0; i < 3 && graphname[i] != '\0'; i++) // 填充至‘0’为止
    {
        graph->graphic_name[2 - i] = graphname[i]; // 按内存地址增大方向填充，所以会有i与2-i
    }

    graph->operate_tpye = Graph_Operate;
    graph->graphic_tpye = UI_Graph_Line;
    graph->layer        = Graph_Layer;
    graph->color        = Graph_Color;

    graph->start_angle = 0;
    graph->end_angle   = 0;
    graph->width       = Graph_Width;
    graph->start_x     = Start_x;
    graph->start_y     = Start_y;
    graph->radius      = 0;
    graph->end_x       = End_x;
    graph->end_y       = End_y;
}

/************************************************绘制矩形*************************************************
**参数：*graph Graph_Data类型变量指针，用于存放图形数据
        graphname[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Width    图形线宽
        Start_x、Start_y    起点xy坐标
        End_x、End_y        对角顶点xy坐标
**********************************************************************************************************/
void UIRectangleDraw(Graph_Data_t *graph, char graphname[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color,
                     uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t End_x, uint32_t End_y)
{
    int i;
    for (i = 0; i < 3 && graphname[i] != '\0'; i++) {
        graph->graphic_name[2 - i] = graphname[i];
    }

    graph->graphic_tpye = UI_Graph_Rectangle;
    graph->operate_tpye = Graph_Operate;
    graph->layer        = Graph_Layer;
    graph->color        = Graph_Color;

    graph->start_angle = 0;
    graph->end_angle   = 0;
    graph->width       = Graph_Width;
    graph->start_x     = Start_x;
    graph->start_y     = Start_y;
    graph->radius      = 0;
    graph->end_x       = End_x;
    graph->end_y       = End_y;
}

/************************************************绘制整圆*************************************************
**参数：*graph Graph_Data类型变量指针，用于存放图形数据
        graphname[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Width    图形线宽
        Start_x、Start_y    圆心xy坐标
        Graph_Radius  圆形半径
**********************************************************************************************************/

void UICircleDraw(Graph_Data_t *graph, char graphname[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color,
                  uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t Graph_Radius)
{
    int i;
    for (i = 0; i < 3 && graphname[i] != '\0'; i++) {
        graph->graphic_name[2 - i] = graphname[i];
    }

    graph->graphic_tpye = UI_Graph_Circle;
    graph->operate_tpye = Graph_Operate;
    graph->layer        = Graph_Layer;
    graph->color        = Graph_Color;

    graph->start_angle = 0;
    graph->end_angle   = 0;
    graph->width       = Graph_Width;
    graph->start_x     = Start_x;
    graph->start_y     = Start_y;
    graph->radius      = Graph_Radius;
    graph->end_x       = 0;
    graph->end_y       = 0;
}
/************************************************绘制椭圆*************************************************
**参数：*graph Graph_Data类型变量指针，用于存放图形数据
        graphname[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Width    图形线宽
        Start_x、Start_y    圆心xy坐标
        End_x、End_y        xy半轴长度
**********************************************************************************************************/
void UIOvalDraw(Graph_Data_t *graph, char graphname[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color,
                uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t end_x, uint32_t end_y)
{
    int i;
    for (i = 0; i < 3 && graphname[i] != '\0'; i++) {
        graph->graphic_name[2 - i] = graphname[i];
    }

    graph->graphic_tpye = UI_Graph_Ellipse;
    graph->operate_tpye = Graph_Operate;
    graph->layer        = Graph_Layer;
    graph->color        = Graph_Color;
    graph->width        = Graph_Width;

    graph->start_angle = 0;
    graph->end_angle   = 0;
    graph->width       = Graph_Width;
    graph->start_x     = Start_x;
    graph->start_y     = Start_y;
    graph->radius      = 0;
    graph->end_x       = end_x;
    graph->end_y       = end_y;
}

/************************************************绘制圆弧*************************************************
**参数：*graph Graph_Data类型变量指针，用于存放图形数据
        graphname[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_StartAngle,Graph_EndAngle    起始终止角度
        Graph_Width    图形线宽
        Start_y,Start_y    圆心xy坐标
        x_Length,y_Length   xy半轴长度
**********************************************************************************************************/

void UIArcDraw(Graph_Data_t *graph, char graphname[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color,
               uint32_t Graph_StartAngle, uint32_t Graph_EndAngle, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y,
               uint32_t end_x, uint32_t end_y)
{
    int i;
    for (i = 0; i < 3 && graphname[i] != '\0'; i++) {
        graph->graphic_name[2 - i] = graphname[i];
    }

    graph->graphic_tpye = UI_Graph_Arc;
    graph->operate_tpye = Graph_Operate;
    graph->layer        = Graph_Layer;
    graph->color        = Graph_Color;

    graph->start_angle = Graph_StartAngle;
    graph->end_angle   = Graph_EndAngle;
    graph->width       = Graph_Width;
    graph->start_x     = Start_x;
    graph->start_y     = Start_y;
    graph->radius      = 0;
    graph->end_x       = end_x;
    graph->end_y       = end_y;
}

/************************************************绘制浮点型数据*************************************************
**参数：*graph Graph_Data类型变量指针，用于存放图形数据
        graphname[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Size     字号
        Graph_Digit    小数位数
        Graph_Width    图形线宽
        Start_x、Start_y    开始坐标
        radius=a&0x3FF;   a为浮点数乘以1000后的32位整型数
        end_x=(a>>10)&0x7FF;
        end_y=(a>>21)&0x7FF;
**********************************************************************************************************/

void UIFloatDraw(Graph_Data_t *graph, char graphname[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color,
                 uint32_t Graph_Size, uint32_t Graph_Digit, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, int32_t Graph_Float)
{

    int i;
    for (i = 0; i < 3 && graphname[i] != '\0'; i++) {
        graph->graphic_name[2 - i] = graphname[i];
    }
    graph->graphic_tpye = UI_Graph_Float;
    graph->operate_tpye = Graph_Operate;
    graph->layer        = Graph_Layer;
    graph->color        = Graph_Color;

    graph->width       = Graph_Width;
    graph->start_x     = Start_x;
    graph->start_y     = Start_y;
    graph->start_angle = Graph_Size;
    graph->end_angle   = Graph_Digit;

    graph->radius = Graph_Float & 0x3FF;
    graph->end_x  = (Graph_Float >> 10) & 0x7FF;
    graph->end_y  = (Graph_Float >> 21) & 0x7FF;
}

/************************************************绘制整型数据*************************************************
**参数：*graph Graph_Data类型变量指针，用于存放图形数据
        graphname[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Size     字号
        Graph_Width    图形线宽
        Start_x、Start_y    开始坐标
        radius=a&0x3FF;   a为32位整型数
        end_x=(a>>10)&0x7FF;
        end_y=(a>>21)&0x7FF;
**********************************************************************************************************/
void UIIntDraw(Graph_Data_t *graph, char graphname[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color,
               uint32_t Graph_Size, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, int32_t Graph_Integer)
{
    int i;
    for (i = 0; i < 3 && graphname[i] != '\0'; i++) {
        graph->graphic_name[2 - i] = graphname[i];
    }
    graph->graphic_tpye = UI_Graph_Int;
    graph->operate_tpye = Graph_Operate;
    graph->layer        = Graph_Layer;
    graph->color        = Graph_Color;

    graph->start_angle = Graph_Size;
    graph->end_angle   = 0;
    graph->width       = Graph_Width;
    graph->start_x     = Start_x;
    graph->start_y     = Start_y;
    graph->radius      = Graph_Integer & 0x3FF;
    graph->end_x       = (Graph_Integer >> 10) & 0x7FF;
    graph->end_y       = (Graph_Integer >> 21) & 0x7FF;
}

/************************************************绘制字符型数据*************************************************
**参数：*graph Graph_Data类型变量指针，用于存放图形数据
        graphname[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Size     字号
        Graph_Width    图形线宽
        Start_x、Start_y    开始坐标

**参数：*graph Graph_Data类型变量指针，用于存放图形数据
        fmt需要显示的字符串
        此函数的实现和具体使用类似于printf函数
**********************************************************************************************************/
void UICharDraw(String_Data_t *graph, char graphname[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color,
                uint32_t Graph_Size, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, char *fmt, ...)
{
    int i;
    for (i = 0; i < 3 && graphname[i] != '\0'; i++) {
        graph->Graph_Control.graphic_name[2 - i] = graphname[i];
    }

    graph->Graph_Control.graphic_tpye = UI_Graph_Char;
    graph->Graph_Control.operate_tpye = Graph_Operate;
    graph->Graph_Control.layer        = Graph_Layer;
    graph->Graph_Control.color        = Graph_Color;

    graph->Graph_Control.width       = Graph_Width;
    graph->Graph_Control.start_x     = Start_x;
    graph->Graph_Control.start_y     = Start_y;
    graph->Graph_Control.start_angle = Graph_Size;
    graph->Graph_Control.radius      = 0;
    graph->Graph_Control.end_x       = 0;
    graph->Graph_Control.end_y       = 0;

    // va_list ap;
    // va_start(ap, fmt);
    // vsprintf((char *)graph->show_Data, fmt, ap); // 使用参数列表进行格式化并输出到字符串
    // va_end(ap);
    graph->Graph_Control.end_angle = strlen((const char *)graph->show_Data);
}

/* UI推送函数（使更改生效）
   参数： cnt   图形个数
            ...   图形变量参数
   Tips：：该函数只能推送1，2，5，7个图形，其他数目协议未涉及
 */
void UIGraphRefresh(referee_id_t *_id, int cnt, ...)
{
    UI_GraphReFresh_t UI_GraphReFresh_data;
    Graph_Data_t graphData;

    uint8_t temp_datalength = LEN_HEADER + LEN_CMDID + Interactive_Data_LEN_Head + UI_Operate_LEN_PerDraw * cnt + LEN_TAIL; // 计算交互数据长度

    static uint8_t buffer[512]; // 交互数据缓存

    va_list ap;        // 创建一个 va_list 类型变量
    va_start(ap, cnt); // 初始化 va_list 变量为一个参数列表

    UI_GraphReFresh_data.FrameHeader.SOF        = REFEREE_SOF;
    UI_GraphReFresh_data.FrameHeader.DataLength = Interactive_Data_LEN_Head + cnt * UI_Operate_LEN_PerDraw;
    UI_GraphReFresh_data.FrameHeader.Seq        = UI_Seq;
    UI_GraphReFresh_data.FrameHeader.CRC8       = Get_CRC8_Check_Sum((uint8_t *)&UI_GraphReFresh_data, LEN_CRC8, 0xFF);

    UI_GraphReFresh_data.CmdID = ID_student_interactive;

    switch (cnt) {
        case 1:
            UI_GraphReFresh_data.datahead.data_cmd_id = UI_Data_ID_Draw1;
            break;
        case 2:
            UI_GraphReFresh_data.datahead.data_cmd_id = UI_Data_ID_Draw2;
            break;
        case 5:
            UI_GraphReFresh_data.datahead.data_cmd_id = UI_Data_ID_Draw5;
            break;
        case 7:
            UI_GraphReFresh_data.datahead.data_cmd_id = UI_Data_ID_Draw7;
            break;
    }

    UI_GraphReFresh_data.datahead.receiver_ID = _id->Cilent_ID;
    UI_GraphReFresh_data.datahead.sender_ID   = _id->Robot_ID;
    memcpy(buffer, (uint8_t *)&UI_GraphReFresh_data, LEN_HEADER + LEN_CMDID + Interactive_Data_LEN_Head); // 将帧头、命令码、交互数据帧头三部分复制到缓存中

    for (uint8_t i = 0; i < cnt; i++) // 发送交互数据的数据帧，并计算CRC16校验值
    {
        graphData = va_arg(ap, Graph_Data_t); // 访问参数列表中的每个项,第二个参数是你要返回的参数的类型,在取值时需要将其强制转化为指定类型的变量
        memcpy(buffer + (LEN_HEADER + LEN_CMDID + Interactive_Data_LEN_Head + UI_Operate_LEN_PerDraw * i), (uint8_t *)&graphData, UI_Operate_LEN_PerDraw);
    }
    Append_CRC16_Check_Sum(buffer, temp_datalength);
    RefereeSend(buffer, temp_datalength);

    va_end(ap); // 结束可变参数的获取
}

/************************************************UI推送字符（使更改生效）*********************************/
void UICharRefresh(referee_id_t *_id, String_Data_t string_Data)
{
    static UI_CharReFresh_t UI_CharReFresh_data;

    uint8_t temp_datalength = Interactive_Data_LEN_Head + UI_Operate_LEN_DrawChar; // 计算交互数据长度

    UI_CharReFresh_data.FrameHeader.SOF        = REFEREE_SOF;
    UI_CharReFresh_data.FrameHeader.DataLength = temp_datalength;
    UI_CharReFresh_data.FrameHeader.Seq        = UI_Seq;
    UI_CharReFresh_data.FrameHeader.CRC8       = Get_CRC8_Check_Sum((uint8_t *)&UI_CharReFresh_data, LEN_CRC8, 0xFF);

    UI_CharReFresh_data.CmdID = ID_student_interactive;

    UI_CharReFresh_data.datahead.data_cmd_id = UI_Data_ID_DrawChar;

    UI_CharReFresh_data.datahead.receiver_ID = _id->Cilent_ID;
    UI_CharReFresh_data.datahead.sender_ID   = _id->Robot_ID;

    UI_CharReFresh_data.String_Data = string_Data;

    UI_CharReFresh_data.frametail = Get_CRC16_Check_Sum((uint8_t *)&UI_CharReFresh_data, LEN_HEADER + LEN_CMDID + temp_datalength, 0xFFFF);

    RefereeSend((uint8_t *)&UI_CharReFresh_data, LEN_HEADER + LEN_CMDID + temp_datalength + LEN_TAIL); // 发送

    UI_Seq++; // 包序号+1
}

/**
 * @brief  判断各种ID，选择客户端ID
 * @param  referee_info_t *referee_info
 * @retval none
 * @attention
 */
static void DeterminRobotID()
{
    // id小于7是红色,大于7是蓝色,0为红色，1为蓝色   #define Robot_Red 0    #define Robot_Blue 1
    referee_info.referee_id.Robot_Color       = referee_info.GameRobotState.robot_id > 7 ? Robot_Blue : Robot_Red;
    referee_info.referee_id.Cilent_ID         = 0x0100 + referee_info.GameRobotState.robot_id; // 计算客户端ID
    referee_info.referee_id.Robot_ID          = referee_info.GameRobotState.robot_id;          // 计算机器人ID
    referee_info.referee_id.Receiver_Robot_ID = 0;
}

char Send_Once_Flag = 0; // 初始化标志
uint32_t Rect_De[4] = {1540, 555, 1660, 645};
int16_t AIM_Rect_X, AIM_Rect_Y; // 自瞄框中心点的坐标信息
int16_t AIM_Rect_half_length = 50;
int16_t AIM_Rec_Color;

void My_UIGraphRefresh()
{
    DeterminRobotID();
    SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);

    // const float arc = 45.0f; // 弧长
    // const uint16_t Mechangle_offset = 10546;
    // float mid_point_angle = fmod(720.0f - (yaw_angle - YAW_CHASSIS_ALIGN_ECD) * (360.0f / 8192.0f), 360.0f);
    // float angle_start = fmod(mid_point_angle + 360.0f - arc / 2.0f, 360.0f);
    // float angle_end = fmod(mid_point_angle + arc / 2.0f, 360.0f);

    if (Send_Once_Flag == 0) {
        Send_Once_Flag = 1;
        // 清空UI          GameRobotState
        UIDelete(&referee_info.referee_id, UI_Data_Del_ALL, 9);
        UIDelete(&referee_info.referee_id, UI_Data_Del_ALL, 8);
        UIDelete(&referee_info.referee_id, UI_Data_Del_ALL, 7);
        UIDelete(&referee_info.referee_id, UI_Data_Del_ALL, 6);
        // 射击准点
        // 射击线
        //  UILineDraw(&UI_shoot_line[0], "sl0", UI_Graph_ADD, 7, UI_Color_Yellow, 1, SCREEN_LENGTH/2,SCREEN_WIDTH/2,SCREEN_LENGTH/2,SCREEN_WIDTH/2-500);
        //  UILineDraw(&UI_shoot_line[2], "sl2", UI_Graph_ADD, 7, UI_Color_Yellow, 1,SCREEN_LENGTH/2-80,SCREEN_WIDTH/2-90,SCREEN_LENGTH/2+80,SCREEN_WIDTH/2-90);
        //  UILineDraw(&UI_shoot_line[4], "sl4", UI_Graph_ADD, 7, UI_Color_Yellow, 1,SCREEN_LENGTH/2-40,SCREEN_WIDTH/2-220,SCREEN_LENGTH/2+40,SCREEN_WIDTH/2-220);
        //  UILineDraw(&UI_shoot_line[5], "sl5", UI_Graph_ADD, 7, UI_Color_Yellow, 1, SCREEN_LENGTH / 2, SCREEN_WIDTH / 2 + 20, SCREEN_LENGTH / 2, SCREEN_WIDTH / 2 - 210);
        //  UILineDraw(&UI_shoot_line[7], "sl7", UI_Graph_ADD, 7, UI_Color_Yellow, 1,SCREEN_LENGTH/2-90,SCREEN_WIDTH/2-40,SCREEN_LENGTH/2+90,SCREEN_WIDTH/2-40);
        //  UILineDraw(&UI_shoot_line[8], "sl8", UI_Graph_ADD, 7, UI_Color_Yellow, 1,SCREEN_LENGTH/2-70,SCREEN_WIDTH/2-120,SCREEN_LENGTH/2+70,SCREEN_WIDTH/2-120);

        //自瞄指示圈
        UICircleDraw(&UI_Circle_t[5], "sc5", UI_Graph_ADD, 9, UI_Color_White, 2, SCREEN_LENGTH / 2, SCREEN_LENGTH / 2, 250); 
        // 位置标定线
        UILineDraw(&UI_Deriction_line[0], "sq0", UI_Graph_ADD, 6, UI_Color_White, 1, SCREEN_LENGTH / 2 - 22 + 30, SCREEN_WIDTH / 2 - 47, SCREEN_LENGTH / 2 - 22 + 5, SCREEN_WIDTH / 2 - 47);
        UILineDraw(&UI_Deriction_line[1], "sq1", UI_Graph_ADD, 6, UI_Color_White, 1, SCREEN_LENGTH / 2 - 22, SCREEN_WIDTH / 2 - 47 + 30, SCREEN_LENGTH / 2 - 22, SCREEN_WIDTH / 2 - 47 + 5);
        UILineDraw(&UI_Deriction_line[2], "sq2", UI_Graph_ADD, 6, UI_Color_White, 1, SCREEN_LENGTH / 2 - 22 - 5, SCREEN_WIDTH / 2 - 47, SCREEN_LENGTH / 2 - 22 - 30, SCREEN_WIDTH / 2 - 47);
        UILineDraw(&UI_Deriction_line[3], "sq3", UI_Graph_ADD, 6, UI_Color_White, 1, SCREEN_LENGTH / 2 - 22, SCREEN_WIDTH / 2 - 47 - 5, SCREEN_LENGTH / 2 - 22, SCREEN_WIDTH / 2 - 47 - 30);
        // 小陀螺
        // sprintf(UI_State_sta[0].show_Data,"Rotate");
        // UICharDraw(&UI_State_sta[0], "ss0", UI_Graph_ADD, 9, UI_Color_Yellow, 20, 2, 660, 100, "Rotate");
        // UICharRefresh(&referee_info.referee_id,UI_State_sta[0]);
        UICircleDraw(&UI_Circle_t[0], "sc0", UI_Graph_ADD, 9, UI_Color_White, 20, 700, 160, 8);
        // 摩擦轮
        //  sprintf(UI_State_sta[2].show_Data,"Fric");
        //  UICharDraw(&UI_State_sta[2], "ss2", UI_Graph_ADD, 9, UI_Color_Yellow, 20, 2, 1160,100, "Fric");
        //  UICharRefresh(&referee_info.referee_id, UI_State_sta[2]);
        UICircleDraw(&UI_Circle_t[2], "sc2", UI_Graph_ADD, 9, UI_Color_White, 20, 1180, 160, 8);  // 摩擦轮是否开启显示
        UICircleDraw(&UI_Circle_t[3], "sc3", UI_Graph_ADD, 9, UI_Color_Orange, 20, 1280, 160, 8); // 摩擦轮是否正常显示
        // 电容
        // 显示电容是否正常开启
        UICircleDraw(&UI_Circle_t[4], "sc4", UI_Graph_ADD, 9, UI_Color_White, 20, 580, 160, 8);

        sprintf(UI_State_sta[3].show_Data, "SuperCap");
        UICharDraw(&UI_State_sta[3], "ss3", UI_Graph_ADD, 9, UI_Color_Yellow, 20, 2, 80, 800, "SuperCap");
        UICharRefresh(&referee_info.referee_id, UI_State_sta[3]);
        UIRectangleDraw(&UI_Rectangle[1], "sr1", UI_Graph_ADD, 9, UI_Color_White, 4, 80, 700, 480, 740);
        UILineDraw(&UI_Energy[1], "sn1", UI_Graph_ADD, 9, UI_Color_Green, 20, 80, 720, (uint32_t)((Super_condition_volt * Super_condition_volt - 144) / 532 * 400 + 80), 720); // 超电电压在12V-26V之间
        // 初始自瞄框
        /*
        if((NUC_Data.yaw_offset==0) & (NUC_Data.pit_offset==0))
        {
            UIRectangleDraw(&UI_Rectangle[2],"sr2",UI_Graph_ADD,9,(int)AIM_Rec_Color+1,2,960,540,961,541);
        }
        else
        {
            UIRectangleDraw(&UI_Rectangle[2],"sr2",UI_Graph_ADD,9,(int)AIM_Rec_Color+1,2,AIM_Rect_X-AIM_Rect_half_length,AIM_Rect_Y-AIM_Rect_half_length,AIM_Rect_X+AIM_Rect_half_length,AIM_Rect_Y+AIM_Rect_half_length);
        }
        */

        // 云台朝向圆弧
        // if (mid_point_angle > 360.0f - arc / 2.0f || mid_point_angle < arc / 2.0f)
        // {
        // 	UIArcDraw(&UI_Arco_t[0], "sol", UI_Graph_ADD, 8, UI_Color_Green, angle_start, 360, 8, 960, 540, 100, 100);
        // 	UIArcDraw(&UI_Arco_t[1], "sor", UI_Graph_ADD, 8, UI_Color_Green, 0, angle_end, 8, 960, 540, 100, 100);
        // }
        // else
        // {
        // 	UIArcDraw(&UI_Arco_t[0], "sol", UI_Graph_ADD, 8, UI_Color_Green, angle_start, mid_point_angle, 8, 960, 540, 100, 100);
        // 	UIArcDraw(&UI_Arco_t[1], "sor", UI_Graph_ADD, 8, UI_Color_Green, mid_point_angle, angle_end, 8, 960, 540, 100, 100);
        // }
        // pitch角度
        sprintf(UI_State_sta[4].show_Data, "Pitch");
        UICharDraw(&UI_State_sta[4], "ss4", UI_Graph_ADD, 9, UI_Color_Yellow, 20, 2, 300, 700, "Pitch");
        UICharRefresh(&referee_info.referee_id, UI_State_sta[4]);

        UIFloatDraw(&UI_Number_t[0], "sm1", UI_Graph_ADD, 9, UI_Color_Yellow, 20, 5, 3, 300 + 100, 700, Pitch_Angle * 1000);

        // 射击准点
        UIGraphRefresh(&referee_info.referee_id, 7, UI_Deriction_line[0], UI_Deriction_line[1], UI_Deriction_line[2], UI_Deriction_line[3], UI_State_sta[0], UI_State_sta[2], UI_State_sta[4]);
        // 将位置标定线，小陀螺，弹舱盖，摩擦轮，电容一共7个图形打包一块发
        UIGraphRefresh(&referee_info.referee_id, 7, UI_Deriction_line[0], UI_Deriction_line[1], UI_Circle_t[0], UI_Circle_t[1], UI_Circle_t[2],UI_State_sta[5], UI_Rectangle[1]);
        // UIGraphRefresh(&referee_info.referee_id, 5, UI_Circle_t[0],UI_Circle_t[2],UI_Circle_t[3],UI_Rectangle[1],&UI_Energy[1]);
        UIGraphRefresh(&referee_info.referee_id, 7, UI_Circle_t[0], UI_Circle_t[2], UI_Circle_t[3], UI_Circle_t[4], UI_Energy[1], UI_Number_t[0],UI_Circle_t[5]);

    }

    else {
        //自瞄指示圈
        if(AutoShooting_flag == AutoShooting_Find){
            UICircleDraw(&UI_Circle_t[5], "sc5", UI_Graph_Change, 9, UI_Color_Green, 2, SCREEN_LENGTH / 2, SCREEN_LENGTH / 2, 250); 
        }
        else if(AutoShooting_flag == AutoShooting_Open){
            UICircleDraw(&UI_Circle_t[5], "sc5", UI_Graph_Change, 9, UI_Color_Orange, 2, SCREEN_LENGTH / 2, SCREEN_LENGTH / 2, 250); 
        }
        else{
            UICircleDraw(&UI_Circle_t[5], "sc5", UI_Graph_Change, 9, UI_Color_White, 2, SCREEN_LENGTH / 2, SCREEN_LENGTH / 2, 250); 
        }
        // 底盘模式
        if (Referee_Interactive_info.chassis_mode == CHASSIS_ROTATE) {
            UICircleDraw(&UI_Circle_t[0], "sc0", UI_Graph_Change, 9, UI_Color_Green, 20, 700, 160, 8);
        } else if (Referee_Interactive_info.chassis_mode == CHASSIS_FOLLOW_GIMBAL_YAW) {
            UICircleDraw(&UI_Circle_t[0], "sc0", UI_Graph_Change, 9, UI_Color_White, 20, 700, 160, 8);
        }
        // 摩擦轮
        // if (Referee_Interactive_info.friction_mode == FRICTION_ON) // 摩擦轮开启模式下
        // {
        //     UICircleDraw(&UI_Circle_t[2], "sc2", UI_Graph_Change, 9, UI_Color_Green, 20, 1180, 160, 8);
        // } else if (Referee_Interactive_info.friction_mode == FRICTION_OFF) // 摩擦轮关闭
        // {
        //     UICircleDraw(&UI_Circle_t[2], "sc2", UI_Graph_Change, 9, UI_Color_White, 20, 1180, 160, 8);
        // }
        if (Referee_Interactive_info.friction_mode == FRICTION_ON) // 摩擦轮开启模式下
        {
            UICircleDraw(&UI_Circle_t[2], "sc2", UI_Graph_Change, 9, UI_Color_Green, 20, 1180, 160, 8);
        } else  // 摩擦轮关闭
        {
            UICircleDraw(&UI_Circle_t[2], "sc2", UI_Graph_Change, 9, UI_Color_White, 20, 1180, 160, 8);
        }

        if (Referee_Interactive_info.shoot_mode == SHOOT_ON) // 发射模式开启
        {
            UICircleDraw(&UI_Circle_t[3], "sc3", UI_Graph_Change, 9, UI_Color_Green, 20, 1280, 160, 8);
        } else if (Referee_Interactive_info.shoot_mode == SHOOT_OFF) // 发射模式关闭
        {
            UICircleDraw(&UI_Circle_t[3], "sc3", UI_Graph_Change, 9, UI_Color_Orange, 20, 1280, 160, 8);
        }

        // 电容是否工作
        if (Super_condition == 0) {
            UICircleDraw(&UI_Circle_t[4], "sc4", UI_Graph_Change, 9, UI_Color_Green, 20, 580, 160, 8);
        } else {
            UICircleDraw(&UI_Circle_t[4], "sc4", UI_Graph_Change, 9, UI_Color_White, 20, 580, 160, 8);
        }

        // 电容
        UILineDraw(&UI_Energy[1], "sn1", UI_Graph_Change, 9, UI_Color_Green, 20, 80, 720, (uint32_t)((Super_condition_volt * Super_condition_volt - 144) / 532 * 200 + 80), 720); // 超电电压在12V-26V之间

        UIFloatDraw(&UI_Number_t[0], "sm1", UI_Graph_Change, 9, UI_Color_Yellow, 20, 5, 3, 300 + 100, 700, (uint32_t)(Pitch_Angle * 1000));
        // 云台朝向圆弧,中供弹暂时不需要
        // if (mid_point_angle > 360.0f - arc / 2.0f || mid_point_angle < arc / 2.0f)
        // {
        // 	UIArcDraw(&UI_Arco_t[0], "sol", UI_Graph_Change, 8, UI_Color_Green, angle_start, 360, 8, 960, 540, 100, 100);
        // 	UIArcDraw(&UI_Arco_t[1], "sor", UI_Graph_Change, 8, UI_Color_Green, 0, angle_end, 8, 960, 540, 100, 100);
        // }
        // else
        // {
        // 	UIArcDraw(&UI_Arco_t[0], "sol", UI_Graph_Change, 8, UI_Color_Green, angle_start, mid_point_angle, 8, 960, 540, 100, 100);
        // 	UIArcDraw(&UI_Arco_t[1], "sor", UI_Graph_Change, 8, UI_Color_Green, mid_point_angle, angle_end, 8, 960, 540, 100, 100);
        // }
        // 动态自瞄框
        /*等待自瞄接口
        if((NUC_Data.yaw_offset==0) & (NUC_Data.pit_offset==0))
        {
            UIRectangleDraw(&UI_Rectangle[2],"sr2",UI_Graph_Change,9,(int)AIM_Rec_Color+1,2,960,540,961,541  );
        }
            else
        {
            UIRectangleDraw(&UI_Rectangle[2],"sr2",UI_Graph_Change,9,(int)AIM_Rec_Color+1,2,AIM_Rect_X-AIM_Rect_half_length,AIM_Rect_Y-AIM_Rect_half_length,AIM_Rect_X+AIM_Rect_half_length,AIM_Rect_Y+AIM_Rect_half_length);
        }
        */

        // 发送4个指示圈+超电剩余电压条
        UIGraphRefresh(&referee_info.referee_id, 7, UI_Circle_t[0], UI_Circle_t[2], UI_Circle_t[3], UI_Circle_t[4], UI_Number_t[0], UI_Circle_t[5], UI_Energy[1]);
    }
}
