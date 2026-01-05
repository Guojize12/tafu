#include "app_config.h"
#include "app_user.h"
#include "main.h"
#include <math.h>

uint32_t g_duoji_id = 0;//当前碰撞塔机ID // 20250620
uint16_t g_dj_leixing = 1; //0.平臂吊,1 动臂吊

uint16_t g_yj_warn[2] = {0}; //0上限 1,下限
uint16_t g_yj_alarm[2] = {0};//0上限 1,下限


static Timer g_timer_user = {0};
union IEEE754   g_enent = {0}, g_enent_last = {0};

uint8_t user_cfg_warn_ctl = 1;
uint32_t g_event_reboot = 0;
new_bef g_new_bef = {0};
sensor_state_def g_sensor_state =
{
#if 1
    .weight = 1,
    .range = 1,
    .rotation = 1,
    .height = 1,
    .wind_speed = 1,
    .angle = 1,
    .duoji = 1,
    .control = 1,
    .bit9 = 1,
    .gps = 1,
    .control_jiansu = 1,
#else
    0
#endif
};
relay_ctl_def g_relay_ctl = {0};
sensor_able_def g_sensor_able =
{
    .weight = 1,
    .range = 1,
    .rotation = 1,
    .height = 1,
    .wind_speed = 1,
    .angle = 1,
    .duoji = 1,
    .control = 1,
    .elevation = 1,
};

alarm_able_def g_alarm_able =
{
    .weight = 1,
    .range = 1,
    .rotation = 1,
    .height = 1,
    .wind_speed = 1,
    .angle = 1,
    .duoji = 1,
    .quyu = 1,
    .elevation = 1,
};

charg_board_bef g_charg_board =
{
    .timeout_num = 150,
    .voltage = 125,
    .charge_status = 0,
    .electricity = 60,
    .connection_status = 0,
    .laser_status = 1,
    .work_mode_laser_ipc = 1,


};
//charg_board_bef g_charg_board_new = {0};




weight_board_bef g_weight_board = {0};

static uint16_t g_sensor_last = 0;
static uint16_t g_alarm_last = 0;

/********************继电器******************************/
//void GPIO_13_SET(uint8_t status)
//{
//    bsp_realy_ctl(BSP_RELAY_RELAY0,status);
//}

//#define PI                            3.1415926f                          //Π=3.1415926

/*OUT1幅度向外1*/
#define     LONGOUTON               bsp_realy_ctl(BSP_RELAY_RELAY0,1)
#define     LONGOUTOFF              bsp_realy_ctl(BSP_RELAY_RELAY0,0)
#define     LONGOUT                 bsp_realy_status(BSP_RELAY_RELAY0)

/*OUT8小车向外*/
#define     LONGINON                bsp_realy_ctl(BSP_RELAY_RELAY1,1)
#define     LONGINOFF               bsp_realy_ctl(BSP_RELAY_RELAY1,0)
#define     LONGIN                  bsp_realy_status(BSP_RELAY_RELAY1)
/*OUT2高度上升2*/
#define     HIGHUPON                bsp_realy_ctl(BSP_RELAY_RELAY2,1)
#define     HIGHUPOFF               bsp_realy_ctl(BSP_RELAY_RELAY2,0)
#define     HIGHUP                  bsp_realy_status(BSP_RELAY_RELAY2)
/*高度下降*/
#define     HIGHDOWNON              bsp_realy_ctl(BSP_RELAY_RELAY3,1)   // 第61行
#define     HIGHDOWNOFF             bsp_realy_ctl(BSP_RELAY_RELAY3,0)
#define     HIGHDOWN                bsp_realy_status(BSP_RELAY_RELAY3)
/*回转向左*/
#define     ANGLELEFTON             bsp_realy_ctl(BSP_RELAY_RELAY4,1)
#define     ANGLELEFTOFF            bsp_realy_ctl(BSP_RELAY_RELAY4,0)
#define     ANGLELEFT               bsp_realy_status(BSP_RELAY_RELAY4)
/*回转向右*/
#define     ANGLERIGHTON            bsp_realy_ctl(BSP_RELAY_RELAY5,1)
#define     ANGLERIGHTOFF           bsp_realy_ctl(BSP_RELAY_RELAY5,0)
#define     ANGLERIGHT              bsp_realy_status(BSP_RELAY_RELAY5)

uint8_t g_remote_lock = 0; //0x0005：塔机（升降机、新迪文电表）锁机配置 参数 0正常， 1锁机
/*塔辅没有锁定功能，设定为1*/

uint32_t    g_stop_delay = 10; //单位分钟
uint8_t nucSendWorkDataFlag = 1;
uint8_t nucFaceFlag = 0;

uint32_t    nucGainCVAdress = 0; /*获取系数地址    2020-03-23*/

uint16_t        nucWeightMinData = 0; /*起重量量程下限-8000    2020-03-22*/
uint16_t        nucWeightMinSignle = 0; /*起重量信号下限-8002   2020-03-22*/
uint16_t        nucWeightMaxData = 0; /*起重量量程上限-8004    2020-03-22*/
uint16_t        nucWeightMaxSignle = 0; /*起重量信号上限-8006   2020-03-22*/

uint32_t        nucLongMinData = 0;     /*幅度量程下限-8008   2020-03-22*/
uint32_t        nucLongMinSignle = 0; /*幅度信号下限-800A   2020-03-22*/
uint32_t        nucLongMaxData = 0;     /*幅度量程上限-800C   2020-03-22*/
uint32_t        nucLongMaxSignle = 0; /*幅度信号上限-800E   2020-03-22*/

uint32_t        nucHighMinData = 0;     /*高度量程下限-8010   2020-03-22*/
uint32_t        nucHighMinSignle = 0; /*高度信号下限-8012   2020-03-22*/
uint32_t        nucHighMaxData = 0;     /*高度量程上限-8014   2020-03-22*/
uint32_t        nucHighMaxSignle = 0; /*高度信号上限-8016   2020-03-22*/

uint32_t        nucAngleMinData = 0;        /*回转量程下限-8018   2020-03-22*/
uint32_t        nucAngleMinSignle = 0;      /*回转信号下限-801A   2020-03-22*/
uint32_t        nucAngleMaxData = 0;        /*回转量程上限-801C   2020-03-22*/
uint32_t        nucAngleMaxSignle = 0;  /*回转信号上限-801E   2020-03-22*/

uint16_t        nucWindZero = 0;        /*风速零点-8020     2020-03-22*/
uint16_t        nucWindCV = 0;          /*风速系数-8022     2020-03-22*/

uint16_t        nucDispAngleXZero = 0;  /*倾角X零点-8024        2020-03-22*/
uint16_t        nucDispAngleYZero = 0;  /*倾角Y零点-8026        2020-03-22*/
//uint16_t      nucDispAngleZZero=0;    /*倾角Y零点-8026        2020-03-22*/





uint16_t        nucTowerStartWeight = 0; /*力矩曲线起始重量-8028   2020-03-22*/
uint16_t        nucTowerStartLong = 0;  /*力矩曲线起始幅度-802A     2020-03-22*/
uint16_t        nucTowerMidWeight = 0;  /*力矩曲线中间重量-802C     2020-03-22*/
uint16_t        nucTowerMidLong = 0; /*力矩曲线中间幅度-802E     2020-03-22*/
uint16_t        nucTowerEndWeight = 0;  /*力矩曲线终止重量-8030     2020-03-22*/
uint16_t        nucTowerEndLong = 0; /*力矩曲线终止幅度-8032     2020-03-22*/

uint16_t        nucTowerLong = 0;   /*塔机臂长-8036 2020-03-22*/
uint16_t        nucTowerMaxHigh = 0; /*塔机高度-8038 2020-03-22*/
uint16_t        nucTowerPHLong = 0;     /*塔机平衡臂长-803A   2020-03-22*/
uint16_t        nucTowerTMHigh = 0;     /*塔机塔帽高度-803C   2020-03-22*/
uint16_t        nucTowerBL = 0;         /*塔机倍率-803E 2020-03-22*/

uint32_t        nucSendDTUTime = 60;    /*传送数据时间    2020-03-22*/
uint32_t        nucSendDTUTime_Work = 10;   /*传送数据时间    2020-03-22*/

uint16_t        nucDanJStartLongTbl[8];/*单机防碰撞起始幅度-8050、8052、8054、8056、8058、805A、805C、805E*/
uint16_t        nucDanJEndLongTbl[8];   /*单机防碰撞终止幅度-8060、8062、8064、8066、8068、806A、806C、806E*/
uint16_t        nucDanJStartHighTbl[8];/*单机防碰撞起始高度-8070、8072、8074、8076、8078、807A、807C、807E*/
uint16_t        nucDanJEndHighTbl[8];   /*单机防碰撞终止高度-8080、8082、8084、8086、8088、808A、808C、808E*/
uint16_t        nucDanJStartAngleTbl[8];/*单机防碰撞起始角度-8090、8092、8094、8096、8098、809A、809C、809E*/
uint16_t        nucDanJEndAngleTbl[8];/*单机防碰撞终止角度-80A0、80A2、80A4、80A6、80A8、80AA、80AC、80AE*/
uint16_t        nucDuoJIDTbl_last[8];
uint16_t        nucDuoJIDTbl[8];        /*多机防碰撞设置ID-80B0、80B2、80B4、80B6、80B8、80BA、80BC、80BE*/
uint16_t        nucDuoJTowerLongTbl[8];/*多机防碰撞塔机臂长-80C0、80C2、80C4、80C6、80C8、80CA、80CC、80CE*/
uint16_t        nucDuoJXDDisdanceTbl[8];/*多机防碰撞设置距离-80D0、80D2、80D4、80D6、80D8、80DA、80DC、80DE*/
uint16_t        nucDuoJXDAngleTbl[8];   /*多机防碰撞设置相对角度-80E0、80E2、80E4、80E6、80E8、80EA、80EC、80EE*/
int16_t     nucDuoJXDHighTbl[8];    /*多机防碰撞设置相对高度-80F0、80F2、80F4、80F6、80F8、80FA、80FC、80FE*/
uint16_t        nucDanJStartAngleTbl_Reference[8];
uint16_t        nucHighMode = 0;        //高度零点位置        -811C               2020.12.18

uint16_t        nucDuoJTowerLongTbl_temp[8];/*幅度比较误差*/
int16_t         nucDuoJXDHighTbl_temp[8];/*塔机高度比较误差*/
uint16_t        nucDuoJTowerLongTbl_back[8];/*多机防碰撞塔机后臂长 */

uint16_t        nucControlFlag = 0;     /*系统控制功能-8040   2020-03-22*/
uint16_t        nucLongOutControl = 0;  /*幅度向外控制-8042   2020-03-22*/
uint16_t        nucLongInControl = 0; /*幅度向内控制-8044   2020-03-22*/
uint16_t        nucHighUpControl = 0; /*高度向上控制-8046   2020-03-22*/
uint16_t        nucHighDownControl = 0; /*高度向下控制-8048  2020-03-22*/
uint16_t        nucAngleLeftControl = 0; /*回转左转控制-804A 2020-03-22*/
uint16_t        nucAngleRightControl = 0; /*回转向右控制-804C    2020-03-22*/
uint16_t        nucDebugFlag = 0;   /*系统调试功能-804E   2020-03-22*/

uint16_t  DuoJiBianHao[8];                      //多机编号          -8104、8106、8108、810A、810C、810E、8110、8112
uint8_t  WIND_ALARM_OFF;                            //风速报警启用标志  -8114
uint8_t  QJ_ALARM_OFF;                              //倾角报警启用标志  -8116
uint8_t  WEIGHT_ALARM_OFF;                      //重量报警启用标志  -8118
int16_t QJ_ALARM_DATA;                          //倾角报警值
int16_t QJ_ALARM_DATA_1;                            //倾角预警值
uint16_t  WIND_ALARM_DATA;
uint16_t  WIND_ALARM_DATA_1;
uint16_t  HIGH_ALARM_DATA;
uint16_t  HIGH_ALARM_DATA_1;
uint16_t  LONG_ALARM_DATA;
uint16_t  LONG_ALARM_DATA_1;
uint16_t  DANJI_ALARM_DATA;
uint16_t  DANJI_ALARM_DATA_1;
uint16_t  DUOJI_ALARM_DATA;
uint16_t  DUOJI_ALARM_DATA_1;

/**********************************************参数计算参数********************************/
float   nucWeightSlope = 0;         /*重量斜率  2020-03-22*/
float   nucWeightCV = 0;            /*重量常数  2020-03-22*/

float   nucLongSlope = 0;       /*幅度斜率  2020-03-22*/
float   nucLongCV = 0;              /*幅度常数  2020-03-22*/

float   nucHighSlope = 0;       /*高度斜率  2020-03-22*/
float   nucHighCV = 0;              /*高度常数  2020-03-22*/

float   nucAngleSlope = 0;          /*回转斜率  2020-03-22*/
float   nucAngleCV = 0;             /*回转常数  2020-03-22*/

float   nucTowerSlope1 = 0;         /*塔吊力矩曲线1段斜率  2020-03-22*/
float   nucTowerCV1 = 0;            /*塔吊力矩曲线1段常数  2020-03-22*/

float   nucTowerSlope2 = 0;         /*塔吊力矩曲线1段斜率  2020-03-22*/
float   nucTowerCV2 = 0;            /*塔吊力矩曲线1段常数  2020-03-22*/

uint16_t        nucSysStatuse = 0;          /*系统状态-1022 2020-03-21*/
float   nucDuoJAngleTbl_temp[8];
float   nucDuoJAngleTbl[8];     /*多机防碰撞回转角度-1024、102A、1030、1036、103C、1042、1048、104E*/
float   nucDuoJLongTbl[8];      /*多机防碰撞回转角度-1026、102C、1032、1038、103E、1044、104A、1050*/
float   nucDuoJHighTbl[8];      /*多机防碰撞回转角度-1028、102E、1034、103A、1040、1046、104C、1052*/


float   nucWeightData = 0;          /*重量数据-1000 2020-03-21*/
float   nucLongData = 0;            /*幅度数据-1002 2020-03-21*/
float   nucHighData = 0;            /*高度数据-1004 2020-03-21*/
float   nucAngleData = 0;       /*回转数据-1006 2020-03-21*/
float   nucMomentData = 0;          /*力矩数据-1008 2020-03-21*/
float   nucMomentPerData = 0;   /*力矩百分比数据-100A    2020-03-21*/
float   nucWindData = 0;            /*风速数据-100C 2020-03-21*/
float   nucMaxWeightData = 0;   /*额定重量-1012 2020-03-21*/


uint8_t nucAlarmTbl[4] = {0};           /*报警状态2015-03-11*/

uint8_t DTU_SIGNLE_FLG = 0;                     //DTU信号标志
uint8_t DTU_SEND = 0;                   //实时数据发送成功标志

uint8_t nucSendBendiSetDataFlag = 0; //上传设置参数标志位  为1时向显示端发送本地设置参数

uint32_t    nucSendFreqCounter = 0;
uint32_t g_system_ticks = 0;
uint32_t g_start_ticks = 0;
uint32_t g_bosrd_start_ticks = 0;
uint8_t g_work_freq = 0;
uint8_t g_board_work_freq = 10;
float   nucWeightData_last1 = 0;
float   nucLongData_last1 = 0;
float   nucHighData_last1 = 0;

//uint8_t  nucSysGetTimeFlag=0;
//uint8_t  nucSysGetTimeFlag_clear=0;
uint8_t  nucSysNetFlag = 0;
uint8_t nucSendXinTiaoFlag = 0;


uint32_t    nucDTuPowerCounter = 0;
uint8_t nucSysCounter = 0;
uint8_t nucSysSendCounter = 0;
//uint8_t   nucSysSendFlag=0;
uint8_t nucDtuReceiveFlag = 0;
uint8_t nucOldTimeFlag = 0;


uint32_t        nucTestCounter = 0;
uint32_t        nucTestCounter2 = 0;

static int32_t g_CycleCount[3] = {0};
float   nucWeightData_work = 0;         /*工作循环重量数据相对0点-1000   2020-03-21*/
uint32_t nucWeightData_sum = 0;
uint32_t nucWeightData_num = 0;

/*********************************************塔机报警状态**************************/
uint8_t nucWeightStatuse = 0;   /*起重量状态       2020-03-24*/

uint8_t nucLongStatuse = 0;         /*幅度状态      2020-03-24*/
uint8_t nucLongStatuse1 = 0;        /*幅度状态1 多机向外 报警 2025-06-20*/
uint8_t nucLongStatuse2 = 0;        /*幅度状态2 多机向外 预警 2025-06-20*/

uint8_t nucHighStatuse = 0;         /*高度状态      2020-03-24*/
uint8_t nucDispAngleStatuse = 0; /*倾角状态      2020-03-24*/
uint8_t nucDanJiStatuse = 0;    /*单机防碰撞状态 2020-03-24*/

uint8_t nucMomentStatuse = 0;   /*力矩状态  2015-03-20*/
uint8_t nucAngleStatuse = 0;    /*角度状态  2015-03-20*/
uint8_t nucFPZFlag = 0;             /*防碰撞状态2015-03-20*/
uint8_t nucWindStatuse = 0;         /*风速状态  2015-03-20*/

uint8_t nucElevationStatuse = 0;         /*仰角状态  2015-03-20*/


uint8_t g_twist_alarm_status = 0;       /*扭度状态  2015-03-20*/

uint8_t nucSysDuoJiControlFlag = 0;
//uint8_t       nucDuojiStatuse=0;      //多机防碰撞状态     2020.11.02
uint8_t     nucDuojiStatuse = 0;
uint8_t     nucWindTimeCounter = 0;

uint8_t     nucAngleLeftControlFlag = 0; /*向左控制标志位 2021-1-9*/
uint8_t     nucAngleRightControlFlag = 0; /*向右控制标志位 2021-1-9*/
uint8_t     nucLongOutControlFlag = 0;      /*向外控制标志位 2021-1-9*/
uint8_t     nucLongInControlFlag = 0;   /*向内控制标志位 2021-1-9*/
uint8_t     nucSysDanJiControlFlag = 0; /*单机防碰撞控制标志位    2021-1-9*/
uint8_t     nucSysDanJiJiansulFlag = 0; /*单机防碰撞控制标志位    2021-1-9*/
/****************************************工作循环相关变量****************************************/

uint16_t    nucSysStartYear = 0;    /*工作循环时间-起始年  2020-03-22*/
uint8_t nucSysStartMonth = 0;   /*工作循环时间-起始月  2020-03-22*/
uint8_t nucSysStartDay = 0;         /*工作循环时间-起始日  2020-03-22*/
uint8_t nucSysStartHour = 0;    /*工作循环时间-起始时  2020-03-22*/
uint8_t nucSysStartMinute = 0;      /*工作循环时间-起始分  2020-03-22*/
uint8_t nucSysStartSecond = 0;      /*工作循环时间-起始秒  2020-03-22*/

uint16_t    nucSysEndYear = 0;          /*工作循环时间-终止年  2020-03-22*/
uint8_t nucSysEndMonth = 0;         /*工作循环时间-终止月  2020-03-22*/
uint8_t nucSysEndDay = 0;       /*工作循环时间-终止日  2020-03-22*/
uint8_t nucSysEndHour = 0;          /*工作循环时间-终止时  2020-03-22*/
uint8_t nucSysEndMinute = 0;    /*工作循环时间-终止分  2020-03-22*/
uint8_t nucSysEndSecond = 0;    /*工作循环时间-终止秒  2020-03-22*/

uint16_t    nucSysStartWeightData = 0;  /*工作循环时间-起始重量       2020-03-22*/
uint16_t    nucSysStartLongData = 0; /*工作循环时间-起始幅度       2020-03-22*/
uint16_t    nucSysStartHighData = 0; /*工作循环时间-起始高度       2020-03-22*/
uint16_t    nucSysStartAngleData = 0; /*工作循环时间-起始回转       2020-03-22*/
float   nucSysMaxMomentData = 0;

uint16_t    nucSysEndWeightData = 0; /*工作循环时间-终止重量       2020-03-22*/
uint16_t    nucSysEndLongData = 0;      /*工作循环时间-终止幅度       2020-03-22*/
uint16_t    nucSysEndHighData = 0;      /*工作循环时间-终止高度       2020-03-22*/
uint16_t    nucSysEndAngleData = 0;     /*工作循环时间-终止回转       2020-03-22*/

uint8_t nucSysStartWorkFlag = 0; /*工作循环开始标志位       2020-03-22*/
uint8_t nucSysWorkENdFlag = 0;      /*工作循环结束标志位       2020-03-22*/

uint8_t nucEXT6Flag = 0;            /*中断状态标识位*/
uint8_t nucEXT8Flag = 0;            /*中断状态标识位*/

uint32_t    nucSysCycleTime = 0; /*工作循环时长    2020-9-25*/
uint32_t    nucSysCycleTime_last = 0;
uint8_t nucOldTime = 0;

uint8_t nucSendDTUCounter = 0;

uint32_t    nucTowerParCounter = 36000;

static float    nucLongData_last = 0;
static float    nucHighData_last = 0;
static float    nucAngleData_last = 0;

uint8_t nucAlarmTbl_last[4] = {0};          /*报警状态2015-03-11*/
static int32_t      g_direction_new = 0; //方向

void nucSysReLayOpen(void)
{
    LONGOUTON;
    LONGINON;
    HIGHUPON;
    HIGHDOWNON;
    ANGLELEFTON;
    ANGLERIGHTON;
}

void nucSysRlayClose(void)
{
    LONGOUTOFF;
    LONGINOFF;
    HIGHUPOFF;
    HIGHDOWNOFF;
    ANGLELEFTOFF;
    ANGLERIGHTOFF;
}
uint8_t face_ctl_get(void)
{
    return g_new_bef.face_enable;
}

void nucSysParaCVLoad(void)
{
    float aucx1 = 0, aucx2 = 0, aucy1 = 0, aucy2 = 0;

    /*************重量参数计算*************************/
    aucx1 = nucWeightMinSignle;
    aucy1 = nucWeightMinData;
    aucy1 = aucy1 / 100;
    aucx2 = nucWeightMaxSignle;
    aucy2 = nucWeightMaxData;
    aucy2 = aucy2 / 100;
    if (aucx1 != aucx2)
    {
        nucWeightSlope = (aucy1 - aucy2) / (aucx1 - aucx2);
        nucWeightCV = aucy1 - nucWeightSlope * aucx1;
    }

    /*************幅度参数计算*************************/
    aucx1 = nucLongMinSignle;
    aucy1 = nucLongMinData;
    aucy1 = aucy1 / 100;
    aucx2 = nucLongMaxSignle;
    aucy2 = nucLongMaxData;
    aucy2 = aucy2 / 100;
    LOG("aucy1=%f,aucy2=%f\n", aucy1, aucy2);
    if (aucx1 != aucx2)
    {
        nucLongSlope = (aucy1 - aucy2) / (aucx1 - aucx2);
        nucLongCV = aucy1 - nucLongSlope * aucx1;
        LOG("nucLongSlope=%f,nucLongCV=%f\n", nucLongSlope, nucLongCV);
    }

    /*************高度参数计算*************************/
    aucx1 = nucHighMinSignle;
    aucy1 = nucHighMinData;
    aucy1 = aucy1 / 100;
    aucx2 = nucHighMaxSignle;
    aucy2 = nucHighMaxData;
    aucy2 = aucy2 / 100;
    if (aucx1 != aucx2)
    {
        nucHighSlope = (aucy1 - aucy2) / (aucx1 - aucx2);
        nucHighCV = aucy1 - nucHighSlope * aucx1;
    }

    /*************回转参数计算*************************/
    aucx1 = nucAngleMinSignle;
    aucy1 = nucAngleMinData;
    aucy1 = aucy1;
    aucx2 = nucAngleMaxSignle;
    aucy2 = nucAngleMaxData;
    aucy2 = aucy2;
    if (aucx1 != aucx2)
    {
        nucAngleSlope = (aucy1 - aucy2) / (aucx1 - aucx2);
        nucAngleCV = aucy1 - nucAngleSlope * aucx1;
    }

    /*************力矩曲线参数计算*************************/
    aucx1 = nucTowerStartLong;
    aucx1 = aucx1 / 100;
    aucy1 = nucTowerStartWeight;
    aucy1 = aucy1 / 1000;

    aucx2 = nucTowerMidLong;
    aucx2 = aucx2 / 100;
    aucy2 = nucTowerMidWeight;
    aucy2 = aucy2 / 1000;
    if (aucx1 != aucx2)
    {
        nucTowerSlope1 = (aucy1 - aucy2) / (aucx1 - aucx2);
        nucTowerCV1 = aucy1 - nucTowerSlope1 * aucx1;
    }

    aucx1 = nucTowerMidLong;
    aucx1 = aucx1 / 100;
    aucy1 = nucTowerMidWeight;
    aucy1 = aucy1 / 1000;

    aucx2 = nucTowerEndLong;
    aucx2 = aucx2 / 100;
    aucy2 = nucTowerEndWeight;
    aucy2 = aucy2 / 1000;
    if (aucx1 != aucx2)
    {
        nucTowerSlope2 = (aucy1 - aucy2) / (aucx1 - aucx2);
        nucTowerCV2 = aucy1 - nucTowerSlope2 * aucx1;
    }
}
//从flash中读取默认值
void APP_USER_Config_Init(void)
{
    uint8_t   auccounter = 0;
#if 1
    uint32_t test = bsp_eeprom_read_word(FLASH_ENABLE);
//        LOG("test: %d\n",test);
    if (test != 123456) //写入默认值
    {
        /////////20250625//新板把风速传感器的0点置为0////////////
        bsp_eeprom_write_word(FLASHWINDZERO, 0);
        nucWindZero = 0 ;
        ////////20250625///////////
        bsp_eeprom_write_word(FLASH_ENABLE, 123456);
        bsp_eeprom_write_word(APP_DTU_IP_STATUS, 0);

        g_app_config.app_config_sys.device_id = 0;
        bsp_eeprom_write_word(4 * 450, 0);
        bsp_eeprom_write_word(FLASH_RENSHU, 1);
    }
    else
    {

        g_app_sys.ip_status = BSP_EEPROM_ReadU8(APP_DTU_IP_STATUS);
        g_app_config.app_config_sys.device_id = bsp_eeprom_read_word(FLASHID);                    /*设备ID-8034     2020-03-22*/
//        LOG("device id: %d,ip updata status[%d]\n",g_app_config.app_config_sys.device_id,g_app_sys.ip_status);
//        bsp_eeprom_read_word(FLASH_ENABLE);
    }
#else
    g_app_config.app_config_sys.device_id = bsp_eeprom_read_word(FLASHID);                    /*设备ID-8034     2020-03-22*/
    LOG("device id: %05d\n", g_app_config.app_config_sys.device_id);
#endif

    g_face_peo.peo_sum = bsp_eeprom_read_word(FLASH_RENSHU);
    if (g_face_peo.peo_sum > 10000)
    {
        g_face_peo.peo_sum = 1;
    }

    if (BSP_EEPROM_ReadU8(APP_IAP_ADDR_RESET) > 5)
    {
        BSP_EEPROM_WriteU8(APP_IAP_ADDR_RESET, 0);
    }
//      return;
    DWIN_PASSWORD = bsp_eeprom_read_word(FLASH_DWIN_PASSWORD); //读迪文密码
    ALARM_OFF_FLG = bsp_eeprom_read_word(FLASHALARMOFFFLG);//语音开关
    nucWeightMinData = bsp_eeprom_read_word(FLASHMINWEIGHTDATA);      /*起重量量程下限-8000    2020-03-22*/
    nucWeightMinSignle = bsp_eeprom_read_word(FLASHMINWEIGHTSIGNLE); /*起重量信号下限-8002    2020-03-22*/
    nucWeightMaxData = bsp_eeprom_read_word(FLASHMAXWEIGHTDATA);      /*起重量量程上限-8004    2020-03-22*/
    nucWeightMaxSignle = bsp_eeprom_read_word(FLASHMAXWEIGHTSIGNLE); /*起重量信号上限-8006    2020-03-22*/

    nucLongMinData = bsp_eeprom_read_word(FLASHMINLONGDATA);          /*幅度量程下限-8008   2020-03-22*/
    nucLongMinSignle = bsp_eeprom_read_word(FLASHMINLONGSIGNLE);      /*幅度信号下限-800A   2020-03-22*/
//    LOG("nucLongMinSignle=%d\n",nucLongMinSignle);
    nucLongMaxData = bsp_eeprom_read_word(FLASHMAXLONGDATA);          /*幅度量程上限-800C   2020-03-22*/
    nucLongMaxSignle = bsp_eeprom_read_word(FLASHMAXLONGSIGNLE);      /*幅度信号上限-800E   2020-03-22*/
//    LOG("nucLongMaxSignle=%d\n",nucLongMaxSignle);

    nucHighMinData = bsp_eeprom_read_word(FLASHMINHIGHDATA);          /*高度量程下限-8010   2020-03-22*/
    nucHighMinSignle = bsp_eeprom_read_word(FLASHMINHIGHSIGNLE);      /*高度信号下限-8012   2020-03-22*/
    nucHighMaxData = bsp_eeprom_read_word(FLASHMAXHIGHDATA);          /*高度量程上限-8014   2020-03-22*/
    nucHighMaxSignle = bsp_eeprom_read_word(FLASHMAXHIGHSIGNLE);      /*高度信号上限-8016   2020-03-22*/

    nucAngleMinData = bsp_eeprom_read_word(FLASHMINANGLEDATA);    /*回转量程下限-8018   2020-03-22*/
    nucAngleMinSignle = bsp_eeprom_read_word(FLASHMINANGLESIGNLE); /*回转信号下限-801A   2020-03-22*/
    nucAngleMaxData = bsp_eeprom_read_word(FLASHMAXANGLEDATA);    /*回转量程上限-801C   2020-03-22*/
    nucAngleMaxSignle = bsp_eeprom_read_word(FLASHMAXANGLESIGNLE); /*回转信号上限-801E   2020-03-22*/

    nucWindZero = bsp_eeprom_read_word(FLASHWINDZERO);            /*风速零点-8020     2020-03-22*/
    nucWindCV = bsp_eeprom_read_word(FLASHWINDCV);                /*风速系数-8022     2020-03-22*/

    nucDispAngleXZero = bsp_eeprom_read_word(FLASHDISPANGLEXZERO); /*倾角X零点-8024        2020-03-22*/
    nucDispAngleYZero = bsp_eeprom_read_word(FLASHDISPANGLEYZERO); /*倾角Y零点-8026        2020-03-22*/




    nucTowerStartWeight = bsp_eeprom_read_word(FLASHSTARTWEIGHT); /*力矩曲线起始重量-8028 2020-03-22*/
    nucTowerStartLong = bsp_eeprom_read_word(FLASHSTARTLONG);     /*力矩曲线起始幅度-802A     2020-03-22*/
    nucTowerMidWeight = bsp_eeprom_read_word(FLASHMIDWEIGHT);     /*力矩曲线中间重量-802C     2020-03-22*/
    nucTowerMidLong = bsp_eeprom_read_word(FLASHMIDLONG);         /*力矩曲线中间幅度-802E     2020-03-22*/
    nucTowerEndWeight = bsp_eeprom_read_word(FLASHENDWEIGHT);     /*力矩曲线终止重量-8030     2020-03-22*/
    nucTowerEndLong = bsp_eeprom_read_word(FLASHENDLONG);         /*力矩曲线终止幅度-8032     2020-03-22*/

    nucTowerLong = bsp_eeprom_read_word(FLASHTOWERLONG);              /*塔机臂长-8036 2020-03-22*/
//    LOG("nucTowerLong: %d\n",nucTowerLong);
    nucTowerMaxHigh = bsp_eeprom_read_word(FLASHTOWERHIGH);       /*塔机高度-8038 2020-03-22*/
    nucTowerPHLong = bsp_eeprom_read_word(FLASHTOWERPHLONG);          /*塔机平衡臂长-803A   2020-03-22*/
    nucTowerTMHigh = bsp_eeprom_read_word(FLASHTOWERTMHIGH);          /*塔机塔帽高度-803C   2020-03-22*/
    nucTowerBL = bsp_eeprom_read_word(FLASHTOWERBL);                  /*塔机倍率-803E 2020-03-22*/

    g_remote_lock =  bsp_eeprom_read_word(FLASH_REMOTE_LOCK);
    nucSendDTUTime =  bsp_eeprom_read_word(FLASH_DTU_SEND);
    nucSendDTUTime_Work =  bsp_eeprom_read_word(FLASH_DTU_SEND_WORK);
    g_charg_board.id =  bsp_eeprom_read_word(FLASH_CHARGING_BOARD_ID);
    g_charg_board.switch_charge =    bsp_eeprom_read_word(FLASH_CHARGING_BOARD_SW);
    g_charg_board.switch_laser =    bsp_eeprom_read_word(FLASH_CHARGING_LASER_SW);

    g_charg_board.work_mode_ipc =   bsp_eeprom_read_word(FLASH_CHARGING_WORK_MODE);
    if (g_charg_board.work_mode_ipc == 1) //正常和强制模式
    {
        g_charg_board.work_mode = 0;//低功耗模式
    }
    else
    {
        g_charg_board.work_mode = g_charg_board.work_mode_ipc;
    }

    g_charg_board.work_mode_laser_ipc = bsp_eeprom_read_word(FLASH_CHARGING_LASER_MODE);

    g_new_bef.site_num = bsp_eeprom_read_word(FLASH_SITE_NUM);
    g_new_bef.range_type = bsp_eeprom_read_word(FLASH_RANGE_TYPE);
    g_new_bef.height_type = bsp_eeprom_read_word(FLASH_HEIGHT_TYPE);
    g_new_bef.rotation_type = bsp_eeprom_read_word(FLASH_ROTATION_TYPE);
    g_new_bef.cunstom_num = bsp_eeprom_read_word(FLASH_CUSTOM_NUM);
    g_new_bef.twist_cal = bsp_eeprom_read_word(FLASH_TWIST_CAL);

//        bsp_eeprom_write_word(FLASH_Z_ZERO,100);
    g_new_bef.z_zero = bsp_eeprom_read_word(FLASH_Z_ZERO);
//    LOG("zero=%d\n",g_new_bef.z_zero);
    g_new_bef.face_enable = bsp_eeprom_read_word(FLASH_FACE_ENABLE);

    g_new_bef.face_num = bsp_eeprom_read_word(FLASH_FACE_NUM);
    if (g_new_bef.face_num > 2)
    {
        g_new_bef.face_num = 1;
    }
    g_new_bef.pic_enable =   bsp_eeprom_read_word(FLASH_PIC_ENABLE);

    g_bsp_adc.vol_bat_status =   bsp_eeprom_read_word((4 * 238)); //读取电池 是否存在 //临时用

    for (auccounter = 0; auccounter < 8; auccounter++)
    {
        nucDanJStartLongTbl[auccounter] = bsp_eeprom_read_word(FLASHDANJI1STARTLONG + 2 * auccounter);
        /*单机防碰撞起始幅度-8050、8052、8054、8056、8058、805A、805C、805E*/

        nucDanJEndLongTbl[auccounter] = bsp_eeprom_read_word(FLASHDANJI1ENDLONG + 2 * auccounter);
        /*单机防碰撞终止幅度-8060、8062、8064、8066、8068、806A、806C、806E*/

//        nucDanJStartHighTbl[auccounter] = bsp_eeprom_read_word(FLASHDANJI1STARTHIGH + 2 * auccounter);
//        /*单机防碰撞起始高度-8070、8072、8074、8076、8078、807A、807C、807E*/

        nucDanJEndHighTbl[auccounter] = bsp_eeprom_read_word(FLASHDANJI1ENDHIGH + 2 * auccounter);
        /*单机防碰撞终止高度-8080、8082、8084、8086、8088、808A、808C、808E - 地址已被仰角传感器参数占用*/

        nucDanJStartAngleTbl[auccounter] = bsp_eeprom_read_word(FLASHDANJI1STARTANGLE + 2 * auccounter);
        /*单机防碰撞起始角度-8090、8092、8094、8096、8098、809A、809C、809E*/

        nucDanJEndAngleTbl[auccounter] = bsp_eeprom_read_word(FLASHDANJI1ENDANGLE + 2 * auccounter);
        /*单机防碰撞终止角度-80A0、80A2、80A4、80A6、80A8、80AA、80AC、80AE*/

        nucDuoJIDTbl[auccounter] = bsp_eeprom_read_word(FLASHDUOJI1ID + 2 * auccounter);
        /*多机防碰撞设置ID-80B0、80B2、80B4、80B6、80B8、80BA、80BC、80BE*/

        nucDuoJTowerLongTbl[auccounter] = bsp_eeprom_read_word(FLASHDUOJI1MAXLONG + 2 * auccounter);
        /*多机防碰撞塔机臂长-80C0、80C2、80C4、80C6、80C8、80CA、80CC、80CE*/

        nucDuoJXDDisdanceTbl[auccounter] = bsp_eeprom_read_word(FLASHDUOJI1DISDANCE + 2 * auccounter);
        /*多机防碰撞设置距离-80D0、80D2、80D4、80D6、80D8、80DA、80DC、80DE*/

        nucDuoJXDAngleTbl[auccounter] = bsp_eeprom_read_word(FLASHDUOJI1XDANGLE + 2 * auccounter);
        /*多机防碰撞设置相对角度-80E0、80E2、80E4、80E6、80E8、80EA、80EC、80EE*/

        nucDuoJXDHighTbl[auccounter] = bsp_eeprom_read_word(FLASHDUOJI1XDHIGH + 2 * auccounter);
        /*多机防碰撞设置相对高度-80F0、80F2、80F4、80F6、80F8、80FA、80FC、80FE*/
        nucDanJStartAngleTbl_Reference[auccounter] = bsp_eeprom_read_word(FLASHDANJI1STARTANGLE_TEMP + 2 * auccounter);


        nucDuoJTowerLongTbl_temp[auccounter] = bsp_eeprom_read_word(FLASHDUOLONG_TEMP1 + 2 * auccounter); /*幅度比较误差*/
        nucDuoJXDHighTbl_temp[auccounter] = bsp_eeprom_read_word(FLASHDUOHIGH_TEMP1 + 2 * auccounter); /*塔机高度比较误差*/
        nucDuoJTowerLongTbl_back[auccounter] = bsp_eeprom_read_word(FLASHDUOHIGH_BACK1 + 2 * auccounter); /*多机防碰撞塔机后臂长 */

    }

    // 读取仰角传感器Z轴零点值
    nucDispAngleZZero1_set =    bsp_eeprom_read_word(FLASHZSET);
    nucDispAngleZZero1 = bsp_eeprom_read_word(FLASHDANJI1STARTHIGH);
    nucDanJStartHighTbl[0] = nucDispAngleZZero1;
    g_yj_warn[0] =  bsp_eeprom_read_word(FLASHDANJI2STARTHIGH);
    nucDanJStartHighTbl[1] =  g_yj_warn[0];
    g_yj_alarm[0] =  bsp_eeprom_read_word(FLASHDANJI3STARTHIGH);
    nucDanJStartHighTbl[2] =  g_yj_alarm[0];
    g_yj_warn[1] =   bsp_eeprom_read_word(FLASHDANJI4STARTHIGH);
    nucDanJStartHighTbl[3] =  g_yj_warn[1];
    g_yj_alarm[1] =  bsp_eeprom_read_word(FLASHDANJI5STARTHIGH);
    nucDanJStartHighTbl[4] =  g_yj_alarm[1];
    g_dj_leixing =   bsp_eeprom_read_word(FLASHDANJI6STARTHIGH);
    nucDanJStartHighTbl[5] =  g_dj_leixing;
    BenJiBianHao = bsp_eeprom_read_word(FLASHBENJIBIANHAO);   //本机编号          -8102               2020.11.02
    for (auccounter = 0; auccounter < 8; auccounter++)
    {
        //多机编号            -8104、8106、8108、810A、810C、810E、8110、8112                  2020.11.02
        DuoJiBianHao[auccounter] = bsp_eeprom_read_word(FLASHDUOJI1BIANHAO + 2 * auccounter);
    }
#ifdef  AIO
#else
    uint16_t *sensor = (uint16_t *)&g_sensor_able;
    uint16_t *alarm = (uint16_t *)&g_alarm_able;
    *sensor = bsp_eeprom_read_word(FLASHSENSORSTATUS);    //报警静音标志    -8100               2020.11.02
    *alarm = bsp_eeprom_read_word(FLASHALARMSTATUS);      //本机编号          -8102               2020.11.02
//      LOG("*alarm=%d\n",*alarm);
#endif
    nucControlFlag = g_sensor_able.control;
    for (auccounter = 0; auccounter < 8; auccounter++)
    {
        //多机编号            -8104、8106、8108、810A、810C、810E、8110、8112                  2020.11.02
        DuoJiBianHao[auccounter] = bsp_eeprom_read_word(FLASHDUOJI1BIANHAO + 2 * auccounter);
    }
    WIND_ALARM_OFF = bsp_eeprom_read_word(FLASHWINDALARMOFF);             //风速报警启用标志  -8114       2020.11.02
    QJ_ALARM_OFF = bsp_eeprom_read_word(FLASHQJALARMOFF);                     //倾角报警启用标志  -8116       2020.11.02
    WEIGHT_ALARM_OFF = bsp_eeprom_read_word(FLASHWEIGHTALARMOFF);     //重量报警启用标志  -8118       2020.11.02
    QJ_ALARM_DATA = bsp_eeprom_read_word(FLASHQJALARMDATA);               //倾角报警值               -811A       2020.11.20
    QJ_ALARM_DATA_1 = bsp_eeprom_read_word(FLASHQJALARMDATA_1);               //倾角预警值               -811A       2020.11.20

    WIND_ALARM_DATA = bsp_eeprom_read_word(FLASHWINDALARMDATA);               //风速报警值               -811A       2020.11.20
    WIND_ALARM_DATA_1 = bsp_eeprom_read_word(FLASHWINDALARMDATA_1);               //风速预警值               -811A       2020.11.20

    LONG_ALARM_DATA = bsp_eeprom_read_word(FLASHLONGALARMDATA);               //幅度报警值               -811A       2020.11.20
    LONG_ALARM_DATA_1 = bsp_eeprom_read_word(FLASHLONGALARMDATA_1);               //幅度预警值               -811A       2020.11.20

    HIGH_ALARM_DATA = bsp_eeprom_read_word(FLASHHIGHALARMDATA);               //高度报警值               -811A       2020.11.20
    HIGH_ALARM_DATA_1 = bsp_eeprom_read_word(FLASHHIGHALARMDATA_1);               //高度预警值               -811A       2020.11.20

    DUOJI_ALARM_DATA_1 =   bsp_eeprom_read_word(FLASHDUOJIALARMDATA_1); //多级预警值
    DUOJI_ALARM_DATA =   bsp_eeprom_read_word(FLASHDUOJIALARMDATA);//多级报警值

    nucHighMode = bsp_eeprom_read_word(FLASHHIGHMODE);                        //高度零点位置            -811C       2020.12.18

    g_weight_board.connection_type = bsp_eeprom_read_word(FLASH_WEIGHT_MODE);//重量传感器类型 1：无线 0：有线（默认）
    g_weight_board.id = bsp_eeprom_read_word(FLASH_WEIGHT_ID);//重量传感器类型 1：无线 0：有线（默认）
//    for(auccounter=0; auccounter<3; auccounter++)
//    {
//        if(bsp_eeprom_read_word(FLASH_DTU_PORT0+auccounter*4) != 0)
//        {
//            APP_DTU_AT_Ip_Get();
//            break;
//        }
//    }
    if (g_app_config.app_config_sys.device_id > 0)
    {
        memset(&g_app_dtu_ip, 0, sizeof(app_dtu_ip_def) * 4);
        BSP_EEPROM_ReadBuf(FLASH_DTU_IP, (uint8_t*)&g_app_dtu_ip, sizeof(app_dtu_ip_def) * 4);
#if 0
        for (int i = 0; i < 4; i++)
        {
            LOG("read ip[%d]:\"%s\",port:%d,en:%d\n", i, g_app_dtu_ip[i].ip, g_app_dtu_ip[i].port, g_app_dtu_ip[i].en);
        }
#endif
    }
    g_stop_delay = bsp_eeprom_read_word(FLASH_STOP_DELAY);
    if (g_stop_delay == 0)
    {
        g_stop_delay = 60;
    }
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if (g_app_config.app_config_sys.device_id == 0)                       //如果设备ID号为0, 所有参数都使用默认值                     2020.11.30
    {
        g_app_config.app_config_sys.device_id = 9999;                             /*设备ID号*/
        bsp_eeprom_write_word(FLASHID, g_app_config.app_config_sys.device_id);
        g_weight_board.id = g_app_config.app_config_sys.device_id;
        bsp_eeprom_write_word(FLASH_WEIGHT_ID, g_weight_board.id);
        BSP_EEPROM_WriteBuf(FLASH_REMOTE, (uint8_t *)&g_dtu_remote, sizeof(dtu_remote_def));

        uint8_t *dtu_remote = (uint8_t *)&g_dtu_remote;
        BSP_EEPROM_ReadBuf(FLASH_REMOTE, dtu_remote, sizeof(dtu_remote_def));

        LOG("sn:%02d ", g_dtu_remote.uint_sn);
        LOG("model:%02d ", g_dtu_remote.uint_model);
        LOG("ver:%02d\n", g_dtu_remote.uint_ver);

        BSP_EEPROM_WriteBuf(FLASH_OTA_MODEL, (uint8_t *)g_app_sys.ota_model, 20);
        uint8_t dtu_model[20];
        BSP_EEPROM_ReadBuf(FLASH_OTA_MODEL, dtu_model, 20);
        LOG("\nhead model:%s\n", dtu_model);
#ifdef  AIO
#else
        *sensor = 0xFFFF;
        *alarm = 0xFFFF;
        nucControlFlag = g_sensor_able.control;
        bsp_eeprom_write_word(FLASHCONTROL, nucControlFlag);
        bsp_eeprom_write_word(FLASHSENSORSTATUS, *sensor);
        bsp_eeprom_write_word(FLASHALARMSTATUS, *alarm);
//    uint16_t  tests = bsp_eeprom_read_word(FLASHALARMSTATUS);
//    LOG("*alarm=%d\n",tests);
#endif
        user_cfg_warn_ctl = 1;
        bsp_eeprom_write_word(FLASH_WARN_CTL, user_cfg_warn_ctl);

        g_new_bef.range_type = 2;
        bsp_eeprom_write_word(FLASH_RANGE_TYPE, g_new_bef.range_type); //幅度传感器类型
        ALARM_OFF_FLG = 1;
        bsp_eeprom_write_word(FLASHALARMOFFFLG, ALARM_OFF_FLG); //语音开关
        DWIN_PASSWORD = 123456;
        bsp_eeprom_write_word(FLASH_DWIN_PASSWORD, DWIN_PASSWORD);
        nucTowerLong = 5000;                  /*塔机臂长*/
        bsp_eeprom_write_word(FLASHTOWERLONG, nucTowerLong);
        nucTowerMaxHigh = 5000;           /*塔机高度*/
        bsp_eeprom_write_word(FLASHTOWERHIGH, nucTowerMaxHigh);
        nucTowerPHLong = 0;                   /*塔机平衡臂长*/
        bsp_eeprom_write_word(FLASHTOWERPHLONG, nucTowerPHLong);
        nucTowerTMHigh = 0;                   /*塔机塔帽高度*/
        bsp_eeprom_write_word(FLASHTOWERTMHIGH, nucTowerTMHigh);

        nucTowerStartWeight = 5000;   /*力矩曲线起始重量*/
        bsp_eeprom_write_word(FLASHSTARTWEIGHT, nucTowerStartWeight);
        nucTowerStartLong = 5000;         /*力矩曲线起始幅度*/
        bsp_eeprom_write_word(FLASHSTARTLONG, nucTowerStartLong);
        nucTowerMidWeight = 2000;         /*力矩曲线中间重量*/
        bsp_eeprom_write_word(FLASHMIDWEIGHT, nucTowerMidWeight);
        nucTowerMidLong = 2000;           /*力矩曲线中间幅度*/
        bsp_eeprom_write_word(FLASHMIDLONG, nucTowerMidLong);
        nucTowerEndWeight = 1000;         /*力矩曲线终止重量*/
        bsp_eeprom_write_word(FLASHENDWEIGHT, nucTowerEndWeight);
        nucTowerEndLong = 5000;           /*力矩曲线终止幅度*/
        bsp_eeprom_write_word(FLASHENDLONG, nucTowerEndLong);

        nucWindCV = 60;                           /*风速系数*/
        bsp_eeprom_write_word(FLASHWINDCV, nucWindCV);
        QJ_ALARM_DATA = 300;                  /*倾角报警值*/
        bsp_eeprom_write_word(FLASHQJALARMDATA, QJ_ALARM_DATA);
        QJ_ALARM_DATA_1 = 200;                /*倾角预警值*/
        bsp_eeprom_write_word(FLASHQJALARMDATA_1, QJ_ALARM_DATA_1);

        WIND_ALARM_DATA = 10.8 * 100;             /*风速报警值*/
        bsp_eeprom_write_word(FLASHWINDALARMDATA, WIND_ALARM_DATA);
        WIND_ALARM_DATA_1 = 8.8 * 100;                /*风速预警值*/
        bsp_eeprom_write_word(FLASHWINDALARMDATA_1, WIND_ALARM_DATA_1);

        LONG_ALARM_DATA = 1000;               /*幅度报警值*/
        bsp_eeprom_write_word(FLASHLONGALARMDATA, LONG_ALARM_DATA);
        LONG_ALARM_DATA_1 = 200;                  /*幅度预警值*/
        bsp_eeprom_write_word(FLASHLONGALARMDATA_1, LONG_ALARM_DATA_1);

        HIGH_ALARM_DATA = 1000;               /*高度报警值*/
        bsp_eeprom_write_word(FLASHHIGHALARMDATA, HIGH_ALARM_DATA);
        HIGH_ALARM_DATA_1 = 200;                  /*高度预警值*/
        bsp_eeprom_write_word(FLASHHIGHALARMDATA_1, HIGH_ALARM_DATA_1);
        DUOJI_ALARM_DATA_1 = 2000;
        bsp_eeprom_write_word(FLASHDUOJIALARMDATA_1, DUOJI_ALARM_DATA_1);
        DUOJI_ALARM_DATA = 1000;
        bsp_eeprom_write_word(FLASHDUOJIALARMDATA, DUOJI_ALARM_DATA);
        bsp_eeprom_write_word(FLASHPASSWORDATA, g_app_config.app_config_sys.password);

        nucDanJStartLongTbl[0] = 0;   /*单机防碰撞起始幅度*/
        bsp_eeprom_write_word(FLASHDANJI1STARTLONG, nucDanJStartLongTbl[0]);
        nucDanJEndLongTbl[0] = 5000;  /*单机防碰撞终止幅度*/
        bsp_eeprom_write_word(FLASHDANJI1ENDLONG, nucDanJEndLongTbl[0]);
        nucDanJStartHighTbl[0] = 0;   /*单机防碰撞起始高度*/
        bsp_eeprom_write_word(FLASHDANJI1STARTHIGH, nucDanJStartHighTbl[0]);
        nucDanJEndHighTbl[0] = 0;  /*单机防碰撞终止高度*/
        bsp_eeprom_write_word(FLASHDANJI1ENDHIGH, nucDanJEndHighTbl[0]); // 地址已被仰角传感器参数占用
        nucDanJStartAngleTbl[0] = 0;  /*单机防碰撞起始角度*/
        bsp_eeprom_write_word(FLASHDANJI1STARTANGLE, nucDanJStartAngleTbl[0]);
        nucDanJEndAngleTbl[0] = 50;   /*单机防碰撞终止角度*/
        bsp_eeprom_write_word(FLASHDANJI1ENDANGLE, nucDanJEndAngleTbl[0]);

        nucDanJStartAngleTbl_Reference[0] = ((180 - (nucDanJEndAngleTbl[0] - nucDanJStartAngleTbl[0])) / 2);
        bsp_eeprom_write_word(FLASHDANJI1STARTANGLE_TEMP, nucDanJStartAngleTbl_Reference[0]);


        nucDuoJTowerLongTbl_temp[0] = 0; /*多机防碰撞 幅度比较误差*/
        bsp_eeprom_write_word(FLASHDUOLONG_TEMP1, nucDuoJTowerLongTbl_temp[0]);
        nucDuoJXDHighTbl_temp[0] = 0; /*多机防碰撞 高度比较误差*/
        bsp_eeprom_write_word(FLASHDUOHIGH_TEMP1, nucDuoJXDHighTbl_temp[0]);
        nucDuoJTowerLongTbl_back[0] = 0; /*多机防碰撞 后臂长*/
        bsp_eeprom_write_word(FLASHDUOHIGH_BACK1, nucDuoJTowerLongTbl_back[0]);


        nucWeightMinData = 0;                 /*起重量量程下限*/
        bsp_eeprom_write_word(FLASHMINWEIGHTDATA, nucWeightMinData);
        nucWeightMinSignle = 32000;   /*起重量信号下限*/
        bsp_eeprom_write_word(FLASHMINWEIGHTSIGNLE, nucWeightMinSignle);
        nucWeightMaxData = 1000;          /*起重量量程上限*/
        bsp_eeprom_write_word(FLASHMAXWEIGHTDATA, nucWeightMaxData);
        nucWeightMaxSignle = 40000;   /*起重量信号上限*/
        bsp_eeprom_write_word(FLASHMAXWEIGHTSIGNLE, nucWeightMaxSignle);

        nucLongMinData = 0;                   /*幅度量程下限*/
        bsp_eeprom_write_word(FLASHMINLONGDATA, nucLongMinData);
        nucLongMinSignle = 2232360;       /*幅度信号下限*/
        bsp_eeprom_write_word(FLASHMINLONGSIGNLE, nucLongMinSignle);
        nucLongMaxData = 5000;            /*幅度量程上限*/
        bsp_eeprom_write_word(FLASHMAXLONGDATA, nucLongMaxData);
        nucLongMaxSignle = 2342360;       /*幅度信号上限*/
        bsp_eeprom_write_word(FLASHMAXLONGSIGNLE, nucLongMaxSignle);

        nucHighMinData = 0;                   /*高度量程下限*/
        bsp_eeprom_write_word(FLASHMINHIGHDATA, nucHighMinData);
        nucHighMinSignle = 10000;         /*高度信号下限*/
        bsp_eeprom_write_word(FLASHMINHIGHSIGNLE, nucHighMinSignle);
        nucHighMaxData = 5000;            /*高度量程上限*/
        bsp_eeprom_write_word(FLASHMAXHIGHDATA, nucHighMaxData);
        nucHighMaxSignle = 12000;         /*高度信号上限*/
        bsp_eeprom_write_word(FLASHMAXHIGHSIGNLE, nucHighMaxSignle);

        nucAngleMinData = 0;                  /*回转量程下限*/
        bsp_eeprom_write_word(FLASHMINANGLEDATA, nucAngleMinData);
        nucAngleMinSignle = 0;            /*回转信号下限*/
        bsp_eeprom_write_word(FLASHMINANGLESIGNLE, nucAngleMinSignle);
        nucAngleMaxData = 360;            /*回转量程上限*/
        bsp_eeprom_write_word(FLASHMAXANGLEDATA, nucAngleMaxData);
        nucAngleMaxSignle = 5555;         /*回转信号上限*/
        bsp_eeprom_write_word(FLASHMAXANGLESIGNLE, nucAngleMaxSignle);

        nucHighMode = 0;                          /*高度零点位置*/            //2020.12.18
        bsp_eeprom_write_word(FLASHHIGHMODE, nucHighMode);

        // 仰角传感器相关参数默认值初始化 2025-07-24

        g_charg_board.work_mode_laser_ipc = 1;
        bsp_eeprom_write_word(FLASH_CHARGING_LASER_SW, g_charg_board.work_mode_laser_ipc);

        nucSendDTUTime = 60;
        nucSendDTUTime_Work = 10;
        bsp_eeprom_write_word(FLASH_DTU_SEND, nucSendDTUTime);
        bsp_eeprom_write_word(FLASH_DTU_SEND_WORK, nucSendDTUTime_Work);

        g_new_bef.rotation_type = 0;//默认绝对值
        bsp_eeprom_write_word(FLASH_ROTATION_TYPE, g_new_bef.rotation_type);

        for (auccounter = 0; auccounter < 4; auccounter++) //初始化IP
        {
//            APP_CONFIG_Get_Ip_Chl(1+auccounter);
            APP_CONFIG_Ip_Save(g_app_dtu_ip[auccounter]);
        }

        g_stop_delay = 60;//默认60分
        bsp_eeprom_write_word(FLASH_STOP_DELAY, g_stop_delay);

        g_new_bef.face_enable = 0;
        bsp_eeprom_write_word(FLASH_FACE_ENABLE, g_new_bef.face_enable);


        g_new_bef.face_num = 1;
        bsp_eeprom_write_word(FLASH_FACE_NUM, g_new_bef.face_num);

        g_new_bef.pic_enable = 0;
        bsp_eeprom_write_word(FLASH_PIC_ENABLE, g_new_bef.pic_enable);
        device_model_set_();

        g_remote_lock = 0;
        bsp_eeprom_write_word(FLASH_REMOTE_LOCK, g_remote_lock);

    }

    device_model_get(g_version_model);
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    nucSysParaCVLoad();

    APP_DTU_Remote_Head_Init();
}

void nucSysADDataLoad(void)
{
//    if(nucAD3Counter>=ADS1115_READ_SUM)
//    {
//        if(g_sensor_able.rotation == 1)
//        {
//            nucAngleDataLoad();
//        }
//        else
//        {
//            nucAngleSignle=0;
//        }
//        if(g_sensor_able.weight == 1)
//        {
//            nucWeightAndQuceDataLoad();
//        }
//        else
//        {
//            nucWeightSignle=0;
//        }
//        if(g_sensor_able.height == 1)
//        {
//            nucHighDataLoad();
//        }
//        else
//        {
//            nucHighSignle=0;
//        }
//        if(g_sensor_able.range == 1)
//        {
//            nucLongDataLoad();
//        }
//        else
//        {
//            nucLongSignle = 0;
//        }
//    }
}
void APP_USER_Work_Freq(void)//工作频率
{
    uint8_t status = 0;

    if (fabs(nucWeightData - nucWeightData_last1) >= 0.4)
    {
        status = 1;
        nucWeightData_last1 = nucWeightData;
    }
    if (fabs(nucLongData - nucLongData_last1) >= 1.0)
    {
        status = 1;
        nucLongData_last1 = nucLongData;
    }
    if (fabs(nucHighData - nucHighData_last1) >= 1.0)
    {
        status = 1;
        nucHighData_last1 = nucHighData;
    }
    if (status == 1)
    {
        g_work_freq = 1;
        g_start_ticks = BSP_TIMER_Ticks_Get();
        g_board_work_freq = 1;
        g_bosrd_start_ticks =  BSP_TIMER_Ticks_Get();
    }

    if (BSP_TIMER_Ticks_Get() - g_bosrd_start_ticks > 10000 * 6 * 30) //30分
    {
        g_board_work_freq = 0;
    }

    if (BSP_TIMER_Ticks_Get() - g_start_ticks > nucSendDTUTime_Work * 1000) //10s
    {
        g_work_freq = 0;
    }
}


//速度计算  变幅速度一般为 10~30m/min
static uint32_t g_user_fd_num = 0;
static float g_user_fd = 0;
static uint64_t g_fd_time1 = 0, g_fd_time2 = 0;
void APP_USER_Fudu_Speed(void)
{
    uint64_t temp_time = BSP_TIMER_Ticks_Get();
    if (temp_time > g_fd_time2)
    {
        g_fd_time1 += temp_time - g_fd_time2 + 4; //累计时间  ms
        g_user_fd = fabs(g_app_dtu_new.range_data_last - nucLongData); //幅度差 m
        if (g_user_fd > 0.012) //防抖 > 12mm  变幅速度一般为 10~30m/min
        {
            if (g_app_dtu_new.range_speed > 0)
            {
                g_app_dtu_new.range_speed = g_user_fd * 1000.000f / ((float)g_fd_time1); //速度  m/s
            }
            else //去掉首次计算不准
            {
                if (g_app_dtu_new.range_data_last > 0)
                {
                    g_app_dtu_new.range_speed = 0.01;
                }
            }
            g_fd_time1 = 0;//累计时间 0
            g_user_fd_num = 0; //超时计数 0
//            LOGT("fudu speed =%f m/s\n",g_app_dtu_new.range_speed);
        }

        g_app_dtu_new.range_data_last = nucLongData; //记录当前幅度
        if (++g_user_fd_num > 20) //超时2s 清零
        {
            g_fd_time1 = 0;
            g_user_fd_num = 0;
            g_app_dtu_new.range_speed = 0;
        }
    }
    else
    {
        g_app_dtu_new.range_data_last = nucLongData; //记录当前幅度
    }

    g_fd_time2 = temp_time; //记录当前时间
}

void    nucSysDataLoad(void)
{
    float aucLong1 = 0, aucLong2 = 0, aucLong3 = 0, aucData = 0;
    int16_t   aucData1 = 0;
    if (g_sensor_able.weight == 1)
    {
        nucWeightData = nucWeightSlope * nucWeightSignle + nucWeightCV; /*重量数据计算    2020-03-22*/
        if (nucWeightData < 0)
        {
            nucWeightData = 0;
        }
    }
    else
    {
        nucWeightData = 0;
    }

    //计算单位时间
    if (g_app_dtu_new.systick > 0)
    {
        g_app_dtu_new.sec = (float)abs(g_app_dtu_new.systick - BSP_TIMER_Ticks_Get()) / (float)1000;
    }
    else
    {
        g_app_dtu_new.sec = 0;
    }


    g_app_dtu_new.systick = BSP_TIMER_Ticks_Get();

//  LOG("g_dj_leixing ==%d\n", g_dj_leixing);
    if (g_sensor_able.range == 1) /*幅度数据计算    2020-03-22*/              /*g_sensor_able.range可以设置*/
    {
        if (g_dj_leixing == 1)
        {
            nucLongData = nucTowerLong * (float)cos((PI * nucDispAngleZData1) / 180) / 100;
            //     LOG(" nucLongData = %0.1f\n", nucLongData);
        }
        else
        {
//            LOG(" nucLongData = nucLongSlope * nucLongSignle + nucLongCV; g_dj_leixing=%d\n", g_dj_leixing);
            nucLongData = nucLongSlope * nucLongSignle + nucLongCV;   /*幅度数据计算    2020-03-22*/
            if (nucLongData < 0)
            {
                nucLongData = 0;
            }
            else if (nucLongData >= nucTowerLong / 100)
            {
                nucLongData = nucTowerLong / 100;
            }

//              LOG("nucLongData=%f\n",nucLongData);
//              if(g_app_dtu_new.range_data_last != nucLongData)
//              {
//
//                  LOG("new=%f, old=%f\n",nucLongData,g_app_dtu_new.range_data_last);
//              }
            //计算方向
            if (nucLongData > g_app_dtu_new.range_data_last)
            {
                g_app_dtu_new.range_direction = 1;
            }
            else if (nucLongData < g_app_dtu_new.range_data_last)
            {
                g_app_dtu_new.range_direction = 2;
            }
            else
            {
                g_app_dtu_new.range_direction = 0;
            }
        }
        //计算速度
#if 0
        if (g_app_dtu_new.sec > 0 && g_app_dtu_new.range_data_last > 0)
        {
            g_app_dtu_new.range_speed = fabs(nucLongData - g_app_dtu_new.range_data_last) / g_app_dtu_new.sec;
        }
        g_app_dtu_new.range_data_last = nucLongData;
#else
        APP_USER_Fudu_Speed();
#endif
    }


    else
    {
        nucLongData = 0;
    }

    if (g_sensor_able.height == 1) /*高度数据计算 2020-03-22*/
    {
        nucHighData = nucHighSlope * nucHighSignle + nucHighCV;   /*高度数据计算    2020-03-22*/
        if (nucHighData < 0)
        {
            nucHighData = 0;
        }
//              if(g_app_dtu_new.height_data_last != nucHighData)
//              {
//
//                  LOG("new=%f, old=%f\n",nucHighData,g_app_dtu_new.height_data_last);
//              }
        //计算方向
        if (nucHighData > g_app_dtu_new.height_data_last)
        {
            if (nucHighMode == 1)
            {
                g_app_dtu_new.height_direction = 1;
            }
            else
            {
                g_app_dtu_new.height_direction = 2;
            }
        }
        else if (nucHighData < g_app_dtu_new.height_data_last)
        {
            if (nucHighMode == 1)
            {
                g_app_dtu_new.height_direction = 2;
            }
            else
            {
                g_app_dtu_new.height_direction = 1;
            }
        }
        else
        {
            g_app_dtu_new.height_direction = 0;
        }
        //计算速度
        if (g_app_dtu_new.sec > 0 && g_app_dtu_new.height_data_last > 0)
        {
            g_app_dtu_new.height_speed = fabs(nucHighData - g_app_dtu_new.height_data_last) / g_app_dtu_new.sec;
        }
        g_app_dtu_new.height_data_last = nucHighData;
    }
    else
    {
        nucHighData = 0;
    }


    if (g_sensor_able.rotation == 1) //绝对值/电位器
    {
        if (g_new_bef.rotation_type != 2) //绝对值/电位器
        {
            nucAngleData = nucAngleSlope * nucAngleSignle + nucAngleCV; /*回转数据计算    2020-03-22*/

            while (nucAngleData < 0)
            {
                nucAngleData = nucAngleData + 360;
            }
            aucData1 = nucAngleData;
            aucData1 = aucData1 % 360;
            nucAngleData = aucData1;
        }
        else
        {
            nucAngleData = nucAngleSignle / 10;
//                LOG("nucAngleData=%f\n",nucAngleData);

            //计算方向
            g_app_dtu_new.rotation_direction = g_direction_new;
            //计算速度
            if (g_app_dtu_new.sec > 0 && g_app_dtu_new.rotation_data_last > 0)
            {
                g_app_dtu_new.rotation_speed = fabs(nucAngleData - g_app_dtu_new.rotation_data_last) / g_app_dtu_new.sec;
            }
            g_app_dtu_new.rotation_data_last = nucAngleData;
        }

//              LOG("nucAngleData=%f\n",nucAngleData);
    }
    else
    {
        nucAngleData = 0;
    }

    if (g_sensor_able.wind_speed == 1) /*风速计算       2020-03-22*/
    {
#if 0
//           LOG("wind signle: %d\n",nucWindSignle);
        if (nucWindSignle > nucWindZero)                              /*风速计算      2020-03-22*/
        {
            aucData = nucWindCV;
            aucData = aucData / 1000;
            nucWindData = (nucWindSignle - nucWindZero) * aucData;
        }
        else
        {
            nucWindData = 0;
        }
#else
        if (nucWindSignle > nucWindZero)                              /*风速计算      2020-03-22*/
        {
            aucData = nucWindCV;
            aucData = aucData / 1000;
            nucWindData = (nucWindSignle) * aucData;
        }
        else
        {
            nucWindData = 0;
        }
#endif
    }
    else
    {
        nucWindData = 0;
    }
//    LOG("AngleX--0 == %d:%d\n",nucDispAngleXSignle,nucDispAngleXZero);
//    LOG("AngleY--0 == %d:%d\n",nucDispAngleYSignle,nucDispAngleYZero);
#if 1
    if (g_sensor_able.angle == 1) //倾角
    {
        if (nucDispAngleXSignle >= nucDispAngleXZero)
        {
            nucDispAngleXData = (nucDispAngleXSignle - nucDispAngleXZero); /*2015-04-09*/
            nucDispAngleXData = nucDispAngleXData / 10;
        }
        else
        {
            nucDispAngleXSignle_temp = nucDispAngleXSignle + 3599;
            nucDispAngleXData = (nucDispAngleXSignle_temp - nucDispAngleXZero); /*2015-04-09*/
            nucDispAngleXData = nucDispAngleXData / 10;
        }

        if (nucDispAngleYSignle >= nucDispAngleYZero)
        {
            nucDispAngleYData = (nucDispAngleYSignle - nucDispAngleYZero); /*2015-04-09*/
            nucDispAngleYData = nucDispAngleYData / 10;
        }
        else
        {
            nucDispAngleYSignle_temp = nucDispAngleYSignle + 3599;
            nucDispAngleYData = (nucDispAngleYSignle_temp - nucDispAngleYZero); /*2015-04-09*/
            nucDispAngleYData = nucDispAngleYData / 10;
        }


#ifdef USE_DWIN

        if (nucDispAngleXData < -180)            nucDispAngleXData += 360;
        else if (nucDispAngleXData > 180)    nucDispAngleXData -= 360;

        if (nucDispAngleXData < 0)
        {
            nucDispAngleXData = 0 - nucDispAngleXData;
        }

        if (nucDispAngleYData < -180)            nucDispAngleYData += 360;
        else if (nucDispAngleYData > 180)    nucDispAngleYData -= 360;
        if (nucDispAngleYData < 0)
        {
            nucDispAngleYData = 0 - nucDispAngleYData;
        }
//        LOG("AngleX == %.2f\n",nucDispAngleXData);
//        LOG("AngleY == %.2f\n",nucDispAngleYData);

#endif

#if 0
        if (nucDispAngleZSignle >= g_new_bef.z_zero)
        {
            nucDispAngleZData = (nucDispAngleZSignle - g_new_bef.z_zero); /*2015-04-09*/
            nucDispAngleZData = nucDispAngleZData / 10;
        }
        else
        {
            nucDispAngleZSignle_temp = nucDispAngleZSignle + 3599;
            nucDispAngleZData = (nucDispAngleZSignle_temp - g_new_bef.z_zero); /*2015-04-09*/
            nucDispAngleZData = nucDispAngleZData / 10;
        }



#else

        float Lx, Ly, Lz;
#if 1
//        nucDispAngleXData=nucDispAngleXSignle/10;
//        nucDispAngleYData=nucDispAngleYSignle/10;

        Lx = sinf(3.1415926f * nucDispAngleXData / 180.0f); //x方向偏移量
        Ly = sinf(3.1415926f * nucDispAngleYData / 180.0f); //y方向偏移量

        Lz = sqrt(Lx * Lx + Ly * Ly); //勾股定理求z方向偏移量 -> sqrt(Lx*Lx+Ly*Ly)
        nucDispAngleZData = asinf(Lz) * 180 / 3.1415926f;

#else
        nucDispAngleXData = nucDispAngleXSignle / 10;
        nucDispAngleYData = nucDispAngleYSignle / 10;

        Lx = sin(nucDispAngleXData) * nucHighData  ; //x方向偏移量
        Ly = sin(nucDispAngleYData) * nucHighData ; //y方向偏移量

        Lz = sqrt(Lx * Lx + Ly * Ly); //勾股定理求z方向偏移量
        nucDispAngleZData = asin(Lz / nucHighData);

#endif
//        LOG("X[%f]-Y[%f]\n",nucDispAngleXData,nucDispAngleYData);
//        LOG("Z0[%f]\n",nucDispAngleZData);
//        nucDispAngleZData = nucDispAngleZData-g_new_bef.z_zero;
//        LOG("Z[%f]\n",nucDispAngleZData);

//        LOG("Angle_dataXY == %.2f:%.2f\n",nucDispAngleXData,nucDispAngleYData);
#endif
//        LOG("Angle_signleXY == %d:%d\n",nucDispAngleXSignle,nucDispAngleYSignle);
//        LOG("Angle_dataXY == %.2f:%.2f\n",nucDispAngleXData,nucDispAngleYData);

    }
    else
    {
        nucDispAngleXData = 0;
        nucDispAngleYData = 0;
    }
#endif
    /*************************额定重量计算********************/
    aucLong1 = nucTowerStartLong;
    aucLong1 = aucLong1 / 100;
    aucLong2 = nucTowerMidLong;
    aucLong2 = aucLong2 / 100;
    aucLong3 = nucTowerEndLong;
    aucLong3 = aucLong3 / 100;

    if (nucLongData <= aucLong1)
    {
        nucMaxWeightData = nucTowerStartWeight;
        nucMaxWeightData = nucMaxWeightData / 1000;
    }
    else if ((nucLongData > aucLong1) && (nucLongData <= aucLong2))
    {
        nucMaxWeightData = nucTowerSlope1 * nucLongData + nucTowerCV1;
    }
    else
    {
        nucMaxWeightData = nucTowerSlope2 * nucLongData + nucTowerCV2;
        if ((nucMaxWeightData * 1000) < nucTowerEndWeight)
        {
            nucMaxWeightData = nucTowerEndWeight;
            nucMaxWeightData = nucMaxWeightData / 1000;
        }
    }
    if (nucMaxWeightData < 0)
    {
        nucMaxWeightData = 0;
    }

    /*******************************力矩百分比****************************/
    if (nucMaxWeightData != 0)
    {
        nucMomentPerData = nucWeightData / nucMaxWeightData;
    }
    else
    {
        nucMomentPerData = 0;
        nucMomentStatuse = 0;
    }
    /*******************************力矩计算****************************/
    if (nucLongData <= 1)
    {
        nucMomentData = nucWeightData * 9.8f * 1;
    }
    else
    {
        nucMomentData = nucWeightData * 9.8f * nucLongData;                           //力矩数据计算, 单位KN*M
    }
}

uint32_t g_no_signal_num = 20;
void nucSysAlarmDataHandle(void)
{
    if (g_no_signal_num != 0) //防止开机误报警
    {
        g_no_signal_num--;
        return;
    }
    float aucData = 0;

    if (g_sensor_state.weight == 0)           //重量正常
    {
        /********************************重量报警判断****************/
        if (nucWeightData >= nucMaxWeightData)
        {
            nucWeightStatuse = 2;     /*重量报警  2020-03-24*/
        }
        else if ((nucWeightData >= (0.8f * nucMaxWeightData)) && (nucWeightData <
                 nucMaxWeightData))
        {
            nucWeightStatuse = 1;     /*重量预警  2020-03-24*/
        }
        else
        {
            nucWeightStatuse = 0;     /*重量状态正常    2020-03-24*/
        }
    }
    else
    {
        nucWeightStatuse = 0;         //重量状态正常        2020.11.02
    }
    /************************************幅度状态判断*************************/

    if ((g_sensor_state.range == 0) && (g_alarm_able.range))              //幅度正常
    {

        if (g_dj_leixing  == 1)
        {
            /********************************仰角状态判断********************/
            // 仰角报警判断 - 动臂吊仰角使用幅度报警位
            // 判断仰角上限报警

            if (nucDispAngleZData1 < ((float)g_yj_alarm[1] / 100)) //仰角下限 报警1000
            {
                nucLongStatuse = 2;
            }
            else if ((nucDispAngleZData1 >= ((float)g_yj_alarm[1] / 100)) && (nucDispAngleZData1 <= ((float)g_yj_warn[1] / 100))) // //仰角下限 预警  1000  2000
            {
                nucLongStatuse = 1;
            }
            else if ((nucDispAngleZData1 >= ((float)g_yj_warn[0] / 100)) && (nucDispAngleZData1 <= (float)g_yj_alarm[0] / 100)) //仰角上限 预警  8000    7000
            {
                nucLongStatuse = 3;
            }
            else if (nucDispAngleZData1 >= ((float)g_yj_alarm[0] / 100)) // 7000
            {
                nucLongStatuse = 4;
            }
            else
            {
                nucLongStatuse = 0;     /*正常*/
            }
        }
        else
        {
            aucData = nucTowerLong;
            aucData = aucData / 100;
            if (nucLongData < ((float)LONG_ALARM_DATA / 100))
            {
                nucLongStatuse = 2;   /*幅度向内报警    2020-03-24*/
            }
            else if ((nucLongData >= ((float)LONG_ALARM_DATA / 100)) && (nucLongData <= ((float)LONG_ALARM_DATA_1 / 100)))
            {
                nucLongStatuse = 1;   /*幅度向内预警    2020-03-24*/
            }
            else if ((nucLongData >= (aucData - ((float)LONG_ALARM_DATA_1 / 100))) && (nucLongData <= (aucData - (float)LONG_ALARM_DATA / 100)))
            {
                nucLongStatuse = 3;   /*幅度向外预警    2020-03-24*/
            }
            else if (nucLongData >= (aucData - (float)LONG_ALARM_DATA / 100))
            {
                nucLongStatuse = 4;   /*幅度向外报警    2020-03-24*/
            }
            else
            {
                nucLongStatuse = 0;   /*幅度正常  2020-03-24*/
            }
        }
    }
    else
    {
        nucLongStatuse = 0;   /*幅度正常  2020-03-24*/
    }

    /******************************高度状态判断*****************/

    if (g_sensor_state.height == 0)           //高度正常
    {
        if (nucHighMode == 0)     //高度零点位置为大臂
        {
            if (nucHighData < ((float)HIGH_ALARM_DATA / 100))
            {
                nucHighStatuse = 2;   /*高度报警  2020-03-24*/
            }
            else if ((nucHighData <= ((float)HIGH_ALARM_DATA_1 / 100)) && (nucHighData >= ((float)HIGH_ALARM_DATA / 100)))
            {
                nucHighStatuse = 1;   /*高度预警  2020-03-24*/
            }
            else
            {
                nucHighStatuse = 0;   /*高度正常  2020-03-24*/
            }
        }
        else                                  //高度零点位置为地面
        {
            if ((nucHighData * 100) > (nucTowerMaxHigh - HIGH_ALARM_DATA * 1))
            {
                nucHighStatuse = 2;   /*高度报警  2020-03-24*/
            }
            else if (((nucHighData * 100) >= (nucTowerMaxHigh - HIGH_ALARM_DATA_1 * 1)) && ((nucHighData * 100) <= (
                         nucTowerMaxHigh - HIGH_ALARM_DATA * 1)))
            {
                nucHighStatuse = 1;   /*高度预警  2020-03-24*/
            }
            else
            {
                nucHighStatuse = 0;   /*高度正常  2020-03-24*/
            }
        }
    }
    else
    {
        nucHighStatuse = 0;   /*高度正常  2020-03-24*/
    }

    /*******************************风速状态判断***************/

    if (g_sensor_state.wind_speed == 0)           //风速正常
    {
        if (nucWindData >= ((float)WIND_ALARM_DATA / 100))
        {
            nucWindStatuse = 2;   /*风速报警  2020-03-24*/
        }
        else if ((nucWindData >= ((float)WIND_ALARM_DATA_1 / 100)) && (nucWindData < ((float)WIND_ALARM_DATA) / 100))
        {
            nucWindStatuse = 1;   /*风速预警  2020-03-24*/
        }
        else
        {
            nucWindStatuse = 0;   /*风速正常  2020-03-24*/
        }
    }
    else
    {
        nucWindStatuse = 0;       //风速正常      2020.11.02
    }

    if (g_sensor_state.angle == 0)            //倾角正常
    {
        /********************************倾角状态判断********************/

#if 1
            LOG("nucDispAngleZData=%f\n",nucDispAngleZData);
            LOG("QJ_ALARM_DATA=%f\n",QJ_ALARM_DATA);
            LOG("QJ_ALARM_DATA_1=%f\n",QJ_ALARM_DATA_1);
        if ((nucDispAngleZData >= (((float)(QJ_ALARM_DATA)) / 100)))
        {
            nucDispAngleStatuse = 2;      //倾角报警
            LOG("nucDispAngleStatuse=%d\n",nucDispAngleStatuse);
        }
        else   if ((nucDispAngleZData >= ((((float)(QJ_ALARM_DATA_1))) / 100)))
        {
            nucDispAngleStatuse = 1;  //倾角预警
            LOG("nucDispAngleStatuse=%d\n",nucDispAngleStatuse);
        }
        else
        {
            nucDispAngleStatuse = 0;  //倾角正常
            LOG("nucDispAngleStatuse=%d\n",nucDispAngleStatuse);
        }
#else
        if (nucDispAngleXData < 180)
        {
            if (nucDispAngleYData < 180)
            {
                if ((nucDispAngleXData >= (((float)(QJ_ALARM_DATA)) / 100)) || (nucDispAngleYData >= (((float)(QJ_ALARM_DATA)) / 100)))
                {
                    nucDispAngleStatuse = 2;      //倾角报警      2020.11.02
                }
                else   if ((nucDispAngleXData >= ((((float)(QJ_ALARM_DATA_1))) / 100))
                           || (nucDispAngleYData >= ((((float)(QJ_ALARM_DATA_1))) / 100)))
                {
                    nucDispAngleStatuse = 1;  //倾角预警      2020.11.02
                }
                else
                {
                    nucDispAngleStatuse = 0;  //倾角正常      2020.11.02
                }
            }
            else
            {
                if ((nucDispAngleXData >= (((float)(QJ_ALARM_DATA)) / 100)) || (nucDispAngleYData <= (((float)(36000 - QJ_ALARM_DATA)) / 100)))
                {
                    nucDispAngleStatuse = 2;      //倾角报警      2020.11.02
                }
                else  if ((nucDispAngleXData >= ((((float)(QJ_ALARM_DATA_1))) / 100))
                          || (nucDispAngleYData <= ((((float)(36000 - QJ_ALARM_DATA_1))) / 100)))
                {
                    nucDispAngleStatuse = 1;  //倾角预警      2020.11.02
                }
                else
                {
                    nucDispAngleStatuse = 0;  //倾角正常      2020.11.02
                }

            }
        }
        else
        {
            if (nucDispAngleYData < 180)
            {
                if ((nucDispAngleXData <= (((float)(36000 - QJ_ALARM_DATA)) / 100)) || (nucDispAngleYData >= (((float)(QJ_ALARM_DATA)) / 100)))
                {
                    nucDispAngleStatuse = 2;      //倾角报警      2020.11.02
                }
                else if ((nucDispAngleXData <= ((((float)(36000 - QJ_ALARM_DATA_1))) / 100))
                         || (nucDispAngleYData >= ((((float)(QJ_ALARM_DATA_1))) / 100)))
                {
                    nucDispAngleStatuse = 1;  //倾角预警      2020.11.02
                }
                else
                {
                    nucDispAngleStatuse = 0;  //倾角正常      2020.11.02
                }

            }
            else
            {
                if ((nucDispAngleXData <= (((float)(36000 - QJ_ALARM_DATA)) / 100)) || (nucDispAngleYData <= (((float)(36000 - QJ_ALARM_DATA)) / 100)))
                {
                    nucDispAngleStatuse = 2;      //倾角报警      2020.11.02
                }
                else if ((nucDispAngleXData <= ((((float)(36000 - QJ_ALARM_DATA_1))) / 100))
                         || (nucDispAngleYData <= ((((float)(36000 - QJ_ALARM_DATA_1))) / 100)))
                {
                    nucDispAngleStatuse = 1;  //倾角预警      2020.11.02
                }
                else
                {
                    nucDispAngleStatuse = 0;  //倾角正常      2020.11.02
                }

            }
        }
#endif
    }
    else
    {
        nucDispAngleStatuse = 0;
    }

    /*
            //取消回转扭矩预警报警
      if(g_sensor_state.rotation == 0) //回转正常
        {
          if(g_new_bef.twist >= 4860 && g_new_bef.twist <5400) //回转右限位/扭度预警
            {
              g_twist_alarm_status = 1;
            }
          else if(g_new_bef.twist >= 5400)//回转右限位/扭度报警
            {
              g_twist_alarm_status = 2;
            }
          else if(g_new_bef.twist <= -4860 && g_new_bef.twist > -5400) //回转左限位/扭度预警
            {
              g_twist_alarm_status = 3;
            }
          else if(g_new_bef.twist <= -5400)//回转左限位/扭度报警
            {
              g_twist_alarm_status = 4;
            }
          else
            {
              g_twist_alarm_status = 0;
            }
        }
      else
        {
          g_twist_alarm_status = 0;
        }
    */


///////////////////////////////////////////////////////////////////////////////
//////////////
//修改报警上传标志位               - 2020.12.21
///////////////////////////////////////////////////////////////////////////////
//////////////
    uint16_t *sensor = (uint16_t*)&g_sensor_able;
    uint16_t *alarm = (uint16_t*)&g_alarm_able;
    if (*sensor != g_sensor_last || *alarm  != g_alarm_last)
    {
        nucAlarmTbl[3] = 0;
        nucAlarmTbl[2] = 0;
        nucAlarmTbl[1] = 0;
        nucAlarmTbl[0] = 0;
        g_sensor_last = *sensor ;
        g_alarm_last = *alarm;
    }

//  if(g_sensor_able.rotation == 1) //回转传感器启用，20250507取消左右回转扭矩报警预警
//    {
//      if(g_alarm_able.rotation== 1)
//        {
//          if(g_twist_alarm_status == 1) //回转右限位/扭度预警
//            {
//              nucAlarmTbl[1]=nucAlarmTbl[1]|0x80;
//            }
//          else
//            {
//              nucAlarmTbl[1]=nucAlarmTbl[1]&(~0x80);
//            }

//          if(g_twist_alarm_status == 2)//回转右限位/扭度报警
//            {
//              nucAlarmTbl[1]=nucAlarmTbl[1]|0x20;
//            }
//          else
//            {
//              nucAlarmTbl[1]=nucAlarmTbl[1]&(~0x20);
//            }
//          if(g_twist_alarm_status == 3) //回转左限位/扭度预警
//            {
//              nucAlarmTbl[1]=nucAlarmTbl[1]|0x40;
//            }
//          else
//            {
//              nucAlarmTbl[1]=nucAlarmTbl[1]&(~0x40);
//            }

//          if(g_twist_alarm_status == 4)//回转左限位/扭度报警
//            {
//              nucAlarmTbl[1]=nucAlarmTbl[1]|0x10;
//            }
//          else
//            {
//              nucAlarmTbl[1]=nucAlarmTbl[1]&(~0x10);
//            }
//        }
//      else
//        {
//          nucAlarmTbl[1]=nucAlarmTbl[1]&(~0x80);
//          nucAlarmTbl[1]=nucAlarmTbl[1]&(~0x20);
//          nucAlarmTbl[1]=nucAlarmTbl[1]&(~0x40);
//          nucAlarmTbl[1]=nucAlarmTbl[1]&(~0x10);
//        }
//    }
    if (g_sensor_able.angle)  //倾角预警控制
    {
        if (g_alarm_able.angle)
        {
            if (nucDispAngleStatuse == 1)         //倾角预警控制
            {
                nucAlarmTbl[3] = nucAlarmTbl[3] | 0x40;
            }
            else
            {
                nucAlarmTbl[3] = nucAlarmTbl[3] & (~0x40);
            }
            if (nucDispAngleStatuse == 2)         //倾角报警控制
            {
                nucAlarmTbl[3] = nucAlarmTbl[3] | 0x10;
            }
            else
            {
                nucAlarmTbl[3] = nucAlarmTbl[3] & (~0x10);
            }
        }
        else
        {
            nucAlarmTbl[3] = nucAlarmTbl[3] & (~0x10);
            nucAlarmTbl[3] = nucAlarmTbl[3] & (~0x40);
            nucDispAngleStatuse = 0;
        }
    }
    else
    {
        nucAlarmTbl[3] = nucAlarmTbl[3] & (~0x10);
        nucAlarmTbl[3] = nucAlarmTbl[3] & (~0x40);
        nucDispAngleStatuse = 0;
    }
    //////////仰角预警报警控制
    // 注意：动臂吊的仰角报警已通过 nucLongStatuse（幅度状态）实现，不需要单独处理
    // 仰角报警使用幅度报警位，见代码第1470行注释

    if (g_sensor_able.weight)
    {
        if (g_alarm_able.weight)
        {
            if (nucWeightStatuse == 1)    /*起重量预警*/
            {
                nucAlarmTbl[3] = nucAlarmTbl[3] | 0x08;
                nucAlarmTbl[3] = nucAlarmTbl[3] | 0x04;
            }
            else
            {
                nucAlarmTbl[3] = nucAlarmTbl[3] & (~0x08);
                nucAlarmTbl[3] = nucAlarmTbl[3] & (~0x04);
            }
            if (nucWeightStatuse == 2)    /*起重量报警   2020-04-02*/
            {
                nucAlarmTbl[3] = nucAlarmTbl[3] | 0x02;
                nucAlarmTbl[3] = nucAlarmTbl[3] | 0x01;
            }
            else
            {
                nucAlarmTbl[3] = nucAlarmTbl[3] & (~0x02);
                nucAlarmTbl[3] = nucAlarmTbl[3] & (~0x01);
            }
        }
        else
        {
            nucAlarmTbl[3] = nucAlarmTbl[3] & (~0x02);
            nucAlarmTbl[3] = nucAlarmTbl[3] & (~0x01);
            nucAlarmTbl[3] = nucAlarmTbl[3] & (~0x08);
            nucAlarmTbl[3] = nucAlarmTbl[3] & (~0x04);
            nucWeightStatuse = 0;
        }
    }
    else
    {
        nucAlarmTbl[3] = nucAlarmTbl[3] & (~0x02);
        nucAlarmTbl[3] = nucAlarmTbl[3] & (~0x01);
        nucAlarmTbl[3] = nucAlarmTbl[3] & (~0x08);
        nucAlarmTbl[3] = nucAlarmTbl[3] & (~0x04);
        nucWeightStatuse = 0;
    }

    if (g_sensor_able.range)  /*幅度向内预警*/
    {
        if (g_alarm_able.range)
        {
            if (nucLongStatuse == 1) /*幅度向内预警*/
            {
                nucAlarmTbl[2] = nucAlarmTbl[2] | 0x02;
            }
            else
            {
                nucAlarmTbl[2] = nucAlarmTbl[2] & (~0x02);
            }
            if (nucLongStatuse == 2) /*幅度向内报警    2020-04-02*/
            {
                nucAlarmTbl[1] = nucAlarmTbl[1] | 0x02;
            }
            else
            {
                nucAlarmTbl[1] = nucAlarmTbl[1] & (~0x02);
            }
            if (nucLongStatuse == 3) /*幅度向外预警*/
            {
                nucAlarmTbl[2] = nucAlarmTbl[2] | 0x01;
            }
            else
            {
                nucAlarmTbl[2] = nucAlarmTbl[2] & (~0x01);
            }
            if (nucLongStatuse == 4) /*幅度向外报警    2020-04-02*/
            {
                nucAlarmTbl[1] = nucAlarmTbl[1] | 0x01;
            }
            else
            {
                nucAlarmTbl[1] = nucAlarmTbl[1] & (~0x01);
            }
        }
        else
        {
            nucAlarmTbl[2] = nucAlarmTbl[2] & (~0x02);
            nucAlarmTbl[1] = nucAlarmTbl[1] & (~0x02);
            nucAlarmTbl[2] = nucAlarmTbl[2] & (~0x01);
            nucAlarmTbl[1] = nucAlarmTbl[1] & (~0x01);
            nucLongStatuse = 0;
        }
    }
    else
    {
        nucAlarmTbl[2] = nucAlarmTbl[2] & (~0x02);
        nucAlarmTbl[1] = nucAlarmTbl[1] & (~0x02);
        nucAlarmTbl[2] = nucAlarmTbl[2] & (~0x01);
        nucAlarmTbl[1] = nucAlarmTbl[1] & (~0x01);

        nucLongStatuse = 0;
    }

    if (g_sensor_able.height) /*高度向上预警*/
    {
        if (g_alarm_able.height)
        {
            if (nucHighStatuse == 1) /*高度向上预警*/
            {
                nucAlarmTbl[2] = nucAlarmTbl[2] | 0x04;
            }
            else
            {
                nucAlarmTbl[2] = nucAlarmTbl[2] & (~0x04);
            }
            if (nucHighStatuse == 2) /*高度向上报警    2020-04-02*/
            {
                nucAlarmTbl[1] = nucAlarmTbl[1] | 0x04;
            }
            else
            {
                nucAlarmTbl[1] = nucAlarmTbl[1] & (~0x04);
            }
        }
        else
        {
            nucAlarmTbl[1] = nucAlarmTbl[1] & (~0x04);
            nucAlarmTbl[2] = nucAlarmTbl[2] & (~0x04);
            nucHighStatuse = 0;
        }
    }
    else
    {
        nucAlarmTbl[1] = nucAlarmTbl[1] & (~0x04);
        nucAlarmTbl[2] = nucAlarmTbl[2] & (~0x04);
        nucHighStatuse = 0;
    }

    if (nucDanJiStatuse == 1) /*单机防碰撞报警 2020-04-02*/
    {
        //        nucDanJiStatuse=0;
        nucAlarmTbl[0] = nucAlarmTbl[0] | 0x02;
    }
    else
    {
        nucAlarmTbl[0] = nucAlarmTbl[0] & (~0x02);
    }

//    if (nucSysDanJiJiansulFlag == 1 && nucDanJiStatuse == 0) /*单机防碰撞预警，信息上传*/
    if (nucSysDanJiJiansulFlag == 1)
    {
//        nucDanJiStatuse=0;
        nucAlarmTbl[0] = nucAlarmTbl[0] | 0x04;
    }
    else
    {
        nucAlarmTbl[0] = nucAlarmTbl[0] & (~0x04);
    }


    if (g_sensor_able.duoji)
    {
        if (g_alarm_able.duoji)
        {
            if (nucDuojiStatuse == 1)         //多机防碰撞预警
            {
                nucAlarmTbl[0] = nucAlarmTbl[0] | 0x08;
            }
            else
            {
                nucAlarmTbl[0] = nucAlarmTbl[0] & (~0x08);
            }
            if (nucDuojiStatuse == 2)         //多机防碰撞报警
            {
                nucAlarmTbl[0] = nucAlarmTbl[0] | 0x01;
            }
            else
            {
                nucAlarmTbl[0] = nucAlarmTbl[0] & (~0x01);
            }
        }
        else
        {
            nucAlarmTbl[0] = nucAlarmTbl[0] & (~0x08);
            nucAlarmTbl[0] = nucAlarmTbl[0] & (~0x01);
            nucDuojiStatuse = 0;
        }
    }
    else
    {
        nucAlarmTbl[0] = nucAlarmTbl[0] & (~0x08);
        nucAlarmTbl[0] = nucAlarmTbl[0] & (~0x01);
        nucDuojiStatuse = 0;
    }

    if (g_sensor_able.wind_speed)
    {
        if (g_alarm_able.wind_speed)
        {
            if (nucWindStatuse == 1)      //风速预警
            {
                nucAlarmTbl[0] = nucAlarmTbl[0] | 0x40;
            }
            else
            {
                nucAlarmTbl[0] = nucAlarmTbl[0] & (~0x40);
            }
            if (nucWindStatuse == 2)      //风速报警
            {
                nucAlarmTbl[0] = nucAlarmTbl[0] | 0x20;
            }
            else
            {
                nucAlarmTbl[0] = nucAlarmTbl[0] & (~0x20);
            }
        }
        else
        {
            nucWindStatuse = 0;
            nucAlarmTbl[0] = nucAlarmTbl[0] & (~0x40);
            nucAlarmTbl[0] = nucAlarmTbl[0] & (~0x20);
        }
    }
    else
    {
        nucWindStatuse = 0;
        nucAlarmTbl[0] = nucAlarmTbl[0] & (~0x40);
        nucAlarmTbl[0] = nucAlarmTbl[0] & (~0x20);
    }

    g_enent.BUFF[3] = nucAlarmTbl[3];
    g_enent.BUFF[2] = nucAlarmTbl[2];
    g_enent.BUFF[1] = nucAlarmTbl[1];
    g_enent.BUFF[0] = nucAlarmTbl[0];
///////////////////////////////////////////////////////////////////////////////
//////////////

    if (nucDispAngleStatuse == 2)         //倾角报警控制
    {
        if (g_sensor_able.angle)
        {
            if (g_alarm_able.angle)
            {
                if (nucControlFlag > 0 && nucDebugFlag == 0)
                {
                    //LONGOUTON;
                    //HIGHUPON;
                }
            }
        }
    }
    //20250707增加nucLongOutControl的控制增加 当单机防碰撞为0非debug模式下为0


    //继电器检测关闭
    //nucControlFlag塔机控制功能
    if (nucControlFlag > 0 && nucDebugFlag == 0)
    {
//      if(nucWeightStatuse!=2 && nucLongStatuse!=4 && nucLongOutControlFlag!=1 &&nucFaceFlag!=2)
        if (nucLongOutControl == 0 && nucWeightStatuse != 2 && nucLongStatuse != 4 && nucLongOutControlFlag != 1 && nucLongStatuse1 != 2)
        {
            LONGOUTOFF;
        }
        if (nucLongStatuse != 2 && nucLongInControlFlag != 1)
        {
            LONGINOFF;
        }
//        if(nucWeightStatuse!=2 && nucHighStatuse !=2 &&nucFaceFlag!=2)
        if (nucWeightStatuse != 2 && nucHighStatuse != 2)
        {
            HIGHUPOFF;
        }
        if (nucSysDanJiControlFlag == 0 && nucSysDuoJiControlFlag == 0)
        {
            ANGLELEFTOFF;
            ANGLERIGHTOFF;
        }
    }
//    LOG("nucLongOutControl=%d,nucWeightStatuse=%d,nucLongStatuse=%d,nucLongOutControlFlag=%d,nucLongStatuse1=%d",
//        nucLongOutControl,   nucWeightStatuse, nucLongStatuse, nucLongOutControlFlag, nucLongStatuse1);
    if (nucWeightStatuse == 2)    /*起重量报警控制*/
    {
        if (g_sensor_able.weight)
        {
            if (g_alarm_able.weight)
            {
                if (nucControlFlag > 0 && nucDebugFlag == 0)
                {
                    LONGOUTON;
                    HIGHUPON;

                    LOG("11111\n");
                }
            }
        }
    }

    if (nucLongStatuse == 2) /*幅度向内报警控制*/
    {
        if (g_sensor_able.range)
        {
            if (g_alarm_able.range)
            {
                if (nucControlFlag > 0 && nucDebugFlag == 0)
                {
                    LONGINON;
                }
            }
        }
    }

    if (nucLongStatuse == 4) /*幅度向外报警控制*/
    {
        if (g_sensor_able.range)
        {
            if (g_alarm_able.range)
            {
                if (nucControlFlag > 0 && nucDebugFlag == 0)
                {
                    LONGOUTON;
                }
            }
        }
    }

    if (nucLongStatuse1 == 2) /*多机幅度向外报警控制*/
    {
        LONGOUTON;
    }
    if (nucHighStatuse == 2) /*高度向上报警控制*/
    {
        if (g_sensor_able.height)
        {
            if (g_alarm_able.height)
            {
                if (nucControlFlag > 0 && nucDebugFlag == 0)
                {
                    HIGHUPON;
                }
            }
        }
    }

    if (nucSysDanJiControlFlag == 1 || nucSysDuoJiControlFlag != 0) //单机多机防碰撞继电器处理
// if( nucSysDuoJiControlFlag!=0)
    {
        if (nucControlFlag > 0 && nucDebugFlag == 0)
        {

            if (nucSysDuoJiControlFlag == 3)
            {
                ANGLELEFTON;
                ANGLERIGHTON;
            }
            else
            {
                if (nucAngleLeftControlFlag == 1 || nucSysDuoJiControlFlag == 2)
                {
                    ANGLELEFTON;
                    if (nucSysDuoJiControlFlag != 1 && nucAngleRightControlFlag == 0)
                    {
                        ANGLERIGHTOFF;
                    }
                }

                if (nucAngleRightControlFlag == 1 || nucSysDuoJiControlFlag == 1)
                {
                    ANGLERIGHTON;
                    if (nucSysDuoJiControlFlag != 2 && nucAngleLeftControlFlag == 0)
                    {
                        ANGLELEFTOFF;
                    }
                }
//        LOG("nucAngleLeftControlFlag[%d],nucAngleRightControlFlag[%d],nucSysDuoJiControlFlag[%d]\n",nucAngleLeftControlFlag,nucAngleRightControlFlag,nucSysDuoJiControlFlag);

                if (nucLongOutControlFlag == 1)
                {
                    LONGOUTON;  //GPIO_13_SET(1);
                }

                if (nucLongInControlFlag == 1)
                {
                    LONGINON;
                }
            }
        }
    }
//    if(nucFaceFlag==2)
//    {
//        if(nucControlFlag>0&&nucDebugFlag==0)
//        {
//            LONGOUTON;
//            HIGHUPON;
//        }
//    }

    if (nucWeightStatuse == 2)    /*????? 2020-04-02*/
    {
        if (g_sensor_able.weight)
        {
            if (g_alarm_able.weight)
            {
                nucSysStatuse = 1;
            }
        }
    }

    else if (nucLongStatuse == 2) /*??????    2020-04-02*/
    {
        if (g_sensor_able.range)
        {
            if (g_alarm_able.range)
            {
                nucSysStatuse = 2;
            }
        }
    }
    else if (nucLongStatuse == 4) /*??????    2020-04-02*/
    {
        if (g_sensor_able.range)
        {
            if (g_alarm_able.range)
            {
                nucSysStatuse = 3;
            }
        }
    }
    else if (nucHighStatuse == 2) /*??????    2020-04-02*/
    {
        if (g_sensor_able.height)
        {
            if (g_alarm_able.height)
            {
                nucSysStatuse = 4;
            }
        }
    }
    else if (nucDanJiStatuse == 1) /*???????   2020-04-02*/
    {
        nucSysStatuse = 8;
    }
    else if (nucDuojiStatuse == 1)        //???????
    {
        if (g_sensor_able.duoji)
        {
            if (g_alarm_able.duoji)
            {
                nucSysStatuse = 24;
            }
        }
    }
    else if (nucDuojiStatuse == 2)        //???????
    {
        if (g_sensor_able.duoji)
        {
            if (g_alarm_able.duoji)
            {
                nucSysStatuse = 9;
            }
        }
    }
    else if (nucWindStatuse == 2)         //????
    {
        if (g_sensor_able.wind_speed)
        {
            if (g_alarm_able.wind_speed)
            {
                nucSysStatuse = 14;
            }
        }
    }
    else if (nucWeightStatuse == 1)   /*?????*/
    {
        if (g_sensor_able.weight)
        {
            if (g_alarm_able.weight)
            {
                nucSysStatuse = 16;
            }
        }
    }
    else if (nucLongStatuse == 1) /*??????*/
    {
        if (g_sensor_able.range)
        {
            if (g_alarm_able.range)
            {
                nucSysStatuse = 18;
            }
        }
    }
    else if (nucLongStatuse == 3) /*??????*/
    {
        if (g_sensor_able.range)
        {
            if (g_alarm_able.range)
            {
                nucSysStatuse = 17;
            }
        }
    }
    else if (nucHighStatuse == 1) /*??????*/
    {
        if (g_sensor_able.height)
        {
            if (g_alarm_able.height)
            {
                nucSysStatuse = 19;
            }
        }
    }
    else if (nucWindStatuse == 1)         //????
    {
        if (g_sensor_able.wind_speed)
        {
            if (g_alarm_able.wind_speed)
            {
                nucSysStatuse = 25;
            }
        }
    }
    else if (nucDispAngleStatuse == 1)        //??????
    {
        if (g_sensor_able.angle)
        {
            if (g_alarm_able.angle)
            {
                nucSysStatuse = 26;
            }
        }
    }
    else if (nucDispAngleStatuse == 2)
    {
        if (g_sensor_able.angle)
        {
            if (g_alarm_able.angle)
            {
                nucSysStatuse = 15;
            }
        }
    }
    else if (nucFaceFlag == 2)
    {
        nucSysStatuse = 0;
    }
    else
    {
        nucSysStatuse = 0;
    }

}
void nucSysDanJiAlarmHandle(void)
{
    uint8_t   auccounter = 0;
    float     nucLongData_Temp = 0;
    float     nucAngleData_Temp = 0;
    double a;
    for (auccounter = 0; auccounter < 8; auccounter++)
    {
        if (nucHighMode == 0)     //高度零点位置为大臂
        {
            if (nucDanJEndHighTbl[auccounter] > nucTowerMaxHigh) //目标高于塔机
            {
                continue;
            }
            if ((nucHighData * 100) > (nucTowerMaxHigh - nucDanJEndHighTbl[auccounter]))
            {

            }
            else
            {
                continue;
            }
        }
        else
        {
            if (((nucHighData * 100) > 0) && ((nucHighData * 100) < nucDanJEndHighTbl[auccounter]))
            {

            }
            else
            {
                continue;
            }
        }

        if (((nucAngleData) >= nucDanJStartAngleTbl[auccounter]) && ((nucAngleData) < nucDanJEndAngleTbl[auccounter]))
        {

            nucAngleData_Temp = nucAngleData - nucDanJStartAngleTbl[auccounter];
//            LOG("nucLongData = %f,nucDanJStartAngleTbl[auccounter]=%d\n",nucLongData,nucDanJStartAngleTbl[auccounter]);
            if (nucDanJStartAngleTbl_Reference[auccounter] + nucAngleData_Temp  <= 90)
            {
                a = (PI * (nucDanJStartAngleTbl_Reference[auccounter] + nucAngleData_Temp)) / 180;
                nucLongData_Temp = nucLongData * (float)sin(a);
//                LOG("nucLongData_Temp = %f,jia0=%f---sin=%f\n",nucLongData_Temp,(nucDanJStartAngleTbl_Reference[auccounter]+nucAngleData_Temp),(float)sin(a));
            }
            else
            {
                a = (PI * (nucDanJStartAngleTbl_Reference[auccounter] + nucAngleData_Temp - 90)) / 180;
                nucLongData_Temp = nucLongData * (float)cos(a);
//                LOG("nucLongData_Temp = %f,jia0=%f---cos=%f\n",nucLongData_Temp,(nucDanJStartAngleTbl_Reference[auccounter]+nucAngleData_Temp-90),(float)cos(a));
            }
//            LOG("nucDanJStartLongTbl[auccounter]) = %d,nucDanJEndLongTbl[auccounter]=%d\n",nucDanJStartLongTbl[auccounter],nucDanJEndLongTbl[auccounter]);
            if (((nucLongData_Temp * 100) > nucDanJStartLongTbl[auccounter]) && ((nucLongData_Temp * 100) < nucDanJEndLongTbl[auccounter]))
            {

                if (nucControlFlag > 0 && nucDebugFlag == 0)
                {
                    nucSysDanJiControlFlag = 1;
                    if ((nucAngleData - nucDanJStartAngleTbl[auccounter]) < (nucDanJEndAngleTbl[auccounter] - nucAngleData))
                    {
                        //禁止右转, 允许左转
                        nucAngleRightControlFlag = 1;
                        nucAngleLeftControlFlag = 0;
                        if (((nucLongData * 100) - nucDanJStartLongTbl[auccounter]) < (nucDanJEndLongTbl[auccounter] - (nucLongData * 100)))
                        {
                            nucLongOutControlFlag = 1;
                            nucLongInControlFlag = 0;
                        }
                        else
                        {
                            nucLongOutControlFlag = 0;
                            nucLongInControlFlag = 1;
                        }
                    }
                    else
                    {
                        //禁止左转, 允许右转
                        nucAngleRightControlFlag = 0;
                        nucAngleLeftControlFlag = 1;
                        if (((nucLongData * 100) - nucDanJStartLongTbl[auccounter]) < (nucDanJEndLongTbl[auccounter] - (nucLongData * 100)))
                        {
                            nucLongOutControlFlag = 1;
                            nucLongInControlFlag = 0;
                        }
                        else
                        {
                            nucLongOutControlFlag = 0;
                            nucLongInControlFlag = 1;
                        }
                    }
                }
                break;
            }
        }
    }
    if (auccounter == 8)
    {
        nucSysDanJiControlFlag = 0;
        nucAngleRightControlFlag = 0;
        nucAngleLeftControlFlag = 0;
        nucLongOutControlFlag = 0;
        nucLongInControlFlag = 0;
    }
}
void nucSysWorkCycle_Weight(void)
{
    nucWeightData_sum += nucWeightData * 100;
    nucWeightData_num++;
    if (nucWeightData_sum > 0x0fffffff)
    {
        nucSysCycleTime_last = 0xffffffff;//停止计算重量均值 标志位
        nucSysStartWeightData = nucWeightData_sum / nucWeightData_num;
    }
}
void nucSysWorkCycleCount(void)
{
    if (g_app_config.app_config_sys.dtu_status == 0) //未校时,返回
    {
        return;
    }

    if (nucWeightData >= nucWeightData_work)
    {
        if (nucWeightData - nucWeightData_work >= 0.3f)
        {
            if (nucSysStartWorkFlag == 0) /*重量大于0.3T，工作循环开始 2020-03-22*/
            {
                if (++g_CycleCount[0] > 9) //状态稳定
                {
                    g_CycleCount[0] = 0;

                    BSP_LED_On(LED_NUM1, LED_IO_HIGH);
                    nucSysStartWorkFlag = 1; //循环开始
                    nucSysCycleTime = 0;
                    nucSysCycleTime_last = nucSysCycleTime;
                    nucWeightData_sum = nucWeightData_num = 0;
                    nucOldTime = g_bsp_rtc.second;
                    nucSysStartYear = g_bsp_rtc.year;         /*年   2019-09-05*/
                    nucSysStartMonth = g_bsp_rtc.month;       /*月   2019-09-05*/
                    nucSysStartDay = g_bsp_rtc.day;           /*日   2019-09-05*/
                    nucSysStartHour = g_bsp_rtc.hour;         /*时   2019-09-05*/
                    nucSysStartMinute = g_bsp_rtc.minute;         /*分   2019-09-05*/
                    nucSysStartSecond = g_bsp_rtc.second;         /*秒   2019-09-05*/

                    nucSysStartWeightData = nucWeightData * 100; /*起始重量带入    2020-03-22*/
                    nucSysStartLongData = nucLongData * 100;  /*起始幅度带入    2020-03-22*/
                    nucSysStartHighData = nucHighData * 100;  /*起始高度带入    2020-03-22*/
                    nucSysStartAngleData = nucAngleData;          /*起始回转带入    2020-03-22*/
                    nucSysMaxMomentData = nucMomentPerData;

                    if (g_new_bef.pic_enable == 1) //工作开始拍照
                    {
                        if (g_app_pic.pic_status == 0)
                        {
                            nucSendPhoto(1, 2, 1);
                        }
                        else
                        {
                            g_app_pic.pic_wait = 1;
                        }
                    }
                }
            }
            else//if(nucSysStartWorkFlag==1)/*重量大于0.3T，工作循环开始   2020-03-22*/
            {
                if (nucOldTime > g_bsp_rtc.second)
                {
                    nucSysCycleTime = (g_bsp_rtc.second + 60) - nucOldTime + nucSysCycleTime;
                }
                else
                {
                    nucSysCycleTime = g_bsp_rtc.second - nucOldTime + nucSysCycleTime;
                }
                nucOldTime = g_bsp_rtc.second;
                if (nucMomentPerData > nucSysMaxMomentData)
                {
                    nucSysMaxMomentData = nucMomentPerData;
                }
                if (nucSysCycleTime > nucSysCycleTime_last)
                {
                    nucSysCycleTime_last = nucSysCycleTime;
                    nucSysWorkCycle_Weight();
                }

            }

            g_CycleCount[1] = 0; //消除<0.3计数

        }
        else //if(nucWeightData-nucWeightData_work < 0.3)
        {
            if (nucSysStartWorkFlag == 1 && nucSysWorkENdFlag == 0) //有开始
            {
                if (++g_CycleCount[1] > 9) //状态稳定
                {
                    g_CycleCount[1] = 0;

                    nucSysWorkENdFlag = 1;
                    nucSysEndYear = g_bsp_rtc.year;           /*年   2019-09-05*/
                    nucSysEndMonth = g_bsp_rtc.month;             /*月   2019-09-05*/
                    nucSysEndDay = g_bsp_rtc.day;             /*日   2019-09-05*/
                    nucSysEndHour = g_bsp_rtc.hour;           /*时   2019-09-05*/
                    nucSysEndMinute = g_bsp_rtc.minute;           /*分   2019-09-05*/
                    nucSysEndSecond = g_bsp_rtc.second;           /*秒   2019-09-05*/

                    nucSysEndWeightData = nucWeightData * 100;    /*终止重量带入    2020-03-22*/
                    nucSysEndLongData = nucLongData * 100;        /*终止幅度带入    2020-03-22*/
                    nucSysEndHighData = nucHighData * 100;        /*终止高度带入    2020-03-22*/
                    nucSysEndAngleData = nucAngleData;        /*终止回转带入    2020-03-22*/
                    nucSysStartWeightData = nucWeightData_sum / nucWeightData_num;


                }
            }
            else
            {
                g_CycleCount[1] = 0;
            }
            g_CycleCount[0] = 0; //消除>=0.3计数
        }
        g_CycleCount[2] = 0; //消除 相对0点 > 当前重量 计数
    }
    else
    {
        if (nucWeightData_work - nucWeightData >= 0.1f) //相对0点 > 当前重量
        {
            if (++g_CycleCount[2] > 9) //状态稳定
            {
                g_CycleCount[2] = 0;
                nucWeightData_work = nucWeightData; //相对0点 重新赋值
            }
        }
        else
        {
            g_CycleCount[2] = 0; //消除 相对0点 > 当前重量 计数
        }

        if (nucSysStartWorkFlag == 1 && nucSysWorkENdFlag == 0) //有开始
        {
            if (++g_CycleCount[1] > 9) //状态稳定
            {
                g_CycleCount[1] = 0;

//                LOG("WorkCycle End Time Get:%d-%d-%d  %d:%d:%d\n",g_bsp_rtc.DATE.Year,g_bsp_rtc.DATE.Month,g_bsp_rtc.DATE.Date,g_bsp_rtc.TIME.Hours,g_bsp_rtc.TIME.Minutes,g_bsp_rtc.TIME.Seconds);
                nucSysWorkENdFlag = 1;
                nucSysEndYear = g_bsp_rtc.year;           /*年   2019-09-05*/
                nucSysEndMonth = g_bsp_rtc.month;             /*月   2019-09-05*/
                nucSysEndDay = g_bsp_rtc.day;             /*日   2019-09-05*/
                nucSysEndHour = g_bsp_rtc.hour;           /*时   2019-09-05*/
                nucSysEndMinute = g_bsp_rtc.minute;           /*分   2019-09-05*/
                nucSysEndSecond = g_bsp_rtc.second;           /*秒   2019-09-05*/

                nucSysEndWeightData = nucWeightData * 100;    /*终止重量带入    2020-03-22*/
                nucSysEndLongData = nucLongData * 100;        /*终止幅度带入    2020-03-22*/
                nucSysEndHighData = nucHighData * 100;        /*终止高度带入    2020-03-22*/
                nucSysEndAngleData = nucAngleData;        /*终止回转带入    2020-03-22*/
                nucSysStartWeightData = nucWeightData_sum / nucWeightData_num;
            }
        }
        else
        {
            g_CycleCount[1] = 0;
        }
    }
}
void nucSendDtuData(void)
{
#if 0
    nucSendDTUCounter++;          /*DTU计数器  2020-03-22*/
    nucTowerParCounter++;         /*发送设置数据计数器   2020-03-22*/
    if (nucSendDTUCounter >= 30)
    {
        nucSendDTUCounter = 0;    /*计数器清零   2020-03-22*/
        if (nucSendXinTiaoFlag == 0)
        {
            APP_DTU_Response_Hearbeat();  /*发送心跳数据    2020-03-22*/
        }
        else if ((nucSendXinTiaoFlag == 1) && (nucSysGetTimeFlag == 0))
        {
            nucGetSysTime();          /*获取服务器时间 2020-3-19*/
        }
        else if ((nucSysPowerOnFlag == 0) && (nucSysGetTimeFlag == 1))
        {
            nucSysPowerOnFlag = 1;
            nucSendPowerOn();
        }
        else if ((nucSysWorkENdFlag == 1) && (nucSysGetTimeFlag == 1))
        {
            nucSysWorkENdFlag = 0;
            nucSysStartWorkFlag = 0;
            float highData = (float)(abs(nucSysEndHighData - nucSysStartHighData)) / 100;
            float longData = (float)(abs(nucSysEndLongData - nucSysStartLongData)) / 100;
            if (highData >= 1 || longData >= 1)
            {
                LOG("WorkCycle Send...\n");
                nucSendSysWorkCycleData();/*发送工作循环数据  2020-3-19*/
            }
//                      nucSendWorkCycleData(); //向工控机发送
        }
        else if ((nucSysGetTimeFlag == 1) && (nucTowerParCounter >= 36000))
        {
            nucTowerParCounter = 0;
            nucSendTowerParaData();   /*发送设置参数    2020-03-19*/
        }
        else
        {
            nucSendFreqCounter++;
//            LOG("%f\n g_work_freq =%d\n",(float)nucSendFreqCounter*3.6f,g_work_freq);
            if (g_work_freq == 1) //working
            {
                if ((float)nucSendFreqCounter * 3.6 >= (float)nucSendDTUTime_Work) //3->10
                {
                    nucSendFreqCounter = 0;
                    nucSendSYDTUData();       /*·¢ËÍ¹¤×÷Êý¾Ý  2020-3-19*/
//                    LOG("Working!!![%d]\n",nucSendDTUTime_Work);
                }
            }
            else
            {
                if ((float)nucSendFreqCounter * 3.6 >= (float)nucSendDTUTime) //17->60
                {
                    nucSendFreqCounter = 0;
                    nucSendSYDTUData();       /*·¢ËÍ¹¤×÷Êý¾Ý  2020-3-19*/

//                    LOG("No Working!!![%d]\n",nucSendDTUTime);
                }
            }

        }
    }
#endif
}


void nucDuoJFPZJudge1_reset(uint8_t num)
{
    if (nucDuoJIDTbl_last[num] != nucDuoJIDTbl[num])
    {
        nucDuoJAngleTbl[num] = 0;
        nucDuoJLongTbl[num] = 0;
        nucDuoJHighTbl[num] = 0;
        nucDuoJIDTbl_last[num] = nucDuoJIDTbl[num];
    }
}
//纽度计算-
double deg_to_rad(double degrees)
{
    return degrees * PI / 180.0;
}

float bianchang(double angle1, double angle2, float common_side)
{

    double angle3, side1, side2;

    // 计算第三个角度
    angle3 = 180.0 - angle1 - angle2;

    // 检查输入是否有效
    if (angle3 <= 0 || angle1 <= 0 || angle2 <= 0)
    {
        return -1;
    }

    // 使用正弦定理计算另外两条边
//    side1 = common_side * sin(deg_to_rad(angle1)) / sin(deg_to_rad(angle3));
    side2 = common_side * sin(deg_to_rad(angle2)) / sin(deg_to_rad(angle3));

    return side2;
}
float bianchang1(double angle_degrees, double a, float b)
{
    double   angle_radians, c;


    // 验证输入的边长为正数
    if (a <= 0 || b <= 0)
    {

        return 1;
    }

    // 将角度转换为弧度
    angle_radians = angle_degrees * PI / 180.0;

    // 使用余弦定理计算第三边
    c = sqrt(a * a + b * b - 2 * a * b * cos(angle_radians));


    return c;
}
//后臂防碰撞
void nucDuoJFPZJudge2(void);
//本机后臂防碰撞-多机大臂
void nucDuoJFPZJudge3(void);

uint8_t g_duoji_rsta_warn[8] = {0}; //记录进入区域
uint8_t g_duoji_rsta_alarm[8] = {0}; //记录碰撞区域
uint8_t g_duoji_rsta_warn1[8] = {0}; //记录进入区域
float g_bc_last[8] = {0}; //记录进入区域
float g_huiz_last[8] = {0}; //记录进入区域
uint8_t g_bc_last_num = {0}; //记录进入区域
float g_bc_last_save[8];
//本机大臂防碰撞-多机大臂
void nucDuoJFPZJudge1(void)
{
    uint8_t auccounter = 0;
    float   aucMaxLong = 0;
    float   aucDuoJiLong = 0;
    float   aucDuoJiDisdance = 0;
    float AngleBuff[2][2];
    float aucAngle[2], aucAngleDJ[2];
    float   a;
    uint8_t flg[2][2];
    uint8_t g_duoji_state[8] = {0}; //记录报警状态
    uint8_t g_duoji_relay[8] = {0}; //记录继电器状态
    uint8_t g_duoji_relay1[8] = {0}; //记录继电器状态 幅度向外
    uint8_t g_duoji_relay2[8] = {0}; //记录继电器状态 幅度向外
//////////////////////////////////////////////////////
//多机防碰撞判断,配合 碰撞模型图计算
//////////////////////////////////////////////////////
//    if(g_dj_leixing == 1)//动臂吊
//    {
//        aucMaxLong=nucLongData;
//    }
//    else
    {
        aucMaxLong = nucTowerLong;  //本机臂长
        aucMaxLong = aucMaxLong / 100; //本机臂长
    }

    aucAngle[0] = nucAngleData;     //本机当前回转角度
    nucDuojiStatuse = 0;
    nucLongStatuse1 = 0;
    nucLongStatuse2 = 0;
    for (auccounter = 0; auccounter < 8; auccounter++)
    {
        /*nucDuoJIDTbl多机防碰撞设置ID-80B0、80B2、80B4、80B6、80B8、80BA、80BC、80BE*/
        if (nucDuoJIDTbl[auccounter] != 0 && nucDuoJTowerLongTbl[auccounter] != 0 && nucDuoJXDDisdanceTbl[auccounter] != 0)
        {
//
//          LOG("nucDuoJAngleTbl[auccounter]= %f\n",nucDuoJAngleTbl[auccounter]);
//              LOG("nucDuoJLongTbl[auccounter]= %d\n",nucDuoJTowerLongTbl[auccounter]);
//              LOG("nucDuoJHighTbl[auccounter]= %d\n",nucDuoJXDDisdanceTbl[auccounter]);

            if (nucDuoJAngleTbl[auccounter] == 0 && nucDuoJLongTbl[auccounter] == 0 && nucDuoJHighTbl[auccounter] == 0)
            {
                continue;
            }
#if 1
            if (g_dj_leixing == 1 || nucDanJEndHighTbl[auccounter] == 1)
            {
                if (g_dj_leixing == 1) //本机动臂吊
                {
                    //相对距离 > 本机幅度+幅度误差+多机臂长
                    if (nucDuoJXDDisdanceTbl[auccounter] > nucLongData * 100 + nucDuoJTowerLongTbl_temp[auccounter] + nucDuoJTowerLongTbl[auccounter])
                    {
                        continue;
                    }
                }
                else if (nucDanJEndHighTbl[auccounter] == 1) //多机动臂吊
                {
                    //相对距离 > 本机臂长+幅度误差+多机幅度
                    if (nucDuoJXDDisdanceTbl[auccounter] > nucTowerLong + nucDuoJTowerLongTbl_temp[auccounter] + nucDuoJLongTbl[auccounter] * 100)
                    {
                        continue;
                    }
                }
                else
                {
                    //相对距离 > 本机幅度+幅度误差+多机幅度
                    if (nucDuoJXDDisdanceTbl[auccounter] > nucLongData * 100 + nucDuoJTowerLongTbl_temp[auccounter] + nucDuoJLongTbl[auccounter] * 100)
                    {
                        continue;
                    }
                }
            }
            //相对高度为正同时>高度误差  (本机塔臂高 > 多机塔臂高 )
            if ((int16_t)nucDuoJXDHighTbl[auccounter] > nucDuoJXDHighTbl_temp[auccounter])
            {

            }
            //(本机塔臂高 < 多机塔臂高 )
            else if (abs((int16_t)nucDuoJXDHighTbl[auccounter]) > nucDuoJXDHighTbl_temp[auccounter])
            {
                //多机钩不在 区域A内
#if 0
                //距离-本机幅度 +幅度差 < 多机后臂
                if ((nucDuoJXDDisdanceTbl[auccounter] - nucLongData * 100 + nucDuoJTowerLongTbl_temp[auccounter]) < nucDuoJTowerLongTbl_back[auccounter])
                {
                    continue;
                }
#else
                //距离>本机臂长 +幅度差 + 多机臂长
                if (nucDuoJXDDisdanceTbl[auccounter] > nucTowerLong + nucDuoJTowerLongTbl_temp[auccounter] + nucDuoJLongTbl[auccounter] * 100)
                {
                    continue;
                }
#endif
            }
#endif
            aucAngle[1] = nucDuoJAngleTbl[auccounter];          //多机当前回转角度
            aucAngleDJ[0] = nucDuoJXDAngleTbl[auccounter];  //本机当前设置相对角度
            if (aucAngleDJ[0] >= 0 && aucAngleDJ[0] < 180)                 aucAngleDJ[1] = aucAngleDJ[0] + 180; //多机当前相对角度
            else if (aucAngleDJ[0] >= 180 && aucAngleDJ[0] < 360)  aucAngleDJ[1] = aucAngleDJ[0] - 180; //多机当前相对角度

//            if(nucDanJEndHighTbl[auccounter] == 1)//多机是动臂吊
//            {
//                aucDuoJiLong=nucDuoJLongTbl[auccounter]*100+nucDuoJTowerLongTbl_temp[auccounter]; //多机幅度
//                aucDuoJiLong=aucDuoJiLong/100;                  //多机幅度
//            }
//            else
            {
                aucDuoJiLong = nucDuoJTowerLongTbl[auccounter] + nucDuoJTowerLongTbl_temp[auccounter]; //多机臂长
                aucDuoJiLong = aucDuoJiLong / 100;                          //多机臂长
            }
            aucDuoJiDisdance = nucDuoJXDDisdanceTbl[auccounter];    //多机相对距离
            aucDuoJiDisdance = aucDuoJiDisdance / 100;                          //多机相对距离
            if (aucDuoJiLong > 0) /*臂长大于0*/
            {
                float a0, a1, a2, a3;
                //本机为参考,相交扇形区域,最大扇形夹角的一半
                a = ((acos((aucMaxLong * aucMaxLong + aucDuoJiDisdance * aucDuoJiDisdance - aucDuoJiLong * aucDuoJiLong) / (2 * aucMaxLong * aucDuoJiDisdance))) * 180) / PI;

                float   a_temp = 0; //(float)DUOJI_ALARM_DATA_1/100;;//增加扇区最大边界,防止交点正好在扇区边界
                AngleBuff[0][0] = ((float)(((uint32_t)((aucAngleDJ[0] + 360 - a - a_temp) * 100)) % 36000)) / 100; //本机交叉起始角度
                AngleBuff[0][1] = ((float)(((uint32_t)((aucAngleDJ[0] + a + a_temp) * 100)) % 36000)) / 100;    //本机交叉结束角度

                if (AngleBuff[0][0] < aucAngleDJ[0] && aucAngleDJ[0] < AngleBuff[0][1])                 //不跨零点
                {
                    a1 = AngleBuff[0][0];
                    a2 = aucAngleDJ[0];
                    a3 = AngleBuff[0][1];

                    a0 = aucAngle[0];
                }
                else if (AngleBuff[0][0] < aucAngleDJ[0] && aucAngleDJ[0] > AngleBuff[0][1])        //结束角度跨零点
                {
                    a1 = AngleBuff[0][0];
                    a2 = aucAngleDJ[0];
                    a3 = AngleBuff[0][1] + 360;

                    if (aucAngle[0] < AngleBuff[0][1])     a0 = aucAngle[0] + 360;
                    else                                                            a0 = aucAngle[0];
                }
                else if (AngleBuff[0][0] > aucAngleDJ[0] && aucAngleDJ[0] < AngleBuff[0][1])        //相对角度和结束角度都跨零点
                {
                    a1 = AngleBuff[0][0];
                    a2 = aucAngleDJ[0] + 360;
                    a3 = AngleBuff[0][1] + 360;

                    if (aucAngle[0] < AngleBuff[0][1])     a0 = aucAngle[0] + 360;
                    else                                                            a0 = aucAngle[0];
                }
                else
                {
                    a1 = 0;
                    a2 = 1;
                    a3 = 2;
                    a0 = 3;
                }

                if (a0 < a1 || a0 > a3)     //本机未进入交叉区域
                {
                    flg[0][0] = 0;
                    flg[0][1] = 0;
                }
                else                                    //本机进入交叉区域
                {
                    if (a0 < a2)                    //进入前半个扇区
                    {
                        flg[0][0] = 1;
                        flg[0][1] = 0;
                    }
                    else                                //进入后半个扇区
                    {
                        flg[0][0] = 0;
                        flg[0][1] = 1;
                    }

                }
                a = ((acos((aucDuoJiLong * aucDuoJiLong + aucDuoJiDisdance * aucDuoJiDisdance - aucMaxLong * aucMaxLong) / (2 * aucDuoJiLong * aucDuoJiDisdance))) * 180) / PI;

                AngleBuff[1][0] = ((float)(((uint32_t)((aucAngleDJ[1] + 360 - a - a_temp) * 100)) % 36000)) / 100; //多机交叉起始角度
                AngleBuff[1][1] = ((float)(((uint32_t)((aucAngleDJ[1] + a + a_temp) * 100)) % 36000)) / 100;    //多机交叉结束角度

                if (AngleBuff[1][0] < aucAngleDJ[1] && aucAngleDJ[1] < AngleBuff[1][1])                 //不跨零点
                {
                    a1 = AngleBuff[1][0];
                    a2 = aucAngleDJ[1];
                    a3 = AngleBuff[1][1];

                    a0 = aucAngle[1];
                }
                else if (AngleBuff[1][0] < aucAngleDJ[1] && aucAngleDJ[1] > AngleBuff[1][1])        //结束角度跨零点
                {
                    a1 = AngleBuff[1][0];
                    a2 = aucAngleDJ[1];
                    a3 = AngleBuff[1][1] + 360;

                    if (aucAngle[1] < AngleBuff[1][1])     a0 = aucAngle[1] + 360;
                    else                                                            a0 = aucAngle[1];
                }
                else if (AngleBuff[1][0] > aucAngleDJ[1] && aucAngleDJ[1] < AngleBuff[1][1])        //相对角度和结束角度都跨零点
                {
                    a1 = AngleBuff[1][0];
                    a2 = aucAngleDJ[1] + 360;
                    a3 = AngleBuff[1][1] + 360;

                    if (aucAngle[1] < AngleBuff[1][1])     a0 = aucAngle[1] + 360;
                    else                                                            a0 = aucAngle[1];
                }
                else
                {
                    a1 = 0;
                    a2 = 1;
                    a3 = 2;
                    a0 = 3;
                }

                if (a0 < a1 || a0 > a3)     //多机未进入交叉区域
                {
                    flg[1][0] = 0;
                    flg[1][1] = 0;

                }
                else                                    //多机进入交叉区域
                {
                    if (a0 < a2)                    //进入前半个扇区
                    {
                        flg[1][0] = 1;
                        flg[1][1] = 0;
                    }
                    else                                //进入后半个扇区
                    {
                        flg[1][0] = 0;
                        flg[1][1] = 1;
                    }
                }
                uint8_t range0 = 0, range1 = 0; //非交半区提示
                if ((flg[0][0] == 0 && flg[0][1] == 0)  //本机不在区域，不报警
                        || (flg[1][0] == 0 && flg[1][1] == 0)) //多机不在区域，不报警
                {
                    nucDuojiStatuse = 0;
                    nucSysDuoJiControlFlag = 0;
                    g_bc_last_num = 0;
                }
                else
                {
                    if (flg[0][0] == 1 && flg[0][1] == 0)           //本机在前半扇区
                    {
                        range1 = 1;
                        if (flg[1][0] == 1 && flg[1][1] == 0)       //多机在前半扇区，不在半个扇区，预警
                        {
                            range0 = 1; //提示范围,从下半区进入
                            g_duoji_rsta_warn[auccounter] = 1;
                            g_duoji_rsta_alarm[auccounter] = 0;
                        }
                        else if (flg[1][0] == 0 && flg[1][1] == 1) //多机在后半扇区，在半个扇区，报警
                        {
                            nucDuojiStatuse = 5; //上半区相交
                            g_duoji_rsta_alarm[auccounter] = 5;
                        }
                    }
                    else if (flg[0][0] == 0 && flg[0][1] == 1)  //本机在后半扇区
                    {
                        range1 = 2;
                        if (flg[1][0] == 1 && flg[1][1] == 0)       //多机在前半扇区，在半个扇区，报警
                        {
                            nucDuojiStatuse = 6; //下半区相交
                            g_duoji_rsta_alarm[auccounter] = 6;
                        }
                        else if (flg[1][0] == 0 && flg[1][1] == 1) //多机在后半扇区，不在半个扇区，预警
                        {
                            range0 = 2; //提示范围,从上半区进入
                            g_duoji_rsta_warn[auccounter] = 2;
                            g_duoji_rsta_alarm[auccounter] = 0;
                        }
                    }

                    if (nucDuojiStatuse >= 5 || range0 > 0)
                    {
                        float side_1;
                        float angle_1, angle_2, angle_3, angle_4, angle_a1, angle_a2, angle_1_a, angle_2_a;

                        angle_1 = fabs(aucAngleDJ[1] - aucAngle[1]); //多机对应中分线的角度
                        if (angle_1 > 180)
                        {
                            angle_1 = 360 - angle_1;
                        }
                        angle_1_a = angle_1;
                        angle_2_a = fabs(aucAngleDJ[0] - aucAngle[0]); //本机臂长 与 中分线的角度 (相对角度-本机回转)
                        if (angle_2_a > 180)
                        {
                            angle_2_a = 360 - angle_2_a;
                        }

                        angle_1 = (PI * angle_1) / 180;
                        //余弦定理
                        side_1 = sqrt(aucDuoJiLong * aucDuoJiLong + aucDuoJiDisdance * aucDuoJiDisdance - 2 * aucDuoJiLong * aucDuoJiDisdance * cos(angle_1)); //判断范围-大三角形-- 本机与多机相交点的臂长

                        angle_4 = ((acos((side_1 * side_1 + aucDuoJiDisdance * aucDuoJiDisdance - aucDuoJiLong * aucDuoJiLong) / (2 * side_1 * aucDuoJiDisdance))) * 180) / PI; //判断范围-大三角形-- 本机臂长与中分线的角度

                        //正弦定理
                        angle_3 = aucDuoJiDisdance * ((float)sin(angle_1)) / aucMaxLong; //判断范围-小三角形-- 钝角
                        angle_3 = 180 - ((asin(angle_3)) * 180) / PI;
                        angle_2 = 180 - angle_3 - angle_1 * 180 / PI; //判断范围-小三角形-- 本机臂长与中分线的角度

                        if (nucDuojiStatuse == 5 || range0 == 2)
                        {
                            angle_a1 = aucAngleDJ[0] - angle_4;
                            angle_a2 = aucAngleDJ[0] - angle_2;
                        }
                        else
                        {
                            angle_a1 = aucAngleDJ[0] + angle_2;
                            angle_a2 = aucAngleDJ[0] + angle_4;
                        }

                        float aucAngle0 = aucAngle[0];
                        float g_angle_alarm = (float)nucDanJEndLongTbl[auccounter] / 100;
                        float g_angle_warn = (float)nucDanJStartLongTbl[auccounter] / 100;
                        //  1, a1 < 0
                        //  2, a2 >360
                        angle_a1 = angle_a1 - g_angle_warn;
                        angle_a2 = angle_a2 + g_angle_warn;

                        if (angle_a1 < 0)
                        {
                            if (aucAngle0 < 360 && aucAngle0 > 180)
                            {
                                aucAngle0 = aucAngle0 - 360;
                            }
                        }
                        else if (angle_a2 > 360)
                        {
                            if (aucAngle0 < 180)
                            {
                                aucAngle0 = aucAngle0 + 360;
                            }
                        }
                        //本机 高度 > 多机
                        if ((int16_t)nucDuoJXDHighTbl[auccounter] > nucDuoJXDHighTbl_temp[auccounter])
                        {
                            float cha = (float)nucDuoJTowerLongTbl_temp[auccounter] / 100;
                            // 幅度满足碰撞条件
                            if (nucLongData + cha > side_1)
                            {
                                float bc = bianchang(angle_2_a, angle_1_a, aucDuoJiDisdance);

//                                                                  LOG("nucLongData[%.2f][%.2f],aucAngle0[%.2f]-[%.2f][%.2f]\n",nucLongData,bc,aucAngle0,angle_a1,angle_a2);
                                if (bc < 0)
                                {
                                    bc = g_bc_last_save[auccounter];
                                }
                                else if (bc > nucTowerLong / 100)
                                {
                                    bc = nucTowerLong / 100;
                                }
                                g_bc_last_save[auccounter] = bc;

//                                LOG("nucLongData[%.2f][%.2f],aucAngle0[%.2f]-[%.2f][%.2f]\n",nucLongData,bc,aucAngle0,angle_a1,angle_a2);
//                                LOG("bc=%f,ent[%d]-[%d]\n",bc,g_duoji_rsta_warn[auccounter],g_duoji_rsta_alarm[auccounter]);

                                if (nucLongData + cha < bc && g_duoji_rsta_alarm[auccounter] >= 5) //同一区域
                                {
                                    bc = (bc - nucLongData) * sin((PI * (180 - angle_3)) / 180);

#if 0
//                                    LOG("cha[%.2f][%.2f]-[%.2f]\n",cha,bc,side_1);
                                    if (cha > bc) // 幅度报警
                                    {
                                        g_duoji_relay1[auccounter] = 1;

                                        angle_a1 = angle_a1 + (g_angle_warn - g_angle_alarm);
                                        angle_a2 = angle_a2 - (g_angle_warn - g_angle_alarm);

                                        if (aucAngle0 >= angle_a1 && aucAngle0 <= angle_a2)
                                        {
                                            if (nucControlFlag > 0 && nucDebugFlag == 0)
                                            {
                                                if (nucDuojiStatuse == 5)
                                                {
                                                    if (angle_2_a > angle_1_a)
                                                    {
                                                        nucSysDuoJiControlFlag = 1;//禁止右转
                                                    }
                                                    else if (angle_2_a < angle_1_a)
                                                    {
                                                        nucSysDuoJiControlFlag = 2;//禁止左转
                                                    }
                                                }
                                                else if (nucDuojiStatuse == 6)
                                                {
                                                    if (angle_2_a > angle_1_a)
                                                    {
                                                        nucSysDuoJiControlFlag = 2;//禁止左转
                                                    }
                                                    else if (angle_2_a < angle_1_a)
                                                    {
                                                        nucSysDuoJiControlFlag = 1;//禁止右转
                                                    }
                                                }
                                                else if (range0 == 1)
                                                {
                                                    nucSysDuoJiControlFlag = 1;//禁止右转
                                                }
                                                else if (range0 == 2)
                                                {
                                                    nucSysDuoJiControlFlag = 2;//禁止左转
                                                }
                                            }

                                            nucDuojiStatuse = 2;//报警
                                        }
                                        else
                                        {
                                            nucSysDuoJiControlFlag = 0;
                                            nucDuojiStatuse = 1;//预警
                                        }
                                    }
                                    else if (cha + 5 > bc) // 幅度预警
                                    {
                                        g_duoji_relay2[auccounter] = 1;
                                        nucSysDuoJiControlFlag = 0;
                                        nucDuojiStatuse = 1;//预警
                                    }
                                    else
                                    {
                                        continue;
                                    }
#else
                                    angle_a1 = angle_a1 + (g_angle_warn - g_angle_alarm);
                                    angle_a2 = angle_a2 - (g_angle_warn - g_angle_alarm);

                                    if (aucAngle0 >= angle_a1 && aucAngle0 <= angle_a2)
                                    {
                                        if (nucControlFlag > 0 && nucDebugFlag == 0)
                                        {
                                            if (nucDuojiStatuse == 5)
                                            {
                                                if (angle_2_a > angle_1_a)
                                                {
                                                    nucSysDuoJiControlFlag = 1;//禁止右转
                                                }
                                                else if (angle_2_a < angle_1_a)
                                                {
                                                    nucSysDuoJiControlFlag = 2;//禁止左转
                                                }
                                            }
                                            else if (nucDuojiStatuse == 6)
                                            {
                                                if (angle_2_a > angle_1_a)
                                                {
                                                    nucSysDuoJiControlFlag = 2;//禁止左转
                                                }
                                                else if (angle_2_a < angle_1_a)
                                                {
                                                    nucSysDuoJiControlFlag = 1;//禁止右转
                                                }
                                            }
                                            else if (range0 == 1)
                                            {
                                                nucSysDuoJiControlFlag = 1;//禁止右转
                                            }
                                            else if (range0 == 2)
                                            {
                                                nucSysDuoJiControlFlag = 2;//禁止左转
                                            }
                                        }

                                        nucDuojiStatuse = 2;//报警
                                    }
                                    else
                                    {
                                        nucSysDuoJiControlFlag = 0;
                                        nucDuojiStatuse = 1;//预警
                                    }

                                    if (cha > bc) // 幅度报警
                                    {
                                        g_duoji_relay1[auccounter] = 1;
                                        nucDuojiStatuse = 2;//报警
                                    }
                                    else if (cha + 5 > bc) // 幅度预警
                                    {
                                        g_duoji_relay2[auccounter] = 1;
                                        if (nucDuojiStatuse < 2)
                                        {
                                            nucDuojiStatuse = 1;//预警
                                        }
                                    }
//                                    else
//                                    {
//                                        continue;
//                                    }
#endif
                                }
//                                else if(nucLongData>=bc&&aucAngle0>=angle_a1 && aucAngle0<=angle_a2)
                                else if (aucAngle0 >= angle_a1 && aucAngle0 <= angle_a2)
                                {
                                    angle_a1 = angle_a1 + (g_angle_warn - g_angle_alarm);
                                    angle_a2 = angle_a2 - (g_angle_warn - g_angle_alarm);

                                    if (aucAngle0 >= angle_a1 && aucAngle0 <= angle_a2)
                                    {
                                        if (nucControlFlag > 0 && nucDebugFlag == 0)
                                        {
                                            if (nucDuojiStatuse == 5)
                                            {
                                                if (angle_2_a > angle_1_a)
                                                {
                                                    nucSysDuoJiControlFlag = 1;//禁止右转
                                                }
                                                else if (angle_2_a < angle_1_a)
                                                {
                                                    nucSysDuoJiControlFlag = 2;//禁止左转
                                                }
                                            }
                                            else if (nucDuojiStatuse == 6)
                                            {
                                                if (angle_2_a > angle_1_a)
                                                {
                                                    nucSysDuoJiControlFlag = 2;//禁止左转
                                                }
                                                else if (angle_2_a < angle_1_a)
                                                {
                                                    nucSysDuoJiControlFlag = 1;//禁止右转
                                                }
                                            }
                                            else if (range0 == 1)
                                            {
                                                nucSysDuoJiControlFlag = 1;//禁止右转
                                            }
                                            else if (range0 == 2)
                                            {
                                                nucSysDuoJiControlFlag = 2;//禁止左转
                                            }
                                        }
                                        nucDuojiStatuse = 2;//报警
                                    }
                                    else
                                    {
                                        nucSysDuoJiControlFlag = 0;
                                        nucDuojiStatuse = 1;//预警
                                    }
                                    g_duoji_id = nucDuoJIDTbl[auccounter];
                                }
                                else
                                {
                                    nucSysDuoJiControlFlag = 0;
                                    nucDuojiStatuse = 0;
                                }
                            }
                            else
                            {
                                nucSysDuoJiControlFlag = 0;
                                nucDuojiStatuse = 0;
                            }

                            g_duoji_id = nucDuoJIDTbl[auccounter];
                        }
                        //回转判断
                        else if (aucAngle0 >= angle_a1 && aucAngle0 <= angle_a2)
                        {
                            angle_a1 = angle_a1 + (g_angle_warn - g_angle_alarm);
                            angle_a2 = angle_a2 - (g_angle_warn - g_angle_alarm);

                            if (aucAngle0 >= angle_a1 && aucAngle0 <= angle_a2)
                            {
                                if (nucControlFlag > 0 && nucDebugFlag == 0)
                                {
                                    if (nucDuojiStatuse == 5)
                                    {
                                        if (angle_2_a > angle_1_a)
                                        {
                                            nucSysDuoJiControlFlag = 1;//禁止右转
                                        }
                                        else if (angle_2_a < angle_1_a)
                                        {
                                            nucSysDuoJiControlFlag = 2;//禁止左转
                                        }
                                    }
                                    else if (nucDuojiStatuse == 6)
                                    {
                                        if (angle_2_a > angle_1_a)
                                        {
                                            nucSysDuoJiControlFlag = 2;//禁止左转
                                        }
                                        else if (angle_2_a < angle_1_a)
                                        {
                                            nucSysDuoJiControlFlag = 1;//禁止右转
                                        }
                                    }
                                    else if (range0 == 1)
                                    {
                                        nucSysDuoJiControlFlag = 1;//禁止右转
                                    }
                                    else if (range0 == 2)
                                    {
                                        nucSysDuoJiControlFlag = 2;//禁止左转
                                    }
                                    if (range0 > 0)
                                    {
//                                        LOG("dj range[%d]\n",range0);
                                    }
                                }
                                nucDuojiStatuse = 2;//报警
                            }
                            else
                            {
                                nucSysDuoJiControlFlag = 0;
                                nucDuojiStatuse = 1;//预警
                            }
                            g_duoji_id = nucDuoJIDTbl[auccounter];
                        }
                        else
                        {
                            nucSysDuoJiControlFlag = 0;
                            nucDuojiStatuse = 0;
                        }

                        g_duoji_state[auccounter] = nucDuojiStatuse; //记录某一个多机 记录报警状态
                        g_duoji_relay[auccounter] = nucSysDuoJiControlFlag; //记录某一个多机 记录继电器状态
                    }
                }
            }
        }
        else
        {
//            nucSysDuoJiControlFlag = 0;
//            nucDuojiStatuse=0;
        }
    }

    int i;
    for (i = 0; i < 8; i++)
    {
        if (g_duoji_state[i] == 2)
        {
            nucDuojiStatuse = 2;
            break;
        }
    }

    if (nucDuojiStatuse != 2)
    {
        for (i = 0; i < 8; i++)
        {
            if (g_duoji_state[i] == 1)
            {
                nucDuojiStatuse = 1;
                break;
            }
        }
    }

    //多机幅度 向外报警
    for (i = 0; i < 8; i++)
    {
        if (g_duoji_relay1[i] == 1)
        {
            nucLongStatuse1 = 2; //多机幅度向外控制
            break;
        }
    }
    //多机幅度向外 预警
    if (nucLongStatuse1 != 2)
    {
        for (i = 0; i < 8; i++)
        {
            if (g_duoji_relay2[i] == 1)
            {
                nucLongStatuse2 = 1;
                break;
            }
        }
    }

    uint8_t duoji_relay[2] = {0};
    for (i = 0; i < 8; i++)
    {
        if (g_duoji_relay[i] == 1)
        {
            duoji_relay[0] = 1;
        }
        else if (g_duoji_relay[i] == 2)
        {
            duoji_relay[1] = 2;
        }


    }
    if (g_alarm_able.duoji)
    {
        nucSysDuoJiControlFlag = duoji_relay[0] + duoji_relay[1];
    }
    else
    {
        nucSysDuoJiControlFlag = 0;
    }

    nucDuoJFPZJudge2();
    nucDuoJFPZJudge3();
}
//本机大臂防碰撞-多机后臂
void nucDuoJFPZJudge2(void)
{
    if (nucDuojiStatuse > 0)
    {
        return;
    }

    uint8_t auccounter = 0;
    float   aucMaxLong = 0;
    float   aucDuoJiLong = 0;
    float   aucDuoJiDisdance = 0;
    float AngleBuff[2][2];
    float aucAngle[2], aucAngleDJ[2];
    float   a;
    uint8_t flg[2][2];
    uint8_t g_duoji_state[8] = {0}; //记录报警状态
    uint8_t g_duoji_relay[8] = {0}; //记录继电器状态
    uint8_t g_duoji_relay1[8] = {0}; //记录继电器状态 幅度向外
    uint8_t g_duoji_relay2[8] = {0}; //记录继电器状态 幅度向外
//////////////////////////////////////////////////////
//多机防碰撞判断,配合 碰撞模型图计算
//////////////////////////////////////////////////////
//    if(g_dj_leixing == 1)//动臂吊
//    {
//        aucMaxLong=nucLongData;
//    }
//    else
    {
        aucMaxLong = nucTowerLong;  //本机臂长
        aucMaxLong = aucMaxLong / 100; //本机臂长
    }

    aucAngle[0] = nucAngleData;     //本机当前回转角度
//    nucDuojiStatuse=0;

//    nucLongStatuse1=0;
//    nucLongStatuse2=0;
    for (auccounter = 0; auccounter < 8; auccounter++)
    {
        if (nucDuoJIDTbl[auccounter] != 0 && nucDuoJTowerLongTbl_back[auccounter] != 0 && nucDuoJXDDisdanceTbl[auccounter] != 0)
        {
            if (nucDuoJAngleTbl[auccounter] == 0 && nucDuoJLongTbl[auccounter] == 0 && nucDuoJHighTbl[auccounter] == 0)
            {
                continue;
            }
#if 1
            //本机是动臂吊20250721
            if (g_dj_leixing == 1)
            {
                //相对距离 > 本机幅度+幅度误差+多机后臂长
                if (nucDuoJXDDisdanceTbl[auccounter] > nucLongData * 100 + nucDuoJTowerLongTbl_temp[auccounter] + nucDuoJTowerLongTbl_back[auccounter])
                {
                    continue;
                }
            }
            //相对高度为正同时>高度误差  (本机塔臂高 > 多机塔臂高 )
            else if ((int16_t)nucDuoJXDHighTbl[auccounter] > nucDuoJXDHighTbl_temp[auccounter])
            {
                //本机钩在区域B内
                //距离-多机后臂 > 本机幅度+幅度误差
                if (nucDuoJXDDisdanceTbl[auccounter] - nucDuoJTowerLongTbl_back[auccounter] > nucLongData * 100 + nucDuoJTowerLongTbl_temp[auccounter])
                {
                    continue;
                }
            }
            //(本机塔臂高 < 多机塔臂高 )
            else if (abs((int16_t)nucDuoJXDHighTbl[auccounter]) > nucDuoJXDHighTbl_temp[auccounter])
            {
                //多机钩不在 区域A内
#if 0
                //距离-本机幅度 +幅度差 < 多机后臂
                if ((nucDuoJXDDisdanceTbl[auccounter] - nucLongData * 100 + nucDuoJTowerLongTbl_temp[auccounter]) < nucDuoJTowerLongTbl_back[auccounter])
                {
                    continue;
                }
#else
                //距离>本机臂长 +幅度差
                if (nucDuoJXDDisdanceTbl[auccounter] > nucTowerLong + nucDuoJTowerLongTbl_temp[auccounter])
                {
                    continue;
                }

#endif
            }
#endif
            aucAngle[1] = nucDuoJAngleTbl[auccounter];          //多机当前回转角度
            if (aucAngle[1] > 180)
            {
                aucAngle[1] -= 180;
            }
            else
            {
                aucAngle[1] += 180;
            }
            aucAngleDJ[0] = nucDuoJXDAngleTbl[auccounter];  //本机当前设置相对角度
            if (aucAngleDJ[0] >= 0 && aucAngleDJ[0] < 180)                 aucAngleDJ[1] = aucAngleDJ[0] + 180; //多机当前相对角度
            else if (aucAngleDJ[0] >= 180 && aucAngleDJ[0] < 360)  aucAngleDJ[1] = aucAngleDJ[0] - 180; //多机当前相对角度
            aucDuoJiLong = nucDuoJTowerLongTbl_back[auccounter] + nucDuoJTowerLongTbl_temp[auccounter]; //多机后臂长+幅度比较误差
            aucDuoJiLong = aucDuoJiLong / 100;                          //多机后臂长
            aucDuoJiDisdance = nucDuoJXDDisdanceTbl[auccounter];    //多机相对距离
            aucDuoJiDisdance = aucDuoJiDisdance / 100;                          //多机相对距离
            if (aucDuoJiLong > 0) /*臂长大于0*/
            {
                float a0, a1, a2, a3;
                //本机为参考,相交扇形区域,最大扇形夹角的一半
                a = ((acos((aucMaxLong * aucMaxLong + aucDuoJiDisdance * aucDuoJiDisdance - aucDuoJiLong * aucDuoJiLong) / (2 * aucMaxLong * aucDuoJiDisdance))) * 180) / PI;

                float   a_temp = 0; //(float)DUOJI_ALARM_DATA_1/100;;//增加扇区最大边界,防止交点正好在扇区边界
                AngleBuff[0][0] = ((float)(((uint32_t)((aucAngleDJ[0] + 360 - a - a_temp) * 100)) % 36000)) / 100; //本机交叉起始角度
                AngleBuff[0][1] = ((float)(((uint32_t)((aucAngleDJ[0] + a + a_temp) * 100)) % 36000)) / 100;    //本机交叉结束角度

                if (AngleBuff[0][0] < aucAngleDJ[0] && aucAngleDJ[0] < AngleBuff[0][1])                 //不跨零点
                {
                    a1 = AngleBuff[0][0];
                    a2 = aucAngleDJ[0];
                    a3 = AngleBuff[0][1];

                    a0 = aucAngle[0];
                }
                else if (AngleBuff[0][0] < aucAngleDJ[0] && aucAngleDJ[0] > AngleBuff[0][1])        //结束角度跨零点
                {
                    a1 = AngleBuff[0][0];
                    a2 = aucAngleDJ[0];
                    a3 = AngleBuff[0][1] + 360;

                    if (aucAngle[0] < AngleBuff[0][1])     a0 = aucAngle[0] + 360;
                    else                                                            a0 = aucAngle[0];
                }
                else if (AngleBuff[0][0] > aucAngleDJ[0] && aucAngleDJ[0] < AngleBuff[0][1])        //相对角度和结束角度都跨零点
                {
                    a1 = AngleBuff[0][0];
                    a2 = aucAngleDJ[0] + 360;
                    a3 = AngleBuff[0][1] + 360;

                    if (aucAngle[0] < AngleBuff[0][1])     a0 = aucAngle[0] + 360;
                    else                                                            a0 = aucAngle[0];
                }
                else
                {
                    a1 = 0;
                    a2 = 1;
                    a3 = 2;
                    a0 = 3;
                }

                if (a0 < a1 || a0 > a3)     //本机未进入交叉区域
                {
                    flg[0][0] = 0;
                    flg[0][1] = 0;
                }
                else                                    //本机进入交叉区域
                {
                    if (a0 < a2)                    //进入前半个扇区
                    {
                        flg[0][0] = 1;
                        flg[0][1] = 0;
                    }
                    else                                //进入后半个扇区
                    {
                        flg[0][0] = 0;
                        flg[0][1] = 1;
                    }

                }
                a = ((acos((aucDuoJiLong * aucDuoJiLong + aucDuoJiDisdance * aucDuoJiDisdance - aucMaxLong * aucMaxLong) / (2 * aucDuoJiLong * aucDuoJiDisdance))) * 180) / PI;

                AngleBuff[1][0] = ((float)(((uint32_t)((aucAngleDJ[1] + 360 - a - a_temp) * 100)) % 36000)) / 100; //多机交叉起始角度
                AngleBuff[1][1] = ((float)(((uint32_t)((aucAngleDJ[1] + a + a_temp) * 100)) % 36000)) / 100;    //多机交叉结束角度

                if (AngleBuff[1][0] < aucAngleDJ[1] && aucAngleDJ[1] < AngleBuff[1][1])                 //不跨零点
                {
                    a1 = AngleBuff[1][0];
                    a2 = aucAngleDJ[1];
                    a3 = AngleBuff[1][1];

                    a0 = aucAngle[1];
                }
                else if (AngleBuff[1][0] < aucAngleDJ[1] && aucAngleDJ[1] > AngleBuff[1][1])        //结束角度跨零点
                {
                    a1 = AngleBuff[1][0];
                    a2 = aucAngleDJ[1];
                    a3 = AngleBuff[1][1] + 360;

                    if (aucAngle[1] < AngleBuff[1][1])     a0 = aucAngle[1] + 360;
                    else                                                            a0 = aucAngle[1];
                }
                else if (AngleBuff[1][0] > aucAngleDJ[1] && aucAngleDJ[1] < AngleBuff[1][1])        //相对角度和结束角度都跨零点
                {
                    a1 = AngleBuff[1][0];
                    a2 = aucAngleDJ[1] + 360;
                    a3 = AngleBuff[1][1] + 360;

                    if (aucAngle[1] < AngleBuff[1][1])     a0 = aucAngle[1] + 360;
                    else                                                            a0 = aucAngle[1];
                }
                else
                {
                    a1 = 0;
                    a2 = 1;
                    a3 = 2;
                    a0 = 3;
                }

                if (a0 < a1 || a0 > a3)     //多机未进入交叉区域
                {
                    flg[1][0] = 0;
                    flg[1][1] = 0;

                }
                else                                    //多机进入交叉区域
                {
                    if (a0 < a2)                    //进入前半个扇区
                    {
                        flg[1][0] = 1;
                        flg[1][1] = 0;
                    }
                    else                                //进入后半个扇区
                    {
                        flg[1][0] = 0;
                        flg[1][1] = 1;
                    }
                }
                uint8_t range0 = 0; //非交半区提示
                if ((flg[0][0] == 0 && flg[0][1] == 0)  //本机不在区域，不报警
                        || (flg[1][0] == 0 && flg[1][1] == 0)) //多机不在区域，不报警
                {
                    nucDuojiStatuse = 0;
                    nucSysDuoJiControlFlag = 0;

                }
                else
                {
                    if (flg[0][0] == 1 && flg[0][1] == 0)           //本机在前半扇区
                    {
                        if (flg[1][0] == 1 && flg[1][1] == 0)       //多机在前半扇区，不在半个扇区，预警
                        {
//                            auccounter=8;
                            range0 = 1; //提示范围,从下半区进入
                            g_duoji_rsta_alarm[auccounter] = 0;
                        }
                        else if (flg[1][0] == 0 && flg[1][1] == 1) //多机在后半扇区，在半个扇区，报警
                        {
                            nucDuojiStatuse = 5; //上半区相交
                            g_duoji_rsta_alarm[auccounter] = 5;
//                            auccounter=8;
                        }
                    }
                    else if (flg[0][0] == 0 && flg[0][1] == 1)  //本机在后半扇区
                    {
                        if (flg[1][0] == 1 && flg[1][1] == 0)       //多机在前半扇区，在半个扇区，报警
                        {
                            nucDuojiStatuse = 6; //下半区相交
                            g_duoji_rsta_alarm[auccounter] = 6;
//                            auccounter=8;
                        }
                        else if (flg[1][0] == 0 && flg[1][1] == 1) //多机在后半扇区，不在半个扇区，预警
                        {
//                            auccounter=8;
                            range0 = 2; //提示范围,从上半区进入
                            g_duoji_rsta_alarm[auccounter] = 0;
                        }
                    }

                    if (nucDuojiStatuse >= 5 || range0 > 0)
                    {
                        float side_1;
                        float angle_1, angle_2, angle_3, angle_4, angle_a1, angle_a2, angle_1_a, angle_2_a;
#if 0
                        angle_1 = fabs(aucAngleDJ[1] - aucAngle[1]); //多机对应中分线的角度
                        angle_1_a = angle_1;
                        angle_2_a = fabs(aucAngleDJ[0] - aucAngle[0]); //本机臂长 与 中分线的角度
#else
                        angle_1 = fabs(aucAngleDJ[1] - aucAngle[1]); //多机对应中分线的角度
                        if (angle_1 > 180)
                        {
                            angle_1 = 360 - angle_1;
                        }
                        angle_1_a = angle_1;
                        angle_2_a = fabs(aucAngleDJ[0] - aucAngle[0]); //本机臂长 与 中分线的角度
                        if (angle_2_a > 180)
                        {
                            angle_2_a = 360 - angle_2_a;
                        }
#endif

//                        LOG("aucAngle[%f],aucAngleDJ= %f\n",aucAngle[1],aucAngleDJ[1]);
//                        LOG("angle compare [%f]:[%f]\n",angle_2_a,angle_1_a);
                        angle_1 = (PI * angle_1) / 180;
                        //余弦定理
                        side_1 = sqrt(aucDuoJiLong * aucDuoJiLong + aucDuoJiDisdance * aucDuoJiDisdance - 2 * aucDuoJiLong * aucDuoJiDisdance * cos(angle_1)); //判断范围-大三角形-- 本机与多机相交点的臂长
//                            LOG("side_1= %f\n",side_1);
#if 0
                        // (本机塔臂高 > 多机塔臂高 )
                        if ((int16_t)nucDuoJXDHighTbl[auccounter] > nucDuoJXDHighTbl_temp[auccounter])
                        {
                            if (nucLongData < side_1) // 不碰撞
                            {
                                continue;
                            }
                            else
                            {
                                g_duoji_relay1[auccounter] = 1; //幅度向外
                            }
                        }
#endif
                        angle_4 = ((acos((side_1 * side_1 + aucDuoJiDisdance * aucDuoJiDisdance - aucDuoJiLong * aucDuoJiLong) / (2 * side_1 * aucDuoJiDisdance))) * 180) / PI; //判断范围-大三角形-- 本机臂长与中分线的角度
//                            LOG("angle_4= %f\n",angle_4);
                        //正弦定理
                        angle_3 = aucDuoJiDisdance * ((float)sin(angle_1)) / aucMaxLong; //判断范围-小三角形-- 钝角

//                            LOG("angle_3= %f\n",angle_3);
                        angle_3 = 180 - ((asin(angle_3)) * 180) / PI;
//                        LOG("angle_3= %f\n",angle_3);
                        angle_2 = 180 - angle_3 - angle_1 * 180 / PI; //判断范围-小三角形-- 本机臂长与中分线的角度
//                            LOG("angle_2= %f\n",angle_2);
                        if (nucDuojiStatuse == 5 || range0 == 2)
                        {
                            angle_a1 = aucAngleDJ[0] - angle_4;
                            angle_a2 = aucAngleDJ[0] - angle_2;
                        }
                        else
                        {
                            angle_a1 = aucAngleDJ[0] + angle_2;
                            angle_a2 = aucAngleDJ[0] + angle_4;
                        }

                        float aucAngle0 = aucAngle[0];
                        float g_angle_alarm = (float)nucDanJEndLongTbl[auccounter] / 100;
                        float g_angle_warn = (float)nucDanJStartLongTbl[auccounter] / 100;
//                            LOG("aucAngle0[%f],angle_a1[%f],angle_a2[%f]\n",aucAngle0,angle_a1,angle_a2);
                        //  1, a1 < 0
                        //  2, a2 >360
                        angle_a1 = angle_a1 - g_angle_warn;
                        angle_a2 = angle_a2 + g_angle_warn;

                        if (angle_a1 < 0)
                        {
                            if (aucAngle0 < 360 && aucAngle0 > 180)
                            {
                                aucAngle0 = aucAngle0 - 360;
                            }
                        }
                        else if (angle_a2 > 360)
                        {
                            if (aucAngle0 < 180)
                            {
                                aucAngle0 = aucAngle0 + 360;
                            }
                        }


                        //本机 高度 > 多机
                        if ((int16_t)nucDuoJXDHighTbl[auccounter] > nucDuoJXDHighTbl_temp[auccounter])
                        {

//                            LOG("l=%f,s[%f]\n",nucLongData,side_1);
                            float cha = (float)nucDuoJTowerLongTbl_temp[auccounter] / 100;
                            // 幅度满足碰撞条件
                            if (nucLongData + cha > side_1)
                            {
                                float bc = bianchang1(angle_1_a, (float)nucDuoJTowerLongTbl_back[auccounter] / 100, aucDuoJiDisdance);
//                                LOG("bc=%f,ent[%d]-[%d]\n",bc,g_duoji_rsta_warn[auccounter],g_duoji_rsta_alarm[auccounter]);
                                if (nucLongData >= bc && g_duoji_rsta_alarm[auccounter] >= 5) //同一区域
                                {
                                    bc = (aucMaxLong - nucLongData) * sin((PI * (180 - angle_3)) / 180);
//                                    LOG("cha=%f,bc[%f]--%f\n",cha,bc,angle_3);
                                    if (cha > bc) // 幅度报警
                                    {
                                        g_duoji_relay1[auccounter] = 1;

#if 1
                                        angle_a1 = angle_a1 + (g_angle_warn - g_angle_alarm);
                                        angle_a2 = angle_a2 - (g_angle_warn - g_angle_alarm);
                                        if (aucAngle0 >= angle_a1 && aucAngle0 <= angle_a2)
                                        {
//                                            LOG("%f-%f-%f\n",aucAngle0,angle_a1,angle_a2);
                                            if (nucControlFlag > 0 && nucDebugFlag == 0)
                                            {
                                                if (nucDuojiStatuse == 5)
                                                {
                                                    if (angle_2_a > angle_1_a)
                                                    {
                                                        nucSysDuoJiControlFlag = 1;//禁止右转
                                                    }
                                                    else if (angle_2_a < angle_1_a)
                                                    {
                                                        nucSysDuoJiControlFlag = 2;//禁止左转
                                                    }
                                                }
                                                else if (nucDuojiStatuse == 6)
                                                {
                                                    if (angle_2_a > angle_1_a)
                                                    {
                                                        nucSysDuoJiControlFlag = 2;//禁止左转
                                                    }
                                                    else if (angle_2_a < angle_1_a)
                                                    {
                                                        nucSysDuoJiControlFlag = 1;//禁止右转
                                                    }
                                                }
                                                else if (range0 == 1)
                                                {
                                                    nucSysDuoJiControlFlag = 1;//禁止右转
                                                }
                                                else if (range0 == 2)
                                                {
                                                    nucSysDuoJiControlFlag = 2;//禁止左转
                                                }
                                            }
                                        }
#endif
                                        nucDuojiStatuse = 2;//报警
                                    }
                                    else if (cha + 5 > bc) // 幅度预警
                                    {
                                        g_duoji_relay2[auccounter] = 1;
                                        nucSysDuoJiControlFlag = 0;

                                        nucDuojiStatuse = 1;//预警
                                    }
                                    else
                                    {
                                        continue;
                                    }

#if 0
                                    if (g_duoji_relay1[auccounter] > 0)
                                    {
                                        LOG("fudu_ctl=%d\n", g_duoji_rsta_warn[auccounter]);
                                        if (g_duoji_rsta_alarm[auccounter] == 5)
                                        {
                                            if (g_duoji_rsta_warn[auccounter] == 2)
                                            {
                                                nucSysDuoJiControlFlag = 2;
                                            }
                                            else if (g_duoji_rsta_warn[auccounter] == 0)
                                            {
                                                nucSysDuoJiControlFlag = 1;
                                            }
                                        }
                                        else if (g_duoji_rsta_alarm[auccounter] == 6)
                                        {
                                            if (g_duoji_rsta_warn[auccounter] == 1)
                                            {
                                                nucSysDuoJiControlFlag = 1;
                                            }
                                            else if (g_duoji_rsta_warn[auccounter] == 0)
                                            {
                                                nucSysDuoJiControlFlag = 2;
                                            }
                                        }
                                    }
#endif
                                }
                                else if (nucLongData >= bc && aucAngle0 >= angle_a1 && aucAngle0 <= angle_a2)
                                {
                                    angle_a1 = angle_a1 + (g_angle_warn - g_angle_alarm);
                                    angle_a2 = angle_a2 - (g_angle_warn - g_angle_alarm);

                                    if (aucAngle0 >= angle_a1 && aucAngle0 <= angle_a2)
                                    {
                                        if (nucControlFlag > 0 && nucDebugFlag == 0)
                                        {
                                            if (nucDuojiStatuse == 5)
                                            {
                                                if (angle_2_a > angle_1_a)
                                                {
                                                    nucSysDuoJiControlFlag = 1;//禁止右转
                                                }
                                                else if (angle_2_a < angle_1_a)
                                                {
                                                    nucSysDuoJiControlFlag = 2;//禁止左转
                                                }
                                            }
                                            else if (nucDuojiStatuse == 6)
                                            {
                                                if (angle_2_a > angle_1_a)
                                                {
                                                    nucSysDuoJiControlFlag = 2;//禁止左转
                                                }
                                                else if (angle_2_a < angle_1_a)
                                                {
                                                    nucSysDuoJiControlFlag = 1;//禁止右转
                                                }
                                            }
                                            else if (range0 == 1)
                                            {
                                                nucSysDuoJiControlFlag = 1;//禁止右转
                                            }
                                            else if (range0 == 2)
                                            {
                                                nucSysDuoJiControlFlag = 2;//禁止左转
                                            }
                                            if (range0 > 0)
                                            {
//                                        LOG("dj range[%d]\n",range0);
                                            }
                                        }
                                        nucDuojiStatuse = 2;//报警
                                    }
                                    else
                                    {
                                        nucSysDuoJiControlFlag = 0;

                                        nucDuojiStatuse = 1;//预警
                                    }
                                    g_duoji_id = nucDuoJIDTbl[auccounter];
                                }
                            }
                            else
                            {
                                nucSysDuoJiControlFlag = 0;

                                nucDuojiStatuse = 0;
                            }

                            g_duoji_id = nucDuoJIDTbl[auccounter];
                        }
                        else  if (aucAngle0 >= angle_a1 && aucAngle0 <= angle_a2)
                        {
                            angle_a1 = angle_a1 + (g_angle_warn - g_angle_alarm);
                            angle_a2 = angle_a2 - (g_angle_warn - g_angle_alarm);

                            if (aucAngle0 >= angle_a1 && aucAngle0 <= angle_a2)
                            {
                                if (nucControlFlag > 0 && nucDebugFlag == 0)
                                {
                                    if (nucDuojiStatuse == 5)
                                    {
                                        if (angle_2_a > angle_1_a)
                                        {
                                            nucSysDuoJiControlFlag = 1;//禁止右转
                                        }
                                        else if (angle_2_a < angle_1_a)
                                        {
                                            nucSysDuoJiControlFlag = 2;//禁止左转
                                        }
                                    }
                                    else if (nucDuojiStatuse == 6)
                                    {
                                        if (angle_2_a > angle_1_a)
                                        {
                                            nucSysDuoJiControlFlag = 2;//禁止左转
                                        }
                                        else if (angle_2_a < angle_1_a)
                                        {
                                            nucSysDuoJiControlFlag = 1;//禁止右转
                                        }
                                    }
                                    else if (range0 == 1)
                                    {
                                        nucSysDuoJiControlFlag = 1;//禁止右转
                                    }
                                    else if (range0 == 2)
                                    {
                                        nucSysDuoJiControlFlag = 2;//禁止左转
                                    }
                                    if (range0 > 0)
                                    {
//                                        LOG("dj range[%d]\n",range0);
                                    }
                                }
                                nucDuojiStatuse = 2;//报警
                            }
                            else
                            {
                                nucSysDuoJiControlFlag = 0;
                                nucDuojiStatuse = 1;//预警
                            }
                            g_duoji_id = nucDuoJIDTbl[auccounter];
                        }
                        else
                        {
                            nucSysDuoJiControlFlag = 0;
                            nucDuojiStatuse = 0;
                        }

                        g_duoji_state[auccounter] = nucDuojiStatuse; //记录某一个多机 记录报警状态
                        g_duoji_relay[auccounter] = nucSysDuoJiControlFlag; //记录某一个多机 记录继电器状态
//                            LOG("aucAngle0[%f],angle_a1[%f],angle_a2[%f]\n",aucAngle0,angle_a1,angle_a2);
//                            LOG("nucDuojiStatuse[%d],nucSysDuoJiControlFlag[%d]\n",nucDuojiStatuse,nucSysDuoJiControlFlag);
                    }
                }
            }
        }
        else
        {
//            nucSysDuoJiControlFlag = 0;
//            nucDuojiStatuse=0;
        }
    }

    int i;
    for (i = 0; i < 8; i++)
    {
        if (g_duoji_state[i] == 2)
        {
            nucDuojiStatuse = 2;
            break;
        }
    }

    if (nucDuojiStatuse != 2)
    {
        for (i = 0; i < 8; i++)
        {
            if (g_duoji_state[i] == 1)
            {
                nucDuojiStatuse = 1;
                break;
            }
        }
    }


    //多机幅度 向外报警
    for (i = 0; i < 8; i++)
    {
        if (g_duoji_relay1[i] == 1)
        {
            nucLongStatuse1 = 2; //多机幅度向外控制
            break;
        }
    }
    //多机幅度向外 预警
    if (nucLongStatuse1 != 2)
    {
        for (i = 0; i < 8; i++)
        {
            if (g_duoji_relay2[i] == 1)
            {
                nucLongStatuse2 = 1;
                break;
            }
        }
    }

    uint8_t duoji_relay[2] = {0};
    for (i = 0; i < 8; i++)
    {
        if (g_duoji_relay[i] == 1)
        {
            duoji_relay[0] = 1;
        }
        else if (g_duoji_relay[i] == 2)
        {
            duoji_relay[1] = 2;
        }

//        if(g_duoji_relay1[i]==1)
//        {
//            nucLongStatuse1=2; //多机幅度向外控制
//        }
    }
    if (g_alarm_able.duoji)
    {
        nucSysDuoJiControlFlag = duoji_relay[0] + duoji_relay[1];
    }
    else
    {
        nucSysDuoJiControlFlag = 0;

    }
}

//本机后臂防碰撞-多机大臂
void nucDuoJFPZJudge3(void)
{
    if (nucDuojiStatuse > 0)
    {
        return;
    }

    uint8_t auccounter = 0;
    float   aucMaxLong = 0;
    float   aucDuoJiLong = 0;
    float   aucDuoJiDisdance = 0;
    float AngleBuff[2][2];
    float aucAngle[2], aucAngleDJ[2];
    float   a;
    uint8_t flg[2][2];
    uint8_t g_duoji_state[8] = {0}; //记录报警状态
    uint8_t g_duoji_relay[8] = {0}; //记录继电器状态
//    uint16_t long_temp=0;
//////////////////////////////////////////////////////
//多机防碰撞判断,配合 碰撞模型图计算
//////////////////////////////////////////////////////
    aucMaxLong = nucTowerPHLong; //塔机平衡臂长
    aucMaxLong = aucMaxLong / 100; //塔机平衡臂长
    aucAngle[0] = nucAngleData;     //本机当前回转角度

//    LOG("aucMaxLong=%f\n",aucMaxLong);
    if (aucAngle[0] > 180)
    {
        aucAngle[0] -= 180;
    }
    else
    {
        aucAngle[0] += 180;
    }
//    nucDuojiStatuse=0;
    for (auccounter = 0; auccounter < 8; auccounter++)
    {
        if (nucDuoJIDTbl[auccounter] != 0 && nucDuoJTowerLongTbl[auccounter] != 0 && nucDuoJXDDisdanceTbl[auccounter] != 0)
        {
            if (nucDuoJAngleTbl[auccounter] == 0 && nucDuoJLongTbl[auccounter] == 0 && nucDuoJHighTbl[auccounter] == 0)
            {
                continue;
            }
            //多机是动臂吊20250721
            if (nucDanJEndHighTbl[auccounter] == 1)
            {
                //相对距离 > 本机后臂长+幅度误差+多机幅度
                if (nucDuoJXDDisdanceTbl[auccounter] > nucTowerPHLong + nucDuoJTowerLongTbl_temp[auccounter] + nucDuoJLongTbl[auccounter] * 100)
                {
                    continue;
                }
            }
            //相对高度为正同时>高度误差  (本机塔臂高 > 多机塔臂高 )
            else if ((int16_t)nucDuoJXDHighTbl[auccounter] > nucDuoJXDHighTbl_temp[auccounter])
            {
                //区域B内
#if 0
                //距离-多机臂长 > 后臂
                if (nucDuoJXDDisdanceTbl[auccounter] - nucDuoJTowerLongTbl[auccounter] > aucMaxLong * 100)
                {
                    continue;
                }
#else

                //距离 > 多机臂长+ 幅度误差
                if (nucDuoJXDDisdanceTbl[auccounter] > nucDuoJTowerLongTbl[auccounter] + nucDuoJTowerLongTbl_temp[auccounter])
                {
                    continue;
                }
#endif
//                long_temp =   nucDuoJTowerLongTbl_temp[auccounter]; //幅度差
            }
            //(本机塔臂高 < 多机塔臂高 )
            else if (abs((int16_t)nucDuoJXDHighTbl[auccounter]) > nucDuoJXDHighTbl_temp[auccounter])
            {
                //多机钩不在 区域A内
                //距离 - 本机后臂长 > 多机幅度 + 幅度差
                if (nucDuoJXDDisdanceTbl[auccounter] - nucTowerPHLong > nucDuoJLongTbl[auccounter] * 100 + nucDuoJTowerLongTbl_temp[auccounter])
                {
                    continue;
                }
//                long_temp=nucDuoJTowerLongTbl_temp[auccounter]; //幅度差
            }

            aucAngle[1] = nucDuoJAngleTbl[auccounter];          //多机当前回转角度
            aucAngleDJ[0] = nucDuoJXDAngleTbl[auccounter];  //本机当前设置相对角度

            if (aucAngleDJ[0] >= 0 && aucAngleDJ[0] < 180)                 aucAngleDJ[1] = aucAngleDJ[0] + 180; //多机当前相对角度
            else if (aucAngleDJ[0] >= 180 && aucAngleDJ[0] < 360)  aucAngleDJ[1] = aucAngleDJ[0] - 180; //多机当前相对角度

//            if(nucDanJEndHighTbl[auccounter] == 1)//多机是动臂吊
//            {
//                aucDuoJiLong=nucDuoJLongTbl[auccounter]*100+nucDuoJTowerLongTbl_temp[auccounter]; //多机幅度
//                aucDuoJiLong=aucDuoJiLong/100;                  //多机幅度
//            }
//            else
            {
                aucDuoJiLong = nucDuoJTowerLongTbl[auccounter] + nucDuoJTowerLongTbl_temp[auccounter]; //多机臂长+幅度差
                aucDuoJiLong = aucDuoJiLong / 100;                          //多机臂长
            }

            aucDuoJiDisdance = nucDuoJXDDisdanceTbl[auccounter];    //多机相对距离
            aucDuoJiDisdance = aucDuoJiDisdance / 100;                          //多机相对距离
            if (aucDuoJiLong > 0) /*臂长大于0*/
            {
                float a0, a1, a2, a3;
                //本机为参考,相交扇形区域,最大扇形夹角的一半
                a = ((acos((aucMaxLong * aucMaxLong + aucDuoJiDisdance * aucDuoJiDisdance - aucDuoJiLong * aucDuoJiLong) / (2 * aucMaxLong * aucDuoJiDisdance))) * 180) / PI;

                float   a_temp = 0; //(float)DUOJI_ALARM_DATA_1/100;;//增加扇区最大边界,防止交点正好在扇区边界
                AngleBuff[0][0] = ((float)(((uint32_t)((aucAngleDJ[0] + 360 - a - a_temp) * 100)) % 36000)) / 100; //本机交叉起始角度
                AngleBuff[0][1] = ((float)(((uint32_t)((aucAngleDJ[0] + a + a_temp) * 100)) % 36000)) / 100;    //本机交叉结束角度

                if (AngleBuff[0][0] < aucAngleDJ[0] && aucAngleDJ[0] < AngleBuff[0][1])                 //不跨零点
                {
                    a1 = AngleBuff[0][0];
                    a2 = aucAngleDJ[0];
                    a3 = AngleBuff[0][1];

                    a0 = aucAngle[0];
                }
                else if (AngleBuff[0][0] < aucAngleDJ[0] && aucAngleDJ[0] > AngleBuff[0][1])        //结束角度跨零点
                {
                    a1 = AngleBuff[0][0];
                    a2 = aucAngleDJ[0];
                    a3 = AngleBuff[0][1] + 360;

                    if (aucAngle[0] < AngleBuff[0][1])     a0 = aucAngle[0] + 360;
                    else                                                            a0 = aucAngle[0];
                }
                else if (AngleBuff[0][0] > aucAngleDJ[0] && aucAngleDJ[0] < AngleBuff[0][1])        //相对角度和结束角度都跨零点
                {
                    a1 = AngleBuff[0][0];
                    a2 = aucAngleDJ[0] + 360;
                    a3 = AngleBuff[0][1] + 360;

                    if (aucAngle[0] < AngleBuff[0][1])     a0 = aucAngle[0] + 360;
                    else                                                            a0 = aucAngle[0];
                }
                else
                {
                    a1 = 0;
                    a2 = 1;
                    a3 = 2;
                    a0 = 3;
                }

                if (a0 < a1 || a0 > a3)     //本机未进入交叉区域
                {
                    flg[0][0] = 0;
                    flg[0][1] = 0;
                }
                else                                    //本机进入交叉区域
                {
                    if (a0 < a2)                    //进入前半个扇区
                    {
                        flg[0][0] = 1;
                        flg[0][1] = 0;
                    }
                    else                                //进入后半个扇区
                    {
                        flg[0][0] = 0;
                        flg[0][1] = 1;
                    }

                }
                a = ((acos((aucDuoJiLong * aucDuoJiLong + aucDuoJiDisdance * aucDuoJiDisdance - aucMaxLong * aucMaxLong) / (2 * aucDuoJiLong * aucDuoJiDisdance))) * 180) / PI;

                AngleBuff[1][0] = ((float)(((uint32_t)((aucAngleDJ[1] + 360 - a - a_temp) * 100)) % 36000)) / 100; //多机交叉起始角度
                AngleBuff[1][1] = ((float)(((uint32_t)((aucAngleDJ[1] + a + a_temp) * 100)) % 36000)) / 100;    //多机交叉结束角度

                if (AngleBuff[1][0] < aucAngleDJ[1] && aucAngleDJ[1] < AngleBuff[1][1])                 //不跨零点
                {
                    a1 = AngleBuff[1][0];
                    a2 = aucAngleDJ[1];
                    a3 = AngleBuff[1][1];

                    a0 = aucAngle[1];
                }
                else if (AngleBuff[1][0] < aucAngleDJ[1] && aucAngleDJ[1] > AngleBuff[1][1])        //结束角度跨零点
                {
                    a1 = AngleBuff[1][0];
                    a2 = aucAngleDJ[1];
                    a3 = AngleBuff[1][1] + 360;

                    if (aucAngle[1] < AngleBuff[1][1])     a0 = aucAngle[1] + 360;
                    else                                                            a0 = aucAngle[1];
                }
                else if (AngleBuff[1][0] > aucAngleDJ[1] && aucAngleDJ[1] < AngleBuff[1][1])        //相对角度和结束角度都跨零点
                {
                    a1 = AngleBuff[1][0];
                    a2 = aucAngleDJ[1] + 360;
                    a3 = AngleBuff[1][1] + 360;

                    if (aucAngle[1] < AngleBuff[1][1])     a0 = aucAngle[1] + 360;
                    else                                                            a0 = aucAngle[1];
                }
                else
                {
                    a1 = 0;
                    a2 = 1;
                    a3 = 2;
                    a0 = 3;
                }

                if (a0 < a1 || a0 > a3)     //多机未进入交叉区域
                {
                    flg[1][0] = 0;
                    flg[1][1] = 0;

                }
                else                                    //多机进入交叉区域
                {
                    if (a0 < a2)                    //进入前半个扇区
                    {
                        flg[1][0] = 1;
                        flg[1][1] = 0;
                    }
                    else                                //进入后半个扇区
                    {
                        flg[1][0] = 0;
                        flg[1][1] = 1;
                    }
                }
                uint8_t range0 = 0; //非交半区提示
                if ((flg[0][0] == 0 && flg[0][1] == 0)  //本机不在区域，不报警
                        || (flg[1][0] == 0 && flg[1][1] == 0)) //多机不在区域，不报警
                {
                    nucDuojiStatuse = 0;
                    nucSysDuoJiControlFlag = 0;
                }
                else
                {
                    if (flg[0][0] == 1 && flg[0][1] == 0)           //本机在前半扇区
                    {
                        if (flg[1][0] == 1 && flg[1][1] == 0)       //多机在前半扇区，不在半个扇区，预警
                        {
//                            auccounter=8;
                            range0 = 1; //提示范围,从下半区进入
                        }
                        else if (flg[1][0] == 0 && flg[1][1] == 1) //多机在后半扇区，在半个扇区，报警
                        {
                            nucDuojiStatuse = 5; //上半区相交
//                            auccounter=8;
                        }
                    }
                    else if (flg[0][0] == 0 && flg[0][1] == 1)  //本机在后半扇区
                    {
                        if (flg[1][0] == 1 && flg[1][1] == 0)       //多机在前半扇区，在半个扇区，报警
                        {
                            nucDuojiStatuse = 6; //下半区相交
//                            auccounter=8;
                        }
                        else if (flg[1][0] == 0 && flg[1][1] == 1) //多机在后半扇区，不在半个扇区，预警
                        {
//                            auccounter=8;
                            range0 = 2; //提示范围,从上半区进入
                        }
                    }

                    if (nucDuojiStatuse >= 5 || range0 > 0)
                    {
                        float side_1;
                        float angle_1, angle_2, angle_3, angle_4, angle_a1, angle_a2, angle_1_a, angle_2_a;
#if 0
                        angle_1 = fabs(aucAngleDJ[1] - aucAngle[1]); //多机对应中分线的角度
                        angle_1_a = angle_1;
                        angle_2_a = fabs(aucAngleDJ[0] - aucAngle[0]); //本机臂长 与 中分线的角度
#else
                        angle_1 = fabs(aucAngleDJ[1] - aucAngle[1]); //多机对应中分线的角度
                        if (angle_1 > 180)
                        {
                            angle_1 = 360 - angle_1;
                        }
                        angle_1_a = angle_1;
                        angle_2_a = fabs(aucAngleDJ[0] - aucAngle[0]); //本机臂长 与 中分线的角度
                        if (angle_2_a > 180)
                        {
                            angle_2_a = 360 - angle_2_a;
                        }
#endif

//                        LOG("aucAngle[%f],aucAngleDJ= %f\n",aucAngle[1],aucAngleDJ[1]);
//                        LOG("angle compare [%f]:[%f]\n",angle_2_a,angle_1_a);
                        angle_1 = (PI * angle_1) / 180;
                        //余弦定理
                        side_1 = sqrt(aucDuoJiLong * aucDuoJiLong + aucDuoJiDisdance * aucDuoJiDisdance - 2 * aucDuoJiLong * aucDuoJiDisdance * cos(angle_1)); //判断范围-大三角形-- 本机与多机相交点的臂长
//                            LOG("side_1= %f\n",side_1);
                        angle_4 = ((acos((side_1 * side_1 + aucDuoJiDisdance * aucDuoJiDisdance - aucDuoJiLong * aucDuoJiLong) / (2 * side_1 * aucDuoJiDisdance))) * 180) / PI; //判断范围-大三角形-- 本机臂长与中分线的角度
//                            LOG("angle_4= %f\n",angle_4);
                        //正弦定理
                        angle_3 = aucDuoJiDisdance * ((float)sin(angle_1)) / aucMaxLong; //判断范围-小三角形-- 钝角

//                            LOG("angle_3= %f\n",angle_3);
                        angle_3 = 180 - ((asin(angle_3)) * 180) / PI;
//                        LOG("angle_3= %f\n",angle_3);
                        angle_2 = 180 - angle_3 - angle_1 * 180 / PI; //判断范围-小三角形-- 本机臂长与中分线的角度
//                            LOG("angle_2= %f\n",angle_2);
                        if (nucDuojiStatuse == 5 || range0 == 2)
                        {
                            angle_a1 = aucAngleDJ[0] - angle_4;
                            angle_a2 = aucAngleDJ[0] - angle_2;
                        }
                        else
                        {
                            angle_a1 = aucAngleDJ[0] + angle_2;
                            angle_a2 = aucAngleDJ[0] + angle_4;
                        }

                        float aucAngle0 = aucAngle[0];
                        float g_angle_alarm = (float)nucDanJEndLongTbl[auccounter] / 100;
                        float g_angle_warn = (float)nucDanJStartLongTbl[auccounter] / 100;
//                            LOG("aucAngle0[%f],angle_a1[%f],angle_a2[%f]\n",aucAngle0,angle_a1,angle_a2);
                        //  1, a1 < 0
                        //  2, a2 >360
                        angle_a1 = angle_a1 - g_angle_warn;
                        angle_a2 = angle_a2 + g_angle_warn;

                        if (angle_a1 < 0)
                        {
                            if (aucAngle0 < 360 && aucAngle0 > 180)
                            {
                                aucAngle0 = aucAngle0 - 360;
                            }
                        }
                        else if (angle_a2 > 360)
                        {
                            if (aucAngle0 < 180)
                            {
                                aucAngle0 = aucAngle0 + 360;
                            }
                        }
                        if (aucAngle0 >= angle_a1 && aucAngle0 <= angle_a2)
                        {
                            angle_a1 = angle_a1 + (g_angle_warn - g_angle_alarm);
                            angle_a2 = angle_a2 - (g_angle_warn - g_angle_alarm);

                            if (aucAngle0 >= angle_a1 && aucAngle0 <= angle_a2)
                            {
                                if (nucControlFlag > 0 && nucDebugFlag == 0)
                                {
                                    if (nucDuojiStatuse == 5)
                                    {
                                        if (angle_2_a > angle_1_a)
                                        {
                                            nucSysDuoJiControlFlag = 1;//禁止右转
                                        }
                                        else if (angle_2_a < angle_1_a)
                                        {
                                            nucSysDuoJiControlFlag = 2;//禁止左转
                                        }
                                    }
                                    else if (nucDuojiStatuse == 6)
                                    {
                                        if (angle_2_a > angle_1_a)
                                        {
                                            nucSysDuoJiControlFlag = 2;//禁止左转
                                        }
                                        else if (angle_2_a < angle_1_a)
                                        {
                                            nucSysDuoJiControlFlag = 1;//禁止右转
                                        }
                                    }
                                    else if (range0 == 1)
                                    {
                                        nucSysDuoJiControlFlag = 1;//禁止右转
                                    }
                                    else if (range0 == 2)
                                    {
                                        nucSysDuoJiControlFlag = 2;//禁止左转
                                    }
                                    if (range0 > 0)
                                    {
//                                        LOG("dj range[%d]\n",range0);
                                    }
                                }
                                nucDuojiStatuse = 2;//报警
                            }
                            else
                            {
                                nucSysDuoJiControlFlag = 0;
                                nucDuojiStatuse = 1;//预警

                            }
                            g_duoji_id = nucDuoJIDTbl[auccounter];
                        }
                        else
                        {
                            nucSysDuoJiControlFlag = 0;
                            nucDuojiStatuse = 0;
                        }

                        g_duoji_state[auccounter] = nucDuojiStatuse; //记录某一个多机 记录报警状态
                        g_duoji_relay[auccounter] = nucSysDuoJiControlFlag; //记录某一个多机 记录继电器状态
//                            LOG("aucAngle0[%f],angle_a1[%f],angle_a2[%f]\n",aucAngle0,angle_a1,angle_a2);
//                            LOG("nucDuojiStatuse[%d],nucSysDuoJiControlFlag[%d]\n",nucDuojiStatuse,nucSysDuoJiControlFlag);
                    }
                }
            }
        }
        else
        {
//            nucSysDuoJiControlFlag = 0;
//            nucDuojiStatuse=0;
        }
    }

    int i;
    for (i = 0; i < 8; i++)
    {
        if (g_duoji_state[i] == 2)
        {
            nucDuojiStatuse = 2;
            break;
        }
    }

    if (nucDuojiStatuse != 2)
    {
        for (i = 0; i < 8; i++)
        {
            if (g_duoji_state[i] == 1)
            {
                nucDuojiStatuse = 1;
                break;
            }
        }
    }
    uint8_t duoji_relay[2] = {0};
    for (i = 0; i < 8; i++)
    {
        if (g_duoji_relay[i] == 1)
        {
            duoji_relay[0] = 1;
        }
        else if (g_duoji_relay[i] == 2)
        {
            duoji_relay[1] = 2;
        }
    }
    if (g_alarm_able.duoji)
    {
        nucSysDuoJiControlFlag = duoji_relay[0] + duoji_relay[1];
    }
    else
    {
        nucSysDuoJiControlFlag = 0;

    }
}


static int32_t      g_circle = 0; //圈数
static uint32_t     g_sum1 = 0; //计数
static uint32_t     g_sum2 = 0; //计数
static uint32_t   position = 0; //象限位置


static int32_t      g_signle_last = 0; //旧值
static int32_t      g_signle_temp;//差值

//#define  DIRECTION_NUM 5
void bsp_rotation_twist_get(void)
{
    if (g_sensor_able.rotation == 0)
    {
        return;
    }
    if (position == 0)
    {
        //TODO g_new_bef.twist_last 读取
        position = 1;
        g_circle = g_new_bef.twist_last / 3600;
    }
    g_signle_temp = (int32_t)nucAngleSignle - g_signle_last;

    if (abs(g_signle_temp) < 2)
    {
        return;
    }

//    LOG("g_signle_temp[%d]\n",g_signle_temp);

    if (g_signle_temp > 1800) //逆时针 && 过0点
    {
        g_circle--;
    }
    else  if (g_signle_temp < - 1800) //顺时针 && 过0点
    {
        g_circle++;
    }

    g_new_bef.twist = nucAngleSignle + g_circle * 3600 - g_new_bef.twist_cal;
    if (g_new_bef.twist > 6000)
    {
        g_new_bef.twist = 6000;
    }
    else  if (g_new_bef.twist < -6000)
    {
        g_new_bef.twist = -6000;
    }

#if 1
    if (g_new_bef.twist > 0)
    {
        if (g_new_bef.twist > g_new_bef.twist_last)
        {
            g_direction_new = 1;
        }
        else if (g_new_bef.twist < g_new_bef.twist_last)
        {
            g_direction_new = 2;
        }
        else
        {
            g_direction_new = 1;
        }
    }
    else
    {
        if (g_new_bef.twist < g_new_bef.twist_last)
        {
            g_direction_new = 2;
        }
        else if (g_new_bef.twist > g_new_bef.twist_last)
        {
            g_direction_new = 1;
        }
        else
        {
            g_direction_new = 0;
        }
    }
#endif

    g_new_bef.twist_last = g_new_bef.twist;
    g_signle_last = nucAngleSignle;

//    LOG("g_sum1 = %d,g_sum2= %d\n",g_sum1,g_sum2);
//    LOG("g_direction[(%d)],g_circle=%d\n",g_direction_new,g_circle);
//    LOG("nucAngleSignle = %d, g_new_bef.twist = %d\n\n",nucAngleSignle, g_new_bef.twist);
}

void twist_cal(int argc, char**argv)
{
    g_circle = 0;
    g_direction_new = 0;
    g_sum1 = g_sum2 = 0;
    g_signle_last = nucAngleSignle;
    g_new_bef.twist_cal = nucAngleSignle;
    LOG("g_new_bef.twist_cal=%d\n", g_new_bef.twist_cal);
}
void APP_USER_Twist_Cal_One(int32_t twist)
{
    if (twist > 5400 || twist < -5400)
    {
        return;
    }

    if (twist == 0)
    {
        g_circle = 0;
        g_direction_new = 0;
        g_sum1 = g_sum2 = 0;
        g_signle_last = nucAngleSignle;
        g_new_bef.twist_cal = nucAngleSignle;
    }
    else
    {
        g_circle = twist / 3600;
        if (twist % 3600 > 0)
        {
            g_circle += 1;
        }
        else if (twist % 3600 < 0)
        {
            g_circle -= 1;
        }

        g_signle_last = nucAngleSignle;
        g_new_bef.twist_cal = nucAngleSignle + g_circle * 3600 - twist;
        g_new_bef.twist = twist;
        LOG("twist_cal=%d\n", g_new_bef.twist_cal);
    }
}
void APP_USER_Twist_Cal(int32_t twist)
{
    g_circle = 0;
    g_direction_new = 0;
    g_sum1 = g_sum2 = 0;
    g_signle_last = twist;
    g_new_bef.twist_cal = twist;
}

#define LOG_FORM  "%d,"
#define LOG_FORM_END  "%d\r\n"
#if 0
static void SYSCLKConfig_STOP(void)
{
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    uint32_t pFLatency = 0;

    /* 启用电源控制时钟 */
    __HAL_RCC_PWR_CLK_ENABLE();

    /* 根据内部 RCC 寄存器获取振荡器配置 */
    HAL_RCC_GetOscConfig(&RCC_OscInitStruct);

    /* 从停止模式唤醒后重新配置系统时钟: 启用 HSE 和 PLL */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        while (1)
        {
            ;
        }
    }
    /* 根据内部 RCC 寄存器获取时钟配置 */
    HAL_RCC_GetClockConfig(&RCC_ClkInitStruct, &pFLatency);
    /* 选择 PLL 作为系统时钟源, 并配置 HCLK、PCLK1 和 PCLK2 时钟分频系数 */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, pFLatency) != HAL_OK)
    {
        while (1)
        {
            ;
        }
    }
}
#endif
#if 0
static int32_t STOP_IsReady(void)
{
    int32_t i32Ret = LL_OK;
    uint8_t tmp1;
    uint8_t tmp2;
    uint8_t tmp3;

    tmp1 = (uint8_t)((READ_REG32(CM_EFM->FSR) & (EFM_FSR_RDY0 | EFM_FSR_RDY1)) == (EFM_FSR_RDY0 | EFM_FSR_RDY1));

    tmp2 = (uint8_t)((READ_REG32(CM_DMA1->CHSTAT) & DMA_CHSTAT_DMAACT) == 0x00U);
    tmp3 = (uint8_t)((READ_REG32(CM_DMA2->CHSTAT) & DMA_CHSTAT_DMAACT) == 0x00U);

    if (0U == (tmp1 & tmp2 & tmp3))
    {
        i32Ret = LL_ERR_NOT_RDY;
    }
    return i32Ret;
}
#endif
static uint32_t g_low_power_sum = 0;
uint32_t g_low_power_state = 0;
static uint32_t g_low_power_sleep_sum = 0;
extern void SystemClock_Config(void);
uint8_t g_send_dc_real = 0;
//低功耗模式
int APP_USER_Work_Mode(void)
{
    int ret = 0;
#if 1
    if (g_bsp_adc.vol_dc_in < 800) // <阈值,未插电
    {
        if (++g_low_power_sum < 600 && DTU_SEND == 0) //稳定60s 没电源输入 或 实时数据已发送完成
        {
            g_bsp_adc.vol_dc_status = 0;
            return 0;
        }
        g_low_power_sleep_sum = 0;//定时唤醒时间 赋 0
        g_low_power_state = 1; //关闭外设flag
    }
    else //供电正常
    {
        g_send_dc_real = 1;
        g_bsp_adc.vol_dc_status = 1;
        g_low_power_sum = 0;
//        if(g_low_power_state == 2) //恢复正常模式
//        {
//            BSP_CONFIG_System_Reset();//重启
//            g_low_power_state = 0;
//        }
        return 0;
    }

    if (g_send_dc_real > 0)
    {
        g_send_dc_real = 0;
        APP_DTU_Send_DTURealTimeRecord();//实时数据
        BSP_DELAY_MS(500);
    }
    if (g_low_power_state == 1) //关闭外设
    {
        BSP_POWER_Off(); //外设电源关
        g_low_power_state = 2;
        LOG("BSP_POWER_Off\n");
    }
    if (g_low_power_state == 2) //进入低功耗
    {
        SysTick_Suspend();
        BSP_TIM_Tmr2Stop();
        BSP_TIM_Tmr0Start();
        while (1)
        {
            BSP_IWDG_Refresh();
            if (BSP_ADC_Callback() > 0) //检测电压
            {
//                LOG("RTC_Stamp[%d]\n",BSP_RTC_Stamp_Get());
                LOG("Restart timing [%d/%d]\n", g_low_power_sleep_sum, g_stop_delay * 6 - 2);
                if (++g_low_power_sleep_sum > (g_stop_delay * 6 - 2)  || g_bsp_adc.vol_dc_in >= 1000) //计时,然后发送实时数据
                {
                    g_low_power_state = 0;
                    g_low_power_sum = 0;
                    LOG("mcu wake up\n");

                    BSP_CONFIG_System_Reset(); //重启
                    return 0;
                }

#if 1
//                LOG("mcu sleep\n");
                PWC_SLEEP_Enter();
#else
                if (LL_OK == STOP_IsReady())
                {
                    LOG("mcu stop\n");
                    PWC_STOP_Enter(PWC_STOP_WFI);
                }
#endif

                ret = 1;
            }
        }
    }
#endif
    return ret;
}


void APP_USER_Realy_Status(void)
{
    g_relay_ctl.bit1 = LONGOUT;
    g_relay_ctl.bit2 = LONGIN;
    g_relay_ctl.bit3 = HIGHUP;
    g_relay_ctl.bit4 = HIGHDOWN;
    g_relay_ctl.bit5 = ANGLELEFT;
    g_relay_ctl.bit6 = ANGLERIGHT;
}

void APP_MAIN_Callback_Heartbeat_Board(void)//2秒运行1次
{
    APP_WIRELESS_Read(g_charg_board.type, 0x1000, 0); //向充电板发心跳包
    if (++g_charg_board.timeout_num < 150) //2s*150=300s=5min
    {
        g_charg_board.connection_status = 1; //通信成功，充电板有回复了会将该标志位置零
    }
    else //5分钟超时失败
    {
        g_charg_board.timeout_num = 150;
        g_charg_board.connection_status = 0; //通信失败
    }
    if (g_charg_board.work_mode_ipc == 1)
    {
        if (g_board_work_freq == 0) //不工作，30分钟进入低功耗模式
        {
            if (g_charg_board.work_mode == 0)
            {
                g_charg_board.work_mode = 1;
                APP_WIRELESS_Write(g_charg_board.type, 0x1004, &g_charg_board.work_mode, 2);
            }
        }
        else
        {
            if (g_charg_board.work_mode == 1)
            {
                g_charg_board.work_mode = 0;
                APP_WIRELESS_Write(g_charg_board.type, 0x1004, &g_charg_board.work_mode, 2);
            }
        }
    }
}

static int APP_USER_Event_Handle(void)
{
    int ret = 0;
    if (g_app_config.app_config_sys.dtu_status > 0)
    {
        if (g_enent.BUFF_u32 != g_enent_last.BUFF_u32)
        {
            for (int i = 0; i < 32; i++)
            {
                if ((g_enent.BUFF_u32  & (1 << i)) > (g_enent_last.BUFF_u32 & (1 << i))) //找使能位
                {
                    g_enent_last.BUFF_u32 |= (1 << i); //按位更新

                    APP_DTU_Send_DTUEventRecord((1 << i));
                    LOG("send event  bit[%d]\n", i);
                    break;
                }
                else if ((g_enent.BUFF_u32 & (1 << i)) < (g_enent_last.BUFF_u32 & (1 << i))) //找使能位
                {
                    g_enent_last.BUFF_u32 &= ~(1 << i); //按位更新
                }
            }
        }
    }

    return ret;
}

uint8_t g_page_dis_status = 0;
//uint32_t g_page_dis_num = 100;
uint32_t g_page_read_status = 0;
uint8_t g_show_face = 0;
uint8_t g_face_event = 0;
void FACE_DWIN_Show(void)
{
    if (g_new_bef.face_enable > 0 && g_show_face == 0)
    {
        g_show_face = 1;
        DWIN_TX(0x2350, 0, 0);
        DWIN_TX(0x2330, 1, 0); //未识别
        DWIN_TX(0x2360, 1, 0);
    }
}

void FACE_EVENT_Handle(void)
{
    if (g_face_event > 0  && g_app_config.app_config_sys.dtu_status > 0)
    {
        if (g_show_face > 0)
        {
            nucSendDTUFaceEvent(1);
        }
        else
        {
            nucSendDTUFaceEvent(2);
        }
        g_face_event = 0;
    }
}
//人脸动画
void APP_DWIN_SUB_Face(void)
{
#if 0
//            char g_test[4]="2342";
//            DWIN_TX(0x5900,2,0);//人脸成功图片
//            DWIN_TX_IP(0x5920,g_test,4);//身份证后4位
//            DWIN_TX(0x5940,1,0);//身份证号 不遮挡
//            DWIN_TX(0x5910,0,0);//人脸成功图片补充 显示

//    DWIN_TX(0x5900,1,0);//人脸未成功
//    DWIN_TX(0x5940,0,0);//身份证号 不遮挡
//    DWIN_TX(0x5910,1,0);//人脸成功图片补充 不显示

    DWIN_TX(0x5900, 0, 0); //人脸不显示
    DWIN_TX(0x5940, 0, 0); //身份证号 遮挡
    DWIN_TX(0x5910, 1, 0); //身份证号 遮挡
#endif
#if 1
    if (g_new_bef.face_enable > 0)
    {
        if (g_show_face > 0)
        {
            DWIN_TX(0x5900, 2, 0); //人脸成功图片
//            DWIN_TX_IP(0x5920,g_id_buf+14,4);//身份证后4位
            DWIN_TX(0x5940, 1, 0); //身份证号 不遮挡
            DWIN_TX(0x5910, 0, 0); //人脸成功图片补充 显示

            nucFaceFlag = 1;
        }
        else
        {
            DWIN_TX(0x5900, 1, 0); //人脸未成功
            DWIN_TX(0x5940, 0, 0); //身份证号 遮挡
            DWIN_TX(0x5910, 1, 0); //人脸成功图片补充 不显示

            nucFaceFlag = 0;
        }
    }
    else
    {
        DWIN_TX(0x5900, 0, 0); //人脸不显示
        DWIN_TX(0x5940, 0, 0); //身份证号 遮挡
        DWIN_TX(0x5910, 1, 0); //身份证号 遮挡
    }
#endif
}
//数据 变大->预警->报警
void APP_DWIN_SUB_Bar1(uint16_t addr, float value, float warn, float alarm)
{
    float x;
    uint64_t y;

    if (value < warn)
    {
        x = 17 / warn;
        y = value * x;
        if (y == 0)
        {
            y = 1;
        }
        if (y > 16)
        {
            y = 16;
        }
    }
    else if (value < alarm)
    {
        x = 2 / (fabs(alarm - warn));
        y = 17 + fabs(value - warn) * x;
        if (y > 18)
        {
            y = 18;
        }
    }
    else
    {
        x = 19 / alarm;
        y = value * x;
        if (y > 20)
        {
            y = 20;
        }
    }

    DWIN_TX(addr, y, 0);
//      if(addr==0xA001 ||addr==0xA002)
//    LOG("v-y[%f-%lld],a-w[%f-%f]\n",value,y,alarm,warn);
}


//报警<-预警<-变小 数据 变大->预警->报警
void APP_DWIN_SUB_Bar2(uint16_t addr, float value, float warn, float alarm, float warn1, float alarm1)
{
    float x;
    uint64_t y;
    if (value > warn) //数据变大
    {
        if (value < warn1)
        {
            x = 13 / (fabs(warn1 - warn));
            y = 5 + fabs(value - warn) * x;
            if (y > 16)
            {
                y = 16;
            }
        }
        else if (value < alarm1)
        {
            x = 2 / (fabs(alarm1 - warn1));
            y = 17 + fabs(value - warn1) * x;
            if (y > 18)
            {
                y = 18;
            }
        }
        else
        {
            x = 19 / alarm1;
            y = value * x;
            if (y > 20)
            {
                y = 20;
            }
        }
//        LOG("1111v-y[%f-%lld],a-w[%f-%f],w1-a1[%f-%f]\n",value,y,alarm,warn,warn1,alarm1);
    }
    else
    {
        if (value <= alarm)
        {
            if (alarm > 0)
            {
                x = 2 / alarm;
            }
            else
            {
                x = 1;
            }
            y = value * x;
            if (y == 0)
            {
                y = 1;
            }
            if (y > 2)
            {
                y = 2;
            }
        }
        else if (value <= warn)
        {
            x = 2 / (fabs(warn - alarm));
            y = 3 + fabs(value - alarm) * x;
            if (y > 4)
            {
                y = 4;
            }
        }
//        LOGT("2222v-y[%f-%lld],a-w[%f-%f],w1-a1[%f-%f]\n",value,y,alarm,warn,warn1,alarm1);
    }
    DWIN_TX(addr, y, 0);
}
void dwin_event_handle(void)
{
    IEEE754_B alarm = {0};
    alarm.u8_buf[3] = nucAlarmTbl[3];
    alarm.u8_buf[2] = nucAlarmTbl[2];
    alarm.u8_buf[1] = nucAlarmTbl[1];
    alarm.u8_buf[0] = nucAlarmTbl[0];

//    uint16_t *serson = (uint16_t*)&g_sensor_state; //传感器状态
    //0正常 1预警 2报警 3异常 4无
    if (g_sensor_able.weight > 0) //重量
    {
        if (g_sensor_state.weight == 1) //重量无传感器
        {
            DWIN_TX(0x3500, 3, 0);
            DWIN_TX(0xA250, 0, 0);
            DWIN_TX(0xA150, 0, 0);
        }
        else
        {
            if ((alarm.u32 & 0x08000000) == 0x08000000) //重量预警
            {
                DWIN_TX(0x3500, 1, 0);
                DWIN_TX(0xA250, 1, 0);
                DWIN_TX(0xA150, 0, 0);
            }
            else if ((alarm.u32 & 0x02000000) == 0x02000000) //重量报警
            {
                DWIN_TX(0x3500, 2, 0);
                DWIN_TX(0xA250, 0, 0);
                DWIN_TX(0xA150, 1, 0);
            }
            else
            {
                DWIN_TX(0x3500, 0, 0); //重量正常
                DWIN_TX(0xA250, 0, 0);
                DWIN_TX(0xA150, 0, 0);
            }
        }
    }
    else
    {
        DWIN_TX(0x3500, 4, 0); //重量无
        DWIN_TX(0xA250, 0, 0);
        DWIN_TX(0xA150, 0, 0);
    }


    if (g_sensor_able.angle > 0) //倾角
    {
        if (g_sensor_state.angle == 1) //无传感器
        {
            DWIN_TX(0x3502, 3, 0);
            DWIN_TX(0x3504, 3, 0);
            DWIN_TX(0xA200, 0, 0);
            DWIN_TX(0xA100, 0, 0);
            DWIN_TX(0xA210, 0, 0);
            DWIN_TX(0xA110, 0, 0);
        }
        else
        {
            if (nucDispAngleXData >= ((float)(QJ_ALARM_DATA / 100)))
            {
                DWIN_TX(0x3502, 2, 0);
                DWIN_TX(0xA200, 0, 0);
                DWIN_TX(0xA100, 1, 0);
            }
            else if (nucDispAngleXData >= ((float)(QJ_ALARM_DATA_1 / 100)))
            {
                DWIN_TX(0x3502, 1, 0);
                DWIN_TX(0xA200, 1, 0);
                DWIN_TX(0xA100, 0, 0);
            }
            else
            {
                DWIN_TX(0x3502, 0, 0); //正常
                DWIN_TX(0xA200, 0, 0);
                DWIN_TX(0xA100, 0, 0);
            }

            if (nucDispAngleYData >= ((float)(QJ_ALARM_DATA / 100)))
            {
                DWIN_TX(0x3504, 2, 0);
                DWIN_TX(0xA210, 0, 0);
                DWIN_TX(0xA110, 1, 0);
            }
            else if (nucDispAngleYData >= ((float)(QJ_ALARM_DATA_1 / 100)))
            {
                DWIN_TX(0x3504, 1, 0);
                DWIN_TX(0xA210, 1, 0);
                DWIN_TX(0xA110, 0, 0);
            }
            else
            {
                DWIN_TX(0x3504, 0, 0); //正常
                DWIN_TX(0xA210, 0, 0);
                DWIN_TX(0xA110, 0, 0);
            }
        }
    }
    else
    {
        DWIN_TX(0x3502, 4, 0); //无
        DWIN_TX(0x3504, 4, 0); //无
        DWIN_TX(0xA200, 0, 0);
        DWIN_TX(0xA100, 0, 0);
        DWIN_TX(0xA210, 0, 0);
        DWIN_TX(0xA110, 0, 0);
    }

    if (g_sensor_able.height > 0) //高度
    {
        if (g_sensor_state.height == 1) //无传感器
        {
            DWIN_TX(0x3506, 3, 0);
            DWIN_TX(0xA220, 0, 0);
            DWIN_TX(0xA120, 0, 0);
        }
        else
        {
            if ((alarm.u32 & 0x00040000) == 0x00040000) //预警
            {
                DWIN_TX(0x3506, 1, 0);
                DWIN_TX(0xA220, 1, 0);
                DWIN_TX(0xA120, 0, 0);
            }
            else if ((alarm.u32 & 0x00000400) == 0x00000400) //报警
            {
                DWIN_TX(0x3506, 2, 0);
                DWIN_TX(0xA220, 0, 0);
                DWIN_TX(0xA120, 1, 0);
            }
            else
            {
                DWIN_TX(0x3506, 0, 0); //正常
                DWIN_TX(0xA220, 0, 0);
                DWIN_TX(0xA120, 0, 0);
            }
        }
    }
    else
    {
        DWIN_TX(0x3506, 4, 0); //无
        DWIN_TX(0xA220, 0, 0);
        DWIN_TX(0xA120, 0, 0);
    }

    if (g_sensor_able.range > 0) //幅度
    {
        if (g_sensor_state.range == 1) //无传感器
        {
            DWIN_TX(0x3508, 3, 0);
            DWIN_TX(0xA230, 0, 0);
            DWIN_TX(0xA130, 0, 0);
        }
        else
        {
            if ((alarm.u32 & 0x00010000) == 0x00010000 || (alarm.u32 & 0x00020000) == 0x00020000) //预警
            {
                DWIN_TX(0x3508, 1, 0);
                DWIN_TX(0xA230, 1, 0);
                DWIN_TX(0xA130, 0, 0);
            }
            else if ((alarm.u32 & 0x00000200) == 0x00000200 || (alarm.u32 & 0x00000100) == 0x00000100) //报警
            {
                DWIN_TX(0x3508, 2, 0);
                DWIN_TX(0xA230, 0, 0);
                DWIN_TX(0xA130, 1, 0);
            }
            else
            {
                DWIN_TX(0x3508, 0, 0); //正常
                DWIN_TX(0xA230, 0, 0);
                DWIN_TX(0xA130, 0, 0);
            }
        }
    }
    else
    {
        DWIN_TX(0x3508, 4, 0); //无
        DWIN_TX(0xA230, 0, 0);
        DWIN_TX(0xA130, 0, 0);
    }

    if (g_sensor_able.wind_speed > 0) //风速
    {
        if (g_sensor_state.wind_speed == 1) //无传感器
        {
            DWIN_TX(0x350A, 3, 0);
            DWIN_TX(0xA240, 0, 0);
            DWIN_TX(0xA140, 0, 0);
        }
        else
        {
            if ((alarm.u32 & 0x00000040) == 0x00000040) //预警
            {
                DWIN_TX(0x350A, 1, 0);
                DWIN_TX(0xA240, 1, 0);
                DWIN_TX(0xA140, 0, 0);
            }
            else if ((alarm.u32 & 0x00000020) == 0x00000020) //报警
            {
                DWIN_TX(0x350A, 2, 0);
                DWIN_TX(0xA240, 0, 0);
                DWIN_TX(0xA140, 1, 0);
            }
            else
            {
                DWIN_TX(0x350A, 0, 0); //正常
                DWIN_TX(0xA240, 0, 0);
                DWIN_TX(0xA140, 0, 0);
            }
        }
    }
    else
    {
        DWIN_TX(0x350A, 4, 0); //无
        DWIN_TX(0xA240, 0, 0);
        DWIN_TX(0xA140, 0, 0);
    }

}

#if USE_DWIN
uint32_t g_set_ewr = 0;
extern uint32_t g_pin;
extern uint32_t g_pin_pw;
//屏幕处理
void DWIN_HANDLE(void)
{
//    if(++DWIN_DISP_FLG>=5)
    {
        DWIN_DISP_FLG = 0;
        uint8_t i;
        uint16_t buff;
        double buff_double, sf, P1 = 60, P2 = 80;


        if (DWIN_PAGE != 5 && g_page_dis_status == 0)
        {
            DWIN_TX(0x2310, 4, 0);
            DWIN_TX(0x2400, 101, 0);
            g_page_dis_status = 1;
        }

        switch (DWIN_PAGE)
        {
        case 0:           //页面0
            DWIN_ROUND_NUMB = 0;
            DWIN_TX(0x400C, g_app_config.app_config_sys.device_id, 0);                                                                  //显示设备ID号, 16位
//            FACE_DWIN_Show();
            g_show_face = 0;
            DWIN_TX(0x2310, 4, 0);
            break;

        case 1:           //页面1
            if (DWIN_ROUND_NUMB <= 3)      DWIN_ROUND_NUMB++;
            dwin_event_handle();
//            FACE_DWIN_Show();
            APP_DWIN_SUB_Face();
            DWIN_SET_RTC();                                                                                             //更新屏幕时间
            DWIN_TX(0x400C, g_app_config.app_config_sys.device_id, 1);                                                                  //显示设备ID号, 16位
            DWIN_TX(0x400F, g_app_config.app_config_sys.dtu_status, 0);                                                         //显示DTU信号, 16位
            DWIN_TX(0x400E, 1 - ALARM_OFF_FLG, 0);                                                      //报警静音标志, 16位
            //重量数据显示
            DWIN_TX(0x4958, ((int32_t)(nucWeightData * 1000)), 1);                          //重量数据, 32位
            DWIN_TX(0x4948, ((int32_t)(nucMaxWeightData * 1000)), 1);                       //额定重量, 32位
            DWIN_TX(0x4730, nucTowerMaxHigh, 0);                                    //高度数据, 16位
            if (g_charg_board.switch_charge > 0)
            {
                DWIN_TX(0xB103, g_charg_board.power_switch.bit2, 0);                                                        //激光控制标志, 16位
                DWIN_TX(0xB105, 1 - g_charg_board.connection_status, 0);                                                        //充电板通信状态, 16位
            }
            else
            {
                DWIN_TX(0xB103, 2, 0);                                                          //激光控制标志, 16位
                DWIN_TX(0xB105, 2, 0);                                                          //充电板通信状态, 16位
            }

            //倾角数据显示
            DWIN_TX(0x4954, ((int16_t)(nucDispAngleXData * 100)), 0);                       //倾角X方向角度数据, 16位
            DWIN_TX(0x4956, ((int16_t)(nucDispAngleYData * 100)), 0);                       //倾角Y方向角度数据, 16位
            APP_DWIN_SUB_Bar1(0xA001, nucDispAngleXData, QJ_ALARM_DATA_1 / 100, QJ_ALARM_DATA / 100);
            APP_DWIN_SUB_Bar1(0xA002, nucDispAngleYData, QJ_ALARM_DATA_1 / 100, QJ_ALARM_DATA / 100);
            //幅度数据显示
            DWIN_TX(0x4966, ((uint16_t)(nucLongData * 100)), 0);                                    //幅度数据, 16位
            APP_DWIN_SUB_Bar2(0x5001, nucLongData, (float)LONG_ALARM_DATA_1 / 100, (float)LONG_ALARM_DATA / 100, (float)(nucTowerLong - LONG_ALARM_DATA_1) / 100, (float)(nucTowerLong - LONG_ALARM_DATA) / 100);
//            LOG("log[%f]=%f-%f\n",nucLongData,(float)LONG_ALARM_DATA_1/100,(float)LONG_ALARM_DATA/100);

            //高度数据显示
//            DWIN_TX(0x4970,(nucHighData*100),0);                                      //高度数据, 16位
#if 0
            if (nucHighMode == 0)       //吊钩零点位置为大臂
            {
                if ((nucHighData * 100) > nucTowerMaxHigh)
                {
                    DWIN_TX(0x5002, 19, 0);                                                                         //高度比例, 16位
                }
                else
                {
                    if (nucHighData > 0)           DWIN_TX(0x5002, (uint16_t)((nucHighData * 100 * 19) / nucTowerMaxHigh), 0); //高度比例, 16位
                    else                                    DWIN_TX(0x5002, 0, 0);                              //高度比例, 16位
                }

            }
            else                                        //吊钩零点位置为地面
            {
                if ((nucHighData * 100) > nucTowerMaxHigh)
                {
                    DWIN_TX(0x5002, 0, 0);                                                                          //高度比例, 16位
                }
                else
                {
                    if (nucHighData > 0)           DWIN_TX(0x5002, (uint16_t)(((nucTowerMaxHigh - (nucHighData * 100)) * 19) / nucTowerMaxHigh), 0); //高度比例, 16位
                    else                                    DWIN_TX(0x5002, 0, 0);                              //高度比例, 16位
                }


            }
#else
            if (nucHighMode ==   0)         //吊钩零点位置为大臂
            {

                DWIN_TX(0x4970, ((uint16_t)(nucTowerMaxHigh - nucHighData * 100)), 0);                                  //高度数据, 16位
                APP_DWIN_SUB_Bar1(0x5002, (float)nucTowerMaxHigh / 100 - nucHighData, (float)nucTowerMaxHigh / 100 - (float)HIGH_ALARM_DATA_1 / 100, (float)nucTowerMaxHigh / 100 - (float)HIGH_ALARM_DATA / 100);
            }
            else
            {
                if (nucHighData > nucTowerMaxHigh / 100)
                {
                    nucHighData = nucTowerMaxHigh / 100;
                }
                DWIN_TX(0x4970, ((uint16_t)(nucHighData * 100)), 0);                                    //高度数据, 16位
                APP_DWIN_SUB_Bar1(0x5002, nucHighData, (float)nucTowerMaxHigh / 100 - (float)HIGH_ALARM_DATA_1 / 100, (float)nucTowerMaxHigh / 100 - (float)HIGH_ALARM_DATA / 100);
            }
#endif
            //力矩数据显示
            DWIN_TX(0x4962, ((uint32_t)(nucMomentData * 100)), 1);                              //力矩数据, 32位
            APP_DWIN_SUB_Bar1(0x5003, nucWeightData, 0.8f * nucMaxWeightData, nucMaxWeightData);

            DWIN_TX(0x5006, (uint16_t)(nucMomentPerData * 10000), 0);                       //力矩比例, 16位

            //回转数据显示
            DWIN_TX(0x4950, ((uint16_t)(nucAngleData * 100)), 0);                               //回转数据, 16位

            //风速数据显示
            DWIN_TX(0x4952, ((uint16_t)(nucWindData * 100)), 0);                                    //风速数据, 16位
            APP_DWIN_SUB_Bar1(0xA003, nucWindData, WIND_ALARM_DATA_1 / 100, WIND_ALARM_DATA / 100);

            //塔机臂长显示
            DWIN_TX(0x4946, ((uint16_t)(nucTowerLong)), 0);                                         //塔机臂长, 16位

            //系统倍率显示
            if (nucTowerBL == 2)               DWIN_TX(0x4944, 2, 0);                               //系统倍率, 16位
            else if (nucTowerBL == 4)  DWIN_TX(0x4944, 4, 0);                               //系统倍率, 16位
            else                                        DWIN_TX(0x4944, 2, 0);                                  //系统倍率, 16位

            //吊钩位置显示
            if ((nucLongData * 100) >= nucTowerLong)     buff = 9 * 30;
            else        buff = (uint16_t)((nucLongData * 100 * 10) / nucTowerLong) * 30;
            if (nucHighMode == 0)       //吊钩零点位置为大臂
            {
                if ((nucHighData * 100) >= nucTowerMaxHigh)      buff = buff + 29;
                else        buff = buff + ((uint16_t)((nucHighData * 100 * 30) / nucTowerMaxHigh));
            }
            else                                        //吊钩零点位置为地面
            {
                if ((nucHighData * 100) >= nucTowerMaxHigh)      buff = buff;
                else        buff = buff + (29 - ((uint16_t)((nucHighData * 100 * 30) / nucTowerMaxHigh)));
            }
            DWIN_TX(0x4007, buff, 0);                                                                           //吊钩位置, 16位

            //左右上下前后运动指示
            if (ANGLELEFT == 0)    DWIN_TX(0x4001, 1, 0);                                           //左, 16位
            else                            DWIN_TX(0x4001, 0, 0);                                              //左, 16位
            if (ANGLERIGHT == 0)   DWIN_TX(0x4002, 1, 0);                                           //右, 16位
            else                            DWIN_TX(0x4002, 0, 0);                                              //右, 16位
            if (HIGHUP == 0)           DWIN_TX(0x4003, 1, 0);                                           //上, 16位
            else                            DWIN_TX(0x4003, 0, 0);                                              //上, 16位
            if (HIGHDOWN == 0)     DWIN_TX(0x4004, 1, 0);                                           //下, 16位
            else                            DWIN_TX(0x4004, 0, 0);                                              //下, 16位
            if (LONGOUT == 0)      DWIN_TX(0x4005, 1, 0);                                           //前, 16位
            else                            DWIN_TX(0x4005, 0, 0);                                              //前, 16位
            if (LONGIN == 0)           DWIN_TX(0x4006, 1, 0);                                           //后, 16位
            else                            DWIN_TX(0x4006, 0, 0);                                              //后, 16位

            //多机编号显示
            DWIN_TX(0x4920, BenJiBianHao, 0);                                                           //本机编号, 16位
            DWIN_TX(0x4922, DuoJiBianHao[0], 0);                                                    //1#机编号, 16位
            DWIN_TX(0x4924, DuoJiBianHao[1], 0);                                                    //2#机编号, 16位
            DWIN_TX(0x4926, DuoJiBianHao[2], 0);                                                    //3#机编号, 16位
            DWIN_TX(0x4928, DuoJiBianHao[3], 0);                                                    //4#机编号, 16位
            DWIN_TX(0x492A, DuoJiBianHao[4], 0);                                                    //5#机编号, 16位
            DWIN_TX(0x492C, DuoJiBianHao[5], 0);                                                    //6#机编号, 16位
            DWIN_TX(0x492E, DuoJiBianHao[6], 0);                                                    //7#机编号, 16位
            DWIN_TX(0x4930, DuoJiBianHao[7], 0);                                                    //8#机编号, 16位

            //塔机防碰撞显示     X轴区域:90~420       Y轴区域:360~650
            //本机位置、半径

            sf = (((double)(nucTowerLong)) / 100) / 70;                                         //显示缩放倍数
            DWIN_R_X[0] = 812;
            DWIN_R_Y[0] = 260;

            DWIN_R_R[0] = (nucTowerLong / 100) / sf;
            if (DWIN_R_R[0] == 0)      DWIN_R_R[0] = 1;
            //其余8个塔机位置、半径、与本机交叉区域填充坐标
            for (i = 1; i < 9; i++)
            {
                if (nucDuoJTowerLongTbl[i - 1] > 0 && DuoJiBianHao[i - 1] > 0 && nucDuoJIDTbl[i - 1] > 0)
                {
                    double x, y, a;

                    buff_double = nucDuoJXDAngleTbl[i - 1];
                    if (buff_double < 90)
                    {
                        a = (PI * buff_double) / 180;
                        x = DWIN_R_X[0] + ((sin(a) * nucDuoJXDDisdanceTbl[i - 1]) / 100) / sf;
                        y = DWIN_R_Y[0] - ((cos(a) * nucDuoJXDDisdanceTbl[i - 1]) / 100) / sf;
                    }
                    else if (buff_double >= 90 && buff_double < 180)
                    {
                        a = (PI * (buff_double - 90)) / 180;
                        x = DWIN_R_X[0] + ((cos(a) * nucDuoJXDDisdanceTbl[i - 1]) / 100) / sf;
                        y = DWIN_R_Y[0] + ((sin(a) * nucDuoJXDDisdanceTbl[i - 1]) / 100) / sf;
                    }
                    else if (buff_double >= 180 && buff_double < 270)
                    {
                        a = (PI * (buff_double - 180)) / 180;
                        x = DWIN_R_X[0] - ((sin(a) * nucDuoJXDDisdanceTbl[i - 1]) / 100) / sf;
                        y = DWIN_R_Y[0] + ((cos(a) * nucDuoJXDDisdanceTbl[i - 1]) / 100) / sf;
                    }
                    else if (buff_double >= 270 && buff_double < 360)
                    {
                        a = (PI * (buff_double - 270)) / 180;
                        x = DWIN_R_X[0] - ((cos(a) * nucDuoJXDDisdanceTbl[i - 1]) / 100) / sf;
                        y = DWIN_R_Y[0] - ((sin(a) * nucDuoJXDDisdanceTbl[i - 1]) / 100) / sf;
                    }
                    else
                    {
                        a = 0;
                        x = DWIN_R_X[0];
                        y = DWIN_R_Y[0];
                    }

                    if (x > 0 && x < 1024 && y > 0 && y < 600)

                    {
                        DWIN_R_X[i] = (uint16_t)x;
                        DWIN_R_Y[i] = (uint16_t)y;
                    }
                    else
                    {
                        DWIN_R_X[i] = DWIN_R_X[0];
                        DWIN_R_Y[i] = DWIN_R_Y[0];
                    }

                    if (DWIN_R_X[i] == DWIN_R_X[0] && DWIN_R_Y[i] == DWIN_R_Y[0])
                    {
                        DWIN_R_R[i] = 0;
                    }
                    else
                    {
                        DWIN_R_R[i] = (nucDuoJTowerLongTbl[i - 1] / 100) / sf;
                    }
                }
                else
                {
                    DWIN_R_X[i] = DWIN_R_X[0];
                    DWIN_R_Y[i] = DWIN_R_Y[0];
                    DWIN_R_R[i] = 0;
                }
            }
            //发送画圆命令
            if (DWIN_ROUND_NUMB <= 3)
            {
                DWIN_TX_ROUND(0x1A40, 9);                                                       //画9个圆
                for (i = 0; i < 9; i++)
                {
                    if (DWIN_R_R[i] != 0)  DWIN_R_R[i] = DWIN_R_R[i] + 1; //半径加1
                }
                DWIN_TX_ROUND(0x1A80, 9);                                                       //重复描圆轮廓, 增强显示
                for (i = 0; i < 9; i++)
                {
                    if (DWIN_R_R[i] != 0)  DWIN_R_R[i] = DWIN_R_R[i] + 1; //半径加1
                }
                DWIN_TX_ROUND(0x1AC0, 9);                                                       //重复描圆轮廓, 增强显示
            }
            //8个扇形防碰撞区域的角度起点坐标、角度终点坐标、填充坐标(已改为画线坐标)
            for (i = 0; i < 8; i++)
            {
                if (nucDanJStartAngleTbl[i] != nucDanJEndAngleTbl[i])
                {
                    double x, y, a, X[4], Y[4];

                    //角度起点坐标与画线起点坐标
                    buff_double = nucDanJStartAngleTbl[i];
                    if (buff_double < 90)
                    {
                        a = (PI * buff_double) / 180;
                        x = DWIN_R_X[0] + ((sin(a) * nucTowerLong) / 100) / sf;
                        y = DWIN_R_Y[0] - ((cos(a) * nucTowerLong) / 100) / sf;
                        X[0] = DWIN_R_X[0] + (((sin(a) * nucTowerLong * P1) / 100) / 100) / sf;
                        Y[0] = DWIN_R_Y[0] - (((cos(a) * nucTowerLong * P1) / 100) / 100) / sf;
                        X[2] = DWIN_R_X[0] + (((sin(a) * nucTowerLong * P2) / 100) / 100) / sf;
                        Y[2] = DWIN_R_Y[0] - (((cos(a) * nucTowerLong * P2) / 100) / 100) / sf;
                    }
                    else if (buff_double >= 90 && buff_double < 180)
                    {
                        a = (PI * (buff_double - 90)) / 180;
                        x = DWIN_R_X[0] + ((cos(a) * nucTowerLong) / 100) / sf;
                        y = DWIN_R_Y[0] + ((sin(a) * nucTowerLong) / 100) / sf;
                        X[0] = DWIN_R_X[0] + (((cos(a) * nucTowerLong * P1) / 100) / 100) / sf;
                        Y[0] = DWIN_R_Y[0] + (((sin(a) * nucTowerLong * P1) / 100) / 100) / sf;
                        X[2] = DWIN_R_X[0] + (((cos(a) * nucTowerLong * P2) / 100) / 100) / sf;
                        Y[2] = DWIN_R_Y[0] + (((sin(a) * nucTowerLong * P2) / 100) / 100) / sf;
                    }
                    else if (buff_double >= 180 && buff_double < 270)
                    {
                        a = (PI * (buff_double - 180)) / 180;
                        x = DWIN_R_X[0] - ((sin(a) * nucTowerLong) / 100) / sf;
                        y = DWIN_R_Y[0] + ((cos(a) * nucTowerLong) / 100) / sf;
                        X[0] = DWIN_R_X[0] - (((sin(a) * nucTowerLong * P1) / 100) / 100) / sf;
                        Y[0] = DWIN_R_Y[0] + (((cos(a) * nucTowerLong * P1) / 100) / 100) / sf;
                        X[2] = DWIN_R_X[0] - (((sin(a) * nucTowerLong * P2) / 100) / 100) / sf;
                        Y[2] = DWIN_R_Y[0] + (((cos(a) * nucTowerLong * P2) / 100) / 100) / sf;
                    }
                    else if (buff_double >= 270 && buff_double < 360)
                    {
                        a = (PI * (buff_double - 270)) / 180;
                        x = DWIN_R_X[0] - ((cos(a) * nucTowerLong) / 100) / sf;
                        y = DWIN_R_Y[0] - ((sin(a) * nucTowerLong) / 100) / sf;
                        X[0] = DWIN_R_X[0] - (((cos(a) * nucTowerLong * P1) / 100) / 100) / sf;
                        Y[0] = DWIN_R_Y[0] - (((sin(a) * nucTowerLong * P1) / 100) / 100) / sf;
                        X[2] = DWIN_R_X[0] - (((cos(a) * nucTowerLong * P2) / 100) / 100) / sf;
                        Y[2] = DWIN_R_Y[0] - (((sin(a) * nucTowerLong * P2) / 100) / 100) / sf;
                    }
                    else
                    {
                        a = 0;
                        x = DWIN_R_X[0];
                        y = DWIN_R_Y[0];
                        X[0] = DWIN_R_X[0];
                        Y[0] = DWIN_R_Y[0];
                        X[2] = DWIN_R_X[0];
                        Y[2] = DWIN_R_Y[0];
                    }
                    DWIN_L_S_X[i][0] = (uint16_t)x;
                    DWIN_L_S_Y[i][0] = (uint16_t)y;
                    DWIN_F_X[i][0] = X[0];
                    DWIN_F_Y[i][0] = Y[0];
                    DWIN_F_X[i][2] = X[2];
                    DWIN_F_Y[i][2] = Y[2];

                    //角度终点坐标与画线终点坐标
                    buff_double = nucDanJEndAngleTbl[i];
                    if (buff_double < 90)
                    {
                        a = (PI * buff_double) / 180;
                        x = DWIN_R_X[0] + ((sin(a) * nucTowerLong) / 100) / sf;
                        y = DWIN_R_Y[0] - ((cos(a) * nucTowerLong) / 100) / sf;
                        X[1] = DWIN_R_X[0] + (((sin(a) * nucTowerLong * P1) / 100) / 100) / sf;
                        Y[1] = DWIN_R_Y[0] - (((cos(a) * nucTowerLong * P1) / 100) / 100) / sf;
                        X[3] = DWIN_R_X[0] + (((sin(a) * nucTowerLong * P2) / 100) / 100) / sf;
                        Y[3] = DWIN_R_Y[0] - (((cos(a) * nucTowerLong * P2) / 100) / 100) / sf;
                    }
                    else if (buff_double >= 90 && buff_double < 180)
                    {
                        a = (PI * (buff_double - 90)) / 180;
                        x = DWIN_R_X[0] + ((cos(a) * nucTowerLong) / 100) / sf;
                        y = DWIN_R_Y[0] + ((sin(a) * nucTowerLong) / 100) / sf;
                        X[1] = DWIN_R_X[0] + (((cos(a) * nucTowerLong * P1) / 100) / 100) / sf;
                        Y[1] = DWIN_R_Y[0] + (((sin(a) * nucTowerLong * P1) / 100) / 100) / sf;
                        X[3] = DWIN_R_X[0] + (((cos(a) * nucTowerLong * P2) / 100) / 100) / sf;
                        Y[3] = DWIN_R_Y[0] + (((sin(a) * nucTowerLong * P2) / 100) / 100) / sf;
                    }
                    else if (buff_double >= 180 && buff_double < 270)
                    {
                        a = (PI * (buff_double - 180)) / 180;
                        x = DWIN_R_X[0] - ((sin(a) * nucTowerLong) / 100) / sf;
                        y = DWIN_R_Y[0] + ((cos(a) * nucTowerLong) / 100) / sf;
                        X[1] = DWIN_R_X[0] - (((sin(a) * nucTowerLong * P1) / 100) / 100) / sf;
                        Y[1] = DWIN_R_Y[0] + (((cos(a) * nucTowerLong * P1) / 100) / 100) / sf;
                        X[3] = DWIN_R_X[0] - (((sin(a) * nucTowerLong * P2) / 100) / 100) / sf;
                        Y[3] = DWIN_R_Y[0] + (((cos(a) * nucTowerLong * P2) / 100) / 100) / sf;
                    }
                    else if (buff_double >= 270 && buff_double < 360)
                    {
                        a = (PI * (buff_double - 270)) / 180;
                        x = DWIN_R_X[0] - ((cos(a) * nucTowerLong) / 100) / sf;
                        y = DWIN_R_Y[0] - ((sin(a) * nucTowerLong) / 100) / sf;
                        X[1] = DWIN_R_X[0] - (((cos(a) * nucTowerLong * P1) / 100) / 100) / sf;
                        Y[1] = DWIN_R_Y[0] - (((sin(a) * nucTowerLong * P1) / 100) / 100) / sf;
                        X[3] = DWIN_R_X[0] - (((cos(a) * nucTowerLong * P2) / 100) / 100) / sf;
                        Y[3] = DWIN_R_Y[0] - (((sin(a) * nucTowerLong * P2) / 100) / 100) / sf;
                    }
                    else
                    {
                        a = 0;
                        x = DWIN_R_X[0];
                        y = DWIN_R_Y[0];
                        X[1] = DWIN_R_X[0];
                        Y[1] = DWIN_R_Y[0];
                        X[3] = DWIN_R_X[0];
                        Y[3] = DWIN_R_Y[0];
                    }
                    DWIN_L_S_X[i][1] = (uint16_t)x;
                    DWIN_L_S_Y[i][1] = (uint16_t)y;
                    DWIN_F_X[i][1] = X[1];
                    DWIN_F_Y[i][1] = Y[1];
                    DWIN_F_X[i][3] = X[3];
                    DWIN_F_Y[i][3] = Y[3];
                }
                else
                {
                    DWIN_L_S_X[i][0] = DWIN_R_X[0];
                    DWIN_L_S_Y[i][0] = DWIN_R_Y[0];
                    DWIN_L_S_X[i][1] = DWIN_R_X[0];
                    DWIN_L_S_Y[i][1] = DWIN_R_Y[0];
                    DWIN_F_X[i][0] = DWIN_R_X[0];
                    DWIN_F_Y[i][0] = DWIN_R_Y[0];
                    DWIN_F_X[i][1] = DWIN_R_X[0];
                    DWIN_F_Y[i][1] = DWIN_R_Y[0];
                    DWIN_F_X[i][2] = DWIN_R_X[0];
                    DWIN_F_Y[i][2] = DWIN_R_Y[0];
                    DWIN_F_X[i][3] = DWIN_R_X[0];
                    DWIN_F_Y[i][3] = DWIN_R_Y[0];
                }
            }
            //画扇形边界线命令
            if (DWIN_ROUND_NUMB <= 3)
            {
                for (i = 0; i < 16; i++)                        //8个扇形区域的边界线
                {
                    DWIN_TX_LINE((0x1B00+(0x40 * i)), DWIN_L_S_C, DWIN_R_X[0], DWIN_R_Y[0], DWIN_L_S_X[i / 2][i % 2], DWIN_L_S_Y[i / 2][i % 2]);
                }
                for (i = 0; i < 8; i++)                         //8个扇形区域的填充(已改为画线)
                {
                    DWIN_TX_LINE((0x1640+(0x40 * i)), DWIN_F_C, DWIN_F_X[i][0], DWIN_F_Y[i][0], DWIN_F_X[i][1], DWIN_F_Y[i][1]);
                    DWIN_TX_LINE((0x1840+(0x40 * i)), DWIN_F_C, DWIN_F_X[i][2], DWIN_F_Y[i][2], DWIN_F_X[i][3], DWIN_F_Y[i][3]);
                }
            }
            //本机当前指向坐标
            if (nucAngleData < 90)
            {
                buff_double = (PI * nucAngleData) / 180;
                DWIN_L_X[0] = DWIN_R_X[0] + ((sin(buff_double) * nucTowerLong) / 100) / sf;
                DWIN_L_Y[0] = DWIN_R_Y[0] - ((cos(buff_double) * nucTowerLong) / 100) / sf;
            }
            else if (nucAngleData >= 90 && nucAngleData < 180)
            {
                buff_double = (PI * (nucAngleData - 90)) / 180;
                DWIN_L_X[0] = DWIN_R_X[0] + ((cos(buff_double) * nucTowerLong) / 100) / sf;
                DWIN_L_Y[0] = DWIN_R_Y[0] + ((sin(buff_double) * nucTowerLong) / 100) / sf;
            }
            else if (nucAngleData >= 180 && nucAngleData < 270)
            {
                buff_double = (PI * (nucAngleData - 180)) / 180;
                DWIN_L_X[0] = DWIN_R_X[0] - ((sin(buff_double) * nucTowerLong) / 100) / sf;
                DWIN_L_Y[0] = DWIN_R_Y[0] + ((cos(buff_double) * nucTowerLong) / 100) / sf;
            }
            else if (nucAngleData >= 270 && nucAngleData < 360)
            {
                buff_double = (PI * (nucAngleData - 270)) / 180;
                DWIN_L_X[0] = DWIN_R_X[0] - ((cos(buff_double) * nucTowerLong) / 100) / sf;
                DWIN_L_Y[0] = DWIN_R_Y[0] - ((sin(buff_double) * nucTowerLong) / 100) / sf;
            }
            //其余8个塔机当前指向坐标
            for (i = 1; i < 9; i++)
            {
                if (nucDuoJTowerLongTbl[i - 1] > 0 && DuoJiBianHao[i - 1] > 0 && nucDuoJIDTbl[i - 1] > 0)
                {
                    double x, y, a;

                    buff_double = nucDuoJAngleTbl[i - 1];
                    if (buff_double < 90)
                    {
                        a = (PI * buff_double) / 180;
                        x = DWIN_R_X[i] + ((sin(a) * nucDuoJTowerLongTbl[i - 1]) / 100) / sf;
                        y = DWIN_R_Y[i] - ((cos(a) * nucDuoJTowerLongTbl[i - 1]) / 100) / sf;
                    }
                    else if (buff_double >= 90 && buff_double < 180)
                    {
                        a = (PI * (buff_double - 90)) / 180;
                        x = DWIN_R_X[i] + ((cos(a) * nucDuoJTowerLongTbl[i - 1]) / 100) / sf;
                        y = DWIN_R_Y[i] + ((sin(a) * nucDuoJTowerLongTbl[i - 1]) / 100) / sf;
                    }
                    else if (buff_double >= 180 && buff_double < 270)
                    {
                        a = (PI * (buff_double - 180)) / 180;
                        x = DWIN_R_X[i] - ((sin(a) * nucDuoJTowerLongTbl[i - 1]) / 100) / sf;
                        y = DWIN_R_Y[i] + ((cos(a) * nucDuoJTowerLongTbl[i - 1]) / 100) / sf;
                    }
                    else if (buff_double >= 270 && buff_double < 360)
                    {
                        a = (PI * (buff_double - 270)) / 180;
                        x = DWIN_R_X[i] - ((cos(a) * nucDuoJTowerLongTbl[i - 1]) / 100) / sf;
                        y = DWIN_R_Y[i] - ((sin(a) * nucDuoJTowerLongTbl[i - 1]) / 100) / sf;
                    }
                    else
                    {
                        a = 0;
                        x = DWIN_R_X[i];
                        y = DWIN_R_Y[i];
                    }
                    if (DWIN_R_X[i] == DWIN_R_X[0] && DWIN_R_Y[i] == DWIN_R_Y[0])
                    {
                        DWIN_L_X[i] = DWIN_R_X[i];
                        DWIN_L_Y[i] = DWIN_R_Y[i];
                    }
                    else
                    {
                        DWIN_L_X[i] = (uint16_t)x;
                        DWIN_L_Y[i] = (uint16_t)y;
                    }
                }
                else
                {
                    DWIN_L_X[i] = DWIN_R_X[i];
                    DWIN_L_Y[i] = DWIN_R_Y[i];
                }
            }
            //画当前指向命令
            for (i = 0; i < 9; i++)
            {
                DWIN_TX_LINE((0x1400+(0x40 * i)), DWIN_L_C[i], DWIN_R_X[i], DWIN_R_Y[i], DWIN_L_X[i], DWIN_L_Y[i]);
//                BSP_DELAY_MS(1);
            }

            break;

        case 2:           //页面2
            DWIN_ROUND_NUMB = 0;
            //风速设置
            DWIN_TX(0x4852, nucWindSignle, 0);                                                          //风速信号, 16位
            DWIN_TX(0x4848, nucWindZero, 0);                                                            //风速零点, 16位
            DWIN_TX(0x4850, nucWindCV, 0);                                                                  //风速系数, 16位

            //倾角设置
            DWIN_TX(0x4854, ((int16_t)(nucDispAngleXSignle * 100)), 0);                 //X倾角信号, 16位
            DWIN_TX(0x4856, ((int16_t)(nucDispAngleYSignle * 100)), 0);                 //Y倾角信号, 16位
            DWIN_TX(0x4858, nucDispAngleXZero * 100, 0);                                                //X倾角零点, 16位
            DWIN_TX(0x4860, nucDispAngleYZero * 100, 0);                                                //Y倾角零点, 16位
            BSP_DELAY_MS(1);

            //重量与幅度设置
            DWIN_TX(0x4862, nucTowerStartWeight, 0);                                            //起始重量, 16位
            DWIN_TX(0x4864, nucTowerStartLong, 0);                                                  //起始幅度, 16位
            DWIN_TX(0x4866, nucTowerMidWeight, 0);                                                  //中间重量, 16位
            DWIN_TX(0x4868, nucTowerMidLong, 0);                                                    //中间幅度, 16位
            DWIN_TX(0x4870, nucTowerEndWeight, 0);                                                  //终止重量, 16位
            DWIN_TX(0x4872, nucTowerEndLong, 0);                                                    //终止幅度, 16位

            //设备ID号设置
            DWIN_TX(0x4874, g_app_config.app_config_sys.device_id, 1);                                                                  //设备ID号, 16位

            //本机编号显示
            DWIN_TX(0x4900, BenJiBianHao, 0);                                                           //本机编号, 16位
            BSP_DELAY_MS(1);

            //长度高度设置
            DWIN_TX(0x4876, nucTowerLong, 0);                                                           //塔机臂长, 16位
            DWIN_TX(0x4878, nucTowerMaxHigh, 0);                                                    //塔机高度, 16位
            DWIN_TX(0x4880, nucTowerPHLong, 0);                                                         //平衡臂长, 16位
            DWIN_TX(0x4882, nucTowerTMHigh, 0);                                                         //塔帽高度, 16位

            //塔机倍率设置
            if (nucTowerBL == 2)               DWIN_TX(0x4884, 0, 0);                           //塔机倍率, 16位
            else if (nucTowerBL == 4)      DWIN_TX(0x4884, 1, 0);                           //塔机倍率, 16位
            else                                            DWIN_TX(0x4884, 0, 0);                              //塔机倍率, 16位

            DWIN_TX(0x4950, ((uint16_t)(nucAngleData * 100)), 0);                               //回转数据, 16位
            break;

        case 3:           //页面3
            DWIN_ROUND_NUMB = 0;
            //第1个扇区设置数据
            DWIN_TX(0x4500, nucDanJStartLongTbl[0], 0);                                         //起始幅度, 16位
            DWIN_TX(0x4502, nucDanJEndLongTbl[0], 0);                                           //终止幅度, 16位
            DWIN_TX(0x4504, nucDanJStartHighTbl[0], 0);                                         //起始高度, 16位
            DWIN_TX(0x4506, nucDanJEndHighTbl[0], 0);                                           //终止高度, 16位
            DWIN_TX(0x4508, nucDanJStartAngleTbl[0] * 10, 0);                               //起始角度, 16位
            DWIN_TX(0x4510, nucDanJEndAngleTbl[0] * 10, 0);                                 //终止角度, 16位

            //第2个扇区设置数据
            DWIN_TX(0x4512, nucDanJStartLongTbl[1], 0);                                         //起始幅度, 16位
            DWIN_TX(0x4514, nucDanJEndLongTbl[1], 0);                                           //终止幅度, 16位
            DWIN_TX(0x4516, nucDanJStartHighTbl[1], 0);                                         //起始高度, 16位
            DWIN_TX(0x4518, nucDanJEndHighTbl[1], 0);                                           //终止高度, 16位
            DWIN_TX(0x4520, nucDanJStartAngleTbl[1] * 10, 0);                               //起始角度, 16位
            DWIN_TX(0x4522, nucDanJEndAngleTbl[1] * 10, 0);                                 //终止角度, 16位

            //第3个扇区设置数据
            DWIN_TX(0x4524, nucDanJStartLongTbl[2], 0);                                         //起始幅度, 16位
            DWIN_TX(0x4526, nucDanJEndLongTbl[2], 0);                                           //终止幅度, 16位
            DWIN_TX(0x4528, nucDanJStartHighTbl[2], 0);                                         //起始高度, 16位
            DWIN_TX(0x4530, nucDanJEndHighTbl[2], 0);                                           //终止高度, 16位
            DWIN_TX(0x4532, nucDanJStartAngleTbl[2] * 10, 0);                               //起始角度, 16位
            DWIN_TX(0x4534, nucDanJEndAngleTbl[2] * 10, 0);                                 //终止角度, 16位

            //第4个扇区设置数据
            DWIN_TX(0x4536, nucDanJStartLongTbl[3], 0);                                         //起始幅度, 16位
            DWIN_TX(0x4538, nucDanJEndLongTbl[3], 0);                                           //终止幅度, 16位
            DWIN_TX(0x4540, nucDanJStartHighTbl[3], 0);                                         //起始高度, 16位
            DWIN_TX(0x4542, nucDanJEndHighTbl[3], 0);                                           //终止高度, 16位
            DWIN_TX(0x4544, nucDanJStartAngleTbl[3] * 10, 0);                               //起始角度, 16位
            DWIN_TX(0x4546, nucDanJEndAngleTbl[3] * 10, 0);                                 //终止角度, 16位

            //第5个扇区设置数据
            DWIN_TX(0x4548, nucDanJStartLongTbl[4], 0);                                         //起始幅度, 16位
            DWIN_TX(0x4550, nucDanJEndLongTbl[4], 0);                                           //终止幅度, 16位
            DWIN_TX(0x4552, nucDanJStartHighTbl[4], 0);                                         //起始高度, 16位
            DWIN_TX(0x4554, nucDanJEndHighTbl[4], 0);                                           //终止高度, 16位
            DWIN_TX(0x4556, nucDanJStartAngleTbl[4] * 10, 0);                               //起始角度, 16位
            DWIN_TX(0x4558, nucDanJEndAngleTbl[4] * 10, 0);                                 //终止角度, 16位

            //第6个扇区设置数据
            DWIN_TX(0x4560, nucDanJStartLongTbl[5], 0);                                         //起始幅度, 16位
            DWIN_TX(0x4562, nucDanJEndLongTbl[5], 0);                                           //终止幅度, 16位
            DWIN_TX(0x4564, nucDanJStartHighTbl[5], 0);                                         //起始高度, 16位
            DWIN_TX(0x4566, nucDanJEndHighTbl[5], 0);                                           //终止高度, 16位
            DWIN_TX(0x4568, nucDanJStartAngleTbl[5] * 10, 0);                               //起始角度, 16位
            DWIN_TX(0x4570, nucDanJEndAngleTbl[5] * 10, 0);                                 //终止角度, 16位

            //第7个扇区设置数据
            DWIN_TX(0x4572, nucDanJStartLongTbl[6], 0);                                         //起始幅度, 16位
            DWIN_TX(0x4574, nucDanJEndLongTbl[6], 0);                                           //终止幅度, 16位
            DWIN_TX(0x4576, nucDanJStartHighTbl[6], 0);                                         //起始高度, 16位
            DWIN_TX(0x4578, nucDanJEndHighTbl[6], 0);                                           //终止高度, 16位
            DWIN_TX(0x4580, nucDanJStartAngleTbl[6] * 10, 0);                               //起始角度, 16位
            DWIN_TX(0x4582, nucDanJEndAngleTbl[6] * 10, 0);                                 //终止角度, 16位

            //第8个扇区设置数据
            DWIN_TX(0x4584, nucDanJStartLongTbl[7], 0);                                         //起始幅度, 16位
            DWIN_TX(0x4586, nucDanJEndLongTbl[7], 0);                                           //终止幅度, 16位
            DWIN_TX(0x4588, nucDanJStartHighTbl[7], 0);                                         //起始高度, 16位
            DWIN_TX(0x4590, nucDanJEndHighTbl[7], 0);                                           //终止高度, 16位
            DWIN_TX(0x4592, nucDanJStartAngleTbl[7] * 10, 0);                               //起始角度, 16位
            DWIN_TX(0x4594, nucDanJEndAngleTbl[7] * 10, 0);                                 //终止角度, 16位

            break;

        case 4:           //页面4
            DWIN_ROUND_NUMB = 0;
            //第1个塔机数据
            DWIN_TX(0x4600, nucDuoJIDTbl[0], 0);                                                    //设备ID号, 16位
            DWIN_TX(0x4602, nucDuoJTowerLongTbl[0], 0);                                         //塔机臂长, 16位
            DWIN_TX(0x4604, nucDuoJXDDisdanceTbl[0], 0);                                    //相对距离, 16位
            DWIN_TX(0x4606, nucDuoJXDAngleTbl[0] * 10, 0);                                  //相对角度, 16位
            DWIN_TX(0x4608, nucDuoJXDHighTbl[0], 0);                                            //相对高度, 16位
            DWIN_TX(0x4610, (uint16_t)(nucDuoJHighTbl[0] * 100), 0);                            //当前高度, 16位
            DWIN_TX(0x4612, (uint16_t)(nucDuoJLongTbl[0] * 100), 0);                            //当前幅度, 16位
            DWIN_TX(0x4614, (uint16_t)(nucDuoJAngleTbl[0] * 10), 0);                            //当前角度, 16位

            //第2个塔机数据
            DWIN_TX(0x4616, nucDuoJIDTbl[1], 0);                                                    //设备ID号, 16位
            DWIN_TX(0x4618, nucDuoJTowerLongTbl[1], 0);                                         //塔机臂长, 16位
            DWIN_TX(0x4620, nucDuoJXDDisdanceTbl[1], 0);                                    //相对距离, 16位
            DWIN_TX(0x4622, nucDuoJXDAngleTbl[1] * 10, 0);                                  //相对角度, 16位
            DWIN_TX(0x4624, nucDuoJXDHighTbl[1], 0);                                            //相对高度, 16位
            DWIN_TX(0x4626, (uint16_t)(nucDuoJHighTbl[1] * 100), 0);                            //当前高度, 16位
            DWIN_TX(0x4628, (uint16_t)(nucDuoJLongTbl[1] * 100), 0);                            //当前幅度, 16位
            DWIN_TX(0x4630, (uint16_t)(nucDuoJAngleTbl[1] * 10), 0);                            //当前角度, 16位

            //第3个塔机数据
            DWIN_TX(0x4632, nucDuoJIDTbl[2], 0);                                                    //设备ID号, 16位
            DWIN_TX(0x4634, nucDuoJTowerLongTbl[2], 0);                                         //塔机臂长, 16位
            DWIN_TX(0x4636, nucDuoJXDDisdanceTbl[2], 0);                                    //相对距离, 16位
            DWIN_TX(0x4638, nucDuoJXDAngleTbl[2] * 10, 0);                                  //相对角度, 16位
            DWIN_TX(0x4640, nucDuoJXDHighTbl[2], 0);                                            //相对高度, 16位
            DWIN_TX(0x4642, (uint16_t)(nucDuoJHighTbl[2] * 100), 0);                            //当前高度, 16位
            DWIN_TX(0x4644, (uint16_t)(nucDuoJLongTbl[2] * 100), 0);                            //当前幅度, 16位
            DWIN_TX(0x4646, (uint16_t)(nucDuoJAngleTbl[2] * 10), 0);                            //当前角度, 16位

            //第4个塔机数据
            DWIN_TX(0x4648, nucDuoJIDTbl[3], 0);                                                    //设备ID号, 16位
            DWIN_TX(0x4650, nucDuoJTowerLongTbl[3], 0);                                         //塔机臂长, 16位
            DWIN_TX(0x4652, nucDuoJXDDisdanceTbl[3], 0);                                    //相对距离, 16位
            DWIN_TX(0x4654, nucDuoJXDAngleTbl[3] * 10, 0);                                  //相对角度, 16位
            DWIN_TX(0x4656, nucDuoJXDHighTbl[3], 0);                                            //相对高度, 16位
            DWIN_TX(0x4658, (uint16_t)(nucDuoJHighTbl[3] * 100), 0);                            //当前高度, 16位
            DWIN_TX(0x4660, (uint16_t)(nucDuoJLongTbl[3] * 100), 0);                            //当前幅度, 16位
            DWIN_TX(0x4662, (uint16_t)(nucDuoJAngleTbl[3] * 10), 0);                            //当前角度, 16位

            //第5个塔机数据
            DWIN_TX(0x4664, nucDuoJIDTbl[4], 0);                                                    //设备ID号, 16位
            DWIN_TX(0x4666, nucDuoJTowerLongTbl[4], 0);                                         //塔机臂长, 16位
            DWIN_TX(0x4668, nucDuoJXDDisdanceTbl[4], 0);                                    //相对距离, 16位
            DWIN_TX(0x4670, nucDuoJXDAngleTbl[4] * 10, 0);                                  //相对角度, 16位
            DWIN_TX(0x4672, nucDuoJXDHighTbl[4], 0);                                            //相对高度, 16位
            DWIN_TX(0x4674, (uint16_t)(nucDuoJHighTbl[4] * 100), 0);                            //当前高度, 16位
            DWIN_TX(0x4676, (uint16_t)(nucDuoJLongTbl[4] * 100), 0);                            //当前幅度, 16位
            DWIN_TX(0x4678, (uint16_t)(nucDuoJAngleTbl[4] * 10), 0);                            //当前角度, 16位

            //第6个塔机数据
            DWIN_TX(0x4680, nucDuoJIDTbl[5], 0);                                                    //设备ID号, 16位
            DWIN_TX(0x4682, nucDuoJTowerLongTbl[5], 0);                                         //塔机臂长, 16位
            DWIN_TX(0x4684, nucDuoJXDDisdanceTbl[5], 0);                                    //相对距离, 16位
            DWIN_TX(0x4686, nucDuoJXDAngleTbl[5] * 10, 0);                                  //相对角度, 16位
            DWIN_TX(0x4688, nucDuoJXDHighTbl[5], 0);                                            //相对高度, 16位
            DWIN_TX(0x4690, (uint16_t)(nucDuoJHighTbl[5] * 100), 0);                            //当前高度, 16位
            DWIN_TX(0x4692, (uint16_t)(nucDuoJLongTbl[5] * 100), 0);                            //当前幅度, 16位
            DWIN_TX(0x4694, (uint16_t)(nucDuoJAngleTbl[5] * 10), 0);                            //当前角度, 16位

            //第7个塔机数据
            DWIN_TX(0x4696, nucDuoJIDTbl[6], 0);                                                    //设备ID号, 16位
            DWIN_TX(0x4698, nucDuoJTowerLongTbl[6], 0);                                         //塔机臂长, 16位
            DWIN_TX(0x4700, nucDuoJXDDisdanceTbl[6], 0);                                    //相对距离, 16位
            DWIN_TX(0x4702, nucDuoJXDAngleTbl[6] * 10, 0);                                  //相对角度, 16位
            DWIN_TX(0x4704, nucDuoJXDHighTbl[6], 0);                                            //相对高度, 16位
            DWIN_TX(0x4706, (uint16_t)(nucDuoJHighTbl[6] * 100), 0);                            //当前高度, 16位
            DWIN_TX(0x4708, (uint16_t)(nucDuoJLongTbl[6] * 100), 0);                            //当前幅度, 16位
            DWIN_TX(0x4710, (uint16_t)(nucDuoJAngleTbl[6] * 10), 0);                            //当前角度, 16位

            //第8个塔机数据
            DWIN_TX(0x4712, nucDuoJIDTbl[7], 0);                                                    //设备ID号, 16位
            DWIN_TX(0x4714, nucDuoJTowerLongTbl[7], 0);                                         //塔机臂长, 16位
            DWIN_TX(0x4716, nucDuoJXDDisdanceTbl[7], 0);                                    //相对距离, 16位
            DWIN_TX(0x4718, nucDuoJXDAngleTbl[7] * 10, 0);                                  //相对角度, 16位
            DWIN_TX(0x4720, nucDuoJXDHighTbl[7], 0);                                            //相对高度, 16位
            DWIN_TX(0x4722, (uint16_t)(nucDuoJHighTbl[7] * 100), 0);                            //当前高度, 16位
            DWIN_TX(0x4724, (uint16_t)(nucDuoJLongTbl[7] * 100), 0);                            //当前幅度, 16位
            DWIN_TX(0x4726, (uint16_t)(nucDuoJAngleTbl[7] * 10), 0);                            //当前角度, 16位

            //本机实时数据
            DWIN_TX(0x4728, ((uint16_t)(nucLongData * 100)), 0);                                    //幅度数据, 16位
            DWIN_TX(0x4730, ((uint16_t)(nucHighData * 100)), 0);                                    //高度数据, 16位
            DWIN_TX(0x4732, ((uint16_t)(nucAngleData * 10)), 0);                                    //回转数据, 16位
            DWIN_TX(0x4734, ((uint16_t)(nucTowerLong)), 0);                                     //塔机臂长, 16位

            //8个塔机编号
            DWIN_TX(0x4902, DuoJiBianHao[0], 0);                                                    //第1个塔机编号, 16位
            DWIN_TX(0x4904, DuoJiBianHao[1], 0);                                                    //第2个塔机编号, 16位
            DWIN_TX(0x4906, DuoJiBianHao[2], 0);                                                    //第3个塔机编号, 16位
            DWIN_TX(0x4908, DuoJiBianHao[3], 0);                                                    //第4个塔机编号, 16位
            DWIN_TX(0x490A, DuoJiBianHao[4], 0);                                                    //第5个塔机编号, 16位
            DWIN_TX(0x490C, DuoJiBianHao[5], 0);                                                    //第6个塔机编号, 16位
            DWIN_TX(0x490E, DuoJiBianHao[6], 0);                                                    //第7个塔机编号, 16位
            DWIN_TX(0x4910, DuoJiBianHao[7], 0);                                                    //第8个塔机编号, 16位

            break;

        case 5:           //页面5
//            if(g_log_copy_status == 0)
        {
            //当前软件版本
            DWIN_TX(0x5008, APP_VERSION_Get_Soft_Int(), 1);                                                                     //软件版本, 16位
            DWIN_ROUND_NUMB = 0;
            //重量
            DWIN_TX(0x4800, nucWeightMinSignle, 0);                                                 //信号下限, 16位
            DWIN_TX(0x4802, nucWeightMinData, 0);                                                   //量程下限, 16位
            DWIN_TX(0x4804, nucWeightMaxSignle, 0);                                                 //信号上限, 16位
            DWIN_TX(0x4806, nucWeightMaxData, 0);                                                   //量程上限, 16位
            DWIN_TX(0x4808, nucWeightSignle, 0);                                                    //实时信号, 16位
            DWIN_TX(0x4810, ((int32_t)(nucWeightData * 100)), 1);                               //实时数据, 32位

            //幅度
//              LOG("nucLongMinSignle=%d\n",nucLongMinSignle);
            DWIN_TX(0x4812, nucLongMinSignle, 1);                                                   //信号下限, 16位
            DWIN_TX(0x4814, nucLongMinData, 0);                                                         //量程下限, 16位
            DWIN_TX(0x4816, nucLongMaxSignle, 1);                                                   //信号上限, 16位
            DWIN_TX(0x4818, nucLongMaxData, 0);                                                         //量程上限, 16位
            DWIN_TX(0x4820, nucLongSignle, 1);                                                          //实时信号, 16位
            DWIN_TX(0x4822, ((uint16_t)(nucLongData * 100)), 0);                                    //实时数据, 16位

            //高度
            DWIN_TX(0x4824, nucHighMinSignle, 1);                                                   //信号下限, 16位
            DWIN_TX(0x4826, nucHighMinData, 0);                                                         //量程下限, 16位
            DWIN_TX(0x4828, nucHighMaxSignle, 1);                                                   //信号上限, 16位
            DWIN_TX(0x4830, nucHighMaxData, 0);                                                         //量程上限, 16位
            DWIN_TX(0x4832, nucHighSignle, 1);                                                          //实时信号, 16位
            DWIN_TX(0x4834, ((uint16_t)(nucHighData * 100)), 0);                                    //实时数据, 16位

            //角度
            DWIN_TX(0x4836, nucAngleMinSignle, 0);                                                  //信号下限, 16位
            DWIN_TX(0x4838, (nucAngleMinData * 10), 0);                                         //量程下限, 16位
            DWIN_TX(0x4840, nucAngleMaxSignle, 0);                                                  //信号上限, 16位
            DWIN_TX(0x4842, (nucAngleMaxData * 10), 0);                                         //量程上限, 16位
            DWIN_TX(0x4844, nucAngleSignle, 0);                                                         //实时信号, 16位
            DWIN_TX(0x4846, ((uint16_t)(nucAngleData * 10)), 0);                                    //实时数据, 16位

            DWIN_TX(0x4972, (nucHighMode % 2), 0);                                                  //高度零点位置, 16位

            if (g_sensor_able.log_out > 0)
            {
                DWIN_TX(0xB106, 0, 0);
            }
            else
            {
                DWIN_TX(0xB106, 1, 0);
            }
        }
        break;

        case 6:           //页面6
            DWIN_ROUND_NUMB = 0;
            DWIN_TX(0x4980, (nucControlFlag % 2), 0);                                               //系统控制功能, 16位
            DWIN_TX(0x4982, (nucLongOutControl % 2), 0);                                        //幅度向外控制, 16位
            DWIN_TX(0x4984, (nucLongInControl % 2), 0);                                         //幅度向内控制, 16位
            DWIN_TX(0x4986, (nucHighUpControl % 2), 0);                                         //高度向上控制, 16位
            DWIN_TX(0x4988, (nucHighDownControl % 2), 0);                                       //高度向下控制, 16位
            DWIN_TX(0x498A, (nucAngleLeftControl % 2), 0);                                  //回转左转控制, 16位
            DWIN_TX(0x498C, (nucAngleRightControl % 2), 0);                                 //回转向右控制, 16位
            DWIN_TX(0x498E, (nucDebugFlag % 2), 0);                                                 //系统调试功能, 16位

            break;
        case 7:           //页面7
            DWIN_ROUND_NUMB = 0;
            DWIN_TX(0xA300, WIND_ALARM_DATA_1, 0);                                                  //风速预警
            DWIN_TX(0xA310, WIND_ALARM_DATA, 0);                                                  //风速报警

            DWIN_TX(0xA302, LONG_ALARM_DATA_1, 0);                                                  //幅度预警
            DWIN_TX(0xA312, LONG_ALARM_DATA, 0);                                                  //幅度报警

            DWIN_TX(0xA304, HIGH_ALARM_DATA_1, 0);                                                  //高度预警
            DWIN_TX(0xA314, HIGH_ALARM_DATA, 0);                                                  //高度报警

            DWIN_TX(0xA308, QJ_ALARM_DATA_1, 0);                                                  //角度预警
            DWIN_TX(0xA318, QJ_ALARM_DATA, 0);                                                      //角度报警

            //报警启用位设置
            DWIN_TX(0x4886, g_alarm_able.wind_speed, 0);                                                        //风速报警启用标志, 16位
            DWIN_TX(0x4888, g_alarm_able.angle, 0);                                                             //倾角报警启用标志, 16位
            DWIN_TX(0x4890, g_alarm_able.weight, 0);                                                    //重量报警启用标志, 16位

            DWIN_TX(0xA100, g_alarm_able.height, 0);                                                    //高度报警启用标志, 16位
            DWIN_TX(0xA102, g_alarm_able.range, 0);                                                     //幅度报警启用标志, 16位
            DWIN_TX(0xA104, g_alarm_able.rotation, 0);                                                      //回转报警启用标志, 16位
            DWIN_TX(0xA106, g_alarm_able.quyu, 0);                                                      //单机报警启用标志, 16位
            DWIN_TX(0xA108, g_alarm_able.duoji, 0);                                                     //多机报警启用标志, 16位


            break;
        case 8:           //页面68

            DWIN_TX(0xA400, g_charg_board.switch_charge, 0);                                                        //充电板启用禁用
            DWIN_TX(0xA402, g_charg_board.work_mode_ipc, 0);                                                    //充电板工作模式
            DWIN_TX(0xA404, g_charg_board.id, 1);                                                   //小车ID
            DWIN_TX(0xA406, g_charg_board.power_switch.bit1, 0);                                                    //摄像头控制
            DWIN_TX(0xA408, g_charg_board.power_switch.bit2, 0);                                                        //激光控制
            DWIN_TX(0xA40C, g_charg_board.work_mode_laser_ipc, 0);                                                          //激光控制

            break;
        case 9:           //页面9

            DWIN_ROUND_NUMB = 0;
//            DWIN_TX_IP(0x8100,(char*)g_app_dtu_ip[0].ip, strlen((char*)g_app_dtu_ip[0].ip));      //发送IP地址1
//            DWIN_TX(0x8010,g_app_dtu_ip[0].port,1);                               //发送端口1
            DWIN_TX_IP(0x8500, (char*)g_app_dtu_ip[1].ip, strlen((char*)g_app_dtu_ip[1].ip));       //发送IP地址2
            DWIN_TX(0x8410, g_app_dtu_ip[1].port, 1);                           //发送端口2
            DWIN_TX_IP(0x8700, (char*)g_app_dtu_ip[2].ip, strlen((char*)g_app_dtu_ip[2].ip));       //发送IP地址3
            DWIN_TX(0x8610, g_app_dtu_ip[2].port, 1);                           //发送端口3
            DWIN_TX_IP(0x8900, (char*)g_app_dtu_ip[3].ip, strlen((char*)g_app_dtu_ip[3].ip));       //发送IP地址4
            DWIN_TX(0x8810, g_app_dtu_ip[3].port, 1);                           //发送端口4

            break;
        case 13:
            DWIN_TX(0xB202, g_pin, 1);
            break;
        case 14:
            DWIN_TX(0xB210, g_new_bef.face_enable, 0);                                                          //人脸功能
            DWIN_TX(0xB212, g_sensor_able.gps, 0);                                                      //定位功能
            DWIN_TX(0xB214, g_charg_board.switch_charge, 0);                                                                 //小车智能版
            DWIN_TX(0xB216, g_sensor_able.control, 0);                                                      //控制板
            DWIN_TX(0xB218, g_sensor_able.log_out, 0);                                                          //数据导出

            DWIN_TX_IP(0xB300, g_version_model, 20);
            break;
        default:      //页面错误
            DWIN_ROUND_NUMB = 0;
            DWIN_TX(0x0084, 0x5A010001, 1);                             //切换进入首页
            break;
        }

        DWIN_RX(0x0014, 0); //读取页面
    }
}
//屏幕报警语音处理
void DWIN_ALARM_HANDLE(void)
{
    uint8_t i;
    if (ALARM_OFF_FLG == 0)           //启用语音报警
    {
        if ((nucWeightStatuse +
                nucLongStatuse +
                nucHighStatuse +
                nucWindStatuse +
                nucDispAngleStatuse +
                nucDanJiStatuse +
                nucDuojiStatuse) > 0)     //发生报警
        {
            if (DWIN_ALARM_FLG[0] != nucWeightStatuse)
            {
                if (nucWeightStatuse == 1)                        //重量预警
                {
                    if (DWIN_ALARM_NUMB != 1)    DWIN_ALARM_NUMB = 2;
                    DWIN_ALARM_FLG[0] = nucWeightStatuse;
                }
                else if (nucWeightStatuse == 2)           //重量报警
                {
                    if (DWIN_ALARM_NUMB != 1)    DWIN_ALARM_NUMB = 3;
                    DWIN_ALARM_FLG[0] = nucWeightStatuse;
                }
                else
                {
                    for (i = 0; i < 7; i++)    DWIN_ALARM_FLG[i] = 0;
                    nucWeightStatuse = 0;
                }
            }
            if (DWIN_ALARM_FLG[1] != nucLongStatuse)
            {
                if (nucLongStatuse == 1)                          //幅度向内预警
                {
                    if (DWIN_ALARM_NUMB != 1)    DWIN_ALARM_NUMB = 4;
                    DWIN_ALARM_FLG[1] = nucLongStatuse;
                }
                else if (nucLongStatuse == 2)                 //幅度向内报警
                {
                    if (DWIN_ALARM_NUMB != 1)    DWIN_ALARM_NUMB = 5;
                    DWIN_ALARM_FLG[1] = nucLongStatuse;
                }
                else if (nucLongStatuse == 3)                 //幅度向外预警
                {
                    if (DWIN_ALARM_NUMB != 1)    DWIN_ALARM_NUMB = 6;
                    DWIN_ALARM_FLG[1] = nucLongStatuse;
                }
                else if (nucLongStatuse == 4)                 //幅度向外报警
                {
                    if (DWIN_ALARM_NUMB != 1)    DWIN_ALARM_NUMB = 7;
                    DWIN_ALARM_FLG[1] = nucLongStatuse;
                }
                else
                {
                    for (i = 0; i < 7; i++)    DWIN_ALARM_FLG[i] = 0;
                    nucLongStatuse = 0;
                }
            }
            if (DWIN_ALARM_FLG[2] != nucHighStatuse)
            {
                if (nucHighStatuse == 1)                          //高度预警
                {
                    if (DWIN_ALARM_NUMB != 1)    DWIN_ALARM_NUMB = 8;
                    DWIN_ALARM_FLG[2] = nucHighStatuse;
                }
                else if (nucHighStatuse == 2)                 //高度报警
                {
                    if (DWIN_ALARM_NUMB != 1)    DWIN_ALARM_NUMB = 9;
                    DWIN_ALARM_FLG[2] = nucHighStatuse;
                }
                else
                {
                    for (i = 0; i < 7; i++)    DWIN_ALARM_FLG[i] = 0;
                    nucHighStatuse = 0;
                }
            }
            if (DWIN_ALARM_FLG[3] != nucWindStatuse)
            {
                if (nucWindStatuse == 1)                          //风速预警
                {
                    if (DWIN_ALARM_NUMB != 1)    DWIN_ALARM_NUMB = 10;
                    DWIN_ALARM_FLG[3] = nucWindStatuse;
                }
                else if (nucWindStatuse == 2)                 //风速报警
                {
                    if (DWIN_ALARM_NUMB != 1)    DWIN_ALARM_NUMB = 11;
                    DWIN_ALARM_FLG[3] = nucWindStatuse;
                }
                else
                {
                    for (i = 0; i < 7; i++)    DWIN_ALARM_FLG[i] = 0;
                    nucWindStatuse = 0;
                }
            }
            if (DWIN_ALARM_FLG[4] != nucDispAngleStatuse)
            {
                if (nucDispAngleStatuse == 1)                 //倾角预警
                {
                    if (DWIN_ALARM_NUMB != 1)    DWIN_ALARM_NUMB = 12;
                    DWIN_ALARM_FLG[4] = nucDispAngleStatuse;
                }
                else if (nucDispAngleStatuse == 2)        //倾角报警
                {
                    if (DWIN_ALARM_NUMB != 1)    DWIN_ALARM_NUMB = 13;
                    DWIN_ALARM_FLG[4] = nucDispAngleStatuse;
                }
                else
                {
                    for (i = 0; i < 7; i++)    DWIN_ALARM_FLG[i] = 0;
                    nucDispAngleStatuse = 0;
                }
            }
            if (DWIN_ALARM_FLG[5] != nucDanJiStatuse)
            {
                if (nucDanJiStatuse == 1)                         //区域防碰撞报警
                {
                    if (DWIN_ALARM_NUMB != 1)    DWIN_ALARM_NUMB = 14;
                    DWIN_ALARM_FLG[5] = nucDanJiStatuse;
                }
                else
                {
                    for (i = 0; i < 7; i++)    DWIN_ALARM_FLG[i] = 0;
                    nucDanJiStatuse = 0;
                }
            }
            if (DWIN_ALARM_FLG[6] != nucDuojiStatuse)
            {
                if (nucDuojiStatuse == 1)                         //多机防碰撞预警
                {
                    if (DWIN_ALARM_NUMB != 1)    DWIN_ALARM_NUMB = 15;
                    DWIN_ALARM_FLG[6] = nucDuojiStatuse;
                }
                else if (nucDuojiStatuse == 2)                //多机防碰撞报警
                {
                    if (DWIN_ALARM_NUMB != 1)    DWIN_ALARM_NUMB = 16;
                    DWIN_ALARM_FLG[6] = nucDuojiStatuse;
                }
                else
                {
                    for (i = 0; i < 7; i++)    DWIN_ALARM_FLG[i] = 0;
                    nucDuojiStatuse = 0;
                }
            }
        }
        else
        {
            if (DWIN_ALARM_NUMB != 1)    DWIN_ALARM_NUMB = 0;
            for (i = 0; i < 7; i++)    DWIN_ALARM_FLG[i] = 0;
        }
    }
    else                                              //禁用语音报警
    {
        if (DWIN_ALARM_NUMB != 1)    DWIN_ALARM_NUMB = 0;
        for (i = 0; i < 7; i++)    DWIN_ALARM_FLG[i] = 0;
    }

    if (DWIN_WAV_FLG == 1)
    {
        DWIN_WAV_FLG = 0;
        if (DWIN_ALARM_NUMB == 1)
        {
            DWIN_WAV_NUMB = 100;                                                  //开机音效, 延时10秒
            DWIN_WAV(DWIN_ALARM_NUMB);                                    //播放指定音频文件
            DWIN_ALARM_NUMB = 0;                                                  //开机音效只播放一次
        }
        else
        {
            if (DWIN_ALARM_NUMB == 0)    DWIN_WAV_NUMB = 10; //等待报警, 延时1秒
            else                                      DWIN_WAV_NUMB = 30; //报警音效, 延时3秒
            DWIN_WAV(DWIN_ALARM_NUMB);//播放指定音频文件
        }
    }
}
#endif
#define IPC_CNT_TIMEOUT   (10*60) //1分钟
static uint8_t g_duoji_send_status = 0;
uint32_t g_ipc_cnt_status = 0;//IPC_CNT_TIMEOUT;
uint32_t g_ipc_cnt_status_ncmp = 0;//配置同步;
uint32_t g_g_charg_board_num = 0;
extern uint32_t g_last_ipc_time;
void APP_USER_Handle(void)//100ms运行1次
{
    if (DWIN_WAV_FLG == 0)
    {
        if (DWIN_WAV_NUMB > 0)
        {
            DWIN_WAV_NUMB--;
        }
        else
        {
            DWIN_WAV_FLG = 1;
        }
    }
    g_last_ipc_time++;
    if (++g_ipc_cnt_status_ncmp > 30) //配置同步;
    {
        g_ipc_cnt_status_ncmp = 0;
        g_sendip_set_status = 5;
    }
    if (g_app_pic.pic_timeout < 40) //拍照超时处理
    {
        if (++g_app_pic.pic_timeout >= 40)
        {
            if (++g_app_pic.pic_read_time < 6)
            {
                nucSendGetPic(g_app_pic.pic_start, g_app_pic.pic_size_get); //读图片
                LOG("get pic again[%d]...\n", g_app_pic.pic_read_time);
            }
            else
            {
                if (g_app_pic.pic_status == 3) //平台要图片
                {
                    nucSendPicToServer(1);
                    LOG("Remote Get Pic timeout!!\n");
                }
                else//工作循环图片
                {
                    g_app_pic.pic_status = 0;
                    LOG("WorkCycle Get Pic timeout!!\n");
                }
            }
        }
    }
//    nucGetADSignle();     /*AD采集  */
//    nucSysADDataLoad();   /*AD数据整合    */
    if (g_app_pic.pic_wait > 0 && g_app_pic.pic_status == 0) //拍照延迟
    {
        nucSendPhoto(1, 2, g_app_pic.pic_wait);
        g_app_pic.pic_wait = 0;
    }
    if (++g_g_charg_board_num > 20) //100ms运行20次=2秒
    {
        g_g_charg_board_num = 0;
        APP_MAIN_Callback_Heartbeat_Board();//charge
    }
//    bsp_adc_vol_get();//检测电压
    if (APP_USER_Work_Mode() == 1)
    {
        return;
    }

    APP_USER_Work_Freq();

    bsp_rotation_twist_get();
    nucSysDataLoad();         /*工作数据载入*/

    if (g_nucWeightData_work_start == 100) //重量初值
    {
        g_nucWeightData_work_start = 99;
        nucWeightData_work = nucWeightData + 1; //相对0点>真实值
    }

    BSP_RTC_Date_Get(&g_bsp_rtc);/*获取时间*/
    if (++g_ipc_cnt_status < IPC_CNT_TIMEOUT)
    {
        if (nucSendWorkDataFlag == 2)
        {
            nucSendWorkDataFlag = 3;
            nucSendPasswordSetData(0x8204, BSP_RTC_Stamp_Get());
        }
    }
    else
    {
        g_ipc_cnt_status = IPC_CNT_TIMEOUT;
        nucSendWorkDataFlag = 2;
    }

    nucSysAlarmDataHandle();  /*系统报警数据处理*/
    APP_USER_Event_Handle();
//  nucSysDanJiAlarmHandle();/*单机防碰撞报警处理*/
    nucSysWorkCycleCount();   /*塔机工作循环计算*/
    nucSendDtuData();         /*发送DTU数据   */
    nucDuoJFPZJudge1();/*多机防碰撞报警处理*/

    APP_WIRELESS_Duoji_Check();
    APP_WIRELESS_Charging_Overtime_Check();

//以下黑匣子主动发送广播
    if (g_sensor_able.duoji == 1)
    {
        if (nucLongData_last != nucLongData ||
                nucHighData_last != nucHighData ||
                nucAngleData_last != nucAngleData)
        {
            APP_WIRELESS_SendDuoJiData();

            nucLongData_last = nucLongData;
            nucHighData_last = nucHighData;
            nucAngleData_last = nucAngleData;
        }
        else if (++g_duoji_send_status >= 10)
        {
            g_duoji_send_status = 0;
            APP_WIRELESS_SendDuoJiData();
        }
    }


    if (nucDebugFlag == 0) //未进入debug模式
    {
        if (nucControlFlag > 0) //控制盒打开使能
        {
            if (nucDanJiStatuse == 0 && nucDuojiStatuse < 2)
            {
                ANGLELEFTOFF;
                ANGLERIGHTOFF;
                if ((nucLongStatuse != 4) && (nucWeightStatuse != 2))
                {
                    LONGOUTOFF;//20250807
                }

//                 LONGINOFF;  //20250707
            }
            //   LOG("nucDuojiStatuse=%d\n",nucDuojiStatuse);
            //多机减速 或 单机减速
            //  if(nucDuojiStatuse==1 ||  nucLongStatuse2==1)
            if (nucDuojiStatuse == 1 || nucDuojiStatuse == 2  || nucSysDanJiJiansulFlag == 1 || nucLongStatuse2 == 1)// 20250707
            {
                if (user_cfg_warn_ctl > 0) //第二路全控
                {
                    BSP_RELAY_Ctl(255, 1);//单控制减速的函数，新板继电器板采用modbus通讯协议
                }
                else
                {
                    BSP_RELAY_Ctl(BSP_RELAY_RELAY6, 1); //高度向下控制（实际映射到 HIGHDOWN）
                }
            }
            else
            {
                if (user_cfg_warn_ctl > 0) //第二路全控
                {
                    BSP_RELAY_Ctl(255, 0);//单控制减速的函数，新板继电器板采用modbus通讯协议
					BSP_RELAY_Ctl(BSP_RELAY_RELAY6, 0); //高度向下控制
                }
                else
                {
                    BSP_RELAY_Ctl(BSP_RELAY_RELAY6, 0); //高度向下控制
				//	HIGHDOWNOFF;//高度向下不好用
                }
            }

        }
    }
    else
    {
        /*幅度向内控制-8044 2020-03-22*/
        if (nucLongInControl > 0)
        {
            LONGINON;
        }
        else
        {
            LONGINOFF;
        }
        if (nucLongOutControl > 0)
        {
            LONGOUTON;
        }
        else
        {
            LONGOUTOFF;
        }

        if (nucHighUpControl > 0)
        {
            HIGHUPON;
        }
        else
        {
            HIGHUPOFF;
        }
        if (nucHighDownControl > 0)
        {
            HIGHDOWNON;
        }
        else
        {
            HIGHDOWNOFF;
        }
        if (nucAngleLeftControl > 0)
        {
            ANGLELEFTON;
        }
        else
        {
            ANGLELEFTOFF;
        }

        if (nucAngleRightControl > 0)
        {
            ANGLERIGHTON;
        }
        else
        {
            ANGLERIGHTOFF;
        }
    }


#if 1
    //报警事件
    if (nucAlarmTbl_last[0] != nucAlarmTbl[0]
            || nucAlarmTbl_last[1] != nucAlarmTbl[1]
            || nucAlarmTbl_last[2] != nucAlarmTbl[2]
            || nucAlarmTbl_last[3] != nucAlarmTbl[3])
    {
        if (nucAlarmTbl_last[0] < nucAlarmTbl[0]
                || nucAlarmTbl_last[1] < nucAlarmTbl[1]
                || nucAlarmTbl_last[2] < nucAlarmTbl[2]
                || nucAlarmTbl_last[3] < nucAlarmTbl[3])
        {
            nucSendAlarmData();
        }
        nucAlarmTbl_last[0] = nucAlarmTbl[0];
        nucAlarmTbl_last[1] = nucAlarmTbl[1];
        nucAlarmTbl_last[2] = nucAlarmTbl[2];
        nucAlarmTbl_last[3] = nucAlarmTbl[3];
    }

#endif
#if USE_DWIN
    DWIN_HANDLE();
    DWIN_ALARM_HANDLE();
    FACE_EVENT_Handle();
#endif

}
int APP_USER_rotation_type(void)
{
    return g_new_bef.rotation_type;
}
void APP_USER_Init(void)
{
    APP_USER_Config_Init();//从flash中读取默认值
//    log_def log_save = {0};
//    BSP_FAT_Read_Log(&log_save, sizeof(log_def));
//      uint8_t *p = (uint8_t*)&log_save;
//      LOG_HEX(p, sizeof(log_def));
//    g_new_bef.twist_last = log_save.dlnd;
////    LOG("twist_last = %d\n",g_new_bef.twist_last);

//    bsp_power_off_no_flash();
//    rt_thread_mdelay(1000);
//    bsp_power_on_no_flash();

    BSP_TIMER_Init(&g_timer_user, APP_USER_Handle, TIMEOUT_100MS, TIMEOUT_100MS);
    BSP_TIMER_Start(&g_timer_user);

}

/*****END OF FILE****/
