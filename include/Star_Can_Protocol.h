#ifndef _STAR_CAN_PROTOCOL_H
#define _STAR_CAN_PROTOCOL_H

#include <stdint.h>
#include <string.h>
#include <Bsp_Can_Linux.h>


#define CAN_MAX_DLC         8	  // 标准帧最大长度
#define CAN_MAX_DATA_LEN    1024  // 复帧最大长度
#define Plat_Can_Msg BSP_Can_Msg



/* --- 节点地址定义 (对应文档 表5) --- */
typedef enum {
    NODE_ADDR_OBC          = 0x00, // 星务计算机 (主节点)
    NODE_ADDR_GNSS_SIM     = 0x01, // GNSS模拟器
    NODE_ADDR_GPU          = 0x05, // 边缘计算GPU
    NODE_ADDR_TMTC_SIM     = 0x06, // 测控应答机模拟器
    NODE_ADDR_DTS_SIM      = 0x07, // 数传载荷模拟器
} NodeAddress_t;

/* --- 帧类型定义 (对应文档 表4) --- */
typedef enum {
    FRAME_TYPE_CMD         = 0x00, // 控制指令 (00)
    FRAME_TYPE_BROADCAST   = 0x01, // 广播通信 (01)
    FRAME_TYPE_BUS_CTRL    = 0x02, // 总线相关数据与命令 (10)
    FRAME_TYPE_POLL        = 0x03  // 轮询指令 (11)
} FrameType_t;

/* --- 单复帧信息 (对应文档 表7) --- */
typedef enum {
    MULTI_FRAME_SINGLE     = 0x00, // 单帧
    MULTI_FRAME_FIRST      = 0x01, // 复帧首帧
    MULTI_FRAME_MIDDLE     = 0x02, // 复帧中间帧
    MULTI_FRAME_LAST       = 0x03  // 复帧末帧
} MultiFrameType_t;

/* --- 广播数据类型 (对应文档 6.1, 6.2) --- */
typedef enum {
    BROADCAST_STAR_TIME    		= 0x11, // 星上时间
    BROADCAST_SATELLITE_ORBIT   = 0x22  // 卫星轨道
} BroadcastDataType_t;

/* --- CAN ID 解析结构体 --- */
typedef struct {
    uint8_t bus_owner;      // ID10: 总线占用标志
    uint8_t frame_type;     // ID9-8：帧类型	
    uint8_t node_addr;      // ID7-3：节点地址
    uint8_t bus_flag;       // ID2: AB总线标志
    uint8_t multi_frame;    // ID1-0：单复帧信息
} CanIdInfo_t;

typedef enum{
	Portocol_OK      = 0,
    Portocol_ERROR   = 1,
	Portocol_BUSY    = 2,
    Portocol_TIMEOUT = 3,
}Portocol_Status;
	
typedef enum{
	Star_Computer = 0x00000,
	GNSS_Simulator = 0x00001,
	GPU_Simulator = 0X00101,
	Telemetry_Transponder_Simulator = 0x00110,
	Data_Transmission_Payload_Simulator = 0x00111
}target_addr;




typedef struct {
    uint32_t id;      
    uint8_t  data[8]; 
    uint8_t  dlc;     
} Portocol_CanFrame_t;

/* 上层接收回调：完整应用帧（单帧或复帧拼接完成）上传 */
typedef void (*Can_Protocol_RxCallback_t)(uint32_t can_id, uint8_t frame_type,
                                          const uint8_t *data, uint16_t len);



void Can_Protocol_RxHandler(uint32_t can_id, uint8_t data[], uint8_t DLC);
void Can_Protocol_SetRxCallback(Can_Protocol_RxCallback_t cb);
void Can_Protocol_SetSendFd(int fd);
void Can_Protocol_SendPollCmd(uint8_t target_addr, uint8_t packet_id, uint8_t bus_sel);
void Can_Protocol_SendData(uint8_t target_addr, uint8_t bus_sel, uint8_t *data, uint8_t DLC);


#endif
