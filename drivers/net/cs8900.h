#ifndef CS8900_H
#define CS8900_H





#define PP_Address        0x0a    
#define PP_Data            0x0c    



#define PP_ProductID        0x0000    
#define PP_MemBase            0x002c    
#define PP_IntNum            0x0022    
#define PP_EEPROMCommand    0x0040    
#define PP_EEPROMData        0x0042    
#define PP_RxCFG            0x0102    
#define PP_RxCTL            0x0104    
#define PP_TxCFG            0x0106    
#define PP_BufCFG            0x010a    
#define PP_LineCTL            0x0112    
#define PP_SelfCTL            0x0114    
#define PP_BusCTL            0x0116    
#define PP_TestCTL            0x0118    
#define PP_ISQ                0x0120    
#define PP_TxEvent            0x0128    
#define PP_BufEvent            0x012c    
#define PP_RxMISS            0x0130    
#define PP_TxCOL            0x0132    
#define PP_SelfST            0x0136    
#define PP_BusST            0x0138    
#define PP_TxCMD            0x0144    
#define PP_TxLength            0x0146    
#define PP_IA                0x0158    
#define PP_RxStatus            0x0400    
#define PP_RxLength            0x0402    
#define PP_RxFrame            0x0404    
#define PP_TxFrame            0x0a00    




#define INTRQ0            0x0000
#define INTRQ1            0x0001
#define INTRQ2            0x0002
#define INTRQ3            0x0003


#define EISA_REG_CODE    0x630e
#define REVISION(x)        (((x) & 0x1f00) >> 8)
#define VERSION(x)        ((x) & ~0x1f00)

#define CS8900A            0x0000
#define REV_B            7
#define REV_C            8
#define REV_D            9


#define Skip_1            0x0040
#define StreamE            0x0080
#define RxOKiE            0x0100
#define RxDMAonly        0x0200
#define AutoRxDMAE        0x0400
#define BufferCRC        0x0800
#define CRCerroriE        0x1000
#define RuntiE            0x2000
#define ExtradataiE        0x4000


#define IAHashA            0x0040
#define PromiscuousA    0x0080
#define RxOKA            0x0100
#define MulticastA        0x0200
#define IndividualA        0x0400
#define BroadcastA        0x0800
#define CRCerrorA        0x1000
#define RuntA            0x2000
#define ExtradataA        0x4000


#define Loss_of_CRSiE    0x0040
#define SQErroriE        0x0080
#define TxOKiE            0x0100
#define Out_of_windowiE    0x0200
#define JabberiE        0x0400
#define AnycolliE        0x0800
#define T16colliE        0x8000


#define SWint_X            0x0040
#define RxDMAiE            0x0080
#define Rdy4TxiE        0x0100
#define TxUnderruniE    0x0200
#define RxMissiE        0x0400
#define Rx128iE            0x0800
#define TxColOvfiE        0x1000
#define MissOvfloiE        0x2000
#define RxDestiE        0x8000


#define SerRxON            0x0040
#define SerTxON            0x0080
#define AUIonly            0x0100
#define AutoAUI_10BT    0x0200
#define ModBackoffE        0x0800
#define PolarityDis        0x1000
#define L2_partDefDis    0x2000
#define LoRxSquelch        0x4000


#define RESET            0x0040
#define SWSuspend        0x0100
#define HWSleepE        0x0200
#define HWStandbyE        0x0400
#define HC0E            0x1000
#define HC1E            0x2000
#define HCB0            0x4000
#define HCB1            0x8000


#define ResetRxDMA        0x0040
#define DMAextend        0x0100
#define UseSA            0x0200
#define MemoryE            0x0400
#define DMABurst        0x0800
#define IOCHRDYE        0x1000
#define RxDMAsize        0x2000
#define EnableRQ        0x8000


#define DisableLT        0x0080
#define ENDECloop        0x0200
#define AUIloop            0x0400
#define DisableBackoff    0x0800
#define FDX                0x4000


#define RegNum(x) ((x) & 0x3f)
#define RegContent(x) ((x) & ~0x3d)

#define RxEvent            0x0004
#define TxEvent            0x0008
#define BufEvent        0x000c
#define RxMISS            0x0010
#define TxCOL            0x0012


#define IAHash            0x0040
#define Dribblebits        0x0080
#define RxOK            0x0100
#define Hashed            0x0200
#define IndividualAdr    0x0400
#define Broadcast        0x0800
#define CRCerror        0x1000
#define Runt            0x2000
#define Extradata        0x4000

#define HashTableIndex(x) ((x) >> 0xa)


#define After5            0
#define After381        1
#define After1021        2
#define AfterAll        3
#define TxStart(x) ((x) << 6)

#define Force            0x0100
#define Onecoll            0x0200
#define InhibitCRC        0x1000
#define TxPadDis        0x2000


#define TxBidErr        0x0080
#define Rdy4TxNOW        0x0100


#define Loss_of_CRS        0x0040
#define SQEerror        0x0080
#define TxOK            0x0100
#define Out_of_window    0x0200
#define Jabber            0x0400
#define T16coll            0x8000

#define TX_collisions(x) (((x) >> 0xb) & ~0x8000)


#define SWint            0x0040
#define RxDMAFrame        0x0080
#define Rdy4Tx            0x0100
#define TxUnderrun        0x0200
#define RxMiss            0x0400
#define Rx128            0x0800
#define RxDest            0x8000


#define MissCount(x) ((x) >> 6)


#define ColCount(x) ((x) >> 6)


#define T3VActive        0x0040
#define INITD            0x0080
#define SIBUSY            0x0100
#define EEPROMpresent    0x0200
#define EEPROMOK        0x0400
#define ELpresent        0x0800
#define EEsize            0x1000


#define EEWriteEnable 0x00F0
#define EEWriteDisable 0x0000
#define EEWriteRegister    0x0100
#define EEReadRegister    0x0200
#define EEEraseRegister    0x0300
#define ELSEL            0x0400

#endif  
