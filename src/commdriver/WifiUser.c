
#include "commdriver/Wifi.h"

//#######################################################################################
void	Wifi_UserInit(void)
{
//	Wifi_SetMode(WifiMode_Station);
  Wifi_SetMode(WifiMode_SoftAp);

//  while (Wifi_Station_ConnectToAp("L70_9587","33559366",NULL) == false);
  Wifi_SoftAp_Create("TEST", "", 2, WifiEncryptionType_Open, 2, FALSE);
 
    
}
//#######################################################################################
void  Wifi_UserProcess(void)
{

}
//#######################################################################################
void  Wifi_UserGetUdpData(uint8_t LinkId,uint16_t DataLen,uint8_t *Data)
{
  Wifi_TcpIp_SendDataUdp(LinkId,2,(uint8_t*)"OK");
}
//#######################################################################################
void  Wifi_UserGetTcpData(uint8_t LinkId,uint16_t DataLen,uint8_t *Data)
{
  Wifi_TcpIp_SendDataTcp(LinkId,2,(uint8_t*)"OK");
}
//#######################################################################################
