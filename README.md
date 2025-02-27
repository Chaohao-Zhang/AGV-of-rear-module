## Enviroment：UBUNTU 18.04
## Rear Vehicle Functions  
1. ​**Remote controller**​ for robot movement  
2. ​**IO module control**​ to open/close the secondary arm  
3. ​**Motor control**​ for opening/closing the main arm  
4. ​**Speed synchronization**​ with the front vehicle via TCP protocol  

*Technical Notes:*  
- "副臂/主臂" = `secondary arm/main arm`  
- "开合"可根据机械需求为 `open/close` 或 `expand/retract`  
- "前车"指联动的前端设备（`front module`）  
- 协议实现需确保 TCP 通信的实时性和稳定性  
