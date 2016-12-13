#reader   
config other device
#PIN Define  
UART  
TX-P0.5  
RX-P0.4  
  
2016/12/2 add bootloader  
2016/12/5 modify code:  
1-add LNA_Board ,but not define.  
2-add, Write reader ID(FFFFFFF9) to ID_BEGIN,because bootload need to compare id.  
0x3d000-high byte 0x3d003-low byte  
3-Modify if(0x80 == (payload[6]&0xff)) to if(0x80 == (payload[6]&0x80))