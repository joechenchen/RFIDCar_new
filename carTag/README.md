#carTag  
2.4G send carTag info  
inter parameter area config  
reserve area pameter config   
#PIN Define 
UART  
TX-P0.5  
RX-P0.6  
2016/12/2 
射频周期性1s发送，但是接收端未能收到标签ID，因为发送成功后，未diable射频，直接关闭外部时钟，增加disable之后，一切正常。  
后期需增加低电状态位、工作模式状态上报。

