#include "esphome.h"

static const char *TAG = "Vimar.UART";
namespace esphome {
class MyCustomComponent : public Component, public UARTDevice, public Sensor {
 public:
  MyCustomComponent(UARTComponent *parent) : UARTDevice(parent) {}
  Sensor *temperature_sensor = new Sensor();
  Sensor *temperature_setted = new Sensor();
  Sensor *VimarOnOff = new Sensor();
  Sensor *VimarHeatCold = new Sensor();
  void setup() override {
    // nothing to do here
  }

  int readline(int readch, char *buffer, int len)
  {
    static int pos = 0;
    int rpos;

    //if (readch > 0) {
      switch (readch) {
        //case '\n': // Ignore new-lines
          //break;
        case '\n': // Return on CR
		if(pos < len-1) 
		{
		}
		else
		{
		  rpos = pos;
          pos = 0;  // Reset position index ready for next time
          return rpos;
		}
   
        default:
          if (pos < len-1) {
            buffer[pos++] = readch;
            buffer[pos] = 0;
          }
      }
    //}
    // No end of line has been found, so return -1.
    return -1;
  }

  void loop() override {
    const int max_line_length = 8;
    static char buffer[max_line_length];
	int16_t Temp;
	int16_t TempSetted;

	
    while (available()) {
      if(readline(read(), buffer, max_line_length) > 0) {
		  
		  //convert to float string
		  Temp = buffer[0] | buffer[1] << 8;
		  TempSetted = buffer[5] | buffer[6] << 8;
		  
		  //ESP_LOGD(TAG, "Temp UART: %d", Temp);
		  //ESP_LOGD(TAG, "Tempsetted UART: %d", TempSetted);	  
		  //ESP_LOGD(TAG, "UART: %02x %02x %02x %02x %02x %02x %02x %02x", buffer[0],buffer[1],buffer[2],buffer[3],buffer[4],buffer[5],buffer[6],buffer[7]);	  
        //publish_state(buffer);
		float temperature = (float)Temp/10;
		float temperaturesetted = (float)TempSetted/10;
		float VimarState;
		float VimarHC;
		//OnOFF
		if(buffer[3] == '\0') VimarState = std::stof("1.0");
		else VimarState = std::stof("0");
		
		if(buffer[2] == '\0') VimarHC = std::stof("1.0");
		else VimarHC = std::stof("0");
		
		ESP_LOGD(TAG, "UART read: %02x %02x %02x %02x %02x %02x %02x %02x", buffer[0],buffer[1],buffer[2],buffer[3],buffer[4],buffer[5],buffer[6],buffer[7]);	  
		
		temperature_sensor->publish_state(temperature);
		temperature_setted->publish_state(temperaturesetted);
		VimarOnOff->publish_state(VimarState);
		VimarHeatCold->publish_state(VimarHC);
      }
    }
  }
};
}