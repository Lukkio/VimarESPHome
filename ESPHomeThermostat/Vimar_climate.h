#include "esphome.h"
static const char *TAG2 = "Vimar.climate";
class VimarClimate : public Climate, public Component {
 public:
  sensor::Sensor *sensor_{nullptr};
  sensor::Sensor *tempsetted_{nullptr};
  sensor::Sensor *OnOff_{nullptr};
  sensor::Sensor *HeatCold_{nullptr};
  UARTComponent *UARTDevice_{nullptr};
  
  void setup() override {
    // This will be called by App.setup()
	if (this->sensor_) {
      this->sensor_->add_on_state_callback([this](float state) {
        this->current_temperature = state;

        // current temperature changed, publish state
        //this->publish_state();
      });
      this->current_temperature = this->sensor_->state;
    } else
      this->current_temperature = NAN;
  
	if (this->tempsetted_) {
      this->tempsetted_->add_on_state_callback([this](float state) {
        this->target_temperature = state;

        // current temperature changed, publish state
        this->publish_state();
      });
      //this->target_temperature = this->tempsetted_->state;
    } else
      this->target_temperature = NAN;
  
	if (this->OnOff_) {
      this->OnOff_->add_on_state_callback([this](float state) {
		  if((int)state >=1)
		  { 
			if((int)this->HeatCold_->state>=1)
			  {
				  this->mode = ClimateMode::CLIMATE_MODE_HEAT;
			  }
			  else this->mode = ClimateMode::CLIMATE_MODE_COOL;
			  
			//this->mode = ClimateMode::CLIMATE_MODE_HEAT;
			//this->action = climate::CLIMATE_ACTION_HEATING;
			//this->action = climate::CLIMATE_ACTION_IDLE;
		  }
		  else
		  { 
			this->mode = ClimateMode::CLIMATE_MODE_OFF;
			//this->action = climate::CLIMATE_ACTION_IDLE;
		  }

        // current temperature changed, publish state
        this->publish_state();
      });
      //this->target_temperature = this->tempsetted_->state;
    }
	
	if (this->HeatCold_) {
      this->HeatCold_->add_on_state_callback([this](float state) {
		  if((int)state >=1)
		  { 
			  if((int)this->OnOff_->state>=1)
			  {
				  this->mode = ClimateMode::CLIMATE_MODE_HEAT;
			  }
			  else this->mode = ClimateMode::CLIMATE_MODE_OFF;
			  
			//this->mode = ClimateMode::CLIMATE_MODE_COOL;
			//this->action = climate::CLIMATE_ACTION_HEATING;
			//this->action = climate::CLIMATE_ACTION_IDLE;
		  }
		  else
		  { 
			if((int)this->OnOff_->state>=1)
			  {
				  this->mode = ClimateMode::CLIMATE_MODE_COOL;
			  }
			  else this->mode = ClimateMode::CLIMATE_MODE_OFF;
			  
			//this->mode = ClimateMode::CLIMATE_MODE_HEAT;
			//this->action = climate::CLIMATE_ACTION_IDLE;
		  }

        // current temperature changed, publish state
        this->publish_state();
      });
      //this->target_temperature = this->tempsetted_->state;
    }
  
  
  
  
	this->publish_state();
  
  }
  void control(const ClimateCall &call) override {
	  char bufferuart[9];
    if (call.get_mode().has_value()) {
      // User requested mode change
      ClimateMode mode = *call.get_mode();
      // Send mode to hardware
      // ...
	  if(mode == climate::CLIMATE_MODE_OFF)
	  {
		  sprintf(bufferuart,"Set4 \x01\n");
		  UARTDevice_->write_array((const uint8_t*)bufferuart,7);
		  ESP_LOGD(TAG2,"UART write: %d,", mode);
	  }
	  if(mode == climate::CLIMATE_MODE_COOL)
	  {
		  sprintf(bufferuart,"Set7 xx\n");
		  bufferuart[5]=0x00;
		  bufferuart[6]=0x01;

		  
		  UARTDevice_->write_array((const uint8_t*)bufferuart,8);
		  ESP_LOGD(TAG2,"UART write: %d,", mode);
		  //turn on
		  //sprintf(bufferuart,"Set4 \x00");
		  //UARTDevice_->write_array((const uint8_t*)bufferuart,6);
	  }
	  if(mode == climate::CLIMATE_MODE_HEAT)
	  {
		  sprintf(bufferuart,"Set7 xx\n");
		  bufferuart[5]=0x00;
		  bufferuart[6]=0x00;
		  UARTDevice_->write_array((const uint8_t*)bufferuart,8);
		  ESP_LOGD(TAG2,"UART write: %d,", mode);
		  //turn on
		  //sprintf(bufferuart,"Set4 \x00");
		  //UARTDevice_->write_array((const uint8_t*)bufferuart,6);
	  }
	  
		//ESP_LOGD(TAG2,"Seeted mode: %d,", mode);
      // Publish updated state
      this->mode = mode;
      this->publish_state();
    }
    if (call.get_target_temperature().has_value()) {
      // User requested target temperature change
      float temp = *call.get_target_temperature();
      // Send target temp to climate
	  int16_t temp1 = temp*10;
	  //ESP_LOGD(TAG2,"Seeted: %ld, %04x", temp1, temp1);
	  
	  
	  sprintf(bufferuart,"Set6 00");
	  bufferuart[5]=(char)temp1;
	  bufferuart[6]=(char)(temp1>>8);
	  bufferuart[7] = '\0';
	  
	  //ESP_LOGD(TAG2,"Seeted: %02x %02x %02x %02x %02x %02x %02x %02x", bufferuart[0],bufferuart[1],bufferuart[2],bufferuart[3],bufferuart[4],bufferuart[5],bufferuart[6],bufferuart[7]);
	  
      UARTDevice_->write_array((const uint8_t*)bufferuart,7);
	  ESP_LOGD(TAG2,"UART write: %d,", temp1);
    }
  }
  ClimateTraits traits() override {
    // The capabilities of the climate device
    auto traits = climate::ClimateTraits();
    traits.set_supports_current_temperature(true);
    traits.set_supported_modes({climate::CLIMATE_MODE_HEAT});	
	traits.add_supported_mode(climate::CLIMATE_MODE_OFF);
	traits.add_supported_mode(climate::CLIMATE_MODE_COOL);
	
	
	traits.set_supports_two_point_target_temperature(false);
    return traits;
  }
  
  void set_sensor(sensor::Sensor *sensor) { this->sensor_ = sensor; }
  void set_sensor2(sensor::Sensor *sensor) { this->tempsetted_ = sensor; }
  void set_sensor3(sensor::Sensor *sensor) { this->HeatCold_ = sensor; }
  void set_sensor4(sensor::Sensor *sensor) { this->OnOff_ = sensor; }
  void set_UARTDevice(UARTComponent *parent) { this->UARTDevice_ = parent; }
  
};