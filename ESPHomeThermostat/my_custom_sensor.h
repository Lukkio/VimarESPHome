class MyCustomSensor : public PollingComponent, public Sensor {
 public:
  // constructor
  MyCustomSensor() : PollingComponent(15000) {}

  float get_setup_priority() const override { return esphome::setup_priority::XXXX; }

  void setup() override {
    // This will be called by App.setup()
  }
  void update() override {
    // This will be called every "update_interval" milliseconds.
  }
};