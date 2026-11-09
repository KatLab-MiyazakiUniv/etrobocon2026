#ifndef COLOR_SENSOR_DEBUG_H
#define COLOR_SENSOR_DEBUG_H

#include "BaseMotion.h"

class ColorSensorDebug : public BaseMotion {
 public:
  ColorSensorDebug(
      Robot& _robot,
      std::unique_ptr<BaseContinuationCondition> _continuationCondition);

  ~ColorSensorDebug() override;

 protected:
  bool canStart() override;
  void executeStep() override;
  void finish() override;
};

#endif