//# Copyright (c) 2025 MDU Solar Team
#ifndef INPUT_HPP
#define INPUT_HPP

#include <stdint.h>
#define NUM_POTENTIOMETERS 2

class Buttons {
public:
  Buttons(): buttons(0), buttonsJustChanged(0) {}
  void update();
  void set(uint16_t buttonMask, bool state);
  uint16_t get(uint16_t buttonMask) const;
  uint16_t getJustChanged(uint16_t buttonMask) const;
  uint16_t getJustPressed(uint16_t buttonMask) const;
  uint16_t getJustReleased(uint16_t buttonMask) const;
private:
  uint16_t buttons;
  uint16_t buttonsJustChanged;
};

class SmoothSliders {
public:
  SmoothSliders(): sliders {} {}
  void set(int slider, float value);
  float getFloat(int slider) const;

  private:
    float sliders[NUM_POTENTIOMETERS];
};

#endif