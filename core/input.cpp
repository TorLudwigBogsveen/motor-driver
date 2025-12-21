//# Copyright (c) 2025 MDU Solar Team

#include "input.hpp"
#include <math.h>

constexpr float POT_DEAD_ZONE_THRESHOLD = 5.0f;
constexpr float EMA_ALPHA = 0.1f;

void Buttons::update() {
  buttonsJustChanged = 0;
}

void Buttons::set(uint16_t buttonMask, bool state) {
  uint16_t newButtons;
  // Update buttons state based on the requested state
  if (state) {
    newButtons = buttons | buttonMask;  // Set the button(s) as pressed
  } else {
    newButtons = buttons & ~buttonMask;  // Set the button(s) as released
  }


  buttonsJustChanged |= buttons ^ newButtons;
  buttons = newButtons;
}


uint16_t Buttons::get(uint16_t buttonMask) const {
  return buttons & buttonMask;
}

uint16_t Buttons::getJustChanged(uint16_t buttonMask) const {
  return buttonsJustChanged & buttonMask;
}

uint16_t Buttons::getJustPressed(uint16_t buttonMask) const {
  return getJustChanged(buttonMask) & get(buttonMask);
}

uint16_t Buttons::getJustReleased(uint16_t buttonMask) const {
  return getJustChanged(buttonMask) & ~get(buttonMask);
}

void Sliders::set(int slider, uint16_t value) {
  sliders[slider] = value;
}

uint16_t Sliders::get(int slider) const {
  return sliders[slider];
}

void SmoothSliders::set(int slider, uint16_t value) {
  float emaCandidate = EMA_ALPHA * value + (1 - EMA_ALPHA) * sliders[slider];
  if (fabs(emaCandidate - sliders[slider]) > POT_DEAD_ZONE_THRESHOLD) {
    sliders[slider] = emaCandidate;
  }
}

uint16_t SmoothSliders::get(int slider) const {
  float val = fmax(sliders[slider] - 100.0f, 0.0f);
  return uint16_t(val);
}

float SmoothSliders::getFloat(int slider) const {
  float val = fmax(sliders[slider] - 100.0f, 0.0f);
  return sliders[slider];
}