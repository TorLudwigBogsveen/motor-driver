// # Copyright (c) 2025 MDU Solar Team

#include "input.hpp"
#include <math.h>

constexpr float EMA_ALPHA = 0.1f;

void Buttons::update()
{
  buttonsJustChanged = 0;
}

void Buttons::set(uint16_t buttonMask, bool state)
{
  uint16_t newButtons;
  // Update buttons state based on the requested state
  if (state)
  {
    newButtons = buttons | buttonMask; // Set the button(s) as pressed
  }
  else
  {
    newButtons = buttons & ~buttonMask; // Set the button(s) as released
  }

  buttonsJustChanged |= buttons ^ newButtons;
  buttons = newButtons;
}

uint16_t Buttons::get(uint16_t buttonMask) const
{
  return buttons & buttonMask;
}

uint16_t Buttons::getJustChanged(uint16_t buttonMask) const
{
  return buttonsJustChanged & buttonMask;
}

uint16_t Buttons::getJustPressed(uint16_t buttonMask) const
{
  return getJustChanged(buttonMask) & get(buttonMask);
}

uint16_t Buttons::getJustReleased(uint16_t buttonMask) const
{
  return getJustChanged(buttonMask) & ~get(buttonMask);
}

void SmoothSliders::set(int slider, float value)
{
  float emaCandidate = EMA_ALPHA * value + (1 - EMA_ALPHA) * sliders[slider];
  sliders[slider] = emaCandidate;
}

float SmoothSliders::getFloat(int slider) const
{
  return sliders[slider];
}