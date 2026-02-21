// LowPassFilter.cpp
#include "LowPassFilter.h"

static float Clamp01(float a)
{
  if (a < 0.0f) return 0.0f;
  if (a > 1.0f) return 1.0f;
  return a;
}

// -------------------- 1D --------------------

LowPassFilter::LowPassFilter()
: m_alpha(0.7f), m_y(0.0f), m_initialized(false)
{
}

LowPassFilter::LowPassFilter(float alpha)
: m_alpha(Clamp01(alpha)), m_y(0.0f), m_initialized(false)
{
}

void LowPassFilter::setAlpha(float alpha)
{
  m_alpha = Clamp01(alpha);
}

float LowPassFilter::getAlpha() const
{
  return m_alpha;
}

void LowPassFilter::reset()
{
  m_y = 0.0f;
  m_initialized = false;
}

void LowPassFilter::reset(float value)
{
  m_y = value;
  m_initialized = true;
}

float LowPassFilter::update(float input)
{
  // erstes Sample direkt übernehmen, damit es keinen Sprung gibt
  if (!m_initialized)
  {
    m_y = input;
    m_initialized = true;
    return m_y;
  }

  m_y = (m_alpha * m_y) + ((1.0f - m_alpha) * input);
  return m_y;
}

float LowPassFilter::value() const
{
  return m_y;
}

// -------------------- 3D --------------------

LowPassFilter3::LowPassFilter3()
: m_fx(0.7f), m_fy(0.7f), m_fz(0.7f)
{
}

LowPassFilter3::LowPassFilter3(float alpha)
: m_fx(alpha), m_fy(alpha), m_fz(alpha)
{
}

void LowPassFilter3::setAlpha(float alpha)
{
  m_fx.setAlpha(alpha);
  m_fy.setAlpha(alpha);
  m_fz.setAlpha(alpha);
}

void LowPassFilter3::reset()
{
  m_fx.reset();
  m_fy.reset();
  m_fz.reset();
}

void LowPassFilter3::reset(const Vec3f &value)
{
  m_fx.reset(value.x);
  m_fy.reset(value.y);
  m_fz.reset(value.z);
}

Vec3f LowPassFilter3::update(const Vec3f &input)
{
  Vec3f out;
  out.x = m_fx.update(input.x);
  out.y = m_fy.update(input.y);
  out.z = m_fz.update(input.z);
  return out;
}

Vec3f LowPassFilter3::value() const
{
  Vec3f out;
  out.x = m_fx.value();
  out.y = m_fy.value();
  out.z = m_fz.value();
  return out;
}