// LowPassFilter.h
#ifndef LOWPASSFILTER_H
#define LOWPASSFILTER_H

#include <Arduino.h>

struct Vec3f
{
  float x;
  float y;
  float z;
};

/*
 * 1st-order IIR Low-Pass:
 * y[n] = alpha * y[n-1] + (1-alpha) * x[n]
 *
 * alpha nahe 1.0  -> starke Glättung (träge)
 * alpha nahe 0.0  -> kaum Glättung (direkt)
 */
class LowPassFilter
{
public:
  LowPassFilter();
  explicit LowPassFilter(float alpha);

  void setAlpha(float alpha);
  float getAlpha() const;

  void reset();
  void reset(float value);

  float update(float input);
  float value() const;

private:
  float m_alpha;
  float m_y;
  bool  m_initialized;
};

class LowPassFilter3
{
public:
  LowPassFilter3();
  explicit LowPassFilter3(float alpha);

  void setAlpha(float alpha);
  void reset();
  void reset(const Vec3f &value);

  Vec3f update(const Vec3f &input);
  Vec3f value() const;

private:
  LowPassFilter m_fx;
  LowPassFilter m_fy;
  LowPassFilter m_fz;
};

#endif