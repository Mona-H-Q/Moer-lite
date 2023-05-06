#pragma once

#include "NDF.h"

class BeckmannDistribution : public NDF {
public:
  BeckmannDistribution() noexcept = default;
  virtual ~BeckmannDistribution() noexcept = default;
  virtual float getD(const Vector3f &whLocal,
                     const Vector2f &alpha) const noexcept override {
    // TODO
    // 根据公式即可
    Vector3f h = normalize(whLocal);
    float cos2_theta = h[1] * h[1];
    float cos4_theta = cos2_theta * cos2_theta;
    float tan2_theta = (1.f - cos2_theta) / cos2_theta;
    float alpha2 = alpha[0] * alpha[0];
    return exp(-tan2_theta / alpha2) / (PI * alpha2 * cos4_theta);
  }
  // tips:
  // float getG1(...) {}
  float getG1(const Vector3f &w, const float &alpha) const {
    Vector3f v = normalize(w);
    float cos_theta_v = v[1];
    float tan_theta_v = sqrt(1.f / (cos_theta_v * cos_theta_v) - 1.f);
    float a = 1.f / (alpha * tan_theta_v);
    if(a < 1.6f) return (3.535f * a + 2.181f * a * a) / (1.f + 2.276f * a + 2.577f * a * a);
    return 1.f;
  }
  virtual float getG(const Vector3f &woLocal, const Vector3f &wiLocal,
                     const Vector2f &alpha) const noexcept override {
    // TODO
    // 根据公式即可
    // tips: return getG1(wo) * getG1(wi);
    return getG1(woLocal, alpha[0]) * getG1(wiLocal, alpha[0]);
  }
  virtual float pdf(const Vector3f &woLocal, const Vector3f &whLocal,
                    const Vector2f &alpha) const noexcept override {
    return getD(whLocal, alpha) * whLocal[1];
  }
  virtual Vector3f sampleWh(const Vector3f &woLocal, const Vector2f &alpha,
                            const Vector2f &sample) const noexcept override {
    float a = alpha[0];
    float tan_theta_2 = -std::log(1 - sample[0]) * a * a;
    float phi = sample[1] * 2 * PI;

    float cos_theta = std::sqrt(1.f / (1.f + tan_theta_2));
    float sin_theta = std::sqrt(1.f - cos_theta * cos_theta);
    return {sin_theta * std::cos(phi), sin_theta * std::sin(phi), cos_theta};
  }
};