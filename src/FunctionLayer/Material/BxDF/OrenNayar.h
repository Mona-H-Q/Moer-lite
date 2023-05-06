#pragma once
#include "BSDF.h"
#include "Warp.h"

class OrenNayarBSDF : public BSDF {
public:
  OrenNayarBSDF(const Vector3f &_normal, const Vector3f &_tangent,
                const Vector3f &_bitangent, Spectrum _albedo, float _sigma)
      : BSDF(_normal, _tangent, _bitangent), albedo(_albedo), sigma(_sigma) {}

  virtual Spectrum f(const Vector3f &wo, const Vector3f &wi) const override {
    // TODO
    // 1. 转换坐标系到局部坐标
    // 2. 计算 A, B, \alpha, \beta（可以直接求\sin\alpha,\tan\beta）,
    // \cos(\phi_i-\phi_o)
    // 3. return Oren-Nayar brdf

    Vector3f l = normalize(toLocal(wi));
    Vector3f v = normalize(toLocal(wo));

    float sigma_2 = sigma * sigma;
    float A = 1.f - sigma_2 / (2.f * (sigma_2 + .33f));
    float B = .45f * sigma_2 / (sigma_2 + .09f);

    float cos_theta_max = l[1], cos_theta_min = v[1];
    if(cos_theta_max < cos_theta_min) 
      std::swap(cos_theta_max, cos_theta_min);
    float sin_alpha = sqrt(1.f - cos_theta_min * cos_theta_min);
    float tan_beta = sqrt(1.f / (cos_theta_max * cos_theta_max) - 1.f);

    Vector3f l_xoz = normalize(Vector3f(l[0], .0f, l[2]));
    Vector3f v_xoz = normalize(Vector3f(v[0], .0f, v[2]));
    float cos_delta_phi = dot(l_xoz, v_xoz);

    return {albedo / PI * (A + B * std::max(.0f, cos_delta_phi) * sin_alpha * tan_beta) * l[1]};
  }

  virtual BSDFSampleResult sample(const Vector3f &wo,
                                  const Vector2f &sample) const override {
    Vector3f wi = squareToCosineHemisphere(sample);
    float pdf = squareToCosineHemispherePdf(wi);
    return {albedo, toWorld(wi), pdf, BSDFType::Diffuse};
  }

private:
  Spectrum albedo;
  float sigma;
};