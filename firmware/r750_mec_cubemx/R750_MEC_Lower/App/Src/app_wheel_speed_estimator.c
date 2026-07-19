#include "app_wheel_speed_estimator.h"

#include <limits.h>
#include <stddef.h>
#include <string.h>

static uint32_t AppWheelSpeed_AbsI16(int16_t value)
{
  return value < 0 ? (uint32_t)(-(int32_t)value) : (uint32_t)value;
}

static bool AppWheelSpeed_SignsCompatible(int32_t m_delta_sum, float t_speed_rpm)
{
  return m_delta_sum == 0 ||
         (m_delta_sum > 0 && t_speed_rpm > 0.0f) ||
         (m_delta_sum < 0 && t_speed_rpm < 0.0f);
}

bool AppWheelSpeedEstimator_Init(
  AppWheelSpeedEstimator *estimator,
  const AppWheelSpeedEstimatorConfig *config)
{
  if (estimator == NULL || config == NULL ||
      config->encoder_counts_per_revolution == 0U ||
      config->control_rate_hz == 0U ||
      config->timestamp_clock_hz == 0U ||
      config->period_events_per_revolution == 0U ||
      config->t_stale_timeout_cycles == 0U ||
      config->m_window_ticks == 0U ||
      config->m_window_ticks > APP_WHEEL_SPEED_M_WINDOW_MAX_TICKS ||
      config->switch_to_t_max_activity_counts >=
        config->switch_to_m_min_activity_counts) {
    return false;
  }

  memset(estimator, 0, sizeof(*estimator));
  estimator->config = *config;
  return true;
}

bool AppWheelSpeedEstimator_Update(
  AppWheelSpeedEstimator *estimator,
  const AppWheelSpeedEstimatorInput *input,
  AppWheelSpeedEstimatorOutput *output)
{
  if (estimator == NULL || input == NULL || output == NULL) {
    return false;
  }

  const uint8_t ring_index = estimator->m_ring_index;
  const int16_t replaced_delta = estimator->m_delta_ring[ring_index];
  estimator->m_delta_sum -= (int32_t)replaced_delta;
  estimator->m_activity_sum -= AppWheelSpeed_AbsI16(replaced_delta);
  estimator->m_delta_ring[ring_index] = input->encoder_delta;
  estimator->m_delta_sum += (int32_t)input->encoder_delta;
  const uint32_t added_activity = AppWheelSpeed_AbsI16(input->encoder_delta);
  if (added_activity > (UINT32_MAX - estimator->m_activity_sum)) {
    estimator->m_activity_sum = UINT32_MAX;
  } else {
    estimator->m_activity_sum += added_activity;
  }
  estimator->m_ring_index++;
  if (estimator->m_ring_index >= estimator->config.m_window_ticks) {
    estimator->m_ring_index = 0U;
  }
  if (estimator->m_fill_count < estimator->config.m_window_ticks) {
    estimator->m_fill_count++;
  }

  const bool m_valid =
    estimator->m_fill_count == estimator->config.m_window_ticks;
  float m_speed_rpm = 0.0f;
  if (m_valid) {
    m_speed_rpm =
      (60.0f * (float)estimator->config.control_rate_hz *
       (float)estimator->m_delta_sum) /
      ((float)estimator->config.encoder_counts_per_revolution *
       (float)estimator->config.m_window_ticks);
  }

  const bool direction_reset =
    (input->period.flags & APP_ENCODER_PERIOD_FLAG_DIRECTION_RESET) != 0U;
  const bool aggregate_dropped =
    (input->period.flags & APP_ENCODER_PERIOD_FLAG_AGGREGATE_DROPPED) != 0U;
  if (direction_reset || aggregate_dropped) {
    estimator->t_valid = false;
  }

  bool t_updated = false;
  if (!direction_reset && !aggregate_dropped &&
      input->period.period_count > 0U &&
      input->period.period_sum_cycles > 0U &&
      (input->period.direction == -1 || input->period.direction == 1)) {
    const float magnitude_rpm =
      (60.0f * (float)estimator->config.timestamp_clock_hz *
       (float)input->period.period_count) /
      ((float)estimator->config.period_events_per_revolution *
       (float)input->period.period_sum_cycles);
    estimator->last_t_speed_rpm =
      input->period.direction < 0 ? -magnitude_rpm : magnitude_rpm;
    estimator->t_valid = true;
    t_updated = true;
  }

  const bool has_edge =
    (input->period.flags & APP_ENCODER_PERIOD_FLAG_HAS_EDGE) != 0U;
  const bool t_stale =
    !has_edge ||
    input->period.last_edge_age_cycles >
      estimator->config.t_stale_timeout_cycles;
  const bool t_valid =
    estimator->t_valid && !t_stale &&
    AppWheelSpeed_SignsCompatible(
      estimator->m_delta_sum, estimator->last_t_speed_rpm);

  if (estimator->source == APP_WHEEL_SPEED_SOURCE_M) {
    if (t_valid &&
        estimator->m_activity_sum <=
          estimator->config.switch_to_t_max_activity_counts) {
      estimator->source = APP_WHEEL_SPEED_SOURCE_T;
    } else if (!m_valid && t_valid) {
      estimator->source = APP_WHEEL_SPEED_SOURCE_T;
    } else if (!m_valid) {
      estimator->source = APP_WHEEL_SPEED_SOURCE_NONE;
    }
  } else if (estimator->source == APP_WHEEL_SPEED_SOURCE_T) {
    if (!t_valid ||
        estimator->m_activity_sum >=
          estimator->config.switch_to_m_min_activity_counts) {
      estimator->source =
        m_valid ? APP_WHEEL_SPEED_SOURCE_M : APP_WHEEL_SPEED_SOURCE_NONE;
    }
  } else if (m_valid &&
             estimator->m_activity_sum >=
               estimator->config.switch_to_m_min_activity_counts) {
    estimator->source = APP_WHEEL_SPEED_SOURCE_M;
  } else if (t_valid) {
    estimator->source = APP_WHEEL_SPEED_SOURCE_T;
  } else if (m_valid) {
    estimator->source = APP_WHEEL_SPEED_SOURCE_M;
  }

  memset(output, 0, sizeof(*output));
  output->m_speed_rpm = m_speed_rpm;
  output->t_speed_rpm = estimator->last_t_speed_rpm;
  output->m_activity_counts = estimator->m_activity_sum;
  output->source = estimator->source;
  output->m_valid = m_valid;
  output->t_valid = t_valid;
  output->t_updated = t_updated;
  output->t_stale = t_stale;
  if (estimator->source == APP_WHEEL_SPEED_SOURCE_M && m_valid) {
    output->speed_rpm = m_speed_rpm;
    output->valid = true;
  } else if (estimator->source == APP_WHEEL_SPEED_SOURCE_T && t_valid) {
    output->speed_rpm = estimator->last_t_speed_rpm;
    output->valid = true;
  }
  return true;
}
