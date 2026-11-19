#pragma once
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  int32_t width, height;
  double K[9], kb4[4];
  int32_t orientation_degrees;
} fhm_camera;
typedef struct {
  fhm_camera views[2];
  double second_from_first_rotation[9], second_from_first_translation_m[3];
} fhm_calibration;
typedef struct {
  float position_m[3], quaternion_xyzw[4], radius_m;
  uint64_t relation_flags;
} fhm_joint;
typedef struct {
  int32_t active;
  fhm_joint joints[26];
} fhm_hand;
typedef struct {
  int64_t timestamp_ns;
  fhm_hand hands[2];
} fhm_result;
typedef struct {
  uint64_t tracked_frames[2], frames_before_display;
  int32_t this_frame_detected[2], previous_frame_detected[2], hand_seen_before[2];
  float roi_confidence[4], roi_center_px[8], roi_size_px[4];
  int32_t roi_found[4], roi_provenance[4];
  float target_hand_size_parameter;
  int32_t roi_input_found[4], keypoint_active[4], rejection_stage[2], optimizer_ran[2];
  float reprojection_error[2];
  float raw_detector_confidence[4], acquisition_confidence_average[2];
  int32_t detector_ran[2];
} fhm_diagnostics;

uint32_t fhm_abi_version(void);
const char* fhm_source_revision(void);
const char* fhm_last_error(void);
void* fhm_create(const fhm_calibration* calib, const char* models_folder);
int fhm_set_parameter(void* handle, const char* name, double value);
int fhm_get_native_second_from_first(void* handle, double matrix_rowmajor[16]);
int fhm_get_diagnostics(void* handle, fhm_diagnostics* out);
uint32_t fhm_diagnostics_abi_version(void);
const char* fhm_diagnostics_patch_sha256(void);
const char* fhm_acceptance_variant(void);
const char* fhm_acceptance_patch_sha256(void);
double fhm_get_acquisition_mean_threshold(void* handle);
int fhm_process(void* handle, const uint8_t* first, uint32_t first_stride, const uint8_t* second,
                uint32_t second_stride, int64_t timestamp_ns, fhm_result* out);
void fhm_destroy(void* handle);

#define FHM_MAX_VIEWS 4
typedef struct {
  int32_t num_views;
  fhm_camera views[FHM_MAX_VIEWS];
  double view_from_first_rotation[FHM_MAX_VIEWS][9];
  double view_from_first_translation_m[FHM_MAX_VIEWS][3];
  float boundary_circle[FHM_MAX_VIEWS][3];
} fhm_rig;
typedef struct {
  int32_t num_views, nview_policy;
  uint64_t tracked_frames[2];
  int32_t rejection_stage[2], optimizer_ran[2];
  float reprojection_error[2], target_hand_size_parameter, acquisition_confidence[2];
  int32_t view_used[2][FHM_MAX_VIEWS], roi_found[2][FHM_MAX_VIEWS], roi_provenance[2][FHM_MAX_VIEWS];
  float roi_center_px[2][FHM_MAX_VIEWS][2], roi_size_px[2][FHM_MAX_VIEWS];
  int32_t detector_ran[FHM_MAX_VIEWS];
  float detector_confidence[FHM_MAX_VIEWS][2], view_weight[FHM_MAX_VIEWS];
} fhm_diagnostics_n;
uint32_t fhm_nview_abi_version(void);
void* fhm_create_n(const fhm_rig* rig, const char* models_folder);
int fhm_process_n(void* handle, const uint8_t* const* images, const uint32_t* strides, int64_t timestamp_ns,
                  fhm_result* out);
int fhm_get_diagnostics_n(void* handle, fhm_diagnostics_n* out);
int fhm_get_native_view_from_first(void* handle, int view, double matrix_rowmajor[16]);
int fhm_set_detector_transform_n(void* handle, int num_views, const float* angles_deg, const float* crops, int enable);
int fhm_set_crop_twists_n(void* handle, int num_views, const float* values);

typedef struct {
  int32_t views_used;
  int32_t rejection_stage;
  float is_hand;
  float reprojection_error;
  float hand_confidence;
  float joint_confidence[21];
} fhm_hand_quality;
typedef struct {
  fhm_hand_quality hands[2];
  int32_t detector_ran;
  int32_t detections_suppressed;
  uint64_t frame_index;
} fhm_quality;
uint32_t fhm_quality_abi_version(void);
int fhm_get_quality(void* handle, fhm_quality* out);

int fhm_set_external_regions(void* handle, const float* values, int enable);
int fhm_external_roi_mode(void* handle);
int fhm_set_external_crop_twists(void* handle, const float* values);
const char* fhm_research_variant(void);
int fhm_set_external_policy(void* handle, int policy);
int fhm_get_external_state(void* handle, int32_t* out);
uint32_t fhm_detector_transform_abi(void);
int fhm_set_detector_transform(void* handle, const float* angles_deg, const float* crops, int enable);
#ifdef __cplusplus
}
#endif
