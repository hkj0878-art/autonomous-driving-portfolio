# TASK-05 rev2 evaluation

| Group | n | 2D ATE RMSE, SE3+lever2D (m) | Endpoint vector error, SE3 (m) | RPE 100 m, lever2D (%) | Loop consistency | MME (nats) |
|---|---:|---:|---:|---:|---:|---:|
| loop_on | 3 | 0.3059 ± 0.1141 | 0.2554 ± 0.1021 | 1.4669 ± 0.1952 | 1.0000 ± 0.0000; 7/7 pairs | -3.6964 ± 0.0130 |
| loop_off | 3 | 0.3893 ± 0.2204 | 0.3917 ± 0.2054 | 1.4878 ± 0.1964 | N/A (no factors) | -3.7208 ± 0.0191 |
| even | 1 | 0.2396 (n=1) | 0.2699 (n=1) | 1.3299 (n=1) | 1.0000 (n=1); 2/2 pairs | -3.7231 (n=1) |

TASK-06 representative run: **loop_on_r2** (median final 2D SE3+lever2D ATE among ON, n=3).
The SE3-only align_se3.json files remain the map alignment; horizontal lever fitting does not change them.

Group values are mean ± sample std (ddof=1); even has one observation and no estimated repeatability.

## Interpretation (n=3)

Loop closure changes horizontal ATE from OFF 0.3893 ± 0.2204 m to ON 0.3059 ± 0.1141 m; OFF−ON = 0.0834 m (21.4% reduction; negative means degradation).
The mean difference is smaller than the larger group std (0.2204 m). With only three runs per group, this does not establish statistical significance or generalization to another drive.
Yaw-only RPE removes the artificial mounting-pitch contribution; 100 m translation is now consistent with sub-metre ATE. The GNSS noise floor below uses ten independent 1.4 cm perturbations and matched final-pose sampling.
Lever2D estimates are fitted on the same GT used to score ATE; agreement across repeats supports a plausible mounting offset but is not independent calibration. ON/OFF variation can also absorb trajectory distortion.
Loop consistency checks the relative position vector, not an ICP rotation or an independent ground-truth loop transform. Online odometry already contains earlier loop updates, so online/final differences do not isolate a single constraint.

## Methods and limits

GNSS accepted 3298/3298. ENU origin is first fix. Status >=2 and horizontal sigma <=0.05 m; vertical sigma is recorded only.
SE3 Umeyama fits rigid alignment without scale and is unchanged; evo independently checks 3D ATE to <1 mm.
Lever2D jointly fits R_align, t and lx, ly with lz=0: p_gnss = R_align p_est + t + Rz(yaw(R_align R_est)) [lx,ly,0]. Yaw is extracted after alignment; roll/pitch including the approximately 9.3° LiDAR mounting pitch is removed. Legacy unconstrained 3-DoF fits remain in metrics.json under se3_lever_3d_diagnostic only.
Primary RPE uses the same lever2D-corrected antenna positions and joint alignment as primary ATE, with yaw-only aligned estimated rotations. SE3-only RPE is retained separately in metrics.json under rpe_se3_diagnostic (about 3–4% at 100 m): its position-only alignment is biased by the roughly 1.5 m rotating origin offset, including about 2.3° alignment yaw change when lever2D is fitted. The correction is fitted in-sample and is therefore optimistic. GT yaw is a centered 5 m trajectory tangent (roll/pitch zero, flat-ground approximation). All start poses use first endpoint >= target length and actual GT arc length; segments overlap and are correlated. Translation is reported at 10/25/50/100 m; yaw only at 25/50/100 m. Tangent yaw at corners and imperfect lever calibration remain limitations.
Endpoint vector error = norm((aligned last−first) − (GT last−first)), evaluated at each trajectory’s synchronized support; this is not merely the start–end gap.

## Trajectory metrics

| Run | Trajectory | SE3 2D ATE (m) | Lever2D 2D ATE (m) | SE3 3D ATE (m) | Lever2D 3D ATE (m) | lx / ly (m) | RPE 100 m, lever2D (%) |
|---|---|---:|---:|---:|---:|---|---:|
| loop_on_r1 | final | 0.6699 | 0.2222 | 0.6724 | 0.2295 | -1.5453 / -0.0830 | 1.2757 |
| loop_on_r1 | online | 0.7476 | 0.2903 | 0.7563 | 0.3120 | -1.6399 / -0.1793 | 1.2028 |
| loop_on_r2 | final | 0.6997 | 0.2596 | 0.7022 | 0.2662 | -1.5805 / -0.0545 | 1.4592 |
| loop_on_r2 | online | 0.7681 | 0.2920 | 0.7779 | 0.3171 | -1.7056 / -0.1604 | 1.2774 |
| loop_on_r3 | final | 0.7832 | 0.4359 | 0.7855 | 0.4400 | -1.5219 / -0.2100 | 1.6659 |
| loop_on_r3 | online | 0.7666 | 0.4120 | 0.7703 | 0.4189 | -1.4739 / -0.2624 | 1.3570 |
| loop_off_r1 | final | 0.7278 | 0.2605 | 0.7310 | 0.2694 | -1.6488 / -0.1250 | 1.2962 |
| loop_off_r1 | online | 0.7388 | 0.2651 | 0.7429 | 0.2762 | -1.6606 / -0.1289 | 1.2996 |
| loop_off_r2 | final | 0.7200 | 0.2636 | 0.7226 | 0.2705 | -1.6178 / -0.1318 | 1.4784 |
| loop_off_r2 | online | 0.7407 | 0.2683 | 0.7443 | 0.2779 | -1.6050 / -0.1457 | 1.1838 |
| loop_off_r3 | final | 0.9304 | 0.6437 | 0.9321 | 0.6461 | -1.4449 / -0.3109 | 1.6887 |
| loop_off_r3 | online | 0.8802 | 0.6279 | 0.8821 | 0.6306 | -1.3303 / -0.3085 | 1.6820 |
| even_loop_on | final | 0.6637 | 0.2396 | 0.6670 | 0.2485 | -1.5450 / -0.0616 | 1.3299 |
| even_loop_on | online | 0.7446 | 0.2599 | 0.7498 | 0.2742 | -1.6606 / -0.1321 | 1.1933 |

## Repeatability: ON/OFF mean ± sample std

| Group | Trajectory | Metric | Mean | Std | n |
|---|---|---|---:|---:|---:|
| loop_on | final | ate_se3_2d_rmse_m | 0.71762 | 0.05872 | 3 |
| loop_on | final | ate_lever_2d_rmse_m | 0.30593 | 0.11412 | 3 |
| loop_on | final | ate_se3_3d_rmse_m | 0.72003 | 0.05863 | 3 |
| loop_on | final | ate_lever_3d_rmse_m | 0.31193 | 0.11246 | 3 |
| loop_on | final | endpoint_error_m | 0.25542 | 0.10212 | 3 |
| loop_on | final | lever_norm_m | 1.55511 | 0.02352 | 3 |
| loop_on | final | lever_x_m | -1.54924 | 0.02952 | 3 |
| loop_on | final | lever_y_m | -0.11583 | 0.08277 | 3 |
| loop_on | final | rpe_10m_translation_percent | 3.03935 | 0.17888 | 3 |
| loop_on | final | rpe_25m_translation_percent | 2.19555 | 0.20160 | 3 |
| loop_on | final | rpe_25m_yaw_deg_per_100m | 7.18513 | 0.35241 | 3 |
| loop_on | final | rpe_50m_translation_percent | 2.00716 | 0.27281 | 3 |
| loop_on | final | rpe_50m_yaw_deg_per_100m | 3.17236 | 0.11383 | 3 |
| loop_on | final | rpe_100m_translation_percent | 1.46694 | 0.19519 | 3 |
| loop_on | final | rpe_100m_yaw_deg_per_100m | 1.79924 | 0.14918 | 3 |
| loop_off | final | ate_se3_2d_rmse_m | 0.79275 | 0.11931 | 3 |
| loop_off | final | ate_lever_2d_rmse_m | 0.38930 | 0.22037 | 3 |
| loop_off | final | ate_se3_3d_rmse_m | 0.79521 | 0.11859 | 3 |
| loop_off | final | ate_lever_3d_rmse_m | 0.39532 | 0.21716 | 3 |
| loop_off | final | endpoint_error_m | 0.39167 | 0.20543 | 3 |
| loop_off | final | lever_norm_m | 1.58491 | 0.09383 | 3 |
| loop_off | final | lever_x_m | -1.57052 | 0.10986 | 3 |
| loop_off | final | lever_y_m | -0.18922 | 0.10540 | 3 |
| loop_off | final | rpe_10m_translation_percent | 3.15284 | 0.58276 | 3 |
| loop_off | final | rpe_25m_translation_percent | 2.30635 | 0.46810 | 3 |
| loop_off | final | rpe_25m_yaw_deg_per_100m | 6.70621 | 0.59309 | 3 |
| loop_off | final | rpe_50m_translation_percent | 2.06902 | 0.44777 | 3 |
| loop_off | final | rpe_50m_yaw_deg_per_100m | 3.34516 | 0.57369 | 3 |
| loop_off | final | rpe_100m_translation_percent | 1.48776 | 0.19644 | 3 |
| loop_off | final | rpe_100m_yaw_deg_per_100m | 1.90598 | 0.25911 | 3 |
| loop_on | online | ate_se3_2d_rmse_m | 0.76077 | 0.01141 | 3 |
| loop_on | online | ate_lever_2d_rmse_m | 0.33145 | 0.06974 | 3 |
| loop_on | online | ate_se3_3d_rmse_m | 0.76820 | 0.01098 | 3 |
| loop_on | online | ate_lever_3d_rmse_m | 0.34933 | 0.06032 | 3 |
| loop_on | online | endpoint_error_m | 0.18581 | 0.06277 | 3 |
| loop_on | online | lever_norm_m | 1.61996 | 0.11107 | 3 |
| loop_on | online | lever_x_m | -1.60647 | 0.11943 | 3 |
| loop_on | online | lever_y_m | -0.20067 | 0.05428 | 3 |
| loop_on | online | rpe_10m_translation_percent | 2.96353 | 0.35049 | 3 |
| loop_on | online | rpe_25m_translation_percent | 2.04110 | 0.23377 | 3 |
| loop_on | online | rpe_25m_yaw_deg_per_100m | 5.99316 | 0.21049 | 3 |
| loop_on | online | rpe_50m_translation_percent | 1.76605 | 0.21229 | 3 |
| loop_on | online | rpe_50m_yaw_deg_per_100m | 2.81175 | 0.12302 | 3 |
| loop_on | online | rpe_100m_translation_percent | 1.27904 | 0.07713 | 3 |
| loop_on | online | rpe_100m_yaw_deg_per_100m | 1.55008 | 0.07866 | 3 |
| loop_off | online | ate_se3_2d_rmse_m | 0.78659 | 0.08109 | 3 |
| loop_off | online | ate_lever_2d_rmse_m | 0.38707 | 0.20855 | 3 |
| loop_off | online | ate_se3_3d_rmse_m | 0.78976 | 0.08000 | 3 |
| loop_off | online | ate_lever_3d_rmse_m | 0.39488 | 0.20412 | 3 |
| loop_off | online | endpoint_error_m | 0.39411 | 0.19669 | 3 |
| loop_off | online | lever_norm_m | 1.54759 | 0.15989 | 3 |
| loop_off | online | lever_x_m | -1.53195 | 0.17683 | 3 |
| loop_off | online | lever_y_m | -0.19438 | 0.09922 | 3 |
| loop_off | online | rpe_10m_translation_percent | 3.09933 | 0.76171 | 3 |
| loop_off | online | rpe_25m_translation_percent | 2.24758 | 0.52571 | 3 |
| loop_off | online | rpe_25m_yaw_deg_per_100m | 6.14478 | 0.16277 | 3 |
| loop_off | online | rpe_50m_translation_percent | 1.92408 | 0.41045 | 3 |
| loop_off | online | rpe_50m_yaw_deg_per_100m | 2.90923 | 0.22331 | 3 |
| loop_off | online | rpe_100m_translation_percent | 1.38848 | 0.26074 | 3 |
| loop_off | online | rpe_100m_yaw_deg_per_100m | 1.66575 | 0.15593 | 3 |
| loop_on | map | mme_nats | -3.69639 | 0.01300 | 3 |
| loop_on | map | mpv_m2 | 0.00175 | 0.00005 | 3 |
| loop_on | map | eligible_fraction | 0.96691 | 0.00252 | 3 |
| loop_on | map | overlap_median_m | 0.12436 | 0.02900 | 3 |
| loop_on | map | overlap_p95_m | 0.72315 | 0.00975 | 3 |
| loop_on | map | double_wall_ratio | 0.34541 | 0.06719 | 3 |
| loop_on | map | A_overlap_fraction | 0.65198 | 0.00680 | 3 |
| loop_on | map | B_overlap_fraction | 0.52816 | 0.00459 | 3 |
| loop_off | map | mme_nats | -3.72083 | 0.01909 | 3 |
| loop_off | map | mpv_m2 | 0.00174 | 0.00002 | 3 |
| loop_off | map | eligible_fraction | 0.96680 | 0.00157 | 3 |
| loop_off | map | overlap_median_m | 0.12233 | 0.02260 | 3 |
| loop_off | map | overlap_p95_m | 0.74595 | 0.01547 | 3 |
| loop_off | map | double_wall_ratio | 0.35815 | 0.03826 | 3 |
| loop_off | map | A_overlap_fraction | 0.66814 | 0.01048 | 3 |
| loop_off | map | B_overlap_fraction | 0.53303 | 0.01074 | 3 |

## Lever2D consistency

| Population | n | lx (m) | ly (m) | Norm (m) |
|---|---:|---:|---:|---:|
| loop_on / final | 3 | -1.5492 ± 0.0295 | -0.1158 ± 0.0828 | 1.5551 ± 0.0235 |
| loop_off / final | 3 | -1.5705 ± 0.1099 | -0.1892 ± 0.1054 | 1.5849 ± 0.0938 |
| all seven / final | 7 | -1.5578 ± 0.0668 | -0.1395 ± 0.0923 | 1.5666 ± 0.0585 |
| loop_on / online | 3 | -1.6065 ± 0.1194 | -0.2007 ± 0.0543 | 1.6200 ± 0.1111 |
| loop_off / online | 3 | -1.5320 ± 0.1768 | -0.1944 ± 0.0992 | 1.5476 ± 0.1599 |
| all seven / online | 7 | -1.5823 ± 0.1333 | -0.1882 ± 0.0699 | 1.5955 ± 0.1221 |

These are yaw-only ENU heading body offsets with lz fixed at zero, not the tilted LiDAR-frame x/z components. Per-run fits (including even) are listed above. Across-all-run std mixes loop modes and the single even trial; use within-group std to assess repeatability.

## RPE GNSS noise floor

Clean GT and GT + isotropic Gaussian position noise σ=0.014 m, 10 trials, seed 20261026. Reference is clean matched GT for each run. No alignment or lever fit; noisy estimate yaw is derived from its own 5 m tangent. Values are mean ± sample std of ten trial means, not of overlapping segments. All clean-GT translation/yaw means are numerically zero. This tests metric sensitivity at final-keyframe sampling, not a complete GNSS error model.

| Run | Segment (m) | Translation noise (%) | Yaw noise (deg/100 m) |
|---|---:|---:|---:|
| loop_on_r1 | 10 | 0.4623 ± 0.0873 | — (not reported) |
| loop_on_r1 | 25 | 0.3421 ± 0.0908 | 1.0686 ± 0.3623 |
| loop_on_r1 | 50 | 0.2939 ± 0.0934 | 0.5448 ± 0.1867 |
| loop_on_r1 | 100 | 0.2092 ± 0.0385 | 0.2500 ± 0.0715 |
| loop_on_r2 | 10 | 0.4576 ± 0.0840 | — (not reported) |
| loop_on_r2 | 25 | 0.3447 ± 0.0869 | 1.0022 ± 0.2956 |
| loop_on_r2 | 50 | 0.3077 ± 0.0991 | 0.5524 ± 0.1960 |
| loop_on_r2 | 100 | 0.2213 ± 0.0643 | 0.2354 ± 0.0432 |
| loop_on_r3 | 10 | 0.4968 ± 0.0891 | — (not reported) |
| loop_on_r3 | 25 | 0.3890 ± 0.0916 | 1.3886 ± 0.4594 |
| loop_on_r3 | 50 | 0.3478 ± 0.0956 | 0.5966 ± 0.1297 |
| loop_on_r3 | 100 | 0.2629 ± 0.1271 | 0.3106 ± 0.0915 |
| loop_off_r1 | 10 | 0.4295 ± 0.0310 | — (not reported) |
| loop_off_r1 | 25 | 0.2984 ± 0.0181 | 0.9159 ± 0.0603 |
| loop_off_r1 | 50 | 0.2478 ± 0.0138 | 0.4719 ± 0.0592 |
| loop_off_r1 | 100 | 0.1908 ± 0.0174 | 0.2362 ± 0.0277 |
| loop_off_r2 | 10 | 0.4479 ± 0.0582 | — (not reported) |
| loop_off_r2 | 25 | 0.3088 ± 0.0472 | 1.0015 ± 0.2150 |
| loop_off_r2 | 50 | 0.2678 ± 0.0515 | 0.4809 ± 0.1048 |
| loop_off_r2 | 100 | 0.2092 ± 0.0588 | 0.2557 ± 0.0704 |
| loop_off_r3 | 10 | 0.4104 ± 0.0252 | — (not reported) |
| loop_off_r3 | 25 | 0.2867 ± 0.0214 | 0.8361 ± 0.0968 |
| loop_off_r3 | 50 | 0.2432 ± 0.0297 | 0.4124 ± 0.0605 |
| loop_off_r3 | 100 | 0.1711 ± 0.0132 | 0.2095 ± 0.0229 |
| even_loop_on | 10 | 0.5129 ± 0.1105 | — (not reported) |
| even_loop_on | 25 | 0.4047 ± 0.1233 | 1.3353 ± 0.6366 |
| even_loop_on | 50 | 0.3747 ± 0.1369 | 0.7267 ± 0.3456 |
| even_loop_on | 100 | 0.2031 ± 0.0167 | 0.2995 ± 0.1015 |

## Loop constraint consistency

| Run | Current / previous keyframe | GT horizontal distance (m), reference only | Relative position error, 3D (m) | Verdict |
|---|---|---:|---:|---|
| loop_on_r1 | 156 / 0 | 1.0243 | 0.2726 | CONSISTENT(TP) |
| loop_on_r1 | 157 / 0 | 0.1429 | 0.2207 | CONSISTENT(TP) |
| loop_on_r2 | 148 / 0 | 2.5094 | 0.4019 | CONSISTENT(TP) |
| loop_on_r2 | 149 / 0 | 1.3718 | 0.3025 | CONSISTENT(TP) |
| loop_on_r2 | 150 / 0 | 0.2073 | 0.1752 | CONSISTENT(TP) |
| loop_on_r3 | 165 / 0 | 1.7240 | 0.3340 | CONSISTENT(TP) |
| loop_on_r3 | 166 / 0 | 0.5395 | 0.3704 | CONSISTENT(TP) |
| loop_off_r1 | None / None | — | — | NOT_APPLICABLE |
| loop_off_r2 | None / None | — | — | NOT_APPLICABLE |
| loop_off_r3 | None / None | — | — | NOT_APPLICABLE |
| even_loop_on | 164 / 0 | 2.3032 | 0.4924 | CONSISTENT(TP) |
| even_loop_on | 165 / 0 | 0.2887 | 0.2699 | CONSISTENT(TP) |

After unchanged SE3 alignment, Δ = (p_i−p_j) − (g_i−g_j), with GT interpolated at the two final keyframe times. ||Δ|| <0.5 m is CONSISTENT(TP); otherwise INCONSISTENT. GT distance is reference only. For pairs traversed in the same direction the antenna lever effect approximately cancels. Marker matching must be <=0.2 m. This checks position consistency only.

## Preserved map quality

| Run | MME (nats) | MPV (m²) | Overlap median (m) | p95 (m) | >0.2 m ratio |
|---|---:|---:|---:|---:|---:|
| loop_on_r1 | -3.69387 | 0.0017816 | 0.1485 | 0.7336 | 0.403 |
| loop_on_r2 | -3.68483 | 0.0017841 | 0.1324 | 0.7144 | 0.361 |
| loop_on_r3 | -3.71046 | 0.0016980 | 0.0922 | 0.7214 | 0.272 |
| loop_off_r1 | -3.74093 | 0.0017292 | 0.1484 | 0.7627 | 0.402 |
| loop_off_r2 | -3.71860 | 0.0017734 | 0.1110 | 0.7428 | 0.331 |
| loop_off_r3 | -3.70295 | 0.0017321 | 0.1077 | 0.7323 | 0.342 |
| even_loop_on | -3.72307 | 0.0017170 | 0.1087 | 0.7146 | 0.291 |

MME/MPV use the same 0.1 m ENU voxel, 0.3 m radius, >=6 neighbors and covariance eigenvalue floor 1e-9 m², cropped to GNSS XY bounds with unbounded height. Existing map metrics and first/last 15 s overlap clouds are reused because source maps and SE3 alignment are unchanged. Overlap uses both NN directions gated at 1 m; coverage is in map_quality.csv. No deskew. Even is not held-out localization TASK-05b.

v3 provenance: two divergent even attempts remain archived under results/archive_v3_failed_attempts; canonical even used sensor readiness and DDS discovery delay with unchanged params/rate. No experiment replay in rev2.

Maximum evo ATE difference: 5.33e-15 m.

## Verification status

Independent evaluation audit: PASS; all 12 figures directly inspected. Evaluation tests: 16 passed. Full tools pytest: **59 passed, 1 failed** in the out-of-scope legacy TASK-06 test `test_nav2_roundtrip_and_spec_threshold_conflict`, which still requires the auxiliary `map2d_nav2.yaml` removed by TASK-06 rev1. Thus the whole-suite completion criterion is not met. Details: final_audit.json, pytest_all_rev2.txt, task05_rev2.md. The prior rev1 audit (including superseded GT-distance FP labels) is preserved under archive_rev1_audit/.
