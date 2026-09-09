# 검증 근거와 재현 범위

현재 판정은 [진행 상황](CURRENT_STATUS.md)과 [R2 검증 기준](R2_SIM2REAL_CONTRACT.md)을 따른다. 공개 요약과 원본 실험 파일을 구분한다.

## 공개 자료

| 자료 | 내용과 해석 범위 |
|---|---|
| [R2 결과 요약](results/r2_checkpoint.json) | 2026-09-08 시험 5개의 종료 상태, 원본 결과 해시, S1 보호 입력·결과 16개의 해시. R2 미완료 상태의 요약이며 실행 가능한 checkpoint는 아님 |
| [수건 물성](../config/towel_isaac_s1_material.json) | 질량·작업대 마찰·굽힘·낙하 측정과 모델 적용 범위. 턱의 실측 압착력 검증과는 별개 |
| [작업대 보정](../ros2_ws/src/manipulation_camera_manager/config/top_worktable_homography.yaml), [카메라 구성](../ros2_ws/src/manipulation_camera_manager/config/cameras.yaml) | 등록 장치와 작업대 좌표 |
| [관절 계약](../config/so101_joint_contract.json), [운용 한계](../config/bimanual_operational_limits.json) | 명령·좌표 변환과 제한 |
| [로봇 모델](../ros2_ws/src/so101_description/README.md) | 실제 형상, 마운트, 자산 출처 |

검수 영상과 라벨은 `datasets/towel_yolo_source/`, `datasets/towel_yolo_annotations/`에, 기준 영상 모델은 `artifacts/models/towel_yolo26n_seg_expanded_r1/best.pt`에 보관한다. 이 자료는 영상 관측 기반이며 현재 S2 진단 경로의 실물 영상 제어 성공을 의미하지 않는다.

## 로컬 원본

다음 경로는 저장소 루트 기준의 로컬 보관 위치다. GitHub 배포 대상에 포함되지 않으며 공개 요약의 SHA-256으로 원본을 식별한다.

| 기준 디렉터리 | 파일 | 용도 |
|---|---|---|
| `artifacts/bimanual/planning/so101_surface_matched_pad_20260906/` | `validated_native_recipe.json`, `geometry.json` | S1 실행 입력과 2.2 mm 패드 형상 |
| 위 디렉터리 | `grasp_review/live_speed_restore/fine_compact/result.json` | S1 접힘 비교 결과 |
| 위 디렉터리 | `second_fold_native/latest_second_fold_result.json` | 현재 S2 실행·진단 인덱스 |
| `artifacts/bimanual/planning/r2_contact_foundation_20260908/` | `comparison.json`, `handoff_verification.json` | 구동·반력 비교와 보존 검사 |
| 위 디렉터리 | `plan_local_primitive.json`, `local_primitive/result.json` | 재개 계획과 최신 중단 결과 |

S2 인덱스에는 이전 U자 형성의 단면·관통 분석과 녹화 경로도 남아 있다. 이는 형성 진단이며 네 겹 하중 유지 성공은 아니다. 최신 중단 실행에는 최종 상태·녹화가 저장되지 않았다.

## 재현의 한계

S1 recipe에는 원본 작업 환경의 보정·manifest 파일을 참조하는 절대 경로가 있다. 보호 입력 15개와 결과 1개의 해시는 2026-09-09에 재확인했지만, 외부 입력 패키징은 아직 완료되지 않았다. 공개 요약의 `external_inputs/`는 외부 파일의 식별명이며 저장소 내 디렉터리가 아니다. 해시는 동일 파일인지 확인하는 수단으로, 파일 배포나 물리 검증을 대신하지 않는다.

원본 결과와 recipe는 보존하며, 공개 요약에는 필요한 상태·식별자만 추출한다. 과거 실험 도구의 기본 경로 중 일부는 현재 배포 자료에 없는 입력을 참조한다. 전체 접기 재현은 R2 G0의 남은 과제다.

## 회귀 자료

`artifacts/bimanual/planning/towel_bimanual_then_single_robot_near_to_far_r2_s0.json`과 `towel_first_fold_surface_drag_r2_s1_summary.json`은 기존 코드·설정·시험이 읽는 회귀 입력이다. 파일에 기록된 과거 전략이나 완료 표시는 현재 패드·집기 경로의 완료 판정에 사용하지 않는다.

공개 S1 회귀 요약의 결과 위치는 저장소 상대 경로로 표기한다. 일부 과거 실행은 배포 자료에 포함되지 않으며, 경로 표기 정리는 측정값이나 원본 실험 결과를 변경하지 않는다.
