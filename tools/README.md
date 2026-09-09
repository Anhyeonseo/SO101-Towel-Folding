# 도구 사용

저장소 루트에서 실행한다. 기본 Python 의존성은 [requirements](../requirements/README.md), 현재 검증 범위는 [진행 상황](../docs/CURRENT_STATUS.md)을 따른다.

## 구성

| 경로 | 용도 |
|---|---|
| `run/` | 태스크 계획, 데이터 처리, 검증 진입점 |
| `lib/` | 기하·계획·관측·구동·판정 공통 코드 |
| `setup/camera_calibration/` | 표적 생성, 캡처, 카메라 보정 |
| `setup/can_perception/` | 선행 캔 데이터와 그리퍼 실측 |
| `setup/firmware/` | 프로토콜 헤더와 펌웨어 검사 |
| `setup/isaac/` | 작업셀, 강체 회귀 검사, 수건 물리 실행 |
| `setup/resident_gate/` | 양팔 실행 브리지의 단계별 검증 |
| `diagnostics/`, `contract_evidence/` | 상태 진단과 제어 계약 근거 수집 |

## 모터 없는 계약·계획 확인

```bash
python tools/run/validate_protocol_manifest.py
python tools/run/validate_camera_schedule.py
python tools/run/validate_towel_contract.py
python tools/run/validate_towel_schemas.py
python tools/run/validate_towel_dataset.py config/towel_annotation.example.json --output tmp/towel_dataset_manifest.json
python tools/run/plan_towel_task_once.py config/towel_observation.example.json --output tmp/towel_plan.json
python tools/run/replay_towel_task.py config/towel_replay.example.json --output tmp/towel_replay.json
```

예제 관측과 replay는 상태·계획 계약을 검사한다. 실제 카메라나 수건 물리의 성공을 검증하는 실행은 아니다.

## 수건 개발 진입점

| 작업 | 파일 | 범위 |
|---|---|---|
| 작업셀 경로 계획 | [plan_towel_fold_sequence_once.py](run/plan_towel_fold_sequence_once.py) | 등록 자산, 5축 IK, MoveIt 검사; 로컬 보정 입력 필요 |
| 영상 수집 | [capture_towel_yolo_interactive.py](run/capture_towel_yolo_interactive.py) | 재배치별 episode와 프레임 기록 |
| 검수 데이터 내보내기 | [export_towel_yolo_segmentation.py](run/export_towel_yolo_segmentation.py) | 승인된 라벨과 split·해시 검사 |
| 영상 모델 평가 | [evaluate_towel_yolo_segmentation.py](run/evaluate_towel_yolo_segmentation.py) | 검출·빈 장면 거절·mask IoU |
| 물성 보정 | [run_towel_newton_material_calibration.py](setup/isaac/run_towel_newton_material_calibration.py) | 실측과 시뮬 반응 비교 |
| S2 양팔 계획 | [plan_native_bimanual_second_fold.py](run/plan_native_bimanual_second_fold.py) | 국소 접근·집기 후보 생성 |
| S2 물리 진단 | [run_native_bimanual_second_fold.py](run/run_native_bimanual_second_fold.py) | 위치 목표 구동과 접촉·들기·놓기 진단 |
| 패드 관통 검사 | [audit_r2_pad_intersection.py](run/audit_r2_pad_intersection.py) | 저장 수건 면과 패드 기하 검사 |

관통 검사와 정적 경로 표본은 전체 연속 동작의 무충돌 보증이 아니다. 최신 물리 시험은 중단 상태이며 실행 입력과 재현 제한은 [시뮬레이션 안내](../isaac_sim/README.md)를 따른다. 과거 실험용 도구의 기본 입력은 공개 저장소에 없는 파일을 참조할 수 있다.

영상 라벨은 검수 전 학습에 사용하지 않는다. 가림 장면은 거절 평가로 구분하고, 같은 배치의 프레임이 학습과 평가에 나뉘지 않도록 episode 단위 split을 유지한다. 검출·mask 성능만으로 숨은 겹수나 접기 완료를 승인하지 않는다.

## 선행 캔 집기

[can_pick_contract.candidate.json](../config/can_pick_contract.candidate.json), [can_pick_application.py](lib/can_pick_application.py), [plan_can_pick_left_once.py](run/plan_can_pick_left_once.py), [run_can_pick_left_once.py](run/run_can_pick_left_once.py)는 왼팔 캔 집기 후보와 공통 기하 검증용이다. 수거함까지 운반하고 놓는 단계는 구현되지 않았다.

완전히 보이는 캔 한 개를 대상으로 작업대·도달 영역을 검사하고, 캔 장축과 직교하는 턱 방향 및 손목 회전 분기를 포함해 5축 자세를 푼다. MoveIt 구간 검사, 실측 jaw mapping, 계획·보정 해시 검증이 필요하다.

승격 순서는 카메라 보정→jaw mapping→plan-only→validate-only→집기 높이의 개방 자세 확인→감독하 단발 집기→제한 반복이다. 수거함 단계에는 개구부·충돌 형상, 운반 중 충돌 범위, 놓기·재인식과 실패 시 유지/퇴피 정책이 추가로 필요하다.

실제 모터 실행에는 도구별 전원·확인 조건과 별도 실행 승인이 필요하다. 시뮬레이션 결과나 plan-only 통과로 이를 대체하지 않는다.
