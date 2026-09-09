# SO101 Towel Folding

> **Frozen development snapshot — 2026-09-09.** This repository preserves the towel-folding work at the R2 checkpoint. Physical grasp validation, continuous two-fold execution, and sim-to-real transfer remain incomplete. Ongoing project development continues in [Bimanual-Household-Manipulation](https://github.com/Anhyeonseo/Bimanual-Household-Manipulation).

SO-ARM101 두 대와 상단·손목 카메라로 수건을 펼치고 직교 방향으로 두 번 접는 심투리얼 프로젝트다. Raspberry Pi 5의 ROS 2 Jazzy와 STM32G474 기반 제어기를 사용한다.

**현재 R2 진행 중이며 실험은 일시 중단 상태다.** 작업셀·통신·영상 관측 기반과 1차 접기 시뮬레이션 비교 결과를 확보했다. 실물과 대응되는 집기·접촉, 연속 두 번 접기와 실물 전이는 아직 검증되지 않았다. [현재 진행 상황](docs/CURRENT_STATUS.md)에서 구현 범위와 남은 문제를 확인할 수 있다.

## 목표 동작

```text
카메라 관측 → 펼치기 → 평탄화·정렬 → 1차 접기
           → 중간 관측·보정 → 직교 2차 접기 → 최종 검사
```

대상은 작업대 안의 수건 한 장으로, 명목 크기는 300×300 mm다. 각 조작 후 손을 치우고 다시 관측하며, 가림·미끄러짐·형상 오류에는 횟수와 시간이 제한된 복구를 적용한다. 여러 장의 얽힘, 매듭, 다른 물체 아래에 낀 수건은 초기 범위에서 제외한다.

최종 목표는 시뮬레이션의 관측·행동 계약과 학습 정책을 실제 로봇으로 옮겨 검증하는 것이다. [수건 조작 설계](docs/TOWEL_FOLDING.md)에 대상 범위와 최종 품질 기준을 정리했다.

## 하드웨어와 소프트웨어

| 구성 | 역할 |
|---|---|
| SO-ARM101 양팔, 2.2 mm 패드 | 집기·펼치기·접기 |
| 상단 카메라 1개, 손목 카메라 2개 | 수건 형상과 접근 영역 관측 |
| Raspberry Pi 5 / ROS 2 Jazzy | 카메라 수집, 인식, 계획, 실행 관리 |
| STM32G474 | 모터 명령·피드백, 통신 및 정지 처리 |
| Isaac Sim / Newton | 접촉 진단, 조작 개발, 심투리얼 평가 기반 |

## 빠른 확인

저장소 루트에서 실행한다. 아래 명령은 로컬 계약 검사이며 모터나 시뮬레이션을 실행하지 않는다.

```bash
python3 -m venv .venv-host
source .venv-host/bin/activate
python -m pip install -r requirements/host.txt
python tools/run/validate_protocol_manifest.py
python tools/run/validate_camera_schedule.py
python tools/run/validate_towel_contract.py
python tools/run/validate_towel_schemas.py
```

기하·상태·계획 계약의 단위 시험:

```bash
python -m pytest -c config/pytest.ini --rootdir=. -q \
  tests/test_towel_geometry.py \
  tests/test_towel_fold_path.py \
  tests/test_towel_task_runtime.py \
  tests/test_towel_task_planning.py \
  tests/test_towel_task_replay.py \
  tests/test_towel_schemas.py
```

ROS 2·MoveIt·STM32 빌드와 Isaac 실행 환경은 별도 설치가 필요하다. 일부 실험 입력과 녹화는 로컬 보관 자료로, 저장소 복제만으로 전체 접기를 재현할 수 있는 상태는 아니다. 공개 자료와 재현 범위는 [검증 근거](docs/EVIDENCE.md)를 따른다.

## 저장소 구조

| 경로 | 내용 |
|---|---|
| `config/` | 관절·운용 한계, 카메라, 수건 태스크 계약 |
| `firmware/` | STM32 제어기 |
| `hardware/` | 배선과 하드웨어 자료 |
| `protocol/` | Pi–STM32 프로토콜 |
| `ros2_ws/src/` | 카메라, 인식, 로봇 모델, MoveIt, 실행 브리지 |
| `isaac_sim/` | 시뮬레이션 자산과 실행 안내 |
| `tools/`, `tests/` | 계획·진단·검증 도구와 단위 시험 |
| `docs/` | 진행 상황, 설계, 로드맵, 검증 기준 |

`single_arm_bridge`와 `stm32_g474_single_arm`은 기존 배포와의 호환성을 위해 유지한 이름이다. 실제 양팔 명령은 resident adapter를 통한다.

## 문서

- [현재 진행 상황](docs/CURRENT_STATUS.md) · [단계별 로드맵](docs/ROADMAP.md)
- [시스템 구조](docs/ARCHITECTURE.md) · [수건 조작 설계와 품질 기준](docs/TOWEL_FOLDING.md)
- [R2 검증 기준](docs/R2_SIM2REAL_CONTRACT.md) · [검증 근거와 재현 범위](docs/EVIDENCE.md)
- [도구 사용](tools/README.md) · [시뮬레이션](isaac_sim/README.md) · [프로토콜](protocol/README.md)

선행 펜 조작 데모는 [Bimanual-Pick-And-Place](https://github.com/Anhyeonseo/Bimanual-Pick-And-Place)에 별도로 보관한다.

## License

자체 작성 코드와 문서는 [Apache License 2.0](LICENSE)을 따른다. 로봇 모델, STM32 HAL·CMSIS·BSP 등은 [제3자 고지](docs/THIRD_PARTY_NOTICES.md)와 각 원본 라이선스를 따른다.
