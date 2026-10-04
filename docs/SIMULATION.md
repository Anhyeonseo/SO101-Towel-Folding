# 실행 환경과 보존 범위

## 환경

Ubuntu x86-64, Python 3.12, NVIDIA GPU와 정상 동작하는 드라이버가 필요하다. 보존 실행은 Isaac Sim 6.0.1 기반 환경에서 수행했다. 이 실행기는 Newton 결합을 제공하는 Isaac Lab 소스가 필요하며 일반적인 PhysX 전용 Isaac Lab 설치만으로는 실행되지 않는다.

패키징 시 확인한 개발 환경은 아래와 같다. 원래 실행의 모든 의존성 버전이 별도로 기록되지는 않았으므로 이 표를 새 설치에서의 재현 보증으로 해석하지 않는다.

| 구성 요소 | 설치 환경 |
| --- | --- |
| Isaac Sim | 6.0.1.0 |
| Isaac Lab | 6.1.17, 소스 `367e498138c233e56896f3deb818aaed8a094dd8` |
| Newton | 1.2.1 |
| Warp | 1.13.0 |
| Torch | 2.10.0+cu128 |
| MuJoCo / mujoco-warp | 3.8.0 / 3.8.0.3 |

Isaac Lab에는 `isaaclab_physx`, `isaaclab_newton`, `isaaclab_contrib.deformable`이 모두 있어야 한다. Python 보조 의존성은 `requirements-simulation.txt`에 있다. `urdf-parser-py`를 직접 설치하므로 실물 ROS 노드나 ROS 워크스페이스를 빌드할 필요는 없다.

`config/first_fold_recipe.json`은 성공 실행의 수치 설정과 인자를 보존한다. OMP·MKL·OpenBLAS 스레드 수는 각각 8이다. 물성·패드·해상도·solver 설정을 변경하면 별도 실험으로 취급한다.

## 검증과 출력

`--check`는 입력 해시, 로봇 메시 참조, 장면·패드 계약, 저장된 형상 및 놓기 결과를 검사한다. Isaac Sim의 새 실행을 의미하지 않는다. `--run`은 입력 검사 후 물리를 실행하고 종료 시 생성된 결과를 검사한다. `--gui`에서는 시뮬레이션 종료 후 창을 닫으면 결과 검사가 이어진다.

```bash
"$ISAAC_PYTHON" tools/run_first_fold.py --run --output output/my_first_fold
```

출력 폴더는 새 이름이어야 한다. `isaac.log`는 실행 로그, `result.json`은 전체 결과, `validation.json`은 새 실행의 검사 결과다. 실패 시 로그를 보존하고 오류로 종료한다.

보존 결과는 `results/first_fold/verification.json`에 요약되어 있다. 이 파일의 `simulation_first_fold_completed`는 명목 시뮬레이션 검사를 뜻한다. 원본 `result.json`의 `completion_claim.s1_completed=false`는 실물 마찰·전체 형상 결정성 등 더 넓은 조건이 검증되지 않았다는 별도 표시이며 그대로 유지했다.

## 패키징 변경

- 원본 스냅샷: `5b16fff82e400e4cca8cdcff96a6d1548058ef80`.
- 성공 실행의 물리 실행기와 시뮬레이션 보조 코드는 그대로 유지했다. 공용 코드에 남아 있는 후속 실험 분기는 공개 진입점에서 사용하지 않는다.
- 개인 경로를 저장소 상대 경로로 바꿨다. 장면 검사가 참조하던 실물 카메라 보정 파일은 동일한 작업대 크기·위치만 담은 `config/worktable.json`으로 대체하고 해당 식별 해시를 갱신했다.
- 원래 입력 해시는 `results/first_fold/provenance.json`, 배포 파일 해시는 `config/package_sha256.json`에 있다. 경로를 정리한 파일의 해시는 원본과 다르며 수치 결과는 유지했다.
- 경로 생성 당시의 출처 메타데이터에는 현재 배포하지 않는 과거 설정 이름이 남아 있다. 실행 입력은 `first_fold_recipe.json`과 보조 코드가 읽는 포함 파일로 완결된다.
- 패키징 검사는 통과했다. 패키징 당시 로컬 NVIDIA 드라이버에 연결할 수 없어 GPU 전체 재실행은 수행하지 않았다. 공개된 성공 결과는 기존 실행 기록이다.
