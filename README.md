# SO101 Towel Folding

Isaac Sim과 Newton에서 두 대의 SO-101로 수건을 집고, 들어 올려 반으로 접은 뒤 내려놓는 시뮬레이션이다. 마지막으로 전체 동작과 놓기 검사를 통과한 `fine_compact` 1차 접기 버전을 보존한다.

![저장된 1차 접기 결과](docs/first_fold_result.png)

## 동작과 결과

위에서 수건 안쪽을 양손으로 집고 → 들어 올리고 → 접는 경로를 따라 이동하고 → 재료 끝선 정렬을 보정하고 → 작업대에 내려놓고 → 그리퍼를 열어 물러난다.

| 항목 | 보존 결과 |
| --- | --- |
| 수건 | 300 × 300 mm, 63 × 63 요소 / 4,096 노드 |
| 그리퍼 | 고정 턱 면에 맞춘 2.2 mm 패드 |
| 접힌 두 부분의 길이 비율 | 50.8 : 49.2 |
| 대응 노드 XY 오차, p95 | 3.53 mm |
| 접힌 폭 | 161.82 mm |
| 놓기 완료 시 접촉 패치–턱 최소 거리 | 83.43 mm |
| 보존된 전체 성공 실행 | 1회 |

입자 고정이나 수건 부착 없이 접촉으로 수행한 **명목 조건의 시뮬레이션 결과**다. 로봇 관절은 지정 상태로 구동하며 Newton 결합은 단방향이다. 높은 곡률에서의 수건 연화와 패드 마찰·압착력은 실물에 맞춰 검증되지 않았다. 2차 접기와 반복 성공률은 이 버전의 완료 범위에 포함하지 않는다.

## 확인과 실행

저장된 입력과 결과를 확인하려면 일반 Python으로 실행한다. GPU나 Isaac Sim을 시작하지 않는다.

```bash
python -m pip install -r requirements-check.txt
python tools/run_first_fold.py --check
```

전체 시뮬레이션을 다시 계산하려면 [실행 환경](docs/SIMULATION.md)을 준비하고 저장소 루트에서 실행한다.

```bash
export ISAAC_PYTHON=/path/to/isaac-environment/bin/python
"$ISAAC_PYTHON" -m pip install -r requirements-simulation.txt
"$ISAAC_PYTHON" tools/run_first_fold.py --run
```

Isaac Sim 창에서 보려면 마지막 명령에 `--gui`를 추가한다. 이 명령은 물리를 다시 계산한다. 저장 영상의 실시간 재생 기능은 포함하지 않는다. 새 결과는 `output/` 아래 별도 폴더에 저장되며 보존 결과를 덮어쓰지 않는다.

## 구성

- `tools/`: 당시 물리 실행기, 필요한 시뮬레이션 보조 코드, 단일 실행 진입점
- `config/`: 물성·관절 제한·작업대·실행 설정과 입력 해시
- `artifacts/`: 로봇 URDF, 패드 형상, 접근 및 접기 경로
- `ros2_ws/src/so101_description/meshes/`: URDF가 참조하는 메시 자산만 보관
- `results/first_fold/`: 성공 실행의 형상·접촉·정렬·놓기 결과와 검증 기록

실물 로봇 제어, 펌웨어, 통신 브릿지, 카메라 인식·학습 데이터는 이 저장소의 현재 버전에서 제외했다. 자산의 출처와 변경 사항은 [NOTICE](NOTICE.md)에 정리했다.
