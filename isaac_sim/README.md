# 수건 시뮬레이션

Isaac Sim과 Newton을 이용해 수건–그리퍼–작업대 접촉을 진단한다. 현재는 R2 구동·접촉 검증 단계이며, [R2 검증 기준](../docs/R2_SIM2REAL_CONTRACT.md)의 G1을 통과하지 못했다.

## 자산과 실행 경로

- 로봇 형상: `ros2_ws/src/so101_description/`, `artifacts/bimanual/preview/`
- 수건 측정값: [towel_isaac_s1_material.json](../config/towel_isaac_s1_material.json)
- 보존 S1 실행기: [run_towel_s1_vertex_patch_lift.py](../tools/setup/isaac/run_towel_s1_vertex_patch_lift.py)
- 현재 S2 진단: [run_native_bimanual_second_fold.py](../tools/run/run_native_bimanual_second_fold.py)
- 실행 결과와 입력 식별자: [검증 근거](../docs/EVIDENCE.md)

S1은 기존 접촉 조건의 접힘 비교 결과다. 직접 관절 상태 입력과 단방향 결합의 한계가 있어 실물 압착력 검증으로 사용하지 않는다. S2 위치 구동 경로는 초기화 이후 목표 명령을 적용하지만, 들기·놓기의 물리 검증은 아직 끝나지 않았다.

## 실행 환경과 재현 범위

개발 환경은 Isaac Sim 6.0.1 기반 Python이다. 실행 시 `OMP_NUM_THREADS=8`, `OPENBLAS_NUM_THREADS=1`을 사용했다. 다른 설치 환경에서는 해당 Python 실행 경로를 `ISAAC_PYTHON`에 지정하고 Isaac Lab·Newton 의존성을 별도로 확인한다. 일반 host 의존성 설치만으로는 실행할 수 없다.

S1 recipe와 최신 S2 계획·녹화는 로컬 보관 자료다. 일부 입력에는 외부 보정 파일의 절대 경로가 있어, 공개 저장소만으로 전체 실행을 재현하는 패키징은 G0의 남은 과제다. 재실행에는 입력 해시 확인과 별도 출력 디렉터리가 필요하다.

수건 물성·해상도·solver·2.2 mm 패드는 기준본을 유지한다. 저장된 S1 수건 형상과 0 속도로 시작하는 국소 시험은 전체 solver checkpoint 복원이나 S1→S2 연속 실행과 구분한다.

## 결과 확인과 재생

저장 영상·프레임을 재생하면 물리 계산 없이 결과를 확인할 수 있다. `replay_verified_native_fold.py`는 물리를 다시 계산하는 실행기이므로 영상 재생과 용도가 다르다. 로봇 관절만 재생하는 강체 회귀 도구도 수건 접촉을 검증하지 않는다.

최신 `local_primitive` 실행은 초기화 단계까지 저장된 중단 결과다. 최종 녹화가 없으므로 해당 실행의 들기·놓기 완료를 재생하거나 판정할 수 없다.
