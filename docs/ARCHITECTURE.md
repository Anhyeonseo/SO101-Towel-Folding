# 시스템 구조

## 공통 관측·행동 경로

```text
Top/손목 영상 + 관절/허용된 부하 정보
  → 보정·freshness·clear-view 검사
  → mask/edge/형상 요약 + 가림/unknown
  → heuristic 또는 learned policy
  → primitive + 제한 파라미터 + confidence/abstain
  → 도달·접촉·충돌·예산 검사
  → simulation adapter / real resident adapter
  → measured outcome → 퇴피·settle·재관측 → episode
```

같은 observation/action 계약을 시뮬레이션과 실물에 적용한다. 실물 정책에 수건 재료 ID·완전한 노드 위치·정확한 시뮬 접촉력은 넣지 않는다. simulator oracle은 진단/teacher/reward/평가 경로로 분리한다.

## 책임 경계

| 구성 | 책임 | 금지 |
|---|---|---|
| SceneConfig | 실제 자산·물성 후보·solver·초기 상태와 identity | 숨은 런타임 물성 변경 |
| DrivePolicy/adapter | 제한된 목표와 실제 응답 | 접촉 중 관절 state 강제 입력으로 성공 생성 |
| ContactObserver | solver 접촉/힘과 post-step 기하의 분리 기록 | 관측 중 물리·기준점 변경 |
| GraspValidator | 적층 형성·유지·미끄러짐·놓기 | 같은 입자 양턱 접촉을 네 겹으로 선언 |
| Perception | 실제 관측으로 가능한 상태와 unknown | 숨은 겹/가린 모서리의 근거 없는 확정 |
| Policy | primitive/파라미터/abstain 제안 | serial 직접 접근, gate 우회 |
| Executor/task manager | bounded 실행·정지·복구·새 관측 | 무한 retry/예산 초기화 |
| Evidence/evaluator | 입력 해시·episode split·성과/실패 | 시뮬 성공을 실물 승인으로 대체 |

## 현재 구현과 목표의 차이

bridge/protocol·관절 한계·등록 형상·영상 데이터·기본 task/perception 코드는 존재한다. 현재 native S2는 S1 실행 모듈을 동적 import하고 전역 args/callback을 바꾸며 정확한 cloth state를 읽는 진단 실행기다. 공통 관측 기반 실물 정책의 완성본이 아니다.

S2에서 필요한 Scene/Drive/Observer/Validator 경계를 먼저 정리한다. 기존 S1 전체 모듈의 재작성은 선행 조건으로 삼지 않는다. [R2 종료 계약](R2_SIM2REAL_CONTRACT.md)이 승격 범위를 정한다.

## 실물 실행 불변식

- 부팅/재연결로 모터가 움직이지 않는다. 상위 정책은 serial에 직접 접근하지 않는다.
- 실제 실행은 resident 양팔 adapter와 등록된 명령/관절 계약을 따른다.
- timestamp·보정/모델/계약 identity가 stale이면 거절한다.
- 한 팔 fault/slip/추종 실패는 양팔 정지로 연결한다.
- 실제 종료 판정은 measured feedback와 새 clear observation을 함께 사용한다.
- 시뮬 승인과 실물 승인을 구분한다. 실제 힘/장력 한계는 실측/승인 범위 안에서만 사용한다.

## 학습과 검증

R1의 실제 episode split을 재사용하고 R2에서 제한된 환경과 baseline을 만든다. R3/R4 실제 동작으로 모델 차이와 primitive 결과를 보정하고 R5에서 learned policy의 전이를 평가한다. 시뮬 물리 오류를 domain randomization으로 덮지 않는다. 실제 평가 데이터를 추가 학습에 사용하면 평가 split을 새로 만든다.
