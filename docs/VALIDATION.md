# 검사 재현

검증 환경: Windows, FreeCAD 1.0.2의 Python 3.11, 별도 CPython 3.12. CAD Python에 시뮬레이션용 NumPy를 설치하지 않는다.

시뮬레이션 환경을 만든다. 다음 명령은 저장소 루트에서 실행한다.

~~~powershell
py -3.12 -m venv .venv
.venv\Scripts\python.exe -m pip install -r requirements-sim-lock.txt
.venv\Scripts\python.exe scripts/run_checks.py --freecad-python '<FreeCAD 설치 폴더>/bin/python.exe'
~~~

CAD 재생성을 제외하고 커밋된 모델을 검사하려면 마지막 명령에서 --freecad-python 인수를 생략한다. FreeCAD 설치 경로는 자신의 환경에 맞춘다. 기록된 의존성 버전을 설치하지 못하면 임의로 바꾸지 말고 원인을 기록한다.

검사 순서: CAD 재생성 → 기준 자세 전체 간섭 → 각 다리 27자세 → CAD 질량·관성 추출 → MuJoCo/URDF 생성 → 독립 관절/운동학 계약 검사(33자세) → 5초 정지 자세(관절별 토크·처짐 포함) → 10° 단축 응답 예측 → P 게인 비교 → STL·3MF 검사 → 형상 미리보기.

reports/checkpoint-validation.json에는 실제 명령, 종료 코드, 의존성 버전과 입력/핵심 산출물 SHA-256(텍스트 줄바꿈은 LF로 정규화, FCStd는 원본 바이트)을 남긴다. _runs/ 로그는 로컬 진단용이며 커밋하지 않는다. 실패한 검사는 0 이외의 종료 코드를 반환한다. 단축 응답 스크립트의 정상 종료는 계산 성공만 의미하며 반응성 목표 통과가 아니다.

범위: 각 다리를 따로 움직인 유한 표본 검사이다. 여러 다리 동시 동작, 연속 이동 경로, 나사머리·공구·케이블, 강도와 열 검증은 별도다. URDF는 관절/관성/시각 모델이며 Isaac의 접촉·구동계 검증을 대신하지 않는다. 5초 정지 자세는 보행 성공이 아니다.

## 응답 판정과 실패 경로

~~~powershell
.venv\Scripts\python.exe scripts/run_checks.py --suite response
.venv\Scripts\python.exe scripts/check_step_response.py --require-targets
.venv\Scripts\python.exe scripts/check_failure_gates.py --freecad-python '<FreeCAD 설치 폴더>/bin/python.exe'
~~~

응답 전용 검사는 회귀검사 18개, 양방향 10°/부하 3종의 6조건, P 게인 비교를 계산한다. 판정은 빠르기만 한다: 최초동작 ≤40ms, 명령 시점부터 실제 도달 변위 90%까지 ≤150ms(design.json response_acceptance_targets). 하중 정확도는 하중당 처짐(°/N·m)으로 보고서 static_compliance에 기록하며 합격 기준은 미정이다. 명령 각도 ±0.2° 정착과 마지막 실제 위치 근처 정착은 둘 다 진단값이다.

마지막 100ms 전체가 관찰되고 범위 ≤0.1°, 선형회귀 드리프트 절댓값 ≤1°/s일 때만 최종 위치 추정값을 속도 계산에 사용한다. 경계 직전 표본을 포함하므로 샘플 간격이 100ms에 나누어떨어질 필요는 없다. 실제 변위/명령 변위도 최소 0.5여야 한다. 임계값은 design.json에 있으며, 위반하면 speed_metric_valid=false 및 rise_to_90percent_of_final_ms=null로 기록한다. 이 선별은 유한 시간 관찰의 유효성을 확인할 뿐 점근 안정성이나 정확도 보장이 아니다. 지속 진동·관찰 시간 부족·드리프트·도달량 부족을 회귀검사로 거부한다.

actual_to_command_step_ratio와 terminal_goal_error_deg를 속도 옆에 기록한다. 정확도 예산과 실기 검증 전에는 accuracy_target_met 및 overall_response_accepted가 null이다. single_trial_targets_met와 all_single_trial_targets_met는 결정론적 속도 선별 결과만 뜻한다. 게인 요약에서 계산 불가능한 속도는 null로 유지하며 해당 조건 수를 표시한다.

scripts/check_gain_sweep.py는 P 400/800/1600/3200 및 현재 설계 게인(목록에 없을 때 추가)에서 같은 6조건을 계산해 reports/gain-sweep.json에 쓴다. design.json의 게인은 바꾸지 않는다.

현재 --require-targets 명령은 설계 게인에서 6조건 모두 빠르기 목표 미달이므로 종료 코드 1을 반환한다. 일반 계산 명령은 계산 성공 시 0을 반환하되 reports/step-response.json의 all_single_trial_targets_met는 false다. 실행기 보고서도 all_commands_passed와 response_numerical_targets_met를 나누며 p95/하드웨어 성공을 주장하지 않는다.

실패 경로 검사는 _runs 아래의 별도 CAD 복사본에 FL_calf_front와 완전히 겹치는 고정 장애물을 삽입한다. 원본 CAD는 건드리지 않는다. 기준 자세/다리 동작 검사 둘 다 간섭 보고서와 종료 코드 1을 내야 시험이 통과한다. reports/failure-gate-regression.json에 기록한다. 전체 CAD 실행기에도 이 검사를 포함한다.

초기 전체 CAD 재생성/검사의 실행 증거는 커밋 f6dea2c 및 reports/archive/cp003/에 남겼다. 최신 보고서 suite/cad_regenerated/commands 필드로 이번에 실제 실행한 범위를 확인한다.
