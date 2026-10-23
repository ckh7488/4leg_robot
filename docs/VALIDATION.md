# 검사 재현

검증 환경: Windows, FreeCAD 1.0.2의 Python 3.11, 별도 CPython 3.12. CAD Python에 시뮬레이션용 NumPy를 설치하지 않는다.

시뮬레이션 환경을 만든다. 다음 명령은 저장소 루트에서 실행한다.

~~~powershell
py -3.12 -m venv .venv
.venv\Scripts\python.exe -m pip install -r requirements-sim-lock.txt
.venv\Scripts\python.exe scripts/run_checks.py --freecad-python '<FreeCAD 설치 폴더>/bin/python.exe'
~~~

CAD 재생성을 제외하고 커밋된 모델을 검사하려면 마지막 명령에서 --freecad-python 인수를 생략한다. FreeCAD 설치 경로는 자신의 환경에 맞춘다. 기록된 의존성 버전을 설치하지 못하면 임의로 바꾸지 말고 원인을 기록한다.

검사 순서: CAD 재생성 → 기준 자세 전체 간섭 → 각 다리 27자세 → CAD 질량·관성 추출 → MuJoCo/URDF 생성 → 독립 관절/운동학 계약 검사(33자세) → 5초 정지 자세 → 10° 단축 응답 예측 → STL·3MF 검사 → 형상 미리보기.

reports/checkpoint-validation.json에는 실제 명령, 종료 코드, 의존성 버전과 입력/핵심 산출물 SHA-256(텍스트 줄바꿈은 LF로 정규화, FCStd는 원본 바이트)을 남긴다. _runs/ 로그는 로컬 진단용이며 커밋하지 않는다. 실패한 검사는 0 이외의 종료 코드를 반환한다. 단축 응답 스크립트의 정상 종료는 계산 성공만 의미하며 반응성 목표 통과가 아니다.

범위: 각 다리를 따로 움직인 유한 표본 검사이다. 여러 다리 동시 동작, 연속 이동 경로, 나사머리·공구·케이블, 강도와 열 검증은 별도다. URDF는 관절/관성/시각 모델이며 Isaac의 접촉·구동계 검증을 대신하지 않는다. 5초 정지 자세는 보행 성공이 아니다.
