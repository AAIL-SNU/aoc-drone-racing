# AOC 경기 화면과 채점 대시보드 데모

학생별 비행을 재생하면서 비행 상태, 게이트 통과, 채점 기록과 현재 순위를 함께 보여주는 회의용 데모다. 조교들이 화면 구성과 데이터 연결 방식을 검토하고 이후 평가기·렌더링 연동을 논의하는 데 사용한다.

기본 시연에는 합성 비행 데이터를 사용한다. 테스트베드가 저장한 실제 JSON 로그도 화면에서 선택해 재생할 수 있다. 학생 제어기의 자동 실행과 공식 채점 기준 확정은 후속 작업이다.

실행하려면 이 폴더의 파일을 함께 내려받고 `index.html`을 브라우저로 연다. GitHub에서는 해당 브랜치를 선택한 뒤 **Code → Download ZIP**으로 내려받아 압축을 풀면 된다. 별도 패키지 설치나 빌드 과정은 필요하지 않다.

1. **시연 시작**을 눌러 예시 참가자의 비행을 재생한다.
2. **종료 3초 전**으로 이동하면 종료 직후 기록과 순위 발표를 빠르게 확인할 수 있다.
3. **다음 참가자**로 경기를 이어가고, 순위표의 참가자 이름을 눌러 다시 볼 수 있다.
4. **로그 불러오기**에서 테스트베드의 JSON 파일을 선택하면 실제 기록을 확인할 수 있다.

| 파일 | 구현 역할 |
| --- | --- |
| [index.html](index.html) | 비행 화면, 채점판, 순위표, 안내 창의 구조 |
| [style.css](style.css) | 색상, 글꼴, 배치, 화면 크기에 따른 표시 |
| [course.js](course.js) | Canvas에 코스·게이트·기체·궤적을 그리는 기능과 3D·평면 보기 |
| [race-data.js](race-data.js) | 예시 비행 데이터 생성, JSON 로그 검증, 재생에 필요한 수치 계산 |
| [app.js](app.js) | 재생 시각, 버튼 조작, 비행 이벤트, 기록 공개와 순위 갱신 |

구현 흐름은 **예시 데이터 또는 선택한 JSON → 현재 재생 시각의 상태와 이벤트 → 비행 화면·수치 갱신 → 종료 기록 공개 → 순위 갱신**이다. 화면 파일은 HTML·CSS·JavaScript로 구성되어 있고, 브라우저 안에서 실행된다. 선택한 JSON도 브라우저에서 읽는다.

테스트베드와 연결할 때는 `schema_version: 1` 형식의 로그를 사용한다. `samples`는 시간별 상태와 제어입력, `events`는 게이트 통과와 종료 사건, `summary`는 최종 결과를 담는다. `config`와 `scenario`에는 코스와 실행 조건이 필요하다. `realization`과 `provenance`는 제공되면 함께 보존하며, `provenance`의 Python·패키지 버전은 순위 비교 그룹을 정할 때 사용한다. 전체 입력 검사는 `race-data.js`의 `normalizeLog` 함수를 참고한다. [테스트베드 로그 생성 코드](../../src/quad_race/env.py)

재생 중 속도·추력은 저장된 샘플을 사용하고, 하중·누적 벌점은 샘플 사이의 속도 변화로 추정한다. 종료 시에는 로그의 최종 `summary`를 표시한다. 공식 누적 벌점까지 정확히 맞추려면 평가기에서 해당 값을 기록하고 전달하는 규격을 함께 정해야 한다.

데모에서는 완주자의 환산 기록이 짧을수록 높은 순위를 부여하고, 0.01초 단위로 반올림한 기록이 같으면 공동 순위를 준다. 미완주는 DNF, 시뮬레이션 오류는 검토 상태로 표시한다. 실제 로그는 코스·설정·시나리오·실행 버전이 같은 그룹끼리 비교한다. 이 규칙의 공식 채택 여부는 조교 협의 항목이다.

구현 과정을 공유할 때는 **작업 브랜치 → commit → push → Pull Request → 조교 의견 반영** 순서로 진행한다. 브랜치는 작업 공간을 구분하고, commit은 변경 이력을 저장하며, push는 그 이력을 GitHub에 올린다. Pull Request에서는 변경 목적과 구현 내용을 설명하고 파일별 의견을 주고받을 수 있다. 각 변경에는 그 시점에 실제로 수정한 내용과 확인 결과를 남긴다. [GitHub Pull Request 안내](https://docs.github.com/en/pull-requests/reference/pull-requests)

첫 Pull Request의 제목은 `Add race dashboard demo`로 사용할 수 있다. 설명에는 시연 목적, 실행 방법, 현재 데이터의 성격, 조교에게 검토받을 내용을 넣는다. 논의할 항목은 다음과 같다.

- 렌더링 담당자의 비행 화면과 채점판을 어떤 데이터·시각 기준으로 맞출지
- 공식 점수식, 미완주 처리, 동점 규칙을 어떻게 정할지
- 학생·팀 ID와 실행 결과를 어떻게 연결할지
- 사전 평가 후 공개와 현장 실행 중 어떤 운영안을 사용할지

웹 주소로 화면을 열게 하려면 GitHub Pages를 추가로 설정할 수 있다. 현재 파일들은 정적 웹페이지 형태라서 게시 대상으로 사용할 수 있다. 브랜치 방식의 배포 원본은 저장소 루트 또는 `/docs`이며, 현재 폴더만 배포하려면 GitHub Actions에서 `docs/race-demo`를 게시 대상으로 지정하는 구성을 사용할 수 있다. 배포 후 GitHub Pages 설정에 표시되는 주소를 공유한다. [GitHub Pages 배포 안내](https://docs.github.com/en/pages/getting-started-with-github-pages/configuring-a-publishing-source-for-your-github-pages-site)

조직의 비공개 저장소에서 Pages를 사용할 수 있는지는 요금제와 조직 설정에 따라 달라진다. 일반 Pages 사이트는 공개 웹사이트이며, 사이트 접근을 비공개로 제한하려면 조직의 GitHub Enterprise Cloud 기능이 필요하다. 조교 내부 공유 목적에 맞춰 공개 범위를 먼저 정한다. [조직의 Pages 공개 범위 안내](https://docs.github.com/en/organizations/managing-organization-settings/managing-the-publication-of-github-pages-sites-for-your-organization)
