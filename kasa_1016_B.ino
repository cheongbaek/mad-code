// ============================================================
//  kasa_1016_B.ino : B보드 — 조향(PD) + 제동(★시간 제어★ 2단) + 주행모드 스위치
//                    (Arduino Mega 2560)
//
//  kasa_1015_B.ino 기반. ★바뀐 것은 셋이다★ — 리니어를 가변저항 대신 시간으로 움직인다([1016]),
//  제동등은 A5 스위치 또는 1·2단 지령으로 켠다([1016-B]), ★"STOP" 줄을 없애고 텔레메트리 셋째 칸
//  (0 평상 / 1 제동등 / 2 E-stop)으로 알린다([1016-C])★. E_STOP_TEST·핀·조향 PD·캘리브레이션·
//  EEPROM·핸드셰이크·AS 상태([1007])는 1015 그대로다.
//
// ════════════════════════════════════════════════════════════════════════════
//  ★★ [1016] 리니어는 ★시간으로만★ 움직인다 — 가변저항(A5)을 보지 않는다 (사용자 지시) ★★
// ════════════════════════════════════════════════════════════════════════════
//    PWM 255 고정. 단계 전이마다 정해진 시간만 밀고 끊는다 (상수 LIN_MS_*, 고치는 곳은 거기뿐):
//        0 → 1단   나옴   0.20 s        1 → 0단   들어감 0.5 s
//        0 → 2단   나옴   0.40 s        2 → 0단   들어감 0.5 s
//        1 → 2단   나옴   0.30 s        2 → 1단   들어감 0.5 s → 나옴 0.20 s (= 2→0 뒤 0→1)
//    ▶ ★이동이 도중에 끊기면 자리를 모른다★ (E-stop · 수동 전환 · 캘리브레이션 'a' — linAbort)
//      그때는 다음 이동이 이렇게 고른다 : 0단 = 들어감 0.5 s(끝까지) / 2단 = 나옴 0.4 s
//      (더 밀려도 2단 쪽 끝에 닿을 뿐이다) / 1단 = 들어감 0.5 s → 나옴 0.20 s.
//      E-stop 해제는 2단에 있다고 보고 2→0(들어감 0.5 s), 수동 진입은 자리를 모른다고 보고 끝까지 넣는다.
//    ▶ 지시가 바뀌어도 ★진행 중인 이동은 끝까지 한다★(최대 0.7 s, 2→1단) — 끝난 뒤 새 지시로 간다.
//      E-stop · 수동 전환만 그 자리에서 끊는다(종전과 같다).
//    ▶ ★걷어낸 것★ — 가변저항으로 하던 일 전부 : 목표 위치 도달 판정([0813-1]), 처지면 되미는
//      유지 구동([0909-1]), 가드, A5 아날로그 값을 보던 제동등([0821-1]). 그래서
//        · 사람이 페달을 밟고 있다가 떼도 리니어는 되밀지 않는다(유지 구동이 없다).
//        · 텔레메트리 셋째 칸(옛 A5 값)은 ★0 고정★ — 칸 수 4 는 지킨다(gold arduino.py parse_b 가 4칸만 받는다).
//
// ════════════════════════════════════════════════════════════════════════════
//  ★★ [1016-B] 제동등 D11 — A5 스위치가 GND 에 이어지면 ★또는★ 1·2단 지령이면 켠다 (사용자 지시) ★★
// ════════════════════════════════════════════════════════════════════════════
//    켜짐 = digitalRead(A5) == LOW (스위치 닫힘)  ||  brake_cmd_level 이 1단·2단 (E-stop 중에는 2단 지령이다)
//    ▶ 배선 : ★A5 ── 스위치 ── GND★ — D5 모드 스위치·D12 E-stop 과 같은 꼴이다(Mega 의 A5 = 디지털 59번).
//      A5 는 INPUT_PULLUP — 열려 있으면 HIGH(꺼짐), 닫혀 GND 에 이어지면 LOW(켬). 외부 저항·5V 선은 필요 없다.
//      ★A5 에 5V 신호선이나 풀다운 저항을 같이 물리지 않는다★ — 풀업과 맞당기거나, 스위치가 닫히면 5V 가 GND 로 단락된다.
//    ▶ 선이 끊기면 스위치로는 켜지지 않는다(지령으로는 켜진다). 선이 길거나 모터 곁을 지나 깜빡이면
//      A5–5V 에 4.7~10kΩ 풀업을 덧대거나 시간 확인을 넣는다.
//    ▶ 모드·E-stop 을 따로 보지 않는다 — 지령과 A5 두 가지뿐이다. 수동조종에서는 지령이 늘 0단이라
//      A5 만 남는다. 리니어가 실제로 어디 있는지는 보지 않는다(지령 기준 — 0단 지령 뒤 들어가는
//      0.5 s 동안은 이미 꺼진다).
//    ▶ D11 출력 배선은 0821 그대로 — 모터드라이버 IN1 = HIGH / IN2 = LOW 고정, PWM 선만 D11, digitalWrite.
//
// ════════════════════════════════════════════════════════════════════════════
//  ★★ [1016-C] "STOP" 줄을 없앤다 — 텔레메트리 셋째 칸이 상태다 (사용자 지시) ★★
// ════════════════════════════════════════════════════════════════════════════
//      "P,<조향각>,<상태>,<모드>"  — ★E-stop 중에도 이 한 가지 양식뿐이다★
//         상태 2 = E-stop 효력(estop_active) / 1 = 제동등 조건([1016-B] — A5 스위치 닫힘 또는 1·2단 지령)
//              / 0 = 그 밖. 2 가 1 을 이긴다(E-stop 중에는 2단 지령이라 1 조건도 참이다).
//         모드 = 실제 D5 (1 자율 / 0 수동) — 종전 "STOP" 에는 모드가 없어 상위가 '자율' 로 짐작했다.
//    ▶ 상위(gold nxde/arduino.py)는 상태 2 = E-stop, 1 = 제동 → ★둘 다 A보드 쓰로틀 0★(모드 무관).
//    ▶ ★E-stop 중이어도 핸드셰이크 '-' 에 "YES" 를 한 번 답한다★([0904-2] 그대로) — 그리고 P, 줄이
//      늘 나가므로 상위가 E-stop 이 걸린 채로도 이 보드를 B 로 식별한다(종전에는 STOP 만 와서 못 했다).
//    ▶ 핸드셰이크 유예(2초) 중에도 ★E-stop 이면 P, 줄을 내보낸다★ — 종전 STOP 과 같은 규칙이다.
//    ▶ 아래 머리말의 "STOP" 이야기([1007-1]·[1015-1]·[0904-*]·통신)는 ★1015 까지의 기록★ 이다.
//
//    ▶ 아래 [0909-1]·[0821-1]·[0821-2]·[0813-1] 머리말은 ★기록으로만★ 남긴다 — 1016 에는 해당 코드가 없다
//      (제동등 배선·D11 은 [1016-B] 가 되살렸다 — 판정은 그쪽을 본다).
//
// ════════════════════════════════════════════════════════════════════════════
//  ★★ [1015-1] E_STOP_TEST — 수동조종에서도 E-stop 이 리니어를 물게 할지 ★★
// ════════════════════════════════════════════════════════════════════════════
//    E_STOP_TEST = false : ★1007 그대로 (대회 규정 AS-Off)★
//        E-stop 효력 = D12 체결 AND D5 자율. 수동조종에서는 E-stop 을 보지 않는다.
//    E_STOP_TEST = true  : ★0909 처럼★ 모드와 무관하게 E-stop 이 효력을 갖는다.
//        수동조종에서도 E-stop → 리니어 2단 · 조향 힘빼기 · "STOP". 해제하면 0단 복귀.
//
//    ▶ 바뀌는 곳은 '효력' 한 줄뿐이다 — estopEffective() = estop_line && (auto_mode || E_STOP_TEST).
//      applyEstop·해제 분기·"STOP"·명령 무시·'a' 금지는 1007 과 같이 estop_active 를 본다.
//    ▶ ★자율→수동 0단 복귀(retractForManual)는 효력이 있는 동안 건너뛴다★ — true 에서
//      E-stop 으로 2단을 문 채 수동으로 넘기면, 그 복귀가 2단을 빼 버리고 applyEstop 은
//      이미 래치돼 다시 걸지 않으므로 'STOP 인데 리니어는 들어가 있는' 상태가 된다.
//      false 에서는 수동이면 효력이 늘 없으므로 1007 과 한 글자도 같게 돈다.
//    ▶ ★E-stop 밖의 일은 바꾸지 않는다★ — 수동조종에서 상위의 브레이크 1·2 지시를 0 으로
//      받는 [1007-2] 는 true 에서도 그대로다(이 스위치는 E-stop 시험용이다).
//    ▶ ★true 에서 상위가 보는 것★ — 수동 + E-stop 에서 "STOP" 이 나가므로 gold nxde/arduino.py
//      는 그동안 주행모드를 '자율' 로 읽는다(1007 짝 : STOP = 자율 + E-stop). HUD 는
//      AS-EMERGENCY 로 보이고, arduino.py compose (1) 이 페달 구동을 끊는다 — 0909 와 같다.
//      해제되면 "P,…,0" 이 다시 나와 '수동' 으로 돌아온다.
//    ▶ ★대회(검차·주행)에는 반드시 false 로 굽는다★ — true 는 규정 2.16.1 <표 4> AS-Off
//      'Brake Actuator unavailable' 에 어긋난다.
//
// ════════════════════════════════════════════════════════════════════════════
//  ★★ [1015-2] DC·리니어의 DIR ↔ PWM 핀 교체 — 배선에 맞춘다 (사용자 지시) ★★
// ════════════════════════════════════════════════════════════════════════════
//    조향 DC  : DIR 6 → ★7★ · PWM 7 → ★6★
//    리니어   : DIR 8 → ★9★ · PWM 9 → ★8★
//    ★이 파일에만 적용했다★ — 0909·1007 은 옛 배선(DIR 6/8 · PWM 7/9) 그대로다. 옛 파일을 지금
//    배선에 구우면 DIR 선에 PWM 이, PWM 선에 방향 레벨이 나간다.
//    PWM 은 셋 다 하드웨어 PWM 핀이다(D6·D8 = Timer4, 기본 ≈490 Hz — 종전 D7 Timer4 · D9 Timer2 와
//    같은 주파수). 제동등 D11(Timer1, digitalWrite 만)과 겹치지 않는다. 그 밖의 로직은 그대로다.
//
//  ── 아래는 1007 에서 물려받은 내용 ──
//
//  kasa_0909_B.ino 기반. ★바뀐 것은 '리니어는 자율주행 모드에서만 쓴다' 하나다★
//  (대회 규정 2.16.1 <표 4>·<표 5> 의 AS 상태). 조향 PD·캘리브레이션 동작·유지 구동·
//  핸드셰이크·제동등·출력 양식("P,..."/"STOP")은 한 줄도 건드리지 않았다.
//
// ════════════════════════════════════════════════════════════════════════════
//  ★★ [1007-1] E-stop 의 효력 = D12 체결 ★AND★ D5 자율주행 — 매 루프 ★레벨★ 판정 ★★
// ════════════════════════════════════════════════════════════════════════════
//    ※ [1015-1] 아래는 ★E_STOP_TEST = false★ 일 때의 동작이다. true 면 효력 = D12 체결
//      하나이고(0909 와 같다), 수동조종에서도 아래 AS-Emergency 와 똑같이 돈다.
//    ▶ 상태 (규정 <표 4>·<표 5> 와 같은 정의)
//        AS-ON  · RUN    : D5 자율, E-stop 해제 → 구동·조향·리니어 사용
//        AS-Emergency · PAUSE : D5 자율, E-stop 체결 → 리니어 2단 · 조향 힘빼기 · "STOP"
//        AS-OFF          : D5 수동 → ★E-stop 을 보지 않는다★ 리니어도 쓰지 않는다
//
//    ▶ 무엇이 문제였나 — 0909 까지는 E-stop 이 모드를 보지 않았다. 사람이 페달로 몰고
//      있어도 E-stop 을 누르면 리니어가 2단을 물고, 텔레메트리가 "STOP" 한 줄로 바뀌어
//      조향각·A5·모드가 끊겼다. 규정 AS-Off 는 'Brake Actuator unavailable' · '상태
//      데이터 유지' 다 — 셋 다 어긋났다.
//
//    ▶ 어떻게 고쳤나 — 변수를 둘로 갈랐다.
//        estop_line   : D12 확인 결과(100ms). ★모드와 무관하게 늘 돈다★ (종전 estop_active)
//        estop_active : ★효력★ = estopEffective() [1015-1]. 아래 모든 분기가 이것을 본다.
//      그래서 따로 짤 것이 없다 — 효력이 바뀌는 순간 종전의 체결·해제 경로가 그대로 돈다:
//        · 수동 + E-stop        : 아무 일도 없다. "P,…,0" 계속, 'x,0' 명령 수용
//        · 자율 + 체결 중 → 수동 : 효력이 내려간다 → 종전 '해제 분기' 가 0단 복귀
//        · 수동 + 체결 중 → 자율 : 효력이 올라간다 → applyEstop 이 2단 (D5 확인 50ms 뒤)
//      estop_line 을 수동에서도 계속 세는 이유 — 자율로 넘기는 순간 100ms 확인을 다시
//      기다리지 않는다. '즉시' 의 지연은 D5 확인(MODE_CONFIRM_MS 50ms) 하나뿐이다.
//
//    ▶ loop 순서 : updateEstop → ★updateMode★ → 효력 계산 → pollSerial → 분기.
//      종전에는 updateMode 가 pollSerial 뒤였다 — '발동 직후 들어온 줄이 한 번 먹는 틈을
//      없앤다' 는 E-stop 의 원칙을 모드 게이트에도 그대로 적용한다.
//
//    ▶ ★"STOP" 은 이제 '자율주행 + E-stop' 이라는 뜻이다★ 수동에서는 나오지 않는다
//      (E_STOP_TEST = true 면 수동에서도 나온다 — 헤더 [1015-1]).
//      상위(gold nxde/arduino.py)는 STOP 을 받으면 모드를 '자율' 로 본다 — ★이 펌웨어와
//      짝이다★(구 펌웨어 0909 는 수동에서도 STOP 을 낸다).
//
// ════════════════════════════════════════════════════════════════════════════
//  ★★ [1007-2] 수동조종 = 리니어는 ★들어간 채★ — 진입 즉시 0단 복귀 + 지시 무시 ★★
// ════════════════════════════════════════════════════════════════════════════
//    ▶ D5 가 수동으로 확정되는 순간(자율→수동 엣지, 그리고 수동으로 부팅한 순간)
//      진행 중인 이동을 그 자리에서 끊고 0단 복귀(들어감 — [1016] 시간표)를 건다 — retractForManual().
//      ★[1016] 자리를 모른다고 보고(BRAKE_UNKNOWN) 끝까지 넣는다★ — 체결 도중에 끊었어도,
//      수동으로 부팅했어도 같다.
//      리니어는 끝까지 들어가도 문제가 없고 자체 잠금이라, 이미 들어가 있어도 무해하다.
//    ▶ 수동인 동안 handleLine 은 브레이크 1·2 를 0 으로 받는다 — 규정 2.16.1 다 'ASMS
//      Off 에서는 AS 요청으로 제동 불가' 를 ★펌웨어가★ 보장한다(종전에는 상위가 수동에서
//      늘 0 을 보내는 것에만 기댔다).
//    ▶ ★조향은 건드리지 않는다★ — 수동에서는 D5 스위치가 조향 구동 배선을 물리적으로
//      끊는다(사용자 확인 2026-10-07). 같은 일을 소프트웨어로 한 번 더 하지 않는다.
//    ▶ 대가 — 상위의 '수동조종 AEB'(/aeb_stop, lidar one_launch 전용)는 이제 리니어를
//      못 문다(구동 차단만 남는다). 규정 AS-Off 와 애초에 맞지 않는 기능이다.
//
//    ▶ (1007 당시 제동등 D11 은 A5 를 봤다 — [1016] 에서 걷어냈다.)
//
//  ── 아래는 0909 에서 물려받은 내용 ──
//
// ════════════════════════════════════════════════════════════════════════════
//  ★★ [0909-1] 1·2단이 유효한 동안 ★값이 내려가면 PWM 100 으로 따라간다★ ★★
// ════════════════════════════════════════════════════════════════════════════
//    ▶ 무엇이 문제였나
//      종전에는 1·2단을 지시받으면 목표를 향해 ★한 번★ 움직이고 끝이었다
//      (LIN_MOVE → LIN_IDLE). 그런데 ★사람이 발로 페달을 밟고 있는 동안★ 에는
//      A5 가 이미 목표값 근처(또는 그 위)라 리니어가 거의/전혀 움직이지 않는다.
//      그 상태에서 사람이 발을 떼면 페달이 돌아오는데 ★리니어는 뒤에 있어서
//      제동력이 하나도 걸리지 않는다★. 상위(자율주행)는 "2단을 물었다"고 믿고
//      있는데 차는 굴러간다 — 이 조합이 제일 위험하다.
//
//    ▶ 어떻게 고쳤나
//      1·2단이 유효한 동안 A5 가 목표보다 BRAKE_SAG 만큼 내려가면, 그때마다
//      ★PWM 100 으로 천천히★ 목표까지 다시 민다(새 상태 LIN_HOLD).
//      ★올라갈 때는 움직이지 않는다★ — 사람이 더 세게 밟은 경우가 그것이고,
//      되돌리려 들면 사람과 힘겨루기가 된다. 목표에 닿으면 그 자리에서 끊는다.
//
//    ▶ 왜 255 가 아니라 100 인가
//      이것은 '제동을 거는 동작'이 아니라 '이미 걸어 둔 제동이 빠지지 않게
//      따라가는 동작'이다. 255 로 때리면 페달을 걷어차듯 물려 기구가 왕복하고
//      (리니어에 제일 나쁘다) 사람 발과도 부딪친다. 천천히 따라가면 사람이 발을
//      떼는 속도에 맞춰 자연스럽게 이어받는다.
//
//    ▶ 종전 [0813-3] '처짐 보충' 이 이 자리에 있었다
//      트리거(BRAKE_SAG=10)는 그대로 쓰고, ★단발 PWM 255 재이동★ 을
//      ★연속 PWM 100 유지★ 로 바꾼 것이다. 그 장치는 원래 '기구 처짐'(값이 조금
//      내려가는 것)만 보던 것이라, 발을 떼는 경우(값이 한꺼번에 크게 내려간다)에
//      255 로 밀면 과했다.
//
//    ▶ E-stop 중에도 돈다
//      applyEstop 은 updateBrake 를 부르지 않으므로 유지의 시작·종료를 그 안에서
//      직접 돌린다. E-stop 으로 2단을 문 뒤 사람이 발을 떼는 경우가 실제로 있다.
//
//    ▶ 안전장치
//      · 목표 도달 즉시 정지 (BRAKE_CONFIRM_N 연속 확인 — LIN_MOVE 와 같은 판정)
//      · BRAKE_GUARD_HI 이탈 시 즉시 정지 (기구 보호)
//      · 지시 단계가 바뀌면 즉시 중단하고 일반 이동에 넘긴다
//      · BRAKE_HOLD_MAX_MS(3s) 를 넘기면 못 가고 있다는 뜻 — 끊고
//        BRAKE_HOLD_COOL_MS(1s) 쉬었다 다시 본다 (모터를 계속 물리지 않는다)
//
//  ── 아래는 0904 에서 물려받은 내용 ──
//
// ════════════════════════════════════════════════════════════════════════════
//  ★★ [0904-1] E-stop 확인시간 500ms → ★100ms★ (발동·해제 대칭은 그대로) ★★
// ════════════════════════════════════════════════════════════════════════════
//    발동 : 100ms 연속 개방(HIGH) / 해제 : 100ms 연속 단락(LOW).
//    A보드(kasa_0904_A.ino)의 같은 상수도 함께 100 으로 바꿨다 — 두 보드의 규약은 늘 같다.
//
//    ★★ 단, ★차량의 E-stop 을 실제로 판정하는 것은 이 보드 하나다★ ★★
//      A보드는 0904 에서 ESTOP_ENABLED = false 로 꺼 두었다(그쪽 헤더 [0904-2] 참고).
//      D12 NC 라인은 두 보드에 병렬로 물려 있지만, 이제 그 라인을 보는 것은 여기뿐이다.
//      그래서 ★이 보드의 ESTOP_ENABLED 는 반드시 true 여야 한다★ — 여기까지 끄면
//      차량에 소프트웨어 비상정지가 남지 않는다.
//      상위(nxde/arduino.py update_estop)는 A·B 어느 쪽의 "STOP" 이든 OR 로 받으므로,
//      이 보드의 STOP 한 줄이 곧 차량 전체의 비상정지가 된다(A보드 인휠도 그때 멈춘다).
//
//    [왜 줄이는가] 0804 의 500ms 는 '주행 중 오발동이 더 위험하다'는 판단으로 둔 값이고,
//      그 대가가 ★비상정지가 최대 0.5초 늦는 것★ 이었다. 접점 채터링은 수 ms 단위라
//      100ms 로도 충분히 걸러지므로, 확인 시간을 노이즈를 막는 최소치까지 낮추고
//      정지 지연을 그만큼 되돌린다. 늦는 거리는 A보드 헤더 [0904-1] 표 참고.
//
// ════════════════════════════════════════════════════════════════════════════
//  ★★ [0904-2] 연결확인 핸드셰이크 : '-' 한 줄을 받으면 "YES" 한 줄 ★★
// ════════════════════════════════════════════════════════════════════════════
//    ▶ 왜 : 텔레메트리('P,...')가 나온다는 것은 ★보드→PC 한 방향★ 만 증명한다.
//      PC→보드 방향은 아무것도 증명하지 못한다(TX 선이 빠져도, 포트를 다른 프로세스가
//      물고 있어 write 가 먹히지 않아도 출력은 멀쩡히 보인다). 그래서 상위
//      (nxde/arduino.py identify_port, nxde/check.py)가 '-' 를 보내고 "YES" 를 받아야
//      비로소 ★양방향★ 연결로 인정한다. 못 받으면 포트를 닫고 다시 열어 재확인한다.
//    ▶ 동작 : '-' 한 줄(다른 문자 없이)을 받으면 즉시 "YES" 를 한 번 찍고, 그 뒤
//      HANDSHAKE_HOLD_MS(2000ms) 동안 ★평상시 텔레메트리를 내보내지 않는다★.
//      상위가 "YES" 를 'P,...' 에 섞이지 않은 채로 읽게 하기 위함이다. 유예가 끝나면
//      아무 상태도 남기지 않고 원래대로 돌아간다.
//    ▶ ★millis() 로만 센다 — delay() 를 쓰지 않는다★ 유예 중에도 조향 PD·리니어·
//      제동등·E-stop 판정은 평소대로 돈다. 멈추는 것은 출력 한 줄뿐이다.
//    ▶ ★E-stop 의 "STOP" 은 유예 중에도 나간다★ 상위가 그 문자열로 비상정지를
//      판정하므로, 여기서 가리면 2초 동안 비상정지가 상위에 보이지 않는다.
//    ▶ 조향·제동에는 아무 영향이 없다 — 마지막 명령을 그대로 물고 있는다.
//      캘리브레이션('a') 중에도 받는다. 구동 명령이 아니라 '통신이 살아 있는가' 의 답이다.
//
// ════════════════════════════════════════════════════════════════════════════
//  ★★ [0821-1] 제동등 (D11) — 모드·E-stop 과 무관하게 ★상시 작동★ ★★
// ════════════════════════════════════════════════════════════════════════════
//    0813 까지 A5(리니어 가변저항)가 쓰인 곳은 ★리니어의 목표위치 도달 판정 하나뿐★ 이었다.
//    0821 부터는 같은 값이 제동등도 켠다.
//
//    ▶ ★판정에 쓰는 것은 A5 값 하나뿐이다★ brake_level 도, auto_mode 도, estop_active 도
//      보지 않는다. 조건은 '차가 무엇을 지시받았는가'가 아니라 ★페달이 실제로 내려갔는가★
//      이기 때문이다. 그래서 자율주행이 리니어로 밟든, 사람이 발로 직접 밟든, E-stop 이
//      밟든 똑같이 켜진다 — 뒤차 입장에서 그 셋은 구분할 이유가 없는 같은 사건이다.
//
//    ▶ 문턱 : A5 >= BRAKELIGHT_ON_RAW(★370★ — [1016-1], 옛 가변저항은 350) 이면 점등, 아니면 소등.
//      370 의 위치 — 0단(놓음)은 360 이하이고 1단이 650, 2단이 843 이므로
//      ★1단 체결이 끝나기 전에 이미 켜진다★ (제동등은 빨리 켜지는 쪽이 옳다).
//      ※ 문턱 근처에 발이 머무를 때 등이 깜빡이지 않도록 BRAKELIGHT_HYST(8) 만큼의
//        히스테리시스를 두었다(점등 370 / 소등 362). 단일 문턱 그대로를 원하면 0 으로.
//
//    ▶ ★어떤 분기에도 걸려 있지 않다★ loop 의 맨 앞에서 매 루프 갱신하므로 E-stop 중에도,
//      캘리브레이션 중에도, 수동조종에서도 똑같이 동작한다. 배선상 리니어·DC 쪽 12V 가
//      끊겨도 제동등 회로는 살아 있게 되어 있으니, 소프트웨어도 그 전제를 깨지 않는다.
//
//    ▶ ★A5 샘플링을 살려 두어야 했다★ 0813 은 '0단으로 대기 중'일 때 A5 를 아예 읽지
//      않았다(리니어를 움직일 때와 1·2단 처짐 확인 때만 읽었다). 그래서 ★사람이 발로
//      밟은 것을 볼 수 없었다★ — 제동등이 가장 필요한 경우가 그것이다. refreshLinPot()
//      이 정지 중에도 LIN_SAMPLE_MS(20ms)마다 읽어 lin_pot 을 최신으로 유지한다.
//
//    ▶ 배선 : 제동등은 12V 이고 모터드라이버를 경유한다. 단순 ON/OFF 만 하므로
//        IN1 = HIGH 고정 / IN2 = LOW 고정 (★배선에서 고정 — 아두이노 핀을 쓰지 않는다★)
//        PWM 선만 ★D11★ 에 물리고, 세기 조절이 없으므로 analogWrite 가 아니라 digitalWrite.
//      ※ D11 은 PWM 가능 핀(Mega Timer1)이지만 digitalWrite 만 쓰므로 타이머를 잡지
//        않는다 — 조향(D6)·리니어(D8) PWM 과 간섭하지 않는다. (핀은 [1015-2] 기준)
//
// ════════════════════════════════════════════════════════════════════════════
//  ★★ [0821-2] 텔레메트리에 A5 원본값 — 출력 필드가 3개다 ★★
// ════════════════════════════════════════════════════════════════════════════
//      0813 :  "P,<조향각>,<모드>"
//      0821~:  "P,<조향각>,★<A5원본>★,<모드>"
//    A5 는 ★환산하지 않은 raw(0~1023)★ 다. 단계(0/1/2)가 아니라 값 자체를 내보내는 이유는,
//    상위가 '리니어가 시킨 대로 갔는가'와 '사람이 발로 밟았는가'를 구분할 수 있어야 하기
//    때문이다 — 단계로 바꿔 보내면 후자가 사라진다.
//    ※ 정확히는 9점 중앙값이다(단일 analogRead 가 아니다). 스파이크만 걷어낸 값이고
//      환산은 하지 않는다. 제동등 판정·리니어 제어가 쓰는 값과 ★완전히 같은 수★ 라서,
//      로그만 보고도 보드가 무엇을 보고 판단했는지 그대로 재현된다.
//
//    ★★ 송신측(gold/arduino.py) 과 ★필드 수가 정확히 맞아야 한다★ ★★
//      arduino.py 의 parse_b 는 ★len(fields) != 4 인 줄을 통째로 버린다★ (2026-08-21
//      개정. 구버전 호환 분기를 일부러 두지 않았다 — 3토큰의 셋째는 모드, 4토큰의
//      셋째는 A5 라 자리가 겹쳐, 두 세대를 다 받으려면 의미가 흐려지기 때문이다).
//      버려도 예외·경고가 남지 않으므로, 필드 수를 바꾸면 /steer_angle_measured 와
//      /vehicle_mode 가 마지막 값에 조용히 얼어붙는다.
//      → ★펌웨어 출력 양식을 바꾸면 parse_b 도 같이 바꾼다★ 가 규약이다.
//      영향이 없는 것 : 보드 식별(identify_port 는 'P,' 접두어만 본다),
//                       E-stop 판정('STOP' 문자열 비교),
//                       연결확인 핸드셰이크("YES" 문자열 비교 — [0904-2]).
//
// ════════════════════════════════════════════════════════════════════════════
//  ★★ [0813-1] 브레이크 : 엔코더 시간표 → ★가변저항 절대위치★ ★★
// ════════════════════════════════════════════════════════════════════════════
//    0804 는 리니어에 로터리 엔코더를 붙이고 "몇 ms 밀면 몇 카운트" 실측표로 시간을
//    환산해 밀었다. 그런데 증분 엔코더는 절대위치를 모르고, 실측에서 기본위치가
//    사이클마다 −37~−55 카운트씩 밀렸다(linear_0803_pot.ino 가 잰 미끄러짐).
//    그래서 매번 하드스톱까지 밀어 재영점을 잡아야 했고, 그 재영점이 조금만 어긋나도
//    이동량이 통째로 흔들렸다. 표 자체도 PWM 255 전용이라 힘을 바꾸면 전부 무효였다.
//
//    ★가변저항(A5)은 절대값이다★ — 재영점도, 드리프트 보정도, 시간표도 필요 없다.
//    raw 값 하나가 곧 위치다. 조향(DC모터 + A2 가변저항)과 같은 구조가 된다.
//    linear_0813_throttle.ino / linear_0813_auto.ino 로 검증한 방식을 그대로 옮겼다.
//
//    ▶ 단계 (BRAKE_POS_1 / BRAKE_POS_2)
//        0 = 기본위치 : ★위치를 보지 않는다★ 무조건 들어가는 방향(REV)으로 1000ms.
//                       리니어는 안쪽으로 끝까지 들어가도 문제가 없고 자체 잠금이라,
//                       어디에 있든 이 시간이면 확실히 끝까지 들어간다.
//                       ★'값이 멎으면 하드스톱' 판정을 쓰지 않는 이유★ — 이미 끝에
//                       닿아 있으면 값이 안 변하는 것이 당연한데, 그것을 '모터가 안
//                       돈다'로 읽고 헛동작을 반복하다 두 판을 날렸다(0813 실측).
//        1 = 약한 제동 : raw 650 에 닿으면 정지      ([1016-1] — 옛 가변저항 600)
//        2 = 풀브레이킹 : raw 843 에 닿으면 정지      ([1016-1] — 옛 가변저항 850)
//    ▶ 단계 이동(LIN_MOVE)은 PWM ★255★ 하나뿐이다. 세기를 나누지 않는다.
//      (1·2단을 ★지키는★ 유지 구동만 PWM 100 — [0909-1])
//    ▶ 단계 이동은 ★1초를 넘기지 않는다★ (BRAKE_MAX_MS) — 가변저항 선이 빠지면 값이
//      떠서 영영 목표에 못 닿는데, 그때 모터를 계속 돌리지 않기 위한 최후 방어다.
//      (유지 구동은 한 번에 BRAKE_HOLD_MAX_MS 3초까지 밀고 1초 쉰다 — [0909-1])
//    ▶ 도달 판정은 ★목표선을 정확히★ 본다(여유를 빼지 않는다). 잡음은 ★연속 2회★
//      확인으로 막는다(실측 잡음 ±10, 중앙값 9점 뒤).
//    ▶ 1·2단 도달 뒤 값이 목표보다 BRAKE_SAG(10) 만큼 내려가면 다시 민다 — 0909 부터
//      PWM 100 유지 구동이다([0909-1]). ★올라간 쪽은 절대 건드리지 않는다★.
//    ▶ E-stop = 2단 체결. 목표도 판정도 일반 이동과 같은 식을 쓴다.
//
// ════════════════════════════════════════════════════════════════════════════
//  ★★ [0813-2] 조향 영점 캘리브레이션 ('a' 한 줄) + EEPROM ★★
// ════════════════════════════════════════════════════════════════════════════
//    ★왜 필요한가★ 조향 가변저항의 중심값은 링키지를 만질 때마다 달라졌다(실측
//    C=469 → 473 → …). 그때마다 DEF_RAW_LEFT/RIGHT_LIMIT 를 고쳐 다시 굽는 것은 번거롭고,
//    현장에서는 아예 불가능하다.
//
//    ▶ 사용법 : ★핸들을 일자로 둔 상태에서★ 시리얼로 "a" 한 줄을 보낸다.
//    ▶ 동작 : CAL_WATCH_MS 동안 가변저항을 지켜보며 ★최빈값★ 을 잡는다(평균이 아니라
//      최빈값인 이유는 스파이크가 평균을 끌고 가기 때문이다. 잡음 ±10 을 CAL_BIN 폭으로
//      묶어 가장 표가 많이 몰린 칸의 중앙을 쓴다).
//    ▶ 그 값을 중심으로 ★+STEER_HALF_SPAN 을 좌측 최대, −STEER_HALF_SPAN 을 우측 최대★
//      로 잡는다(145 = 코드 기본값 684/394 의 반폭과 같다. 행정 폭은 그대로 두고
//      중심만 옮기는 것이다).
//    ▶ EEPROM 에 저장한다. 이후 재부팅하면 ★EEPROM 값이 코드값보다 우선★ 이다.
//      저장된 값이 없거나 깨졌으면(매직·체크섬 불일치) 코드값을 쓴다.
//    ▶ 캘리브레이션 중에는 조향·제동 모두 정지하고, 끝나면 그 자리를 목표로 잡아
//      대기 상태로 돌아간다(급조향 방지).
//    ※ E-stop 중에는 'a' 를 받지 않는다 — 그 상황에서 사람이 핸들을 잡고 있을 리 없다.
//      ★'E-stop 중' = 효력(estop_active)이다★ E_STOP_TEST = false 면 수동에서는 E-stop 이
//      눌려 있어도 받고, true 면 수동이어도 E-stop 을 풀어야 받는다(조용히 버린다).
//
// ════════════════════════════════════════════════════════════════════════════
//  통신 (입력은 0821 + '-' 한 줄, 출력 양식은 0821 그대로 + "YES")
// ════════════════════════════════════════════════════════════════════════════
//    입력 : "<조향각>,<브레이크단계>"   예) "-12,0"  "x,2"
//             조향각  : −40~+40 (− 좌 / + 우), 또는 'x' = 힘빼기(릴리즈)
//             브레이크: 0/1/2
//           "a"      : 조향 영점 캘리브레이션 ([0813-2])
//             ※ ★arduino.py 에 'a' 송신 경로가 없는 것은 의도된 것이다★ 핸들을 일자로
//               두고 사람이 하는 일회성 조작이라, ROS 토픽으로 걸어 두면 자동으로 나갈
//               길이 생긴다. 의도의 전문은 gold white1/BOARD_B.md 3절.
//           ★"-"     : 연결확인 핸드셰이크 ([0904-2]) — "YES" 를 한 번 돌려준다★
//             ※ "-12,0" 처럼 ★'-' 로 시작하는 조향각과 겹치지 않는다★ — 이 판정은
//               '-' 한 글자로만 이루어진 줄에서만 성립한다.
//    출력 : "P,<실측조향각>,<상태>,<주행모드>"  (★[1016-C] 늘 이 양식 — "STOP" 은 없다★)
//             실측조향각 : −40~+40 (− 좌 / + 우)
//             상태       : ★[1016-C] 2 = E-stop / 1 = 제동등 조건 / 0 = 그 밖★ (옛 A5 자리)
//             주행모드   : 1 자율 / 0 수동조종
//           ("STOP" 은 1015 까지 — [1016-C] 부터는 상태 2)
//           "YES"   (핸드셰이크 응답, [0904-2])
//           "# ..." (캘리브레이션 결과 등 진단 줄 — 상위 파서는 '#' 줄을 버린다)
//
//  핀
//      A2      조향 가변저항          D7/D6   조향 DC DIR/PWM   ★[1015-2]★
//                                     D9/D8   리니어 DIR/PWM    ★[1015-2]★
//      D5      주행모드 스위치        D12     E-stop (NC)
//      ★A5     제동등 스위치 (A5 ── 스위치 ── GND, 닫히면 켬)    D11  제동등 (모터드라이버 PWM 선, ON/OFF)★  [1016-B]
//      ※ [1016] A5 를 리니어 가변저항으로는 ★쓰지 않는다★
//      ※ D2/D3(리니어 엔코더)는 ★더 이상 쓰지 않는다★ — 관련 코드 전부 삭제
//
//  ★ 켤 때 주의 ★ [1016] 수동조종으로 켜지면 곧바로 리니어를 0.5 s 들여보낸다(끝까지).
//    자율로 켜지면 움직이지 않는다 — 0단에 있다고 보고 상위 지시를 기다린다.
// ============================================================

#include <EEPROM.h>

// ================= 통신 =================
const unsigned long BAUD = 115200;
const unsigned long TELE_MS = 50;


// ================= 핀 =================
const uint8_t DC_DIR_PIN = 7;     // ★[1015-2] 6 ↔ 7 교체★
const uint8_t DC_PWM_PIN = 6;
const uint8_t DC_POT_PIN = A2;    // 조향 가변저항

const uint8_t LINEAR_DIR_PIN = 9;   // ★[1015-2] 8 ↔ 9 교체★
const uint8_t LINEAR_PWM_PIN = 8;

const uint8_t ESTOP_PIN = 12;
// ★차량의 소프트웨어 E-stop 을 판정하는 것은 이 보드 하나다★ A보드(kasa_0904_A.ino)는
//   0904 에서 ESTOP_ENABLED = false 로 꺼 두었다. ★여기까지 false 로 두지 말 것★ —
//   그러면 D12 라인을 보는 보드가 하나도 남지 않는다. 자세한 이유는 헤더 [0904-1] 참고.
const bool ESTOP_ENABLED = true;     // false 로 두면 핀 e-stop 비활성

// ★[1015-1] E-stop 시험 스위치★ (헤더 [1015-1])
//   true  : 수동조종에서도 E-stop 이 리니어 2단을 문다 (0909 동작)
//   false : 자율주행에서만 E-stop 이 효력을 갖는다 (1007 동작 = 대회 규정 AS-Off)
//   ★대회에는 false 로 굽는다★
const bool E_STOP_TEST = true;

const uint8_t MODE_PIN = 5;          // 주행모드 스위치 (D5 ── 스위치 ── GND)

// ★[1016-B] 제동등★ — 켜짐 = A5 스위치 닫힘(LOW) 또는 1·2단 지령 (헤더 [1016-B])
//   배선 A5 ── 스위치 ── GND, INPUT_PULLUP (D5·D12 와 같은 꼴)
const uint8_t BRAKE_SW_PIN   = A5;   // 제동등 스위치 (LOW = 닫힘 = 켬)
const uint8_t BRAKELIGHT_PIN = 11;   // 제동등 출력 (모터드라이버 PWM 선, digitalWrite 만)



// ================= 방향 규약 =================
#define DIR_CW   HIGH   // 조향 왼쪽 (raw 증가 방향)
#define DIR_CCW  LOW    // 조향 오른쪽 (raw 감소 방향)

// 리니어 : 정방향(밟는 방향, 로드가 나옴) = LOW / 역방향(놓는 방향, 들어감) = HIGH
#define LINEAR_FWD  LOW
#define LINEAR_REV  HIGH


// ================= 조향 각도 범위 =================
const int STEER_ANGLE_MAX =  40;
const int STEER_ANGLE_MIN = -STEER_ANGLE_MAX;

// ★코드 기본값★ EEPROM 에 저장된 값이 있으면 그쪽이 우선한다([0813-2]).
const int DEF_RAW_LEFT_LIMIT  = 684;   // 왼쪽 끝 (하드 리밋)
const int DEF_RAW_RIGHT_LIMIT = 394;   // 오른쪽 끝 (하드 리밋)

// 캘리브레이션이 쓰는 반폭. 684/394 의 반폭(145)과 같게 두어 행정 크기를 보존한다.
const int STEER_HALF_SPAN = 145;

const int SAFETY_MARGIN = 10;   // 하드 리밋에서 안쪽으로 두는 여유(raw 카운트)

// ★런타임 값★ setup 에서 EEPROM 또는 코드값으로 채운다. 캘리브레이션이 갱신한다.
int raw_left_limit  = DEF_RAW_LEFT_LIMIT;
int raw_right_limit = DEF_RAW_RIGHT_LIMIT;
int pot_at_angle_min = 0;   // 각도 −40 에 대응하는 raw (왼쪽)
int pot_at_angle_max = 0;   // 각도 +40 에 대응하는 raw (오른쪽)
int raw_hi_limit = 0;       // 하드 리밋 게이팅용 (큰 쪽)
int raw_lo_limit = 0;       //                    (작은 쪽)

// 위 파생값을 한꺼번에 다시 계산한다(코드값·EEPROM·캘리브레이션 공통 경로).
void applySteerLimits() {
  pot_at_angle_min = raw_left_limit  - SAFETY_MARGIN;
  pot_at_angle_max = raw_right_limit + SAFETY_MARGIN;
  raw_hi_limit = (raw_left_limit > raw_right_limit) ? raw_left_limit  : raw_right_limit;
  raw_lo_limit = (raw_left_limit > raw_right_limit) ? raw_right_limit : raw_left_limit;
}


// ================= 조향 PD (0804 와 동일) =================
float KP_S = 6.0f;
float KD_S = 0.1f;
const int STEER_MIN_PWM = 110;
const int STEER_MAX_PWM = 255;
const unsigned long CONTROL_WINDOW_MS = 20;

const uint8_t POT_MEDIAN_N = 9;   // 반드시 홀수
const float STEER_ADC_SMOOTH_ALPHA = 0.3;
float steerAdcFiltered = -1;
int   lastPotMedian = 512;

const int STEER_TOLERANCE_ENTER = 3;
const int STEER_TOLERANCE_EXIT  = 6;
const unsigned long SETTLE_MS = 500;

enum CtrlState { ST_ACTIVE, ST_SETTLED };
CtrlState steer_state = ST_SETTLED;

int  steer_angle_cmd = 0;
int  target_pos = 512;
int  prev_pos   = 512;
bool settleTimerRunning = false;
unsigned long settleStart = 0;
unsigned long steer_win_t = 0;


// ================= ★[1016] 브레이크 : 시간 제어 2단★ =================
const uint8_t BRAKE_NONE = 0;
const uint8_t BRAKE_SOFT = 1;
const uint8_t BRAKE_FULL = 2;
const uint8_t BRAKE_LEVEL_MAX = 2;
// 이동이 도중에 끊겨(E-stop·수동 전환·캘리브레이션) 리니어가 어디 있는지 모른다
const uint8_t BRAKE_UNKNOWN = 255;

// ★[1016] 단계 전이 시간 [ms] — 고치는 곳은 여기뿐★ (PWM 은 BRAKE_PWM 255 고정)
const unsigned long LIN_MS_0_1 = 200;   // 0 → 1단 : 나옴
const unsigned long LIN_MS_0_2 = 400;   // 0 → 2단 : 나옴
const unsigned long LIN_MS_1_2 = 300;   // 1 → 2단 : 나옴
const unsigned long LIN_MS_1_0 = 500;   // 1 → 0단 : 들어감
const unsigned long LIN_MS_2_0 = 500;   // 2 → 0단 : 들어감
//   2 → 1단 = 2 → 0단(LIN_MS_2_0 들어감) 뒤 0 → 1단(LIN_MS_0_1 나옴)

const int BRAKE_PWM = 255;              // ★체결·해제의 세기★ — 하나뿐이다

uint8_t brake_cmd_level = BRAKE_NONE;   // 상위에서 지시한 단계
uint8_t brake_level     = BRAKE_NONE;   // 지금 있다고 보는 단계 (BRAKE_UNKNOWN = 모름)
int     brake_output    = 0;            // 진단용 (지금 리니어에 내는 PWM)

//  LIN_MOVE 는 구간이 최대 둘이다(2 → 1단 : 들어감 → 나옴). 첫 구간이 끝나면 둘째로 잇는다.
enum LinState { LIN_IDLE, LIN_MOVE };
LinState lin_state = LIN_IDLE;
uint8_t  lin_tgt_level = BRAKE_NONE;
unsigned long lin_end_t = 0;      // 지금 구간이 끝나는 시각
uint8_t  lin_dir2 = HIGH;         // 둘째 구간 방향
unsigned long lin_ms2 = 0;        // 둘째 구간 시간 (0 = 없음)


// ================= E-stop ([0904-1] 확인시간 100ms, 발동·해제 대칭) =================
// 발동 : 100ms 연속 개방(HIGH) / 해제 : 100ms 연속 단락(LOW).
//   중간에 반대 레벨이 한 번이라도 관측되면 해당 타이머는 0으로 리셋된다(= '연속'이 조건).
//   ★A보드(kasa_0904_A.ino)의 같은 상수와 항상 같은 값으로 유지할 것★
//
//   ★[1007-1] 둘로 갈랐다★
//     estop_line   : D12 확인 결과. 모드와 무관하게 updateEstop 이 늘 갱신한다(종전 estop_active).
//     estop_active : ★효력★ = estopEffective() — loop 가 매 루프 다시 계산한다.
//                    false : estop_line && auto_mode / ★true : estop_line 그대로★ [1015-1]
//                    applyEstop·해제 분기·"STOP"·명령 무시·'a' 금지가 전부 이것을 본다.
bool estop_line   = false;
bool estop_active = false;
bool estop_latched = false;
const unsigned long ESTOP_TRIGGER_CONFIRM_MS = 100;
const unsigned long ESTOP_RELEASE_CONFIRM_MS = 100;
unsigned long estop_high_t = 0;
unsigned long estop_low_t  = 0;


// ================= 주행모드 스위치 =================
bool auto_mode = false;
uint8_t mode_pin_last = HIGH;
unsigned long mode_change_t = 0;
const unsigned long MODE_CONFIRM_MS = 50;

// ★[1015-1] E-stop 의 효력★ — 판정의 단일 소유자. loop 의 estop_active 와 updateMode 의
//   0단 복귀 판단이 같은 식을 쓴다(어긋나면 'STOP 인데 리니어는 들어가 있는' 틈이 생긴다).
//   E_STOP_TEST = false 면 1007 의 estop_line && auto_mode 와 같다.
bool estopEffective() {
  return estop_line && (auto_mode || E_STOP_TEST);
}


// ================= ★[0813-2] 캘리브레이션★ =================
const unsigned long CAL_WATCH_MS = 1200;   // 이 시간 동안 지켜보며 최빈값을 잡는다
const int  CAL_BIN      = 8;               // 최빈값 히스토그램 칸 폭 (잡음 ±10 을 묶는다)
const int  CAL_BIN_LO   = 200;             // 히스토그램 범위 (조향 pot 의 현실적 구간)
const int  CAL_BIN_HI   = 840;
const uint8_t CAL_BINS  = (CAL_BIN_HI - CAL_BIN_LO) / CAL_BIN + 1;

bool cal_running = false;
unsigned long cal_end_t = 0;
uint8_t cal_hist[CAL_BINS];

// EEPROM 배치 : [매직 2B][좌 2B][우 2B][체크섬 1B]
//   ★구조체를 쓰지 않는다★ 아두이노 IDE 는 함수 프로토타입을 파일 맨 위(구조체 정의보다
//   앞)에 자동 삽입한다. 그래서 사용자 정의 타입을 함수 인자에 쓰면 "does not name a
//   type" 컴파일 오류가 난다. 기본형만 주고받으면 그 문제가 원천적으로 없다.
const int  EE_ADDR_MAGIC = 0;
const int  EE_ADDR_LEFT  = 2;
const int  EE_ADDR_RIGHT = 4;
const int  EE_ADDR_SUM   = 6;
const uint16_t EE_MAGIC  = 0x4B31;   // 'K1' — 이 판의 형식임을 표시

uint8_t calChecksum(uint16_t magic, int16_t l, int16_t r) {
  return (uint8_t)(magic ^ (magic >> 8) ^ l ^ (l >> 8) ^ r ^ (r >> 8) ^ 0xA5);
}


// ================= 출력용 =================
unsigned long tele_t = 0;
char rxBuf[24];
uint8_t rxLen = 0;


// ================= ★[0904-2] 연결확인 핸드셰이크 ('-' → "YES")★ =================
// 상위(nxde/arduino.py identify_port, nxde/check.py)가 ★PC→보드 방향이 실제로
// 살아 있는지★ 확인하는 유일한 수단이다. 텔레메트리는 보드→PC 한 방향만 증명한다.
//   '-' 한 줄  → "YES" 한 줄을 즉시 찍고, 이 시간 동안 평상시 텔레메트리를 멈춘다.
//   멈추는 이유는 상위가 "YES" 를 'P,...' 에 섞이지 않은 채로 읽게 하기 위함이다.
//   ★millis() 로만 센다 — delay() 를 쓰지 않는다★ 유예 중에도 조향 PD·리니어·
//   E-stop 판정은 평소대로 돈다. 멈추는 것은 출력 한 줄뿐이고, 끝나면 상태가 남지 않는다.
const unsigned long HANDSHAKE_HOLD_MS = 2000;
bool          handshake_hold = false;   // 지금 유예 중인가
unsigned long handshake_t    = 0;       // "YES" 를 찍은 시각

// ★진단이 필요할 때만 true★ 출력 양식("P,...")을 지키기 위해 기본은 false.
//   '#' 로 시작하므로 파서는 무시한다(이 저장소의 관례).
const bool DEBUG_B = false;


// ================= 선언 =================
int  readPotMedian();
int  smoothPot(int med);
int  angleToPot(int angle);
int  potToAngle(int raw);
void releaseSteer();
void updateSteer(unsigned long now);
void updateBrake(unsigned long now);
void updateLinear(unsigned long now);
void startLinMove(uint8_t lv, unsigned long now);
void updateEstop(unsigned long now);
void applyEstop(unsigned long now);
void linAbort();                          // [1016]
void updateBrakeLight();                  // [1016-B]
bool isValidNumber(const char* s);
bool isReleaseToken(const char* s);
bool isCalToken(const char* s);
bool isHandshakeToken(const char* s);
void handleLine(char* line);
void pollSerial();
void updateMode(unsigned long now);
void retractForManual(unsigned long now);   // [1007-2]
void sendOutput(unsigned long now);
int  readSteerAngle();
void startCal(unsigned long now);
void updateCal(unsigned long now);
bool loadCal();
void saveCal();


// ================= 모터 출력 =================
void dcStop()     { analogWrite(DC_PWM_PIN, 0); }
void dcCW(int p)  { digitalWrite(DC_DIR_PIN, DIR_CW);  analogWrite(DC_PWM_PIN, constrain(p, 0, 255)); }
void dcCCW(int p) { digitalWrite(DC_DIR_PIN, DIR_CCW); analogWrite(DC_PWM_PIN, constrain(p, 0, 255)); }

void linearStop() {
  analogWrite(LINEAR_PWM_PIN, 0);
  brake_output = 0;
}

// ★DIR 을 먼저 쓰고 PWM 을 나중에 쓴다★ 순서가 반대면 전환 순간 잘못된 방향으로 힘이 든다.
void linearDrive(uint8_t dir, int pwm) {
  digitalWrite(LINEAR_DIR_PIN, dir);
  brake_output = constrain(pwm, 0, 255);
  analogWrite(LINEAR_PWM_PIN, brake_output);
}


// ================= 가변저항 읽기 =================
// 삽입정렬 중앙값 — N 이 작아 이게 제일 싸다. 스파이크에 사실상 면역이다.
static int medianOf(uint8_t pin, uint8_t n) {
  int s[9];                       // POT_MEDIAN_N 이하
  if (n > 9) n = 9;
  for (uint8_t i = 0; i < n; i++) {
    int v = analogRead(pin);
    uint8_t j = i;
    while (j > 0 && s[j - 1] > v) { s[j] = s[j - 1]; j--; }
    s[j] = v;
  }
  return s[n / 2];
}

int readPotMedian()    { return medianOf(DC_POT_PIN, POT_MEDIAN_N); }

int smoothPot(int med) {
  if (steerAdcFiltered < 0) steerAdcFiltered = med;
  else steerAdcFiltered += STEER_ADC_SMOOTH_ALPHA * ((float)med - steerAdcFiltered);
  return (int)steerAdcFiltered;
}


// ================= 각도 <-> raw =================
int angleToPot(int angle) {
  angle = constrain(angle, STEER_ANGLE_MIN, STEER_ANGLE_MAX);
  return map(angle, STEER_ANGLE_MIN, STEER_ANGLE_MAX, pot_at_angle_min, pot_at_angle_max);
}

int potToAngle(int raw) {
  int angle = map(raw, pot_at_angle_min, pot_at_angle_max, STEER_ANGLE_MIN, STEER_ANGLE_MAX);
  return constrain(angle, STEER_ANGLE_MIN, STEER_ANGLE_MAX);
}

int readSteerAngle() { return potToAngle(lastPotMedian); }


// ================= 조향 =================
void releaseSteer() {
  dcStop();
  steer_state = ST_SETTLED;
  settleTimerRunning = false;
  target_pos = lastPotMedian;
  steer_angle_cmd = potToAngle(lastPotMedian);
}

void updateSteer(unsigned long now) {
  if (now - steer_win_t < CONTROL_WINDOW_MS) return;
  float dt = (now - steer_win_t) / 1000.0f;
  dt = constrain(dt, 0.005f, 0.2f);
  steer_win_t = now;

  int med = readPotMedian();
  lastPotMedian = med;
  int cur = smoothPot(med);

  if (steer_state == ST_SETTLED) {   // 대기 : 가변저항 변화 무시, 정지 유지
    dcStop();
    prev_pos = cur;
    return;
  }

  int err = target_pos - cur;
  float p = KP_S * (float)err;
  float d = -KD_S * ((float)(cur - prev_pos) / dt);
  float output = p + d;
  prev_pos = cur;

  int absErr = abs(err);

  if (absErr <= STEER_TOLERANCE_ENTER) {
    dcStop();
    if (!settleTimerRunning) {
      settleTimerRunning = true;
      settleStart = now;
    } else if (now - settleStart >= SETTLE_MS) {
      steer_state = ST_SETTLED;
    }
  } else if (absErr > STEER_TOLERANCE_EXIT) {
    settleTimerRunning = false;
    int spd = constrain((int)fabs(output), STEER_MIN_PWM, STEER_MAX_PWM);
    bool wantRawUp = (output > 0);

    // 하드 리밋 게이팅 : 파고드는 방향만 차단, 벗어나는 방향은 허용
    if (wantRawUp && med >= raw_hi_limit)       dcStop();
    else if (!wantRawUp && med <= raw_lo_limit) dcStop();
    else if (wantRawUp)                          dcCW(spd);
    else                                         dcCCW(spd);
  } else {
    dcStop();   // 죽은 영역 : 정지하되 타이머는 유지
  }
}


// ================= ★[1016] 브레이크 : 시간 제어 2단★ =================
// 단계 이동 시작. ★위치를 보지 않는다★ — 지금 단계(brake_level)와 목표 단계로 시간표를 고른다.
//   지금 단계를 모르면(BRAKE_UNKNOWN) : 0단 = 끝까지 들어감 / 2단 = 0→2 시간만큼 나옴(더 밀려도
//   2단 쪽 끝에 닿을 뿐이다) / 1단 = 먼저 끝까지 들어간 뒤 0→1.
void startLinMove(uint8_t lv, unsigned long now) {
  uint8_t from = brake_level;
  unsigned long home_ms = (LIN_MS_2_0 > LIN_MS_1_0) ? LIN_MS_2_0 : LIN_MS_1_0;

  lin_tgt_level = lv;
  lin_ms2 = 0;
  if (from == lv) {                  // 이미 그 단계 — 구동할 것이 없다
    linearStop();
    lin_state = LIN_IDLE;
    return;
  }

  uint8_t dir;
  unsigned long ms;
  if (lv == BRAKE_NONE) {
    dir = LINEAR_REV;
    ms  = (from == BRAKE_SOFT) ? LIN_MS_1_0 : (from == BRAKE_FULL) ? LIN_MS_2_0 : home_ms;
  } else if (lv == BRAKE_FULL) {
    dir = LINEAR_FWD;
    ms  = (from == BRAKE_SOFT) ? LIN_MS_1_2 : LIN_MS_0_2;
  } else if (from == BRAKE_NONE) {   // 0 → 1단
    dir = LINEAR_FWD;
    ms  = LIN_MS_0_1;
  } else {                           // 2 → 1단 (또는 모름 → 1단) : 끝까지 들어갔다가 0 → 1단
    dir = LINEAR_REV;
    ms  = (from == BRAKE_FULL) ? LIN_MS_2_0 : home_ms;
    lin_dir2 = LINEAR_FWD;
    lin_ms2  = LIN_MS_0_1;
  }

  lin_end_t = now + ms;
  linearDrive(dir, BRAKE_PWM);
  lin_state = LIN_MOVE;
}

// 구동 중 종료 판정. ★시간 하나뿐이다★ 첫 구간이 끝나면 둘째 구간이 있으면 잇는다.
void updateLinear(unsigned long now) {
  if (lin_state != LIN_MOVE) return;
  if ((long)(now - lin_end_t) < 0) return;

  if (lin_ms2 > 0) {
    lin_end_t = now + lin_ms2;
    linearDrive(lin_dir2, BRAKE_PWM);
    lin_ms2 = 0;
    return;
  }
  linearStop();
  lin_state = LIN_IDLE;
  brake_level = lin_tgt_level;
}

// 진행 중인 이동을 끊는다. ★끊긴 자리는 모른다★ → BRAKE_UNKNOWN (다음 이동이 그에 맞게 고른다)
void linAbort() {
  if (lin_state == LIN_IDLE) return;
  linearStop();
  lin_state = LIN_IDLE;
  lin_ms2 = 0;
  brake_level = BRAKE_UNKNOWN;
}

// 지시 단계와 지금 단계가 다르면 이동을 건다(구동 중에는 끝난 뒤에).
void updateBrake(unsigned long now) {
  if (lin_state != LIN_IDLE) return;
  if (brake_cmd_level != brake_level) startLinMove(brake_cmd_level, now);
}

// ★[1016-B] 제동등 : A5 스위치 닫힘(LOW) 또는 1·2단 지령★ — 어떤 분기에도 걸지 않고 매 루프 부른다.
bool brakelight_on = false;   // [1016-C] 텔레메트리 상태 1 의 출처
void updateBrakeLight() {
  bool on = (digitalRead(BRAKE_SW_PIN) == LOW)
         || brake_cmd_level == BRAKE_SOFT || brake_cmd_level == BRAKE_FULL;
  brakelight_on = on;
  digitalWrite(BRAKELIGHT_PIN, on ? HIGH : LOW);
}


// ================= E-stop =================
// ★[1007-1] 이 함수는 D12 라인만 판정한다(estop_line)★ 모드는 보지 않는다 — 효력
//   (estop_active = estop_line && auto_mode)은 loop 가 updateMode 뒤에 계산한다.
//   수동에서도 계속 세 두므로, 자율로 넘기는 순간 100ms 확인을 다시 기다리지 않는다.
void updateEstop(unsigned long now) {
  if (!ESTOP_ENABLED) {
    estop_line   = false;
    estop_high_t = 0;
    estop_low_t  = 0;
    return;
  }

  bool open_now = (digitalRead(ESTOP_PIN) == HIGH);   // NC 개방 = e-stop 요청

  if (!estop_line) {
    estop_low_t = 0;
    if (!open_now) {
      estop_high_t = 0;
    } else if (estop_high_t == 0) {
      estop_high_t = now;
    } else if (now - estop_high_t >= ESTOP_TRIGGER_CONFIRM_MS) {
      estop_line   = true;
      estop_high_t = 0;
    }
  } else {
    estop_high_t = 0;
    if (open_now) {
      estop_low_t = 0;
    } else if (estop_low_t == 0) {
      estop_low_t = now;
    } else if (now - estop_low_t >= ESTOP_RELEASE_CONFIRM_MS) {
      estop_line  = false;
      estop_low_t = 0;
    }
  }
}

// ★E-stop = 2단 체결★ 조향은 힘빼기, 브레이크는 최대.
void applyEstop(unsigned long now) {
  dcStop();
  steer_angle_cmd = 0;

  if (!estop_latched) {
    estop_latched = true;
    cal_running = false;              // 캘리브레이션 중이었다면 취소
    linAbort();                       // 진행 중이던 이동을 끊고 다시 건다 ([1016] 자리는 모름)
    brake_cmd_level = BRAKE_FULL;
    startLinMove(BRAKE_FULL, now);
  }
  brake_cmd_level = BRAKE_FULL;
  updateLinear(now);
}


// ================= ★[0813-2] 캘리브레이션★ =================
void startCal(unsigned long now) {
  cal_running = true;
  cal_end_t = now + CAL_WATCH_MS;
  for (uint8_t i = 0; i < CAL_BINS; i++) cal_hist[i] = 0;
  dcStop();
  linAbort();                         // [1016] 이동 중이었다면 끊는다 — 끝난 뒤 updateBrake 가 다시 건다
  steer_state = ST_SETTLED;           // 지켜보는 동안 조향은 정지
  settleTimerRunning = false;
  // ★항상 찍는다★ 사람이 손으로 하는 일회성 조작이라 확인이 필요하다.
  //   '#' 로 시작하므로 상위 파서는 무시한다(arduino.py 의 parse_b 는 'P,' 로
  //   시작하지 않는 줄을 조용히 버린다) — 출력 양식은 깨지지 않는다.
  Serial.println(F("# CAL start (1.2s) — 핸들을 일자로 유지할 것"));
}

void updateCal(unsigned long now) {
  int raw = readPotMedian();
  lastPotMedian = raw;                // 텔레메트리는 계속 실측을 내보낸다

  if (raw >= CAL_BIN_LO && raw <= CAL_BIN_HI) {
    uint8_t b = (uint8_t)((raw - CAL_BIN_LO) / CAL_BIN);
    if (b < CAL_BINS && cal_hist[b] < 255) cal_hist[b]++;
  }

  if ((long)(now - cal_end_t) < 0) return;

  // ★최빈값 채택★ 평균이 아니라 최빈값인 이유 : 스파이크 하나가 평균을 끌고 간다.
  uint8_t best = 0;
  uint8_t bestCnt = 0;
  for (uint8_t i = 0; i < CAL_BINS; i++) {
    if (cal_hist[i] > bestCnt) { bestCnt = cal_hist[i]; best = i; }
  }
  cal_running = false;

  if (bestCnt == 0) {                 // 유효 표본이 없다 — 그냥 되돌아간다
    Serial.println(F("# CAL FAIL — 유효 표본 없음 (배선·범위 확인). 기존값 유지"));
  } else {
    int center = CAL_BIN_LO + (int)best * CAL_BIN + CAL_BIN / 2;
    raw_left_limit  = center + STEER_HALF_SPAN;
    raw_right_limit = center - STEER_HALF_SPAN;
    applySteerLimits();
    saveCal();
    // ★결과는 항상 찍는다★ 이 한 줄이 '캘리브레이션이 실제로 됐는가'의 유일한 증거다.
    Serial.print(F("# CAL OK center="));  Serial.print(center);
    Serial.print(F(" L="));               Serial.print(raw_left_limit);
    Serial.print(F(" R="));               Serial.print(raw_right_limit);
    Serial.println(F(" (EEPROM 저장됨)"));
  }

  // 끝난 자리를 목표로 잡고 대기로 — 캘리브레이션 직후 급조향을 막는다
  int cur = readPotMedian();
  lastPotMedian    = cur;
  steerAdcFiltered = cur;
  target_pos = cur;
  prev_pos   = cur;
  steer_state = ST_SETTLED;
  settleTimerRunning = false;
}

// EEPROM 이 우선. 매직·체크섬이 맞을 때만 채택한다(깨진 값으로 조향하면 위험하다).
bool loadCal() {
  uint16_t magic;
  int16_t  l, r;
  uint8_t  sum;
  EEPROM.get(EE_ADDR_MAGIC, magic);
  EEPROM.get(EE_ADDR_LEFT,  l);
  EEPROM.get(EE_ADDR_RIGHT, r);
  EEPROM.get(EE_ADDR_SUM,   sum);

  if (magic != EE_MAGIC) return false;
  if (sum != calChecksum(magic, l, r)) return false;
  if (l == r) return false;

  raw_left_limit  = l;
  raw_right_limit = r;
  return true;
}

void saveCal() {
  uint16_t magic = EE_MAGIC;
  int16_t  l = (int16_t)raw_left_limit;
  int16_t  r = (int16_t)raw_right_limit;
  uint8_t  sum = calChecksum(magic, l, r);
  // put 은 값이 같으면 쓰지 않는다(수명 보호)
  EEPROM.put(EE_ADDR_MAGIC, magic);
  EEPROM.put(EE_ADDR_LEFT,  l);
  EEPROM.put(EE_ADDR_RIGHT, r);
  EEPROM.put(EE_ADDR_SUM,   sum);
}


// ================= 입력 형식 검사 =================
bool isValidNumber(const char* s) {
  if (!s || *s == '\0') return false;
  uint8_t i = 0;
  if (s[0] == '-' || s[0] == '+') i = 1;
  if (s[i] == '\0') return false;
  for (; s[i] != '\0'; i++) {
    if (!isdigit((unsigned char)s[i])) return false;
  }
  return true;
}

bool isReleaseToken(const char* s) {
  return s && (s[0] == 'x' || s[0] == 'X') && s[1] == '\0';
}

bool isCalToken(const char* s) {
  return s && (s[0] == 'a' || s[0] == 'A') && s[1] == '\0';
}

// ★[0904-2] 핸드셰이크 토큰★ '-' ★한 글자로만★ 이루어진 줄에서만 성립한다.
//   그래서 "-12,0" 같은 좌조향 명령과 겹치지 않는다(그쪽은 두 번째 글자가 '1' 이다).
bool isHandshakeToken(const char* s) {
  return s && s[0] == '-' && s[1] == '\0';
}


// ================= 한 줄 처리 =================
void handleLine(char* line) {
  // ★[0904-2] "-" 한 줄 = 연결확인 핸드셰이크★ 조향·제동에는 아무 영향이 없다.
  //   ★e-stop 중에도, 캘리브레이션 중에도 받는다★ 구동 명령이 아니라 '통신이 살아
  //   있는가' 의 답이고, E-stop 이 걸린 채로 연결을 확인해야 하는 상황이 실제로 있다.
  if (isHandshakeToken(line)) {
    Serial.println("YES");
    handshake_hold = true;
    handshake_t = millis();
    return;
  }

  // ★[0813-2] "a" 한 줄 = 조향 영점 캘리브레이션★ (E-stop 중에는 받지 않는다)
  if (isCalToken(line)) {
    if (!estop_active) startCal(millis());
    return;
  }

  char* tok1 = strtok(line, ",");
  char* tok2 = tok1 ? strtok(NULL, ",") : NULL;
  char* tok3 = tok2 ? strtok(NULL, ",") : NULL;   // 토큰이 3개 이상이면 형식 오류

  if (!tok1 || !tok2 || tok3 || !isValidNumber(tok2)) return;

  bool release = isReleaseToken(tok1);
  if (!release && !isValidNumber(tok1)) return;

  int brake = atoi(tok2);

  // e-stop 중에는 구동 명령(조향/브레이크) 미적용 → 리니어 재구동 방지
  //   ★estop_active 는 효력이다(estopEffective)★ E_STOP_TEST = false 면 수동에서는 E-stop 이
  //   눌려 있어도 여기를 지나간다 — 아래에서 브레이크만 0 으로 받는다.
  if (estop_active) return;
  // 캘리브레이션 중에도 받지 않는다 — 핸들을 사람이 잡고 있는 상황이다
  if (cal_running) return;

  if (release) {
    releaseSteer();
  } else {
    steer_angle_cmd = constrain(atoi(tok1), STEER_ANGLE_MIN, STEER_ANGLE_MAX);
    target_pos = angleToPot(steer_angle_cmd);
    settleTimerRunning = false;
    steer_state = ST_ACTIVE;
  }

  // ★[1007-2] 수동조종에서는 브레이크 지시를 받지 않는다 — 리니어는 들어간 채로 둔다★
  //   (규정 2.16.1 다 : ASMS Off 에서는 AS 요청으로 제동 불가). 조향은 그대로 받는다 —
  //   수동에서는 D5 스위치가 조향 구동 배선을 물리적으로 끊는다.
  if (!auto_mode) brake = BRAKE_NONE;

  if (brake >= BRAKE_NONE && brake <= BRAKE_LEVEL_MAX) {
    brake_cmd_level = (uint8_t)brake;
  }
}

void pollSerial() {
  while (Serial.available() > 0) {
    char ch = (char)Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') {
      rxBuf[rxLen] = '\0';
      if (rxLen > 0) handleLine(rxBuf);
      rxLen = 0;
    } else if (rxLen < sizeof(rxBuf) - 1) {
      rxBuf[rxLen++] = ch;
    } else {
      rxLen = 0;
    }
  }
}


// ================= 주행모드 스위치 =================
// ★[1007-1] 이제 '보고 전용' 이 아니다★ auto_mode 가 E-stop 의 효력과 브레이크 지시
//   수용을 정한다. 자율→수동으로 확정되는 순간 리니어를 0단으로 되돌린다([1007-2]).
void updateMode(unsigned long now) {
  uint8_t lv = digitalRead(MODE_PIN);

  if (lv != mode_pin_last) {
    mode_pin_last = lv;
    mode_change_t = now;
  } else if (mode_change_t != 0 && now - mode_change_t >= MODE_CONFIRM_MS) {
    bool was_auto = auto_mode;
    auto_mode = (lv == LOW);
    mode_change_t = 0;
    // ★[1015-1] E-stop 이 효력을 갖는 동안에는 빼지 않는다★ (E_STOP_TEST = true 에서만 생긴다)
    //   2단은 applyEstop 이 계속 지키고, E-stop 이 풀리면 loop 의 해제 분기가 0단으로 되돌린다.
    if (was_auto && !auto_mode && !estopEffective()) retractForManual(now);
  }
}

// ★[1007-2] 수동조종으로 들어가면 리니어를 끝까지 넣어 둔다★ (자율→수동 엣지 · 수동 부팅)
//   · 진행 중인 이동은 그 자리에서 끊는다(linAbort).
//   · ★[1016] brake_level 을 '모름' 으로 두고 0단으로 내린다★ — 들어감 시간 중 긴 쪽(0.5 s)이라
//     어디 있든 끝까지 들어간다. 이미 들어가 있어도 무해하다(리니어는 끝까지 들어가도 문제가 없다).
//   · 곧바로 startLinMove 를 건다 — 이 루프의 분기를 기다리지 않는다. 도중에 캘리브레이션
//     ('a')이 구동을 끊어도 다시 '모름' 이 되어 끝난 뒤 updateBrake 가 다시 건다.
void retractForManual(unsigned long now) {
  linAbort();
  brake_cmd_level = BRAKE_NONE;
  brake_level     = BRAKE_UNKNOWN;    // [1016] 어디 있든 끝까지 넣는다 (들어감 LIN_MS_2_0·1_0 중 긴 쪽)
  startLinMove(BRAKE_NONE, now);
  if (DEBUG_B) Serial.println(F("# MANUAL — 리니어 0단 복귀"));
}


// ================= 출력 =================
// ★[1016-C] "P,<실측조향각>,<상태>,<주행모드>" 하나뿐이다★ — "STOP" 줄은 없다.
//   상태 : 2 = E-stop 효력(estop_active) / 1 = 제동등 조건(brakelight_on) / 0 = 그 밖. 2 가 이긴다.
//   ★estop_active 는 효력이다★ — E_STOP_TEST = false 면 수동에서는 E-stop 이 눌려 있어도 2 가 아니다
//     (규정 AS-Off). 모드 칸은 실제 D5 다.
// ★[0904-2] 핸드셰이크 유예 중에는 평상시 줄을 내보내지 않는다★
//   ★단 E-stop 중이면 내보낸다★ 여기서 막으면 2초 동안 비상정지가 상위에 보이지 않는다
//   (종전 "STOP" 과 같은 규칙).
void sendOutput(unsigned long now) {
  if (now - tele_t < TELE_MS) return;
  tele_t = now;

  if (handshake_hold) {
    if (now - handshake_t < HANDSHAKE_HOLD_MS) {
      if (!estop_active) return;   // E-stop 은 유예 중에도 알린다
    } else {
      handshake_hold = false;      // 유예 종료 — 다음 줄부터 원래대로
    }
  }

  int state = estop_active ? 2 : (brakelight_on ? 1 : 0);

  Serial.print("P,");
  Serial.print(readSteerAngle());
  Serial.print(',');
  Serial.print(state);                    // ★[1016-C] 2 E-stop / 1 제동등 / 0★
  Serial.print(',');
  Serial.println(auto_mode ? 1 : 0);
}


// ================= setup =================
void setup() {
  Serial.begin(BAUD);

  pinMode(DC_DIR_PIN, OUTPUT);
  pinMode(DC_PWM_PIN, OUTPUT);
  pinMode(DC_POT_PIN, INPUT);
  digitalWrite(DC_DIR_PIN, DIR_CW);
  dcStop();

  pinMode(LINEAR_DIR_PIN, OUTPUT);
  pinMode(LINEAR_PWM_PIN, OUTPUT);
  digitalWrite(LINEAR_DIR_PIN, LINEAR_FWD);
  linearStop();

  pinMode(ESTOP_PIN, INPUT_PULLUP);
  estop_line   = false;
  estop_active = false;
  estop_high_t = 0;
  estop_low_t  = 0;

  pinMode(MODE_PIN, INPUT_PULLUP);

  // ★[1016-B] 제동등★ A5 는 스위치 입력(내부 풀업 — 스위치가 GND 로 당긴다), D11 은 출력
  pinMode(BRAKE_SW_PIN, INPUT_PULLUP);
  pinMode(BRAKELIGHT_PIN, OUTPUT);
  digitalWrite(BRAKELIGHT_PIN, LOW);
  mode_pin_last = digitalRead(MODE_PIN);
  auto_mode     = (mode_pin_last == LOW);
  mode_change_t = 0;

  // ★[0813-2] EEPROM 이 코드값보다 우선★ 없거나 깨졌으면 코드값을 쓴다.
  bool fromEE = loadCal();
  applySteerLimits();
  if (DEBUG_B) {
    Serial.print(F("# LIMITS "));
    Serial.print(fromEE ? F("EEPROM ") : F("default "));
    Serial.print(raw_left_limit);
    Serial.print('/');
    Serial.println(raw_right_limit);
  }

  unsigned long now = millis();
  tele_t = now;

  // 브레이크 : 부팅 상태는 '단계 미상'이 아니라 0단으로 두고 대기한다.
  //   상위가 0을 보내도 아무 일이 없고, 1/2를 보내면 그때 움직인다(★자율주행에서만★ — [1007-2]).
  brake_level     = BRAKE_NONE;
  brake_cmd_level = BRAKE_NONE;
  lin_state       = LIN_IDLE;

  // ★[1007-2] 수동조종으로 켜지면 리니어를 끝까지 넣고 시작한다★ 체결 도중에 리셋됐으면
  //   (상위가 포트를 열 때마다 DTR 로 리셋된다) 반쯤 나온 채로 남아 있을 수 있다.
  //   ★자율로 켜진 경우는 종전 그대로 둔다★ — 2단을 물고 서 있던 차가 리셋됐다면 상위가
  //   곧 다시 2단을 지시하는데, 여기서 먼저 빼 버리면 그 사이에 브레이크가 풀린다.
  if (!auto_mode) retractForManual(now);

  updateBrakeLight();        // [1016-B] 부팅 순간의 A5 로 바로 판정한다

  // 조향 PD : 시작 시 현재 위치를 목표로 유지 (대기 상태, 급조향 방지)
  int rawInit = readPotMedian();
  lastPotMedian    = rawInit;
  steerAdcFiltered = rawInit;
  target_pos = rawInit;
  prev_pos   = rawInit;
  steer_win_t = now;

  // ★여기서 안내를 출력하지 않는다★ 출력 양식("P,...")을 그대로 지키기 위함.
}


// ================= loop =================
void loop() {
  unsigned long now = millis();

  // 진행 중인 리니어 구동의 종료 판정 — e-stop 여부와 무관하게 매 루프
  updateLinear(now);

  // ★E-stop 판정은 pollSerial 보다 앞★ 발동 직후 들어온 줄이 한 번 먹는 틈을 없앤다
  updateEstop(now);          // [1007-1] D12 라인(estop_line) — 모드와 무관

  // ★[1007-1] 주행모드도 pollSerial 보다 앞★ 이제 보고 전용이 아니라 E-stop 의 효력과
  //   브레이크 지시 수용을 정한다 — 위와 같은 이유로 판정을 명령보다 먼저 한다.
  //   자율→수동으로 확정되면 여기서 리니어 0단 복귀가 걸린다([1007-2]).
  updateMode(now);

  // ★[1007-1] E-stop 의 효력 = 라인 체결 AND 자율주행 — 엣지가 아니라 매 루프 레벨★
  //   효력이 올라가면 아래 applyEstop 이 2단을 물고, 내려가면 해제 분기가 0단으로 되돌린다.
  //   ★[1015-1] E_STOP_TEST = true 면 모드를 보지 않는다★ (estopEffective)
  estop_active = estopEffective();

  pollSerial();

  if (estop_active) {
    applyEstop(now);
  } else {
    if (estop_latched) {
      estop_latched = false;
      // ★해제는 0단 복귀와 같은 작용★ 송신측이 0을 보내주기를 기다리지 않는다.
      //   ★효력이 내려가는 길★ D12 해제(RUN), 그리고 E_STOP_TEST = false 면 D5 수동 전환도.
      //   어느 쪽이든 여기서 0단으로 되돌린다(수동 전환이면 updateMode 의 retractForManual 도
      //   같은 일을 이미 걸었다 — 같은 수라 겹쳐도 무해하다).
      linAbort();                     // 2단 이동이 아직 끝나지 않았으면 끊는다
      brake_cmd_level = BRAKE_NONE;
      brake_level     = BRAKE_FULL;   // 2단 쪽에 있다고 보고 0단으로 내린다 (2→0 들어감)

      // 조향 PD 목표를 해제 시점의 현재 위치로 재동기화
      int cur = readPotMedian();
      lastPotMedian    = cur;
      steerAdcFiltered = cur;
      target_pos = cur;
      prev_pos   = cur;
      steer_state = ST_SETTLED;
      settleTimerRunning = false;
    }

    if (cal_running) {
      updateCal(now);          // 캘리브레이션 중에는 조향·제동을 건드리지 않는다
    } else {
      updateBrake(now);
      updateSteer(now);
    }
  }

  // ★[1016-B] 제동등 : 어떤 분기에도 걸지 않는다★ 이번 루프의 지령(명령·E-stop·수동 전환)이
  //   반영된 뒤에 판정한다.
  updateBrakeLight();

  sendOutput(now);
}
