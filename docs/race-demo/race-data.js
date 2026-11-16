/* AOC meeting demo. No network, dependencies, or physical simulator is used here. */
(function (global) {
  'use strict';

  const EPS = 1e-7;
  const G = 9.81;
  const cache = new WeakMap();
  const TERMINAL = new Set(['finished', 'collision', 'out_of_bounds', 'skipped_gate',
    'reverse_gate', 'timeout', 'controller_error', 'invalid_action', 'simulation_error']);
  const STATUS_LABELS = Object.freeze({finished: '완주', collision: '충돌',
    out_of_bounds: '영역 이탈', skipped_gate: '게이트 순서 위반', reverse_gate: '역방향 통과',
    timeout: '비행시간 초과', controller_error: '제어기 오류', invalid_action: '잘못된 제어입력',
    simulation_error: '시뮬레이션 오류'});
  const DATA_NOTE = '화면 설명용 합성 운동학 데이터입니다. RotorPy 시뮬레이션 결과나 검증된 비행 성능이 아닙니다.';
  const METRIC_NOTE = '순간 하중은 기록된 속도의 차분으로 추정합니다. 재생 중 누적 하중 벌점도 추정값이며, 종료 점수와 종료 하중 벌점은 원본 summary 값을 사용합니다.';
  const TRACK = {
    name: 'random-10-gates-seed4', start: [0, 0, 1.5],
    gates: [
      [5.386, 0, 1.916, 0], [7.372, 3.076, 2.05, 0.9974],
      [11.146, 6.513, 1.444, 0.7388], [11.392, 11.094, 2.463, 1.5171],
      [11.834, 15.433, 2.305, 1.4693], [8.48, 18.026, 2.557, 2.4833],
      [4.736, 17.109, 2.052, -2.9014], [0.512, 13.768, 2.132, -2.4723],
      [-3.962, 14.21, 1.891, 3.0431], [-9.353, 14.74, 1.69, 3.0435]
    ].map(function (g) {return {center: g.slice(0, 3), yaw_rad: g[3], width_m: 2, height_m: 2, frame_m: 0.1};}),
    obstacles: [], bounds: {lower: [-12.353, -3, 0], upper: [14.834, 21.026, 6]}
  };

  function clone(value) {return JSON.parse(JSON.stringify(value));}
  function norm(v) {return Math.hypot.apply(Math, v);}
  function clamp(value, lo, hi) {return Math.max(lo, Math.min(hi, value));}
  function mix(a, b, f) {return a.map(function (v, j) {return v + (b[j] - v) * f;});}
  function fail(path, message) {throw new Error(path + ': ' + message);}
  function object(value, path) {
    if (value === null || typeof value !== 'object' || Array.isArray(value)) fail(path, '객체가 필요합니다.');
    return value;
  }
  function finite(value, path, lo, hi) {
    if (typeof value !== 'number' || !Number.isFinite(value)) fail(path, '유한한 숫자가 필요합니다.');
    if ((lo !== undefined && value < lo) || (hi !== undefined && value > hi)) fail(path, '허용 범위를 벗어났습니다.');
    return value;
  }
  function integer(value, path, lo, hi) {
    finite(value, path, lo, hi);
    if (!Number.isInteger(value)) fail(path, '정수가 필요합니다.');
    return value;
  }
  function vector(value, length, path) {
    if (!Array.isArray(value) || value.length !== length) fail(path, length + '개 숫자의 배열이 필요합니다.');
    return value.map(function (v, j) {return finite(v, path + '[' + j + ']');});
  }
  function text(value, path, max) {
    if (typeof value !== 'string' || value.length > (max || 200)) fail(path, '짧은 문자열이 필요합니다.');
    if (/[\u0000-\u0008\u000b\u000c\u000e-\u001f]/.test(value)) fail(path, '제어 문자는 사용할 수 없습니다.');
    return value;
  }
  function escapeHTML(value) {
    return String(value).replace(/[&<>"']/g, function (c) {return {'&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;'}[c];});
  }
  function box(value, path) {
    object(value, path);
    const lower = vector(value.lower, 3, path + '.lower');
    const upper = vector(value.upper, 3, path + '.upper');
    if (lower.some(function (n, j) {return n >= upper[j];})) fail(path, 'lower는 upper보다 작아야 합니다.');
    return {lower: lower, upper: upper};
  }

  /* Validate and copy only the supported testbed schema. No HTML is interpreted. */
  function normalizeLog(raw) {
    if (typeof raw === 'string') {
      if (raw.length > 32000000) fail('JSON', '32 MB 이하의 로그를 사용해주세요.');
      try {raw = JSON.parse(raw);} catch (error) {fail('JSON', 'JSON 형식을 읽을 수 없습니다.');}
    }
    object(raw, 'log');
    if (raw.schema_version !== 1) fail('schema_version', '지원하는 값은 1입니다.');
    const sourceConfig = object(raw.config, 'config');
    const sourceTrack = object(sourceConfig.track, 'config.track');
    const radius = finite(sourceConfig.vehicle_radius_m, 'config.vehicle_radius_m', Number.MIN_VALUE);
    if (!Array.isArray(sourceTrack.gates) || !sourceTrack.gates.length || sourceTrack.gates.length > 256) fail('config.track.gates', '1~256개 게이트가 필요합니다.');
    const track = {name: text(sourceTrack.name, 'config.track.name'), start: vector(sourceTrack.start, 3, 'config.track.start'),
      bounds: box(sourceTrack.bounds, 'config.track.bounds'), gates: sourceTrack.gates.map(function (gate, i) {
        const path = 'config.track.gates[' + i + ']'; object(gate, path);
        const output = {center: vector(gate.center, 3, path + '.center'), yaw_rad: finite(gate.yaw_rad, path + '.yaw_rad'),
          width_m: finite(gate.width_m, path + '.width_m', Number.MIN_VALUE), height_m: finite(gate.height_m, path + '.height_m', Number.MIN_VALUE),
          frame_m: finite(gate.frame_m, path + '.frame_m', Number.MIN_VALUE)};
        if (Math.min(output.width_m, output.height_m) <= 2 * radius) fail(path, '게이트 개구부가 기체 지름보다 커야 합니다.');
        return output;
      }), obstacles: []};
    if (!Array.isArray(sourceTrack.obstacles) || sourceTrack.obstacles.length > 1000) fail('config.track.obstacles', '장애물 배열이 필요합니다.');
    track.obstacles = sourceTrack.obstacles.map(function (obstacle, i) {
      object(obstacle, 'obstacle');
      if (obstacle.type !== undefined && obstacle.type !== 'box') fail('config.track.obstacles[' + i + '].type', '현재는 box만 표시할 수 있습니다.');
      return Object.assign({type: 'box'}, box(obstacle, 'config.track.obstacles[' + i + ']'));
    });
    const config = {track: track, vehicle_radius_m: radius};
    ['control_dt_s', 'physics_dt_s', 'nominal_mass_kg', 'gravity_m_s2', 'thrust_max_N', 'body_rate_limit_rad_s', 'body_rate_gain_s_inv'].forEach(function (key) {
      config[key] = finite(sourceConfig[key], 'config.' + key, Number.MIN_VALUE);
    });
    config.thrust_min_N = finite(sourceConfig.thrust_min_N, 'config.thrust_min_N', 0, config.thrust_max_N);
    const ratio = config.control_dt_s / config.physics_dt_s;
    if (Math.round(ratio) < 1 || Math.abs(ratio - Math.round(ratio)) > 1e-5 * Math.max(1, ratio)) fail('config', '제어 주기는 물리 계산 주기의 양의 정수배여야 합니다.');
    const sourceScenario = object(raw.scenario, 'scenario');
    const scenario = {seed: integer(sourceScenario.seed, 'scenario.seed', 0),
      wind_max_m_s: finite(sourceScenario.wind_max_m_s, 'scenario.wind_max_m_s', 0, 2),
      mass_error_fraction: finite(sourceScenario.mass_error_fraction, 'scenario.mass_error_fraction', 0, 0.2),
      max_time_s: finite(sourceScenario.max_time_s, 'scenario.max_time_s', Number.MIN_VALUE),
      load_limit_g: finite(sourceScenario.load_limit_g, 'scenario.load_limit_g', Number.MIN_VALUE),
      load_penalty_s: finite(sourceScenario.load_penalty_s, 'scenario.load_penalty_s', 0),
      failure_penalty_s: finite(sourceScenario.failure_penalty_s, 'scenario.failure_penalty_s', 0)};
    if (!Array.isArray(raw.samples) || raw.samples.length < 1 || raw.samples.length > 200000) fail('samples', '1~200,000개 표본이 필요합니다.');
    let previousTime = -1, previousGate = 0;
    const samples = raw.samples.map(function (sample, i) {
      const path = 'samples[' + i + ']'; object(sample, path);
      const time = finite(sample.time_s, path + '.time_s', 0, scenario.max_time_s + EPS);
      if (time + EPS < previousTime) fail(path + '.time_s', '시간은 감소할 수 없습니다.');
      if (i === 0 && Math.abs(time) > EPS) fail(path + '.time_s', '첫 표본은 0초여야 합니다.');
      const next = integer(sample.next_gate, path + '.next_gate', 0, track.gates.length);
      if (next < previousGate) fail(path + '.next_gate', '다음 게이트 번호는 감소할 수 없습니다.');
      const q = vector(sample.quaternion_xyzw, 4, path + '.quaternion_xyzw');
      if (Math.abs(norm(q) - 1) > 0.05) fail(path + '.quaternion_xyzw', '단위 quaternion이 필요합니다.');
      let action = null;
      if (sample.action !== null && sample.action !== undefined) {
        object(sample.action, path + '.action');
        const rates = vector(sample.action.body_rates_rad_s, 3, path + '.action.body_rates_rad_s');
        if (rates.some(function (n) {return Math.abs(n) > config.body_rate_limit_rad_s + EPS;})) fail(path + '.action', '각속도 명령 한계를 벗어났습니다.');
        action = {thrust_N: finite(sample.action.thrust_N, path + '.action.thrust_N', config.thrust_min_N - EPS, config.thrust_max_N + EPS), body_rates_rad_s: rates};
      }
      previousTime = time; previousGate = next;
      return {time_s: time, position_m: vector(sample.position_m, 3, path + '.position_m'),
        velocity_m_s: vector(sample.velocity_m_s, 3, path + '.velocity_m_s'), quaternion_xyzw: q,
        body_rates_rad_s: vector(sample.body_rates_rad_s, 3, path + '.body_rates_rad_s'), next_gate: next,
        wind_m_s: vector(sample.wind_m_s, 3, path + '.wind_m_s'), action: action};
    });
    if (!Array.isArray(raw.events) || raw.events.length > 20000) fail('events', '이벤트 배열이 필요합니다.');
    let previousEventTime = -1, passed = 0;
    const events = raw.events.map(function (event, i) {
      const path = 'events[' + i + ']'; object(event, path);
      if (event.kind !== 'gate_passed' && !TERMINAL.has(event.kind)) fail(path + '.kind', '알 수 없는 이벤트입니다.');
      const time = finite(event.time_s, path + '.time_s', 0, previousTime + EPS);
      if (time + EPS < previousEventTime) fail(path, '이벤트 시간 순서가 잘못되었습니다.');
      let gate = null;
      if (event.gate !== null && event.gate !== undefined) gate = integer(event.gate, path + '.gate', 0, event.kind === 'timeout' ? track.gates.length : track.gates.length - 1);
      if (event.kind === 'gate_passed') {
        if (gate !== passed) fail(path + '.gate', 'gate_passed 이벤트는 0부터 순서대로 있어야 합니다.');
        passed += 1;
      }
      previousEventTime = time;
      return {kind: event.kind, time_s: time, gate: gate};
    });
    const passEvents = events.filter(function (event) {return event.kind === 'gate_passed';});
    let passCursor = 0;
    samples.forEach(function (sample, i) {
      while (passCursor < passEvents.length && passEvents[passCursor].time_s <= sample.time_s + 1e-9) passCursor += 1;
      if (sample.next_gate !== passCursor) fail('samples[' + i + '].next_gate', '해당 시각의 게이트 통과 이벤트와 일치하지 않습니다.');
    });
    const sourceSummary = object(raw.summary, 'summary');
    if (!TERMINAL.has(sourceSummary.status)) fail('summary.status', '지원하지 않는 종료 상태입니다.');
    if (typeof sourceSummary.completed !== 'boolean' || sourceSummary.completed !== (sourceSummary.status === 'finished')) fail('summary.completed', '완주 상태와 일치해야 합니다.');
    const summary = {status: sourceSummary.status, completed: sourceSummary.completed,
      flight_time_s: finite(sourceSummary.flight_time_s, 'summary.flight_time_s', 0, scenario.max_time_s + EPS),
      gates_passed: integer(sourceSummary.gates_passed, 'summary.gates_passed', 0, track.gates.length),
      gates_total: integer(sourceSummary.gates_total, 'summary.gates_total', 1), error: null};
    if (Math.abs(summary.flight_time_s - previousTime) > 1e-5) fail('summary.flight_time_s', '마지막 표본 시간과 일치해야 합니다.');
    if (summary.gates_total !== track.gates.length || summary.gates_passed !== passed || previousGate !== passed) fail('summary', '게이트 수·이벤트·마지막 표본이 일치하지 않습니다.');
    if (summary.completed && passed !== track.gates.length) fail('summary', '완주에는 모든 게이트 통과가 필요합니다.');
    ['score_s', 'load_penalty_s', 'failure_penalty_s', 'peak_load_g', 'controller_reset_s', 'controller_mean_act_s', 'controller_max_act_s', 'wall_time_s'].forEach(function (key) {
      summary[key] = finite(sourceSummary[key], 'summary.' + key, 0);
    });
    if (sourceSummary.error !== null && sourceSummary.error !== undefined) summary.error = text(sourceSummary.error, 'summary.error', 10000);
    if (summary.completed && !events.some(function (e) {return e.kind === 'finished' && Math.abs(e.time_s - summary.flight_time_s) <= 1e-5;})) fail('events', '완주 종료 이벤트가 필요합니다.');
    if (!summary.completed && events.some(function (e) {return e.kind === 'finished';})) fail('events', '미완주 로그에 finished 이벤트가 있습니다.');
    if (['collision', 'out_of_bounds', 'skipped_gate', 'reverse_gate', 'timeout'].indexOf(summary.status) >= 0) {
      const terminal = events[events.length - 1];
      if (!terminal || terminal.kind !== summary.status || Math.abs(terminal.time_s - summary.flight_time_s) > 1e-5) fail('events', '종료 상태·이벤트·시간이 일치하지 않습니다.');
    }
    const output = {schema_version: 1, config: config, scenario: scenario, samples: samples, events: events, summary: summary};
    if (raw.provenance && typeof raw.provenance === 'object' && !Array.isArray(raw.provenance)) output.provenance = clone(raw.provenance);
    if (raw.realization && typeof raw.realization === 'object' && !Array.isArray(raw.realization)) output.realization = clone(raw.realization);
    if (raw.synthetic === true) output.synthetic = true;
    if (raw.data_kind === 'illustrative_kinematic_data') output.data_kind = raw.data_kind;
    output.metric_note = METRIC_NOTE;
    return output;
  }

  function prepareMetrics(log) {
    if (cache.has(log)) return cache.get(log);
    const results = [], samples = log.samples, gravity = log.config.gravity_m_s2;
    let cumulative = 0;
    for (let i = 0; i < samples.length; i += 1) {
      const sample = samples[i]; let load = null;
      if (i > 0) {
        const before = samples[i - 1], dt = sample.time_s - before.time_s;
        if (dt > 1e-12) {
          const acc = sample.velocity_m_s.map(function (v, j) {return (v - before.velocity_m_s[j]) / dt + (j === 2 ? gravity : 0);});
          load = norm(acc) / gravity;
          cumulative += log.scenario.load_penalty_s * Math.pow(Math.max(0, load - log.scenario.load_limit_g), 2) * dt;
        }
      }
      const atEnd = i === samples.length - 1;
      const penalty = atEnd ? log.summary.load_penalty_s : cumulative;
      const failurePenalty = atEnd ? log.summary.failure_penalty_s : 0;
      results.push(Object.freeze({index: i, time: sample.time_s, flightTime: sample.time_s,
        speed: norm(sample.velocity_m_s), thrust: sample.action ? sample.action.thrust_N : null,
        load: load, penalty: penalty, estimatedPenalty: cumulative,
        failurePenalty: failurePenalty, score: atEnd ? log.summary.score_s : sample.time_s + cumulative,
        loadEstimated: load !== null, penaltyEstimated: !atEnd, scoreEstimated: !atEnd,
        exactFinalSummary: atEnd, synthetic: log.synthetic === true, completed: atEnd && log.summary.completed}));
    }
    cache.set(log, results); return results;
  }
  function metricsAt(log, index) {
    const metrics = prepareMetrics(log);
    return metrics[clamp(Number.isFinite(index) ? Math.floor(index) : 0, 0, metrics.length - 1)];
  }
  function sampleIndexAt(log, time) {
    if (!Number.isFinite(time) || time <= 0) return 0;
    let lo = 0, hi = log.samples.length - 1;
    while (lo < hi) {const mid = Math.ceil((lo + hi) / 2); if (log.samples[mid].time_s <= time) lo = mid; else hi = mid - 1;}
    return lo;
  }
  function rankRuns(runs) {
    return runs.slice().sort(function (a, b) {
      const x = a.log.summary, y = b.log.summary;
      if (x.completed !== y.completed) return x.completed ? -1 : 1;
      if (x.completed) return x.score_s - y.score_s || x.flight_time_s - y.flight_time_s;
      return y.gates_passed - x.gates_passed || x.flight_time_s - y.flight_time_s;
    });
  }

  /* A small port of the public geometric rules is used only to construct coherent
     illustrative events. Imported real logs are never rejudged by this code. */
  function local(point, gate) {
    const c = Math.cos(gate.yaw_rad), s = Math.sin(gate.yaw_rad), d = point.map(function (v, j) {return v - gate.center[j];});
    return [c * d[0] + s * d[1], -s * d[0] + c * d[1], d[2]];
  }
  function slab(a, b, lower, upper, radius) {
    let enter = 0, leave = 1;
    for (let j = 0; j < 3; j += 1) {
      const d = b[j] - a[j], lo = lower[j] - radius, hi = upper[j] + radius;
      if (Math.abs(d) < 1e-14) {if (a[j] < lo || a[j] > hi) return null;}
      else {let u = (lo - a[j]) / d, v = (hi - a[j]) / d; if (u > v) {const temp = u; u = v; v = temp;} enter = Math.max(enter, u); leave = Math.min(leave, v); if (enter > leave) return null;}
    }
    return enter;
  }
  function inspect(start, end, next) {
    const candidates = [], radius = 0.28;
    TRACK.gates.forEach(function (gate, index) {
      const a = local(start, gate), b = local(end, gate), w = gate.width_m / 2, h = gate.height_m / 2, f = gate.frame_m;
      const frames = [[[-f / 2, -w - f, -h - f], [f / 2, -w, h + f]], [[-f / 2, w, -h - f], [f / 2, w + f, h + f]],
        [[-f / 2, -w, -h - f], [f / 2, w, -h]], [[-f / 2, -w, h], [f / 2, w, h + f]]];
      frames.forEach(function (frame) {const alpha = slab(a, b, frame[0], frame[1], radius); if (alpha !== null) candidates.push({kind: 'collision', fraction: alpha, gate: index});});
      [[true, 'forward'], [false, 'reverse_gate']].forEach(function (direction) {
        const crosses = direction[0] ? a[0] < 0 && b[0] >= 0 : a[0] > 0 && b[0] <= 0;
        if (!crosses) return;
        const alpha = -a[0] / (b[0] - a[0]), hit = mix(a, b, alpha);
        if (Math.abs(hit[1]) < w - radius && Math.abs(hit[2]) < h - radius) candidates.push({kind: direction[1], fraction: alpha, gate: index});
      });
    });
    const lower = TRACK.bounds.lower.map(function (v) {return v + radius;}), upper = TRACK.bounds.upper.map(function (v) {return v - radius;});
    if (start.some(function (v, j) {return v <= lower[j] || v >= upper[j];})) candidates.push({kind: 'out_of_bounds', fraction: 0, gate: null});
    else if (end.some(function (v, j) {return v <= lower[j] || v >= upper[j];})) {
      const fractions = [];
      for (let j = 0; j < 3; j += 1) if (end[j] !== start[j]) [lower[j], upper[j]].forEach(function (bound) {const f = (bound - start[j]) / (end[j] - start[j]); if (f >= 0 && f <= 1) fractions.push(f);});
      candidates.push({kind: 'out_of_bounds', fraction: Math.min.apply(Math, fractions), gate: null});
    }
    candidates.sort(function (a, b) {return a.fraction - b.fraction || Number(a.kind !== 'collision') - Number(b.kind !== 'collision');});
    const events = []; let terminal = null;
    for (const event of candidates) {
      if (event.kind === 'forward') {
        if (event.gate === next) {events.push({kind: 'gate_passed', fraction: event.fraction, gate: event.gate}); next += 1; if (next === TRACK.gates.length) {terminal = {kind: 'finished', fraction: event.fraction, gate: event.gate}; events.push(terminal); break;}}
        else if (event.gate > next) {terminal = {kind: 'skipped_gate', fraction: event.fraction, gate: event.gate}; events.push(terminal); break;}
      } else if (event.kind === 'reverse_gate') {if (event.gate >= next) {terminal = event; events.push(event); break;}}
      else {terminal = event; events.push(event); break;}
    }
    return {next: next, events: events, terminal: terminal};
  }
  function baseConfig() {return {track: clone(TRACK), control_dt_s: 0.02, physics_dt_s: 0.002,
    nominal_mass_kg: 0.5, gravity_m_s2: G, thrust_min_N: 0, thrust_max_N: 15,
    body_rate_limit_rad_s: 3, body_rate_gain_s_inv: 25, vehicle_radius_m: 0.28};}
  function makePath(duration, wave, crashed) {
    const gates = crashed ? TRACK.gates.slice(0, 7) : TRACK.gates;
    const points = [TRACK.start.slice()].concat(gates.map(function (gate) {return gate.center.slice();}));
    if (crashed) {
      const gate = gates[gates.length - 1], c = Math.cos(gate.yaw_rad), s = Math.sin(gate.yaw_rad);
      /* Aim at the right frame, a little beyond first sphere contact. */
      points[points.length - 1] = [gate.center[0] - 0.30 * c - 1.05 * s, gate.center[1] - 0.30 * s + 1.05 * c, gate.center[2]];
    }
    const distances = points.slice(1).map(function (p, i) {return norm(p.map(function (v, j) {return v - points[i][j];}));});
    const totalDistance = distances.reduce(function (a, b) {return a + b;}, 0);
    const times = [0]; distances.forEach(function (d) {times.push(times[times.length - 1] + duration * d / totalDistance);});
    times[times.length - 1] = duration;
    const tangents = [[0, 0, 0]].concat(gates.map(function (gate) {return [Math.cos(gate.yaw_rad) * totalDistance / duration, Math.sin(gate.yaw_rad) * totalDistance / duration, 0];}));
    const timestamps = [];
    for (let time = 0; time < duration - 1e-8; time += 0.02) timestamps.push(Math.round(time * 1e9) / 1e9);
    timestamps.push(duration);
    let segment = 0;
    const positions = timestamps.map(function (time) {
      while (segment < points.length - 2 && time > times[segment + 1]) segment += 1;
      const dt = times[segment + 1] - times[segment], u = clamp((time - times[segment]) / dt, 0, 1);
      const h00 = 2 * u * u * u - 3 * u * u + 1, h10 = u * u * u - 2 * u * u + u;
      const h01 = -2 * u * u * u + 3 * u * u, h11 = u * u * u - u * u;
      const p = points[segment].map(function (v, j) {return h00 * v + h10 * dt * tangents[segment][j] + h01 * points[segment + 1][j] + h11 * dt * tangents[segment + 1][j];});
      p[2] += wave * Math.pow(Math.sin(Math.PI * u), 2) * Math.sin(8 * Math.PI * u);
      return p;
    });
    positions[0] = points[0].slice(); positions[positions.length - 1] = points[points.length - 1].slice();
    return timestamps.map(function (time, i) {
      const before = Math.max(0, i - 1), after = Math.min(positions.length - 1, i + 1), dt = timestamps[after] - timestamps[before];
      const velocity = i === 0 ? [0, 0, 0] : positions[after].map(function (v, j) {return (v - positions[before][j]) / dt;});
      const yaw = Math.atan2(velocity[1], velocity[0]);
      return {time_s: time, position_m: positions[i], velocity_m_s: velocity,
        quaternion_xyzw: [0, 0, Math.sin(yaw / 2), Math.cos(yaw / 2)], body_rates_rad_s: [0, 0, 0], next_gate: 0, wind_m_s: [0, 0, 0], action: null};
    });
  }
  function loadCost(samples) {
    let penalty = 0, peak = 0;
    samples.forEach(function (sample, i) {
      if (i === 0) return;
      const previous = samples[i - 1], dt = sample.time_s - previous.time_s;
      if (dt <= 1e-12) return;
      const load = norm(sample.velocity_m_s.map(function (v, j) {return (v - previous.velocity_m_s[j]) / dt + (j === 2 ? G : 0);})) / G;
      peak = Math.max(peak, load); penalty += 2 * Math.pow(Math.max(0, load - 3), 2) * dt;
    });
    return {penalty: penalty, peak: peak};
  }
  function makeDemo(id, name, duration, wave, crashed, description) {
    const source = makePath(duration, wave, crashed), samples = [source[0]], events = [];
    let next = 0, status = 'timeout';
    for (let i = 1; i < source.length; i += 1) {
      const previous = samples[samples.length - 1], candidate = source[i], dt = candidate.time_s - previous.time_s;
      const inspected = inspect(previous.position_m, candidate.position_m, next); next = inspected.next;
      inspected.events.forEach(function (event) {events.push({kind: event.kind, gate: event.gate, time_s: previous.time_s + event.fraction * dt});});
      if (inspected.terminal) {
        const fraction = inspected.terminal.fraction;
        candidate.time_s = previous.time_s + fraction * dt;
        candidate.position_m = mix(previous.position_m, candidate.position_m, fraction);
        candidate.velocity_m_s = mix(previous.velocity_m_s, candidate.velocity_m_s, fraction);
        status = inspected.terminal.kind;
      }
      candidate.next_gate = next;
      const actualDt = candidate.time_s - previous.time_s;
      const load = actualDt > 1e-12 ? norm(candidate.velocity_m_s.map(function (v, j) {return (v - previous.velocity_m_s[j]) / actualDt + (j === 2 ? G : 0);})) / G : 1;
      candidate.action = {thrust_N: clamp(0.5 * G * load, 0, 15), body_rates_rad_s: [0, 0, 0]};
      samples.push(candidate);
      if (inspected.terminal) break;
    }
    if ((!crashed && status !== 'finished') || (crashed && status !== 'collision')) throw new Error('합성 경로 구성 오류: ' + id + ' / ' + status);
    const flight = samples[samples.length - 1].time_s, cost = loadCost(samples), failure = status === 'finished' ? 0 : 30;
    const raw = {schema_version: 1, synthetic: true, data_kind: 'illustrative_kinematic_data',
      config: baseConfig(), scenario: {seed: 0, wind_max_m_s: 0, mass_error_fraction: 0,
        max_time_s: 45, load_limit_g: 3, load_penalty_s: 2, failure_penalty_s: 30},
      realization: {mass_scale: 1, wind_phase_rad: [0, 0, 0]},
      provenance: {source: 'presentation-synthetic', note: DATA_NOTE, commands: 'illustrative only; not dynamics-validated'},
      samples: samples, events: events, summary: {status: status, completed: status === 'finished', flight_time_s: flight,
        gates_passed: next, gates_total: TRACK.gates.length, score_s: flight + cost.penalty + failure,
        load_penalty_s: cost.penalty, failure_penalty_s: failure, peak_load_g: cost.peak,
        controller_reset_s: 0, controller_mean_act_s: 0, controller_max_act_s: 0, wall_time_s: 0, error: null}};
    const log = normalizeLog(raw);
    return {id: id, name: name, synthetic: true, description: description, log: log};
  }

  /* Tune an illustrative oscillation to make the time/load tradeoff visible. */
  let waveLo = 0, waveHi = 0.48;
  for (let iteration = 0; iteration < 16; iteration += 1) {
    const wave = (waveLo + waveHi) / 2;
    if (loadCost(makePath(28.3, wave, false)).penalty < 5.2) waveLo = wave; else waveHi = wave;
  }
  const demoRuns = [
    makeDemo('a01', 'VECTOR', 31.2, 0.025, false, '기존 기록 · 안정적인 완주'),
    makeDemo('a02', 'RAPTOR', 28.3, (waveLo + waveHi) / 2, false, '기존 기록 · 짧은 비행시간, 큰 하중 벌점'),
    makeDemo('a03', 'COMET', 19.4, 0.035, true, '기존 기록 · 7번 게이트 프레임 충돌 / 미완주'),
    makeDemo('a04', 'AURORA', 29.2, 0.045, false, '현재 주자 · 신규 1위 기록 시연'),
    makeDemo('a05', 'ORBIT', 30.0, 0.035, false, '다음 주자 · 2위권 완주 시연')
  ];
  global.RaceData = Object.freeze({demoRuns: demoRuns, defaultRunId: 'a04', defaultRunIndex: 3,
    initialRecordIds: ['a01', 'a02', 'a03'], nextRunId: 'a05', track: clone(TRACK),
    normalizeLog: normalizeLog, metricsAt: metricsAt, sampleIndexAt: sampleIndexAt,
    rankRuns: rankRuns, statusLabels: STATUS_LABELS, escapeHTML: escapeHTML,
    dataNote: DATA_NOTE, metricNote: METRIC_NOTE,
    units: Object.freeze({position: 'm', speed: 'm/s', thrust: 'N', load: 'g', penalty: 's', score: 's'}),
    scoringNote: '시제품 기준: 완주 우선, 완주끼리는 score_s 오름차순. 미완주 score_s는 진단용입니다. 노션의 계주 순위 규칙과는 별개입니다.'});
})(typeof window !== 'undefined' ? window : globalThis);
