/* Standalone course renderer. No network requests or external dependencies. */
(function () {
  'use strict';

  var COLORS = {
    background: '#08121e', grid: '#213447', muted: '#748ca2',
    text: '#dbe8f3', cyan: '#61dbef', mint: '#76e4b7',
    pending: '#58718a', orange: '#ffbc79', danger: '#ff8c8c'
  };
  var courseCache = new WeakMap();
  var isFiniteNumber = Number.isFinite;

  function number(value, fallback) {
    var numeric = typeof value === 'number' ? value : Number(value);
    return isFiniteNumber(numeric) ? numeric : fallback;
  }

  function vector(value, fallback) {
    return Array.isArray(value) && value.length >= 3
      ? value.slice(0, 3).map(function (v, i) { return number(v, fallback[i]); })
      : fallback.slice();
  }

  function add(a, b, multiplier) {
    return a.map(function (v, i) { return v + b[i] * multiplier; });
  }

  function path(ctx, points, closed) {
    if (!points.length) return;
    ctx.beginPath();
    ctx.moveTo(points[0][0], points[0][1]);
    points.slice(1).forEach(function (p) { ctx.lineTo(p[0], p[1]); });
    if (closed) ctx.closePath();
  }

  function line(ctx, points, color, width, dash) {
    if (points.length < 2) return;
    path(ctx, points, false);
    ctx.strokeStyle = color;
    ctx.lineWidth = width;
    ctx.setLineDash(dash || []);
    ctx.stroke();
    ctx.setLineDash([]);
  }

  function circle(ctx, x, y, radius, fill, stroke, width) {
    ctx.beginPath();
    ctx.arc(x, y, radius, 0, Math.PI * 2);
    if (fill) { ctx.fillStyle = fill; ctx.fill(); }
    if (stroke) { ctx.strokeStyle = stroke; ctx.lineWidth = width || 1; ctx.stroke(); }
  }

  function pill(ctx, text, x, y, color, small) {
    ctx.font = (small ? '600 9px' : '600 10px') + ' system-ui, -apple-system, sans-serif';
    var width = ctx.measureText(text).width + 15;
    var height = small ? 19 : 22;
    var left = x - width / 2;
    var top = y - height / 2;
    var radius = 6;
    ctx.beginPath();
    ctx.moveTo(left + radius, top);
    ctx.arcTo(left + width, top, left + width, top + height, radius);
    ctx.arcTo(left + width, top + height, left, top + height, radius);
    ctx.arcTo(left, top + height, left, top, radius);
    ctx.arcTo(left, top, left + width, top, radius);
    ctx.closePath();
    ctx.fillStyle = 'rgba(8,19,31,.92)';
    ctx.fill();
    ctx.strokeStyle = color;
    ctx.globalAlpha = .45;
    ctx.lineWidth = 1;
    ctx.stroke();
    ctx.globalAlpha = 1;
    ctx.fillStyle = color;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(text, x, y + .5);
  }

  function arrow(ctx, from, to, color, width, head) {
    line(ctx, [from, to], color, width);
    var angle = Math.atan2(to[1] - from[1], to[0] - from[0]);
    var size = head || 5;
    path(ctx, [
      [to[0] - size * Math.cos(angle - .55), to[1] - size * Math.sin(angle - .55)],
      to,
      [to[0] - size * Math.cos(angle + .55), to[1] - size * Math.sin(angle + .55)]
    ], false);
    ctx.strokeStyle = color;
    ctx.lineWidth = width;
    ctx.stroke();
  }

  function niceStep(span) {
    var target = Math.max(span / 16, .25);
    var power = Math.pow(10, Math.floor(Math.log10(target)));
    var fraction = target / power;
    return (fraction <= 1 ? 1 : fraction <= 2 ? 2 : fraction <= 5 ? 5 : 10) * power;
  }

  window.drawCourse = function (canvas, track, samples, sampleIndex, events, view) {
    if (!canvas || typeof canvas.getContext !== 'function') return;
    var ctx = canvas.getContext('2d');
    if (!ctx) return;
    track = track || {};
    samples = Array.isArray(samples) ? samples : [];
    events = Array.isArray(events) ? events : [];
    var topView = view === 'top';
    var bounds = canvas.getBoundingClientRect();
    var width = Math.max(1, Math.round(bounds.width || canvas.clientWidth || 800));
    var height = Math.max(1, Math.round(bounds.height || canvas.clientHeight || 480));
    var dpr = Math.min(number(window.devicePixelRatio, 1), 3);
    if (canvas.width !== Math.round(width * dpr)) canvas.width = Math.round(width * dpr);
    if (canvas.height !== Math.round(height * dpr)) canvas.height = Math.round(height * dpr);
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, width, height);
    ctx.lineJoin = 'round';
    ctx.lineCap = 'round';

    var background = ctx.createRadialGradient(width * .48, height * .35, 0,
      width * .48, height * .4, Math.max(width, height) * .8);
    background.addColorStop(0, '#122538');
    background.addColorStop(.65, '#0b1725');
    background.addColorStop(1, COLORS.background);
    ctx.fillStyle = background;
    ctx.fillRect(0, 0, width, height);

    var start = vector(track.start, [0, 0, 1.5]);
    var gates = (Array.isArray(track.gates) ? track.gates : []).map(function (g) {
      var yaw = number(g.yaw_rad, 0);
      return {
        center: vector(g.center, [0, 0, 1.5]), yaw: yaw,
        width: Math.max(.1, number(g.width_m, 2)),
        height: Math.max(.1, number(g.height_m, 2)),
        normal: [Math.cos(yaw), Math.sin(yaw), 0],
        tangent: [-Math.sin(yaw), Math.cos(yaw), 0]
      };
    });
    var index = Math.max(0, Math.min(samples.length - 1,
      Math.floor(number(sampleIndex, samples.length - 1))));
    var current = samples[index] || { position_m: start, next_gate: 0, time_s: 0 };
    var currentPosition = vector(current.position_m, start);
    var currentTime = number(current.time_s, 0);
    var nextGate = Math.max(0, Math.min(gates.length, number(current.next_gate, 0)));
    var cached = courseCache.get(samples);
    if (!cached || cached.track !== track || cached.length !== samples.length) {
      cached = { track: track, length: samples.length, footprint: null, projections: Object.create(null) };
      courseCache.set(samples, cached);
    }
    var floorZ = track.bounds && Array.isArray(track.bounds.lower)
      ? number(track.bounds.lower[2], 0) : 0;
    var worldPoints = [start];

    function gateCorners(g) {
      return [[-1, -1], [1, -1], [1, 1], [-1, 1]].map(function (sign) {
        var p = add(g.center, g.tangent, sign[0] * g.width / 2);
        p[2] += sign[1] * g.height / 2;
        return p;
      });
    }

    gates.forEach(function (g) { worldPoints.push.apply(worldPoints, gateCorners(g)); });
    // Align the course's principal horizontal axis with the wide stage. A slight
    // oblique angle keeps the gate openings visible on mostly straight courses.
    var cameraPoints = [start].concat(gates.map(function (g) { return g.center; }));
    var meanX = cameraPoints.reduce(function (sum, p) { return sum + p[0]; }, 0) / cameraPoints.length;
    var meanY = cameraPoints.reduce(function (sum, p) { return sum + p[1]; }, 0) / cameraPoints.length;
    var varianceX = 0, varianceY = 0, covariance = 0;
    cameraPoints.forEach(function (p) {
      var dx = p[0] - meanX, dy = p[1] - meanY;
      varianceX += dx * dx; varianceY += dy * dy; covariance += dx * dy;
    });
    var principalAngle = .5 * Math.atan2(2 * covariance, varianceX - varianceY);
    var lastCenter = gates.length ? gates[gates.length - 1].center : add(start, [1, 0, 0], 1);
    if ((lastCenter[0] - start[0]) * Math.cos(principalAngle) +
        (lastCenter[1] - start[1]) * Math.sin(principalAngle) < 0) principalAngle += Math.PI;
    var cameraAngle = principalAngle + (topView ? 0 : -.48);
    var cameraCos = Math.cos(cameraAngle), cameraSin = Math.sin(cameraAngle);
    // A shallow elevation on a short, wide stage avoids spending most of its
    // height on empty ground. Taller stages retain a stronger 3D impression.
    var stageRatio = height / width;
    var elevationSin = Math.min(.43, Math.max(.10, .40 * stageRatio + .40 * Math.max(0, stageRatio - .4)));
    var elevationCos = Math.sqrt(1 - elevationSin * elevationSin);
    var projectionKey = (topView ? 'top' : elevationSin.toFixed(7)) + ':' + cameraAngle.toFixed(7);

    // Cache the full recording's footprint once, without copying a large log or
    // expanding it into function arguments. Playback then only visits a bounded
    // number of path samples, including for recordings with 200,000 entries.
    if (!cached.footprint) {
      var footprint = { minX: start[0], maxX: start[0], minY: start[1], maxY: start[1] };
      var firstProjection = { lowX: Infinity, highX: -Infinity, lowY: Infinity, highY: -Infinity };
      for (var sampleNumber = 0; sampleNumber < samples.length; sampleNumber++) {
        var footprintPosition = samples[sampleNumber].position_m || start;
        var footprintX = number(footprintPosition[0], start[0]);
        var footprintY = number(footprintPosition[1], start[1]);
        var footprintZ = number(footprintPosition[2], start[2]);
        if (footprintX < footprint.minX) footprint.minX = footprintX;
        if (footprintX > footprint.maxX) footprint.maxX = footprintX;
        if (footprintY < footprint.minY) footprint.minY = footprintY;
        if (footprintY > footprint.maxY) footprint.maxY = footprintY;
        var firstHorizontal = cameraCos * footprintX + cameraSin * footprintY;
        var firstDepth = -cameraSin * footprintX + cameraCos * footprintY;
        var firstVertical = topView ? -firstDepth : firstDepth * elevationSin - footprintZ * elevationCos;
        if (firstHorizontal < firstProjection.lowX) firstProjection.lowX = firstHorizontal;
        if (firstHorizontal > firstProjection.highX) firstProjection.highX = firstHorizontal;
        if (firstVertical < firstProjection.lowY) firstProjection.lowY = firstVertical;
        if (firstVertical > firstProjection.highY) firstProjection.highY = firstVertical;
      }
      cached.footprint = footprint;
      cached.projections[projectionKey] = firstProjection;
    }
    var obstacles = (Array.isArray(track.obstacles) ? track.obstacles : [])
      .filter(function (o) { return Array.isArray(o.lower) && Array.isArray(o.upper); });
    obstacles.forEach(function (o) {
      var lo = vector(o.lower, [0, 0, 0]), hi = vector(o.upper, [0, 0, 0]);
      [lo[0], hi[0]].forEach(function (x) {
        [lo[1], hi[1]].forEach(function (y) {
          [lo[2], hi[2]].forEach(function (z) { worldPoints.push([x, y, z]); });
        });
      });
    });
    var minX = cached.footprint.minX, maxX = cached.footprint.maxX;
    var minY = cached.footprint.minY, maxY = cached.footprint.maxY;
    worldPoints.forEach(function (p) {
      minX = Math.min(minX, p[0]); maxX = Math.max(maxX, p[0]);
      minY = Math.min(minY, p[1]); maxY = Math.max(maxY, p[1]);
    });
    minX -= 1.1; maxX += 1.1; minY -= 1.1; maxY += 1.1;
    var floorCorners = [[minX, minY, floorZ], [maxX, minY, floorZ],
      [maxX, maxY, floorZ], [minX, maxY, floorZ]];

    function depth(p) {
      return -cameraSin * p[0] + cameraCos * p[1];
    }

    function raw(p) {
      var horizontal = cameraCos * p[0] + cameraSin * p[1];
      return [horizontal, topView ? -depth(p) : depth(p) * elevationSin - p[2] * elevationCos];
    }

    var projectionBounds = cached.projections[projectionKey];
    if (!projectionBounds) {
      projectionBounds = { lowX: Infinity, highX: -Infinity, lowY: Infinity, highY: -Infinity };
      for (var fitIndex = 0; fitIndex < samples.length; fitIndex++) {
        var fitPosition = samples[fitIndex].position_m || start;
        var fitX = number(fitPosition[0], start[0]), fitY = number(fitPosition[1], start[1]);
        var fitZ = number(fitPosition[2], start[2]);
        var fitHorizontal = cameraCos * fitX + cameraSin * fitY;
        var fitDepth = -cameraSin * fitX + cameraCos * fitY;
        var fitVertical = topView ? -fitDepth : fitDepth * elevationSin - fitZ * elevationCos;
        if (fitHorizontal < projectionBounds.lowX) projectionBounds.lowX = fitHorizontal;
        if (fitHorizontal > projectionBounds.highX) projectionBounds.highX = fitHorizontal;
        if (fitVertical < projectionBounds.lowY) projectionBounds.lowY = fitVertical;
        if (fitVertical > projectionBounds.highY) projectionBounds.highY = fitVertical;
      }
      cached.projections[projectionKey] = projectionBounds;
    }
    var lowX = projectionBounds.lowX, highX = projectionBounds.highX;
    var lowY = projectionBounds.lowY, highY = projectionBounds.highY;
    worldPoints.forEach(function (p) {
      var r = raw(p);
      lowX = Math.min(lowX, r[0]); highX = Math.max(highX, r[0]);
      lowY = Math.min(lowY, r[1]); highY = Math.max(highY, r[1]);
    });
    // Fit only real gates, obstacles and flight samples. The decorative floor
    // deliberately extends beyond this frame and never zooms the course out.
    var paddingX = width < 500 ? 42 : 51;
    var paddingTop = 35, paddingBottom = 36;
    var availableWidth = Math.max(1, width - paddingX * 2);
    var availableHeight = Math.max(1, height - paddingTop - paddingBottom);
    var scale = Math.min(availableWidth / Math.max(highX - lowX, 1),
      availableHeight / Math.max(highY - lowY, 1));
    var centerX = (lowX + highX) / 2, centerY = (lowY + highY) / 2;
    var offsetY = paddingTop + availableHeight / 2;

    function project(p) {
      var r = raw(p);
      return [width / 2 + (r[0] - centerX) * scale, offsetY + (r[1] - centerY) * scale];
    }

    path(ctx, floorCorners.map(project), true);
    ctx.fillStyle = 'rgba(21,42,59,.30)';
    ctx.fill();
    ctx.strokeStyle = 'rgba(89,126,151,.20)';
    ctx.lineWidth = 1;
    ctx.stroke();
    var step = niceStep(Math.max(maxX - minX, maxY - minY));
    for (var gx = Math.ceil(minX / step) * step; gx <= maxX; gx += step) {
      line(ctx, [project([gx, minY, floorZ]), project([gx, maxY, floorZ])], 'rgba(96,135,162,.12)', .8);
    }
    for (var gy = Math.ceil(minY / step) * step; gy <= maxY; gy += step) {
      line(ctx, [project([minX, gy, floorZ]), project([maxX, gy, floorZ])], 'rgba(96,135,162,.12)', .8);
    }

    // A subdued center-to-center route makes the gate order readable before takeoff.
    var courseLine = [start].concat(gates.map(function (g) { return g.center; }));
    line(ctx, courseLine.map(project), 'rgba(139,164,186,.22)', 1.1, [3, 7]);

    obstacles.forEach(function (o) {
      var a = vector(o.lower, [0, 0, 0]), b = vector(o.upper, [0, 0, 0]);
      var bottom = [[a[0], a[1], a[2]], [b[0], a[1], a[2]],
        [b[0], b[1], a[2]], [a[0], b[1], a[2]]];
      var upper = bottom.map(function (p) { return [p[0], p[1], b[2]]; });
      if (!topView) {
        [0, 1, 2, 3].forEach(function (i) {
          var j = (i + 1) % 4;
          path(ctx, [bottom[i], bottom[j], upper[j], upper[i]].map(project), true);
          ctx.fillStyle = 'rgba(212,129,104,.13)'; ctx.fill();
          ctx.strokeStyle = 'rgba(231,159,128,.29)'; ctx.lineWidth = 1; ctx.stroke();
        });
      }
      path(ctx, upper.map(project), true);
      ctx.fillStyle = 'rgba(212,129,104,.21)'; ctx.fill();
      ctx.strokeStyle = 'rgba(231,159,128,.52)'; ctx.lineWidth = 1; ctx.stroke();
    });

    var flownPositions = [];
    var stride = Math.max(1, Math.ceil((index + 1) / 1800));
    for (var flownIndex = 0; flownIndex <= index && flownIndex < samples.length; flownIndex += stride) {
      flownPositions.push(vector(samples[flownIndex].position_m, start));
    }
    if (samples.length && index % stride !== 0) flownPositions.push(currentPosition);
    if (!topView) {
      line(ctx, flownPositions.map(function (p) { return project([p[0], p[1], floorZ]); }),
        'rgba(91,199,214,.13)', 1.5);
    }
    ctx.save();
    ctx.shadowColor = 'rgba(79,214,232,.4)';
    ctx.shadowBlur = 9;
    line(ctx, flownPositions.map(project), COLORS.cyan, 2.6);
    ctx.restore();

    gates.map(function (g, i) { return { gate: g, index: i }; })
      .sort(function (a, b) { return depth(a.gate.center) - depth(b.gate.center); })
      .forEach(function (entry) {
        var g = entry.gate, i = entry.index;
        var color = i < nextGate ? COLORS.mint : i === nextGate ? COLORS.cyan : COLORS.pending;
        var active = i === nextGate;
        var corners = gateCorners(g);
        if (topView) {
          var left = add(g.center, g.tangent, -g.width / 2);
          var right = add(g.center, g.tangent, g.width / 2);
          line(ctx, [project(left), project(right)], 'rgba(8,19,31,.9)', 10);
          line(ctx, [project(left), project(right)], color, active ? 5 : 3.5);
          circle(ctx, project(left)[0], project(left)[1], 3.2, color);
          circle(ctx, project(right)[0], project(right)[1], 3.2, color);
        } else {
          path(ctx, corners.map(project), true);
          ctx.fillStyle = active ? 'rgba(97,219,239,.075)' : i < nextGate ? 'rgba(118,228,183,.035)' : 'rgba(91,126,151,.025)';
          ctx.fill();
          if (active) { ctx.shadowColor = 'rgba(97,219,239,.35)'; ctx.shadowBlur = 10; }
          ctx.strokeStyle = color;
          ctx.lineWidth = Math.max(active ? 2.8 : 2, Math.min(scale * .085, 5));
          ctx.stroke();
          ctx.shadowBlur = 0;
          var base = [g.center[0], g.center[1], floorZ];
          line(ctx, [project(base), project([g.center[0], g.center[1], g.center[2] - g.height / 2])],
            'rgba(115,148,170,.25)', 1, [2, 4]);
        }
        var arrowFrom = project(add(g.center, g.normal, -.22));
        var arrowTo = project(add(g.center, g.normal, .65));
        arrow(ctx, arrowFrom, arrowTo, active ? 'rgba(97,219,239,.78)' : 'rgba(141,171,190,.32)', 1.2, 4);
        var labelPosition = project([g.center[0], g.center[1], g.center[2] + (topView ? 0 : g.height / 2)]);
        var label = String(i + 1) + (i === gates.length - 1 ? ' · FINISH' : '');
        pill(ctx, label, labelPosition[0] + (topView ? 12 : 0), labelPosition[1] - (topView ? 21 : 16), color, false);
      });

    var startPoint = project(start);
    circle(ctx, startPoint[0], startPoint[1], 7, 'rgba(118,228,183,.09)', 'rgba(118,228,183,.65)', 1.3);
    circle(ctx, startPoint[0], startPoint[1], 2.5, COLORS.mint);
    pill(ctx, 'START', startPoint[0], startPoint[1] + 23, COLORS.mint, true);

    var dronePoint = project(currentPosition);
    if (!topView) {
      var shadow = project([currentPosition[0], currentPosition[1], floorZ]);
      circle(ctx, shadow[0], shadow[1], 6, 'rgba(97,219,239,.12)');
      line(ctx, [shadow, dronePoint], 'rgba(97,219,239,.15)', 1, [2, 4]);
    }
    var q = Array.isArray(current.quaternion_xyzw) ? current.quaternion_xyzw : [0, 0, 0, 1];
    var qx = number(q[0], 0), qy = number(q[1], 0), qz = number(q[2], 0), qw = number(q[3], 1);
    var bodyX = [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy + qz * qw), 2 * (qx * qz - qy * qw)];
    var bodyY = [2 * (qx * qy - qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz + qx * qw)];
    var arm = Math.max(.19, 7.5 / Math.max(scale, .01));
    ctx.save();
    ctx.shadowColor = 'rgba(97,219,239,.65)'; ctx.shadowBlur = 14;
    [[1, 1], [-1, 1], [-1, -1], [1, -1]].forEach(function (sign) {
      var rotor = project(add(add(currentPosition, bodyX, sign[0] * arm), bodyY, sign[1] * arm));
      line(ctx, [dronePoint, rotor], '#c1f2f7', 2);
      circle(ctx, rotor[0], rotor[1], 3.6, '#133544', '#a3eff4', 1.2);
    });
    circle(ctx, dronePoint[0], dronePoint[1], 4.7, '#e5fdff', COLORS.cyan, 1.7);
    ctx.restore();
    var nose = project(add(currentPosition, bodyX, arm * 1.35));
    line(ctx, [dronePoint, nose], COLORS.orange, 2.1);

    var terminal = events.filter(function (event) {
      return number(event.time_s, Infinity) <= currentTime + 1e-7 &&
        ['gate_passed', 'finished'].indexOf(event.kind) === -1;
    }).pop();
    if (terminal) {
      circle(ctx, dronePoint[0], dronePoint[1], 17, null, 'rgba(255,140,140,.6)', 1.5);
    } else if (nextGate === gates.length && gates.length) {
      circle(ctx, dronePoint[0], dronePoint[1], 17, null, 'rgba(118,228,183,.65)', 1.5);
    }

    ctx.font = '600 9px system-ui, -apple-system, sans-serif';
    ctx.textAlign = 'left'; ctx.textBaseline = 'middle';
    ctx.fillStyle = '#7791a8';
    ctx.fillText(topView ? 'TOP VIEW' : 'COURSE OVERVIEW', 19, 22);
    ctx.font = '10px system-ui, -apple-system, sans-serif';
    ctx.fillStyle = '#638198';
    ctx.fillText('GRID ' + step + ' m', 19, 38);

    var legendY = height - 18;
    circle(ctx, 23, legendY, 3, COLORS.mint);
    ctx.fillStyle = '#93aabb'; ctx.fillText('통과', 33, legendY + .5);
    circle(ctx, 79, legendY, 3, COLORS.cyan);
    ctx.fillText('다음 게이트', 89, legendY + .5);
    if (width > 390) {
      line(ctx, [[166, legendY], [179, legendY]], COLORS.cyan, 2);
      ctx.fillText('비행 궤적', 186, legendY + .5);
    }

    var origin = [width - 43, height - 35];
    var axes = [{ label: 'x', axis: [1, 0, 0], color: '#df9b91' },
      { label: 'y', axis: [0, 1, 0], color: '#92bda0' }];
    if (!topView) axes.push({ label: 'z', axis: [0, 0, 1], color: '#8ebcdb' });
    axes.forEach(function (axis) {
      var direction = raw(axis.axis), length = Math.hypot(direction[0], direction[1]);
      var tip = [origin[0] + direction[0] / length * 19, origin[1] + direction[1] / length * 19];
      arrow(ctx, origin, tip, axis.color, 1.15, 3.5);
      ctx.fillStyle = axis.color;
      ctx.textAlign = 'center';
      ctx.fillText(axis.label, tip[0] + direction[0] / length * 7, tip[1] + direction[1] / length * 7);
    });
    ctx.textAlign = 'left';
  };
})();
