// ==UserScript==
// @name         GeoFS Autopilot Pro
// @namespace    https://github.com/bpleeman
// @version      1.6.5
// @description  Native GeoFS autopilot panel that is always visible and editable, LOC/GS buttons in the same UI style (LOC 30 NM, GS 15 NM within 8° of the runway axis), NAV button = flight-plan waypoints only, full joystick/throttle/pedal override while the AP is engaged. Values only change when you change them (or LOC/GS: heading and V/S).
// @match        https://*.geo-fs.com/geofs.php*
// @grant        none
// @run-at       document-idle
// ==/UserScript==

(function () {
    'use strict';

    // ------------------------------------------------------------ settings
    const CFG = {
        LOC_RANGE_NM: 30,          // localizer usable range
        LOC_CAPTURE_MAX_DEV: 45,   // LOC captures anywhere within this angle from the runway axis (inside the range)
        LOC_CAPTURE_MAX_HDG: 120,  // ... as long as you are not flying away from the runway
        LOC_K: 50,                 // intercept angle (deg) per NM of cross-track error
        LOC_MIN_INTERCEPT: 8,      // below this intercept angle LOC just tracks instead of holding the heading
        LOC_TURN_BANK_FRAC: 0.8,   // the turn-in is planned at this fraction of the max bank angle (later, firmer turn)
        LOC_MIN_BANK: 10,          // during the turn the bank adapts between this and the max bank (wind, early/late start)
        LOC_LEAD_LAG_S: 4,         // extra lead for the autopilot's roll-in time (starts the turn a bit earlier)
        LOC_RESYNC_NM: 10,         // inside this distance LOC silently re-aligns itself if it stops converging
        LOC_MAX_INTERCEPT: 35,     // max intercept angle
        LOC_MIN_JOIN_NM: 8,        // only keep the current heading if it meets the centreline at least this far before the ILS
        GS_RANGE_NM: 15,           // glideslope usable range ...
        GS_MAX_ANGLE: 8,           // ... when within this many degrees of the runway axis
        GS_CAPTURE_BELOW_FT: 250,  // captures when you are at most this far below the glide path ...
        GS_CAPTURE_ABOVE_FT: 1500, // ... or at most this far above it (then it descends to the path)
        GS_GAIN: 1.6,              // ft/min of extra V/S per ft of glide path error
        GS_SMOOTH_S: 1.2,          // V/S command smoothing time constant (s)
        GS_SLEW_FPM_S: 400,        // max change of the V/S command per second
        GS_VS_MIN: -2500,
        GS_VS_MAX: 0,              // never climb on the glideslope: too low = hold altitude until the path comes down
        DISARM_ON_DISCONNECT_NM: 5,// AP disconnect closer than this to the ILS -> LOC/GS switch off
        DEFAULT_SLOPE_DEG: 3,
        NAVAID_ELEV_IS_METERS: true,
        LNAV_WP_PASS_NM: 0.3,      // minimum distance at which a waypoint counts as passed (turns are anticipated, fly-by)
        LNAV_BANK_FRAC: 0.95,      // turn radius for waypoint anticipation uses this fraction of the max bank
        PRIME_PIDS: true,          // on engage: start GeoFS's pitch regulators from the current attitude/elevator (no leftover pull)
        // GeoFS's own pitch regulators are far too weak for some aircraft (slow, oscillating V/S).
        // Tuned live on the Concorde: elevator loop stronger, V/S -> pitch loop calmer.
        PITCH_TUNE_AIRCRAFT: [],   // (old Concorde tuning of GeoFS's PIDs - not used any more, the Concorde has its own pitch control)
        PITCH_TUNE: { elevKp: 0.03, elevKi: 0.005, vsKp: 0.004 },
        ALT_CAPTURE_DECEL: 35,     // altitude capture: V/S is reduced at most this fast (ft/min per second) -> smooth level-off
        VS_SLEW: 200,
        ALT_HOLD_GAIN: 7,          // Concorde mode: ft/min per ft off the selected altitude near it (was 4: faster corrections)              // commanded V/S never jumps: max change in ft/min per second
        PITCH_GUARD: false,        // emergency elevator takeover when the GeoFS AP loses pitch control (fights GeoFS, off by default)
        PG_VS_ERR: 1500,           // ... when V/S is this far (ft/min) from the target ...
        PG_PITCH_MARGIN: 5,        // ... or the nose is this many degrees past the aircraft's pitch limits ...
        PG_TRIGGER_S: 1.5,         // ... for this long
        PG_KP: 0.03,               // elevator per degree of pitch error
        PG_KI: 0.01,               // elevator per degree·second (finds the trim)
        PG_KD: 0.02,               // elevator per degree/second of pitch rate (damping)
        PG_RATE: 0.8,              // max elevator change per second
        PG_CAPTURE: 2.5,           // altitude capture: target V/S = ft to go x this (ft/min), smooth level-off
        ENGAGE_BLEND: false,       // (v1.4.9 attempt, replaced by OWN_PITCH)
        // ---- own pitch control (v1.6.5): the script flies the elevator itself for V/S, ALT and GS;
        //      GeoFS keeps doing heading/bank and throttle. Tuned on a model fitted to logged Concorde data.
        OWN_PITCH: true,
        // ---- sensitivity (all aircraft): GeoFS's own AP gains x this, outputs move smoothly (no abrupt corrections)
        SENS_PITCH: 1.4,           // V/S -> pitch loop (classic mode); the elevator loop itself is left as GeoFS has it
        SENS_ROLL: 1.4,            // heading -> bank loop (all aircraft)
        SENS_AILERON: 1.2,         // bank -> aileron loop (all aircraft)
        SMOOTH_PITCH_RATE: 0,      // 0 = elevator not rate limited (limiting GeoFS's elevator loop made it unstable in tests)
        SMOOTH_ROLL_RATE: 0.8,     // ailerons move at most this much per second
        SENS_GUARD_ON: 0.02,       // output see-saws more than this (rms) -> that axis's gain is lowered automatically
        SENS_GUARD_OFF: 0.006,
        CONCORDE_MODE: ['concorde'],   // aircraft (name contains) that get the Concorde mode: own pitch control, PID priming,
                                       // smooth V/S capture. All others fly GeoFS's own pitch logic, as before the Concorde work.
        OP_KQ: { concorde: 50, default: 50 },   // pitch rate (deg/s) per unit elevator at 220 kts (learnt in flight)
        OP_LEARN: true,            // learn that pitch response per aircraft while flying (stored in this browser)
        OP_VS_GAIN: 1.5,           // V/S error -> pitch: 1.0 = the pitch change that geometrically gives that V/S
        OP_VS_I: 0.2,              // how fast the pitch for a V/S is learnt (per s)
        OP_PITCH_RATE: 2.5,        // pitch target changes at most this many deg/s
        OP_T_PITCH: 0.8,           // pitch error -> pitch rate: rate = error / this (s)
        OP_Q_MAX: 3,               // commanded pitch rate is limited to this (deg/s)
        OP_KPQ: 1.0,               // pitch-rate loop gain
        OP_KIQ: 2.0,               // trim learning in the pitch-rate loop (per s)
        OP_ARREST_S: 0.2,          // on engage your pitch rate is taken over and faded out over this time
        OP_RATE: 1.0,              // elevator moves at most this much per second
        OP_Q_FILTER_S: 0.15,       // smoothing of the measured pitch rate
        OP_GUARD_ON: 0.008,        // oscillation guard: elevator see-sawing above this (rms) -> gains are lowered
        OP_GUARD_OFF: 0.003,       // ...and slowly raised again below this
        OP_GUARD_MAX: 6,           // gains are never lowered more than this factor
        OP_GUARD_RATE: 0.5,        // how fast the gains are lowered while it see-saws (per s)
        OP_GUARD_AFTER_S: 10,      // the guard is not active in the first seconds after engaging (take-over moves the yoke)
        ENGAGE_MATCH_FRAMES: 8,    // frames used to match the elevator PID to your stick
        JOYSTICK_OVERRIDE: true,   // AP on: joystick/throttle/pedals cannot fight the autopilot (set false to test without)
        OVERRIDE_THROTTLE: true,
        TICK_MS: 100
    };

    const NM_TO_M = 1852, M_TO_FT = 3.28084;
    const $ = sel => document.querySelector(sel);
    const log = (...a) => console.log('[AP+]', ...a);
    const FIELDS = { speed: 'setSpeed', course: 'setCourse', altitude: 'setAltitude', verticalSpeed: 'setVerticalSpeed' };

    // ------------------------------------------------------------ state
    const S = {
        loc: 'off', gs: 'off',
        gsVs: null,            // smoothed V/S commanded by the glideslope
        locPhase: null,        // null = decide, 'hold' = keep the heading, 'track' = follow the centreline
        locHold: null, locTurnHdg: null, locWatch: null, lastHdgT: null,
        vsCmd: null, lastVsT: 0,
        lnav: false, lnavIdx: 0,
        tgt: { speed: null, course: null, altitude: null, verticalSpeed: null },
        set: {},
        inTurnOn: false,
        apCmd: null, apThrottle: false, apThr: null,
        pg: { active: false, badT: 0, I: 0, out: undefined, lastTh: null, lastT: 0 },
        hooked: false,
        gsKts: null, track: null, crab: 0, lastPos: null,
        lastFrame: 0,
        loggedNavaid: null, lastG: null
    };
    window.GEOFS_APPLUS = S;

    // ------------------------------------------------------------ CSS
    const style = document.createElement('style');
    style.id = 'geofs-applus-style';
    style.textContent = `
    .geofs-autopilot-bar .geofs-autopilot-controls { display: block !important; }
    .geofs-autopilot-bar { overflow: visible !important; }
    #geofs-applus-pads { position: absolute; display: flex; gap: 4px; z-index: 5; white-space: nowrap; }
    #geofs-applus-pads .geofs-applus-pad { position: relative !important; margin: 0 !important; float: none !important; cursor: pointer; }
    #geofs-applus-pads .geofs-applus-status {
        position: absolute; left: 0; right: 0; top: 100%; margin-top: 2px; pointer-events: none;
        font: bold 8px/9px Arial, sans-serif; letter-spacing: .6px; text-align: center;
        color: #fff; text-shadow: 0 0 2px #000, 0 0 3px #000; white-space: nowrap; }
    #geofs-applus-pads .geofs-applus-status.armed    { color: #ffb340; }
    #geofs-applus-pads .geofs-applus-status.captured { color: #5dff6a; }
    /* pill shape like the AUTOPILOT pad, for the pad, its shadow and the coloured label */
    #geofs-applus-pads .geofs-applus-pad,
    #geofs-applus-pads .geofs-applus-pad .control-pad-label { border-radius: 999px !important; }
    #geofs-applus-pads .geofs-applus-pad .control-pad-label { overflow: hidden; }
    #geofs-applus-pads .geofs-applus-info {
        display: none; position: absolute; left: 50%; top: calc(100% + 14px); transform: translateX(-50%);
        padding: 4px 8px; border-radius: 6px; background: rgba(20, 24, 28, .88); color: #fff;
        font: 11px/14px Arial, sans-serif; white-space: nowrap; pointer-events: none; z-index: 10; }
    #geofs-applus-pads .geofs-applus-info:empty { display: none !important; }
    #geofs-applus-pads .geofs-applus-pad:hover .geofs-applus-info { display: block; }
    #geofs-applus-pads .geofs-applus-pad .control-pad-label.applus-amber {
        background: rgba(255, 152, 0, .72) !important; color: #fff !important; }
    `;
    document.head.appendChild(style);

    // ------------------------------------------------------------ helpers
    const rad = d => d * Math.PI / 180, deg = r => r * 180 / Math.PI;
    const angDiff = (a, b) => ((a - b + 540) % 360 + 360) % 360 - 180;
    const wrap360 = a => (a % 360 + 360) % 360;
    const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));
    function distNm(a, b) {
        const dLat = rad(b.lat - a.lat), dLon = rad(b.lon - a.lon);
        const h = Math.sin(dLat / 2) ** 2 + Math.cos(rad(a.lat)) * Math.cos(rad(b.lat)) * Math.sin(dLon / 2) ** 2;
        return 2 * 3440.065 * Math.asin(Math.min(1, Math.sqrt(h)));
    }
    function bearingDeg(a, b) {
        const dLon = rad(b.lon - a.lon);
        const y = Math.sin(dLon) * Math.cos(rad(b.lat));
        const x = Math.cos(rad(a.lat)) * Math.sin(rad(b.lat)) - Math.sin(rad(a.lat)) * Math.cos(rad(b.lat)) * Math.cos(dLon);
        return wrap360(deg(Math.atan2(y, x)));
    }
    const ap = () => (window.geofs && geofs.autopilot) || null;
    const av = () => (window.geofs && geofs.animation && geofs.animation.values) || {};
    function ownPos() {
        try {
            const l = geofs.aircraft.instance.llaLocation;
            if (l && isFinite(l[0]) && isFinite(l[1])) return { lat: l[0], lon: l[1], altM: l[2] };
        } catch (e) {}
        return null;
    }
    function planeHeading() {
        const v = av();
        if (typeof v.heading360 === 'number') return v.heading360;
        if (typeof v.heading === 'number') return wrap360(v.heading);
        try { return wrap360(geofs.aircraft.instance.htg); } catch (e) { return null; }
    }
    function altFt() {
        const v = av();
        if (typeof v.altitude === 'number') return v.altitude;
        const p = ownPos(); return p ? p.altM * M_TO_FT : null;
    }
    function norm(field, v) {
        const n = Number(v);
        if (v === null || v === undefined || v === '' || !isFinite(n)) return null;
        if (field === 'course') return Math.round(wrap360(n) * 10) / 10;
        if (field === 'speed') return n < 5 ? Math.round(n * 100) / 100 : Math.round(n);
        return Math.round(n);
    }
    // ground speed + ground track from our own position (cancels wind and any heading-reference offset)
    function updateTrack(me) {
        const now = performance.now();
        if (!me) return;
        if (S.lastPos) {
            const dtS = (now - S.lastPos.t) / 1000;
            const dNm = distNm(S.lastPos, me);
            if (dtS > 0.05) {
                const inst = dNm / (dtS / 3600);
                if (inst < 1500) S.gsKts = S.gsKts === null ? inst : S.gsKts * 0.85 + inst * 0.15;
                if (dNm * NM_TO_M > 3) {
                    const trk = bearingDeg(S.lastPos, me), hdg = planeHeading();
                    S.track = S.track === null ? trk : wrap360(S.track + angDiff(trk, S.track) * 0.25);
                    const turnRate = (hdg !== null && S.lastHdgT !== null) ? Math.abs(angDiff(hdg, S.lastHdgT)) / dtS : 99;
                    if (hdg !== null && S.gsKts > 40 && turnRate < 1.5) {   // only while flying straight
                        const c = angDiff(hdg, S.track);
                        if (Math.abs(c) < 25) S.crab = S.crab + (c - S.crab) * 0.05;
                    }
                    S.lastHdgT = hdg;
                }
            }
        }
        S.lastPos = { lat: me.lat, lon: me.lon, t: now };
    }
    const groundSpeedKts = () => (S.gsKts !== null && S.gsKts > 1) ? S.gsKts : (Number(av().kias) || 0);

    // ------------------------------------------------------------ navaid (ILS) access
    function nav1() { try { return geofs.nav.units.NAV1; } catch (e) { return null; } }
    function tunedILS() {
        const u = nav1(); if (!u || !u.navaid) return null;
        const n = u.navaid;
        const t = String(n.type || '').toUpperCase();
        const typeDef = geofs.nav.types && geofs.nav.types[t];
        const isILS = t === 'ILS' || (typeDef && typeDef.ILS) || u.VNAVCapable;
        if (!isILS) return null;
        const lat = Number(n.lat !== undefined ? n.lat : n.latitude);
        const lon = Number(n.lon !== undefined ? n.lon : n.longitude);
        if (!isFinite(lat) || !isFinite(lon)) return null;
        let course = n.course !== undefined ? n.course : (n.heading !== undefined ? n.heading : (n.bearing !== undefined ? n.bearing : n.hdg));
        if (!isFinite(Number(course))) course = Number(u.OBS) || 0;
        let slope = Number(n.slope !== undefined ? n.slope : (n.glideslope !== undefined ? n.glideslope : n.glideSlope));
        if (!isFinite(slope) || slope <= 0 || slope > 10) slope = CFG.DEFAULT_SLOPE_DEG;
        let elev = Number(n.elevation !== undefined ? n.elevation : (n.elev !== undefined ? n.elev : n.alt));
        if (!isFinite(elev)) elev = 0;
        const elevFt = CFG.NAVAID_ELEV_IS_METERS ? elev * M_TO_FT : elev;
        if (S.loggedNavaid !== n) {
            S.loggedNavaid = n; S.locPhase = null; S.crab = 0;
            log('ILS tuned:', n.id || n.name || n.frequency, { lat, lon, course: Number(course), slope, elevRaw: elev, elevFt: Math.round(elevFt), keys: Object.keys(n) });
        }
        return { nav: n, lat, lon, course: wrap360(Number(course)), slope, elevFt };
    }
    function extendILSRange() {
        try {
            const m = CFG.LOC_RANGE_NM * NM_TO_M;
            if (geofs.nav.types && geofs.nav.types.ILS && geofs.nav.types.ILS.range < m) geofs.nav.types.ILS.range = m;
            const u = nav1();
            if (u && u.navaid && typeof u.navaid.range === 'number' && u.navaid.range < m &&
                String(u.navaid.type).toUpperCase() === 'ILS') u.navaid.range = m;
        } catch (e) {}
    }

    // ------------------------------------------------------------ flight plan (NAV button)
    function fpWaypoints() {
        try {
            const fp = geofs.flightPlan;
            let a = fp && (fp.waypointArray || fp.waypoints);
            if (!a && fp && typeof fp.getWaypointArray === 'function') a = fp.getWaypointArray();
            return Array.isArray(a) ? a.filter(w => w && isFinite(w.lat) && isFinite(w.lon)) : [];
        } catch (e) { return []; }
    }

    // ------------------------------------------------------------ UI: LOC / GS pads next to the panel
    // LOC / GS are clones of the native AUTOPILOT pad, so they get exactly the same pill shape and colours
    const WANT = { loc: 'transp-pad', gs: 'transp-pad' };
    function makePad(native, key, text) {
        const p = native.cloneNode(true);             // markup + classes only, no GeoFS event handlers
        const cs = getComputedStyle(native);
        ['id', 'for', 'title', 'data-tooltip-classname', 'data-upgraded', 'tabindex'].forEach(a => p.removeAttribute(a));
        p.querySelectorAll('[title]').forEach(e => e.removeAttribute('title'));
        p.classList.remove('geofs-autopilot-pad', 'green-pad');   // no GeoFS hover/tooltip/click hooks on our pads
        p.style.width = cs.width; p.style.height = cs.height;
        p.classList.add('geofs-applus-pad');
        p.dataset.applus = key;
        const l = p.querySelector('.control-pad-label');
        l.textContent = text;
        l.classList.remove('green-pad', 'orange-pad', 'red-pad', 'applus-amber');
        l.classList.add('transp-pad');
        const st = document.createElement('span');
        st.className = 'geofs-applus-status';
        p.appendChild(st);
        const info = document.createElement('span');
        info.className = 'geofs-applus-info';
        p.appendChild(info);
        // if GeoFS ever recolours every autopilot pad, put our own colour straight back
        new MutationObserver(() => {
            const w = WANT[key];
            if (!l.classList.contains(w) || ['green-pad', 'orange-pad', 'applus-amber', 'transp-pad'].some(c => c !== w && l.classList.contains(c))) {
                l.classList.remove('green-pad', 'orange-pad', 'applus-amber', 'transp-pad');
                l.classList.add(w);
            }
        }).observe(l, { attributes: true, attributeFilter: ['class'] });
        return p;
    }
    function buildButtons() {
        const bar = $('.geofs-autopilot-bar');
        if (!bar || $('#geofs-applus-pads')) return;
        const native = bar.querySelector('.geofs-autopilot-pad');
        if (!native || !native.querySelector('.control-pad-label')) return;
        if (getComputedStyle(bar).position === 'static') bar.style.position = 'relative';
        const wrap = document.createElement('div');
        wrap.id = 'geofs-applus-pads';
        wrap.appendChild(makePad(native, 'loc', 'LOC'));
        wrap.appendChild(makePad(native, 'gs', 'GS'));
        bar.appendChild(wrap);
        wrap.querySelector('[data-applus="loc"]').addEventListener('click', e => { e.stopPropagation(); toggleLOC(); });
        wrap.querySelector('[data-applus="gs"]').addEventListener('click', e => { e.stopPropagation(); toggleGS(); });
        ['mousedown', 'mouseup', 'pointerdown', 'pointerup', 'touchstart', 'touchend', 'dblclick', 'mouseover', 'mouseout', 'pointerover', 'pointerout'].forEach(ev => wrap.addEventListener(ev, e => e.stopPropagation()));
        log('LOC / GS pads added');
    }
    // same height / shape as the AUTOPILOT pad, directly right of the value row
    let lastLayout = '';
    function layoutButtons() {
        const bar = $('.geofs-autopilot-bar'), wrap = $('#geofs-applus-pads');
        if (!bar || !wrap) return;
        const pad = bar.querySelector('.geofs-autopilot-pad');
        const ctl = bar.querySelector('.geofs-autopilot-controls');
        if (!pad) return;
        const br = bar.getBoundingClientRect(), pr = pad.getBoundingClientRect();
        let right = pr.right;
        if (ctl) ctl.querySelectorAll('.geofs-autopilot-control').forEach(c => {
            const r = c.getBoundingClientRect();
            if (r.width && r.right > right) right = r.right;
        });
        const left = Math.round(right - br.left + 6), top = Math.round(pr.top - br.top), h = Math.round(pr.height);
        const key = left + '|' + top + '|' + h;
        if (key === lastLayout || !h) return;
        lastLayout = key;
        wrap.style.left = left + 'px';
        wrap.style.top = top + 'px';
    }
    const TIPS = { loc: '', gs: '' };
    function setBtn(which, state, text, tip) {
        const b = document.querySelector(`.geofs-applus-pad[data-applus="${which}"]`);
        if (!b) return;
        const lab = b.querySelector('.control-pad-label'), st = b.querySelector('.geofs-applus-status');
        const want = state === 'captured' ? 'green-pad' : state === 'armed' ? 'applus-amber' : 'transp-pad';
        WANT[which] = want;
        if (!lab.classList.contains(want)) {
            lab.classList.remove('green-pad', 'transp-pad', 'orange-pad', 'applus-amber');
            lab.classList.add(want);
        }
        const cls = 'geofs-applus-status ' + state;
        if (st.className !== cls) st.className = cls;
        if (st.textContent !== text) st.textContent = text;
        if (tip !== undefined && TIPS[which] !== tip) {
            TIPS[which] = tip;
            const info = b.querySelector('.geofs-applus-info');
            if (info) info.textContent = tip;
        }
        if (b.hasAttribute('title')) b.removeAttribute('title');
    }
    function flash(which) {
        const b = document.querySelector(`.geofs-applus-pad[data-applus="${which}"] .control-pad-label`);
        if (!b) return;
        b.style.outline = '2px solid #ff4a3d';
        setTimeout(() => { b.style.outline = ''; }, 500);
    }

    // ------------------------------------------------------------ LOC / GS logic
    function ilsGeometry() {
        const ils = tunedILS(), me = ownPos();
        if (!ils || !me) return null;
        const d = distNm(me, ils);
        const brg = bearingDeg(me, ils);
        const dev = angDiff(brg, ils.course);                 // + = left of the centreline -> turn right
        const xtk = d * Math.sin(rad(dev));                    // cross-track NM, + = left
        const gpAlt = ils.elevFt + Math.tan(rad(ils.slope)) * d * NM_TO_M * M_TO_FT;
        const alt = altFt();
        return { ils, d, brg, dev, xtk, gpAlt, alt, err: alt === null ? null : gpAlt - alt };
    }
    const locInRange = g => !!g && g.d <= CFG.LOC_RANGE_NM;
    const gsInCone = g => !!g && g.d <= CFG.GS_RANGE_NM && Math.abs(g.dev) <= CFG.GS_MAX_ANGLE;

    function locCanCapture(g) {
        if (!locInRange(g) || Math.abs(g.dev) > CFG.LOC_CAPTURE_MAX_DEV) return false;
        const hdg = planeHeading();
        return hdg === null || Math.abs(angDiff(hdg, g.ils.course)) <= CFG.LOC_CAPTURE_MAX_HDG;
    }
    const gsCanCapture = g => gsInCone(g) && g.err !== null &&
        g.err <= CFG.GS_CAPTURE_BELOW_FT && g.err >= -CFG.GS_CAPTURE_ABOVE_FT;

    function captureLOC(g) {
        S.loc = 'captured'; S.lnav = false; S.locPhase = null; S.locWatch = null;
        log('LOC captured at', g.d.toFixed(1), 'NM, xtk', Math.round(g.xtk * NM_TO_M), 'm');
    }
    function captureGS(g) {
        S.gs = 'captured';
        S.gsVs = Math.min(0, Number(av().verticalSpeed) || 0);
        log('GS captured at', g.d.toFixed(1), 'NM,', Math.round(-g.err), 'ft from the path');
    }
    // armed -> captured as soon as possible; captured -> armed when you leave the usable area
    function checkCaptures(g) {
        if (!g) return;
        if (S.loc === 'armed' && locCanCapture(g)) captureLOC(g);
        else if (S.loc === 'captured' && !locInRange(g)) { S.loc = 'armed'; S.locPhase = null; }
        if (S.gs === 'armed' && gsCanCapture(g)) captureGS(g);
        else if (S.gs === 'captured' && g.d > CFG.GS_RANGE_NM + 1) { S.gs = 'armed'; S.gsVs = null; }
    }

    // button: off -> captured (when AVAIL) or armed;  armed/captured -> off
    function toggleLOC() {
        if (S.loc !== 'off') { S.loc = 'off'; S.locPhase = null; log('LOC off'); return; }
        const g = ilsGeometry();
        if (!g) { flash('loc'); return; }
        S.lnav = false;
        if (locCanCapture(g)) captureLOC(g);
        else { S.loc = 'armed'; log('LOC armed'); }
    }
    function toggleGS() {
        if (S.gs !== 'off') { S.gs = 'off'; S.gsVs = null; log('GS off'); return; }
        const g = ilsGeometry();
        if (!g) { flash('gs'); return; }
        if (gsCanCapture(g)) captureGS(g);
        else { S.gs = 'armed'; log('GS armed'); }
    }

    // planned turn-in: bank, radius and turn rate follow from the ground speed (faster = wider, earlier)
    function turnPlan(bankOverride) {
        const v = Math.max(60, groundSpeedKts()) * 0.514444;          // m/s
        let maxBank = 25, ratio = 1;
        try {
            const d = geofs.autopilot.definition;
            if (Number(d.maxBankAngle) > 0) maxBank = Number(d.maxBankAngle);
            if (Number(d.targetBankAngleRatio) > 0) ratio = Number(d.targetBankAngleRatio);
        } catch (e) {}
        const bank = bankOverride ? clamp(bankOverride, 5, maxBank) : Math.max(8, maxBank * CFG.LOC_TURN_BANK_FRAC);
        const R = v * v / (9.81 * Math.tan(rad(bank)));               // turn radius (m)
        return { v, bank, R, maxBank, rate: deg(v / R), lead: bank / ratio };  // rate in deg/s, lead = heading offset that gives `bank`
    }
    // cross-track distance (m) at which a turn of `interceptDeg` must start to roll out on the centreline
    function turnLeadM(interceptDeg) {
        const P = turnPlan();
        const th = rad(Math.min(90, Math.abs(interceptDeg)));
        return P.R * (1 - Math.cos(th)) + P.v * CFG.LOC_LEAD_LAG_S * Math.sin(th);
    }

    // runs every simulation frame, inside geofs.autopilot.update -> smooth like the vanilla ILS
    function navFrame(A) {
        const now = performance.now();
        const dt = S.lastFrame ? clamp((now - S.lastFrame) / 1000, 0, 0.25) : 0;
        S.lastFrame = now;
        if ((S.loc === 'off' && S.gs === 'off') || dt === 0) return;
        const g = ilsGeometry();
        if (!g) return;
        checkCaptures(g);

        // ---- LOC
        if (S.loc === 'captured') {
            const xtkM = g.xtk * NM_TO_M;
            const ph = planeHeading();
            const trk = S.track !== null ? S.track : ph;
            const ia = trk === null ? 0 : angDiff(trk, g.ils.course);           // + = track right of the course
            const converging = Math.sign(ia) === Math.sign(g.xtk) && Math.abs(ia) >= CFG.LOC_MIN_INTERCEPT && Math.abs(ia) <= 100;

            // where would the current track meet the centreline (NM before the ILS antenna)?
            const tanIa = Math.tan(rad(Math.min(89, Math.abs(ia))));
            const joinNm = tanIa > 1e-3 ? g.d * Math.cos(rad(g.dev)) - Math.abs(g.xtk) / tanIa : -Infinity;
            const joinsEarly = joinNm >= CFG.LOC_MIN_JOIN_NM;
            // captured while banking: roll out on the present heading first, then decide (unless already turning the right way, late)
            const roll = Number(av().aroll) || 0;                             // aroll < 0 = turning right
            const turningDir = Math.abs(roll) > 5 ? (roll < 0 ? 1 : -1) : 0;
            const needDir = Math.sign(angDiff(g.ils.course, trk === null ? 0 : trk));
            const lateTurn = converging && Math.abs(ia) > 20 && !(joinsEarly && Math.abs(xtkM) > turnLeadM(ia));
            let rollOut = false;
            if (S.locPhase === null && turningDir !== 0 && ph !== null && !(lateTurn && turningDir === needDir)) {
                if (!S.locRollOut) { S.locRollOut = true; log('LOC: rolling out on the present heading first'); }
                rollOut = true;
            } else S.locRollOut = false;
            const closing = Math.sign(ia) === Math.sign(g.xtk) && Math.abs(ia) >= CFG.LOC_MIN_INTERCEPT;
            if (S.locPhase === null && !rollOut && closing && Math.abs(ia) > 100 && Math.abs(xtkM) > turnLeadM(90) && ph !== null) {
                // pointing more than 100° away from the runway course (e.g. still on the base turn): turn to a 90° intercept, then hold
                S.locPhase = 'hold'; S.locHold = Math.round(wrap360(g.ils.course + Math.sign(ia) * 90)) % 360;
                log('LOC: turning to a 90° intercept heading', S.locHold + ', holding it until the turn-in point');
            }
            if (S.locPhase === null && !rollOut) {
                if (converging && joinsEarly && Math.abs(xtkM) > turnLeadM(ia) && ph !== null) {
                    S.locPhase = 'hold'; S.locHold = Math.round(ph) % 360;
                    log('LOC: holding heading', S.locHold, 'until the turn-in point');
                } else if (converging && Math.abs(ia) > 20 && ph !== null) {
                    S.locPhase = 'turn'; S.locBank = null; S.locTurnHdg = ph;
                    log('LOC: captured late, turning onto the course now');
                } else S.locPhase = 'track';
            }
            if (S.locPhase === 'hold' && !joinsEarly && Math.abs(xtkM) > turnLeadM(ia)) {
                S.locPhase = 'track';
                log('LOC: heading would join too close to the runway, intercepting the centreline instead');
            }
            if (S.locPhase === 'hold' && (!closing || Math.abs(xtkM) <= turnLeadM(ia))) {
                S.locPhase = 'turn'; S.locBank = null; S.locTurnHdg = ph !== null ? ph : S.locHold;
                const P = turnPlan();
                log('LOC: turning in,', Math.round(xtkM), 'm from the centreline, bank', Math.round(P.bank) + '°, radius', Math.round(P.R), 'm');
            }

            const corr = clamp(g.xtk * CFG.LOC_K, -CFG.LOC_MAX_INTERCEPT, CFG.LOC_MAX_INTERCEPT);
            const trackHdg = wrap360(g.ils.course + corr + S.crab);     // fly the track, not the heading (wind)
            let hdg;
            if (rollOut) hdg = wrap360(ph);
            else if (S.locPhase === 'hold') hdg = S.locHold;
            else if (S.locPhase === 'turn') {
                // a steady, constant-bank turn: the commanded heading walks towards the centreline at the planned rate
                // bank adapts: gentle when there is room, steeper (up to the aircraft's max) when the centreline is close
                let P = turnPlan();
                const th = Math.abs(angDiff(g.ils.course, trk === null ? S.locTurnHdg : trk));
                if (th > 2) {
                    const rReq = Math.abs(xtkM) / (1 - Math.cos(rad(Math.min(th, 180)))) - P.v * 2;
                    const bankReq = deg(Math.atan(P.v * P.v / (9.81 * Math.max(rReq, 200))));
                    if (th > 15) P = turnPlan(Math.max(CFG.LOC_MIN_BANK, bankReq));   // wind / early start -> looser, late -> tighter
                    else if (bankReq > P.bank) P = turnPlan(bankReq);
                    S.locBank = P.bank;
                } else if (S.locBank) P = turnPlan(S.locBank);
                const finalHdg = wrap360(g.ils.course + S.crab + clamp(g.xtk * CFG.LOC_K, -5, 5));
                const rem = angDiff(finalHdg, S.locTurnHdg);
                if (Math.abs(rem) < 3) { S.locPhase = 'track'; hdg = trackHdg; log('LOC: on course,', Math.round(xtkM), 'm from the centreline'); }
                else {
                    S.locTurnHdg = wrap360(S.locTurnHdg + Math.sign(rem) * Math.min(Math.abs(rem), P.rate * dt));
                    const left = angDiff(finalHdg, S.locTurnHdg);
                    hdg = wrap360(S.locTurnHdg + Math.sign(left) * Math.min(Math.abs(left), P.lead));
                }
            } else {
                hdg = trackHdg;

                // silent re-alignment in the last part: not getting closer for 8 s -> start over internally
                if (g.d <= CFG.LOC_RESYNC_NM && Math.abs(xtkM) > 40) {
                    if (!S.locWatch) S.locWatch = { t: now, x: Math.abs(xtkM) };
                    else if (now - S.locWatch.t > 8000) {
                        if (Math.abs(xtkM) >= S.locWatch.x - 10) {
                            S.crab = 0; S.locPhase = null;
                            log('LOC: silent re-align,', Math.round(xtkM), 'm off');
                        }
                        S.locWatch = null;
                    }
                } else S.locWatch = null;
            }
            if (A.mode !== 'HDG' && origSetMode) origSetMode.call(A, 'HDG');
            A.values.course = hdg;
            S.tgt.course = norm('course', hdg);
        }

        // ---- GS
        if (S.gs === 'captured' && g.err !== null) {
            const pathVs = -groundSpeedKts() * 101.2686 * Math.tan(rad(g.ils.slope));
            const raw = clamp(pathVs + g.err * CFG.GS_GAIN, CFG.GS_VS_MIN, CFG.GS_VS_MAX);
            if (S.gsVs === null) S.gsVs = raw;
            const lp = S.gsVs + (raw - S.gsVs) * Math.min(1, dt / CFG.GS_SMOOTH_S);
            const maxStep = CFG.GS_SLEW_FPM_S * dt;
            S.gsVs = Math.min(0, S.gsVs + clamp(lp - S.gsVs, -maxStep, maxStep));
            A.values.verticalSpeed = S.gsVs;
            S.tgt.verticalSpeed = Math.round(S.gsVs);
        }
    }

    // the waypoint the user selected in the GeoFS flight plan (if GeoFS exposes it)
    function fpSelectedIndex(wps) {
        try {
            const fp = geofs.flightPlan || {};
            const sel = fp.trackedWaypoint || fp.activeWaypoint || fp.selectedWaypoint || fp.currentWaypoint;
            if (sel) {
                let i = wps.indexOf(sel);
                if (i < 0 && sel.id !== undefined) i = wps.findIndex(w => w.id === sel.id);
                if (i < 0 && isFinite(sel.lat)) i = wps.findIndex(w => Math.abs(w.lat - sel.lat) < 1e-6 && Math.abs(w.lon - sel.lon) < 1e-6);
                if (i >= 0) return { i, ref: sel };
            }
            for (const k of ['trackedWaypointIndex', 'activeWaypointIndex', 'currentWaypointIndex']) {
                if (Number.isInteger(fp[k]) && fp[k] >= 0 && fp[k] < wps.length) return { i: fp[k], ref: k + ':' + fp[k] };
            }
        } catch (e) {}
        return null;
    }
    function lnavTurnRadiusNm() {
        let maxBank = 25;
        try { const d = geofs.autopilot.definition; if (Number(d.maxBankAngle) > 0) maxBank = Number(d.maxBankAngle); } catch (e) {}
        const v = Math.max(60, groundSpeedKts()) * 0.514444;
        return v * v / (9.81 * Math.tan(rad(maxBank * CFG.LNAV_BANK_FRAC))) / NM_TO_M;
    }
    function lnavStep() {
        const A = ap(); const wps = fpWaypoints(), me = ownPos();
        if (!A || !me || !wps.length) { S.lnav = false; log('NAV off: no flight plan'); return; }
        // the user picked another waypoint in the flight plan -> fly to that one
        const sel = fpSelectedIndex(wps);
        if (sel && sel.ref !== S.lnavSelRef) {
            S.lnavSelRef = sel.ref;
            if (sel.i !== S.lnavIdx) { S.lnavIdx = sel.i; log('NAV: direct to selected waypoint #' + (sel.i + 1)); }
        }
        if (S.lnavIdx >= wps.length) { S.lnav = false; log('NAV: end of flight plan'); return; }
        const wp = wps[S.lnavIdx], d = distNm(me, wp), brg = bearingDeg(me, wp);
        const R = lnavTurnRadiusNm();
        const trk = S.track !== null ? S.track : planeHeading();
        const rel = trk === null ? 0 : Math.abs(angDiff(brg, trk));
        const next = wps[S.lnavIdx + 1];
        // fly-by: start the turn to the next leg early enough to roll out on it (faster = earlier)
        let lead = CFG.LNAV_WP_PASS_NM;
        if (next) {
            const turn = Math.abs(angDiff(bearingDeg(wp, next), brg));
            lead = Math.max(lead, R * Math.tan(rad(Math.min(turn, 120) / 2)) + groundSpeedKts() / 3600 * 3);
        }
        const passed = d < lead;
        // a waypoint inside the turn circle (abeam or behind) can never be reached -> go on instead of circling it
        const unreachable = d < 2 * R && rel > 75;
        if (passed || unreachable) {
            S.lnavIdx++;
            if (S.lnavIdx >= wps.length) { S.lnav = false; log('NAV: end of flight plan, holding the present heading'); return; }
            log('NAV: waypoint #' + S.lnavIdx + (unreachable && !passed ? ' could not be reached in a turn, ' : ' passed, ') + 'next #' + (S.lnavIdx + 1));
            return;
        }
        // write the course directly (setCourse would switch GeoFS to HDG mode and end NAV)
        const hdg = Math.round(brg * 10) / 10 % 360;
        if (A.mode !== 'HDG' && origSetMode) origSetMode.call(A, 'HDG');
        A.values.course = hdg;
        S.tgt.course = norm('course', hdg);
    }
    function startLNAV() {
        const wps = fpWaypoints(), me = ownPos();
        if (!wps.length || !me) return false;
        let sel = fpSelectedIndex(wps);
        S.lnavSelRef = sel ? sel.ref : null;
        if (sel) {   // a selection made long ago that is now behind you is ignored (e.g. the departure point)
            const trk = S.track !== null ? S.track : planeHeading();
            const behind = trk !== null && Math.abs(angDiff(bearingDeg(me, wps[sel.i]), trk)) > 100;
            const recent = performance.now() - (S.fpSelT || 0) < 120000;
            if (behind && !recent) sel = null;
        }
        if (sel) S.lnavIdx = sel.i;                       // the waypoint you selected in the flight plan
        else {
            let best = 0, bd = Infinity;
            wps.forEach((w, i) => { const d = distNm(me, w); if (d < bd) { bd = d; best = i; } });
            S.lnavIdx = bd < 1.5 ? Math.min(best + 1, wps.length - 1) : best;
        }
        S.lnav = true; S.loc = 'off';
        log('LNAV on, waypoint #' + (S.lnavIdx + 1) + '/' + wps.length);
        return true;
    }

    // ------------------------------------------------------------ panel values: only you (or LOC/GS) change them
    function record(field, v) {
        const n = norm(field, v);
        if (n === null) return;
        S.tgt[field] = n;
        S.set[field] = true;
    }
    function restoreValues() {
        const A = ap(); if (!A || !A.values) return;
        Object.keys(FIELDS).forEach(f => {
            const t = S.tgt[f];
            if (t === null) return;
            if (f === 'course' && (S.loc === 'captured' || S.lnav)) return;   // LOC / NAV write the exact value themselves
            if (f === 'verticalSpeed' && S.gs === 'captured') return;   // GS idem
            if (Number(A.values[f]) !== t || A.values[f] === null) A.values[f] = t;
        });
    }
    function inputFor(field) {
        const A = ap();
        if (field === 'speed') return $('.geofs-autopilot-bar ' + (A && A.speedMode === 'mach' ? '.geofs-autopilot-mach' : '.geofs-autopilot-knots'));
        const map = { course: '.geofs-autopilot-course', altitude: '.geofs-autopilot-altitude', verticalSpeed: '.geofs-autopilot-verticalSpeed' };
        return $('.geofs-autopilot-bar ' + map[field]);
    }
    function fmt(field, v) {
        if (field === 'course') return String(Math.round(wrap360(v)) % 360).padStart(3, '0');
        if (field === 'altitude') return String(Math.max(0, Math.round(v))).padStart(5, '0');
        if (field === 'verticalSpeed') { const r = Math.round(v / 10) * 10; return (r < 0 ? '-' : '') + String(Math.abs(r)).padStart(4, '0'); }
        if (field === 'speed' && v < 5) return v.toFixed(2);
        return String(Math.round(v));
    }
    function writeDisplays() {
        Object.keys(FIELDS).forEach(f => {
            const t = S.tgt[f], inp = inputFor(f);
            if (t === null || !inp || document.activeElement === inp) return;
            const s = fmt(f, t);
            if (inp.value === s || norm(f, inp.value) === norm(f, s)) return;
            inp.value = s;
        });
    }
    function bindInputs() {
        const bar = $('.geofs-autopilot-bar');
        if (!bar || bar.__applusBound) return;
        bar.__applusBound = true;
        const readLater = (field, inp) => setTimeout(() => {
            const n = norm(field, inp.value);
            if (n === null) return;
            if (field === 'course' && S.loc === 'captured') return;
            if (field === 'course' && S.lnav) { S.lnav = false; log('NAV off: heading selected by hand'); }
            if (field === 'verticalSpeed' && S.gs === 'captured') return;
            if (norm(field, fmt(field, S.tgt[field] === null ? NaN : S.tgt[field])) === n) return;
            record(field, n);
            const A = ap(); if (A && A.values) A.values[field] = n;
        }, 0);
        const all = [['speed', '.geofs-autopilot-knots'], ['speed', '.geofs-autopilot-mach'], ['course', '.geofs-autopilot-course'],
                     ['altitude', '.geofs-autopilot-altitude'], ['verticalSpeed', '.geofs-autopilot-verticalSpeed']];
        all.forEach(([field, sel]) => {
            const inp = bar.querySelector(sel); if (!inp) return;
            ['change', 'wheel'].forEach(ev => inp.addEventListener(ev, () => readLater(field, inp), { passive: true }));
            inp.addEventListener('keydown', e => { if (e.key === 'Enter') readLater(field, inp); });
            inp.addEventListener('blur', () => readLater(field, inp));
            const ctl = inp.closest('.geofs-autopilot-control');
            if (ctl && !ctl.__applusBtns) {
                ctl.__applusBtns = true;
                ctl.querySelectorAll('.numberUp,.numberDown').forEach(a =>
                    a.addEventListener('click', () => readLater(field, inputFor(field) || inp)));
            }
        });
    }

    // ------------------------------------------------------------ hooks into geofs.autopilot
    // ------------------------------------------------------------ pitch guard
    // The GeoFS AP of some aircraft (Concorde, F-16) at low speed keeps pulling and lets V/S run away.
    // Then we fly the elevator ourselves: V/S target -> pitch attitude target -> elevator (PID).
    function guardTargetVs(A) {
        if (S.gs === 'captured' && S.gsVs !== null) return S.gsVs;
        const cur = altFt();
        if (S.tgt.altitude !== null && cur !== null) {
            const diff = S.tgt.altitude - cur, d = A.definition || {};
            let mag = Math.abs(Number(S.tgt.verticalSpeed));
            if (!isFinite(mag) || mag < 100) mag = Math.abs(diff >= 0 ? (d.baseClimbrate || 700) : (d.baseDescentrate || 1000));
            return Math.sign(diff) * Math.min(mag, Math.abs(diff) * CFG.PG_CAPTURE);
        }
        const t = Number(A.targetVerticalSpeed);
        return isFinite(t) ? t : 0;
    }
    // GeoFS keeps the integral of its pitch PIDs from the last engagement (e.g. "17° nose up" + "full pull").
    // Start them from the current situation instead, so the AP takes over without a jolt.
    function primePIDs(A) {
        if (!CFG.PRIME_PIDS) return;
        const P = A.PIDs, v = av(), c = window.controls;
        if (!P) return;
        const th = -(Number(v.atilt) || 0);                                   // pitch attitude, + = nose up
        const el = clamp(Number(c && c.rawPitch) || 0, -1, 1);
        const pa = P.pitchAngle, ep = P.elevatorPitch;
        if (pa && '_integral' in pa) {
            const lo = isFinite(pa._minOutput) ? pa._minOutput : -20, hi = isFinite(pa._maxOutput) ? pa._maxOutput : 20;
            pa._integral = clamp(th, lo, hi);
            pa._previousInput = Number(v.verticalSpeed) || 0; pa._previousError = 0;   // this PID works in ft/min (verified live)
        }
        if (ep && '_integral' in ep) {
            ep._integral = el;
            ep._previousInput = th; ep._previousError = 0;
        }
        let tuned = '';
        try {
            const name = String(geofs.aircraft.instance.aircraftRecord.name || '').toLowerCase();
            const list = CFG.PITCH_TUNE_AIRCRAFT || [];
            if (list.includes('*') || list.some(n => name.includes(n))) {
                const T = CFG.PITCH_TUNE;
                if (ep) { ep._kp = T.elevKp; ep._ki = T.elevKi; }
                if (pa) { pa._kp = T.vsKp; }
                tuned = ' (pitch tuning for ' + name + ')';
            }
        } catch (e) {}
        log('pitch PIDs primed: pitch', th.toFixed(1) + '°, elevator', el.toFixed(2) + tuned);
    }
    function resetGuard() { S.pg = { active: false, badT: 0, I: 0, out: undefined, lastTh: null, lastT: 0 }; }
    function pitchGuard(A) {
        if (!CFG.PITCH_GUARD) return;
        const v = av(), c = window.controls;
        if (!c) return;
        const now = performance.now();
        const dt = S.pg.lastT ? clamp((now - S.pg.lastT) / 1000, 0, 0.25) : 0;
        S.pg.lastT = now;
        const vs = Number(v.verticalSpeed), th = -Number(v.atilt);          // th: + = nose up
        if (!isFinite(vs) || !isFinite(th) || dt === 0) return;
        const d = A.definition || {};
        const maxP = Number(d.maxPitchAngle) > 0 ? Number(d.maxPitchAngle) : 20;
        const minP = Number(d.minPitchAngle) < 0 ? Number(d.minPitchAngle) : -15;
        const vsT = guardTargetVs(A);

        if (!S.pg.active) {
            const bad = Math.abs(vs - vsT) > CFG.PG_VS_ERR || th > maxP + CFG.PG_PITCH_MARGIN || th < minP - CFG.PG_PITCH_MARGIN;
            S.pg.badT = bad ? S.pg.badT + dt : 0;
            if (S.pg.badT < CFG.PG_TRIGGER_S) { S.pg.lastTh = th; return; }
            S.pg.active = true;
            S.pg.I = Number(c.rawPitch) || 0;                                // start from the current elevator: no jolt
            S.pg.out = S.pg.I;
            log('pitch guard ON: V/S', Math.round(vs), 'target', Math.round(vsT), 'pitch', th.toFixed(1) + '°');
        }

        const tas = Math.max(80, Number(v.ktas) || Number(v.kias) || 200) * 101.27;   // ft/min per unit sin(gamma)
        const gamma = deg(Math.asin(clamp(vs / tas, -0.9, 0.9)));
        const gammaT = deg(Math.asin(clamp(vsT / tas, -0.9, 0.9)));
        const aoa = clamp(th - gamma, -5, 20);
        const thT = clamp(gammaT + aoa, minP, maxP);                         // attitude that gives the target V/S
        const q = S.pg.lastTh === null ? 0 : (th - S.pg.lastTh) / dt;
        S.pg.lastTh = th;
        const e = thT - th;
        S.pg.I = clamp(S.pg.I + CFG.PG_KI * e * dt, -1, 1);
        const raw = clamp(S.pg.I + CFG.PG_KP * e - CFG.PG_KD * q, -1, 1);
        S.pg.out = S.pg.out + clamp(raw - S.pg.out, -CFG.PG_RATE * dt, CFG.PG_RATE * dt);
        c.rawPitch = S.pg.out;
        c.pitch = S.pg.out;
        if (S.apCmd) { S.apCmd.rawPitch = S.pg.out; S.apCmd.pitch = S.pg.out; }
        else S.apCmd = { rawPitch: S.pg.out, pitch: S.pg.out };
    }

    let origSetMode = null, origTurnOn = null, origTurnOff = null, origUpdate = null;
    const MY = {};

    function hookAutopilot() {
        const A = ap();
        if (!A || S.hooked || typeof A.turnOn !== 'function' || !A.values) return;
        S.hooked = true;

        Object.keys(FIELDS).forEach(f => { S.tgt[f] = norm(f, A.values[f]); });

        Object.keys(FIELDS).forEach(f => {
            const o = A[FIELDS[f]];
            if (typeof o !== 'function') return;
            A[FIELDS[f]] = function (v) {
                const r = o.apply(this, arguments);
                if (!S.inTurnOn) record(f, (v !== undefined && v !== null && v !== '' && isFinite(Number(v))) ? v : (this.values ? this.values[f] : null));
                return r;
            };
        });

        origSetMode = A.setMode;
        A.setMode = MY.setMode = function (mode) {
            if (String(mode).toUpperCase() === 'NAV') {
                if (!startLNAV()) { const b = $('.geofs-autopilot-NAV'); if (b) { b.style.outline = '2px solid #ff4a3d'; setTimeout(() => b.style.outline = '', 500); } }
                return origSetMode.call(this, 'HDG');
            }
            S.lnav = false;
            return origSetMode.apply(this, arguments);
        };

        origTurnOn = A.turnOn;
        A.turnOn = MY.turnOn = function () {
            const keep = Object.assign({}, S.tgt), wasSet = Object.assign({}, S.set);
            S.inTurnOn = true;
            let r;
            try { r = origTurnOn.apply(this, arguments); } finally { S.inTurnOn = false; }
            try {
                Object.keys(FIELDS).forEach(f => {
                    let v;
                    if (wasSet[f] && keep[f] !== null) v = keep[f];
                    else if (f === 'verticalSpeed') v = Math.round((Number(av().verticalSpeed) || 0) / 100) * 100;
                    else if (f === 'altitude') v = Math.round((Number(this.values.altitude) || altFt() || 0) / 100) * 100;
                    else if (f === 'course') v = norm('course', this.values.course !== null && this.values.course !== undefined ? this.values.course : planeHeading());
                    else v = norm('speed', this.values.speed !== null && this.values.speed !== undefined ? this.values.speed : av().kias);
                    if (v === null || v === undefined || !isFinite(v)) return;
                    S.tgt[f] = norm(f, v); S.set[f] = true;
                    this.values[f] = S.tgt[f];
                });
            } catch (e) { console.warn('[AP+] engage restore failed', e); }
            S.apThrottle = false; S.apThr = null; S.lastFrame = 0; resetGuard(); S.vsCmd = null; S.lastVsT = 0;
            if (concordeMode()) { try { primePIDs(this); } catch (e) { console.warn('[AP+] prime PIDs', e); } }
            log(concordeMode() ? 'Concorde mode' : 'classic mode (GeoFS pitch logic)');
            S.apVals = null; S.sm = null; S.engageT = performance.now(); S.userElev = {}; S.engageFrames = 0; S.engageF = null; S.op = null;
            writeDisplays();
            log('AP engaged with', Object.assign({}, S.tgt));
            return r;
        };

        origTurnOff = A.turnOff;
        A.turnOff = MY.turnOff = function () {
            const g = S.lastG || ilsGeometry();
            const close = g && g.d <= CFG.DISARM_ON_DISCONNECT_NM;
            if (close) { S.loc = 'off'; S.gs = 'off'; }
            S.locPhase = null;
            if (S.tgt.course !== null) S.tgt.course = Math.round(S.tgt.course) % 360;
            S.gsVs = null; S.lnav = false; S.op = null; S.apVals = null; S.apCmd = null; S.apThrottle = false; resetGuard(); S.vsCmd = null; S.lastVsT = 0;
            const r = origTurnOff.apply(this, arguments);
            restoreValues(); writeDisplays();
            log('AP disengaged' + (close ? ' near the runway: LOC/GS off' : ''));
            return r;
        };

        origUpdate = A.update;
        A.update = MY.update = function () {
            restoreValues();
            if (this.on) { try { navFrame(this); } catch (e) { console.warn('[AP+] nav frame', e); } }
            let savedAlt = null;
            if (this.on && S.gs === 'captured' && S.gsVs !== null) {
                savedAlt = this.values.altitude;
                const cur = altFt() || 0;
                // selected altitude itself is untouched; too low -> hold the present altitude, never climb
                this.values.altitude = S.gsVs > -50 ? Math.round(cur) : cur - 5000;
            }
            // V/S = rate only; it always points towards the selected altitude (GeoFS itself never flips the sign back)
            let savedVs;
            const gsActive = S.gs === 'captured' && S.gsVs !== null;
            if (gsActive) { S.vsCmd = S.gsVs; S.lastVsT = performance.now(); }   // hand back smoothly after GS
            if (this.on && !gsActive && S.tgt.altitude !== null && concordeMode()) {   // classic mode: V/S untouched, as v1.3.1
                const cur = altFt();
                if (cur !== null) {
                    const diff = S.tgt.altitude - cur;
                    const d = this.definition || {};
                    let mag = Math.abs(Number(this.values.verticalSpeed));
                    if (!isFinite(mag) || mag < 100) mag = Math.abs(diff >= 0 ? (d.baseClimbrate || 700) : (d.baseDescentrate || 1000));
                    {
                    // smooth capture: never more V/S than can be removed at ALT_CAPTURE_DECEL before reaching the altitude
                    const profile = Math.sqrt(120 * CFG.ALT_CAPTURE_DECEL * Math.abs(diff));
                    const want = Math.sign(diff) * Math.min(mag, profile, CFG.ALT_HOLD_GAIN * Math.abs(diff));   // linear in the last ~100 ft: no chatter at the altitude
                    const now = performance.now();
                    const dts = S.lastVsT ? clamp((now - S.lastVsT) / 1000, 0, 0.25) : 0;
                    S.lastVsT = now;
                    if (S.vsCmd === null || dts === 0) S.vsCmd = Number(av().verticalSpeed) || want;
                    S.vsCmd += clamp(want - S.vsCmd, -CFG.VS_SLEW * dts, CFG.VS_SLEW * dts);
                    savedVs = this.values.verticalSpeed;
                    this.values.verticalSpeed = S.vsCmd;
                    }
                }
            }
            const c = window.controls;
            // snapshot every numeric control value, so we know exactly what the AP itself writes this frame
            const before = {};
            if (c) for (const k in c) if (typeof c[k] === 'number') before[k] = c[k];
            let r;
            if (this.on) applySensitivity(this);
            try { r = origUpdate.apply(this, arguments); }
            finally {
                if (savedAlt !== null) this.values.altitude = savedAlt;
                if (savedVs !== undefined) this.values.verticalSpeed = savedVs;
            }
            if (c && this.on) {
                // every control the AP has written during this engagement keeps the AP's latest value
                // (also in frames where its output did not change), controls it never writes stay untouched
                if (!S.apVals) { S.apVals = {}; S.apValsT = performance.now(); S.apKeysLogged = false; }
                for (const k in before) if (typeof c[k] === 'number' && c[k] !== before[k]) S.apVals[k] = c[k];
                if (!S.apKeysLogged && performance.now() - S.apValsT > 2000) {
                    S.apKeysLogged = true; log('AP writes:', Object.keys(S.apVals).join(', ') || '(nothing)');
                }
                const cmd = Object.assign({}, S.apVals);
                if (!CFG.OVERRIDE_THROTTLE) delete cmd.throttle;
                smoothOutputs(c, before, cmd);
                engageBlend(this, c, before, cmd);
                try { ownPitch(this, c, before, cmd); } catch (e) { console.warn('[AP+] own pitch', e); }
                S.apCmd = Object.keys(cmd).length ? cmd : null;     // only what the AP actually changed
                if (CFG.OVERRIDE_THROTTLE && typeof cmd.throttle === 'number') S.apThr = cmd.throttle;
                spreadThrottle(c);
                try { pitchGuard(this); } catch (e) { console.warn('[AP+] pitch guard', e); }
            } else { S.apCmd = null; S.apThr = null; S.apVals = null; S.sm = null; }
            return r;
        };

        if (A.UI && typeof A.UI.update === 'function') {
            const o = A.UI.update;
            A.UI.update = function () { restoreValues(); const r = o.apply(this, arguments); writeDisplays(); return r; };
        }
        log('autopilot hooked, frozen values', Object.assign({}, S.tgt));
    }

    // bumpless engage: first the AP's elevator is matched to your input (its integral absorbs the difference,
    // so trim/stick held at the moment of engaging is kept), then the output blends to the AP's own command
    function engageBlend(A, c, before, cmd) {
        // bumpless engage (v1.4.9): GeoFS scales the elevator PID output before it reaches controls.pitch
        // (factor ~0.5 at 200 kts, speed dependent). Priming the PID with the raw stick value was therefore wrong
        // (nose drop or pull). For the first frames we measure that factor and set the PID integral so that its
        // output equals the stick you were holding; from there the AP continues smoothly on its own.
        if (!CFG.ENGAGE_BLEND || !S.engageT) return;
        if (S.engageFrames >= CFG.ENGAGE_MATCH_FRAMES) { S.engageT = null; return; }
        S.engageFrames++;
        const ep = A.PIDs && A.PIDs.elevatorPitch;
        const k = typeof cmd.pitch === 'number' ? 'pitch' : (typeof cmd.rawPitch === 'number' ? 'rawPitch' : null);
        if (!k || !ep || !('_integral' in ep) || typeof before[k] !== 'number') return;
        if (S.userElev[k] === undefined) S.userElev[k] = before[k];
        const u = S.userElev[k];
        const out = clamp(ep._kp * (ep._setPoint - ep._previousInput) + ep._integral, -1, 1);
        let f = Math.abs(out) > 0.02 ? cmd[k] / out : (S.engageF || 1);
        if (!isFinite(f) || f < 0.1 || f > 5) f = S.engageF || 1;
        S.engageF = f;
        ep._integral = clamp(ep._integral + (u - cmd[k]) / f, -1, 1);
        if (S.engageFrames === 1) log('engage: stick', u.toFixed(3), 'AP wanted', cmd[k].toFixed(3), 'output scale', f.toFixed(2));
        cmd[k] = u; c[k] = u;
        if (k === 'pitch' && typeof cmd.rawPitch === 'number') { cmd.rawPitch = before.rawPitch; c.rawPitch = before.rawPitch; }
    }

    // ---- pitch response model: q (deg/s) ~= Kq * (elevator - trim), Kq scales with (speed/220)^2
    // measured on the Concorde: 0.22 elevator above trim -> ~11 deg/s, so Kq ~ 50 at 220 kts
    const PM = { mult: 1, kq: null, name: '', lastT: 0, lastTh: null, q: 0, eLag: 0, n: 0, mx: 0, my: 0, vx: 0, cxy: 0, vy: 0, saveT: 0 };
    function concordeMode() { const n = acName(); return (CFG.CONCORDE_MODE || []).some(k => k === '*' || n.includes(k)); }
    function acName() { try { return String(geofs.aircraft.instance.aircraftRecord.name || '').toLowerCase(); } catch (e) { return ''; } }
    function kqDefault(name) {
        for (const k in CFG.OP_KQ) if (k !== 'default' && name.includes(k)) return CFG.OP_KQ[k];
        return CFG.OP_KQ.default;
    }
    function kqStore() { try { return JSON.parse(localStorage.getItem('applus_kq2') || '{}'); } catch (e) { return {}; } }
    function kq220() {
        const name = acName();
        if (PM.name !== name) {
            PM.name = name; PM.n = 0; PM.mx = PM.my = PM.vx = PM.cxy = PM.vy = 0;
            const st = kqStore()[name];
            const kd = kqDefault(name);
            PM.kq = isFinite(st) && st > 0 ? clamp(st, kd * 0.5, kd * 3) : kd;
            PM.mult = 1;
        }
        return PM.kq;
    }
    function airborne(v) {
        const h = Number(v.haglFeet);
        if (isFinite(h)) return h > 30;
        try { return !geofs.aircraft.instance.groundContact; } catch (e) { return true; }
    }
    // runs every animation frame: measures pitch rate and (when there is enough elevator movement) learns Kq
    function pitchMonitor() {
        try {
            const v = av(), c = window.controls, now = performance.now();
            const th = -Number(v.atilt);
            if (!c || !isFinite(th)) return;
            const dt = PM.lastT ? (now - PM.lastT) / 1000 : 0;
            PM.lastT = now;
            if (dt <= 0 || dt > 0.25 || PM.lastTh === null) { PM.lastTh = th; PM.eLag = Number(c.pitch) || 0; return; }
            PM.q += ((th - PM.lastTh) / dt - PM.q) * Math.min(1, dt / CFG.OP_Q_FILTER_S);
            PM.lastTh = th;
            const e = Number(c.pitch) || 0;
            PM.eLag += (e - PM.eLag) * Math.min(1, dt / 0.3);              // the aircraft answers the elevator with a short lag
            const kias = Number(v.kias) || 0;
            const K0 = kq220();
            const Aon = ap() && ap().on;
            if (!CFG.OP_LEARN || Aon || kias < 100 || !airborne(v)) return;   // learn from your own flying only (closed loop would bias it)
            // exponentially weighted regression of q on elevator (normalised for speed), ~4 s memory
            const x = PM.eLag * Math.pow(kias / 220, 2), y = PM.q, w = Math.min(1, dt / 4);
            PM.n = Math.min(PM.n + dt, 10);
            const dx = x - PM.mx, dy = y - PM.my;
            PM.mx += w * dx; PM.my += w * dy;
            PM.vx = (1 - w) * (PM.vx + w * dx * dx);
            PM.vy = (1 - w) * (PM.vy + w * dy * dy);
            PM.cxy = (1 - w) * (PM.cxy + w * dx * dy);
            if (PM.n < 3 || PM.vx < 0.03 * 0.03) return;                    // not enough elevator movement to learn from
            const r = PM.cxy / Math.sqrt(PM.vx * PM.vy || 1e-9);
            const k = PM.cxy / PM.vx;
            if (r < 0.8 || !(k > 10 && k < 400)) return;
            const kd = kqDefault(PM.name);
            PM.kq = clamp(PM.kq + (k - PM.kq) * Math.min(1, dt / 3), kd * 0.5, kd * 3);
            if (now - PM.saveT > 5000) {
                PM.saveT = now;
                const st = kqStore(); st[PM.name] = Math.round(PM.kq * 10) / 10;
                try { localStorage.setItem('applus_kq2', JSON.stringify(st)); } catch (e) {}
            }
        } catch (e) {}
    }
    (function loop() { pitchMonitor(); requestAnimationFrame(loop); })();

    function setElev(c, cmd, x) {
        cmd.pitch = x; c.pitch = x;
        if (typeof c.rawPitch === 'number') { cmd.rawPitch = x; c.rawPitch = x; }
    }
    // own V/S -> pitch -> pitch-rate -> elevator control.
    // Engage: the trim is estimated from your elevator and pitch rate, your pitch rate is taken over and faded out
    // (so a pull at the moment of engaging stops smoothly instead of continuing), then V/S is followed.
    function ownPitch(A, c, before, cmd) {
        if (!CFG.OWN_PITCH) return;
        const nm = acName();
        if (!concordeMode()) { S.op = null; return; }
        void nm;
        const v = av(), now = performance.now();
        const th = -Number(v.atilt), vs = Number(v.verticalSpeed) || 0;
        if (!isFinite(th)) return;
        const kias = Math.max(80, Number(v.kias) || 0);
        const tas = Math.max(80, Number(v.ktas) || kias);
        const Kq = Math.max(5, kq220() * PM.mult * Math.pow(kias / 220, 2));   // mult > 1 = gains lowered by the guard
        const q = PM.q;
        const d = A.definition || {};
        const maxP = Number(d.maxPitchAngle) > 0 ? Number(d.maxPitchAngle) : 20;
        const minP = Number(d.minPitchAngle) < 0 ? Number(d.minPitchAngle) : -15;
        let P = S.op;
        if (!P) {
            const u = clamp(typeof before.pitch === 'number' ? before.pitch : (Number(c.pitch) || 0), -1, 1);
            const trim = clamp(PM.eLag - q / Kq, -1, 1);
            S.op = P = { eLP: u, hp2: 0, eT: trim, thI: clamp(th + q * CFG.OP_ARREST_S, minP, maxP), thT: th, e: u, lastT: now, t0: now, q0: q };
            log('own pitch control: stick', u.toFixed(3) + ', pitch', th.toFixed(1) + '°, pitch rate', q.toFixed(1) + '°/s, V/S',
                Math.round(vs) + ', trim estimate', trim.toFixed(3) + ', Kq', Math.round(kq220()));
            setElev(c, cmd, u);
            return;
        }
        const dt = clamp((now - P.lastT) / 1000, 0, 0.1);
        P.lastT = now;
        if (dt <= 0) { setElev(c, cmd, P.e); return; }
        const age = (now - P.t0) / 1000;

        // outer loop: V/S -> pitch target (geometric gain from the true airspeed) + learnt pitch for that V/S
        const vsT = S.vsCmd !== null && S.vsCmd !== undefined ? S.vsCmd : (Number(A.targetVerticalSpeed) || 0);
        const kv = CFG.OP_VS_GAIN * 57.3 / (tas * 101.3);
        const err = vsT - vs;
        const raw = P.thI + kv * err;
        if (!((raw >= maxP && err > 0) || (raw <= minP && err < 0))) P.thI = clamp(P.thI + kv * CFG.OP_VS_I * err * dt, minP, maxP);
        const thWant = clamp(P.thI + kv * err, minP, maxP);
        P.thT += clamp(thWant - P.thT, -CFG.OP_PITCH_RATE * dt, CFG.OP_PITCH_RATE * dt);

        // middle loop: pitch error -> pitch rate; your own pitch rate at engage fades out over OP_ARREST_S
        let qCmd = clamp((P.thT - th) / CFG.OP_T_PITCH, -CFG.OP_Q_MAX, CFG.OP_Q_MAX);
        const wq = Math.exp(-age / CFG.OP_ARREST_S);
        qCmd = wq * P.q0 + (1 - wq) * qCmd;

        // inner loop: pitch rate -> elevator through the response model, trim learnt by the integral
        const qe = qCmd - q;
        let e = P.eT + (qCmd + CFG.OP_KPQ * qe) / Kq;
        P.eT = clamp(P.eT + CFG.OP_KIQ * qe / Kq * dt, -1, 1);
        e = clamp(e, -1, 1);
        P.e += clamp(e - P.e, -CFG.OP_RATE * dt, CFG.OP_RATE * dt);

        // oscillation guard: if the elevator keeps see-sawing, the aircraft answers stronger than the model says
        // -> lower the gains until it is calm (kept for this aircraft for the rest of the session)
        P.eLP += (P.e - P.eLP) * Math.min(1, dt / 1.5);
        const hp = P.e - P.eLP;
        P.hp2 += (hp * hp - P.hp2) * Math.min(1, dt / 2);
        if (age > CFG.OP_GUARD_AFTER_S) {
            const rms = Math.sqrt(P.hp2);
            if (rms > CFG.OP_GUARD_ON) {
                const m0 = PM.mult;
                PM.mult = Math.min(CFG.OP_GUARD_MAX, PM.mult * (1 + CFG.OP_GUARD_RATE * dt));
                if (!P.guardLogged && PM.mult > 1.2 && m0 <= 1.2) { P.guardLogged = true; log('pitch: elevator see-saw detected, gains lowered'); }
            } else if (rms < CFG.OP_GUARD_OFF) PM.mult = Math.max(1, PM.mult * (1 - 0.01 * dt));
        }
        setElev(c, cmd, P.e);
    }

    // ---- sensitivity: GeoFS's PID gains per aircraft x SENS (originals kept), with an automatic back-off when it see-saws
    const PID_BASE = {};
    const SENS_LIVE = {};    // per aircraft: { p, r } current multipliers (lowered by the guard if needed)
    function applySensitivity(A) {
        const P = A.PIDs; if (!P) return;
        const name = acName() || '?';
        if (!PID_BASE[name]) {
            PID_BASE[name] = {};
            ['pitchAngle', 'elevatorPitch', 'bankAngle', 'aileronsRoll'].forEach(k => {
                if (P[k] && isFinite(P[k]._kp)) PID_BASE[name][k] = { kp: P[k]._kp, ki: P[k]._ki };
            });
            SENS_LIVE[name] = { p: CFG.SENS_PITCH, r: CFG.SENS_ROLL };
            log('sensitivity: pitch x' + CFG.SENS_PITCH + ', roll x' + CFG.SENS_ROLL + ' for', name);
        }
        const B = PID_BASE[name], L = SENS_LIVE[name];
        const fp = concordeMode() ? 1 : L.p;      // the Concorde's pitch is flown by the own controller
        const set = (k, f) => { if (P[k] && B[k]) { P[k]._kp = B[k].kp * f; P[k]._ki = B[k].ki * f; } };
        set('pitchAngle', fp); set('elevatorPitch', 1); set('bankAngle', L.r); set('aileronsRoll', 1 + (CFG.SENS_AILERON - 1) * (L.r - 1) / Math.max(1e-6, CFG.SENS_ROLL - 1));
    }
    // outputs never jump: elevator / ailerons follow the AP at a limited rate, starting from your stick at engage
    function smoothOutputs(c, before, cmd) {
        const now = performance.now();
        if (!S.sm) {
            S.sm = { t: now, ax: {} };
            ['pitch', 'roll'].forEach(k => { S.sm.ax[k] = { v: typeof before[k] === 'number' ? before[k] : (Number(c[k]) || 0), lp: null, hp2: 0 }; });
        }
        const dt = clamp((now - S.sm.t) / 1000, 0, 0.1);
        S.sm.t = now;
        const name = acName() || '?', L = SENS_LIVE[name];
        [['pitch', CFG.SMOOTH_PITCH_RATE, 'p'], ['roll', CFG.SMOOTH_ROLL_RATE, 'r']].forEach(([k, rate, axis]) => {
            if (k === 'pitch' && (concordeMode() || !(rate > 0))) return;   // own pitch control has its own smoothing
            let want = cmd[k];
            if (k === 'pitch' && typeof want !== 'number' && typeof cmd.rawPitch === 'number') want = cmd.rawPitch;
            if (typeof want !== 'number') return;
            const a = S.sm.ax[k];
            a.v += clamp(want - a.v, -rate * dt, rate * dt);
            cmd[k] = a.v; c[k] = a.v;
            if (k === 'pitch' && typeof cmd.rawPitch === 'number') { cmd.rawPitch = a.v; c.rawPitch = a.v; }
            // see-saw guard: lower this axis's gain while the output keeps going up and down
            if (a.lp === null) a.lp = a.v;
            a.lp += (a.v - a.lp) * Math.min(1, dt / 1.5);
            const hp = a.v - a.lp; a.hp2 += (hp * hp - a.hp2) * Math.min(1, dt / 2);
            if (!L || dt <= 0 || now - (S.engageT || 0) < 10000) return;
            const rms = Math.sqrt(a.hp2), base = axis === 'p' ? CFG.SENS_PITCH : CFG.SENS_ROLL;
            if (rms > CFG.SENS_GUARD_ON) {
                const old = L[axis];
                L[axis] = Math.max(0.6, L[axis] * (1 - 0.3 * dt));
                if (old >= base * 0.95 && L[axis] < base * 0.95) log('sensitivity: ' + k + ' see-saw, gain lowered');
            } else if (rms < CFG.SENS_GUARD_OFF) L[axis] = Math.min(base, L[axis] * (1 + 0.01 * dt));
        });
    }

    // split throttle levers (left/right or per engine) would otherwise keep the user's lever position
    // on some engines while the AP only drives controls.throttle -> V/S depended on the levers
    const THR_KEYS = ['throttleLeft', 'throttleRight', 'throttle1', 'throttle2', 'throttle3', 'throttle4'];
    const REV_KEYS = ['reverse', 'reverseLeft', 'reverseRight', 'reverse1', 'reverse2', 'reverse3', 'reverse4'];
    function spreadThrottle(c) {
        const A = ap();
        if (!c || !CFG.OVERRIDE_THROTTLE || !A || !A.on || typeof S.apThr !== 'number') return;
        c.throttle = S.apThr;
        THR_KEYS.forEach(k => { if (typeof c[k] === 'number') c[k] = S.apThr; });
        REV_KEYS.forEach(k => { if (typeof c[k] === 'number') c[k] = 0; });
    }

    function hookControls() {
        const c = window.controls;
        if (!c || c.__applusHooked) return;
        const restore = () => {
            const A = ap();
            if (!CFG.JOYSTICK_OVERRIDE || !A || !A.on || !S.apCmd) return;
            for (const k in S.apCmd) c[k] = S.apCmd[k];
            // the AP writes rawPitch only on some aircraft; the elevator itself follows rawPitch
            if (typeof S.apCmd.rawPitch === 'number' && typeof S.apCmd.pitch !== 'number' && typeof c.pitch === 'number') c.pitch = S.apCmd.rawPitch;
            spreadThrottle(c);
        };
        let any = false;
        ['update', 'updateJoystick', 'applyJoystick'].forEach(fn => {
            if (typeof c[fn] === 'function') {
                const o = c[fn];
                c[fn] = function () { const r = o.apply(this, arguments); restore(); return r; };
                any = true;
            }
        });
        if (c.joystick && typeof c.joystick.update === 'function') {
            const o = c.joystick.update;
            c.joystick.update = function () { const r = o.apply(this, arguments); restore(); return r; };
            any = true;
        }
        if (any) { c.__applusHooked = true; log('controls hooked (joystick override while AP on)'); }
    }

    // ------------------------------------------------------------ main tick (UI + slow logic)
    function tick() {
        hookAutopilot(); hookControls(); buildButtons(); bindInputs(); extendILSRange(); layoutButtons();
        const A = ap(); if (!A || !S.hooked) return;

        restoreValues();
        writeDisplays();
        updateTrack(ownPos());

        const g = ilsGeometry();
        S.lastG = g;
        if (!g) {
            S.loc = 'off'; S.gs = 'off'; S.gsVs = null;
            setBtn('loc', 'off', '', 'Localizer: tune an ILS first');
            setBtn('gs', 'off', '', 'Glideslope: tune an ILS first');
        } else {
            const dTxt = g.d.toFixed(1) + ' NM';
            const pathTxt = g.err === null ? '' : (g.err >= 0 ? Math.round(g.err) + ' ft below path' : Math.round(-g.err) + ' ft above path');
            const locTip = `Localizer ${g.ils.nav.id || ''} · ${dTxt} · ${Math.abs(g.xtk * NM_TO_M).toFixed(0)} m ${g.xtk >= 0 ? 'left' : 'right'}`;
            const gsTip = `Glideslope · ${dTxt} · ${pathTxt}`;
            checkCaptures(g);
            if (S.loc === 'off') setBtn('loc', 'off', locCanCapture(g) ? 'AVAIL' : '', locTip);
            else setBtn('loc', S.loc, S.loc === 'armed' ? 'ARM' : 'CAP', locTip);
            if (S.gs === 'off') setBtn('gs', 'off', gsCanCapture(g) ? 'AVAIL' : '', gsTip);
            else setBtn('gs', S.gs, S.gs === 'armed' ? 'ARM' : 'CAP', gsTip);
        }

        if (!A.on) S.lastFrame = 0;
        try {   // remember when the selected flight-plan waypoint last changed
            const sel = fpSelectedIndex(fpWaypoints());
            const ref = sel ? sel.ref : null;
            if (ref !== S.fpSelLast) { S.fpSelLast = ref; S.fpSelT = performance.now(); }
        } catch (e) {}
        if (A.on && S.lnav) lnavStep();

        const navB = $('.geofs-autopilot-NAV'), hdgB = $('.geofs-autopilot-HDG');
        if (navB && hdgB) {
            if (navB.classList.contains('green-pad') !== S.lnav) navB.classList.toggle('green-pad', S.lnav);
            if (hdgB.classList.contains('green-pad') === S.lnav) hdgB.classList.toggle('green-pad', !S.lnav);
        }
    }

    setInterval(() => { try { tick(); } catch (e) { console.warn('[AP+]', e); } }, CFG.TICK_MS);
    log('loaded v1.6.5');
})();
