<script>
  import { onMount } from 'svelte';
  import AngleGauge from './lib/AngleGauge.svelte';
  import Firmware from './sections/Firmware.svelte';
  import Debug from './sections/Debug.svelte';
  import Storage from './sections/Storage.svelte';
  import Expert from './sections/Expert.svelte';
  import { calibrationStream, connectAngles, fetchExpertStatus, postJson, requestJson } from './lib/api.js';

  const VERSION = __ROTATOR_VERSION__;
  // The first four are always available. Update, Debug and Storage only show
  // up once expert mode is unlocked - they are the more powerful half of this
  // page, in the same way /log and /fs/* are the more powerful half of the API.
  const ALL_TABS = ['Position', 'Network', 'WLAN', 'Calibration', 'Update', 'Debug', 'Storage'];
  const OPEN_TABS = 4;

  // Reached by address only - #expert has no chip of its own in the tab row,
  // on purpose. There is nothing there for someone who has not gone looking.
  const HASH_SCREENS = { '#expert': 'expert' };
  const HASH_TITLES = { expert: 'Expert mode' };

  let active = $state('Position');
  // Until the rotator says otherwise it is locked. That is the safe way
  // round: an older firmware with no /expert at all answers 404, and the
  // gated tabs stay away rather than being offered against endpoints that
  // refuse them.
  let expert = $state({ enrolled: false, unlocked: false, grace: 0, lockedOut: false });
  let menuOpen = $state(false);
  let navEl = $state(null);
  let notice = $state('');
  let error = $state(false);
  let angle = $state(0);
  let mechAngle = $state(0);
  let direction = $state('cw');
  let network = $state({ ip: '', netmask: '', mac: '' });
  let aps = $state([]);
  let ssid = $state('');
  let password = $state('');
  let wifiStatus = $state('');
  let currentWifi = $state(null);
  let wifiStatusError = $state('');
  let hostname = $state('');
  let hostnameBusy = $state(false);
  let scanBusy = $state(false);
  let zeroProgress = $state(null);
  let angleProgress = $state(null);

  function announce(message, failed = false) {
    notice = message;
    error = failed;
    window.clearTimeout(announce.timer);
    announce.timer = window.setTimeout(() => notice = '', 5000);
  }

  const tabIds = $derived(expert.unlocked ? ALL_TABS : ALL_TABS.slice(0, OPEN_TABS));
  // The chip label most of the time; the hash screen's own title otherwise -
  // "expert" itself would be a strange word to put in a burger menu.
  const activeLabel = $derived(tabIds.includes(active) ? active : HASH_TITLES[active]);

  function select(tab) {
    active = tab;
    menuOpen = false;
    if (tab === 'WLAN' && !aps.length) scan();
    // Leave the hash behind, or reloading the page would land back on it.
    if (location.hash) history.replaceState(null, '', location.pathname);
  }

  /**
   * The expert screen has no chip in the tab row - #expert in the address is
   * the way in. Re-fetched on the way in rather than reusing what is already
   * held: the reset window is counting down, and a value read when the page
   * loaded would be stale by minutes.
   */
  async function syncFromHash() {
    const screen = HASH_SCREENS[location.hash];
    if (!screen) return;
    active = screen;
    try { expert = await fetchExpertStatus(); }
    catch { /* keep what we have */ }
  }

  /**
   * Takes the state the rotator answered with. Locking while a gated tab is
   * open would leave it on screen with every request behind it refused, so
   * step back to the front.
   */
  function applyExpert(next) {
    expert = next;
    if (!next.unlocked && ALL_TABS.indexOf(active) >= OPEN_TABS) active = 'Position';
  }

  function onWindowKeydown(event) {
    if (event.key === 'Escape') menuOpen = false;
  }

  function onWindowPointerdown(event) {
    if (menuOpen && navEl && !navEl.contains(event.target)) menuOpen = false;
  }

  async function loadNetwork() {
    try { network = await requestJson('/api/network/config'); }
    catch (reason) { announce(`Could not load network settings: ${reason.message}`, true); }
  }

  async function saveNetwork(path, body, label) {
    try { await postJson(path, body); announce(`${label} saved`); }
    catch (reason) { announce(`${label} failed: ${reason.message}`, true); }
  }

  function strongestPerName(found) {
    const best = new Map();
    for (const ap of found) {
      if (!ap.ssid) continue;
      const seen = best.get(ap.ssid);
      if (!seen || ap.rssi > seen.rssi) best.set(ap.ssid, ap);
    }
    return [...best.values()].sort((a, b) => b.rssi - a.rssi);
  }

  async function scan() {
    scanBusy = true;
    try {
      const result = await requestJson('/api/wifi/scan');
      aps = strongestPerName(result.aps || []);
    } catch (reason) { announce(`WLAN scan failed: ${reason.message}`, true); }
    finally { scanBusy = false; }
  }

  async function loadWifiStatus() {
    try {
      currentWifi = await requestJson('/api/wifi/status');
      hostname = currentWifi.hostname || '';
      wifiStatusError = '';
    } catch (reason) { wifiStatusError = reason.message; }
  }

  async function saveHostname(event) {
    event.preventDefault();
    if (!hostname.trim() || hostnameBusy) return;
    hostnameBusy = true;
    try {
      const result = await postJson('/api/wifi/hostname', { hostname });
      hostname = result.hostname;
      announce(`Device name saved as ${hostname}.local · restarting`);
    } catch (reason) { announce(`Device name failed: ${reason.message}`, true); }
    finally { hostnameBusy = false; }
  }

  function quality(rssi) {
    return rssi >= -55 ? 'Excellent' : rssi >= -67 ? 'Good' : rssi >= -75 ? 'Fair' : 'Weak';
  }

  async function connectWifi(event) {
    event.preventDefault();
    if (!ssid.trim()) return announce('Enter or select an SSID', true);
    try {
      const result = await postJson('/api/wifi/connect', { ssid, password });
      password = '';
      wifiStatus = result.connected ? `Connected · ${result.ip}` : 'Connection requested';
      announce(wifiStatus);
    } catch (reason) { announce(`Connection failed: ${reason.message}`, true); }
  }

  function bars(rssi) {
    if (rssi >= -55) return 4;
    if (rssi >= -67) return 3;
    if (rssi >= -75) return 2;
    return 1;
  }

  function calibrate(path, completeEvent, kind) {
    if (kind === 'zero') zeroProgress = 0; else angleProgress = 0;
    calibrationStream(path, completeEvent, {
      progress(value) { if (kind === 'zero') zeroProgress = value; else angleProgress = value; },
      complete(value) {
        if (kind === 'zero') zeroProgress = null; else angleProgress = null;
        announce(kind === 'zero' ? `Mechanical zero: ${value}` : 'Angle sensor calibrated');
      },
      error() {
        if (kind === 'zero') zeroProgress = null; else angleProgress = null;
        announce('Calibration connection closed', true);
      }
    });
  }

  onMount(() => {
    loadNetwork();
    loadWifiStatus();
    fetchExpertStatus().then((next) => (expert = next)).catch(() => {
      // Not fatal, and not a reason to unlock anything: the default above
      // already says locked.
    }).finally(syncFromHash);
    const socket = connectAngles((data) => {
      angle = Number(data.angle) || 0;
      mechAngle = Number(data.mechAngle) || 0;
      direction = data.direction === 'ccw' ? 'ccw' : 'cw';
    });
    socket.addEventListener('error', () => announce('Live angle connection unavailable', true));
    return () => socket.close();
  });
</script>

<svelte:window onkeydown={onWindowKeydown} onpointerdown={onWindowPointerdown} onhashchange={syncFromHash} />
<svelte:head><title>AG2998 Camera Rotator</title><meta name="theme-color" content="#f4f5f7" /></svelte:head>

<div class="app">
  <header>
    <h1>AG2998 Camera Rotator</h1>
    <nav bind:this={navEl} class:open={menuOpen} aria-label="Configuration sections">
      <button type="button" class="menu-toggle" aria-expanded={menuOpen} aria-controls="tabs" onclick={() => menuOpen = !menuOpen}>
        <span class="burger" aria-hidden="true"></span><span>{activeLabel}</span>
      </button>
      <div class="tabs" id="tabs">
        {#each tabIds as tab}
          <button type="button" aria-current={active === tab ? 'page' : undefined} onclick={() => select(tab)}>{tab}</button>
        {/each}
      </div>
    </nav>
  </header>

  {#if active === 'Position'}
    <section class="card position-card">
      <h2>Live position</h2>
      <div class="position">
        <div class="gauge"><AngleGauge {angle} {direction} /></div>
        <div class="telemetry">
          <div class="primary-angle"><strong>{angle.toFixed(2)}</strong><span>°</span></div>
          <div class="direction"><span class:ccw={direction === 'ccw'}>↻</span>{direction === 'cw' ? 'clockwise' : 'counter-clockwise'}</div>
          <div class="field"><span class="key">Mechanical angle</span><span>{mechAngle.toFixed(2)}°</span></div>
          <div class="field"><span class="key">Interface</span><span>Alpaca Rotator</span></div>
        </div>
      </div>
    </section>
  {:else if active === 'Network'}
    <section class="card">
      <h2>USB RNDIS network</h2>
      <p class="hint">Settings for the rotator's direct USB network interface.</p>
      <div class="network-fields">
        <div><label for="ip">IP address</label><input id="ip" type="text" bind:value={network.ip} inputmode="decimal" /><button class="primary" onclick={() => saveNetwork('/api/network/set/ip', { ip: network.ip }, 'IP address')}>Save</button></div>
        <div><label for="netmask">Netmask</label><input id="netmask" type="text" bind:value={network.netmask} inputmode="decimal" /><button class="primary" onclick={() => saveNetwork('/api/network/set/netmask', { netmask: network.netmask }, 'Netmask')}>Save</button></div>
        <div><label for="mac">MAC address</label><input id="mac" type="text" bind:value={network.mac} /><button class="primary" onclick={() => saveNetwork('/api/network/set/mac', { mac: network.mac }, 'MAC address')}>Save</button></div>
      </div>
    </section>
  {:else if active === 'WLAN'}
    <section class="card">
      <h2>Connection</h2>
      {#if wifiStatusError}<p class="banner">Status unavailable · {wifiStatusError}</p>
      {:else if currentWifi}
        <div class="field"><span class="key">Network</span><span>{currentWifi.ssid || '—'}</span></div>
        <div class="field"><span class="key">Address</span><span>{currentWifi.ip || '—'}</span></div>
        <div class="field"><span class="key">Signal</span><span>{quality(currentWifi.rssi)} ({currentWifi.rssi} dBm)</span></div>
        <div class="field"><span class="key">Hostname</span><span>{currentWifi.hostname}.local</span></div>
        <div class="field"><span class="key">MAC</span><span>{currentWifi.mac}</span></div>
      {:else}<p class="hint">Loading…</p>{/if}
    </section>

    <section class="card">
      <h2>Device name</h2>
      <form onsubmit={saveHostname}>
        <div class="field"><label for="hostname">Name</label><input id="hostname" type="text" maxlength="32" bind:value={hostname} autocomplete="off" spellcheck="false" /></div>
        <button type="submit" disabled={!hostname.trim() || hostnameBusy}>{hostnameBusy ? 'Restarting…' : 'Save and restart'}</button>
      </form>
      <p class="hint">Enter the name without <code>.local</code>. The rotator will be reachable as <code>{hostname || 'AG2998-Rotator'}.local</code>.</p>
    </section>

    <section class="card">
      <h2>Available networks</h2>
      {#if scanBusy}
        <p class="hint">Scanning…</p>
      {:else if !aps.length}
        <p class="hint">No networks found.</p>
      {:else}
        <ul class="netlist">
          {#each aps as ap (ap.ssid)}
            <li>
              <label class="choice">
                <input type="radio" name="ssid" value={ap.ssid} bind:group={ssid} />
                <span class="netname">{ap.ssid}</span>
                <span class="bars" title="{ap.rssi} dBm" aria-label="Signal strength">
                  {#each [1, 2, 3, 4] as bar}<i class:on={bar <= bars(ap.rssi)}></i>{/each}
                </span>
                {#if ap.authmode}<span class="lock" title="Encrypted">🔒</span>{/if}
              </label>
            </li>
          {/each}
        </ul>
      {/if}
      <button type="button" class="secondary" onclick={scan} disabled={scanBusy}>{scanBusy ? 'Scanning…' : 'Scan again'}</button>
    </section>

    <section class="card">
      <h2>Switch network</h2>
      <form onsubmit={connectWifi}>
        <div class="field"><label for="ssid">Network</label><input id="ssid" type="text" bind:value={ssid} placeholder="SSID" /></div>
        <div class="field"><label for="pass">Password</label><input id="pass" type="password" bind:value={password} autocomplete="off" placeholder="Network password" /></div>
        <button type="submit" disabled={!ssid.trim()}>Connect</button>
      </form>
      {#if wifiStatus}<p class="hint success">{wifiStatus}</p>{/if}
    </section>
  {:else if active === 'Calibration'}
    <section class="card">
      <h2>Mechanical reference</h2>
      <p class="banner warning">These operations move the rotator. Make sure the mechanism can travel safely before starting.</p>
      <div class="calibration-actions">
        <div><h3>Find mechanical zero</h3><p class="hint">Searches the Hall-sensor index and measures its centre.</p><button class="primary" disabled={zeroProgress !== null} onclick={() => calibrate('/api/calibration/zero/stream', 'complete_zero', 'zero')}>Find zero</button>{#if zeroProgress !== null}<div class="progress"><div class="bar" style="width: {zeroProgress}%"></div></div><p class="hint">{zeroProgress}%</p>{/if}</div>
        <div><h3>Calibrate angle sensor</h3><p class="hint">Measures the AS5600 error over a full calibration sequence.</p><button class="primary" disabled={angleProgress !== null} onclick={() => calibrate('/api/calibration/angle/stream', 'complete_angle', 'angle')}>Calibrate sensor</button>{#if angleProgress !== null}<div class="progress"><div class="bar" style="width: {angleProgress}%"></div></div><p class="hint">{angleProgress}%</p>{/if}</div>
      </div>
    </section>
  {:else if active === 'Update'}
    <Firmware />
  {:else if active === 'Debug'}
    <Debug />
  {:else if active === 'Storage'}
    <Storage />
  {:else}
    <Expert {expert} onchange={applyExpert} />
  {/if}

  {#if notice}<p class:error class="banner notice" role="status">{notice}</p>{/if}
  <footer>Firmware {VERSION} · Astro Geeks Munich</footer>
</div>

<style>
  .position { display: grid; grid-template-columns: 1fr 1fr; align-items: center; gap: 2rem; }
  .gauge { display: grid; place-items: center; }
  .primary-angle { display: flex; align-items: flex-start; margin-bottom: .5rem; }
  .primary-angle strong { font: 300 clamp(3.5rem, 10vw, 6rem)/.9 ui-monospace, monospace; letter-spacing: -.08em; }
  .primary-angle span { color: var(--accent); font-size: 1.6rem; }
  .direction { color: var(--muted); margin-bottom: 1rem; text-transform: uppercase; letter-spacing: .08em; font-size: .75rem; }
  .direction span { color: var(--accent); display: inline-block; font-size: 1.2rem; margin-right: .4rem; }
  .direction span.ccw { transform: scaleX(-1); }
  .field > button { flex: 0 0 auto; }
  .network-fields { display: grid; grid-template-columns: 1fr; gap: .75rem; }
  .network-fields > div { display: grid; grid-template-columns: 8rem minmax(0, 1fr) auto; align-items: center; gap: .75rem; }
  .network-fields input { width: 100%; }
  code { color: var(--accent); }
  .calibration-actions { display: grid; grid-template-columns: 1fr 1fr; gap: 1.5rem; }
  .calibration-actions > div + div { border-left: 1px solid var(--border); padding-left: 1.5rem; }
  .calibration-actions h3 { margin: 0 0 .25rem; font-size: 1rem; }
  .progress { height: 10px; margin: .75rem 0 .25rem; border-radius: 999px; background: var(--border); overflow: hidden; }
  .bar { height: 100%; background: var(--accent); }
  .success { color: var(--accent); }
  .warning { margin-top: 0; }
  .notice { position: fixed; right: 1rem; bottom: 1rem; z-index: 30; max-width: 28rem; box-shadow: 0 8px 24px rgb(0 0 0 / .22); }
  footer { color: var(--muted); padding: .5rem .1rem 1rem; font-size: .75rem; }
  @media (max-width: 40rem) {
    .position, .calibration-actions { grid-template-columns: 1fr; }
    .calibration-actions > div + div { border-left: 0; border-top: 1px solid var(--border); padding: 1rem 0 0; }
    .field { align-items: stretch; flex-wrap: wrap; }
    .field > input { flex-basis: 100%; }
    .network-fields > div { grid-template-columns: 1fr; }
  }
</style>
