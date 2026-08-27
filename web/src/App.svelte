<script>
  import { onMount } from 'svelte';
  import AngleGauge from './lib/AngleGauge.svelte';
  import { calibrationStream, connectAngles, postJson, requestJson, uploadImage } from './lib/api.js';

  const VERSION = __ROTATOR_VERSION__;

  const tabs = ['Position', 'Network', 'WLAN', 'Calibration', 'Firmware'];
  let active = $state('Position');
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
  let scanBusy = $state(false);
  let zeroProgress = $state(null);
  let angleProgress = $state(null);
  let firmwareFile = $state(null);
  let filesystemFile = $state(null);
  let firmwareProgress = $state(null);
  let filesystemProgress = $state(null);

  function announce(message, failed = false) {
    notice = message;
    error = failed;
    window.clearTimeout(announce.timer);
    announce.timer = window.setTimeout(() => notice = '', 5000);
  }

  async function loadNetwork() {
    try { network = await requestJson('/api/network/config'); }
    catch (reason) { announce(`Could not load network settings: ${reason.message}`, true); }
  }

  async function saveNetwork(path, body, label) {
    try { await postJson(path, body); announce(`${label} saved`); }
    catch (reason) { announce(`${label} failed: ${reason.message}`, true); }
  }

  async function scan() {
    scanBusy = true;
    try {
      const result = await requestJson('/api/wifi/scan');
      aps = [...(result.aps || [])].sort((a, b) => b.rssi - a.rssi);
    } catch (reason) { announce(`WLAN scan failed: ${reason.message}`, true); }
    finally { scanBusy = false; }
  }

  async function connectWifi() {
    if (!ssid.trim()) return announce('Enter or select an SSID', true);
    try {
      const result = await postJson('/api/wifi/connect', { ssid, password });
      wifiStatus = result.connected ? `Connected · ${result.ip}` : 'Connection requested';
      announce(wifiStatus);
    } catch (reason) { announce(`Connection failed: ${reason.message}`, true); }
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

  async function upload(kind) {
    const firmware = kind === 'firmware';
    const file = firmware ? firmwareFile : filesystemFile;
    if (!file) return announce('Select a binary image first', true);
    if (firmware) firmwareProgress = 0; else filesystemProgress = 0;
    try {
      await uploadImage(firmware ? '/update_raw' : '/update_fs_raw', file,
        value => firmware ? firmwareProgress = value : filesystemProgress = value);
      announce(`${firmware ? 'Firmware' : 'Filesystem'} accepted; the rotator will restart`);
    } catch (reason) { announce(`Upload failed: ${reason.message}`, true); }
    finally {
      if (firmware) firmwareProgress = null; else filesystemProgress = null;
    }
  }

  onMount(() => {
    loadNetwork();
    const socket = connectAngles((data) => {
      angle = Number(data.angle) || 0;
      mechAngle = Number(data.mechAngle) || 0;
      direction = data.direction === 'ccw' ? 'ccw' : 'cw';
    });
    socket.addEventListener('error', () => announce('Live angle connection unavailable', true));
    return () => socket.close();
  });
</script>

<svelte:head><title>AG2998 Rotator</title><meta name="theme-color" content="#0b1118" /></svelte:head>

<header>
  <div class="brand"><span class="mark">AG</span><div><strong>AG2998 Rotator</strong><small>ASCOM Alpaca · ESP32-S3</small></div></div>
  <div class="live"><span></span> live telemetry</div>
</header>

<nav aria-label="Configuration sections">
  {#each tabs as tab}
    <button class:active={active === tab} onclick={() => { active = tab; if (tab === 'WLAN' && !aps.length) scan(); }}>{tab}</button>
  {/each}
</nav>

<main>
  {#if active === 'Position'}
    <section class="position">
      <div class="gauge"><AngleGauge {angle} {direction} /></div>
      <div class="telemetry">
        <p class="eyebrow">Live position</p>
        <div class="primary"><strong>{angle.toFixed(2)}</strong><span>°</span></div>
        <div class="direction"><span class:ccw={direction === 'ccw'}>↻</span>{direction === 'cw' ? 'clockwise' : 'counter-clockwise'}</div>
        <div class="metric"><span>Mechanical angle</span><strong>{mechAngle.toFixed(2)}°</strong></div>
        <div class="metric"><span>Interface</span><strong>Alpaca Rotator</strong></div>
      </div>
    </section>
  {:else if active === 'Network'}
    <section><p class="eyebrow">USB RNDIS</p><h1>Network identity</h1><p class="intro">Settings for the rotator's direct USB network interface.</p>
      <div class="form-grid">
        <label>IP address<input bind:value={network.ip} inputmode="decimal" /><button onclick={() => saveNetwork('/api/network/set/ip', { ip: network.ip }, 'IP address')}>Save</button></label>
        <label>Netmask<input bind:value={network.netmask} inputmode="decimal" /><button onclick={() => saveNetwork('/api/network/set/netmask', { netmask: network.netmask }, 'Netmask')}>Save</button></label>
        <label>MAC address<input bind:value={network.mac} /><button onclick={() => saveNetwork('/api/network/set/mac', { mac: network.mac }, 'MAC address')}>Save</button></label>
      </div>
    </section>
  {:else if active === 'WLAN'}
    <section><div class="section-head"><div><p class="eyebrow">Wireless</p><h1>Join a network</h1></div><button class="secondary" disabled={scanBusy} onclick={scan}>{scanBusy ? 'Scanning…' : 'Scan again'}</button></div>
      <div class="wifi-grid">
        <div class="ap-list">
          {#if !aps.length}<p class="empty">No access points loaded.</p>{/if}
          {#each aps as ap}
            <button onclick={() => ssid = ap.ssid}><span class="signal">{ap.rssi > -60 ? '●●●' : ap.rssi > -75 ? '●●○' : '●○○'}</span><span>{ap.ssid || '(hidden network)'}</span><small>{ap.authmode ? 'locked' : 'open'} · {ap.rssi} dBm</small></button>
          {/each}
        </div>
        <div class="connect-card"><label>SSID<input bind:value={ssid} autocomplete="off" /></label><label>Password<input type="password" bind:value={password} autocomplete="new-password" /></label><button onclick={connectWifi}>Connect</button>{#if wifiStatus}<p class="status">{wifiStatus}</p>{/if}</div>
      </div>
    </section>
  {:else if active === 'Calibration'}
    <section><p class="eyebrow">Mechanical reference</p><h1>Calibration</h1><p class="intro warning">These operations move the rotator. Make sure the mechanism can travel safely before starting.</p>
      <div class="cards">
        <article><span class="number">01</span><h2>Find mechanical zero</h2><p>Searches the Hall-sensor index and measures its centre.</p><button disabled={zeroProgress !== null} onclick={() => calibrate('/api/calibration/zero/stream', 'complete_zero', 'zero')}>Find zero</button>{#if zeroProgress !== null}<progress max="100" value={zeroProgress}></progress><small>{zeroProgress}%</small>{/if}</article>
        <article><span class="number">02</span><h2>Calibrate angle sensor</h2><p>Measures the AS5600 error over a full calibration sequence.</p><button disabled={angleProgress !== null} onclick={() => calibrate('/api/calibration/angle/stream', 'complete_angle', 'angle')}>Calibrate sensor</button>{#if angleProgress !== null}<progress max="100" value={angleProgress}></progress><small>{angleProgress}%</small>{/if}</article>
      </div>
    </section>
  {:else}
    <section><p class="eyebrow">Maintenance</p><h1>Install images</h1><p class="intro warning">An update writes flash and restarts the rotator. Keep power and the network connection stable.</p>
      <div class="cards">
        <article><span class="number">FW</span><h2>Firmware</h2><p>Upload the ESP-IDF application image <code>Rotator.bin</code>.</p><label class="file">{firmwareFile?.name || 'Choose firmware image'}<input type="file" accept=".bin" onchange={(event) => firmwareFile = event.currentTarget.files[0]} /></label><button onclick={() => upload('firmware')}>Upload firmware</button>{#if firmwareProgress !== null}<progress max="100" value={firmwareProgress}></progress>{/if}</article>
        <article><span class="number">FS</span><h2>Web filesystem</h2><p>Upload <code>littlefs.bin</code>, including this interface.</p><label class="file">{filesystemFile?.name || 'Choose filesystem image'}<input type="file" accept=".bin" onchange={(event) => filesystemFile = event.currentTarget.files[0]} /></label><button onclick={() => upload('filesystem')}>Upload filesystem</button>{#if filesystemProgress !== null}<progress max="100" value={filesystemProgress}></progress>{/if}</article>
      </div>
    </section>
  {/if}
</main>

{#if notice}<div class:error class="notice" role="status">{notice}</div>{/if}
<footer>Rotator {VERSION} <span>·</span> Astro Geeks Munich</footer>

<style>
  :global(*) { box-sizing: border-box; }
  :global(html) { color-scheme: dark; background: #0b1118; }
  :global(body) { margin: 0; min-width: 320px; min-height: 100vh; color: #dce8f2; background: radial-gradient(circle at 82% 5%, #173344 0, transparent 30rem), #0b1118; font-family: Inter, ui-sans-serif, system-ui, sans-serif; }
  button, input { font: inherit; }
  header { height: 5.2rem; padding: 0 6vw; display: flex; align-items: center; justify-content: space-between; border-bottom: 1px solid #243340; }
  .brand { display: flex; align-items: center; gap: .8rem; }
  .brand strong { display: block; letter-spacing: .02em; }
  .brand small { color: #7890a3; font-size: .72rem; text-transform: uppercase; letter-spacing: .13em; }
  .mark { display: grid; place-items: center; width: 2.5rem; height: 2.5rem; border: 1px solid #21d4b4; color: #21d4b4; font-weight: 800; }
  .live { color: #8aa0b2; font-size: .8rem; text-transform: uppercase; letter-spacing: .12em; }
  .live span { display: inline-block; width: .5rem; height: .5rem; border-radius: 50%; background: #21d4b4; margin-right: .5rem; box-shadow: 0 0 10px #21d4b4; }
  nav { padding: 0 6vw; display: flex; gap: 2rem; border-bottom: 1px solid #243340; overflow-x: auto; }
  nav button { padding: 1rem 0 .85rem; border: 0; border-bottom: 2px solid transparent; background: none; color: #71899c; cursor: pointer; white-space: nowrap; }
  nav button.active { color: #fff; border-color: #ff7a4d; }
  main { max-width: 1100px; margin: 0 auto; padding: clamp(2rem, 6vw, 5rem) 1.5rem; min-height: calc(100vh - 12.5rem); }
  section { animation: enter .28s ease both; }
  @keyframes enter { from { opacity: 0; transform: translateY(8px); } }
  h1 { margin: .2rem 0 .5rem; font-size: clamp(2rem, 5vw, 3.4rem); letter-spacing: -.04em; }
  h2 { margin: .5rem 0; }
  .eyebrow { color: #21d4b4; text-transform: uppercase; letter-spacing: .18em; font-size: .75rem; font-weight: 700; }
  .intro { color: #8da1b3; max-width: 44rem; margin-bottom: 2rem; }
  .warning { border-left: 2px solid #ff7a4d; padding-left: 1rem; color: #ddaa98; }
  .position { display: grid; grid-template-columns: 1.15fr .85fr; align-items: center; gap: 5rem; }
  .gauge { display: grid; place-items: center; }
  .primary { display: flex; align-items: start; gap: .4rem; margin: .5rem 0; }
  .primary strong { font: 300 clamp(4.6rem, 11vw, 8rem)/.88 ui-monospace, monospace; letter-spacing: -.09em; }
  .primary span { color: #ff7a4d; font-size: 2rem; }
  .direction { color: #91a7b8; margin: 1rem 0 2.5rem; text-transform: uppercase; letter-spacing: .12em; font-size: .76rem; }
  .direction span { color: #21d4b4; display: inline-block; font-size: 1.4rem; margin-right: .5rem; }
  .direction span.ccw { transform: scaleX(-1); }
  .metric { display: flex; justify-content: space-between; border-top: 1px solid #2a3945; padding: 1rem 0; color: #8298aa; }
  .metric strong { color: #dce8f2; }
  .section-head { display: flex; align-items: end; justify-content: space-between; gap: 1rem; }
  .form-grid { display: grid; gap: 1rem; max-width: 48rem; }
  label { color: #8fa4b6; font-size: .82rem; text-transform: uppercase; letter-spacing: .1em; }
  .form-grid label { display: grid; grid-template-columns: 1fr auto; gap: .7rem; }
  .form-grid label input { grid-column: 1; }
  input { width: 100%; margin-top: .45rem; padding: .85rem 1rem; color: #eef6fb; background: #121c25; border: 1px solid #344552; border-radius: 3px; outline: none; }
  input:focus { border-color: #21d4b4; box-shadow: 0 0 0 3px rgb(33 212 180 / .12); }
  button { border: 0; border-radius: 3px; padding: .78rem 1.2rem; color: #07110f; background: #21d4b4; font-weight: 750; cursor: pointer; }
  button:hover { filter: brightness(1.08); }
  button:disabled { opacity: .55; cursor: wait; }
  .secondary { color: #dce8f2; background: #22313d; }
  .wifi-grid { display: grid; grid-template-columns: 1.2fr .8fr; gap: 1.3rem; }
  .ap-list, .connect-card, article { border: 1px solid #2c3b47; background: rgb(17 26 35 / .86); }
  .ap-list { max-height: 25rem; overflow: auto; }
  .ap-list button { width: 100%; display: grid; grid-template-columns: 3rem 1fr auto; text-align: left; color: #dce8f2; background: transparent; border-bottom: 1px solid #273641; border-radius: 0; }
  .ap-list small { color: #71889a; }
  .signal { color: #21d4b4; letter-spacing: -.18em; }
  .empty { padding: 2rem; color: #72889a; }
  .connect-card { padding: 1.5rem; display: grid; gap: 1.2rem; align-content: start; }
  .status { color: #21d4b4; }
  .cards { display: grid; grid-template-columns: repeat(2, 1fr); gap: 1.2rem; }
  article { padding: 1.8rem; position: relative; overflow: hidden; }
  article p { min-height: 3rem; color: #8298aa; }
  article button { margin-top: .7rem; }
  .number { color: #ff7a4d; font: 700 .8rem ui-monospace, monospace; letter-spacing: .12em; }
  progress { display: block; width: 100%; height: .35rem; margin-top: 1.4rem; accent-color: #21d4b4; }
  .file { display: block; padding: 1rem; border: 1px dashed #435462; color: #91a7b8; text-transform: none; letter-spacing: 0; overflow: hidden; text-overflow: ellipsis; }
  .file input { position: absolute; opacity: 0; pointer-events: none; }
  code { color: #d7b57e; }
  .notice { position: fixed; right: 1.5rem; bottom: 1.5rem; max-width: 28rem; padding: 1rem 1.2rem; color: #062019; background: #21d4b4; box-shadow: 0 14px 50px #0009; }
  .notice.error { color: #fff; background: #b94d38; }
  footer { height: 3rem; padding: 0 6vw; display: flex; align-items: center; gap: .6rem; color: #587083; border-top: 1px solid #1d2a34; font-size: .75rem; text-transform: uppercase; letter-spacing: .12em; }
  @media (max-width: 760px) { header { padding: 0 1.2rem; } .live { display: none; } nav { padding: 0 1.2rem; gap: 1.5rem; } .position, .wifi-grid, .cards { grid-template-columns: 1fr; gap: 2rem; } .telemetry { text-align: center; } .metric { text-align: left; } .form-grid label { grid-template-columns: 1fr; } .form-grid label button { grid-column: 1; } .ap-list button { grid-template-columns: 2.5rem 1fr; } .ap-list small { grid-column: 2; } }
</style>
