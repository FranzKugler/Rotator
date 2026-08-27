<script>
  import { onMount, onDestroy } from 'svelte';
  import {
    checkForUpdate,
    fetchOtaStatus,
    installUpdate,
    setOtaConfig,
    uploadImage,
    fetchExpertStatus,
    enrollExpert,
    unlockExpert,
    lockExpert,
    resetExpert
  } from '../lib/api.js';

  let info = $state(null);
  let statusError = $state('');
  let channelError = $state('');
  let checking = $state(false);
  let file = $state(null);
  let kind = $state(null);
  let phase = $state('idle');
  let progress = $state(0);
  let destroyed = false;
  let expert = $state(null);
  let expertPassword = $state('');
  let expertError = $state('');
  const intervals = [0, 6, 12, 24, 72, 168];
  const sleep = (ms) => new Promise((resolve) => setTimeout(resolve, ms));
  const busy = $derived(checking || info?.state === 'downloading' || phase === 'uploading' || phase === 'rebooting');

  onDestroy(() => destroyed = true);

  async function load({ quiet = false } = {}) {
    try {
      [info, expert] = await Promise.all([fetchOtaStatus(), fetchExpertStatus()]);
      statusError = '';
      return true;
    } catch (error) {
      if (!quiet) statusError = error.message;
      return false;
    }
  }

  async function submitExpert() {
    expertError = '';
    try {
      expert = expert.enrolled
        ? await unlockExpert(expertPassword)
        : await enrollExpert(expertPassword);
      expertPassword = '';
    } catch (error) { expertError = error.message; }
  }

  async function lock() { expert = await lockExpert(); }
  async function reset() { expert = await resetExpert(); expertPassword = ''; }

  async function check() {
    checking = true;
    channelError = '';
    try { info = await checkForUpdate(); }
    catch (error) { channelError = error.message; }
    finally { checking = false; }
  }

  async function configure(patch) {
    channelError = '';
    try { info = await setOtaConfig(patch); }
    catch (error) { channelError = error.message; }
  }

  async function install() {
    channelError = '';
    try {
      info = await installUpdate();
      await followInstall();
    } catch (error) { channelError = error.message; }
  }

  async function followInstall() {
    for (let attempt = 0; attempt < 200 && !destroyed; attempt += 1) {
      await sleep(1500);
      if (!(await load({ quiet: true }))) continue;
      if (info.state === 'downloading') continue;
      if (info.state === 'installed') {
        phase = 'rebooting';
        if (await waitForRotator()) {
          await sleep(1000);
          location.reload();
        } else channelError = 'Rotator did not answer after reboot.';
      } else if (info.state === 'failed') channelError = `${info.error}: ${info.errorDetail}`;
      return;
    }
  }

  async function waitForRotator() {
    for (let attempt = 0; attempt < 30 && !destroyed; attempt += 1) {
      await sleep(2000);
      if (await load({ quiet: true })) return true;
    }
    return false;
  }

  async function pick(event) {
    phase = 'idle';
    progress = 0;
    file = event.currentTarget.files?.[0] || null;
    kind = null;
    if (!file) return;
    const head = new Uint8Array(await file.slice(0, 1).arrayBuffer());
    kind = head[0] === 0xe9 ? 'Firmware' : 'Web filesystem';
  }

  async function upload() {
    if (!file) return;
    phase = 'uploading';
    try {
      await uploadImage('/ota/upload', file, (value) => progress = value);
      phase = 'rebooting';
      if (await waitForRotator()) {
        if (kind === 'Web filesystem') location.reload();
        else { phase = 'done'; await load(); }
      } else phase = 'failed';
    } catch (error) {
      if (progress < 99) {
        statusError = error.message;
        phase = 'failed';
      } else {
        phase = 'rebooting';
        if (await waitForRotator()) location.reload(); else phase = 'failed';
      }
    }
  }

  const mb = (bytes) => Number.isFinite(bytes) ? `${(bytes / 1024 / 1024).toFixed(2)} MB` : '—';
  const since = (seconds) => seconds < 0 ? 'Never checked' : seconds < 90 ? 'Checked just now' :
    seconds < 5400 ? `Checked ${Math.round(seconds / 60)} min ago` : `Checked ${Math.round(seconds / 3600)} h ago`;

  onMount(load);
</script>

<div class="stack">
  <section class="panel">
    <p class="eyebrow">Installed</p><h1>Software versions</h1>
    {#if statusError}<p class="banner">Status unavailable · {statusError}</p>{/if}
    {#if info}
      <div class="fields">
        <div><span>Firmware</span><strong>{info.firmwareVersion || '—'}</strong></div>
        <div><span>Web interface</span><strong>{info.fsVersion || 'unknown'}</strong></div>
        <div><span>Application partition</span><strong>{mb(info.sketchSize)}</strong></div>
        <div><span>Room for update</span><strong>{mb(info.freeSpace)}</strong></div>
        <div><span>Running from</span><strong>{info.partition || '—'}</strong></div>
      </div>
    {:else}<p class="muted">Loading installed versions…</p>{/if}
  </section>

  <section class="panel">
    <p class="eyebrow">Protected maintenance</p><h1>Expert access</h1>
    {#if expert}
      {#if expert.unlocked}
        <p class="success">Update controls are unlocked.</p>
        <button class="secondary" onclick={lock}>Lock again</button>
      {:else}
        <p class="muted">{expert.enrolled ? 'Enter the expert password to install or configure updates.' : 'Set an expert password before enabling firmware changes.'}</p>
        <label>Password<input class="password" type="password" bind:value={expertPassword} autocomplete={expert.enrolled ? 'current-password' : 'new-password'} /></label>
        <button onclick={submitExpert} disabled={expert.lockedOut || expertPassword.length < 6}>{expert.enrolled ? 'Unlock' : 'Set password'}</button>
        {#if expert.lockedOut}<p class="banner">Too many attempts. Try again in five minutes.</p>{/if}
        {#if expert.grace > 0 && expert.enrolled}<button class="danger" onclick={reset}>Clear forgotten password ({expert.grace}s)</button>{/if}
      {/if}
      {#if expertError}<p class="banner">{expertError}</p>{/if}
    {/if}
  </section>

  <section class="panel">
    <p class="eyebrow">GitHub releases</p><h1>Update source</h1>
    {#if info}
      <div class="control-grid">
        <label>Channel<select value={info.channel} disabled={busy || !expert?.unlocked} onchange={(event) => configure({ channel: Number(event.currentTarget.value) })}><option value="0">Stable</option><option value="1">Edge</option></select></label>
        <label>Check interval<select value={info.checkInterval} disabled={busy || !expert?.unlocked} onchange={(event) => configure({ checkInterval: Number(event.currentTarget.value) })}>{#each intervals as value}<option {value}>{value === 0 ? 'Never' : `Every ${value} hours`}</option>{/each}</select></label>
        <label class="toggle"><span>Automatic update</span><input type="checkbox" checked={info.autoUpdate} disabled={busy || !expert?.unlocked} onchange={(event) => configure({ autoUpdate: event.currentTarget.checked })} /></label>
      </div>
      <p class="muted">Automatic updates are installed between 02:00 and 05:00 after the configured check interval.</p>
      <div class="offer"><span>Available version</span><strong>{info.availableVersion || '—'}</strong></div>
      {#if info.availableNotes}<p class="notes">{info.availableNotes}</p>{/if}
      {#if info.state === 'downloading'}
        <progress max="100" value={info.progress}></progress><p class="muted">Downloading and verifying · {info.progress}%</p>
      {:else if info.updateAvailable}
        <button onclick={install} disabled={busy || !expert?.unlocked}>Install now</button>
      {:else if info.lastCheck >= 0}<p class="success">The selected channel is up to date.</p>{/if}
      {#if channelError}<p class="banner">{channelError}</p>{/if}
      <div class="check"><span>{since(info.lastCheck)}</span><button class="secondary" onclick={check} disabled={busy || !expert?.unlocked}>{checking ? 'Checking…' : 'Check now'}</button></div>
    {/if}
  </section>

  <section class="panel">
    <p class="eyebrow">Local image</p><h1>Upload manually</h1>
    <p class="muted">Firmware and filesystem images are detected from their contents, as on QlockThreeW32.</p>
    <label class="file">{file?.name || 'Choose a .bin image'}<input type="file" accept=".bin,application/octet-stream" onchange={pick} disabled={busy || !expert?.unlocked} /></label>
    {#if file}<div class="offer"><span>Detected as</span><strong>{kind} · {mb(file.size)}</strong></div>{/if}
    {#if phase === 'uploading' || phase === 'rebooting'}<progress max="100" value={progress}></progress><p class="muted">{phase === 'uploading' ? `Writing · ${progress}%` : 'Rebooting…'}</p>{/if}
    {#if phase === 'done'}<p class="success">Update installed successfully.</p>{/if}
    {#if phase === 'failed'}<p class="banner">Update failed or the rotator did not return after reboot.</p>{/if}
    <button onclick={upload} disabled={!file || busy || !expert?.unlocked}>{busy ? 'Update running…' : 'Upload and install'}</button>
  </section>
</div>

<style>
  .stack { display: grid; gap: 1.2rem; }
  .panel { border: 1px solid #2c3b47; background: rgb(17 26 35 / .86); padding: clamp(1.3rem, 4vw, 2rem); }
  .eyebrow { color: #21d4b4; text-transform: uppercase; letter-spacing: .18em; font-size: .72rem; font-weight: 750; margin: 0; }
  h1 { margin: .25rem 0 1.4rem; font-size: clamp(1.65rem, 4vw, 2.5rem); letter-spacing: -.035em; }
  .fields { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 0 2rem; }
  .fields div, .offer, .check { display: flex; justify-content: space-between; gap: 1rem; padding: .9rem 0; border-top: 1px solid #293945; }
  .fields span, .offer span, .check span, .muted { color: #8298aa; }
  .control-grid { display: grid; grid-template-columns: repeat(3, 1fr); gap: 1rem; align-items: end; }
  label { color: #8fa4b6; font-size: .78rem; text-transform: uppercase; letter-spacing: .1em; }
  select, .password { display: block; width: 100%; margin: .45rem 0 1rem; padding: .8rem; color: #eef6fb; background: #121c25; border: 1px solid #344552; }
  .danger { margin-left: .7rem; color: #fff; background: #9f3e30; }
  .toggle { display: flex; justify-content: space-between; align-items: center; min-height: 2.8rem; border: 1px solid #344552; padding: .75rem; }
  .toggle input { accent-color: #21d4b4; width: 1.1rem; height: 1.1rem; }
  button { border: 0; border-radius: 3px; padding: .78rem 1.2rem; color: #07110f; background: #21d4b4; font: inherit; font-weight: 750; cursor: pointer; }
  button:disabled { opacity: .5; cursor: wait; }
  .secondary { color: #dce8f2; background: #22313d; }
  .check { align-items: center; margin-top: 1rem; }
  .notes { color: #b5c5d0; border-left: 2px solid #ff7a4d; padding-left: 1rem; }
  .success { color: #21d4b4; }
  .banner { color: #ffb39b; background: rgb(185 77 56 / .16); border-left: 2px solid #ff7a4d; padding: .8rem 1rem; }
  progress { width: 100%; height: .4rem; accent-color: #21d4b4; }
  .file { display: block; padding: 1rem; margin: 1rem 0; border: 1px dashed #435462; color: #91a7b8; text-transform: none; letter-spacing: 0; overflow: hidden; text-overflow: ellipsis; cursor: pointer; }
  .file input { position: absolute; opacity: 0; pointer-events: none; }
  @media (max-width: 720px) { .fields, .control-grid { grid-template-columns: 1fr; } }
</style>
