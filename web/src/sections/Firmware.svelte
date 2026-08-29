<script>
  import { onMount, onDestroy } from 'svelte';
  import { checkForUpdate, fetchOtaStatus, installUpdate, setOtaConfig, uploadImage } from '../lib/api.js';

  let info = $state(null);
  let statusError = $state('');
  let channelError = $state('');
  let checking = $state(false);
  let file = $state(null);
  let kind = $state(null);
  let phase = $state('idle');
  let progress = $state(0);
  let destroyed = false;
  const intervals = [0, 6, 12, 24, 72, 168];
  const sleep = (ms) => new Promise((resolve) => setTimeout(resolve, ms));
  const busy = $derived(checking || info?.state === 'downloading' || phase === 'uploading' || phase === 'rebooting');

  onDestroy(() => destroyed = true);

  async function load({ quiet = false } = {}) {
    try {
      info = await fetchOtaStatus();
      statusError = '';
      return true;
    } catch (error) {
      if (!quiet) statusError = error.message;
      return false;
    }
  }

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
    kind = head[0] === 0xe9 ? 'Firmware image' : 'Filesystem image';
  }

  async function upload() {
    if (!file) return;
    phase = 'uploading';
    try {
      await uploadImage('/ota/upload', file, (value) => progress = value);
      phase = 'rebooting';
      if (await waitForRotator()) {
        if (kind === 'Filesystem image') location.reload();
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

<section class="card">
  <h2>Installed</h2>
  {#if statusError}<p class="banner">Status unavailable · {statusError}</p>{/if}
  {#if info}
    <div class="field"><span class="key">Firmware</span><span>{info.firmwareVersion || '—'}</span></div>
    <div class="field"><span class="key">Web interface</span><span>{info.fsVersion || 'Unknown'}</span></div>
    <div class="field"><span class="key">Used</span><span>{mb(info.sketchSize)}</span></div>
    <div class="field"><span class="key">Room for update</span><span>{mb(info.freeSpace)}</span></div>
    {#if info.partition}<div class="field"><span class="key">Running from</span><span>{info.partition}</span></div>{/if}
  {:else}<p class="hint">Loading…</p>{/if}
</section>

<section class="card">
  <h2>Update source</h2>
  {#if info}
    <div class="field"><label for="channel">Channel</label><select id="channel" value={info.channel} disabled={busy} onchange={(event) => configure({ channel: Number(event.currentTarget.value) })}><option value={0}>Stable</option><option value={1}>Edge</option></select></div>
    <div class="field"><label for="interval">Check interval</label><select id="interval" value={info.checkInterval} disabled={busy} onchange={(event) => configure({ checkInterval: Number(event.currentTarget.value) })}>{#each intervals as value}<option {value}>{value === 0 ? 'Never' : `Every ${value} hours`}</option>{/each}</select></div>
    <div class="field"><label for="auto">Automatic update</label><span class="switch"><input id="auto" type="checkbox" checked={info.autoUpdate} disabled={busy} onchange={(event) => configure({ autoUpdate: event.currentTarget.checked })} /><span></span></span></div>
    <p class="hint">Automatic updates are installed between 02:00 and 05:00 after the configured check interval.</p>
    <div class="field"><span class="key">Available version</span><span>{info.availableVersion || '—'}</span></div>
    {#if info.availableNotes}<p class="hint">{info.availableNotes}</p>{/if}
    {#if info.state === 'downloading'}
      <div class="progress" role="progressbar" aria-valuenow={info.progress}><div class="bar" style="width: {info.progress}%"></div></div><p class="hint">Downloading and verifying · {info.progress}%</p>
    {:else if info.updateAvailable}
      <button type="button" class="primary" onclick={install} disabled={busy}>Install now</button>
    {:else if info.lastCheck >= 0}<p class="hint success">The selected channel is up to date.</p>{/if}
    {#if channelError}<p class="banner">{channelError}</p>{/if}
    <div class="field"><span class="key">{since(info.lastCheck)}</span><button type="button" class="secondary" onclick={check} disabled={busy}>{checking ? 'Checking…' : 'Check now'}</button></div>
  {:else}<p class="hint">Loading…</p>{/if}
</section>

<section class="card">
  <h2>Upload image</h2>
  <div class="field">
    <span class="key">File</span>
    <span class="filepick">
      <span class="filename" class:empty={!file}>{file ? file.name : 'No file selected'}</span>
      <input id="image" type="file" accept=".bin,application/octet-stream" onchange={pick} disabled={busy} />
      <label class="iconbutton" for="image" title="Choose file" aria-label="Choose file">
        <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.7" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true"><path d="M3 7.5A1.5 1.5 0 0 1 4.5 6h4L11 8h6a1.5 1.5 0 0 1 1.5 1.5V10" /><path d="M3 10h18l-2.1 7.6a1.5 1.5 0 0 1-1.45 1.4H4.5A1.5 1.5 0 0 1 3 17.5z" /></svg>
      </label>
    </span>
  </div>
  {#if file}<div class="field"><span class="key">Detected as</span><span>{kind} · {mb(file.size)}</span></div>{/if}
  {#if phase === 'uploading' || phase === 'rebooting'}<div class="progress"><div class="bar" style="width: {progress}%"></div></div><p class="hint">{phase === 'uploading' ? `Writing · ${progress}%` : 'Rebooting…'}</p>{/if}
  {#if phase === 'done'}<p class="hint success">Update installed successfully.</p>{/if}
  {#if phase === 'failed'}<p class="banner">Update failed or the rotator did not return after reboot.</p>{/if}
  <button type="button" class="primary" onclick={upload} disabled={!file || busy}>{busy ? 'Update running…' : 'Upload and install'}</button>
</section>

<style>
  .progress { height: 10px; margin: .75rem 0 .25rem; border-radius: 999px; background: var(--border); overflow: hidden; }
  .bar { height: 100%; background: var(--accent); transition: width .15s linear; }
  .success { color: var(--accent); }
</style>
