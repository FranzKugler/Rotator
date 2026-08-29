<script>
  /**
   * Debug
   * What the rotator has been saying, and how it is doing while saying it.
   *
   * The log window is the point. The rotator's only other log output - the
   * USB serial console - only shows what is said while someone already has a
   * terminal open, and the lines worth having are the ones from the two
   * seconds after a restart that nobody is ever in time for. The firmware
   * keeps them in a ring, so this tab starts by fetching the beginning
   * rather than subscribing to the present.
   */
  import { onMount, onDestroy, tick } from 'svelte';
  import { fetchLog } from '../lib/api.js';

  // Matches the firmware's ring, which is what limits what can be shown; a
  // longer list here would only ever hold lines the rotator has forgotten.
  const KEEP = 200;
  const POLL_MS = 2000;

  const RESET_REASONS = {
    'power-on': 'Power-on',
    external: 'Reset line',
    software: 'Software (restart requested)',
    panic: 'Crash (panic)',
    'watchdog-int': 'Watchdog (interrupt)',
    'watchdog-task': 'Watchdog (task)',
    watchdog: 'Watchdog',
    brownout: 'Brown-out',
    sdio: 'SDIO',
    usb: 'Reset over USB',
    jtag: 'Reset over JTAG',
    efuse: 'eFuse error',
    'power-glitch': 'Power glitch',
    'cpu-lockup': 'CPU lock-up (double exception)',
    'deep-sleep': 'Deep sleep',
    unknown: 'unknown'
  };

  let lines = $state([]);
  let info = $state(null);
  let error = $state(null);
  let paused = $state(false);
  /** Set when the ring wrapped past what we had, so the gap can be named. */
  let missed = $state(0);

  let seq = 0;
  let logEl = $state(null);
  let timer = null;
  let destroyed = false;
  /** One request at a time. Two overlapping rounds could append a line twice,
      and the keyed each below throws on a repeated key - in production too. */
  let polling = false;

  onDestroy(() => {
    destroyed = true;
    clearTimeout(timer);
  });

  /**
   * One round. Keeps asking while the rotator says more is waiting, so
   * opening the tab fills the window in one go instead of a batch every two
   * seconds.
   */
  async function poll() {
    if (polling) return;
    polling = true;
    try {
      let more = true;
      while (more && !destroyed && !paused) {
        const batch = await fetchLog(seq);
        info = batch;
        error = null;

        // The numbering went backwards, so the rotator restarted under us -
        // after an update, or because it crashed. The window is thrown away
        // rather than appended to: two boots run together, with the
        // timestamps starting over in the middle, is worse than losing what
        // was on screen.
        if (batch.seq < seq) {
          lines = [];
          missed = 0;
          seq = 0;
          continue;
        }

        // The rotator says which line it still starts at. Asking for 300 and
        // being told the ring now starts at 480 means 180 lines came and
        // went between two polls - worth saying, rather than leaving a
        // silent gap.
        if (seq > 0 && batch.oldest > seq) missed += batch.oldest - seq;

        // Belt and braces against a repeated key, which takes the whole list
        // down rather than the one row: only ever append lines newer than
        // the newest already held.
        const highest = lines.length ? lines[lines.length - 1].s : -1;
        const fresh = batch.lines.filter((line) => line.s > highest);

        if (fresh.length) {
          const atBottom = isAtBottom();
          lines = [...lines, ...fresh].slice(-KEEP);
          if (atBottom) {
            await tick();
            scrollToBottom();
          }
        }
        seq = batch.seq;
        more = batch.more;
      }
    } catch (err) {
      error = err.message;
    } finally {
      polling = false;
    }
  }

  /**
   * Following the tail is what you want while watching, and the last thing
   * you want while reading something further up - so it follows only when
   * the view is already at the bottom.
   */
  function isAtBottom() {
    if (!logEl) return true;
    return logEl.scrollHeight - logEl.scrollTop - logEl.clientHeight < 24;
  }

  function scrollToBottom() {
    if (logEl) logEl.scrollTop = logEl.scrollHeight;
  }

  async function tickLoop() {
    if (!paused) await poll();
    if (!destroyed) timer = setTimeout(tickLoop, POLL_MS);
  }

  onMount(async () => {
    await tickLoop();
    await tick();
    scrollToBottom();
  });

  function togglePause() {
    paused = !paused;
    if (!paused) poll();
  }

  /** Empties the window only - the rotator keeps its ring, and a reload refills. */
  function clearView() {
    lines = [];
    missed = 0;
  }

  /** millis() since boot as h:mm:ss.mmm - the log of a boot is read by elapsed time. */
  function stamp(ms) {
    const milli = String(ms % 1000).padStart(3, '0');
    const total = Math.floor(ms / 1000);
    const s = String(total % 60).padStart(2, '0');
    const m = String(Math.floor(total / 60) % 60).padStart(2, '0');
    const h = Math.floor(total / 3600);
    return `${h}:${m}:${s}.${milli}`;
  }

  /** This project's own numbering: 1 verbose, 2 debug, 3 info, 4 warning, 5 error. */
  function levelClass(level) {
    if (level >= 5) return 'err';
    if (level === 4) return 'warn';
    if (level === 3) return 'info';
    return 'quiet';
  }

  const duration = (ms) => {
    const total = Math.floor(ms / 1000);
    const d = Math.floor(total / 86400);
    const h = Math.floor(total / 3600) % 24;
    const m = Math.floor(total / 60) % 60;
    return d > 0 ? `${d} d ${h} h` : h > 0 ? `${h} h ${m} min` : `${m} min`;
  };

  const kb = (bytes) => `${Math.round(bytes / 1024)} kB`;
</script>

<section class="card">
  <h2>State</h2>

  {#if error && !info}
    <p class="banner">
      {error === '{"error":"expertLocked"}'
        ? 'Locked · unlock expert mode on the Update tab to see this.'
        : `Status not available — ${error}`}
    </p>
  {:else if info}
    <div class="field"><span class="key">Uptime</span><span>{duration(info.uptime)}</span></div>
    <div class="field">
      <span class="key">Last restart</span><span>{RESET_REASONS[info.reset] ?? info.reset}</span>
    </div>
    <div class="field"><span class="key">Free memory</span><span>{kb(info.heap)}</span></div>
    <div class="field"><span class="key">Lowest since boot</span><span>{kb(info.heapMin)}</span></div>
    <div class="field"><span class="key">Largest free block</span><span>{kb(info.heapBlock)}</span></div>
    <p class="hint">If an update is refused with "could not activate the firmware", this low-water mark is the first number worth looking at.</p>
  {:else}
    <p class="hint">loading …</p>
  {/if}
</section>

<section class="card">
  <h2>Log</h2>

  <div class="log-actions">
    <button type="button" onclick={togglePause}>{paused ? 'Resume' : 'Pause'}</button>
    <button type="button" onclick={clearView}>Clear view</button>
    {#if error}<span class="log-error">{error}</span>{/if}
  </div>

  {#if missed > 0}
    <p class="hint">{missed} lines scrolled out of the rotator's buffer before they got here.</p>
  {/if}

  <div class="log" bind:this={logEl} role="log" aria-live="off">
    {#each lines as line (line.s)}
      <div class="row {levelClass(line.l)}">
        <span class="time">{stamp(line.t)}</span><span class="msg">{line.m}</span>
      </div>
    {:else}
      <div class="row quiet"><span class="msg">Nothing logged yet.</span></div>
    {/each}
  </div>

  <p class="hint">The rotator keeps the last 200 lines in RAM, including the ones from booting. What the bootloader prints before the firmware starts is only on the USB cable.</p>
</section>

<style>
  .log-actions {
    display: flex;
    gap: 0.5rem;
    align-items: center;
    margin-bottom: 0.75rem;
    flex-wrap: wrap;
  }

  .log-error {
    color: var(--danger);
    font-size: 0.85rem;
  }

  .log {
    /* Its own scroller: the page must not grow by a screen every minute, and
       a wide line has to scroll sideways here rather than widen the body. */
    overflow: auto;
    max-height: 60vh;
    background: var(--bg);
    border: 1px solid var(--border);
    border-radius: var(--radius);
    padding: 0.5rem;
    font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
    font-size: 0.78rem;
    line-height: 1.45;
  }

  .row {
    display: flex;
    gap: 0.75rem;
    white-space: pre;
  }

  .time {
    color: var(--muted);
    flex: none;
    /* Tabular figures, so the column does not jitter as the digits change. */
    font-variant-numeric: tabular-nums;
  }

  .msg {
    white-space: pre-wrap;
    word-break: break-word;
  }

  .err .msg {
    color: var(--danger);
  }

  .warn .msg {
    /* Not a token: amber is wanted in both themes and is legible on both. */
    color: #b26a00;
  }

  .quiet .msg {
    color: var(--muted);
  }

  @media (prefers-color-scheme: dark) {
    .warn .msg {
      color: #e0a350;
    }
  }
</style>
